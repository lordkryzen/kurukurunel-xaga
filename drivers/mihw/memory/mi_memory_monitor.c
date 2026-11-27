// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/task.h>
#include <linux/tracepoint.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/uidgid.h>
#include <linux/math64.h>
#include <linux/cpufreq.h>
#include <linux/overflow.h>

#define TRACKED_TASKS 256
#define SCENE_BYTES 64

struct latency_stat { u64 count, total, maximum; };
struct memory_stat {
 struct latency_stat direct, main;
 u64 running[NR_CPUS], runnable[NR_CPUS];
 u64 frequency_time[NR_CPUS], frequency_weight[NR_CPUS];
};
struct tracked_task {
 struct task_struct *task;
 u64 start, epoch, window, running_since, queued_since;
 unsigned int depth, running_cpu;
 bool kswapd;
};
static struct tracked_task tasks[TRACKED_TASKS];
static unsigned int cpu_khz[NR_CPUS];
static struct tracked_task *running_kswapd[NR_CPUS];
static struct memory_stat lifetime, window_stat;
static DEFINE_SPINLOCK(stat_lock);
static bool enabled = true, window_enabled, exiting;
static u64 epoch = 1, window_epoch = 1, enabled_since, window_since, dropped, unmatched;
static char scene[SCENE_BYTES];
static struct kobject *monitor_kobj;
static char *sysfs_name = "mi_memory_monitor";
module_param(sysfs_name, charp, 0444);
static unsigned int test_uid;
module_param(test_uid, uint, 0444);

static u64 add_sat(u64 a, u64 b) { return a > U64_MAX - b ? U64_MAX : a + b; }
static void latency_add(struct latency_stat *stat, u64 duration)
{
 stat->count = add_sat(stat->count, 1);
 stat->total = add_sat(stat->total, duration);
 stat->maximum = max(stat->maximum, duration);
}
/* stat_lock protects the fixed table and each task reference it owns. */
static struct tracked_task *find_task(struct task_struct *task, bool allocate, bool kswapd)
{
 struct tracked_task *available = NULL;
 unsigned int i;
 for (i = 0; i < TRACKED_TASKS; i++) {
  if (tasks[i].task == task && tasks[i].kswapd == kswapd) return &tasks[i];
  if (!tasks[i].task && !available) available = &tasks[i];
 }
 if (!allocate) return NULL;
 if (!available) { dropped++; return NULL; }
 get_task_struct(task);
 *available = (struct tracked_task){ .task = task, .kswapd = kswapd, .epoch = epoch,
                                   .window = window_enabled ? window_epoch : 0 };
 return available;
}
static void weight_frequency(struct memory_stat *stat, unsigned int cpu, u64 duration)
{
 u64 usec = div_u64(duration, NSEC_PER_USEC), weight;
 unsigned int mhz = cpu_khz[cpu] / 1000;
 if (!usec || !mhz) return;
 if (check_mul_overflow(usec, (u64)mhz, &weight)) weight = U64_MAX;
 stat->frequency_time[cpu] = add_sat(stat->frequency_time[cpu], usec);
 stat->frequency_weight[cpu] = add_sat(stat->frequency_weight[cpu], weight);
}
static void add_cpu_time(struct tracked_task *entry, unsigned int cpu, u64 start, u64 now, bool running)
{
 u64 *value, from;
 if (!enabled || cpu >= nr_cpu_ids || now <= start) return;
 from = max(start, enabled_since);
 if (now <= from) return;
 value = running ? lifetime.running : lifetime.runnable;
 value[cpu] = add_sat(value[cpu], now-from);
 if (running) weight_frequency(&lifetime, cpu, now-from);
 if (!window_enabled) return;
 from = max(from, window_since);
 if (now <= from) return;
 value = running ? window_stat.running : window_stat.runnable;
 value[cpu] = add_sat(value[cpu], now-from);
 if (running) weight_frequency(&window_stat, cpu, now-from);
}
static void direct_begin(void *unused, int order, gfp_t flags)
{
 struct tracked_task *entry;
 unsigned long irqflags;
 u64 now;
 if (!READ_ONCE(enabled) || (test_uid && __kuid_val(current_uid()) != test_uid)) return;
 now = ktime_get_ns();
 spin_lock_irqsave(&stat_lock, irqflags);
 if (!enabled || exiting) goto out;
 entry = find_task(current, true, false);
 if (entry && !entry->depth++) entry->start = now;
out:
 spin_unlock_irqrestore(&stat_lock, irqflags);
}
static void direct_end(void *unused, unsigned long reclaimed)
{
 struct tracked_task *entry;
 struct task_struct *release = NULL;
 unsigned long flags;
 u64 now = ktime_get_ns(), duration;
 bool main;
 spin_lock_irqsave(&stat_lock, flags);
 entry = find_task(current, false, false);
 if (!entry) goto out;
 if (--entry->depth) goto out;
 duration = now - entry->start;
 main = !(current->flags & PF_KTHREAD) && task_pid_nr(current) == task_tgid_nr(current);
 if (enabled && entry->epoch == epoch) {
  latency_add(&lifetime.direct, duration);
  if (main) latency_add(&lifetime.main, duration);
  if (window_enabled && entry->window == window_epoch) {
   latency_add(&window_stat.direct, duration);
   if (main) latency_add(&window_stat.main, duration);
  }
 }
 release = entry->task;
 memset(entry, 0, sizeof(*entry));
out:
 spin_unlock_irqrestore(&stat_lock, flags);
 if (release) put_task_struct(release);
}
static void kswapd_wake(void *unused, int node, int zone, int order)
{
 struct tracked_task *entry;
 unsigned long flags;
 u64 now = ktime_get_ns();
 if (!READ_ONCE(enabled)) return;
 spin_lock_irqsave(&stat_lock, flags);
 if (enabled && !exiting) {
  entry = find_task(current, true, true);
  if (entry) {
   if (entry->running_since) add_cpu_time(entry, raw_smp_processor_id(), entry->running_since, now, true);
   entry->running_since = now;
   entry->running_cpu = raw_smp_processor_id();
   running_kswapd[entry->running_cpu] = entry;
  }
 }
 spin_unlock_irqrestore(&stat_lock, flags);
}
static void kswapd_sleep(void *unused, int node)
{
 struct tracked_task *entry;
 struct task_struct *release = NULL;
 unsigned long flags;
 u64 now = ktime_get_ns();
 spin_lock_irqsave(&stat_lock, flags);
 entry = find_task(current, false, true);
 if (entry) {
  if (entry->running_since) add_cpu_time(entry, raw_smp_processor_id(), entry->running_since, now, true);
  release = entry->task;
  if (running_kswapd[entry->running_cpu] == entry) running_kswapd[entry->running_cpu] = NULL;
  memset(entry, 0, sizeof(*entry));
 }
 spin_unlock_irqrestore(&stat_lock, flags);
 if (release) put_task_struct(release);
}
static void task_wakeup(void *unused, struct task_struct *task)
{
 struct tracked_task *entry;
 unsigned long flags;
 if (!(task->flags & PF_KSWAPD) || !READ_ONCE(enabled)) return;
 spin_lock_irqsave(&stat_lock, flags);
 entry = find_task(task, false, true);
 if (entry && !entry->queued_since) entry->queued_since = ktime_get_ns();
 spin_unlock_irqrestore(&stat_lock, flags);
}
static void task_switch(void *unused, bool preempt, struct task_struct *prev,
                        struct task_struct *next, unsigned int prev_state)
{
 struct tracked_task *entry;
 unsigned long flags;
 unsigned int cpu;
 u64 now;
 if (!((prev->flags | next->flags) & PF_KSWAPD) || !READ_ONCE(enabled)) return;
 now = ktime_get_ns();
 spin_lock_irqsave(&stat_lock, flags);
 cpu = raw_smp_processor_id();
 if (prev->flags & PF_KSWAPD) {
  entry = find_task(prev, false, true);
  if (entry) {
   if (entry->running_since) add_cpu_time(entry, cpu, entry->running_since, now, true);
   entry->running_since = 0;
   if (running_kswapd[cpu] == entry) running_kswapd[cpu] = NULL;
   entry->queued_since = prev_state == TASK_RUNNING ? now : 0;
  }
 }
 if (next->flags & PF_KSWAPD) {
  entry = find_task(next, false, true);
  if (entry) {
   if (entry->queued_since) add_cpu_time(entry, cpu, entry->queued_since, now, false);
   entry->queued_since = 0;
   entry->running_since = now;
   entry->running_cpu = raw_smp_processor_id();
   running_kswapd[entry->running_cpu] = entry;
  }
 }
 spin_unlock_irqrestore(&stat_lock, flags);
}
static void cpu_frequency(void *unused, unsigned int khz, unsigned int cpu)
{
 struct tracked_task *entry;
 unsigned long flags;
 u64 now;
 if (cpu >= nr_cpu_ids) return;
 now = ktime_get_ns();
 spin_lock_irqsave(&stat_lock, flags);
 entry = running_kswapd[cpu];
 if (entry && entry->task && entry->running_since) {
  add_cpu_time(entry, cpu, entry->running_since, now, true);
  entry->running_since = now;
 }
 cpu_khz[cpu] = khz;
 spin_unlock_irqrestore(&stat_lock, flags);
}
static void task_exit(void *unused, struct task_struct *task)
{
 unsigned long flags;
 unsigned int i, refs = 0;
 spin_lock_irqsave(&stat_lock, flags);
 for (i=0; i<TRACKED_TASKS; i++) if (tasks[i].task == task) {
  if (tasks[i].kswapd && running_kswapd[tasks[i].running_cpu] == &tasks[i]) running_kswapd[tasks[i].running_cpu] = NULL;
  memset(&tasks[i],0,sizeof(tasks[i])); refs++; unmatched++;
 }
 spin_unlock_irqrestore(&stat_lock, flags);
 while (refs--) put_task_struct(task);
}

static ssize_t enable_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{ return sysfs_emit(buf,"%u\n",READ_ONCE(enabled)); }
static ssize_t enable_store(struct kobject *k, struct kobj_attribute *a, const char *buf, size_t count)
{
 unsigned int value; unsigned long flags;
 if (kstrtouint(buf,10,&value) || value>1) return -EINVAL;
 spin_lock_irqsave(&stat_lock,flags);
 if (enabled != value) { enabled=value; enabled_since=ktime_get_ns(); epoch++; window_epoch++; window_enabled=false; }
 spin_unlock_irqrestore(&stat_lock,flags);return count;
}
static ssize_t enable_scene_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{ return sysfs_emit(buf,"%u\n",READ_ONCE(window_enabled)); }
static ssize_t enable_scene_store(struct kobject *k, struct kobj_attribute *a, const char *buf, size_t count)
{
 char copy[SCENE_BYTES+4], *name; unsigned int value; unsigned long flags;
 if (!count || count>=sizeof(copy)) return -EINVAL;
 memcpy(copy,buf,count);copy[count]=0;strim(copy);
 if ((copy[0]!='0' && copy[0]!='1') || (copy[1] && copy[1]!=' ')) return -EINVAL;
 value=copy[0]-'0';name=copy[1]?strim(copy+2):copy+1;
 if (strlen(name)>=SCENE_BYTES) return -E2BIG;
 spin_lock_irqsave(&stat_lock,flags);
 if (value && !enabled) { spin_unlock_irqrestore(&stat_lock,flags);return -EACCES; }
 if (value) { memset(&window_stat,0,sizeof(window_stat));window_since=ktime_get_ns();strscpy(scene,name,sizeof(scene)); }
 window_enabled=value;window_epoch++;
 spin_unlock_irqrestore(&stat_lock,flags);return count;
}
static ssize_t reset_store(struct kobject *k, struct kobj_attribute *a, const char *buf, size_t count)
{
 unsigned int value; unsigned long flags;
 if (kstrtouint(buf,10,&value) || value!=1) return -EINVAL;
 spin_lock_irqsave(&stat_lock,flags);
 memset(&lifetime,0,sizeof(lifetime));memset(&window_stat,0,sizeof(window_stat));
 epoch++;window_epoch++;enabled_since=ktime_get_ns();window_enabled=false;scene[0]=0;dropped=unmatched=0;
 spin_unlock_irqrestore(&stat_lock,flags);return count;
}
static u64 ms(u64 ns) { return min_t(u64,div_u64(ns,NSEC_PER_MSEC),INT_MAX); }
static ssize_t direct_stat_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
 struct memory_stat *snapshot; struct latency_stat *d,*m; unsigned long flags; ssize_t len;
 snapshot=kmalloc(sizeof(*snapshot),GFP_KERNEL);if(!snapshot)return -ENOMEM;
 spin_lock_irqsave(&stat_lock,flags);
 *snapshot=!strcmp(a->attr.name,"history_direct_reclaim_stat")?lifetime:window_stat;
 spin_unlock_irqrestore(&stat_lock,flags);d=&snapshot->direct;m=&snapshot->main;
 /* Proprietary QoS classification is not inferred from a task's priority. */
 len=sysfs_emit(buf,"ALL_DR_CNT: %llu\nALL_max: %llu\nALL_avg: %llu\nMainThread_DR_CNT: %llu\nMainThread_max: %llu\nMainThread_avg: %llu\n",
 d->count,ms(d->maximum),ms(d->count?div64_u64(d->total,d->count):0),m->count,ms(m->maximum),ms(m->count?div64_u64(m->total,m->count):0));
 kfree(snapshot);return len;
}
static ssize_t kswapd_stat_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
 struct memory_stat *snapshot; unsigned long flags; unsigned int cpu; u64 total=0; ssize_t len;
 snapshot=kmalloc(sizeof(*snapshot),GFP_KERNEL);if(!snapshot)return -ENOMEM;
 spin_lock_irqsave(&stat_lock,flags);*snapshot=!strcmp(a->attr.name,"history_kswapd_stat")?lifetime:window_stat;spin_unlock_irqrestore(&stat_lock,flags);
 for_each_possible_cpu(cpu)total=add_sat(total,snapshot->running[cpu]);
 len=sysfs_emit(buf,"kswapd: %llu\n",ms(total));
 for_each_possible_cpu(cpu)if(len<PAGE_SIZE-64)len+=sysfs_emit_at(buf,len,"CPU%u: %llu|%llu\n",cpu,ms(snapshot->running[cpu]),snapshot->frequency_time[cpu]?div64_u64(snapshot->frequency_weight[cpu],snapshot->frequency_time[cpu]):0);
 kfree(snapshot);return len;
}
static ssize_t status_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
 unsigned long flags;u64 lost,orphans,runnable=0;unsigned int i,active=0;
 spin_lock_irqsave(&stat_lock,flags);lost=dropped;orphans=unmatched;for(i=0;i<TRACKED_TASKS;i++)active+=tasks[i].task!=NULL;for_each_possible_cpu(i)runnable=add_sat(runnable,lifetime.runnable[i]);spin_unlock_irqrestore(&stat_lock,flags);
 return sysfs_emit(buf,"qos_supported: 0\ntracked_tasks: %u\ndropped_starts: %llu\nexited_inflight: %llu\nkswapd_runnable_ms: %llu\n",active,lost,orphans,ms(runnable));
}
static struct kobj_attribute enable_attr=__ATTR(enable,0644,enable_show,enable_store);
static struct kobj_attribute scene_attr=__ATTR(enable_scene,0644,enable_scene_show,enable_scene_store);
static struct kobj_attribute reset_attr=__ATTR(reset,0200,NULL,reset_store);
static struct kobj_attribute direct_attr=__ATTR(direct_reclaim_stat,0444,direct_stat_show,NULL);
static struct kobj_attribute kswapd_attr=__ATTR(kswapd_stat,0444,kswapd_stat_show,NULL);
static struct kobj_attribute history_direct_attr=__ATTR(history_direct_reclaim_stat,0444,direct_stat_show,NULL);
static struct kobj_attribute history_kswapd_attr=__ATTR(history_kswapd_stat,0444,kswapd_stat_show,NULL);
static struct kobj_attribute status_attr=__ATTR(status,0444,status_show,NULL);
static struct attribute *attributes[]={&enable_attr.attr,&scene_attr.attr,&reset_attr.attr,&direct_attr.attr,&kswapd_attr.attr,&history_direct_attr.attr,&history_kswapd_attr.attr,&status_attr.attr,NULL};
static const struct attribute_group group={.attrs=attributes};
struct observer_hook { const char *name; void *probe; struct tracepoint *tracepoint; bool registered; };
static struct observer_hook hooks[]={
 {"mm_vmscan_direct_reclaim_begin",direct_begin},{"mm_vmscan_direct_reclaim_end",direct_end},
 {"mm_vmscan_memcg_reclaim_begin",direct_begin},{"mm_vmscan_memcg_reclaim_end",direct_end},
 {"mm_vmscan_kswapd_wake",kswapd_wake},{"mm_vmscan_kswapd_sleep",kswapd_sleep},
 {"cpu_frequency",cpu_frequency},{"sched_switch",task_switch},{"sched_wakeup",task_wakeup},{"sched_process_exit",task_exit},
};
static void find_tracepoint(struct tracepoint *tracepoint,void *unused)
{ unsigned int i;for(i=0;i<ARRAY_SIZE(hooks);i++)if(!strcmp(tracepoint->name,hooks[i].name))hooks[i].tracepoint=tracepoint; }
static void remove_hooks(void)
{
 unsigned int i;
 for(i=0;i<ARRAY_SIZE(hooks);i++)if(hooks[i].registered)tracepoint_probe_unregister(hooks[i].tracepoint,hooks[i].probe,NULL);
 tracepoint_synchronize_unregister();
}
static void release_tasks(void)
{ unsigned int i;for(i=0;i<TRACKED_TASKS;i++)if(tasks[i].task){put_task_struct(tasks[i].task);tasks[i].task=NULL;} }
static int __init monitor_init(void)
{
 unsigned int i;int ret;
 for_each_possible_cpu(i) cpu_khz[i] = cpufreq_quick_get(i);
 if (!sysfs_name[0] || strchr(sysfs_name,'/') || strlen(sysfs_name)>63)return -EINVAL;
 enabled_since=ktime_get_ns();
 for_each_kernel_tracepoint(find_tracepoint,NULL);
 for(i=0;i<ARRAY_SIZE(hooks);i++) {
  if(!hooks[i].tracepoint){ret=-ENOENT;goto fail;}
  ret=tracepoint_probe_register(hooks[i].tracepoint,hooks[i].probe,NULL);if(ret)goto fail;
  hooks[i].registered=true;
 }
 monitor_kobj=kobject_create_and_add(sysfs_name,kernel_kobj);if(!monitor_kobj){ret=-ENOMEM;goto fail;}
 ret=sysfs_create_group(monitor_kobj,&group);if(ret){kobject_put(monitor_kobj);goto fail;}
 return 0;
fail:
 WRITE_ONCE(enabled,false);remove_hooks();release_tasks();return ret;
}
static void __exit monitor_exit(void)
{
 unsigned long flags;
 sysfs_remove_group(monitor_kobj,&group);kobject_put(monitor_kobj);
 spin_lock_irqsave(&stat_lock,flags);exiting=true;enabled=false;spin_unlock_irqrestore(&stat_lock,flags);
 remove_hooks();release_tasks();
}
module_init(monitor_init);module_exit(monitor_exit);
MODULE_DESCRIPTION("Reclaim latency and kswapd scheduling statistics");
MODULE_LICENSE("GPL");
