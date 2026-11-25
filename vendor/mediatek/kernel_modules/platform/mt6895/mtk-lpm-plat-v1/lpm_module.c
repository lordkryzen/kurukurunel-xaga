// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <asm/cpuidle.h>
#include <asm/suspend.h>
#include <linux/cpu_pm.h>
#include <linux/cpuidle.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/syscalls.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/context_tracking.h>
#include <linux/mutex.h>

#include <lpm.h>
#include <lpm_internal.h>
#include <lpm_module.h>
#include <lpm_plat_common.h>
#include "lpm_registry.h"

#define LPM_MODULE_MAGIC	0xFCD03DC3
#define LPM_CPUIDLE_DRIVER	0xD0
#define LPM_MODLE		0xD1
#define LPM_SYS_ISSUER		0xD2

DEFINE_SPINLOCK(__lpm_sys_sync_lock);

typedef int (*state_enter)(struct cpuidle_device *dev,
			   struct cpuidle_driver *drv, int idx);
typedef int (*s2idle_enter)(struct cpuidle_device *dev,
			   struct cpuidle_driver *drv, int idx);


struct lpm_state_enter_fp {
	state_enter state;
	s2idle_enter s2idle;
};

struct lpm_model_info {
	const char *name;
	void *mod;
};

struct lpm_module_reg {
	int magic;
	int type;
	union {
		struct lpm_issuer *issuer;
		struct lpm_model_info info;
		struct lpm_state_enter_fp fp;
	} data;
};

struct mtk_lpm {
	struct lpm_model suspend;
	struct lpm_issuer *issuer;
};

static struct mtk_lpm lpm_system;

static DEFINE_SPINLOCK(lpm_mod_locker);
static bool core_ready;
module_param(core_ready,bool,0444);
static bool native_platform_config;
module_param(native_platform_config,bool,0444);
static bool syscore_registered, early_registered, pm_registered;

struct lpm_driver_snapshot {
 struct cpuidle_driver *drv;
 struct lpm_states_enter *unused;
 state_enter enter[CPUIDLE_STATE_MAX];
 s2idle_enter s2idle[CPUIDLE_STATE_MAX];
};
static struct lpm_driver_snapshot *snapshots;
static unsigned int snapshot_count;

static DEFINE_SPINLOCK(lpm_sys_locker);
static RAW_NOTIFIER_HEAD(lpm_notifier);


struct lpm_models {
	struct lpm_model *mod[CPUIDLE_STATE_MAX];
};
static DEFINE_PER_CPU(struct lpm_models, lpm_mods);

struct lpm_states_enter {
	state_enter cpuidle[CPUIDLE_STATE_MAX];
	s2idle_enter s2idle[CPUIDLE_STATE_MAX];
};



#define lpm_pm_notify(_id, _data)\
		({raw_notifier_call_chain(&lpm_notifier,\
					  _id, _data); })



#define lpm_notify_var(action, var)\
		((var & LPM_NB_MASK) | action)


#define IS_LPM_MODS_VALID(idx)\
		(idx < CPUIDLE_STATE_MAX)

enum lpm_state_type {
	lpm_state_cpuidle,
	lpm_state_s2idle,
};

static DEFINE_PER_CPU(unsigned long,lpm_idle_counts);
static int idle_entries_get(char *buf,const struct kernel_param *kp)
{
 unsigned long total=0;int cpu;
 for_each_possible_cpu(cpu)total+=READ_ONCE(per_cpu(lpm_idle_counts,cpu));
 return scnprintf(buf,PAGE_SIZE,"%lu\n",total);
}
static const struct kernel_param_ops idle_count_ops={.get=idle_entries_get};
module_param_cb(idle_entries,&idle_count_ops,NULL,0444);
module_param(snapshot_count,uint,0444);

static int lpm_state_enter(int type, struct cpuidle_device *dev,
			     struct cpuidle_driver *drv, int idx);

static int lpm_s2idle_state_enter(struct cpuidle_device *dev,
			   struct cpuidle_driver *drv, int idx)
{
	return lpm_state_enter(lpm_state_s2idle, dev, drv, idx);
}

static int lpm_cpuidle_state_enter(struct cpuidle_device *dev,
			   struct cpuidle_driver *drv, int idx)
{
	return lpm_state_enter(lpm_state_cpuidle, dev, drv, idx);
}

static struct cpuidle_driver *lpm_driver_for_cpu(int cpu)
{struct cpuidle_device lookup={.cpu=cpu};return cpuidle_get_cpu_driver(&lookup); }

static int lpm_cpuidle_state_percpu_set(int cpu, struct lpm_module_reg *p)
{
 return -EOPNOTSUPP; /* Driver ownership is installed once per unique driver below. */
}

static int lpm_model_percpu_set(int cpu, struct lpm_module_reg *p)
{
	struct lpm_models *ptr;
	struct cpuidle_driver *drv;
	int idx;

	drv = lpm_driver_for_cpu(cpu);

	if (!drv) {
		pr_info("[name:mtk_lpm][P] - cpuidle drv is null (%s:%d)\n",
					__func__, __LINE__);
		return -EINVAL;
	}

	for (idx = 0; idx < drv->state_count; ++idx) {
		if (!IS_LPM_MODS_VALID(idx)) {
			pr_info("[name:mtk_lpm][P] - mod(%s) out of index (%d) (%s:%d)\n",
					p->data.info.name, idx,
					__func__, __LINE__);
			break;
		}
		if (!strcmp(p->data.info.name, drv->states[idx].name)) {
			ptr = &per_cpu(lpm_mods, cpu);
			ptr->mod[idx] = p->data.info.mod;
			break;
		}
	}
	return 0;
}

int lpm_notifier_register(struct notifier_block *n)
{int ret;if(!n)return -EINVAL;cpus_read_lock();cpuidle_pause_and_lock();ret=raw_notifier_chain_register(&lpm_notifier,n);cpuidle_resume_and_unlock();cpus_read_unlock();return ret;}
EXPORT_SYMBOL(lpm_notifier_register);

int lpm_notifier_unregister(struct notifier_block *n)
{int ret;if(!n)return -EINVAL;cpus_read_lock();cpuidle_pause_and_lock();ret=raw_notifier_chain_unregister(&lpm_notifier,n);cpuidle_resume_and_unlock();cpus_read_unlock();return ret;}
EXPORT_SYMBOL(lpm_notifier_unregister);

void lpm_system_spin_lock(unsigned long *irqflag)
{
	if (irqflag) {
		unsigned long flag;

		spin_lock_irqsave(&lpm_sys_locker, flag);
		*irqflag = flag;
	} else
		spin_lock(&lpm_sys_locker);
}
void lpm_system_spin_unlock(unsigned long *irqflag)
{
	if (irqflag) {
		unsigned long flag = *irqflag;

		spin_unlock_irqrestore(&lpm_sys_locker, flag);
	} else
		spin_unlock(&lpm_sys_locker);
}

static int lpm_module_register_blockcall(int cpu, void *p)
{
	int ret = 0;
	struct lpm_module_reg *reg = (struct lpm_module_reg *)p;

	if (!reg || (reg->magic != LPM_MODULE_MAGIC)) {
		pr_info("[name:mtk_lpm][P] - registry(%d) fail (%s:%d)\n",
			reg ? reg->type : -1, __func__, __LINE__);
		return -EINVAL;
	}

	switch (reg->type) {
	case LPM_MODLE:
		if (reg->data.info.name)
			ret = lpm_model_percpu_set(cpu, reg);
		break;
	case LPM_SYS_ISSUER:
		lpm_system.issuer =
			(struct lpm_issuer *)reg->data.issuer;
		break;
	case LPM_CPUIDLE_DRIVER:
		ret = lpm_cpuidle_state_percpu_set(cpu, reg);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int __lpm_model_register(const char *name, struct lpm_model *lpm)
{
	struct lpm_module_reg reg = {
		.magic = LPM_MODULE_MAGIC,
		.type = LPM_MODLE,
		.data.info.name = name,
		.data.info.mod = lpm,
	};

	return lpm_do_work(LPM_REG_PER_CPU,
			       lpm_module_register_blockcall,
			       &reg);
}

int lpm_model_register(const char *name, struct lpm_model *lpm)
{
	return __lpm_model_register(name, lpm);
}
EXPORT_SYMBOL(lpm_model_register);

int lpm_model_unregister(const char *name)
{
	return __lpm_model_register(name, NULL);
}
EXPORT_SYMBOL(lpm_model_unregister);

static int __lpm_issuer_register(struct lpm_issuer *issuer)
{
	struct lpm_module_reg reg = {
		.magic = LPM_MODULE_MAGIC,
		.type = LPM_SYS_ISSUER,
		.data.issuer = issuer,
	};

	return lpm_do_work(LPM_REG_ALL_ONLINE,
			       lpm_module_register_blockcall,
			       &reg);
}

int lpm_issuer_register(struct lpm_issuer *issuer)
{
	return __lpm_issuer_register(issuer);
}
EXPORT_SYMBOL(lpm_issuer_register);

int lpm_issuer_unregister(struct lpm_issuer *issuer)
{
	if (issuer != lpm_system.issuer)
		return -EINVAL;
	return __lpm_issuer_register(NULL);
}
EXPORT_SYMBOL(lpm_issuer_unregister);

static int lpm_cpuidle_prepare(struct cpuidle_driver *drv, int index)
{
	struct lpm_models *lpmmods = NULL;
	struct lpm_model *lpm = NULL;
	struct lpm_nb_data nb_data = {0};
	int prompt = 0;
	unsigned int model_flags = 0;
	unsigned long flags;
	const int cpuid = smp_processor_id();

	if (index < 0 || index >= drv->state_count || index >= CPUIDLE_STATE_MAX)
		return -EINVAL;

	lpmmods = this_cpu_ptr(&lpm_mods);

	if (lpmmods && lpmmods->mod[index])
		lpm = lpmmods->mod[index];

	model_flags = (lpm) ? lpm->flag : 0;

	nb_data.cpu = cpuid;
	nb_data.index = index;
	nb_data.model = lpm;
	nb_data.issuer = lpm_system.issuer;

	if (!(drv->states[index].flags & CPUIDLE_FLAG_RCU_IDLE)) ct_idle_exit();

	spin_lock_irqsave(&lpm_mod_locker, flags);

	if (lpm && lpm->op.prompt)
		prompt = lpm->op.prompt(cpuid, nb_data.issuer);

	if (!unlikely(model_flags & LPM_REQ_NOBROADCAST)) {
		prompt = lpm_notify_var(LPM_NB_AFTER_PROMPT, prompt);
		lpm_pm_notify(prompt, &nb_data);
	}

	spin_unlock_irqrestore(&lpm_mod_locker, flags);

	if (lpm && lpm->op.prepare_enter)
		lpm->op.prepare_enter(prompt, cpuid, nb_data.issuer);

	lpm_pm_notify(LPM_NB_PREPARE, &nb_data);

	if (!(drv->states[index].flags & CPUIDLE_FLAG_RCU_IDLE)) ct_idle_enter();

	return 0;
}

static void lpm_cpuidle_resume(struct cpuidle_driver *drv, int index, int ret)
{
	struct lpm_models *lpmmods = NULL;
	struct lpm_model *lpm = NULL;
	struct lpm_nb_data nb_data = {0};
	unsigned int model_flags = 0;
	unsigned long flags;
	const int cpuid = smp_processor_id();

	if (index < 0 || index >= drv->state_count || index >= CPUIDLE_STATE_MAX)
		return;

	lpmmods = this_cpu_ptr(&lpm_mods);

	if (lpmmods && lpmmods->mod[index])
		lpm = lpmmods->mod[index];

	nb_data.cpu = cpuid;
	nb_data.index = index;
	nb_data.model = lpm;
	nb_data.issuer = lpm_system.issuer;
	nb_data.ret = ret;

	model_flags = (lpm) ? lpm->flag : 0;

	if (!(drv->states[index].flags & CPUIDLE_FLAG_RCU_IDLE)) ct_idle_exit();

	lpm_pm_notify(LPM_NB_RESUME, &nb_data);

	if (lpm && lpm->op.prepare_resume)
		lpm->op.prepare_resume(cpuid, nb_data.issuer);

	spin_lock_irqsave(&lpm_mod_locker, flags);

	if (!unlikely(model_flags & LPM_REQ_NOBROADCAST))
		lpm_pm_notify(LPM_NB_BEFORE_REFLECT, &nb_data);

	if (lpm && lpm->op.reflect)
		lpm->op.reflect(cpuid, nb_data.issuer);

	spin_unlock_irqrestore(&lpm_mod_locker, flags);

	if (!(drv->states[index].flags & CPUIDLE_FLAG_RCU_IDLE)) ct_idle_enter();
}

static int lpm_state_enter(int type, struct cpuidle_device *dev,
 struct cpuidle_driver *drv, int idx)
{
 unsigned int i;int ret;state_enter original=NULL;
 if(idx<=0 || idx>=drv->state_count || idx>=CPUIDLE_STATE_MAX)return -EINVAL;
 for(i=0;i<snapshot_count;i++)if(snapshots[i].drv==drv){
  original=type==lpm_state_s2idle ? snapshots[i].s2idle[idx] : snapshots[i].enter[idx];break;
 }
 if(!original)return -EINVAL;
 ret=lpm_cpuidle_prepare(drv,idx);if(ret)return ret;
 this_cpu_inc(lpm_idle_counts);
 ret=original(dev,drv,idx);
 lpm_cpuidle_resume(drv,idx,ret);return ret;
}

static int lpm_suspend_enter(void)
{
	int ret = 0;
	unsigned long flags;
	const int cpuid = smp_processor_id();

	if (lpm_system.suspend.flag & LPM_REQ_NOSUSPEND)
		return -EACCES;

	spin_lock_irqsave(&lpm_mod_locker, flags);
	if (lpm_system.suspend.op.prompt)
		ret = lpm_system.suspend.op.prompt(cpuid,
						       lpm_system.issuer);
	spin_unlock_irqrestore(&lpm_mod_locker, flags);
	return ret;
}

static void lpm_suspend_resume(void)
{
	unsigned long flags;
	const int cpuid = smp_processor_id();

	spin_lock_irqsave(&lpm_mod_locker, flags);
	if (lpm_system.suspend.op.reflect)
		lpm_system.suspend.op.reflect(cpuid, lpm_system.issuer);
	spin_unlock_irqrestore(&lpm_mod_locker, flags);
}

struct syscore_ops lpm_suspend = {
	.suspend = lpm_suspend_enter,
	.resume = lpm_suspend_resume,
};

/* Callers hold lock_system_sleep across registration and module resource changes. */
static struct lpm_model *suspend_owner;
static unsigned int suspend_model_slots;
module_param(suspend_model_slots,uint,0444);
static char suspend_owner_name[32];
int lpm_suspend_registry(const char *name, struct lpm_model *suspend)
{
 int cpu, idx, ret=0; unsigned long flags;
 struct cpuidle_driver *drv;
 if (!name || !suspend || (suspend->flag & LPM_REQ_NOSYSCORE_CB) || !*name || strlen(name)>=sizeof(suspend_owner_name)) return -EINVAL;
 cpus_read_lock(); cpuidle_pause_and_lock();
 if (suspend_owner) { ret=-EBUSY; goto out; }
 if (lpm_system.suspend.flag & LPM_REQ_NOSYSCORE_CB) {
  /* Preflight every possible CPU before publishing any suspend pointer. */
  for_each_possible_cpu(cpu) {
   drv=lpm_driver_for_cpu(cpu);
   if (!drv || drv->state_count<=0 || drv->state_count>CPUIDLE_STATE_MAX) {ret=-ENODEV;goto out;}
   for(idx=0;idx<drv->state_count;idx++) if(!strcmp(name,drv->states[idx].name)) break;
   if(idx==drv->state_count) {ret=-ENOENT;goto out;}
   if(per_cpu(lpm_mods,cpu).mod[idx]) {ret=-EBUSY;goto out;}
  }
  for_each_possible_cpu(cpu) {
   drv=lpm_driver_for_cpu(cpu);
   for(idx=0;idx<drv->state_count;idx++) if(!strcmp(name,drv->states[idx].name)) {
    per_cpu(lpm_mods,cpu).mod[idx]=suspend;suspend_model_slots++;break;
   }
  }
 } else {
  spin_lock_irqsave(&lpm_mod_locker,flags);
  lpm_system.suspend=*suspend;
  spin_unlock_irqrestore(&lpm_mod_locker,flags);
 }
 suspend_owner=suspend;strscpy(suspend_owner_name,name,sizeof(suspend_owner_name));
out:
 cpuidle_resume_and_unlock();cpus_read_unlock();return ret;
}
int lpm_suspend_unregister(const char *name, struct lpm_model *model)
{
 int cpu,idx,ret=0;unsigned long flags;
 if(!name || !model)return -EINVAL;
 cpus_read_lock();cpuidle_pause_and_lock();
 if(suspend_owner!=model || strcmp(name,suspend_owner_name)) {ret=-ENOENT;goto out;}
 if(lpm_system.suspend.flag & LPM_REQ_NOSYSCORE_CB) {
  for_each_possible_cpu(cpu) for(idx=0;idx<CPUIDLE_STATE_MAX;idx++)
   if(per_cpu(lpm_mods,cpu).mod[idx]==model)per_cpu(lpm_mods,cpu).mod[idx]=NULL;
 } else {
  spin_lock_irqsave(&lpm_mod_locker,flags);
  memset(&lpm_system.suspend,0,sizeof(lpm_system.suspend));
  spin_unlock_irqrestore(&lpm_mod_locker,flags);
 }
 suspend_owner=NULL;suspend_owner_name[0]=0;suspend_model_slots=0;
out:
 cpuidle_resume_and_unlock();cpus_read_unlock();return ret;
}
EXPORT_SYMBOL_GPL(lpm_suspend_unregister);
EXPORT_SYMBOL(lpm_suspend_registry);

int lpm_suspend_type_get(void)
{
	return (lpm_system.suspend.flag & LPM_REQ_NOSYSCORE_CB)
		? LPM_SUSPEND_S2IDLE : LPM_SUSPEND_SYSTEM;
}
EXPORT_SYMBOL(lpm_suspend_type_get);

static struct wakeup_source *lpm_lock;

#define SYS_SYNC_TIMEOUT 2000
#define SYS_SYNC_ONGOING 1
#define SYS_SYNC_DONE 0

static int sys_sync_ongoing;

static void suspend_sys_sync(struct work_struct *work);
static struct workqueue_struct *suspend_sys_sync_work_queue;
DECLARE_WORK(suspend_sys_sync_work, suspend_sys_sync);

static void suspend_sys_sync(struct work_struct *work)
{
	unsigned long flags;

	pr_debug("++\n");
	ksys_sync_helper();
	spin_lock_irqsave(&__lpm_sys_sync_lock, flags);
	sys_sync_ongoing = SYS_SYNC_DONE;
	spin_unlock_irqrestore(&__lpm_sys_sync_lock, flags);
	pr_debug("--\n");
}

static int suspend_syssync_enqueue(void)
{
	unsigned long flags;

	if (READ_ONCE(sys_sync_ongoing) != SYS_SYNC_DONE)
		return -EBUSY;
	spin_lock_irqsave(&__lpm_sys_sync_lock, flags);
	sys_sync_ongoing = SYS_SYNC_ONGOING;
	spin_unlock_irqrestore(&__lpm_sys_sync_lock, flags);
	pr_debug("PM: Syncing filesystems ... ");
	queue_work(suspend_sys_sync_work_queue, &suspend_sys_sync_work);

	return 0;
}

static int suspend_syssync_check(void)
{
	int timeout = 10;
	int acc = 0;

	while (READ_ONCE(sys_sync_ongoing) != SYS_SYNC_DONE) {
		msleep(timeout);
		if ((acc + 10) > SYS_SYNC_TIMEOUT) {
			pr_debug("Sync filesystems timeout");
			return -EBUSY;
		}
		acc += 10;
	}
	pr_debug("done.\n");
	return 0;
}

static int lpm_pm_early_event(struct notifier_block *notifier, unsigned long pm_event,
			void *unused)
{
	int ret;

	switch (pm_event) {
	case PM_HIBERNATION_PREPARE:
		return NOTIFY_DONE;
	case PM_RESTORE_PREPARE:
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION:
		return NOTIFY_DONE;
	case PM_SUSPEND_PREPARE:
		ret = suspend_syssync_enqueue();
		if (ret < 0)
			return NOTIFY_BAD;
		return NOTIFY_DONE;
	case PM_POST_SUSPEND:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block lpm_pm_early_notifier_func = {
	.notifier_call = lpm_pm_early_event,
	.priority = 255,
};

static int lpm_pm_event(struct notifier_block *notifier, unsigned long pm_event,
			void *unused)
{
	int ret;

	switch (pm_event) {
	case PM_HIBERNATION_PREPARE:
		return NOTIFY_DONE;
	case PM_RESTORE_PREPARE:
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION:
		return NOTIFY_DONE;
	case PM_SUSPEND_PREPARE:
		ret = suspend_syssync_check();
		if (ret < 0)
			return NOTIFY_BAD;
		return NOTIFY_DONE;
	case PM_POST_SUSPEND:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block lpm_pm_notifier_func = {
	.notifier_call = lpm_pm_event,
	.priority = 0,
};


static int lpm_hooks_install(void)
{
 int cpu,idx,ret=0;unsigned int i;
 snapshots=kcalloc(num_possible_cpus(),sizeof(*snapshots),GFP_KERNEL);
 if(!snapshots)return -ENOMEM;
 cpus_read_lock();cpuidle_pause_and_lock();
 /* Validate and snapshot every unique driver before publishing any wrapper. */
 for_each_possible_cpu(cpu){
  struct cpuidle_driver *drv=lpm_driver_for_cpu(cpu);
  if(!drv){if(cpu_online(cpu)){ret=-ENODEV;goto out;}continue;}
  if(drv->state_count<1 || drv->state_count>CPUIDLE_STATE_MAX){ret=-EINVAL;goto out;}
  for(i=0;i<snapshot_count;i++)if(snapshots[i].drv==drv)break;
  if(i<snapshot_count)continue;
  snapshots[i].drv=drv;
  for(idx=1;idx<drv->state_count;idx++){
   if(!drv->states[idx].enter || drv->states[idx].enter==lpm_cpuidle_state_enter || drv->states[idx].enter_s2idle==lpm_s2idle_state_enter){ret=-EBUSY;goto out;}
   snapshots[i].enter[idx]=drv->states[idx].enter;
   snapshots[i].s2idle[idx]=drv->states[idx].enter_s2idle;
  }
  snapshot_count++;
 }
 for(i=0;i<snapshot_count;i++)for(idx=1;idx<snapshots[i].drv->state_count;idx++){
  snapshots[i].drv->states[idx].enter=lpm_cpuidle_state_enter;
  if(snapshots[i].s2idle[idx])snapshots[i].drv->states[idx].enter_s2idle=lpm_s2idle_state_enter;
 }
out:cpuidle_resume_and_unlock();cpus_read_unlock();
 if(ret){kfree(snapshots);snapshots=NULL;snapshot_count=0;}
 return ret;
}
static void lpm_hooks_remove(void)
{
 unsigned int i;int idx;
 if(!snapshots)return;
 cpus_read_lock();cpuidle_pause_and_lock();
 for(i=0;i<snapshot_count;i++)for(idx=1;idx<snapshots[i].drv->state_count;idx++){
  if(snapshots[i].drv->states[idx].enter==lpm_cpuidle_state_enter)
   snapshots[i].drv->states[idx].enter=snapshots[i].enter[idx];
  if(snapshots[i].drv->states[idx].enter_s2idle==lpm_s2idle_state_enter)
   snapshots[i].drv->states[idx].enter_s2idle=snapshots[i].s2idle[idx];
 }
 cpuidle_resume_and_unlock();cpus_read_unlock();
 kfree(snapshots);snapshots=NULL;snapshot_count=0;
}
static void lpm_core_release(void)
{
 core_ready=false;
 if(pm_registered){unregister_pm_notifier(&lpm_pm_notifier_func);pm_registered=false;}
 if(early_registered){unregister_pm_notifier(&lpm_pm_early_notifier_func);early_registered=false;}
 if(syscore_registered){unregister_syscore_ops(&lpm_suspend);syscore_registered=false;}
 lpm_hooks_remove();
 if(suspend_sys_sync_work_queue){cancel_work_sync(&suspend_sys_sync_work);destroy_workqueue(suspend_sys_sync_work_queue);suspend_sys_sync_work_queue=NULL;}
 if(lpm_lock){__pm_relax(lpm_lock);wakeup_source_unregister(lpm_lock);lpm_lock=NULL;}
 lpm_platform_deinit();
}
static int __init lpm_init(void)
{
 struct device_node *np;const char *method;int ret;
 np=of_find_compatible_node(NULL,NULL,MTK_LPM_DTS_COMPATIBLE);if(!np)return -ENODEV;
 ret=of_property_read_string(np,"suspend-method",&method);
 if(ret || strcmp(method,"enable")){of_node_put(np);return -EOPNOTSUPP;}
 of_node_put(np);
 lpm_system.suspend.flag=pm_suspend_default_s2idle()?LPM_REQ_NOSYSCORE_CB:0;
 suspend_sys_sync_work_queue=alloc_ordered_workqueue("fs_suspend_syssync",WQ_MEM_RECLAIM);
 if(!suspend_sys_sync_work_queue)return -ENOMEM;
 ret=lpm_hooks_install();if(ret)goto fail;
 if(!(lpm_system.suspend.flag&LPM_REQ_NOSYSCORE_CB)){register_syscore_ops(&lpm_suspend);syscore_registered=true;}
 ret=register_pm_notifier(&lpm_pm_early_notifier_func);if(ret)goto fail;early_registered=true;
 ret=register_pm_notifier(&lpm_pm_notifier_func);if(ret)goto fail;pm_registered=true;
 /* Native platform firmware configuration is a separate qualification stage. */
 if(native_platform_config){ret=lpm_platform_init();if(ret)goto fail;}
 core_ready=true;return 0;
fail:lpm_core_release();return ret;
}
static void __exit lpm_deinit(void){lpm_core_release();}
module_init(lpm_init);module_exit(lpm_deinit);
MODULE_LICENSE("GPL");
