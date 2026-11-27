/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MI_TASK_QOS_H
#define MI_TASK_QOS_H
#include <linux/compat.h>
#include <linux/cred.h>
#include <linux/hashtable.h>
#include <linux/miscdevice.h>
#include <linux/pid.h>
#include <linux/uaccess.h>

#define QOS_SET _IOW('x', 7, __u64)
#define QOS_GET_PID _IOWR('x', 8, __u64)
#define QOS_GET_TID _IOWR('x', 9, __u64)
#define QOS_RECORD_LIMIT 4096
struct qos_request { u32 qos; s32 tid; };
struct qos_record {
 struct hlist_node node;
 struct task_struct *task;
 u32 qos;
};
static DEFINE_HASHTABLE(qos_records, 8);
static DEFINE_SPINLOCK(qos_lock);
static unsigned int qos_records_count;
static char *qos_device_name = "xr_fbt";
module_param(qos_device_name, charp, 0444);

/* Explicit task annotations only; no scheduler or inherited-priority policy. */
static struct qos_record *qos_find(struct task_struct *task)
{
 struct qos_record *record;
 hash_for_each_possible(qos_records, record, node, (unsigned long)task)
  if (record->task == task) return record;
 return NULL;
}
static unsigned int task_qos_level(struct task_struct *task)
{
 struct qos_record *record;
 unsigned long flags;
 unsigned int level = 0;
 spin_lock_irqsave(&qos_lock, flags);
 record = qos_find(task);
 if (record) level = record->qos & 0xf;
 spin_unlock_irqrestore(&qos_lock, flags);
 return level;
}
static void task_qos_exit(struct task_struct *task)
{
 struct qos_record *record;
 unsigned long flags;
 spin_lock_irqsave(&qos_lock, flags);
 record = qos_find(task);
 if (record) { hash_del(&record->node); qos_records_count--; }
 spin_unlock_irqrestore(&qos_lock, flags);
 if (record) { put_task_struct(record->task); kfree(record); }
}
static void task_qos_clear(void)
{
 struct qos_record *record;
 struct hlist_node *tmp;
 unsigned int bucket;
 /* Called after the endpoint and tracepoint callbacks are drained. */
 hash_for_each_safe(qos_records, bucket, tmp, record, node) {
  hash_del(&record->node); put_task_struct(record->task); kfree(record);
 }
 qos_records_count = 0;
}
static long qos_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
 struct qos_request request;
 struct qos_record *record, *fresh = NULL;
 struct task_struct *task;
 struct pid *pid;
 unsigned long flags;
 unsigned int bucket, level = 0;
 long ret = 0;
 const struct cred *target_cred;
 kuid_t caller = current_uid();
 bool permitted;
 void __user *user = (void __user *)arg;
 if (cmd != QOS_SET && cmd != QOS_GET_PID && cmd != QOS_GET_TID) return -ENOTTY;
 if (copy_from_user(&request, user, sizeof(request))) return -EFAULT;
 if (request.tid < 1 || (cmd == QOS_SET && (request.qos & 0xf) > 10)) return -EINVAL;
 pid = find_get_pid(request.tid);
 task = pid ? get_pid_task(pid, PIDTYPE_PID) : NULL;
 put_pid(pid);
 if (!task) return -ESRCH;
 target_cred = get_task_cred(task);
 permitted = uid_eq(caller, GLOBAL_ROOT_UID) || __kuid_val(caller) == 1000 ||
             uid_eq(caller, target_cred->uid);
 put_cred(target_cred);
 if (!permitted) { ret = -EPERM; goto put; }
 if (cmd == QOS_SET && (request.qos & 0xf)) {
  fresh = kzalloc(sizeof(*fresh), GFP_KERNEL);
  if (!fresh) { ret = -ENOMEM; goto put; }
 }
 spin_lock_irqsave(&qos_lock, flags);
 if ((task->flags & PF_EXITING) || !pid_alive(task)) { ret = -ESRCH; goto unlock; }
 if (cmd == QOS_SET) {
  record = qos_find(task);
  if (!(request.qos & 0xf)) {
   if (record) { hash_del(&record->node); qos_records_count--; }
  } else if (record) {
   record->qos = request.qos;
   record = NULL;
  } else if (qos_records_count >= QOS_RECORD_LIMIT) {
   ret = -ENOSPC;
  } else {
   get_task_struct(task); fresh->task = task; fresh->qos = request.qos;
   hash_add(qos_records, &fresh->node, (unsigned long)task);
   qos_records_count++; fresh = NULL;
  }
  spin_unlock_irqrestore(&qos_lock, flags);
  if (record) { put_task_struct(record->task); kfree(record); }
  goto put;
 }
 if (cmd == QOS_GET_TID) {
  record = qos_find(task);
  if (record) level = record->qos & 0xf;
 } else {
  hash_for_each(qos_records, bucket, record, node)
   if (record->task->group_leader == task->group_leader)
    level = max(level, record->qos & 0xf);
 }
 request.qos = level;
unlock:
 spin_unlock_irqrestore(&qos_lock, flags);
 if (!ret && cmd != QOS_SET && copy_to_user(user, &request, sizeof(request))) ret = -EFAULT;
put:
 kfree(fresh); put_task_struct(task); return ret;
}
#ifdef CONFIG_COMPAT
static long qos_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{ return qos_ioctl(file, cmd, (unsigned long)compat_ptr(arg)); }
#endif
static const struct file_operations qos_fops = {
 .owner = THIS_MODULE, .unlocked_ioctl = qos_ioctl,
#ifdef CONFIG_COMPAT
 .compat_ioctl = qos_compat_ioctl,
#endif
 .open = nonseekable_open,
};
static struct miscdevice qos_device = {
 .minor = MISC_DYNAMIC_MINOR, .name = "xr_fbt", .fops = &qos_fops, .mode = 0660,
};
#endif
