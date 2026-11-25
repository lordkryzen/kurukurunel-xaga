// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <lpm_call.h>
static DEFINE_SPINLOCK(lpm_plat_call_locker);
static LIST_HEAD(lpm_callees);
int lpm_callee_registry(struct lpm_callee *callee)
{
 struct lpm_callee *p;unsigned long flags;int ret=0;
 if(!callee)return -EINVAL;
 spin_lock_irqsave(&lpm_plat_call_locker,flags);
 list_for_each_entry(p,&lpm_callees,list)if(p==callee || p->uid==callee->uid){ret=-EEXIST;goto out;}
 callee->ref=0;list_add(&callee->list,&lpm_callees);
out:spin_unlock_irqrestore(&lpm_plat_call_locker,flags);return ret;
}
EXPORT_SYMBOL(lpm_callee_registry);
int lpm_callee_unregistry(struct lpm_callee *callee)
{
 struct lpm_callee *p;unsigned long flags;int ret=-ENOENT;
 if(!callee)return -EINVAL;
 spin_lock_irqsave(&lpm_plat_call_locker,flags);
 list_for_each_entry(p,&lpm_callees,list)if(p==callee){if(p->ref)ret=-EBUSY;else{list_del_init(&p->list);ret=0;}break;}
 spin_unlock_irqrestore(&lpm_plat_call_locker,flags);return ret;
}
EXPORT_SYMBOL(lpm_callee_unregistry);
int lpm_callee_get_impl(int uid,const struct lpm_callee **callee)
{
 struct lpm_callee *p;unsigned long flags;int ret=-ENOENT;
 if(!callee)return -EINVAL;*callee=NULL;
 spin_lock_irqsave(&lpm_plat_call_locker,flags);
 list_for_each_entry(p,&lpm_callees,list)if(p->uid==uid){if(p->ref==UINT_MAX)ret=-EOVERFLOW;else{p->ref++;*callee=p;ret=0;}break;}
 spin_unlock_irqrestore(&lpm_plat_call_locker,flags);return ret;
}
EXPORT_SYMBOL(lpm_callee_get_impl);
int lpm_callee_put_impl(const struct lpm_callee *callee)
{
 struct lpm_callee *p;unsigned long flags;int ret=-ENOENT;
 if(!callee)return -EINVAL;
 spin_lock_irqsave(&lpm_plat_call_locker,flags);
 list_for_each_entry(p,&lpm_callees,list)if(p==callee){if(!p->ref)ret=-EINVAL;else{p->ref--;ret=0;}break;}
 spin_unlock_irqrestore(&lpm_plat_call_locker,flags);return ret;
}
EXPORT_SYMBOL(lpm_callee_put_impl);
