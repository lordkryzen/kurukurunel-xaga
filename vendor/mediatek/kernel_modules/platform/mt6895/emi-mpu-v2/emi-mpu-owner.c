// SPDX-License-Identifier: GPL-2.0
/* Isolated reader-lifetime candidate; common EMI APIs are not wired here yet. */
#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/refcount.h>
#include <linux/slab.h>
#include "emi-mpu-owner.h"

struct emimpu_owner {
	struct emi_mpu *mpu;
	refcount_t readers;
	struct completion released;
};

static DEFINE_MUTEX(emimpu_owner_lock);
static struct emimpu_owner __rcu *emimpu_active_owner;

int mtk_emimpu_owner_publish(struct emi_mpu *mpu)
{
	struct emimpu_owner *owner;
	int ret = 0;

	if (!mpu)
		return -EINVAL;
	owner = kzalloc(sizeof(*owner), GFP_KERNEL);
	if (!owner)
		return -ENOMEM;
	owner->mpu = mpu;
	refcount_set(&owner->readers, 1);
	init_completion(&owner->released);

	mutex_lock(&emimpu_owner_lock);
	if (rcu_access_pointer(emimpu_active_owner)) {
		ret = -EBUSY;
	} else {
		rcu_assign_pointer(emimpu_active_owner, owner);
		owner = NULL;
	}
	mutex_unlock(&emimpu_owner_lock);
	kfree(owner);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_emimpu_owner_publish);

struct emimpu_owner *mtk_emimpu_owner_get(void)
{
	struct emimpu_owner *owner;

	rcu_read_lock();
	owner = rcu_dereference(emimpu_active_owner);
	if (owner)
		refcount_inc(&owner->readers);
	rcu_read_unlock();
	return owner;
}
EXPORT_SYMBOL_GPL(mtk_emimpu_owner_get);

void mtk_emimpu_owner_put(struct emimpu_owner *owner)
{
	if (owner && refcount_dec_and_test(&owner->readers))
		complete(&owner->released);
}
EXPORT_SYMBOL_GPL(mtk_emimpu_owner_put);

struct emi_mpu *mtk_emimpu_owner_device(struct emimpu_owner *owner)
{
	return owner ? owner->mpu : NULL;
}
EXPORT_SYMBOL_GPL(mtk_emimpu_owner_device);

void mtk_emimpu_owner_withdraw(struct emi_mpu *mpu)
{
	struct emimpu_owner *owner;

	mutex_lock(&emimpu_owner_lock);
	owner = rcu_dereference_protected(emimpu_active_owner,
		lockdep_is_held(&emimpu_owner_lock));
	if (!owner || owner->mpu != mpu)
		goto out;

	rcu_assign_pointer(emimpu_active_owner, NULL);
	/* Keep the publisher reference until all possible increments finish. */
	synchronize_rcu();
	mtk_emimpu_owner_put(owner);
	wait_for_completion(&owner->released);
	kfree(owner);
out:
	mutex_unlock(&emimpu_owner_lock);
}
EXPORT_SYMBOL_GPL(mtk_emimpu_owner_withdraw);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Isolated EMI MPU reader-lifetime candidate");
