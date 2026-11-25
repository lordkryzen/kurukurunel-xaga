// SPDX-License-Identifier: GPL-2.0
/* Isolated broker candidate; not connected to the aggregate or MPU IRQ yet. */
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/srcu.h>
#include "emi-mpu-md-callback.h"

struct emimpu_md_registration {
	emimpu_md_callback_t callback;
	struct module *owner;
	bool ready;
};

static DEFINE_MUTEX(emimpu_md_writer);
DEFINE_STATIC_SRCU(emimpu_md_srcu);
static struct emimpu_md_registration __rcu *emimpu_md_active;

int mtk_emimpu_md_callback_register(emimpu_md_callback_t callback,
		struct module *owner)
{
	struct emimpu_md_registration *registration, *existing;
	int ret = 0;

	if (!callback)
		return -EINVAL;
	registration = kmalloc(sizeof(*registration), GFP_KERNEL);
	if (!registration)
		return -ENOMEM;
	registration->callback = callback;
	registration->owner = owner;
	registration->ready = false;
	mutex_lock(&emimpu_md_writer);
	existing = rcu_dereference_protected(emimpu_md_active,
		lockdep_is_held(&emimpu_md_writer));
	if (existing) {
		if (existing->callback != callback || existing->owner != owner)
			ret = -EBUSY;
	} else {
		rcu_assign_pointer(emimpu_md_active, registration);
		registration = NULL;
	}
	mutex_unlock(&emimpu_md_writer);
	kfree(registration);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_emimpu_md_callback_register);

int mtk_emimpu_md_callback_activate(struct module *owner)
{
	struct emimpu_md_registration *registration;
	int ret = -ENOENT;

	mutex_lock(&emimpu_md_writer);
	registration = rcu_dereference_protected(emimpu_md_active,
		lockdep_is_held(&emimpu_md_writer));
	if (registration && registration->owner == owner) {
		WRITE_ONCE(registration->ready, true);
		ret = 0;
	}
	mutex_unlock(&emimpu_md_writer);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_emimpu_md_callback_activate);

static int emimpu_md_unregister(emimpu_md_callback_t callback,
		struct module *owner, bool match_callback)
{
	struct emimpu_md_registration *registration;
	int ret = -ENOENT;

	mutex_lock(&emimpu_md_writer);
	registration = rcu_dereference_protected(emimpu_md_active,
		lockdep_is_held(&emimpu_md_writer));
	if (!registration || (match_callback && registration->callback != callback) ||
	    registration->owner != owner)
		goto out;
	rcu_assign_pointer(emimpu_md_active, NULL);
	/* Serialize replacement epochs until every old dispatch has returned. */
	synchronize_srcu(&emimpu_md_srcu);
	kfree(registration);
	ret = 0;
out:
	mutex_unlock(&emimpu_md_writer);
	return ret;
}

int mtk_emimpu_md_callback_unregister(emimpu_md_callback_t callback,
		struct module *owner)
{
	return emimpu_md_unregister(callback, owner, true);
}
EXPORT_SYMBOL_GPL(mtk_emimpu_md_callback_unregister);

int mtk_emimpu_md_callback_unregister_owner(struct module *owner)
{
	return emimpu_md_unregister(NULL, owner, false);
}
EXPORT_SYMBOL_GPL(mtk_emimpu_md_callback_unregister_owner);

void mtk_emimpu_md_callback_notify(const char *message)
{
	struct emimpu_md_registration *registration;
	int index;

	index = srcu_read_lock(&emimpu_md_srcu);
	registration = srcu_dereference(emimpu_md_active, &emimpu_md_srcu);
	if (registration && READ_ONCE(registration->ready) &&
	    try_module_get(registration->owner)) {
		registration->callback(message);
		module_put(registration->owner);
	}
	srcu_read_unlock(&emimpu_md_srcu, index);
}
EXPORT_SYMBOL_GPL(mtk_emimpu_md_callback_notify);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Isolated EMI MPU modem callback ownership candidate");
