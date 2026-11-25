// SPDX-License-Identifier: GPL-2.0
/* Immutable Linux callback registrations; leases may cross worker tasks. */
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/srcu.h>
#include <linux/wait.h>
#include "adsp_helper.h"
#include "adsp_ipi_lifetime.h"

struct adsp_ipi_handler {
	void (*handler)(int, void *, unsigned int);
	unsigned int users;
	wait_queue_head_t wait;
};

/* Stack-owned invocation markers reject self/cross-handler drain deadlocks. */
struct adsp_ipi_invocation {
	struct task_struct *task;
	struct adsp_ipi_invocation *next;
};

static DEFINE_MUTEX(adsp_ipi_writer_lock);
static DEFINE_SPINLOCK(adsp_ipi_handler_lock);
static struct adsp_ipi_handler *adsp_ipi_handlers[ADSP_NR_IPI];
static struct adsp_ipi_invocation *adsp_ipi_invocations;

static bool adsp_ipi_writer_allowed(void)
{
	struct adsp_ipi_invocation *call;
	unsigned long flags;
	bool allowed = true;

	if (in_interrupt() || irqs_disabled() || in_atomic())
		return false;
	spin_lock_irqsave(&adsp_ipi_handler_lock, flags);
	for (call = adsp_ipi_invocations; call; call = call->next) {
		if (call->task == current) {
			allowed = false;
			break;
		}
	}
	spin_unlock_irqrestore(&adsp_ipi_handler_lock, flags);
	return allowed;
}

int adsp_ipi_mutation_check(void)
{
	if (in_interrupt() || irqs_disabled() || in_atomic())
		return -EWOULDBLOCK;
	return adsp_ipi_writer_allowed() ? 0 : -EDEADLK;
}
EXPORT_SYMBOL_GPL(adsp_ipi_mutation_check);

struct adsp_ipi_handler *adsp_ipi_handler_get(unsigned int id,
	void (*match)(int, void *, unsigned int))
{
	struct adsp_ipi_handler *entry = NULL;
	unsigned long flags;

	if (id >= ADSP_NR_IPI)
		return NULL;
	spin_lock_irqsave(&adsp_ipi_handler_lock, flags);
	entry = adsp_ipi_handlers[id];
	if (entry && (!match || entry->handler == match))
		entry->users++;
	else
		entry = NULL;
	spin_unlock_irqrestore(&adsp_ipi_handler_lock, flags);
	return entry;
}
EXPORT_SYMBOL_GPL(adsp_ipi_handler_get);

void adsp_ipi_handler_put(struct adsp_ipi_handler *entry)
{
	unsigned long flags;

	if (!entry)
		return;
	spin_lock_irqsave(&adsp_ipi_handler_lock, flags);
	if (WARN_ON(!entry->users)) {
		spin_unlock_irqrestore(&adsp_ipi_handler_lock, flags);
		return;
	}
	if (!--entry->users)
		wake_up_all(&entry->wait);
	spin_unlock_irqrestore(&adsp_ipi_handler_lock, flags);
}
EXPORT_SYMBOL_GPL(adsp_ipi_handler_put);

static void adsp_ipi_invocation_begin(struct adsp_ipi_invocation *call)
{
	unsigned long flags;

	call->task = current;
	spin_lock_irqsave(&adsp_ipi_handler_lock, flags);
	call->next = adsp_ipi_invocations;
	adsp_ipi_invocations = call;
	spin_unlock_irqrestore(&adsp_ipi_handler_lock, flags);
}

static void adsp_ipi_invocation_end(struct adsp_ipi_invocation *call)
{
	struct adsp_ipi_invocation **cursor;
	unsigned long flags;

	spin_lock_irqsave(&adsp_ipi_handler_lock, flags);
	for (cursor = &adsp_ipi_invocations; *cursor; cursor = &(*cursor)->next) {
		if (*cursor == call) {
			*cursor = call->next;
			break;
		}
	}
	spin_unlock_irqrestore(&adsp_ipi_handler_lock, flags);
}

void adsp_ipi_handler_invoke(struct adsp_ipi_handler *entry, unsigned int id,
	void *buf, unsigned int len)
{
	struct adsp_ipi_invocation call;

	if (!entry)
		return;
	adsp_ipi_invocation_begin(&call);
	entry->handler(id, buf, len);
	adsp_ipi_invocation_end(&call);
}
EXPORT_SYMBOL_GPL(adsp_ipi_handler_invoke);

adsp_ipi_callback_t adsp_ipi_handler_callback(struct adsp_ipi_handler *entry)
{
	return entry->handler; /* Caller owns a lease. */
}

static void adsp_ipi_handler_retire(struct adsp_ipi_handler *entry)
{
	unsigned long flags;

	if (!entry)
		return;
	wait_event(entry->wait, !READ_ONCE(entry->users));
	/* Pair with final put's wake before freeing the wait queue. */
	spin_lock_irqsave(&adsp_ipi_handler_lock, flags);
	spin_unlock_irqrestore(&adsp_ipi_handler_lock, flags);
	kfree(entry);
}

/* Serialized writers first close admission, then drain, then republish. */
static enum adsp_ipi_status adsp_ipi_handler_replace(unsigned int id,
	struct adsp_ipi_handler *replacement)
{
	struct adsp_ipi_handler *old;
	unsigned long flags;

	mutex_lock(&adsp_ipi_writer_lock);
	spin_lock_irqsave(&adsp_ipi_handler_lock, flags);
	old = adsp_ipi_handlers[id];
	if (old && replacement && old->handler == replacement->handler) {
		spin_unlock_irqrestore(&adsp_ipi_handler_lock, flags);
		mutex_unlock(&adsp_ipi_writer_lock);
		kfree(replacement);
		return ADSP_IPI_DONE;
	}
	adsp_ipi_handlers[id] = NULL;
	spin_unlock_irqrestore(&adsp_ipi_handler_lock, flags);
	adsp_ipi_handler_retire(old);
	spin_lock_irqsave(&adsp_ipi_handler_lock, flags);
	adsp_ipi_handlers[id] = replacement;
	spin_unlock_irqrestore(&adsp_ipi_handler_lock, flags);
	mutex_unlock(&adsp_ipi_writer_lock);
	return ADSP_IPI_DONE;
}

enum adsp_ipi_status adsp_ipi_registration(enum adsp_ipi_id id,
	void (*handler)(int, void *, unsigned int), const char *name)
{
	struct adsp_ipi_handler *entry;

	/* Names were never consumed by this transport; do not retain caller memory. */
	(void)name;
	if (id < 0 || id >= ADSP_NR_IPI || !handler || !adsp_ipi_writer_allowed())
		return ADSP_IPI_ERROR;
	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return ADSP_IPI_ERROR;
	entry->handler = handler;
	init_waitqueue_head(&entry->wait);
	return adsp_ipi_handler_replace(id, entry);
}
EXPORT_SYMBOL_GPL(adsp_ipi_registration);

enum adsp_ipi_status adsp_ipi_unregistration(enum adsp_ipi_id id)
{
	if (id < 0 || id >= ADSP_NR_IPI || !adsp_ipi_writer_allowed())
		return ADSP_IPI_ERROR;
	return adsp_ipi_handler_replace(id, NULL);
}
EXPORT_SYMBOL_GPL(adsp_ipi_unregistration);

/* SRCU covers the entire hook call, including synchronous queue waits. */
static DEFINE_MUTEX(adsp_ipi_hook_writer_lock);
DEFINE_STATIC_SRCU(adsp_ipi_hook_srcu);
static int (*adsp_send_hook)(uint32_t, uint32_t, void *, uint32_t, uint32_t);
static int (*adsp_recv_hook)(uint32_t, uint32_t, void *, uint32_t,
	void (*)(int, void *, unsigned int));

void hook_ipi_queue_send_msg_handler(
	int (*handler)(uint32_t, uint32_t, void *, uint32_t, uint32_t))
{
	if (WARN_ON(!adsp_ipi_writer_allowed()))
		return;
	mutex_lock(&adsp_ipi_hook_writer_lock);
	WRITE_ONCE(adsp_send_hook, NULL);
	synchronize_srcu(&adsp_ipi_hook_srcu);
	WRITE_ONCE(adsp_send_hook, handler);
	mutex_unlock(&adsp_ipi_hook_writer_lock);
}
EXPORT_SYMBOL(hook_ipi_queue_send_msg_handler);

void unhook_ipi_queue_send_msg_handler(void)
{
	hook_ipi_queue_send_msg_handler(NULL);
}
EXPORT_SYMBOL(unhook_ipi_queue_send_msg_handler);

void hook_ipi_queue_recv_msg_hanlder(
	int (*handler)(uint32_t, uint32_t, void *, uint32_t,
		void (*)(int, void *, unsigned int)))
{
	if (WARN_ON(!adsp_ipi_writer_allowed()))
		return;
	mutex_lock(&adsp_ipi_hook_writer_lock);
	WRITE_ONCE(adsp_recv_hook, NULL);
	synchronize_srcu(&adsp_ipi_hook_srcu);
	WRITE_ONCE(adsp_recv_hook, handler);
	mutex_unlock(&adsp_ipi_hook_writer_lock);
}
EXPORT_SYMBOL(hook_ipi_queue_recv_msg_hanlder);

void unhook_ipi_queue_recv_msg_hanlder(void)
{
	hook_ipi_queue_recv_msg_hanlder(NULL);
}
EXPORT_SYMBOL(unhook_ipi_queue_recv_msg_hanlder);

int adsp_ipi_queue_send(unsigned int core, unsigned int id, void *buf,
	unsigned int len, unsigned int wait_ms)
{
	struct adsp_ipi_invocation call;
	int (*handler)(uint32_t, uint32_t, void *, uint32_t, uint32_t);
	int index, ret;

	index = srcu_read_lock(&adsp_ipi_hook_srcu);
	handler = READ_ONCE(adsp_send_hook);
	if (handler) {
		adsp_ipi_invocation_begin(&call);
		ret = handler(core, id, buf, len, wait_ms);
		adsp_ipi_invocation_end(&call);
	} else {
		ret = adsp_send_message(id, buf, len, wait_ms, core);
	}
	srcu_read_unlock(&adsp_ipi_hook_srcu, index);
	return ret;
}

int adsp_ipi_queue_recv(unsigned int core, unsigned int id, void *buf,
	unsigned int len, adsp_ipi_callback_t callback)
{
	struct adsp_ipi_invocation call;
	int (*handler)(uint32_t, uint32_t, void *, uint32_t,
		void (*)(int, void *, unsigned int));
	int index, ret = -ENODEV;

	index = srcu_read_lock(&adsp_ipi_hook_srcu);
	handler = READ_ONCE(adsp_recv_hook);
	if (handler) {
		adsp_ipi_invocation_begin(&call);
		ret = handler(core, id, buf, len, callback);
		adsp_ipi_invocation_end(&call);
	}
	srcu_read_unlock(&adsp_ipi_hook_srcu, index);
	return ret;
}
