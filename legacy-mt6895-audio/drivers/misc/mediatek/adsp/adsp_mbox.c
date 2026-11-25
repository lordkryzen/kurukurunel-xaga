// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/vmalloc.h>      /* needed by vmalloc */
#include <linux/interrupt.h>
#include <linux/overflow.h>
#include <linux/wait.h>
#include <linux/sched/clock.h>
#include "adsp_core.h"
#include "adsp_platform.h"
#include "adsp_mbox.h"
#include "adsp_mbox_lifecycle.h"
#include "adsp_ipi_lifetime.h"

static int adsp_mbox_pin_cb(unsigned int id, void *prdata, void *buf,
			    unsigned int len);

static void adsp_ipi_cb(struct mtk_mbox_pin_recv *pin, void *priv);

struct mtk_mbox_info adsp_mbox_table[ADSP_IPI_CH_CNT] = {
	{ .opt = MBOX_OPT_QUEUE_DIR, .is64d = true},
	{ .opt = MBOX_OPT_DIRECT, .is64d = true},
	{ .opt = MBOX_OPT_QUEUE_DIR, .is64d = true},
	{ .opt = MBOX_OPT_DIRECT, .is64d = true},
};

struct mtk_mbox_pin_send adsp_mbox_pin_send[ADSP_TOTAL_SEND_PIN] = {
	{
		.mbox = ADSP_MBOX0_CH_ID,
		.offset = ADSP_MBOX_SEND_SLOT_OFFSET,
		.msg_size = ADSP_MBOX_SLOT_COUNT,
		.pin_index = 0,
	},
	{
		.mbox = ADSP_MBOX2_CH_ID,
		.offset = ADSP_MBOX_SEND_SLOT_OFFSET,
		.msg_size = ADSP_MBOX_SLOT_COUNT,
		.pin_index = 0,
	}
};

struct mtk_mbox_pin_recv adsp_mbox_pin_recv[ADSP_TOTAL_RECV_PIN] = {
	{
		.mbox = ADSP_MBOX1_CH_ID,
		.offset = ADSP_MBOX_RECV_SLOT_OFFSET,
		.cb_ctx_opt = MBOX_CB_IN_PROCESS,
		.msg_size = ADSP_MBOX_SLOT_COUNT,
		.pin_index = 0,
		.mbox_pin_cb = adsp_mbox_pin_cb,
	},
	{
		.mbox = ADSP_MBOX3_CH_ID,
		.offset = ADSP_MBOX_RECV_SLOT_OFFSET,
		.cb_ctx_opt = MBOX_CB_IN_PROCESS,
		.msg_size = ADSP_MBOX_SLOT_COUNT,
		.pin_index = 0,
		.mbox_pin_cb = adsp_mbox_pin_cb,
	}
};

struct mtk_mbox_device adsp_mboxdev = {
	.name = "adsp_mailbox",
	.pin_recv_table = adsp_mbox_pin_recv,
	.pin_send_table = adsp_mbox_pin_send,
	.info_table = adsp_mbox_table,
	.count = ADSP_IPI_CH_CNT,
	.recv_count = ADSP_TOTAL_RECV_PIN,
	.send_count = ADSP_TOTAL_SEND_PIN,
	.ipi_cb = adsp_ipi_cb,
	/*.post_cb = adsp_post_cb, */
};

static void adsp_mbox_dump_active(void)
{
#if IS_ENABLED(CONFIG_MTK_IRQ_DBG)
	mt_irq_dump_status(adsp_mbox_table[ADSP_MBOX1_CH_ID].irq_num);
	mt_irq_dump_status(adsp_mbox_table[ADSP_MBOX3_CH_ID].irq_num);
#endif
	mtk_mbox_dump_recv_pin(&adsp_mboxdev, &adsp_mbox_pin_recv[0]);
	mtk_mbox_dump_recv_pin(&adsp_mboxdev, &adsp_mbox_pin_recv[1]);
}

static int adsp_mbox_send_active(struct mtk_mbox_pin_send *pin_send, void *msg,
		unsigned int wait)
{
	int result = MBOX_DONE;
	struct mtk_mbox_device *mbdev = &adsp_mboxdev;
	ktime_t start_time;
	s64 time_ipc_us;

	if (mutex_trylock(&pin_send->mutex_send) == 0) {
		pr_info("%s, mbox %d mutex_trylock busy",
			__func__, pin_send->mbox);
		return MBOX_PIN_BUSY;
	}

	if (mtk_mbox_check_send_irq(mbdev, pin_send->mbox,
				    pin_send->pin_index)) {
		result = MBOX_PIN_BUSY;
		goto EXIT;
	}

	result = mtk_mbox_write_hd(mbdev, pin_send->mbox,
				pin_send->offset, msg);

	if (result != MBOX_DONE) {
		pr_err("%s() error mbox%d write, result %d\n",
		       __func__, pin_send->mbox, result);
		goto EXIT;
	}

	dsb(SY);

	result = mtk_mbox_trigger_irq(mbdev, pin_send->mbox,
				      0x1 << pin_send->pin_index);

	if (result != MBOX_DONE) {
		pr_err("%s() error mbox%d trigger, result %d\n",
		       __func__, pin_send->mbox, result);
		goto EXIT;
	}

	if (wait) {
		start_time = ktime_get();
		while (mtk_mbox_check_send_irq(mbdev, pin_send->mbox,
						pin_send->pin_index)) {
			time_ipc_us = ktime_us_delta(ktime_get(), start_time);
			if (time_ipc_us > 1000) {/* 1 ms */
				pr_warn("%s, time_ipc_us > 1000", __func__);
				break;
			}
		}
	}
EXIT:
	mutex_unlock(&pin_send->mutex_send);
	return result;
}

static int adsp_mbox_pin_cb(unsigned int id, void *prdata, void *buf,
			    unsigned int len)
{
	struct adsp_ipi_handler *entry;
	adsp_ipi_callback_t handler;
	u32 core_id;
	int ret = -1;

	if (!prdata)
		return ADSP_IPI_ERROR;
	core_id = *(u32 *)prdata;
	if (core_id >= get_adsp_core_total())
		return ADSP_IPI_ERROR;
	adsp_mt_clr_spm(core_id);
	entry = adsp_ipi_handler_get(id, NULL);
	if (!entry)
		return ADSP_IPI_ERROR;
	handler = adsp_ipi_handler_callback(entry);
	if (id != ADSP_IPI_ADSP_A_READY && id != ADSP_IPI_DVFS_SUSPEND &&
	    id != ADSP_IPI_LOGGER_INIT) {
		ret = adsp_ipi_queue_recv(core_id, id, buf, len, handler);
	}
	/* Successful enqueue owns a separate lease acquired by the queue wrapper. */
	if (ret)
		adsp_ipi_handler_invoke(entry, id, buf, len);
	adsp_ipi_handler_put(entry);
	return ADSP_IPI_DONE;
}

static void adsp_ipi_cb(struct mtk_mbox_pin_recv *pin, void *priv)
{
	struct mtk_mbox_pin_snapshot view;
	struct mtk_ipi_msg *ipimsg;
	void *buf;

	if (pin->cb_ctx_opt != MBOX_CB_IN_PROCESS ||
	    !mtk_mbox_pin_get(pin, &view))
		return;
	if (view.callback) {
		ipimsg = view.buf;
		buf = view.buf + sizeof(struct mtk_ipi_msg_hd);
		pin->recv_record.pre_timestamp = cpu_clock(0);
		view.callback(ipimsg->ipihd.id, view.prdata, buf,
				(unsigned int)ipimsg->ipihd.len);
		pin->recv_record.post_timestamp = cpu_clock(0);
		pin->recv_record.cb_count++;
	}
	mtk_mbox_pin_put(&view);
}

struct adsp_mbox_state {
	struct platform_device *pdev;
	unsigned int irqs;
	bool wake[ADSP_IPI_CH_CNT];
	void *buffers[ADSP_TOTAL_RECV_PIN];
	bool published[ADSP_TOTAL_RECV_PIN];
	u32 core_ids[ADSP_TOTAL_RECV_PIN];
	bool ready;
	unsigned int senders;
	wait_queue_head_t send_wait;
};
static DEFINE_MUTEX(adsp_mbox_owner_lock);
static DEFINE_SPINLOCK(adsp_mbox_sender_lock);
static struct adsp_mbox_state *adsp_mbox_owner;

/* Non-sleeping admission also covers callers using the trylock send path. */
static struct adsp_mbox_state *adsp_mbox_get_sender(struct mtk_mbox_pin_send *pin)
{
	struct adsp_mbox_state *state;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&adsp_mbox_sender_lock, flags);
	state = adsp_mbox_owner;
	if (!state || !state->ready)
		goto absent;
	for (i = 0; i < adsp_mboxdev.send_count; i++) {
		if (pin == &adsp_mboxdev.pin_send_table[i]) {
			state->senders++;
			spin_unlock_irqrestore(&adsp_mbox_sender_lock, flags);
			return state;
		}
	}
absent:
	spin_unlock_irqrestore(&adsp_mbox_sender_lock, flags);
	return NULL;
}

static void adsp_mbox_put_sender(struct adsp_mbox_state *state)
{
	unsigned long flags;

	spin_lock_irqsave(&adsp_mbox_sender_lock, flags);
	state->senders--;
	/* Wake under the lock: release cannot free state before this final access. */
	if (!state->senders)
		wake_up_all(&state->send_wait);
	spin_unlock_irqrestore(&adsp_mbox_sender_lock, flags);
}

int adsp_mbox_send(struct mtk_mbox_pin_send *pin, void *msg, unsigned int wait)
{
	struct adsp_mbox_state *state = adsp_mbox_get_sender(pin);
	int ret;

	if (!state)
		return MBOX_PLT_ERR;
	ret = adsp_mbox_send_active(pin, msg, wait);
	adsp_mbox_put_sender(state);
	return ret;
}

void adsp_mbox_dump(void)
{
	struct adsp_mbox_state *state = adsp_mbox_get_sender(&adsp_mbox_pin_send[0]);

	if (!state)
		return;
	adsp_mbox_dump_active();
	adsp_mbox_put_sender(state);
}

/* Installed after all mappings: devres runs this before unmapping registers. */
static void adsp_mbox_release(void *data)
{
	struct adsp_mbox_state *state = data;
	struct mtk_mbox_device *mbdev = &adsp_mboxdev;
	unsigned int i;
	unsigned long flags;

	mutex_lock(&adsp_mbox_owner_lock);
	spin_lock_irqsave(&adsp_mbox_sender_lock, flags);
	state->ready = false;
	spin_unlock_irqrestore(&adsp_mbox_sender_lock, flags);
	mutex_unlock(&adsp_mbox_owner_lock);
	wait_event(state->send_wait, !READ_ONCE(state->senders));
	/* Pair with the final sender put before freeing its wait queue/state. */
	spin_lock_irqsave(&adsp_mbox_sender_lock, flags);
	spin_unlock_irqrestore(&adsp_mbox_sender_lock, flags);
	for (i = state->irqs; i > 0; i--) {
		struct mtk_mbox_info *info = &mbdev->info_table[i - 1];

		WRITE_ONCE(info->enable, false);
		if (state->wake[i - 1])
			disable_irq_wake(info->irq_num);
		free_irq(info->irq_num, info);
	}
	for (i = mbdev->recv_count; i > 0; i--) {
		if (state->published[i - 1]) {
			int ret = mtk_mbox_pin_unregister(&mbdev->pin_recv_table[i - 1]);

			/* Keep an exposed buffer if a violated context contract prevents drain. */
			if (WARN_ON(ret))
				return; /* Quarantine state as well as exposed buffers. */
		}
		vfree(state->buffers[i - 1]);
		state->buffers[i - 1] = NULL;
	}
	mutex_lock(&adsp_mbox_owner_lock);
	spin_lock_irqsave(&adsp_mbox_sender_lock, flags);
	if (adsp_mbox_owner == state)
		adsp_mbox_owner = NULL;
	spin_unlock_irqrestore(&adsp_mbox_sender_lock, flags);
	mutex_unlock(&adsp_mbox_owner_lock);
	kfree(state);
}

int adsp_mbox_probe(struct platform_device *pdev)
{
	struct mtk_mbox_device *mbdev = &adsp_mboxdev;
	struct adsp_mbox_state *state;
	unsigned int i;
	unsigned long flags;
	int ret = mtk_mbox_pin_mutation_check();

	if (ret)
		return ret;
	if (!pdev || mbdev->recv_count != ADSP_TOTAL_RECV_PIN ||
	    mbdev->count != ADSP_IPI_CH_CNT)
		return -EINVAL;
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;
	state->pdev = pdev;
	init_waitqueue_head(&state->send_wait);
	mutex_lock(&adsp_mbox_owner_lock);
	if (adsp_mbox_owner) {
		mutex_unlock(&adsp_mbox_owner_lock);
		kfree(state);
		return -EBUSY;
	}
	spin_lock_irqsave(&adsp_mbox_sender_lock, flags);
	adsp_mbox_owner = state;
	spin_unlock_irqrestore(&adsp_mbox_sender_lock, flags);
	mutex_unlock(&adsp_mbox_owner_lock);

	for (i = 0; i < mbdev->send_count; i++) {
		mutex_init(&mbdev->pin_send_table[i].mutex_send);
		spin_lock_init(&mbdev->pin_send_table[i].pin_lock);
	}
	for (i = 0; i < mbdev->recv_count; i++) {
		struct mtk_mbox_pin_recv *pin = &mbdev->pin_recv_table[i];
		size_t size;

		if (check_mul_overflow((size_t)pin->msg_size, (size_t)MBOX_SLOT_SIZE, &size) ||
		    size < sizeof(struct mtk_ipi_msg_hd)) {
			ret = -EINVAL;
			goto fail;
		}
		init_completion(&pin->notify);
		spin_lock_init(&pin->pin_lock);
		state->buffers[i] = vzalloc(size);
		if (!state->buffers[i]) {
			ret = -ENOMEM;
			goto fail;
		}
		/* No callback until the core binds; early IRQs cannot dereference NULL prdata. */
		ret = mtk_mbox_pin_register(pin, NULL, NULL, state->buffers[i]);
		if (ret)
			goto fail;
		state->published[i] = true;
		state->core_ids[i] = i;
	}
	for (i = 0; i < mbdev->count; i++) {
		ret = mtk_mbox_probe(pdev, mbdev, i);
		if (ret)
			goto fail;
		state->irqs++;
		ret = enable_irq_wake(mbdev->info_table[i].irq_num);
		if (ret)
			goto fail;
		state->wake[i] = true;
	}
	mutex_lock(&adsp_mbox_owner_lock);
	spin_lock_irqsave(&adsp_mbox_sender_lock, flags);
	state->ready = true;
	spin_unlock_irqrestore(&adsp_mbox_sender_lock, flags);
	mutex_unlock(&adsp_mbox_owner_lock);
	return devm_add_action_or_reset(&pdev->dev, adsp_mbox_release, state);
fail:
	adsp_mbox_release(state);
	return ret;
}

int adsp_mbox_bind_core(struct mtk_mbox_pin_recv *pin, unsigned int core_id)
{
	unsigned int i;
	int ret = mtk_mbox_pin_mutation_check();

	if (ret)
		return ret;
	mutex_lock(&adsp_mbox_owner_lock);
	if (!adsp_mbox_owner || !adsp_mbox_owner->ready) {
		ret = -ENODEV;
		goto out;
	}
	for (i = 0; i < adsp_mboxdev.recv_count; i++) {
		if (pin != &adsp_mboxdev.pin_recv_table[i])
			continue;
		if (core_id != adsp_mbox_owner->core_ids[i]) {
			ret = -EINVAL;
			goto out;
		}
		ret = mtk_mbox_cb_register(&adsp_mboxdev, i, adsp_mbox_pin_cb,
				&adsp_mbox_owner->core_ids[i]);
		goto out;
	}
	ret = -EINVAL;
out:
	mutex_unlock(&adsp_mbox_owner_lock);
	return ret;
}
EXPORT_SYMBOL(adsp_mbox_probe);

struct mtk_mbox_pin_send *get_adsp_mbox_pin_send(int index)
{
	int i;
	struct mtk_mbox_info *mbox;
	struct mtk_mbox_pin_send *pin_send;

	if (index < 0 || index >= ADSP_IPI_CH_CNT)
		return ERR_PTR(-EINVAL);

	mbox = adsp_mboxdev.info_table + index;

	if (!READ_ONCE(mbox->enable)) {
		pr_info("the channel %d not enable", index);
		return ERR_PTR(-ENODEV);
	}

	for (i = 0; i < ADSP_TOTAL_SEND_PIN; i++) {
		pin_send = &adsp_mbox_pin_send[i];
		if (pin_send->mbox == index)
			return pin_send;
	}

	pr_info("the channel %d is not send pin", index);
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL(get_adsp_mbox_pin_send);

struct mtk_mbox_pin_recv *get_adsp_mbox_pin_recv(int index)
{
	int i;
	struct mtk_mbox_info *mbox;
	struct mtk_mbox_pin_recv *pin_recv;

	if (index < 0 || index >= ADSP_IPI_CH_CNT)
		return ERR_PTR(-EINVAL);

	mbox = adsp_mboxdev.info_table + index;

	if (!READ_ONCE(mbox->enable)) {
		pr_info("the channel %d not enable", index);
		return ERR_PTR(-ENODEV);
	}

	for (i = 0; i < ADSP_TOTAL_RECV_PIN; i++) {
		pin_recv = &adsp_mbox_pin_recv[i];
		if (pin_recv->mbox == index)
			return pin_recv;
	}

	pr_info("the channel %d is not recv pin", index);
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL(get_adsp_mbox_pin_recv);



