// SPDX-License-Identifier: GPL-2.0
//
// adsp_ipi_queue.c--  Mediatek ADSP IPI with queue
//
// Copyright (c) 2018 MediaTek Inc.


#include <adsp_ipi_queue.h>
#include <adsp_ipi_queue_lifecycle.h>
#include <linux/mutex.h>

#include <linux/types.h>
#include <linux/errno.h>

#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/spinlock.h>

#include <linux/delay.h>
#include <linux/jiffies.h>

#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/completion.h>
#include <linux/kref.h>
#include <linux/slab.h>

#include <uapi/linux/sched/types.h>

#include <audio_log.h>
#include <audio_assert.h>
#include <audio_ipi_platform.h>


#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
#include <adsp_helper.h>
#include <adsp_ipi_lifetime.h>
#endif

#if IS_ENABLED(CONFIG_MTK_AUDIO_CM4_SUPPORT)
#include <scp_ipi.h>
#endif



#include <audio_messenger_ipi.h>



/*
 * =============================================================================
 *                     log
 * =============================================================================
 */

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "[IPI][DSP_Q] %s(), " fmt "\n", __func__


/*
 * =============================================================================
 *                     MACRO
 * =============================================================================
 */

#define MAX_DSP_MSG_NUM_IN_QUEUE (64)
#define DSP_MSG_BUFFER_SIZE ((SHARE_BUF_SIZE) - 16)


/*
 * =============================================================================
 *                     struct def
 * =============================================================================
 */

struct adsp_ipi_handler;
struct dsp_tx_wait;

struct dsp_msg_t {
	/* Linux-only ownership; never transmitted in the DSP payload. */
	struct adsp_ipi_handler *adsp_handler;
	struct dsp_tx_wait *tx_wait;
	uint32_t ipi_id;
	uint8_t buf[DSP_MSG_BUFFER_SIZE]; /* TODO: use ring buf? */
	uint32_t len;

	/* recv msg callback */
	void (*ipi_handler)(int ipi_id, void *buf, unsigned int len);
};


struct dsp_queue_element_t {
	struct dsp_msg_t msg;

	spinlock_t element_lock;
	wait_queue_head_t element_wq;

	bool wait_in_thread;
	bool signal_arrival;
	int send_retval;
	uint32_t queue_counter;
};


enum { /* dsp_path_t */
	DSP_PATH_A2D = 0, /* AP to DSP */
	DSP_PATH_D2A = 1, /* DSP to AP */
	DSP_NUM_PATH
};


struct dsp_msg_queue_t {
	uint32_t dsp_id;   /* enum dsp_id_t */
	uint32_t dsp_path; /* enum dsp_path_t */

	struct dsp_queue_element_t element[MAX_DSP_MSG_NUM_IN_QUEUE];

	uint32_t size;
	uint32_t idx_r;
	uint32_t idx_w;

	uint32_t queue_counter;

	spinlock_t queue_lock;
	wait_queue_head_t queue_wq;

	/* dsp_send_msg_to_dsp() / dsp_process_msg_from_dsp() */
	int (*dsp_process_msg_func)(
		struct dsp_msg_queue_t *msg_queue,
		struct dsp_msg_t *p_dsp_msg);

	struct task_struct *dsp_thread_task;
	bool thread_enable;
	bool processing; /* Front slot remains owned until the sole worker returns. */

	bool init;
	bool enable;
};



/*
 * =============================================================================
 *                     private global members
 * =============================================================================
 */

/* TODO: CM4 */
static struct dsp_msg_queue_t g_dsp_msg_queue[NUM_OPENDSP_TYPE][DSP_NUM_PATH];

/* Caller and queued worker own independent references across timeout/reuse. */
struct dsp_tx_wait {
	struct kref refs;
	struct completion done;
	int result;
};

static void dsp_tx_wait_release(struct kref *refs)
{
	struct dsp_tx_wait *request = container_of(refs, struct dsp_tx_wait, refs);

	kfree(request);
}

static void dsp_tx_wait_finish(struct dsp_msg_t *msg, int result)
{
	struct dsp_tx_wait *request = msg->tx_wait;

	msg->tx_wait = NULL;
	if (!request)
		return;
	request->result = result;
	complete(&request->done);
	kref_put(&request->refs, dsp_tx_wait_release);
}



/*
 * =============================================================================
 *                     utilities
 * =============================================================================
 */

inline bool dsp_check_idx_msg_valid(
	const struct dsp_msg_queue_t *msg_queue,
	const uint32_t idx_msg);

inline bool dsp_check_queue_empty(
	const struct dsp_msg_queue_t *msg_queue);

inline bool dsp_check_queue_to_be_full(
	const struct dsp_msg_queue_t *msg_queue);

inline uint32_t dsp_get_num_messages_in_queue(
	const struct dsp_msg_queue_t *msg_queue);


/*
 * =============================================================================
 *                     private function declaration
 * =============================================================================
 */

static int dsp_push_msg(
	struct dsp_msg_queue_t *msg_queue,
	uint32_t ipi_id,
	void *buf,
	uint32_t len,
	void (*ipi_handler)(int ipi_id, void *buf, unsigned int len),
	bool wait_in_thread,
	uint32_t *p_idx_msg,
	uint32_t *p_queue_counter);

static int dsp_pop_msg(struct dsp_msg_queue_t *msg_queue);

static int dsp_front_msg(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t **pp_dsp_msg,
	uint32_t *p_idx_msg);


static int dsp_get_queue_element(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t **pp_dsp_msg,
	uint32_t *p_idx_msg);


static int dsp_init_single_msg_queue(
	struct dsp_msg_queue_t *msg_queue,
	const uint32_t dsp_id,
	const uint32_t dsp_path);

static int dsp_process_msg_thread(void *data);


static int dsp_send_msg_to_dsp(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t *p_dsp_msg);

static int dsp_process_msg_from_dsp(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t *p_dsp_msg);



/*
 * =============================================================================
 *                     utilities
 * =============================================================================
 */

inline bool dsp_check_idx_msg_valid(
	const struct dsp_msg_queue_t *msg_queue,
	const uint32_t idx_msg)
{
	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return false;
	}

	return (idx_msg < msg_queue->size) ? true : false;
}


inline bool dsp_check_queue_empty(const struct dsp_msg_queue_t *msg_queue)
{
	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return false;
	}

	return (READ_ONCE(msg_queue->idx_r) == READ_ONCE(msg_queue->idx_w));
}


inline bool dsp_check_queue_to_be_full(const struct dsp_msg_queue_t *msg_queue)
{
	uint32_t idx_w_to_be = 0;

	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return false;
	}

	idx_w_to_be = msg_queue->idx_w + 1;
	if (idx_w_to_be == msg_queue->size)
		idx_w_to_be = 0;

	return (idx_w_to_be == msg_queue->idx_r) ? true : false;
}


inline uint32_t dsp_get_num_messages_in_queue(
	const struct dsp_msg_queue_t *msg_queue)
{
	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return 0;
	}

	return (msg_queue->idx_w >= msg_queue->idx_r) ?
	       (msg_queue->idx_w - msg_queue->idx_r) :
	       ((msg_queue->size - msg_queue->idx_r) + msg_queue->idx_w);
}


#define DUMP_IPC_MSG(description, p_dsp_msg) \
	do { \
		struct ipi_msg_t *p_ipi_msg = NULL; \
		if (description == NULL || (p_dsp_msg) == NULL) \
			break; \
		if (p_dsp_msg->len >= IPI_MSG_HEADER_SIZE) { \
			p_ipi_msg = (struct ipi_msg_t *)p_dsp_msg->buf; \
			if (p_ipi_msg->magic == IPI_MSG_MAGIC_NUMBER) \
				DUMP_IPI_MSG(description, p_ipi_msg); \
		} else { \
			pr_info("%s, ipi_id: %u, buf %p, len: %u", \
				description, \
				p_dsp_msg->ipi_id, \
				p_dsp_msg->buf, \
				p_dsp_msg->len); \
		} \
	} while (0)



/*
 * =============================================================================
 *                     create/destroy/init/deinit functions
 * =============================================================================
 */


int ipi_queue_init_by_dsp(uint32_t dsp_id)
{
	struct dsp_msg_queue_t *msg_queue = NULL;

	uint32_t dsp_path = 0;

	int ret = 0;

	if (dsp_id >= NUM_OPENDSP_TYPE) {
		pr_info("dsp_id: %u error!!", dsp_id);
		return -EFAULT;
	}


	for (dsp_path = 0; dsp_path < DSP_NUM_PATH; dsp_path++) {
		msg_queue = &g_dsp_msg_queue[dsp_id][dsp_path];
		ret = dsp_init_single_msg_queue(msg_queue, dsp_id, dsp_path);
		if (ret != 0)
			return ret;
	}

	return ret;
}

static int dsp_dispatch_owned_handler(uint32_t dsp_id, uint32_t ipi_id,
	void *buf, uint32_t len,
	void (*ipi_handler)(int, void *, unsigned int), struct adsp_ipi_handler *entry);

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
int dsp_send_msg_to_queue_wrap(
	uint32_t core_id, /* enum adsp_core_id */
	uint32_t ipi_id,  /* enum adsp_ipi_id */
	void *buf,
	uint32_t len,
	uint32_t wait_ms)
{
	uint32_t dsp_id = adsp_cid_to_ipi_dsp_id(core_id);

	if (dsp_id >= NUM_OPENDSP_TYPE)
		return -1;

	return dsp_send_msg_to_queue(dsp_id, ipi_id, buf, len, wait_ms);
}

int dsp_dispatch_ipi_hanlder_to_queue_wrap(
	uint32_t core_id, /* enum adsp_core_id */
	uint32_t ipi_id,
	void *buf,
	uint32_t len,
	void (*ipi_handler)(int ipi_id, void *buf, unsigned int len))
{
	uint32_t dsp_id = adsp_cid_to_ipi_dsp_id(core_id);
	struct adsp_ipi_handler *entry;
	int ret;

	if (dsp_id >= NUM_OPENDSP_TYPE)
		return -1;

	entry = adsp_ipi_handler_get(ipi_id, ipi_handler);
	if (!entry)
		return -ENOENT;
	ret = dsp_dispatch_owned_handler(dsp_id, ipi_id, buf, len, ipi_handler, entry);
	if (ret)
		adsp_ipi_handler_put(entry);
	return ret;
}
#endif

static DEFINE_MUTEX(dsp_queue_lifecycle_lock);
static bool dsp_queues_started;

static int dsp_queue_mutation_check(void)
{
	unsigned int dsp, path;

	if (in_interrupt() || in_atomic() || irqs_disabled())
		return -EWOULDBLOCK;
	/* Check before the lifecycle mutex: another stopper may await this worker. */
	for (dsp = 0; dsp < NUM_OPENDSP_TYPE; dsp++)
		for (path = 0; path < DSP_NUM_PATH; path++)
			if (READ_ONCE(g_dsp_msg_queue[dsp][path].dsp_thread_task) == current)
				return -EDEADLK;
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	return adsp_ipi_mutation_check();
#else
	return 0;
#endif
}

/* All admissions close before any worker is stopped. Caller holds lifecycle lock. */
static void dsp_stop_queues(void)
{
	unsigned int dsp, path;
	unsigned long flags;

	for (dsp = 0; dsp < NUM_OPENDSP_TYPE; dsp++) {
		for (path = 0; path < DSP_NUM_PATH; path++) {
			struct dsp_msg_queue_t *queue = &g_dsp_msg_queue[dsp][path];

			if (!queue->init)
				continue;
			spin_lock_irqsave(&queue->queue_lock, flags);
			WRITE_ONCE(queue->enable, false);
			WRITE_ONCE(queue->thread_enable, false);
			spin_unlock_irqrestore(&queue->queue_lock, flags);
			wake_up_interruptible(&queue->queue_wq);
		}
	}
	for (dsp = 0; dsp < NUM_OPENDSP_TYPE; dsp++) {
		for (path = 0; path < DSP_NUM_PATH; path++) {
			struct dsp_msg_queue_t *queue = &g_dsp_msg_queue[dsp][path];

			if (!queue->init)
				continue;
			kthread_stop(queue->dsp_thread_task);
			/* No worker can retain a slot now. Drop each pending lease once. */
			spin_lock_irqsave(&queue->queue_lock, flags);
			while (!dsp_check_queue_empty(queue)) {
				struct dsp_msg_t *msg = &queue->element[queue->idx_r].msg;

				dsp_tx_wait_finish(msg, -ESHUTDOWN);
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
				adsp_ipi_handler_put(msg->adsp_handler);
				msg->adsp_handler = NULL;
#endif
				dsp_pop_msg(queue);
			}
			queue->processing = false;
			WRITE_ONCE(queue->dsp_thread_task, NULL);
			WRITE_ONCE(queue->init, false);
			spin_unlock_irqrestore(&queue->queue_lock, flags);
		}
	}
}

int dsp_ipi_queue_start(void)
{
	unsigned int dsp;
	int ret = dsp_queue_mutation_check();

	if (ret)
		return ret;
	mutex_lock(&dsp_queue_lifecycle_lock);
	if (dsp_queues_started) {
		mutex_unlock(&dsp_queue_lifecycle_lock);
		return 0;
	}
	for (dsp = 0; dsp < NUM_OPENDSP_TYPE; dsp++) {
		if (!is_audio_dsp_support(dsp))
			continue;
		ret = ipi_queue_init_by_dsp(dsp);
		if (ret)
			goto fail;
	}
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	hook_ipi_queue_send_msg_handler(dsp_send_msg_to_queue_wrap);
	hook_ipi_queue_recv_msg_hanlder(dsp_dispatch_ipi_hanlder_to_queue_wrap);
#endif
	dsp_queues_started = true;
	mutex_unlock(&dsp_queue_lifecycle_lock);
	return 0;
fail:
	dsp_stop_queues();
	mutex_unlock(&dsp_queue_lifecycle_lock);
	return ret;
}

int dsp_ipi_queue_stop(void)
{
	int ret = dsp_queue_mutation_check();

	if (ret)
		return ret;
	mutex_lock(&dsp_queue_lifecycle_lock);
#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	/* Workers remain live until every borrowed hook invocation returns. */
	unhook_ipi_queue_recv_msg_hanlder();
	unhook_ipi_queue_send_msg_handler();
#endif
	dsp_stop_queues();
	dsp_queues_started = false;
	mutex_unlock(&dsp_queue_lifecycle_lock);
	return 0;
}

void ipi_queue_init(void)
{
	WARN_ON(dsp_ipi_queue_start());
}

int dsp_flush_msg_queue(uint32_t dsp_id)
{
	struct dsp_msg_queue_t *queue;
	uint32_t cursor, keep;
	unsigned long flags;

	if (dsp_id >= NUM_OPENDSP_TYPE)
		return -EOVERFLOW;
	queue = &g_dsp_msg_queue[dsp_id][DSP_PATH_A2D];
	if (!READ_ONCE(queue->init))
		return -ENODEV;
	spin_lock_irqsave(&queue->queue_lock, flags);
	keep = queue->idx_r;
	if (queue->processing) {
		keep++;
		if (keep == queue->size)
			keep = 0;
	}
	/* Never pop/recycle the front while the worker is using its payload. */
	for (cursor = keep; cursor != queue->idx_w; ) {
		dsp_tx_wait_finish(&queue->element[cursor].msg, -ECANCELED);
		if (++cursor == queue->size)
			cursor = 0;
	}
	queue->idx_w = keep;
	spin_unlock_irqrestore(&queue->queue_lock, flags);
	return 0;
}


/*
 * =============================================================================
 *                     public functions
 * =============================================================================
 */

int dsp_send_msg_to_queue(uint32_t dsp_id, uint32_t ipi_id, void *buf,
	uint32_t len, uint32_t wait_ms)
{
	struct dsp_msg_queue_t *queue;
	struct dsp_tx_wait *request = NULL;
	uint32_t index = 0, counter = 0;
	unsigned long flags;
	long waited;
	int ret;

	if (dsp_id >= NUM_OPENDSP_TYPE)
		return -EOVERFLOW;
	if (!buf || len > DSP_MSG_BUFFER_SIZE)
		return -EFAULT;
	if (!is_audio_dsp_ready(dsp_id))
		return -ENODEV;
	queue = &g_dsp_msg_queue[dsp_id][DSP_PATH_A2D];
	if (!READ_ONCE(queue->enable))
		return -ESHUTDOWN;
	/* IRQ producers retain the existing asynchronous-send behavior. */
	if (in_interrupt())
		wait_ms = 0;
	if (wait_ms) {
		if (in_atomic() || irqs_disabled())
			return -EWOULDBLOCK;
		request = kzalloc(sizeof(*request), GFP_KERNEL);
		if (!request)
			return -ENOMEM;
		kref_init(&request->refs);
		init_completion(&request->done);
	}
	spin_lock_irqsave(&queue->queue_lock, flags);
	ret = dsp_push_msg(queue, ipi_id, buf, len, NULL, false, &index, &counter);
	if (!ret && request) {
		kref_get(&request->refs);
		queue->element[index].msg.tx_wait = request;
	}
	spin_unlock_irqrestore(&queue->queue_lock, flags);
	if (ret)
		goto put_caller;
	wake_up_interruptible(&queue->queue_wq);
	if (!request)
		return 0;
	waited = wait_for_completion_interruptible_timeout(&request->done,
		msecs_to_jiffies(wait_ms));
	ret = waited > 0 ? request->result : waited < 0 ? (int)waited : -ETIMEDOUT;
put_caller:
	if (request)
		kref_put(&request->refs, dsp_tx_wait_release);
	return ret;
}
EXPORT_SYMBOL(dsp_send_msg_to_queue);

static int dsp_dispatch_owned_handler(
	uint32_t dsp_id, /* enum dsp_id_t */
	uint32_t ipi_id,
	void *buf,
	uint32_t len,
	void (*ipi_handler)(int ipi_id, void *buf, unsigned int len),
	struct adsp_ipi_handler *entry)
{
	struct dsp_msg_queue_t *msg_queue = NULL;

	uint32_t idx_msg = 0;
	uint32_t queue_counter = 0;

	int retval = 0;
	unsigned long flags = 0;

	if (dsp_id >= NUM_OPENDSP_TYPE)
		return -EOVERFLOW;
	if (buf == NULL || len > DSP_MSG_BUFFER_SIZE)
		return -EFAULT;
	if (ipi_handler == NULL)
		return -EFAULT;

	msg_queue = &g_dsp_msg_queue[dsp_id][DSP_PATH_D2A];
	if (msg_queue->enable == false)
		return -1;


	/* push message to queue */
	spin_lock_irqsave(&msg_queue->queue_lock, flags);
	retval = dsp_push_msg(msg_queue,
			      ipi_id,
			      buf,
			      len,
			      ipi_handler,
			      false,
			      &idx_msg,
			      &queue_counter);
	if (!retval)
		msg_queue->element[idx_msg].msg.adsp_handler = entry;
	spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
	if (retval != 0)
		return retval;

	/* notify queue thread to process it */
	dsb(SY);
	wake_up_interruptible(&msg_queue->queue_wq);

	return 0;
}
int dsp_dispatch_ipi_hanlder_to_queue(uint32_t dsp_id, uint32_t ipi_id,
	void *buf, uint32_t len,
	void (*ipi_handler)(int, void *, unsigned int))
{
	return dsp_dispatch_owned_handler(dsp_id, ipi_id, buf, len, ipi_handler, NULL);
}
EXPORT_SYMBOL(dsp_dispatch_ipi_hanlder_to_queue);


/*
 * =============================================================================
 *                     private implementation
 * =============================================================================
 */

static int dsp_push_msg(
	struct dsp_msg_queue_t *msg_queue,
	uint32_t ipi_id,
	void *buf,
	uint32_t len,
	void (*ipi_handler)(int ipi_id, void *buf, unsigned int len),
	bool wait_in_thread,
	uint32_t *p_idx_msg,
	uint32_t *p_queue_counter)
{
	struct ipi_msg_t *p_ipi_msg = NULL;
	char dump_str[128] = {0};

	struct dsp_msg_t *p_dsp_msg = NULL;
	struct dsp_queue_element_t *p_element = NULL;

	unsigned long flags = 0;
	int n = 0;

	if (msg_queue == NULL || buf == NULL ||
	    p_idx_msg == NULL || p_queue_counter == NULL)
		return -EFAULT;

	if (!msg_queue->enable)
		return -ESHUTDOWN;

	/* check queue full */
	if (dsp_check_queue_to_be_full(msg_queue) == true) {
		n = snprintf(dump_str, sizeof(dump_str),
			     "dsp_id: %u, ipi_id: %u, queue overflow, idx_r: %u, idx_w: %u, drop it",
			     msg_queue->dsp_id,
			     ipi_id, msg_queue->idx_r,
			     msg_queue->idx_w);
		if (n < 0 || n > sizeof(dump_str))
			pr_info("error to get string dump_str");

		p_ipi_msg = (struct ipi_msg_t *)buf;
		if (len >= IPI_MSG_HEADER_SIZE &&
		    p_ipi_msg->magic == IPI_MSG_MAGIC_NUMBER) {
			if (p_ipi_msg->ack_type == AUDIO_IPI_MSG_NEED_ACK ||
			    p_ipi_msg->ack_type == AUDIO_IPI_MSG_ACK_BACK) {
				DUMP_IPI_MSG(dump_str, p_ipi_msg);
			}
		} else {
			pr_info("%s", dump_str);
		}

		/* Success transfers ownership only when a slot was actually filled. */
		return -EOVERFLOW;
	}

	if (dsp_check_idx_msg_valid(msg_queue, msg_queue->idx_w) == false)
		return -1;


	/* push */
	*p_idx_msg = msg_queue->idx_w;
	msg_queue->idx_w++;
	if (msg_queue->idx_w == msg_queue->size)
		msg_queue->idx_w = 0;

	/* copy */
	p_element = &msg_queue->element[*p_idx_msg];
	spin_lock_irqsave(&p_element->element_lock, flags);

	p_dsp_msg = &p_element->msg;
	p_dsp_msg->ipi_id = ipi_id;
	memcpy((void *)p_dsp_msg->buf, buf, len);
	p_dsp_msg->len = len;
	p_dsp_msg->ipi_handler = ipi_handler;
	p_dsp_msg->adsp_handler = NULL;
	p_dsp_msg->tx_wait = NULL;

	p_element->wait_in_thread = wait_in_thread;
	p_element->signal_arrival = false;
	p_element->send_retval = 0;
	p_element->queue_counter = msg_queue->queue_counter;

	*p_queue_counter = msg_queue->queue_counter;
	if (msg_queue->queue_counter == 0xFFFFFFFF)
		msg_queue->queue_counter = 0;
	else
		msg_queue->queue_counter++;
	spin_unlock_irqrestore(&p_element->element_lock, flags);

	return 0;
}


static int dsp_pop_msg(struct dsp_msg_queue_t *msg_queue)
{
	struct dsp_msg_t *p_dsp_msg = NULL;

	if (msg_queue == NULL) {
		pr_info("NULL!! msg_queue: %p", msg_queue);
		return -EFAULT;
	}

	/* check queue empty */
	if (dsp_check_queue_empty(msg_queue) == true) {
		pr_info("dsp_id: %u, queue is empty, idx_r: %u, idx_w: %u",
			msg_queue->dsp_id,
			msg_queue->idx_r,
			msg_queue->idx_w);
		return -1;
	}

	/* pop */
	p_dsp_msg = &msg_queue->element[msg_queue->idx_r].msg;
	msg_queue->idx_r++;
	if (msg_queue->idx_r == msg_queue->size)
		msg_queue->idx_r = 0;


	ipi_dbg("dsp_id: %u, dsp_path: %u, ipi_id: %u, idx_r: %u, idx_w: %u, queue(%u/%u)",
		msg_queue->dsp_id,
		msg_queue->dsp_path,
		p_dsp_msg->ipi_id,
		msg_queue->idx_r,
		msg_queue->idx_w,
		dsp_get_num_messages_in_queue(msg_queue),
		msg_queue->size);

	return 0;
}


static int dsp_front_msg(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t **pp_dsp_msg,
	uint32_t *p_idx_msg)
{
	if (msg_queue == NULL || pp_dsp_msg == NULL || p_idx_msg == NULL) {
		pr_info("NULL!! msg_queue: %p, pp_dsp_msg: %p, p_idx_msg: %p",
			msg_queue, pp_dsp_msg, p_idx_msg);
		return -EFAULT;
	}

	*pp_dsp_msg = NULL;
	*p_idx_msg = 0xFFFFFFFF;

	/* check queue empty */
	if (dsp_check_queue_empty(msg_queue) == true) {
		pr_info("dsp_id: %u, queue empty, idx_r: %u, idx_w: %u",
			msg_queue->dsp_id,
			msg_queue->idx_r, msg_queue->idx_w);
		return -ENOMEM;
	}

	/* front */
	if (dsp_check_idx_msg_valid(msg_queue, msg_queue->idx_r) == false) {
		pr_info("idx_r %u is invalid!! return",
			msg_queue->idx_r);
		return -1;
	}
	*p_idx_msg = msg_queue->idx_r;
	*pp_dsp_msg = &msg_queue->element[*p_idx_msg].msg;

	ipi_dbg("%s(), dsp_id: %u, dsp_path: %u, ipi_id: %u, idx_r: %u, idx_w: %u, queue(%u/%u), *p_idx_msg: %u",
		__func__,
		msg_queue->dsp_id,
		msg_queue->dsp_path,
		msg_queue->element[*p_idx_msg].msg.ipi_id,
		msg_queue->idx_r,
		msg_queue->idx_w,
		dsp_get_num_messages_in_queue(msg_queue),
		msg_queue->size,
		*p_idx_msg);

	return 0;
}


static int dsp_get_queue_element(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t **pp_dsp_msg,
	uint32_t *p_idx_msg)
{
	bool is_empty = true;

	unsigned long flags = 0;
	int retval = 0;

	uint32_t try_cnt = 0;
	const uint32_t k_max_try_cnt = 300; /* retry 3 sec for -ERESTARTSYS */
	const uint32_t k_restart_sleep_min_us = 10 * 1000; /* 10 ms */
	const uint32_t k_restart_sleep_max_us = (k_restart_sleep_min_us + 200);

	spin_lock_irqsave(&msg_queue->queue_lock, flags);
	is_empty = dsp_check_queue_empty(msg_queue);
	spin_unlock_irqrestore(&msg_queue->queue_lock, flags);

	/* wait until message is pushed to queue */
	if (is_empty == true) {
		for (try_cnt = 0; try_cnt < k_max_try_cnt; try_cnt++) {
			retval = wait_event_interruptible(
					 msg_queue->queue_wq,
					 (!dsp_check_queue_empty(msg_queue) ||
					  !READ_ONCE(msg_queue->thread_enable)));
			if (!READ_ONCE(msg_queue->thread_enable)) {
				pr_info("thread disable");
				retval = -1;
				break;
			}
			if (dsp_check_queue_empty(msg_queue) == false) {
				retval = 0;
				break;
			}
			if (retval == 0) { /* got msg in queue */
				pr_notice("wait ret 0, empty %d, enable %d",
					  dsp_check_queue_empty(msg_queue),
					  msg_queue->enable);
				break;
			}
			if (retval == -ERESTARTSYS) {
				pr_info("-ERESTARTSYS, #%u, sleep us: %u",
					try_cnt, k_restart_sleep_min_us);
				retval = -EINTR;
				usleep_range(k_restart_sleep_min_us,
					     k_restart_sleep_max_us);
				continue;
			}
			pr_notice("retval: %d not handle!!", retval);
		}
	}

	if (dsp_check_queue_empty(msg_queue) == false) {
		spin_lock_irqsave(&msg_queue->queue_lock, flags);
		retval = dsp_front_msg(msg_queue, pp_dsp_msg, p_idx_msg);
		if (!retval)
			msg_queue->processing = true;
		spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
	}

	return retval;
}


static int dsp_init_single_msg_queue(
	struct dsp_msg_queue_t *msg_queue,
	const uint32_t dsp_id,
	const uint32_t dsp_path)
{
	struct dsp_queue_element_t *p_element = NULL;
	struct dsp_msg_t *p_dsp_msg = NULL;

	char thread_name[32] = {0};

	int i = 0, n = 0;

	if (msg_queue == NULL) {
		pr_info("NULL!! msg_queue: %p", msg_queue);
		return -EFAULT;
	}
	if (dsp_id >= NUM_OPENDSP_TYPE || dsp_path >= DSP_NUM_PATH) {
		pr_info("dsp_id: %u, dsp_path: %u error!!",
			dsp_id, dsp_path);
		return -EFAULT;
	}


	/* check double init */
	if (msg_queue->init) {
		pr_info("dsp_id: %u already init!!", dsp_id);
		return 0;
	}

	/* init var */
	msg_queue->dsp_id = dsp_id;
	msg_queue->dsp_path = dsp_path;

	for (i = 0; i < MAX_DSP_MSG_NUM_IN_QUEUE; i++) {
		p_element = &msg_queue->element[i];

		p_dsp_msg = &p_element->msg;
		p_dsp_msg->ipi_id = 0;
		memset((void *)p_dsp_msg->buf, 0, DSP_MSG_BUFFER_SIZE);
		p_dsp_msg->len = 0;
		p_dsp_msg->ipi_handler = NULL;
		p_dsp_msg->adsp_handler = NULL;
		p_dsp_msg->tx_wait = NULL;

		spin_lock_init(&p_element->element_lock);
		init_waitqueue_head(&p_element->element_wq);

		p_element->wait_in_thread = false;
		p_element->signal_arrival = false;
		p_element->send_retval = 0;
		p_element->queue_counter = 0;
	}

	msg_queue->size = MAX_DSP_MSG_NUM_IN_QUEUE;
	msg_queue->idx_r = 0;
	msg_queue->idx_w = 0;
	msg_queue->processing = false;

	msg_queue->queue_counter = 0;

	spin_lock_init(&msg_queue->queue_lock);
	init_waitqueue_head(&msg_queue->queue_wq);


	if (dsp_path == DSP_PATH_A2D) {
		msg_queue->dsp_process_msg_func = dsp_send_msg_to_dsp;
		n = snprintf(thread_name,
			     sizeof(thread_name),
			     "dsp_send_thread_id_%u", dsp_id);
		if (n < 0 || n > sizeof(thread_name))
			pr_info("error to get string thread_name");

	} else if (dsp_path == DSP_PATH_D2A) {
		msg_queue->dsp_process_msg_func = dsp_process_msg_from_dsp;
		n = snprintf(thread_name,
			     sizeof(thread_name),
			     "dsp_recv_thread_id_%u", dsp_id);
		if (n < 0 || n > sizeof(thread_name))
			pr_info("error to get string thread_name");
	} else
		WARN_ON(1);

	/* lunch thread */
	msg_queue->dsp_thread_task = kthread_create(
					     dsp_process_msg_thread,
					     msg_queue,
					     "%s",
					     thread_name);
	if (IS_ERR(msg_queue->dsp_thread_task)) {
		int ret = PTR_ERR(msg_queue->dsp_thread_task);

		pr_info("can not create %s kthread: %d", thread_name, ret);
		msg_queue->dsp_thread_task = NULL;
		msg_queue->thread_enable = false;
		msg_queue->enable = false;
		return ret;
	}

	/* Publish a usable queue before waking its sole consumer. */
	msg_queue->thread_enable = true;
	msg_queue->enable = true;
	msg_queue->init = true;
	dsb(SY);
	wake_up_process(msg_queue->dsp_thread_task);
	return 0;
}


static int dsp_process_msg_thread(void *data)
{
	struct dsp_msg_queue_t *msg_queue = (struct dsp_msg_queue_t *)data;
	struct dsp_queue_element_t *p_element = NULL;
	struct dsp_msg_t *p_dsp_msg = NULL;
	uint32_t idx_msg = 0;

	unsigned long flags = 0;
	int retval = 0;
	struct sched_param param = { 0 };

	if (msg_queue == NULL) {
		pr_info("msg_queue == NULL!! return");
		return -EFAULT;
	}

	//set_user_nice(current, -20); /* normal thread highest priority */
	/* set to RT Thread */
	param.sched_priority = 4;
	retval = sched_setscheduler(current, SCHED_FIFO, &param);
	if (retval != 0) {
		pr_err("sched set RT thread failed");
	}
	while (READ_ONCE(msg_queue->thread_enable) && !kthread_should_stop()) {
		/* wait until element pushed */
		retval = dsp_get_queue_element(msg_queue, &p_dsp_msg, &idx_msg);
		if (retval != 0) {
			pr_info("dsp_get_queue_element retval %d", retval);
			continue;
		}
		if (idx_msg >= MAX_DSP_MSG_NUM_IN_QUEUE) {
			pr_info("dsp_get_queue_element idx_msg %u > %u",
				idx_msg, MAX_DSP_MSG_NUM_IN_QUEUE);
			spin_lock_irqsave(&msg_queue->queue_lock, flags);
			dsp_pop_msg(msg_queue);
			spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
			continue;
		}
		p_element = &msg_queue->element[idx_msg];

		/* send to dsp */
		retval = msg_queue->dsp_process_msg_func(msg_queue, p_dsp_msg);

		dsp_tx_wait_finish(&p_element->msg, retval);


		/* pop message from queue */
		spin_lock_irqsave(&msg_queue->queue_lock, flags);
		dsp_pop_msg(msg_queue);
		msg_queue->processing = false;
		spin_unlock_irqrestore(&msg_queue->queue_lock, flags);
	}


	pr_info("thread exit, dsp_id %u, dsp_path %u, idx_r %d, idx_w %d",
		msg_queue->dsp_id, msg_queue->dsp_path,
		msg_queue->idx_r, msg_queue->idx_w);

	return 0;
}

static int audio_ipi_send_msg_to_adsp(const struct dsp_msg_t *p_dsp_msg,
				      const uint32_t dsp_id)
{
#if !IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	DUMP_IPC_MSG("DSP not support!!", p_dsp_msg);
	return -ENODEV;
#else
	uint32_t core_id = 0xFFFFFFFF;
	int ret = -1;

	if (p_dsp_msg == NULL)
		return -EFAULT;

	core_id = ipi_dsp_id_to_adsp_cid(dsp_id);
	if (core_id == 0xFFFFFFFF)
		return -ENODEV;

	ret = adsp_send_message(
		      (enum adsp_ipi_id)p_dsp_msg->ipi_id,
		      (void *)p_dsp_msg->buf,
		      p_dsp_msg->len,
		      1, /* wait until sent or timeout */
		      (enum adsp_core_id)core_id);

	if (ret == ADSP_IPI_DONE)
		ret = 0;
	else if (ret == ADSP_IPI_BUSY)
		ret = -EBUSY;
	else if (ret == ADSP_IPI_ERROR)
		ret = -ECOMM;

#ifdef DEBUG_IPI
	if (ret == 0)
		DUMP_IPC_MSG("send pass", p_dsp_msg);
#endif

	return ret;
#endif
}

static int audio_ipi_send_msg_to_scp(struct dsp_msg_t *p_dsp_msg,
				     const uint32_t dsp_id)
{
#if !IS_ENABLED(CONFIG_MTK_AUDIO_CM4_SUPPORT)
	DUMP_IPC_MSG("DSP not support!!", p_dsp_msg);
	return -ENODEV;
#else
	uint32_t core_id = 0xFFFFFFFF;
	int ret = -1;

	if (p_dsp_msg == NULL)
		return -EFAULT;

	core_id = ipi_dsp_id_to_scp_cid(dsp_id);
	if (core_id == 0xFFFFFFFF)
		return -ENODEV;

	ret = scp_ipi_send(
		      (enum ipi_id)p_dsp_msg->ipi_id,
		      (void *)p_dsp_msg->buf,
		      p_dsp_msg->len,
		      0, /* avoid busy waiting */
		      (enum scp_core_id)core_id);

	if (ret == SCP_IPI_DONE)
		ret = 0;
	else if (ret == SCP_IPI_BUSY)
		ret = -EBUSY;
	else if (ret == SCP_IPI_ERROR)
		ret = -ECOMM;

#ifdef DEBUG_IPI
	if (ret == 0)
		DUMP_IPC_MSG("send pass", p_dsp_msg);
#endif

	return ret;
#endif
}

static int dsp_send_msg_to_dsp(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t *p_dsp_msg)
{
	uint32_t dsp_id = 0xFFFFFFFF;
	uint32_t audio_ipi_id = 0xFFFFFFFF;
	struct ipi_msg_t *p_ipi_msg = NULL;

	static bool start_flag[NUM_OPENDSP_TYPE] = {0};
	bool retry_flag = false;
	char dump_str[64] = {0};

	uint32_t try_cnt = 0;
	const uint32_t k_max_try_cnt = 1000 * 20; /* 1 sec for write ipc */
	const uint32_t k_sleep_min_us = 50;
	const uint32_t k_sleep_max_us = (k_sleep_min_us + 10);

	int ret = 0;
	int n = 0;

	if (msg_queue == NULL || p_dsp_msg == NULL) {
		pr_info("NULL!! msg_queue: %p, p_dsp_msg: %p",
			msg_queue, p_dsp_msg);
		return -EFAULT;
	}

	dsp_id = msg_queue->dsp_id;
	if (!is_audio_dsp_support(dsp_id))
		return -ENODEV;

	audio_ipi_id = audio_get_audio_ipi_id_by_dsp(dsp_id);

	/* IPC */
	for (try_cnt = 0; try_cnt < k_max_try_cnt; try_cnt++) {
		if (is_audio_use_scp(dsp_id))
			ret = audio_ipi_send_msg_to_scp(p_dsp_msg, dsp_id);
		else if (is_audio_use_adsp(dsp_id))
			ret = audio_ipi_send_msg_to_adsp(p_dsp_msg, dsp_id);
		else {
			pr_notice("dsp_id %u not support!!", dsp_id);
			ret = -ENODEV;
			break;
		}

		if (ret == 0) { /* pass */
			start_flag[dsp_id] = true;
			if (p_dsp_msg->ipi_id == audio_ipi_id) {
				p_ipi_msg = (struct ipi_msg_t *)p_dsp_msg->buf;
				if (check_print_msg_info(p_ipi_msg) &&
				    !retry_flag)
					DUMP_IPI_MSG("pass", p_ipi_msg);
			}
			ret = 0;
			break;
		}
		if (start_flag[dsp_id] == false) {
			if (try_cnt == 0) /* only dump log once */
				DUMP_IPC_MSG("booting..., retry", p_dsp_msg);
			retry_flag = true;
			ret = -ENODEV;
			usleep_range(k_sleep_min_us, k_sleep_max_us);
			continue;
		}
		if (is_audio_dsp_ready(dsp_id) == false) {
			n = snprintf(dump_str, sizeof(dump_str),
				     "dsp_id %u dead!!", dsp_id);
			if (n < 0 || n > sizeof(dump_str))
				pr_info("error to get string dump_str");
			DUMP_IPC_MSG(dump_str, p_dsp_msg);
			ret = 0;
			break;
		}
		if (ret == -ECOMM) { /* send fail */
			n = snprintf(dump_str, sizeof(dump_str),
				     "dsp_id %u error!!", dsp_id);
			if (n < 0 || n > sizeof(dump_str))
				pr_info("error to get string dump_str");
			DUMP_IPC_MSG(dump_str, p_dsp_msg);
			ret = -1;
			break;
		}
		if (ret == -EBUSY) { /* busy */
			if (try_cnt == 0) /* only dump log once */
				DUMP_IPC_MSG("IPC busy, retry", p_dsp_msg);
			retry_flag = true;
			ret = -ETIMEDOUT;
			usleep_range(k_sleep_min_us, k_sleep_max_us);
			continue;
		}
		pr_notice("ret: %d not handle!!", ret);
	}

	if (retry_flag == true) {
		if (ret == 0) {
			n = snprintf(dump_str, sizeof(dump_str),
				     "dsp_id %u retry %u pass", dsp_id, try_cnt);
			if (n < 0 || n > sizeof(dump_str))
				pr_info("error to get string dump_str");
		} else {
			n = snprintf(dump_str, sizeof(dump_str),
				     "dsp_id %u retry %u err ret %d",
				     dsp_id, try_cnt, ret);
			if (n < 0 || n > sizeof(dump_str))
				pr_info("error to get string dump_str");
		}
		DUMP_IPC_MSG(dump_str, p_dsp_msg);
	}

	return ret;
}


static int dsp_process_msg_from_dsp(
	struct dsp_msg_queue_t *msg_queue,
	struct dsp_msg_t *p_dsp_msg)
{
	if (msg_queue == NULL || p_dsp_msg == NULL) {
		pr_info("NULL!! msg_queue: %p, p_dsp_msg: %p",
			msg_queue, p_dsp_msg);
		return -EFAULT;
	}

#if IS_ENABLED(CONFIG_MTK_AUDIODSP_SUPPORT)
	if (p_dsp_msg->adsp_handler) {
		struct adsp_ipi_handler *entry = p_dsp_msg->adsp_handler;

		p_dsp_msg->adsp_handler = NULL;
		if (p_dsp_msg->len)
			adsp_ipi_handler_invoke(entry, p_dsp_msg->ipi_id,
				p_dsp_msg->buf, p_dsp_msg->len);
		adsp_ipi_handler_put(entry);
		return p_dsp_msg->len ? 0 : -EFAULT;
	}
#endif
	if (p_dsp_msg->ipi_handler == NULL) {
		pr_info("NULL!! p_dsp_msg->ipi_handler: %p",
			p_dsp_msg->ipi_handler);
		return -EFAULT;
	}

	if (p_dsp_msg->len == 0) {
		pr_info("p_dsp_msg->len: %u", p_dsp_msg->len);
		return -EFAULT;
	}

	p_dsp_msg->ipi_handler(
		p_dsp_msg->ipi_id,
		p_dsp_msg->buf,
		p_dsp_msg->len);

	return 0;
}


