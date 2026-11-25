// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/slab.h>
#include <linux/srcu.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include "conap_scp.h"
#include "conap_scp_status.h"
#include "conap_scp_priv.h"
#include "conap_platform_data.h"
#include "msg_thread.h"
#include "conap_scp_ipi.h"

#include "connectivity_build_in_adapter.h"

struct conap_scp_drv_user g_drv_user[CONAP_SCP_DRV_NUM];
struct conap_scp_core_ctx {
	unsigned int enable;
	unsigned int chip_info;
	phys_addr_t emi_phy_addr;
	unsigned int state; /* stop: 0, ready : 1 */
	struct mutex lock;
	struct msg_thread_ctx tx_msg_thread;

	struct completion msg_send_comp;
	struct completion msg_recv_comp;
};

static struct conap_scp_core_ctx g_core_ctx;
static int native_init_result=-EINPROGRESS;
module_param(native_init_result,int,0444);
module_param_named(connectivity_enabled,g_core_ctx.enable,uint,0444);
module_param_named(firmware_ready,g_core_ctx.state,uint,0444);
/* Static lock survives runtime-context teardown and gates TX publication. */
static DEFINE_MUTEX(g_tx_admission_lock);
static bool g_tx_accepting;

struct conap_scp_tx_request {
	struct kref refs;
	struct completion done;
	enum conap_scp_drv_type type;
	unsigned int msg_id;
	unsigned int size;
	int result;
	uint8_t data[];
};

static void conap_scp_tx_release(struct kref *ref)
{
	struct conap_scp_tx_request *request =
		container_of(ref, struct conap_scp_tx_request, refs);

	kfree(request);
}

/* Immutable registrations: callbacks may sleep, so retirement uses SRCU. */
struct conap_scp_client {
	struct conap_scp_drv_cb cb;
};
static DEFINE_MUTEX(g_client_lock);
DEFINE_STATIC_SRCU(g_client_srcu);
static struct conap_scp_client __rcu *g_clients[CONAP_SCP_DRV_NUM];
static bool g_clients_accepting;

static void conap_scp_notify_state(unsigned int type, int state)
{
	struct conap_scp_client *client;
	int index = srcu_read_lock(&g_client_srcu);

	client = srcu_dereference(g_clients[type], &g_client_srcu);
	if (client && client->cb.conap_scp_state_notify_cb)
		client->cb.conap_scp_state_notify_cb(state);
	srcu_read_unlock(&g_client_srcu, index);
}

static void conap_scp_notify_message(unsigned int type, unsigned int id,
				unsigned int *data, unsigned int size)
{
	struct conap_scp_client *client;
	int index = srcu_read_lock(&g_client_srcu);

	client = srcu_dereference(g_clients[type], &g_client_srcu);
	if (client && client->cb.conap_scp_msg_notify_cb)
		client->cb.conap_scp_msg_notify_cb(id, data, size);
	srcu_read_unlock(&g_client_srcu, index);
}

static void conap_scp_close_clients(void)
{
	struct conap_scp_client *retired[CONAP_SCP_DRV_NUM];
	unsigned int i;

	mutex_lock(&g_client_lock);
	g_clients_accepting = false;
	for (i = 0; i < CONAP_SCP_DRV_NUM; i++) {
		retired[i] = rcu_dereference_protected(g_clients[i],
				lockdep_is_held(&g_client_lock));
		rcu_assign_pointer(g_clients[i], NULL);
		WRITE_ONCE(g_drv_user[i].enable, 0);
	}
	mutex_unlock(&g_client_lock);
	synchronize_srcu(&g_client_srcu);
	for (i = 0; i < CONAP_SCP_DRV_NUM; i++)
		kfree(retired[i]);
}

static int opfunc_send_msg(struct msg_op_data *op);
static int opfunc_is_drv_ready(struct msg_op_data *op);
static int opfunc_is_drv_ready_ack(struct msg_op_data *op);
static int opfunc_scp_state_change(struct msg_op_data *op);
static int opfunc_recv_msg(struct msg_op_data *op);

static const msg_opid_func conap_scp_core_opfunc[] = {
	[CONAP_SCP_OPID_STATE_CHANGE] = opfunc_scp_state_change,
	[CONAP_SCP_OPID_SEND_MSG] = opfunc_send_msg,
	[CONAP_SCP_OPID_DRV_READY] = opfunc_is_drv_ready,
	[CONAP_SCP_OPID_DRV_READY_ACK] = opfunc_is_drv_ready_ack,
	[CONAP_SCP_OPID_RECV_MSG] = opfunc_recv_msg,
};

#define MAX_MSG_SZ		1024
uint8_t g_msg_buf[MAX_MSG_SZ] __aligned(sizeof(uint32_t));

uint16_t g_cur_msg_drv;
uint16_t g_cur_msg_id;
uint16_t g_cur_recv_sz;
uint8_t *g_cur_msg_recv_buf;
static uint16_t g_expected_recv_sz;
static DEFINE_SPINLOCK(g_recv_lock);

struct msg_data_2 {
	uint32_t param0;
	uint32_t param1;
};

/* connsys state notifier */
enum state_chg_drv_type {
	STATE_CHG_DRV_NONE = 0,
	STATE_CHG_DRV_CONN = 1,
	STATE_CHG_DRV_SCP = 2,
};

static int conn_state_event_handler(struct notifier_block *this,
			unsigned long event, void *ptr);
static struct notifier_block g_conn_state_notifier = {
	.notifier_call = conn_state_event_handler,
};


static int _send_msg(enum conap_scp_drv_type drv_type, uint16_t msg_id,
				uint8_t *msg_buf, uint32_t msg_sz)
{
	uint32_t sent_len = 0, sz = 0;
	int wait_ret;
	int ret;
	const unsigned int ipi_msg_data_sz = conap_scp_ipi_msg_sz();

	if (!ipi_msg_data_sz)
		return -EINVAL;
	if (!g_core_ctx.enable || !conap_scp_ipi_is_scp_ready()) {
		pr_warn("[%s] scp is not ready", __func__);
		return -1;
	}

	reinit_completion(&g_core_ctx.msg_send_comp);

	/* send tx request */
	ret = conap_scp_ipi_send_cmd(DRV_TYPE_CORE, CONAP_SCP_CORE_REQ_TX, drv_type, msg_sz);
	if (ret < 0) {
		pr_warn("[%s] ipi_send fail, rety", __func__);
		return -1;
	}

	/* wait for tx request accept */
	wait_ret = wait_for_completion_timeout(&g_core_ctx.msg_send_comp,
					msecs_to_jiffies(2000));
	if (wait_ret == 0) {
		pr_warn("[%s] send msg comp timeout", __func__);
		return CONN_TIMEOUT;
	}

	pr_info("[%s] >>>> type=[%d] id=[%d] sz=[%d] ipi_sz=[%d]", __func__,
			drv_type, msg_id, msg_sz, ipi_msg_data_sz);
	while (sent_len <= msg_sz) {
		sz = min(ipi_msg_data_sz, msg_sz - sent_len);

		ret = conap_scp_ipi_send_data(drv_type, msg_id, msg_sz,
				(msg_buf != NULL ? &(msg_buf[sent_len]) : NULL), sz);
		if (ret < 0)
			return ret;
		if ((ret == 0 && sz) || (unsigned int)ret > sz)
			return -EIO;
		sent_len += ret;
		if (sent_len >= msg_sz)
			break;
	}
	//pr_info("[_send_msg] send data done");
	return 0;
}

/*********************************************************************/
/* CORE OP FUNC */
/*********************************************************************/
static int opfunc_send_msg(struct msg_op_data *op)
{
	struct conap_scp_tx_request *request = (void *)op->op_data[0];
	int ret;

	ret = _send_msg(request->type, request->msg_id,
			request->data, request->size);
	request->result = ret;
	complete(&request->done);
	/* Timed-out callers may already have dropped their reference. */
	kref_put(&request->refs, conap_scp_tx_release);
	return ret;
}

static int opfunc_is_drv_ready(struct msg_op_data *op)
{
	int ret = 0;
	unsigned int drv_type = (unsigned int)op->op_data[0];

	pr_info("[%s] drv=[%d] ", __func__, drv_type);
	ret = conap_scp_ipi_send_cmd(DRV_TYPE_CORE, CONAP_SCP_CORE_DRV_QRY, drv_type, 0);

	if (ret) {
		pr_warn("[%s] fail [%d]", __func__, ret);
		return -2;
	}
	return 0;
}

static int opfunc_is_drv_ready_ack(struct msg_op_data *op)
{
	unsigned int drv_type = (unsigned int)op->op_data[0];
	int ret;

	ret = READ_ONCE(g_drv_user[drv_type].enable);
	pr_info("[%s] ack type=[%d] ret=[%d]", __func__, drv_type, ret);
	ret = conap_scp_ipi_send_cmd(DRV_TYPE_CORE, CONAP_SCP_CORE_DRV_QRY_ACK, drv_type, ret);
	if (ret)
		pr_warn("[%s] send ipi cmd fail [%d]", __func__, ret);
	return ret;
}


static int opfunc_scp_state_change(struct msg_op_data *op)
{
	unsigned int drv_type = (unsigned int)op->op_data[0];
	unsigned int cur_state = (unsigned int)op->op_data[1];
	int ret, i;

	if (drv_type > STATE_CHG_DRV_SCP)
		return -1;


	pr_info("[%s] type=[%d] state=[%d] conn=[%d] scp ready=[%d]", __func__,
				drv_type, cur_state, g_core_ctx.enable,
				conap_scp_ipi_is_scp_ready());
	if (g_core_ctx.enable == 0) {
		pr_info("[%s] conn not enable yet", __func__);
		return 0;
	}
	if (cur_state == 1 && conap_scp_ipi_is_scp_ready() == 0) {
		pr_info("[%s] scp not enable yet", __func__);
		return 0;
	}

	/* SCP ready */
	if (cur_state == 1) {
		pr_info("[%s] send init cmd", __func__);
		ret = conap_scp_ipi_send_cmd(DRV_TYPE_CORE, CONAP_SCP_CORE_INIT,
					connsys_scp_shm_get_addr(), connsys_scp_shm_get_size());
		native_init_result=ret;
		if (ret) {
			pr_info("[SCP_STATE_CHG] ipi handshake ret=[%d]", ret);
			return -1;
		}
	}

	g_core_ctx.state = cur_state;

	/* make sure intf is init done */
	msleep(100);
	for (i = 0; i < CONAP_SCP_DRV_NUM; i++)
		conap_scp_notify_state(i, cur_state);

	return 0;
}

static int opfunc_recv_msg(struct msg_op_data *op)
{
	uint16_t msg_drv, msg_sz;
	unsigned long flags;
	int ret, wait_ret;

	if (op->op_data[0] == DRV_TYPE_CORE ||
	    op->op_data[0] >= CONAP_SCP_DRV_NUM || op->op_data[1] > MAX_MSG_SZ)
		return -EINVAL;
	msg_drv = op->op_data[0];
	msg_sz = op->op_data[1];

	reinit_completion(&g_core_ctx.msg_recv_comp);
	spin_lock_irqsave(&g_recv_lock, flags);
	g_cur_msg_drv = msg_drv;
	g_cur_msg_id = 0;
	g_cur_recv_sz = 0;
	g_expected_recv_sz = msg_sz;
	g_cur_msg_recv_buf = &g_msg_buf[0];
	spin_unlock_irqrestore(&g_recv_lock, flags);

	ret = conap_scp_ipi_send_cmd(DRV_TYPE_CORE, CONAP_SCP_CORE_TX_ACCEP, msg_drv, 0);
	if (ret) {
		ret = CONN_TIMEOUT;
		goto out;
	}

	wait_ret = wait_for_completion_timeout(&g_core_ctx.msg_recv_comp,
		msecs_to_jiffies(2000));
	if (!wait_ret) {
		ret = CONN_TIMEOUT;
		goto out;
	}

	ret = 0;
	if (g_cur_recv_sz == msg_sz)
		conap_scp_notify_message(msg_drv, g_cur_msg_id,
			(unsigned int *)&g_msg_buf[0], msg_sz);

out:
	spin_lock_irqsave(&g_recv_lock, flags);
	g_cur_msg_drv = 0;
	g_cur_msg_id = 0;
	g_cur_recv_sz = 0;
	g_expected_recv_sz = 0;
	g_cur_msg_recv_buf = NULL;
	spin_unlock_irqrestore(&g_recv_lock, flags);
	return ret;
}

/*********************************************************************/
/* PRIVATE API */
/*********************************************************************/
static int _conap_scp_is_scp_ready(void)
{
	if (mutex_lock_killable(&g_core_ctx.lock))
		return -1;
	if (g_core_ctx.state == 0) {
		mutex_unlock(&g_core_ctx.lock);
		return 0;
	}
	mutex_unlock(&g_core_ctx.lock);
	return 1;
}

static void conap_scp_msg_notify(uint16_t drv_type, uint16_t msg_id,
			uint16_t total_sz, uint8_t *data, uint32_t this_sz)
{
	struct msg_data_2 args;
	unsigned long flags;
	int ret;

	if (drv_type >= CONAP_SCP_DRV_NUM || total_sz > MAX_MSG_SZ ||
	    this_sz > total_sz || this_sz > conap_scp_ipi_msg_sz() ||
	    (this_sz && !data))
		return;

	if (drv_type == DRV_TYPE_CORE) {
		if (msg_id == CONAP_SCP_CORE_ACK || msg_id == CONAP_SCP_CORE_TX_ACCEP) {
			complete(&g_core_ctx.msg_send_comp);
			return;
		}
		if (this_sz < sizeof(args))
			return;
		memcpy(&args, data, sizeof(args));
		if (args.param0 >= CONAP_SCP_DRV_NUM)
			return;
		if (msg_id == CONAP_SCP_CORE_DRV_QRY) {
			ret = msg_thread_send_1(&g_core_ctx.tx_msg_thread,
				CONAP_SCP_OPID_DRV_READY_ACK, args.param0);
			if (ret)
				pr_warn("[conap_msg_notify] readiness queue failed [%d]", ret);
		} else if (msg_id == CONAP_SCP_CORE_DRV_QRY_ACK) {
			g_drv_user[args.param0].is_rdy_ret = args.param1;
			complete(&g_drv_user[args.param0].is_rdy_comp);
		} else if (msg_id == CONAP_SCP_CORE_REQ_TX &&
			   args.param0 != DRV_TYPE_CORE && args.param1 <= MAX_MSG_SZ) {
			ret = msg_thread_send_2(&g_core_ctx.tx_msg_thread,
				CONAP_SCP_OPID_RECV_MSG, args.param0, args.param1);
			if (ret)
				pr_warn("[conap_msg_notify] receive queue failed [%d]", ret);
		}
		return;
	}

	spin_lock_irqsave(&g_recv_lock, flags);
	if (!g_cur_msg_recv_buf || drv_type != g_cur_msg_drv ||
	    total_sz != g_expected_recv_sz || g_cur_recv_sz > total_sz ||
	    this_sz > total_sz - g_cur_recv_sz ||
	    (g_cur_recv_sz && msg_id != g_cur_msg_id) ||
	    (!this_sz && total_sz))
		goto out;

	g_cur_msg_id = msg_id;
	if (this_sz)
		memcpy(&g_cur_msg_recv_buf[g_cur_recv_sz], data, this_sz);
	g_cur_recv_sz += this_sz;
	if (g_cur_recv_sz == total_sz) {
		g_cur_msg_recv_buf = NULL;
		complete(&g_core_ctx.msg_recv_comp);
	}
out:
	spin_unlock_irqrestore(&g_recv_lock, flags);
}

static void conap_scp_ipi_ctrl_notify(unsigned int state)
{
	int ret;

	if ((g_core_ctx.state == 1 && state == 1) ||
		(g_core_ctx.state == 0 && state == 0))
		return;

	pr_info("[%s] state=[%d]->[%d]", __func__, g_core_ctx.state, state);

	ret = msg_thread_send_2(&g_core_ctx.tx_msg_thread,
			CONAP_SCP_OPID_STATE_CHANGE, STATE_CHG_DRV_SCP, (size_t)state);
	if (ret)
		pr_warn("[%s] msg_send fail [%d]", __func__, ret);
}

int conn_state_event_handler(struct notifier_block *this,
			unsigned long event, void *ptr)
{
	struct connsys_state_info *info;
	int ret;

	info = (struct connsys_state_info *)ptr;

	if (event == 0) {
		g_core_ctx.enable = 0;
		g_core_ctx.state = 0;
		return 0;
	}
	if (event != 1 || !info)
		return 0;

	pr_info("[%s] ========= event =[%lu] [%x][%llu]",
				__func__, event, info->chip_info, info->emi_phy_addr);

	g_core_ctx.chip_info = info->chip_info;
	g_core_ctx.emi_phy_addr = info->emi_phy_addr;

	ret = connsys_scp_platform_data_init(info->chip_info, info->emi_phy_addr);
	/* check if platform support */
	pr_info("[%s] chip_info=[%x] addr[%llx] ret=[%d]", __func__,
						info->chip_info, info->emi_phy_addr, ret);

	if (ret) {
		pr_info("[%s] conap not support", __func__);
		return 0;
	}
	g_core_ctx.enable = 1;

	ret = msg_thread_send_2(&g_core_ctx.tx_msg_thread,
			CONAP_SCP_OPID_STATE_CHANGE, STATE_CHG_DRV_CONN, 1);
	if (ret)
		pr_warn("[%s] msg_send fail [%d]", __func__, ret);


	return 0;
}


/*********************************************************************/
/* OPEN API */
/*********************************************************************/
int conap_scp_send_message(enum conap_scp_drv_type type,
					unsigned int msg_id,
					unsigned char *buf, unsigned int size)
{
	struct conap_scp_tx_request *request;
	int ret;

	if (type <= DRV_TYPE_CORE || type >= CONAP_SCP_DRV_NUM ||
	    size > MAX_MSG_SZ || size % 4 || (size && !buf) || msg_id > 0xffff)
		return CONN_INVALID_ARGUMENT;

	mutex_lock(&g_tx_admission_lock);
	if (!g_tx_accepting || !READ_ONCE(g_core_ctx.enable)) {
		ret = CONN_CONAP_NOT_SUPPORT;
		goto unlock;
	}
	if (_conap_scp_is_scp_ready() != 1) {
		ret = CONN_NOT_READY;
		goto unlock;
	}
	request = kzalloc(struct_size(request, data, size), GFP_KERNEL);
	if (!request) {
		ret = -ENOMEM;
		goto unlock;
	}
	kref_init(&request->refs);
	init_completion(&request->done);
	request->type = type;
	request->msg_id = msg_id;
	request->size = size;
	if (size)
		memcpy(request->data, buf, size);

	/* Queue owns a reference before publication, independent of caller timeout. */
	kref_get(&request->refs);
	ret = msg_thread_send_1(&g_core_ctx.tx_msg_thread,
			CONAP_SCP_OPID_SEND_MSG, (size_t)request);
	if (ret)
		kref_put(&request->refs, conap_scp_tx_release);
	mutex_unlock(&g_tx_admission_lock);
	if (!ret) {
		if (!wait_for_completion_timeout(&request->done,
				msecs_to_jiffies(MSG_OP_TIMEOUT)))
			ret = CONN_TIMEOUT;
		else
			ret = request->result;
	}
	kref_put(&request->refs, conap_scp_tx_release);
	return ret;
unlock:
	mutex_unlock(&g_tx_admission_lock);
	return ret;
}
EXPORT_SYMBOL(conap_scp_send_message);

int conap_scp_is_drv_ready(enum conap_scp_drv_type type)
{
	int ret = 0, wait_ret = 1;
	struct conap_scp_drv_user *user = NULL;
	int is_ready = 0;

	if (_conap_scp_is_scp_ready() != 1)
		return CONN_NOT_READY;

	if (type >= CONAP_SCP_DRV_NUM)
		return CONN_INVALID_ARGUMENT;

	user = &g_drv_user[type];
	user->is_rdy_ret = 0;
	reinit_completion(&user->is_rdy_comp);

	/* send msg */
	ret = msg_thread_send_1(&g_core_ctx.tx_msg_thread, CONAP_SCP_OPID_DRV_READY, (size_t)type);
	if (ret) {
		pr_warn("[%s] send msg fail [%d]", __func__, ret);
		return ret;
	}

	wait_ret = wait_for_completion_timeout(&user->is_rdy_comp,
				msecs_to_jiffies(2000));

	if (wait_ret == 0) {
		pr_warn("[%s] timeout ", __func__);
		return CONN_TIMEOUT;
	}

	is_ready = user->is_rdy_ret;
	pr_info("[%s] is drv ready type=[%d] ret=[%d]", __func__, type, is_ready);

	return is_ready;
}
EXPORT_SYMBOL(conap_scp_is_drv_ready);

int conap_scp_register_drv(enum conap_scp_drv_type type, struct conap_scp_drv_cb *cb)
{
	struct conap_scp_client *client;
	int ret = 0;

	if (type < 0 || type >= CONAP_SCP_DRV_NUM || !cb)
		return -EINVAL;
	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;
	client->cb = *cb;

	mutex_lock(&g_client_lock);
	if (!g_clients_accepting || !READ_ONCE(g_core_ctx.enable))
		ret = CONN_CONAP_NOT_SUPPORT;
	else if (rcu_access_pointer(g_clients[type]))
		ret = -EEXIST;
	else {
		rcu_assign_pointer(g_clients[type], client);
		WRITE_ONCE(g_drv_user[type].enable, 1);
	}
	mutex_unlock(&g_client_lock);
	if (ret)
		kfree(client);
	return ret;
}
EXPORT_SYMBOL(conap_scp_register_drv);

int conap_scp_unregister_drv(enum conap_scp_drv_type type)
{
	struct conap_scp_client *client;

	if (type < 0 || type >= CONAP_SCP_DRV_NUM)
		return -EINVAL;
	/* Client callbacks run on this worker; synchronizing from one would wait
	 * on itself. Audited GPS callbacks do not unregister themselves.
	 */
	if (current == READ_ONCE(g_core_ctx.tx_msg_thread.pThread))
		return -EDEADLK;
	mutex_lock(&g_client_lock);
	client = rcu_dereference_protected(g_clients[type],
			lockdep_is_held(&g_client_lock));
	if (!client) {
		mutex_unlock(&g_client_lock);
		return -EEXIST;
	}
	rcu_assign_pointer(g_clients[type], NULL);
	WRITE_ONCE(g_drv_user[type].enable, 0);
	mutex_unlock(&g_client_lock);

	/* Retirement is allowed even when the SCP/connection has powered off. */
	synchronize_srcu(&g_client_srcu);
	kfree(client);
	return 0;
}
EXPORT_SYMBOL(conap_scp_unregister_drv);

/*********************************************************************/
/* init/deinit */
/*********************************************************************/
int conap_scp_is_ready(void)
{
	return conap_scp_ipi_is_scp_ready();
}
EXPORT_SYMBOL(conap_scp_is_ready);

int conap_scp_init(void)
{
	int ret, i;
	struct conap_scp_ipi_cb ipi_cb;

	memset(&g_core_ctx, 0, sizeof(struct conap_scp_core_ctx));

	/* init drv_user */
	for (i = 0; i < CONAP_SCP_DRV_NUM; i++) {
		memset(&g_drv_user[i], 0, sizeof(struct conap_scp_drv_user));
		init_completion(&(g_drv_user[i].is_rdy_comp));
	}

	/* tx msg thread */
	ret = msg_thread_init(&g_core_ctx.tx_msg_thread, "conap_scp_core",
					conap_scp_core_opfunc, CONAP_SCP_OPID_MAX);
	if (ret) {
		pr_warn("msg thread init fail [%d]", ret);
		return -1;
	}

	mutex_init(&g_core_ctx.lock);

	/* rx thread */
	init_completion(&g_core_ctx.msg_send_comp);
	init_completion(&g_core_ctx.msg_recv_comp);

	/* init ipi */
	ipi_cb.conap_scp_ipi_msg_notify = conap_scp_msg_notify;
	ipi_cb.conap_scp_ipi_ctrl_notify = conap_scp_ipi_ctrl_notify;
	ret = conap_scp_ipi_init(&ipi_cb);
	if (ret) {
		pr_warn("scp ipi init fail [%d]", ret);
		msg_thread_deinit(&g_core_ctx.tx_msg_thread);
		mutex_destroy(&g_core_ctx.lock);
		return ret;
	}

	mutex_lock(&g_tx_admission_lock);
	g_tx_accepting = true;
	mutex_unlock(&g_tx_admission_lock);
	mutex_lock(&g_client_lock);
	g_clients_accepting = true;
	mutex_unlock(&g_client_lock);
	connectivity_register_state_notifier(&g_conn_state_notifier);

	pr_info("[%s] DONE", __func__);
	return 0;
}

int conap_scp_deinit(void)
{
	/* No TX caller can touch the runtime mutex/queue after this gate closes. */
	mutex_lock(&g_tx_admission_lock);
	g_tx_accepting = false;
	mutex_unlock(&g_tx_admission_lock);
	conap_scp_close_clients();
	connectivity_unregister_state_notifier(&g_conn_state_notifier);
	g_core_ctx.enable = 0;
	g_core_ctx.state = 0;
	conap_scp_ipi_deinit();
	msg_thread_deinit(&g_core_ctx.tx_msg_thread);
	mutex_destroy(&g_core_ctx.lock);
	return 0;
}

