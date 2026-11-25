// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/limits.h>
#include <linux/stddef.h>
#include <linux/notifier.h>

#include "conap_scp.h"
#include "conap_scp_priv.h"
#include "conap_scp_ipi.h"
#include "conap_platform_data.h"

#define MTK_CONAP_IPI_SUPPORT 1

#ifdef MTK_CONAP_IPI_SUPPORT
/* SCP */
#include "scp.h"
#endif

#define MAX_IPI_SEND_DATA_SZ 56
struct ipi_send_data {
	uint16_t drv_type;
	uint16_t msg_id;
	uint16_t total_sz;
	uint16_t this_sz;
	char data[MAX_IPI_SEND_DATA_SZ];
};

struct ipi_ack_data {
	uint16_t drv_type;
	uint16_t msg_id;
	uint16_t total_sz;
	uint16_t this_sz;
};

static struct ipi_send_data g_send_data;

struct conap_scp_ipi_cb g_ipi_cb;

#ifdef MTK_CONAP_IPI_SUPPORT
static char g_ipi_ack_data[128];
static int scp_ctrl_event_handler(struct notifier_block *this,
	unsigned long event, void *ptr);

static struct notifier_block scp_ctrl_notifier = {
	.notifier_call = scp_ctrl_event_handler,
};

int scp_ctrl_event_handler(struct notifier_block *this,
	unsigned long event, void *ptr)
{

	switch (event) {
	case SCP_EVENT_STOP:
		pr_info("[%s] SCP STOP", __func__);
		if (g_ipi_cb.conap_scp_ipi_ctrl_notify)
			(*g_ipi_cb.conap_scp_ipi_ctrl_notify)(0);
		break;
	case SCP_EVENT_READY:
		pr_info("[%s] SCP READY", __func__);
		if (g_ipi_cb.conap_scp_ipi_ctrl_notify)
			(*g_ipi_cb.conap_scp_ipi_ctrl_notify)(1);
		break;
	default:
		pr_info("scp notify event error %lu", event);
	}
	return 0;
}
#endif

unsigned int conap_scp_ipi_is_scp_ready(void)
{
#ifdef MTK_CONAP_IPI_SUPPORT
	return is_scp_ready(SCP_A_ID);
#else
	return 0;
#endif
}


unsigned int conap_scp_ipi_msg_sz(void)
{
	unsigned int bytes = connsys_scp_ipi_mbox_size();

	if (bytes <= offsetof(struct ipi_send_data, data) ||
	    bytes > sizeof(struct ipi_send_data) || bytes % sizeof(uint32_t))
		return 0;
	return bytes - offsetof(struct ipi_send_data, data);
}

int ipi_recv_cb(unsigned int id, void *prdata, void *data, unsigned int len)
{
	struct ipi_send_data packet;
	unsigned int bytes = connsys_scp_ipi_mbox_size();

	/* ISR dispatch reports bytes; process dispatch reports mailbox words.
	 * Both refer to this registered, fixed-capacity receive buffer.
	 */
	if (id != IPI_IN_SCP_CONNSYS || data != &g_ipi_ack_data[0] ||
	    bytes < offsetof(struct ipi_send_data, data) ||
	    bytes > sizeof(packet) || bytes % sizeof(uint32_t) ||
	    (len != bytes && len != bytes / sizeof(uint32_t)))
		return -EINVAL;

	memcpy(&packet, data, bytes);
	if (packet.drv_type >= CONAP_SCP_DRV_NUM ||
	    packet.this_sz > bytes - offsetof(struct ipi_send_data, data) ||
	    packet.this_sz > packet.total_sz)
		return -EINVAL;

	if (g_ipi_cb.conap_scp_ipi_msg_notify)
		g_ipi_cb.conap_scp_ipi_msg_notify(packet.drv_type, packet.msg_id,
			packet.total_sz, (uint8_t *)&packet.data[0], packet.this_sz);
	return 0;
}

int conap_scp_ipi_send_data(enum conap_scp_drv_type drv_type, uint16_t msg_id, uint32_t total_sz,
						uint8_t *buf, uint32_t size)
{
#ifdef MTK_CONAP_IPI_SUPPORT
	unsigned int retry_time = 500;
	unsigned int retry_cnt = 0;
	int ipi_result = -1;
#endif

	unsigned int bytes = connsys_scp_ipi_mbox_size();

	if ((unsigned int)drv_type >= CONAP_SCP_DRV_NUM ||
	    bytes < offsetof(struct ipi_send_data, data) ||
	    bytes > sizeof(g_send_data) || bytes % sizeof(uint32_t) ||
	    size > bytes - offsetof(struct ipi_send_data, data) ||
	    size > sizeof(g_send_data.data) || total_sz > U16_MAX ||
	    size > total_sz || (size && !buf))
		return -EINVAL;

	memset(&g_send_data, 0, sizeof(g_send_data));
	g_send_data.drv_type = drv_type;
	g_send_data.msg_id = msg_id;
	g_send_data.total_sz = total_sz;
	g_send_data.this_sz = size;

	if (buf != NULL && size > 0)
		memcpy(&(g_send_data.data[0]), buf, size);

#ifdef MTK_CONAP_IPI_SUPPORT
	for (retry_cnt = 0; retry_cnt <= retry_time; retry_cnt++) {
		ipi_result = mtk_ipi_send(&scp_ipidev,
							IPI_OUT_SCP_CONNSYS,
							0,
							&g_send_data,
							connsys_scp_ipi_mbox_size()/4,
							0);
		if (ipi_result == IPI_ACTION_DONE)
			break;
		udelay(1000);
	}
	if (ipi_result != 0) {
		pr_err("[ipi_send_data] send fail [%d]", ipi_result);
		return -1;
	}
#else
	pr_notice("[%s] mtk_ipi_send is not support", __func__);
#endif

	return size;
}


int conap_scp_ipi_send_cmd(enum conap_scp_drv_type drv_type, uint16_t msg_id,
					uint32_t p0, uint32_t p1)
{
	struct msg_cmd cmd;
#ifdef MTK_CONAP_IPI_SUPPORT
	unsigned int retry_time = 500;
	unsigned int retry_cnt = 0;
	int ipi_result = -1;
#endif

	if ((unsigned int)drv_type >= CONAP_SCP_DRV_NUM ||
	    connsys_scp_ipi_mbox_size() < sizeof(cmd) ||
	    connsys_scp_ipi_mbox_size() > sizeof(g_send_data) ||
	    connsys_scp_ipi_mbox_size() % sizeof(uint32_t))
		return -EINVAL;

	cmd.drv_type = drv_type;
	cmd.msg_id = msg_id;
	cmd.total_sz = 8;
	cmd.this_sz = 8;
	cmd.param0 = p0;
	cmd.param1 = p1;

#ifdef MTK_CONAP_IPI_SUPPORT
	for (retry_cnt = 0; retry_cnt <= retry_time; retry_cnt++) {
		ipi_result = mtk_ipi_send(&scp_ipidev,
							IPI_OUT_SCP_CONNSYS,
							0,
							&cmd,
							sizeof(struct msg_cmd)/4,
							0);
		if (ipi_result == IPI_ACTION_DONE)
			break;
		mdelay(1);
	}
	if (ipi_result != 0) {
		pr_err("[ipi_send_cmd] send cmd fail=[%d]", ipi_result);
		return -1;
	}
#else
	pr_notice("[%s] mtk_ipi_send is not support", __func__);
#endif
	return 0;
}

int conap_scp_ipi_init(struct conap_scp_ipi_cb *cb)
{
#ifdef MTK_CONAP_IPI_SUPPORT
	int ret = IPI_ACTION_DONE;
#endif

	memcpy(&g_ipi_cb, cb, sizeof(struct conap_scp_ipi_cb));

#ifdef MTK_CONAP_IPI_SUPPORT
	ret = mtk_ipi_register(&scp_ipidev, IPI_IN_SCP_CONNSYS,
					ipi_recv_cb, NULL, &g_ipi_ack_data[0]);
	if (ret != IPI_ACTION_DONE) {
		pr_info("[%s] ipi_register ret=[%d]", __func__, ret);
		return -1;
	}

	scp_A_register_notify(&scp_ctrl_notifier);
#else
	pr_notice("[%s] mtk_ipi_send is not support", __func__);
#endif

	return 0;
}

int conap_scp_ipi_deinit(void)
{
	int ret = 0;

#ifdef MTK_CONAP_IPI_SUPPORT
	scp_A_unregister_notify(&scp_ctrl_notifier);
	ret = mtk_ipi_unregister(&scp_ipidev, IPI_IN_SCP_CONNSYS);
	pr_info("[%s] ipi_unregister ret=[%d]", __func__, ret);
#else
	pr_notice("[%s] mtk_ipi_send is not support", __func__);
#endif
	return ret;
}
