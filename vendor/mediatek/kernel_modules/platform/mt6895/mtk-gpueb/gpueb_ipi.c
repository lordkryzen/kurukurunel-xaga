// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

/**
 * @file    gpueb_ipi_init.c
 * @brief   IPI init flow for gpueb
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <linux/random.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <mboot_params.h>

#include "gpueb_ipi.h"
#include "gpueb_helper.h"
#include "ghpm_wrapper.h"

// MTK common IPI/MBOX
#include <linux/soc/mediatek/mtk_tinysys_ipi.h>
#include <linux/soc/mediatek/mtk-mbox.h>
#include "gpueb_ipi_contract.h"
#include "transport_lifetime.h"
#include "gpueb_lifecycle.h"

struct mtk_mbox_device   gpueb_mboxdev;
struct mtk_ipi_device    gpueb_ipidev;
struct mtk_mbox_info     *gpueb_mbox_info;
struct mtk_mbox_pin_send *gpueb_mbox_pin_send;
struct mtk_mbox_pin_recv *gpueb_mbox_pin_recv;
const char *gpueb_mbox_pin_send_name[20];
const char *gpueb_mbox_pin_recv_name[20];
unsigned int g_mbox_size = 0;
unsigned int g_slot_size = 0;
unsigned int g_ts_mbox;
static void __iomem *g_gpueb_mbox_ipi;
static unsigned int g_registered_irqs;
static bool g_ipi_ready;

#if IPI_TEST
struct test_msg {
	int msg;
	int padding[3];
} msg_tx[32], msg_rx[32];
#endif

static int gpueb_ipi_table_init(struct platform_device *pdev)
{
	struct gpueb_ipi_layout layout;
	struct mtk_mbox_info *info;
	struct mtk_mbox_pin_send *send;
	struct mtk_mbox_pin_recv *receive;
	u32 offsets[GPUEB_MAX_MAILBOXES] = {};
	u32 i;
	int ret;

	if (gpueb_mboxdev.info_table || gpueb_mboxdev.pin_send_table ||
	    gpueb_mboxdev.pin_recv_table)
		return false;
	ret = gpueb_read_ipi_layout(pdev->dev.of_node, &layout);
	if (ret) {
		gpueb_log_i(GPUEB_TAG, "invalid mailbox layout: %d", ret);
		return false;
	}
	info = vzalloc(array_size(layout.mailbox_count, sizeof(*info)));
	if (!info)
		return false;
	send = vzalloc(array_size(layout.send_count, sizeof(*send)));
	if (!send)
		goto free_info;
	receive = vzalloc(array_size(layout.receive_count, sizeof(*receive)));
	if (!receive)
		goto free_send;
	for (i = 0; i < layout.mailbox_count; i++) {
		info[i].id = i;
		info[i].slot = layout.mailbox_words;
		/* The 6.12 mailbox core publishes enable only after IRQ setup. */
		info[i].enable = 0;
		info[i].is64d = 0;
		info[i].opt = MBOX_OPT_SMEM;
	}
	for (i = 0; i < layout.send_count; i++) {
		send[i].chan_id = layout.send[i].channel;
		send[i].pin_index = layout.send[i].channel;
		send[i].mbox = layout.send[i].mailbox;
		send[i].msg_size = layout.send[i].words;
		send[i].offset = offsets[layout.send[i].mailbox];
		offsets[layout.send[i].mailbox] += layout.send[i].words;
	}
	for (i = 0; i < layout.receive_count; i++) {
		receive[i].chan_id = layout.receive[i].channel;
		receive[i].pin_index = layout.receive[i].channel;
		receive[i].mbox = layout.receive[i].mailbox;
		receive[i].msg_size = layout.receive[i].words;
		receive[i].offset = offsets[layout.receive[i].mailbox];
		offsets[layout.receive[i].mailbox] += layout.receive[i].words;
		receive[i].recv_opt = layout.receive[i].receive_option;
		receive[i].cb_ctx_opt = layout.receive[i].callback_context;
	}
	/* All arrays are complete before the caller can register any mailbox. */
	gpueb_mboxdev.name = GPUEB_MBOXDEV_NAME;
	gpueb_mboxdev.count = layout.mailbox_count;
	gpueb_mboxdev.info_table = info;
	gpueb_mboxdev.pin_send_table = send;
	gpueb_mboxdev.pin_recv_table = receive;
	gpueb_mbox_info = info;
	gpueb_mbox_pin_send = send;
	gpueb_mbox_pin_recv = receive;
	for (i = 0; i < layout.send_count; i++)
		gpueb_mbox_pin_send_name[i] = layout.send[i].name;
	for (i = 0; i < layout.receive_count; i++)
		gpueb_mbox_pin_recv_name[i] = layout.receive[i].name;
	g_mbox_size = layout.mailbox_words;
	g_slot_size = layout.slot_bytes;
	g_ts_mbox = layout.timesync_mailbox;
	gpueb_mboxdev.send_count = layout.send_count;
	gpueb_mboxdev.recv_count = layout.receive_count;
	return true;

free_send:
	vfree(send);
free_info:
	vfree(info);
	return false;
}

void gpueb_mbox_setup_pin_table(unsigned int mbox)
{
	unsigned int i;
	int last_ofs = 0;

	for (i = 0; i < gpueb_mboxdev.send_count; i++) {
		if (mbox == gpueb_mbox_pin_send[i].mbox) {
			gpueb_mbox_pin_send[i].offset = last_ofs;
			last_ofs += gpueb_mbox_pin_send[i].msg_size;
		}
	}

	for (i = 0; i < gpueb_mboxdev.recv_count; i++) {
		if (mbox == gpueb_mbox_pin_recv[i].mbox) {
			gpueb_mbox_pin_recv[i].offset = last_ofs;
			last_ofs += gpueb_mbox_pin_recv[i].msg_size;
		}
	}

	if (last_ofs > g_mbox_size)
		gpueb_log_d(GPUEB_TAG, "mbox%d exceed the maximum size", mbox);

	return;
}

void gpueb_plat_ipi_timeout_cb(int ipi_id)
{
	gpueb_log_d(GPUEB_TAG, "Error: possible error IPI %d", ipi_id);

	ipi_monitor_dump(&gpueb_ipidev);
	//mtk_emidbg_dump();
	//BUG_ON(1);

	return;
}

/* Only for failed initialization: no client has received the transport yet. */
static void gpueb_ipi_abort_init(unsigned int registered_irqs)
{
	while (registered_irqs) {
		struct mtk_mbox_info *info = &gpueb_mbox_info[--registered_irqs];

		WRITE_ONCE(info->enable, false);
		/* free_irq waits for any in-progress handler before arrays are freed. */
		free_irq(info->irq_num, info);
	}
	vfree(gpueb_mboxdev.pin_recv_table);
	vfree(gpueb_mboxdev.pin_send_table);
	vfree(gpueb_mboxdev.info_table);
	memset(&gpueb_mboxdev, 0, sizeof(gpueb_mboxdev));
	memset(&gpueb_ipidev, 0, sizeof(gpueb_ipidev));
	gpueb_mbox_info = NULL;
	gpueb_mbox_pin_send = NULL;
	gpueb_mbox_pin_recv = NULL;
	memset(gpueb_mbox_pin_send_name, 0, sizeof(gpueb_mbox_pin_send_name));
	memset(gpueb_mbox_pin_recv_name, 0, sizeof(gpueb_mbox_pin_recv_name));
	g_mbox_size = 0;
	g_slot_size = 0;
	g_ts_mbox = 0;
	g_gpueb_mbox_ipi = NULL;
}

int gpueb_ipi_init(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *mbox_ipi;
	unsigned int i, registered_irqs = 0;
	int ret;

#if IPI_TEST
	/* This diagnostic registers clients after transport publication and has
	 * a separate teardown contract. Do not enter that partial-init path.
	 */
	return -EOPNOTSUPP;
#endif
	if (gpueb_mboxdev.info_table || gpueb_ipidev.ipi_inited)
		return -EBUSY;
	/* Complete fallible resource lookups before registering the IPI device. */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mbox0_send");
	if (!res || resource_size(res) < sizeof(u32))
		return -ENODEV;
	mbox_ipi = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!mbox_ipi)
		return -ENOMEM;
	if (!gpueb_ipi_table_init(pdev))
		return -EINVAL;

	for (i = 0; i < gpueb_mboxdev.count; i++) {
		gpueb_mbox_info[i].mbdev = &gpueb_mboxdev;
		ret = mtk_mbox_probe_disabled(pdev, &gpueb_mboxdev, i);
		if (ret)
			goto abort_init;
		registered_irqs++;
	}
	/* Every IRQ is still masked: the IPI core can initialize the pin locks
	 * and callbacks exactly once without racing receive-side access.
	 */
	gpueb_ipidev.name = "gpueb_ipidev";
	gpueb_ipidev.id = IPI_DEV_GPUEB;
	gpueb_ipidev.mbdev = &gpueb_mboxdev;
	gpueb_ipidev.timeout_handler = gpueb_plat_ipi_timeout_cb;
	ret = mtk_ipi_device_register(&gpueb_ipidev, pdev, &gpueb_mboxdev,
		gpueb_mboxdev.send_count);
	if (ret != IPI_ACTION_DONE)
		goto abort_init;
	g_gpueb_mbox_ipi = mbox_ipi;
	/* All remaining operations are non-failing. No pin state is reinitialized
	 * after this point; offsets were prepared before mailbox registration.
	 */
	g_registered_irqs = registered_irqs;
	for (i = 0; i < registered_irqs; i++)
		enable_irq(gpueb_mbox_info[i].irq_num);
	smp_store_release(&g_ipi_ready, true);
	return 0;

abort_init:
	gpueb_ipi_abort_init(registered_irqs);
	return ret;
}

/* Owner has removed frontends and stopped all producers before entry. */
int gpueb_ipi_exit(void)
{
	int ret;

	smp_store_release(&g_ipi_ready, false);
	if (!gpueb_ipidev.ipi_inited)
		return 0;
	while (g_registered_irqs) {
		struct mtk_mbox_info *info = &gpueb_mbox_info[--g_registered_irqs];

		WRITE_ONCE(info->enable, false);
		free_irq(info->irq_num, info);
	}
	ret = mtk_ipi_device_destroy_quiesced(&gpueb_ipidev);
	if (ret)
		return ret;
	gpueb_ipi_abort_init(0);
	return 0;
}

#if IPI_TEST
int gpueb_ipi_test_init(void)
{
	int ret = 0;
	int ipi = 0;
	unsigned int ipi_count = gpueb_mboxdev.send_count;

	/* Register IPI channel */
	for (ipi = 0; ipi < ipi_count; ipi++) {
		ret = mtk_ipi_register(&gpueb_ipidev,
				ipi,
				NULL,
				NULL,
				(void *)&msg_rx[ipi]);
		if (ret != IPI_ACTION_DONE) {
			gpueb_log_d(GPUEB_TAG, "ipi:#%d register fail! ret = %d",
					ipi, ret);
			if (ret == IPI_DUPLEX) {
				/* ipi already registered, unregister it and register again */
				gpueb_log_d(GPUEB_TAG, "ipi:#%d register: IPI_DUPLEX",
					ipi);
				ret = mtk_ipi_unregister(&gpueb_ipidev, ipi);
				if (ret != IPI_ACTION_DONE) {
					gpueb_log_d(GPUEB_TAG, "ipi:#%d unregister fail! ret = %d",
						ipi, ret);
					break;
				}
				ret = mtk_ipi_register(&gpueb_ipidev, ipi, NULL, NULL,
					(void *)&msg_rx[ipi]);
				if (ret != IPI_ACTION_DONE) {
					gpueb_log_d(GPUEB_TAG, "ipi:#%d register fail again! ret = %d",
						ipi, ret);
					break;
				}
			} else {
				break;
			}
		}
	}
	return ret;
}

int gpueb_ipi_send_compl_test(int ipi, int msg)
{
	int ret = 0;

	/* Test mtk_ipi_send_compl */
	msg_tx[ipi].msg = msg;
	gpueb_log_d(GPUEB_TAG, "ipi:#%d mtk_ipi_send_compl data: %d",
			ipi, msg_tx[ipi].msg);
	ret = mtk_ipi_send_compl(
		&gpueb_ipidev, // GPUEB's IPI device
		ipi, // Send channel
		0, // 0: wait, 1: polling
		(void *)&msg_tx[ipi], // Send data
		1, // 1 slot message = 1 * 4 = 4 bytes
		IPI_TIMEOUT_MS); // Timeout value in milisecond

	if (ret != IPI_ACTION_DONE) {
		gpueb_log_i(GPUEB_TAG, "IPI fail ret=%d", ret);
		return ret;
	}
	gpueb_log_d(GPUEB_TAG, "ipi:#%d ack data: %d",
			ipi, msg_rx[ipi].msg);

	return msg_rx[ipi].msg;
}

int gpueb_get_send_pin_count(void)
{
	return gpueb_mboxdev.send_count;
}
#endif

unsigned int gpueb_get_ts_mbox(void)
{
	return g_ts_mbox;
}

int gpueb_get_send_PIN_offset_by_name(char *send_PIN_name)
{
	int i;

	for (i = 0; i < gpueb_mboxdev.send_count; i++) {
		if (!strcmp(gpueb_mbox_pin_send_name[i], send_PIN_name))
			return gpueb_mbox_pin_send[i].offset;
	}
	return -1;
}

int gpueb_get_send_PIN_ID_by_name(char *send_PIN_name)
{
	int i;

	for (i = 0; i < gpueb_mboxdev.send_count; i++) {
		if (!strcmp(gpueb_mbox_pin_send_name[i], send_PIN_name))
			return gpueb_mbox_pin_send[i].chan_id;
	}
	return -1;
}
EXPORT_SYMBOL_GPL(gpueb_get_send_PIN_ID_by_name);

int gpueb_get_recv_PIN_ID_by_name(char *recv_PIN_name)
{
	int i;

	for (i = 0; i < gpueb_mboxdev.recv_count; i++) {
		if (!strcmp(gpueb_mbox_pin_recv_name[i], recv_PIN_name))
			return gpueb_mbox_pin_recv[i].chan_id;
	}
	return -1;
}
EXPORT_SYMBOL_GPL(gpueb_get_recv_PIN_ID_by_name);

void *get_gpueb_ipidev(void)
{
	return smp_load_acquire(&g_ipi_ready) ? &gpueb_ipidev : NULL;
}
EXPORT_SYMBOL_GPL(get_gpueb_ipidev);

int get_gpueb_slot_size(void)
{
	return g_slot_size;
}
EXPORT_SYMBOL_GPL(get_gpueb_slot_size);

unsigned int gpueb_get_mbox1_irq(void)
{
	unsigned int val = 0;

	if (g_gpueb_mbox_ipi) {
		/* g_gpueb_mbox_ipi (0x13C62000) */
		/* GPUEB to APMCU IRQ[1] SW INT Status (0x13C62088) */
		val = readl(g_gpueb_mbox_ipi + 0x88);
	} else
		gpueb_log_i(GPUEB_TAG, "null g_gpueb_mbox_ipi");

	return val;
}
EXPORT_SYMBOL_GPL(gpueb_get_mbox1_irq);

void gpueb_clr_mbox1_irq(unsigned int val)
{
	if (g_gpueb_mbox_ipi) {
		/* g_gpueb_mbox_ipi (0x13C62000) */
		/* GPUEB to APMCU IRQ[1] SW INT Clear (0x13C62084) */
		writel(val, (g_gpueb_mbox_ipi + 0x84));
	} else
		gpueb_log_i(GPUEB_TAG, "null g_gpueb_mbox_ipi");
}
EXPORT_SYMBOL_GPL(gpueb_clr_mbox1_irq);

int mtk_ipi_send_compl_to_gpueb(int ipi_id, int opt, void *data, int len, unsigned long timeout)
{
	int ret;

	if (!smp_load_acquire(&g_ipi_ready))
		return -EAGAIN;

	gpueb_log_d(GHPM_TAG, "ENTRY, ipi_id=%d", ipi_id);

	/* On mfg0 and gpueb */
	ret = gpueb_ctrl(GHPM_ON, MFG1_OFF, SUSPEND_POWER_ON);
	if (ret) {
		gpueb_log_e(GHPM_TAG, "gpueb on fail, return value=%d\n", ret);
		return ret;
	}

	gpueb_log_d(GHPM_TAG, "gpueb ctrl on finish and ready to mtk_ipi_send_compl");

	ret = mtk_ipi_send_compl(get_gpueb_ipidev(), ipi_id, opt, data, len, timeout);
	if (unlikely(ret != IPI_ACTION_DONE)) {
		gpueb_log_e(GHPM_TAG, "[ABORT] fail to send IPI: ipi_id=%d ret=%d", ipi_id, ret);
		return ret;
	}
	gpueb_log_d(GHPM_TAG, "mtk_ipi_send_compl success\n");

	/* Off mfg0 and gpueb */
	ret = gpueb_ctrl(GHPM_OFF, MFG1_OFF, SUSPEND_POWER_OFF);
	if (ret) {
		gpueb_log_e(GHPM_TAG, "gpueb off fail, return value=%d\n", ret);
		return ret;
	}

	gpueb_log_d(GHPM_TAG, "Exit, ipi_id=%d\n", ipi_id);

	return ret;
}
EXPORT_SYMBOL(mtk_ipi_send_compl_to_gpueb);
