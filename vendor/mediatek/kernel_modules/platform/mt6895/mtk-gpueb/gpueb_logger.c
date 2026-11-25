// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

/**
 * @file    gpueb_logger.c
 * @brief   Mobile log for GPUEB
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <linux/random.h>
#include <linux/seq_file.h>
#include <mboot_params.h>

// MTK common IPI/MBOX
#include <linux/soc/mediatek/mtk_tinysys_ipi.h>
#include <linux/soc/mediatek/mtk-mbox.h>

#include "gpueb_helper.h"
#include "gpueb_lifecycle.h"
#include "gpueb_ipi.h"
#include "gpueb_logger.h"
#include "gpueb_reserved_mem.h"
#include "gpueb_plat_service.h"

static unsigned int gpueb_logger_inited;
static struct buffer_info_s __iomem *gpueb_buf_info;
static DEFINE_MUTEX(gpueb_logger_mutex);
static DEFINE_MUTEX(gpueb_log_enable_mutex);
static struct log_ctrl_s __iomem *gpueb_log_ctl;

unsigned int r_pos_debug;
unsigned int log_ctl_debug;

static int gpueb_log_enable_set(unsigned int enable)
{
	int ret = 0;
	int channel_id;
	union {
		struct plat_ipi_send_data message;
		u32 words[4];
	} plat_send_data = {};

	static_assert(sizeof(plat_send_data) == 4 * sizeof(u32));

	plat_send_data.message.cmd = PLT_LOG_ENABLE;
	plat_send_data.message.u.logger.enable = enable;

	if (enable > 1)
		return -EINVAL;
	if (!gpueb_logger_inited)
		return -ENODEV;

	if (gpueb_logger_inited) {
		// Send ipi to invoke gpueb logger
		channel_id = gpueb_get_send_PIN_ID_by_name("IPI_ID_PLATFORM");
		if (channel_id < 0) {
			gpueb_log_d(GPUEB_TAG, "get channel ID fail!");
			return -ENODEV;
		}

		// CH_PLATFORM message size is 16 byte, 4 slots
		ret = mtk_ipi_send_compl(
			&gpueb_ipidev, // GPUEB's IPI device
			channel_id, // Send channel
			0,   // 0: wait, 1: polling
			(void *)&plat_send_data, // Send data
			4,   // 4 slots message = 4 * 4 = 16tyte
			2000 // Timeout value in milisecond);
		);
		if ((ret == IPI_ACTION_DONE) && (enable == 1))
			writel(1, &gpueb_log_ctl->enable);
		else if ((ret == IPI_ACTION_DONE) && (enable == 0))
			writel(0, &gpueb_log_ctl->enable);

		if (ret != IPI_ACTION_DONE) {
			gpueb_log_i(GPUEB_TAG, "IPI fail ret=%d", ret);
			return ret < 0 ? ret : -EIO;
		}
	}

	return 0;
}


static ssize_t gpueb_mobile_log_show(struct device *kobj,
		struct device_attribute *attr, char *buf)
{
	unsigned int stat;

	stat = (gpueb_logger_inited && readl(&gpueb_log_ctl->enable)) ? 1 : 0;

	return sprintf(buf, "[GPUEB] mobile log is %s\n",
			(stat == 0x1) ? "enabled" : "disabled");
}

static ssize_t gpueb_mobile_log_store(struct device *kobj,
		struct device_attribute *attr, const char *buf, size_t n)
{
	unsigned int enable;
	int ret;

	if (kstrtouint(buf, 0, &enable) != 0)
		return -EINVAL;

	mutex_lock(&gpueb_log_enable_mutex);
	ret = gpueb_log_enable_set(enable);
	mutex_unlock(&gpueb_log_enable_mutex);

	return ret ? ret : n;
}
DEVICE_ATTR_RW(gpueb_mobile_log);

ssize_t gpueb_log_read(char __user *data, size_t len)
{
	u32 read_position, write_position, bytes, offset;
	void *temporary = NULL;
	ssize_t ret;

	mutex_lock(&gpueb_logger_mutex);
	if (!gpueb_logger_inited || !gpueb_buf_info || !gpueb_log_ctl) {
		ret = -ENODEV;
		goto out;
	}
	read_position = readl(&gpueb_buf_info->r_pos);
	write_position = readl(&gpueb_buf_info->w_pos);
	offset = ROUNDUP(sizeof(*gpueb_log_ctl) + sizeof(*gpueb_buf_info), 4);
	if (read_position >= DRAM_BUF_LEN || write_position >= DRAM_BUF_LEN ||
	    readl(&gpueb_log_ctl->buff_ofs) != offset ||
	    readl(&gpueb_log_ctl->buff_size) != DRAM_BUF_LEN) {
		ret = -EIO;
		goto out;
	}
	bytes = read_position <= write_position ? write_position - read_position :
		DRAM_BUF_LEN - read_position;
	bytes = min_t(size_t, bytes, min_t(size_t, len, PAGE_SIZE));
	if (!bytes) {
		ret = 0;
		goto out;
	}
	temporary = kmalloc(bytes, GFP_KERNEL);
	if (!temporary) {
		ret = -ENOMEM;
		goto out;
	}
	memcpy_fromio(temporary, (void __iomem *)gpueb_log_ctl + offset + read_position, bytes);
	if (copy_to_user(data, temporary, bytes)) {
		ret = -EFAULT;
		goto out;
	}
	writel((read_position + bytes) % DRAM_BUF_LEN, &gpueb_buf_info->r_pos);
	ret = bytes;
out:
	kfree(temporary);
	mutex_unlock(&gpueb_logger_mutex);
	return ret;
}

ssize_t gpueb_log_if_read(struct file *file,
		char __user *data, size_t len, loff_t *ppos)
{
	ssize_t ret = 0;

	ret = gpueb_log_read(data, len);

	return ret;
}

int gpueb_log_if_open(struct inode *inode, struct file *file)
{
	if (!READ_ONCE(gpueb_logger_inited))
		return -ENODEV;
	return nonseekable_open(inode, file);
}

unsigned int gpueb_log_poll(void)
{
	unsigned int result = 0;

	mutex_lock(&gpueb_logger_mutex);
	if (!gpueb_logger_inited)
		result = POLLHUP;
	else if (readl(&gpueb_buf_info->r_pos) >= DRAM_BUF_LEN ||
		 readl(&gpueb_buf_info->w_pos) >= DRAM_BUF_LEN)
		result = POLLERR;
	else if (readl(&gpueb_buf_info->r_pos) != readl(&gpueb_buf_info->w_pos))
		result = POLLIN | POLLRDNORM;
	mutex_unlock(&gpueb_logger_mutex);
	return result;
}

unsigned int gpueb_log_if_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;

	if (!(file->f_mode & FMODE_READ))
		return ret;

	/*poll_wait(file, &gpueb_logwait, wait);*/

	ret = gpueb_log_poll();

	return ret;
}

int gpueb_logger_init(struct platform_device *pdev,
	phys_addr_t start, phys_addr_t limit)
{
	int buffer_offset = 0;
	int total_size = 0;

	/* Validate the full layout before the first shared-memory write. */
	if (!start || limit < ROUNDUP(sizeof(*gpueb_log_ctl) + sizeof(*gpueb_buf_info), 4) + DRAM_BUF_LEN)
		return -ENOSPC;

	mutex_lock(&gpueb_log_enable_mutex);
	mutex_lock(&gpueb_logger_mutex);
	if (gpueb_logger_inited) {
		mutex_unlock(&gpueb_logger_mutex);
		mutex_unlock(&gpueb_log_enable_mutex);
		return -EBUSY;
	}
	// Init dram ctrl table
	gpueb_log_ctl = (struct log_ctrl_s __iomem *) start;
	writel(PLT_LOG_ENABLE, &gpueb_log_ctl->base); /* magic */
	writel(0, &gpueb_log_ctl->enable);
	writel(sizeof(*gpueb_log_ctl), &gpueb_log_ctl->size);
	writel(sizeof(*gpueb_log_ctl), &gpueb_log_ctl->info_ofs);

	buffer_offset = sizeof(*gpueb_log_ctl) + sizeof(*gpueb_buf_info);
	buffer_offset = ROUNDUP(buffer_offset, 4);

	// Available start offset for log data
	writel(buffer_offset, &gpueb_log_ctl->buff_ofs);
	// Buffer length for log data
	writel(DRAM_BUF_LEN, &gpueb_log_ctl->buff_size);

	gpueb_buf_info = (struct buffer_info_s __iomem *)
		((void __iomem *)gpueb_log_ctl + sizeof(*gpueb_log_ctl));
	writel(0, &gpueb_buf_info->r_pos);
	writel(0, &gpueb_buf_info->w_pos);

	total_size = buffer_offset + DRAM_BUF_LEN;

	WRITE_ONCE(gpueb_logger_inited, 1);
	mutex_unlock(&gpueb_logger_mutex);
	mutex_unlock(&gpueb_log_enable_mutex);
	return total_size;
}

void gpueb_logger_exit(void)
{
	mutex_lock(&gpueb_log_enable_mutex);
	mutex_lock(&gpueb_logger_mutex);
	WRITE_ONCE(gpueb_logger_inited, 0);
	gpueb_log_ctl = NULL;
	gpueb_buf_info = NULL;
	mutex_unlock(&gpueb_logger_mutex);
	mutex_unlock(&gpueb_log_enable_mutex);
}
