// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <conap_scp.h>
#include <linux/poll.h>

#include "conap_scp_priv.h"
#ifdef AOLTEST_SUPPORT
#include "aoltest_core.h"
#endif

#define CONN_SCP_DEVNAME            "connscp"

struct conn_scp_dev {
	struct class *cls;
	struct device *dev;
	dev_t devno;
	struct cdev chdev;
};

struct conn_scp_dev *g_conn_scp_dev;

const struct file_operations g_conn_scp_dev_fops = {
	.owner = THIS_MODULE,
};

int conn_scp_init(void)
{
	int ret;

	g_conn_scp_dev = kzalloc(sizeof(*g_conn_scp_dev), GFP_KERNEL);
	if (!g_conn_scp_dev)
		return -ENOMEM;

	ret = alloc_chrdev_region(&g_conn_scp_dev->devno, 0, 1, CONN_SCP_DEVNAME);
	if (ret)
		goto err_free;

	cdev_init(&g_conn_scp_dev->chdev, &g_conn_scp_dev_fops);
	g_conn_scp_dev->chdev.owner = THIS_MODULE;
	ret = cdev_add(&g_conn_scp_dev->chdev, g_conn_scp_dev->devno, 1);
	if (ret)
		goto err_region;

	g_conn_scp_dev->cls = class_create("conn_scp");
	if (IS_ERR(g_conn_scp_dev->cls)) {
		ret = PTR_ERR(g_conn_scp_dev->cls);
		goto err_cdev;
	}

	g_conn_scp_dev->dev = device_create(g_conn_scp_dev->cls, NULL,
		g_conn_scp_dev->devno, g_conn_scp_dev, "conn_scp");
	if (IS_ERR(g_conn_scp_dev->dev)) {
		ret = PTR_ERR(g_conn_scp_dev->dev);
		goto err_class;
	}

	ret = conap_scp_init();
	if (ret)
		goto err_device;

	return 0;

err_device:
	device_destroy(g_conn_scp_dev->cls, g_conn_scp_dev->devno);
err_class:
	class_destroy(g_conn_scp_dev->cls);
err_cdev:
	cdev_del(&g_conn_scp_dev->chdev);
err_region:
	unregister_chrdev_region(g_conn_scp_dev->devno, 1);
err_free:
	kfree(g_conn_scp_dev);
	g_conn_scp_dev = NULL;
	return ret;
}

void conn_scp_exit(void)
{
	if (!g_conn_scp_dev)
		return;

	conap_scp_deinit();
	device_destroy(g_conn_scp_dev->cls, g_conn_scp_dev->devno);
	class_destroy(g_conn_scp_dev->cls);
	cdev_del(&g_conn_scp_dev->chdev);
	unregister_chrdev_region(g_conn_scp_dev->devno, 1);
	kfree(g_conn_scp_dev);
	g_conn_scp_dev = NULL;
}


module_init(conn_scp_init);
module_exit(conn_scp_exit);
MODULE_AUTHOR("Willy Yu <Willy.Yu@mediatek.com>");
MODULE_DESCRIPTION("Conn SCP Bridge dev");
MODULE_LICENSE("GPL");

