// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2018 MediaTek Inc.

#include <linux/compat.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/remoteproc.h>
#include <linux/rwsem.h>
#include <linux/platform_data/mtk_ccd.h>
#include <linux/remoteproc/mtk_ccd_mem.h>
#include <linux/rpmsg/mtk_ccd_rpmsg.h>
#include <uapi/linux/mtk_ccd_controls.h>

#include "remoteproc_internal.h"

#define CCD_DEV_NAME	"mtk_ccd"
#define MAX_CODE_SIZE 0x500000

/* The public CCD and daemon records retain their original layouts. */
struct ccd_context {
	struct mtk_ccd ccd;
	struct rw_semaphore daemon_gate;
	bool removing;
};

#define to_ccd_context(c) container_of(c, struct ccd_context, ccd)

/* CCD is a userspace message broker, not an ELF firmware processor. */

struct platform_device *ccd_get_pdev(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *ccd_node;
	struct platform_device *ccd_pdev;

	ccd_node = of_parse_phandle(dev->of_node, "mediatek,scp", 0);
	if (!ccd_node) {
		dev_dbg(dev, "can't get ccd node\n");
		return NULL;
	}

	ccd_pdev = of_find_device_by_node(ccd_node);
	of_node_put(ccd_node);
	if (WARN_ON(!ccd_pdev)) {
		dev_dbg(dev, "ccd pdev failed\n");
		return NULL;
	}

	return ccd_pdev;
}
EXPORT_SYMBOL_GPL(ccd_get_pdev);

void ccd_wdt_handler(struct mtk_ccd *ccd)
{
	rproc_report_crash(ccd->rproc, RPROC_WATCHDOG);
}

void ccd_init_ipi_handler(void *data, unsigned int len, void *priv)
{
	//struct mtk_ccd *ccd = (struct mtk_ccd *)priv;
}

int ccd_ipi_init(struct mtk_ccd *ccd)
{
	return 0;
}


static int ccd_start(struct rproc *rproc)
{
	struct mtk_ccd *ccd = (struct mtk_ccd *)rproc->priv;
	struct device *dev = ccd->dev;
	int ret = 0;

	dev_info(dev, "ccd started: %p\n", dev);

	return ret;
}

static void *ccd_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct mtk_ccd *ccd = (struct mtk_ccd *)rproc->priv;

	dev_info(ccd->dev, "%s: %p\n", __func__, ccd->dev);

	return NULL;
}

static int ccd_stop(struct rproc *rproc)
{
	struct mtk_ccd *ccd = (struct mtk_ccd *)rproc->priv;
	int ret = 0;

	dev_info(ccd->dev, "%s\n", __func__);

	return ret;
}

static const struct rproc_ops ccd_ops = {
	.start		= ccd_start,
	.stop		= ccd_stop,
	.attach		= ccd_start,
	.detach		= ccd_stop,
};

void *ccd_mapping_dm_addr(struct platform_device *pdev, u32 mem_addr)
{
	struct mtk_ccd *ccd = platform_get_drvdata(pdev);
	void *ptr;

	if (!ccd)
		return ERR_PTR(-ENODEV);
	ptr = ccd_da_to_va(ccd->rproc, mem_addr, 0);

	if (!ptr)
		return ERR_PTR(-EINVAL);

	return ptr;
}
EXPORT_SYMBOL_GPL(ccd_mapping_dm_addr);

static struct mtk_ccd_rpmsg_ops ccd_rpmsg_ops = {
	.ccd_send = rpmsg_ccd_ipi_send,
};

static void ccd_add_rpmsg_subdev(struct mtk_ccd *ccd)
{
	ccd->rpmsg_subdev =
		mtk_rpmsg_create_rproc_subdev(to_platform_device(ccd->dev),
					      &ccd_rpmsg_ops);
	if (ccd->rpmsg_subdev)
		rproc_add_subdev(ccd->rproc, ccd->rpmsg_subdev);
}

static void ccd_remove_rpmsg_subdev(struct mtk_ccd *ccd)
{
	if (ccd->rpmsg_subdev) {
		rproc_remove_subdev(ccd->rproc, ccd->rpmsg_subdev);
		mtk_rpmsg_destroy_rproc_subdev(ccd->rpmsg_subdev);
		ccd->rpmsg_subdev = NULL;
	}
}

static ssize_t ccd_debug_read(struct file *filp,
			      char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	char buf[256];
	u32  len = 0;

	len = snprintf(buf, sizeof(buf), "ccu_debug_read\n");

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t ccd_debug_write(struct file *filp,
			       const char __user *buffer,
			       size_t count, loff_t *data)
{
	char desc[64];
	s32 len = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	return count;
}

static int ccd_open(struct inode *inode, struct file *filp)
{
	struct mtk_ccd *ccd = container_of(inode->i_cdev, struct mtk_ccd, ccd_cdev);
	struct ccd_context *ctx = to_ccd_context(ccd);
	int ret = 0;

	/* cdev's parent pins rproc/private storage through the open/remove race. */
	down_read(&ctx->daemon_gate);
	if (READ_ONCE(ctx->removing))
		ret = -ENODEV;
	else {
		get_device(&ccd->rproc->dev);
		filp->private_data = ccd;
	}
	up_read(&ctx->daemon_gate);
	return ret;
}


static int ccd_release(struct inode *inode, struct file *filp)
{
	struct mtk_ccd *ccd = filp->private_data;
	struct ccd_context *ctx = to_ccd_context(ccd);
	struct ccd_master_status_item master_obj = { .state = CCD_MASTER_EXIT };

	down_read(&ctx->daemon_gate);
	if (!READ_ONCE(ctx->removing))
		ccd_master_destroy(ccd, &master_obj);
	up_read(&ctx->daemon_gate);
	put_device(&ccd->rproc->dev);
	return 0;
}


static long ccd_do_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct mtk_ccd *ccd = filp->private_data;
	void __user *user_addr = (void __user *)arg;
	struct ccd_master_listen_item listen_obj = {};
	struct ccd_worker_item work_obj = {};
	struct ccd_master_status_item master_obj = {};

	switch (cmd) {
	case IOCTL_CCD_MASTER_INIT:
		master_obj.state = CCD_MASTER_ACTIVE;
		ccd->master_status.state = CCD_MASTER_ACTIVE;
		return copy_to_user(user_addr, &master_obj, sizeof(master_obj)) ? -EFAULT : 0;
	case IOCTL_CCD_MASTER_DESTROY:
		if (copy_from_user(&master_obj, user_addr, sizeof(master_obj)))
			return -EFAULT;
		ccd->master_status.state = master_obj.state;
		return 0;
	case IOCTL_CCD_MASTER_LISTEN:
		ccd_master_listen(ccd, &listen_obj);
		return copy_to_user(user_addr, &listen_obj, sizeof(listen_obj)) ? -EFAULT : 0;
	case IOCTL_CCD_WORKER_READ:
		if (copy_from_user(&work_obj, user_addr, sizeof(work_obj)))
			return -EFAULT;
		ccd_worker_read(ccd, &work_obj);
		return copy_to_user(user_addr, &work_obj, sizeof(work_obj)) ? -EFAULT : 0;
	case IOCTL_CCD_WORKER_WRITE:
		if (copy_from_user(&work_obj, user_addr, sizeof(work_obj)))
			return -EFAULT;
		if (work_obj.len > sizeof(work_obj.sbuf))
			return -EINVAL;
		ccd_worker_write(ccd, &work_obj);
		return 0;
	default:
		return -ENOTTY;
	}
}

static long ccd_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct mtk_ccd *ccd = filp->private_data;
	struct ccd_context *ctx = to_ccd_context(ccd);
	long ret;

	down_read(&ctx->daemon_gate);
	if (READ_ONCE(ctx->removing))
		ret = -ENODEV;
	else {
		ret = ccd_do_ioctl(filp, cmd, arg);
		if (READ_ONCE(ctx->removing))
			ret = -ENODEV;
	}
	up_read(&ctx->daemon_gate);
	return ret;
}

/* Wake blocked ioctls before waiting for the gate; keep subdev/memory alive
 * until every entered ioctl and release callback has left its critical section.
 */
static void ccd_quiesce(struct mtk_ccd *ccd)
{
	struct ccd_context *ctx = to_ccd_context(ccd);

	WRITE_ONCE(ctx->removing, true);
	if (ccd->rpmsg_subdev)
		mtk_rpmsg_subdev_remove(ccd->rpmsg_subdev);
	down_write(&ctx->daemon_gate);
	up_write(&ctx->daemon_gate);
}

#ifdef CONFIG_COMPAT
static long ccd_ioctl_compat(struct file *filp,
			     unsigned int cmd,
			     unsigned long arg)
{
	long ret = 0;

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	ret = filp->f_op->unlocked_ioctl(filp,
					 cmd,
					 (unsigned long)compat_ptr(arg));

	return ret;
}
#endif

static const struct file_operations ccd_fops = {
	.owner = THIS_MODULE,
	.open = ccd_open,
	.release = ccd_release,
	.read = ccd_debug_read,
	.write = ccd_debug_write,
	.unlocked_ioctl = ccd_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ccd_ioctl_compat,
#endif
};

static int ccd_regcdev(struct mtk_ccd *ccd)
{
	int ret = 0;
	struct device *dev;

	ret = alloc_chrdev_region(&ccd->ccd_devno, 0, 1, CCD_DEV_NAME);
	if (ret < 0) {
		pr_debug("alloc_chrdev_region failed, %d\n", ret);
		return ret;
	}

	/* Attatch file operation. */
	cdev_init(&ccd->ccd_cdev, &ccd_fops);
	ccd->ccd_cdev.owner = THIS_MODULE;
	cdev_set_parent(&ccd->ccd_cdev, &ccd->rproc->dev.kobj);
	/* Add to system */
	ret = cdev_add(&ccd->ccd_cdev, ccd->ccd_devno, 1);
	if (ret < 0) {
		pr_debug("Attach file operation failed, %d\n", ret);
		goto err_cdev_add;
	}

	/* Create class register */
	ccd->ccd_class = class_create("mtk_ccd");
	if (IS_ERR(ccd->ccd_class)) {
		ret = PTR_ERR(ccd->ccd_class);
		pr_debug("Unable to create class, err = %d\n", ret);
		goto err_class_create;
	}

	dev = device_create(ccd->ccd_class, NULL,
			    ccd->ccd_devno, NULL,
			    CCD_DEV_NAME);
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_debug("Failed to create device: /dev/%s, err = %d\n",
		       CCD_DEV_NAME,
		       ret);
		goto err_device_create;
	}

	return ret;

err_device_create:
	class_destroy(ccd->ccd_class);
	ccd->ccd_class = NULL;
err_class_create:
	cdev_del(&ccd->ccd_cdev);
err_cdev_add:
	unregister_chrdev_region(ccd->ccd_devno, 1);
	return ret;
}

static void ccd_unregcdev(struct mtk_ccd *ccd)
{
	/* Release char driver */
	if (ccd->ccd_class) {
		device_destroy(ccd->ccd_class, ccd->ccd_devno);
		class_destroy(ccd->ccd_class);
		ccd->ccd_class = NULL;
	}

	cdev_del(&ccd->ccd_cdev);
	unregister_chrdev_region(ccd->ccd_devno, 1);
}

static int ccd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_ccd *ccd;
	struct rproc *rproc;
	struct resource *res;
	unsigned int i;
	int ret;

	rproc = rproc_alloc(dev, dev->of_node->name, &ccd_ops, "remoteproc_scp", sizeof(struct ccd_context));
	if (!rproc)
		return -ENOMEM;
	/* The CCD daemon owns execution; attach/detach only manages its message bus. */
	rproc->auto_boot = false;
	rproc->state = RPROC_DETACHED;
	ccd = rproc->priv;
	init_rwsem(&to_ccd_context(ccd)->daemon_gate);
	ccd->rproc = rproc;
	ccd->dev = dev;
	platform_set_drvdata(pdev, ccd);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
	if (ret)
		goto free_rproc;
	if (!dev->dma_parms) {
		dev->dma_parms = devm_kzalloc(dev, sizeof(*dev->dma_parms), GFP_KERNEL);
		if (!dev->dma_parms) {
			ret = -ENOMEM;
			goto free_rproc;
		}
	}
	dma_set_max_seg_size(dev, UINT_MAX);
	for (i = 0; i < CCD_MAP_HW_REG_NUM; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			continue;
		ccd->map_base[i].base = res->start;
		ccd->map_base[i].len = resource_size(res);
	}
	ccd->ccd_memory = mtk_ccd_mem_init(dev);
	if (!ccd->ccd_memory) {
		ret = -ENOMEM;
		goto free_rproc;
	}
	ccd_add_rpmsg_subdev(ccd);
	if (!ccd->rpmsg_subdev) {
		ret = -ENOMEM;
		goto free_memory;
	}
ret = rproc_add(rproc);
	if (ret)
		goto remove_subdev;
	ret = ccd_regcdev(ccd);
	if (ret) {
		/* cdev_add may have briefly exposed callbacks even on failure. */
		ccd_quiesce(ccd);
		rproc_del(rproc);
		goto remove_subdev;
	}
	return 0;

remove_subdev:
	ccd_remove_rpmsg_subdev(ccd);
free_memory:
	mtk_ccd_mem_release(ccd);
free_rproc:
	platform_set_drvdata(pdev, NULL);
	rproc_free(rproc);
	return ret;
}

static void ccd_remove(struct platform_device *pdev)
{
	struct mtk_ccd *ccd = platform_get_drvdata(pdev);

	ccd_quiesce(ccd);
	ccd_unregcdev(ccd);
	if (ccd->rproc->state == RPROC_ATTACHED)
		rproc_detach(ccd->rproc);
	rproc_del(ccd->rproc);
	ccd_remove_rpmsg_subdev(ccd);
	mtk_ccd_mem_release(ccd);
	platform_set_drvdata(pdev, NULL);
	rproc_free(ccd->rproc);
}

static const struct of_device_id mtk_ccd_of_match[] = {
	{ .compatible = "mediatek,ccd"},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_ccd_of_match);

static struct platform_driver mtk_ccd_driver = {
	.probe = ccd_probe,
	.remove = ccd_remove,
	.driver = {
		.name = CCD_DEV_NAME,
		.of_match_table = of_match_ptr(mtk_ccd_of_match),
	},
};

static int __init ccd_init(void)
{
	int ret;

	ret = platform_driver_register(&mtk_ccd_driver);
	return ret;
}

static void __exit ccd_exit(void)
{
	platform_driver_unregister(&mtk_ccd_driver);
}

late_initcall(ccd_init);
module_exit(ccd_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek CCD driver");
