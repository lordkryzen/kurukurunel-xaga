// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 MediaTek Inc.
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/module.h>
#include <linux/compat.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "widevine_driver.h"
#include "mtk_heap.h"

static dev_t wv_devt;
static struct cdev *wv_cdev;
static struct class *wv_class;
static struct device *wv_device;
static DEFINE_MUTEX(wv_lifecycle_lock);
static bool wv_number_owned, wv_cdev_added;

int wv_dbg_level;
module_param(wv_dbg_level, int, 0644);

/******open file function********/
int wv_open(struct inode *inode, struct file *filep)
{
	WV_LOG(1, "filep->f_op = %p\n", filep->f_op);
	return 0;
}

/**close file***/
int wv_release(struct inode *inode, struct file *filep)
{
	WV_LOG(1, "filep->f_op = %p\n", filep->f_op);
	return 0;
}

/*******IO control*********/
static long wv_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
 struct WV_FD_TO_SEC_HANDLE wide = {0};
 struct WV_FD_TO_SEC_HANDLE32 native = {0};
 struct dma_buf *dmabuf;
 uint64_t handle;
 bool narrow;
 int fd, ret = 0;

 if (cmd == WV_IOC_GET_SEC_HANDLE32) {
  narrow = true;
  if (copy_from_user(&native, (void __user *)arg, sizeof(native)))
   return -EFAULT;
  fd = native.share_fd;
 } else if (cmd == WV_IOC_GET_SEC_HANDLE) {
  narrow = false;
  if (copy_from_user(&wide, (void __user *)arg, sizeof(wide)))
   return -EFAULT;
  fd = wide.share_fd;
 } else {
  return -EINVAL;
 }
 dmabuf = dma_buf_get(fd);
 if (IS_ERR_OR_NULL(dmabuf))
  return -EFAULT;
 handle = dmabuf_to_secure_handle(dmabuf);
 if (!handle) {
  ret = -EFAULT;
 } else if (narrow) {
  if (handle > U32_MAX)
   ret = -EOVERFLOW;
  else {
   native.sec_handle = (uint32_t)handle;
   if (copy_to_user((void __user *)arg, &native, sizeof(native)))
    ret = -EFAULT;
  }
 } else {
  wide.sec_handle = handle;
  if (copy_to_user((void __user *)arg, &wide, sizeof(wide)))
   ret = -EFAULT;
 }
 dma_buf_put(dmabuf);
 return ret;
}

#if IS_ENABLED(CONFIG_COMPAT)
static long compat_wv_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	WV_LOG(1, "use compatible ioctl\n");
	return wv_ioctl(filep, cmd, (unsigned long)compat_ptr(arg));
}
#endif

/*****file operation structure********/
static const struct file_operations wv_fops = {
	.owner = THIS_MODULE,
	.open  = wv_open,
	.release = wv_release,
	.unlocked_ioctl = wv_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl = compat_wv_ioctl,
#endif
};

/* Both late probe failures and remove retire only acquired resources. */
static void wv_retire(void)
{
 if (wv_device) {
  device_destroy(wv_class, wv_devt);
  wv_device = NULL;
 }
 if (wv_class) {
  class_destroy(wv_class);
  wv_class = NULL;
 }
 if (wv_cdev) {
  if (wv_cdev_added)
   cdev_del(wv_cdev);
  else
   kobject_put(&wv_cdev->kobj);
  wv_cdev = NULL;
  wv_cdev_added = false;
 }
 if (wv_number_owned) {
  unregister_chrdev_region(wv_devt, 1);
  wv_number_owned = false;
 }
}

static int wv_probe(struct platform_device *pdev)
{
 int ret;
 mutex_lock(&wv_lifecycle_lock);
 if (wv_number_owned) { ret = -EBUSY; goto unlock; }
 ret = alloc_chrdev_region(&wv_devt, 0, 1, WV_DEVNAME);
 if (ret) goto unlock;
 wv_number_owned = true;
 wv_cdev = cdev_alloc();
 if (!wv_cdev) { ret = -ENOMEM; goto fail; }
 /* cdev_alloc supplies the dynamic kobject release. cdev_init would replace
  * it with the embedded/static cdev lifetime and leak this allocation. */
 wv_cdev->owner = THIS_MODULE;
 wv_cdev->ops = &wv_fops;
 ret = cdev_add(wv_cdev, wv_devt, 1);
 if (ret) goto fail;
 wv_cdev_added = true;
 wv_class = class_create(WV_DEVNAME);
 if (IS_ERR(wv_class)) { ret = PTR_ERR(wv_class); wv_class = NULL; goto fail; }
 wv_device = device_create(wv_class, &pdev->dev, wv_devt, NULL, WV_DEVNAME);
 if (IS_ERR(wv_device)) { ret = PTR_ERR(wv_device); wv_device = NULL; goto fail; }
 mutex_unlock(&wv_lifecycle_lock);
 return 0;
fail:
 wv_retire();
unlock:
 mutex_unlock(&wv_lifecycle_lock);
 return ret;
}

static void wv_remove(struct platform_device *pdev)
{
 mutex_lock(&wv_lifecycle_lock);
 wv_retire();
 mutex_unlock(&wv_lifecycle_lock);
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id wv_of_match[] = {
	{.compatible = "mediatek,drm_wv",},
	{}
};
#endif

MODULE_DEVICE_TABLE(of, wv_of_match);

static struct platform_driver mtk_wv_driver = {
	.probe = wv_probe,
	.remove = wv_remove,
	.driver = {
		.name = WV_DEVNAME,
		.owner  = THIS_MODULE,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = wv_of_match,
#endif
	},
};

static int __init mtk_wv_init(void)
{
	WV_LOG(0, "Register the widevine driver\n");

	if (platform_driver_register(&mtk_wv_driver)) {
		WV_LOG(0, "failed to register widevine device\n");
		return -ENODEV;
	}

	return 0;
}

/*uninstall device*/
static void __exit mtk_wv_exit(void)
{
	platform_driver_unregister(&mtk_wv_driver);
	WV_LOG(0, "driver exit successful\n");
}

module_init(mtk_wv_init);
module_exit(mtk_wv_exit);
MODULE_AUTHOR("Shan Zhang <shan.zhang@mediatek.com>");
MODULE_DESCRIPTION("Widevine Drm Driver");
MODULE_IMPORT_NS(DMA_BUF);
MODULE_LICENSE("GPL");
