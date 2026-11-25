// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/of_graph.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-common.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "mtk-composite.h"

/******************************************************************************
 * Definition
 *****************************************************************************/

#define MTK_COMPOSITE_NAME "mtk-composite-v4l2-2"

static int
fl_async_bound(struct v4l2_async_notifier *notifier,
		struct v4l2_subdev *subdev,
		struct v4l2_async_connection *asc)
{
	struct mtk_composite_v4l2_device *pfdev = container_of(notifier,
			struct mtk_composite_v4l2_device, notifier);
	unsigned int i;

	for (i = 0; i < pfdev->num_subdevs; i++) {
		if (pfdev->asd[i] == asc) {
			pfdev->subdevs[i] = subdev;
			return 0;
		}
	}
	return -EINVAL;
}

static int fl_probe_complete(struct mtk_composite_v4l2_device *pfdev)
{
	int err;
	struct v4l2_subdev *sd;

	pr_info("%s\n", __func__);
	if (!pfdev->num_subdevs || !pfdev->subdevs[0])
		return -ENODEV;
	/* set first sub device as current one */
	pfdev->v4l2_dev.ctrl_handler = pfdev->subdevs[0]->ctrl_handler;

	err = v4l2_device_register_subdev_nodes(&pfdev->v4l2_dev);
	if (err) {
		pr_info("Unable to v4l2_device_register_subdev_nodes\n");
		goto probe_out;
	}

	list_for_each_entry(sd, &pfdev->v4l2_dev.subdevs, list) {
		if (!(sd->flags & V4L2_SUBDEV_FL_HAS_DEVNODE))
			continue;

#if defined(CONFIG_MEDIA_CONTROLLER)
		pr_info("%s v4l2:%s\n", __func__, sd->entity.name);
#endif
	}

	pr_debug("%s -\n", __func__);
	return 0;

probe_out:
	return err;
}

static int fl_async_complete(struct v4l2_async_notifier *notifier)
{
	struct mtk_composite_v4l2_device *pfdev =
		container_of(notifier->v4l2_dev,
			struct mtk_composite_v4l2_device, v4l2_dev);

	return fl_probe_complete(pfdev);
}


static int mtk_get_pdata(struct platform_device *pdev,
		struct mtk_composite_v4l2_device *pfdev)
{
	struct device_node *endpoint = NULL;
	struct device_node *remote;
	struct v4l2_async_connection *asc;
	unsigned int count = 0;
	int ret = 0;

	if (!IS_ENABLED(CONFIG_OF) || !pdev->dev.of_node)
		return -EINVAL;
	while ((endpoint = of_graph_get_next_endpoint(pdev->dev.of_node,
				endpoint))) {
		if (count == ARRAY_SIZE(pfdev->asd)) {
			ret = -E2BIG;
			break;
		}
		remote = of_graph_get_remote_port_parent(endpoint);
		if (!remote) {
			ret = -ENODEV;
			break;
		}
		asc = v4l2_async_nf_add_fwnode(&pfdev->notifier,
				of_fwnode_handle(remote), struct v4l2_async_connection);
		/* Async core owns its own reference after successful add. */
		of_node_put(remote);
		if (IS_ERR(asc)) {
			ret = PTR_ERR(asc);
			break;
		}
		pfdev->asd[count++] = asc;
		pfdev->num_subdevs = count;
	}
	of_node_put(endpoint);
	if (ret)
		return ret;
	return count ? 0 : -ENODEV;
}

static void mtk_composite_unregister_entities(
		struct mtk_composite_v4l2_device *pfdev)
{
	v4l2_device_unregister(&pfdev->v4l2_dev);
	media_device_unregister(&pfdev->media_dev);
	media_entity_cleanup(&pfdev->vdev->entity);
	media_device_cleanup(&pfdev->media_dev);
	video_device_release(pfdev->vdev);
}

static void fl_async_unbind(struct v4l2_async_notifier *notifier,
		struct v4l2_subdev *subdev,
		struct v4l2_async_connection *asc)
{
	struct mtk_composite_v4l2_device *pfdev = container_of(notifier,
			struct mtk_composite_v4l2_device, notifier);
	unsigned int i;

	for (i = 0; i < pfdev->num_subdevs; i++) {
		if (pfdev->asd[i] == asc) {
			pfdev->subdevs[i] = NULL;
			if (!i)
				pfdev->v4l2_dev.ctrl_handler = NULL;
			break;
		}
	}
}

static const struct v4l2_async_notifier_operations fl_async_notify_ops = {
	.bound = fl_async_bound,
	.unbind = fl_async_unbind,
	.complete = fl_async_complete,
};

static int mtk_composite_probe(struct platform_device *dev)
{
	struct mtk_composite_v4l2_device *pfdev;
	int ret;

	pfdev = devm_kzalloc(&dev->dev, sizeof(*pfdev), GFP_KERNEL);
	if (!pfdev)
		return -ENOMEM;
	pfdev->vdev = video_device_alloc();
	if (!pfdev->vdev)
		return -ENOMEM;
	pfdev->v4l2_dev.mdev = &pfdev->media_dev;
	strscpy(pfdev->media_dev.model, "mtk_camera_af",
			sizeof(pfdev->media_dev.model));
	pfdev->media_dev.dev = &dev->dev;
	media_device_init(&pfdev->media_dev);
	ret = media_entity_pads_init(&pfdev->vdev->entity, 0, NULL);
	if (ret)
		goto err_media_cleanup;
	pfdev->vdev->entity.function = MEDIA_ENT_F_IO_V4L;
	ret = media_device_register(&pfdev->media_dev);
	if (ret)
		goto err_entity;
	ret = v4l2_device_register(&dev->dev, &pfdev->v4l2_dev);
	if (ret)
		goto err_media_unregister;
	v4l2_async_nf_init(&pfdev->notifier, &pfdev->v4l2_dev);
	pfdev->notifier.ops = &fl_async_notify_ops;
	ret = mtk_get_pdata(dev, pfdev);
	if (ret)
		goto err_notifier;
	ret = v4l2_async_nf_register(&pfdev->notifier);
	if (ret)
		goto err_notifier;
	platform_set_drvdata(dev, pfdev);
	__module_get(THIS_MODULE);
	return 0;

err_notifier:
	v4l2_async_nf_cleanup(&pfdev->notifier);
	v4l2_device_unregister(&pfdev->v4l2_dev);
err_media_unregister:
	media_device_unregister(&pfdev->media_dev);
err_entity:
	media_entity_cleanup(&pfdev->vdev->entity);
err_media_cleanup:
	media_device_cleanup(&pfdev->media_dev);
	video_device_release(pfdev->vdev);
	return ret;
}

static void mtk_composite_remove(struct platform_device *dev)
{
	struct mtk_composite_v4l2_device *pfdev = platform_get_drvdata(dev);

	v4l2_async_nf_unregister(&pfdev->notifier);
	v4l2_async_nf_cleanup(&pfdev->notifier);
	mtk_composite_unregister_entities(pfdev);
	platform_set_drvdata(dev, NULL);

	return;
}

static void mtk_composite_shutdown(struct platform_device *dev)
{

}

#ifdef CONFIG_OF
static const struct of_device_id mtk_composite_of_match[] = {
	{.compatible = "mediatek,mtk_composite_v4l2_2"},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_composite_of_match);

#else
static struct platform_device mtk_composite_platform_device[] = {
	{
		.name = MTK_COMPOSITE_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, mtk_composite_platform_device);
#endif

static struct platform_driver mtk_composite_platform_driver = {
	.probe = mtk_composite_probe,
	.remove = mtk_composite_remove,
	.shutdown = mtk_composite_shutdown,
	.driver = {
		   .name = MTK_COMPOSITE_NAME,
		   .owner = THIS_MODULE,
		   .suppress_bind_attrs = true,
#ifdef CONFIG_OF
		   .of_match_table = mtk_composite_of_match,
#endif
	},
};

static int __init mtk_composite_init(void)
{
	int ret;

	pr_debug("Init start\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&mtk_composite_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&mtk_composite_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done\n");

	return 0;
}

static void __exit mtk_composite_exit(void)
{
	platform_driver_unregister(&mtk_composite_platform_driver);
#ifndef CONFIG_OF
	platform_device_unregister(&mtk_composite_platform_device);
#endif
}

late_initcall(mtk_composite_init);
module_exit(mtk_composite_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MTK V4L2 Composite Driver");
