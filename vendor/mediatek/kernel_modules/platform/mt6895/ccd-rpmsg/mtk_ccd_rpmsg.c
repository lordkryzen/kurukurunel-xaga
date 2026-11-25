// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2018 MediaTek Inc.

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include <uapi/linux/mtk_ccd_controls.h>
#include <linux/rpmsg/mtk_ccd_rpmsg.h>
#include <linux/rpmsg/mtk_ccd_client.h>
#include <linux/platform_data/mtk_ccd.h>

#include "rpmsg_internal.h"
#include "mtk_ccd_rpmsg_internal.h"

static const struct rpmsg_endpoint_ops mtk_rpmsg_endpoint_ops;
void ccd_msgdev_exit(void);

static void ccd_bus_release(struct kref *ref)
{
	struct mtk_ccd_bus *bus = container_of(ref, struct mtk_ccd_bus, refcount);

	idr_destroy(&bus->subdev.endpoints);
	put_device(&bus->subdev.pdev->dev);
	kfree(bus);
}

/* Registry removal and endpoint unpublication share this lock.  The endpoint
 * itself pins its device, which pins the bus until the final worker returns.
 */
struct rpmsg_endpoint *mtk_ccd_endpoint_get(struct mtk_rpmsg_rproc_subdev *subdev,
					 u32 src)
{
	struct mtk_rpmsg_device *mdev;
	struct rpmsg_endpoint *ept = NULL;

	mutex_lock(&subdev->endpoints_lock);
	mdev = idr_find(&subdev->endpoints, src);
	if (mdev) {
		ept = to_ccd_device(mdev)->endpoint;
		if (ept)
			kref_get(&ept->refcount);
	}
	mutex_unlock(&subdev->endpoints_lock);
	return ept;
}

void __ept_release(struct kref *kref)
{
	struct rpmsg_endpoint *ept = container_of(kref, struct rpmsg_endpoint,
						  refcount);
	struct mtk_ccd_rpmsg_endpoint *mept = to_mtk_rpmsg_endpoint(ept);
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = mept->mtk_subdev;
	struct rpmsg_device *rpdev = ept->rpdev;

	dev_info(&mtk_subdev->pdev->dev, "free mtk rpmsg endpoint: %p\n",
		 mept);
	kfree(to_mtk_rpmsg_endpoint(ept));

	put_device(&rpdev->dev);
}

void mtk_rpmsg_ipi_handler(void *data, unsigned int len, void *priv)
{
	struct mtk_ccd_rpmsg_endpoint *mept = priv;
	struct rpmsg_endpoint *ept = &mept->ept;

	/* The caller owns an endpoint reference for the duration of this call. */
	mutex_lock(&ept->cb_lock);
	if (ept->cb && atomic_read(&mept->ccd_mep_state) == CCD_MENDPOINT_CREATED)
		ept->cb(ept->rpdev, data, len, ept->priv, ept->addr);
	mutex_unlock(&ept->cb_lock);
}


static struct rpmsg_endpoint *
__rpmsg_create_ept(struct mtk_rpmsg_rproc_subdev *mtk_subdev,
		   struct rpmsg_device *rpdev, rpmsg_rx_cb_t cb, void *priv,
		   u32 id)
{
	struct mtk_ccd_rpmsg_endpoint *mept;
	struct rpmsg_endpoint *ept;
	struct platform_device *pdev = mtk_subdev->pdev;

	mept = kzalloc(sizeof(*mept), GFP_KERNEL);
	if (!mept)
		return NULL;
	mept->mtk_subdev = mtk_subdev;

	ept = &mept->ept;
	kref_init(&ept->refcount);
	mutex_init(&ept->cb_lock);

	get_device(&rpdev->dev);
	ept->rpdev = rpdev;
	ept->cb = cb;
	ept->priv = priv;
	ept->ops = &mtk_rpmsg_endpoint_ops;
	ept->addr = id;

	mept->mchinfo.chinfo.src = id;
	mept->mchinfo.chinfo.dst = RPMSG_ADDR_ANY;
	mept->mchinfo.id = id;

	INIT_LIST_HEAD(&mept->pending_sendq.queue);
	spin_lock_init(&mept->pending_sendq.queue_lock);
	init_waitqueue_head(&mept->worker_readwq);
	init_waitqueue_head(&mept->ccd_paramswq);
	atomic_set(&mept->ccd_cmd_sent, 0);
	atomic_set(&mept->worker_read_rdy, 0);
	atomic_set(&mept->ccd_params_rdy, 0);
	atomic_set(&mept->ccd_mep_state, CCD_MENDPOINT_CREATED);

	dev_dbg(&pdev->dev, "%s: %d\n", __func__, ept->addr);
	return ept;
}

static struct rpmsg_endpoint *
mtk_rpmsg_create_ept(struct rpmsg_device *rpdev, rpmsg_rx_cb_t cb, void *priv,
		     struct rpmsg_channel_info chinfo)
{
	struct mtk_ccd_rpmsg_endpoint *mept;
	struct mtk_rpmsg_rproc_subdev *mtk_subdev =
		to_mtk_rpmsg_device(rpdev)->mtk_subdev;
	struct rpmsg_endpoint *ept =
		__rpmsg_create_ept(mtk_subdev, rpdev, cb, priv, chinfo.src);

	if (!ept)
		return NULL;

	mept = to_mtk_rpmsg_endpoint(ept);
	memcpy(mept->mchinfo.chinfo.name, chinfo.name, RPMSG_NAME_SIZE);
	mutex_lock(&mtk_subdev->endpoints_lock);
	if (to_ccd_bus(mtk_subdev)->stopping ||
	    to_ccd_bus(mtk_subdev)->resetting ||
	    to_ccd_device(to_mtk_rpmsg_device(rpdev))->removing ||
	    to_ccd_device(to_mtk_rpmsg_device(rpdev))->endpoint) {
		mutex_unlock(&mtk_subdev->endpoints_lock);
		kref_put(&ept->refcount, __ept_release);
		return NULL;
	}
	to_ccd_device(to_mtk_rpmsg_device(rpdev))->endpoint = ept;
	mutex_unlock(&mtk_subdev->endpoints_lock);
	return ept;
}

static void mtk_rpmsg_destroy_ept(struct rpmsg_endpoint *ept)
{
	struct mtk_ccd_params *params, *next;
	struct mtk_ccd_rpmsg_endpoint *mept = to_mtk_rpmsg_endpoint(ept);
	struct mtk_rpmsg_rproc_subdev *subdev = mept->mtk_subdev;
	struct mtk_ccd_device *device = to_ccd_device(to_mtk_rpmsg_device(ept->rpdev));
	LIST_HEAD(pending);

	/* Stop lookups before dropping the owner reference.  Do not remove the
	 * device's address here: a later driver bind may create a new endpoint.
	 */
	mutex_lock(&subdev->endpoints_lock);
	if (device->endpoint == ept)
		device->endpoint = NULL;
	if (ept->rpdev->ept == ept)
		ept->rpdev->ept = NULL;
	mutex_unlock(&subdev->endpoints_lock);

	/* Serialize admission with send and detach the entire queue at once. */
	spin_lock(&mept->pending_sendq.queue_lock);
	atomic_set(&mept->ccd_mep_state, CCD_MENDPOINT_DESTROY);
	list_splice_init(&mept->pending_sendq.queue, &pending);
	atomic_set(&mept->ccd_cmd_sent, 0);
	spin_unlock(&mept->pending_sendq.queue_lock);
	wake_up_all(&mept->worker_readwq);
	wake_up_all(&mept->ccd_paramswq);

	/* Preserve the daemon-failure notification contract for pending requests.
	 * No registry or queue lock may be held while invoking a client callback.
	 */
	mutex_lock(&ept->cb_lock);
	list_for_each_entry_safe(params, next, &pending, list_entry) {
		list_del(&params->list_entry);
		if (ept->cb)
			ept->cb(ept->rpdev, params->worker_obj.sbuf,
				params->worker_obj.len, ept->priv, ept->addr);
		kfree(params);
	}
	ept->cb = NULL;
	mutex_unlock(&ept->cb_lock);
	kref_put(&ept->refcount, __ept_release);
}


static int mtk_rpmsg_send(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct mtk_ccd_rpmsg_endpoint *mept = to_mtk_rpmsg_endpoint(ept);
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = mept->mtk_subdev;

	return mtk_subdev->ops->ccd_send(mtk_subdev, mept, data, len, 0);
}

static int mtk_rpmsg_trysend(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev =
		to_mtk_rpmsg_endpoint(ept)->mtk_subdev;
	struct mtk_ccd_rpmsg_endpoint *mept = to_mtk_rpmsg_endpoint(ept);

	/*
	 * TODO: This currently is same as mtk_rpmsg_send, and wait until SCP
	 * received the last command.
	 */
	return mtk_subdev->ops->ccd_send(mtk_subdev, mept, data, len, 0);
}

static const struct rpmsg_endpoint_ops mtk_rpmsg_endpoint_ops = {
	.destroy_ept = mtk_rpmsg_destroy_ept,
	.send = mtk_rpmsg_send,
	.trysend = mtk_rpmsg_trysend,
};

static void mtk_rpmsg_release_device(struct device *dev)
{
	struct mtk_rpmsg_device *mdev = to_mtk_rpmsg_device(to_rpmsg_device(dev));
	struct mtk_rpmsg_rproc_subdev *subdev = mdev->mtk_subdev;

	/* Registration failure also reaches here through the RPMsg core. */
	mutex_lock(&subdev->endpoints_lock);
	if (idr_find(&subdev->endpoints, mdev->rpdev.src) == mdev)
		idr_remove(&subdev->endpoints, mdev->rpdev.src);
	mutex_unlock(&subdev->endpoints_lock);
	of_node_put(dev->of_node);
	kfree(to_ccd_device(mdev));
	kref_put(&to_ccd_bus(subdev)->refcount, ccd_bus_release);
}


static const struct rpmsg_device_ops mtk_rpmsg_device_ops = {
	.create_ept = mtk_rpmsg_create_ept,
};

static struct device_node *
mtk_rpmsg_match_device_subnode(struct device_node *node, const char *channel)
{
	struct device_node *child;
	const char *name;
	int ret;

	for_each_available_child_of_node(node, child) {
		ret = of_property_read_string(child, "mtk,rpmsg-name", &name);
		if (ret)
			continue;

		if (strcmp(name, channel) == 0)
			return child;
	}
	return NULL;
}

static void ccd_unregister_device(struct mtk_rpmsg_device *mdev)
{
	struct mtk_rpmsg_rproc_subdev *subdev = mdev->mtk_subdev;
	struct rpmsg_device *rpdev = &mdev->rpdev;
	struct rpmsg_endpoint *ept;

	/* Unbound camera clients create their own endpoint; the RPMsg core only
	 * destroys default endpoints for bound drivers.  Serialize with binding.
	 */
	device_lock(&rpdev->dev);
	if (!rpdev->dev.driver) {
		mutex_lock(&subdev->endpoints_lock);
		ept = to_ccd_device(mdev)->endpoint;
		mutex_unlock(&subdev->endpoints_lock);
		if (ept)
			rpmsg_destroy_ept(ept);
	}
	device_unlock(&rpdev->dev);
	device_unregister(&rpdev->dev);
}

/* Select, notify and remove one pinned device at a time.  No allocation and
 * no borrowed channel pointers survive the lock or endpoint reference.
 */
void mtk_ccd_remove_clients(struct mtk_rpmsg_rproc_subdev *subdev, bool all)
{
	struct mtk_ccd_bus *bus = to_ccd_bus(subdev);
	struct mtk_rpmsg_device *mdev;
	struct rpmsg_endpoint *ept;
	int id = 0;

	mutex_lock(&bus->reset_lock);
	mutex_lock(&subdev->endpoints_lock);
	WRITE_ONCE(bus->resetting, true);
	mutex_unlock(&subdev->endpoints_lock);
	wake_up_all(&subdev->ccd_listen_wq);
	wait_event(bus->register_wq, !READ_ONCE(bus->registering));

	for (;;) {
		mutex_lock(&subdev->endpoints_lock);
		mdev = idr_get_next(&subdev->endpoints, &id);
		if (!mdev) {
			mutex_unlock(&subdev->endpoints_lock);
			break;
		}
		if (!all && id == MTK_CCD_MSGDEV_ADDR) {
			id++;
			mutex_unlock(&subdev->endpoints_lock);
			continue;
		}
		get_device(&mdev->rpdev.dev);
		ept = to_ccd_device(mdev)->endpoint;
		if (ept)
			kref_get(&ept->refcount);
		/* Claim unregister ownership before invoking a reentrant callback. */
		WRITE_ONCE(to_ccd_device(mdev)->removing, true);
		idr_remove(&subdev->endpoints, id);
		mutex_unlock(&subdev->endpoints_lock);
		if (ept) {
			mutex_lock(&ept->cb_lock);
			if (ept->cb && atomic_read(&to_mtk_rpmsg_endpoint(ept)->ccd_mep_state) ==
			    CCD_MENDPOINT_CREATED)
				ept->cb(ept->rpdev, NULL, 0, ept->priv, -1);
			mutex_unlock(&ept->cb_lock);
			kref_put(&ept->refcount, __ept_release);
		}
		ccd_unregister_device(mdev);
		put_device(&mdev->rpdev.dev);
		if (id == INT_MAX)
			break;
		id++;
	}
	mutex_lock(&subdev->master_listen_lock);
	atomic_set(&subdev->listen_obj_rdy, CCD_LISTEN_OBJECT_PREPARING);
	mutex_lock(&subdev->endpoints_lock);
	WRITE_ONCE(bus->resetting, false);
	mutex_unlock(&subdev->endpoints_lock);
	mutex_unlock(&subdev->master_listen_lock);
	wake_up_all(&subdev->ccd_listen_wq);
	mutex_unlock(&bus->reset_lock);
}

int
mtk_rpmsg_destroy_rpmsgdev(struct mtk_rpmsg_rproc_subdev *subdev,
			 struct rpmsg_channel_info *info)
{
	struct mtk_rpmsg_device *mdev;
	struct rpmsg_device *rpdev;

	/* Claim exact ownership once, rather than finding the device twice by
	 * name.  A second remover cannot unregister it or destroy its ept again.
	 */
	mutex_lock(&subdev->endpoints_lock);
	mdev = idr_find(&subdev->endpoints, info->src);
	if (!mdev || strncmp(mdev->rpdev.id.name, info->name, RPMSG_NAME_SIZE) ||
	    (info->dst != RPMSG_ADDR_ANY && mdev->rpdev.dst != info->dst)) {
		mutex_unlock(&subdev->endpoints_lock);
		return -ENOENT;
	}
	WRITE_ONCE(to_ccd_device(mdev)->removing, true);
	idr_remove(&subdev->endpoints, info->src);
	rpdev = &mdev->rpdev;
	get_device(&rpdev->dev);
	mutex_unlock(&subdev->endpoints_lock);

	ccd_unregister_device(mdev);
	put_device(&rpdev->dev);
	return 0;
}


int mtk_ccd_create_client_endpoint(struct mtk_rpmsg_device *mdev,
				 rpmsg_rx_cb_t cb, void *priv,
				 struct rpmsg_channel_info info)
{
	struct rpmsg_device *rpdev = &mdev->rpdev;
	struct rpmsg_endpoint *ept;
	int ret = 0;

	/* Unregister uses the same device lock for manually owned endpoints. */
	device_lock(&rpdev->dev);
	if (rpdev->dev.driver) {
		ret = -EBUSY;
		goto out;
	}
	ept = mtk_rpmsg_create_ept(rpdev, cb, priv, info);
	if (!ept) {
		ret = -ENOMEM;
		goto out;
	}
	rpdev->ept = ept;
out:
	device_unlock(&rpdev->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_ccd_create_client_endpoint);

int mtk_ccd_send_client(struct mtk_rpmsg_device *mdev, void *data, int len)
{
	struct mtk_rpmsg_rproc_subdev *subdev;
	struct rpmsg_endpoint *ept = NULL;
	int ret;

	if (!mdev)
		return -ENODEV;
	subdev = mdev->mtk_subdev;
	mutex_lock(&subdev->endpoints_lock);
	if (!to_ccd_device(mdev)->removing &&
	    !to_ccd_bus(subdev)->stopping && !to_ccd_bus(subdev)->resetting) {
		ept = to_ccd_device(mdev)->endpoint;
		if (ept)
			kref_get(&ept->refcount);
	}
	mutex_unlock(&subdev->endpoints_lock);
	if (!ept)
		return -ESHUTDOWN;
	ret = rpmsg_send(ept, data, len);
	kref_put(&ept->refcount, __ept_release);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_ccd_send_client);

int mtk_ccd_destroy_client(struct mtk_rpmsg_device *mdev)
{
	struct mtk_rpmsg_rproc_subdev *subdev;
	struct rpmsg_device *rpdev;
	bool claimed = false;
	int ret = -ENOENT;

	if (!mdev)
		return 0;
	subdev = mdev->mtk_subdev;
	rpdev = &mdev->rpdev;
	mutex_lock(&subdev->master_listen_lock);
	if (atomic_read(&subdev->listen_obj_rdy) == CCD_LISTEN_OBJECT_READY) {
		mutex_unlock(&subdev->master_listen_lock);
		wait_event_interruptible_timeout(subdev->ccd_listen_wq,
			atomic_read(&subdev->listen_obj_rdy) == CCD_LISTEN_OBJECT_PREPARING ||
			READ_ONCE(to_ccd_bus(subdev)->stopping) ||
			READ_ONCE(to_ccd_device(mdev)->removing), msecs_to_jiffies(400));
		mutex_lock(&subdev->master_listen_lock);
	}
	/* Claim this exact object. A reset may already have recycled its address. */
	mutex_lock(&subdev->endpoints_lock);
	if (idr_find(&subdev->endpoints, rpdev->src) == mdev) {
		WRITE_ONCE(to_ccd_device(mdev)->removing, true);
		idr_remove(&subdev->endpoints, rpdev->src);
		claimed = true;
	}
	mutex_unlock(&subdev->endpoints_lock);
	if (claimed && !READ_ONCE(to_ccd_bus(subdev)->stopping) &&
	    !READ_ONCE(to_ccd_bus(subdev)->resetting)) {
		memcpy(subdev->listen_obj.name, rpdev->id.name, RPMSG_NAME_SIZE);
		subdev->listen_obj.src = rpdev->src;
		subdev->listen_obj.cmd = CCD_MASTER_CMD_DESTROY;
		atomic_set(&subdev->listen_obj_rdy, CCD_LISTEN_OBJECT_READY);
		wake_up_all(&subdev->master_listen_wq);
	}
	mutex_unlock(&subdev->master_listen_lock);
	if (claimed) {
		ccd_unregister_device(mdev);
		ret = 0;
	}
	put_device(&rpdev->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_ccd_destroy_client);

int
mtk_destroy_client_msgdevice(struct rproc_subdev *subdev,
			     struct rpmsg_channel_info *info)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = to_mtk_subdev(subdev);
	struct device *dev = rpmsg_find_device(&mtk_subdev->pdev->dev, info);

	if (!dev)
		return -ENOENT;
	/* Consume the reference returned by rpmsg_find_device(). */
	return mtk_ccd_destroy_client(to_mtk_rpmsg_device(to_rpmsg_device(dev)));
}
EXPORT_SYMBOL_GPL(mtk_destroy_client_msgdevice);

static struct mtk_rpmsg_device *
mtk_rpmsg_create_rpmsgdev(struct mtk_rpmsg_rproc_subdev *subdev,
			struct rpmsg_channel_info *info)
{
	struct mtk_ccd_bus *bus = to_ccd_bus(subdev);
	struct mtk_ccd_device *device;
	struct mtk_rpmsg_device *mdev;
	struct rpmsg_device *rpdev;
	int id, ret;
	bool stopping;

	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device)
		return NULL;
	mdev = &device->mdev;
	rpdev = &mdev->rpdev;
	mdev->mtk_subdev = subdev;
	rpdev->ops = &mtk_rpmsg_device_ops;
	rpdev->dst = info->dst;
	memcpy(rpdev->id.name, info->name, RPMSG_NAME_SIZE);
	rpdev->dev.parent = &subdev->pdev->dev;
	rpdev->dev.release = mtk_rpmsg_release_device;

	mutex_lock(&subdev->endpoints_lock);
	if (bus->stopping || bus->resetting) {
		mutex_unlock(&subdev->endpoints_lock);
		kfree(device);
		return NULL;
	}
	/* Reserve an address, but never expose an uninitialized device. */
	if (info->src != RPMSG_ADDR_ANY && info->src > INT_MAX - 5)
		id = -EINVAL;
	else
		id = idr_alloc(&subdev->endpoints, NULL,
			info->src == RPMSG_ADDR_ANY ? MTK_CCD_MSGDEV_ADDR + 1 : info->src,
			info->src == RPMSG_ADDR_ANY ? 0 : info->src + 5, GFP_KERNEL);
	if (id < 0) {
		mutex_unlock(&subdev->endpoints_lock);
		kfree(device);
		return NULL;
	}
	bus->registering++;
	/* One device reference and one creator reference, including failures. */
	kref_get(&bus->refcount);
	kref_get(&bus->refcount);
	rpdev->src = id;
	mutex_unlock(&subdev->endpoints_lock);
	rpdev->dev.of_node = mtk_rpmsg_match_device_subnode(subdev->pdev->dev.of_node,
							 info->name);
	ret = rpmsg_register_device(rpdev);
	/* On failure the core has already called release; do not touch mdev. */
	mutex_lock(&subdev->endpoints_lock);
	stopping = bus->stopping || bus->resetting;
	if (ret)
		idr_remove(&subdev->endpoints, id);
	else {
		get_device(&rpdev->dev);
		idr_replace(&subdev->endpoints, mdev, id);
	}
	bus->registering--;
	wake_up_all(&bus->register_wq);
	mutex_unlock(&subdev->endpoints_lock);
	kref_put(&bus->refcount, ccd_bus_release);
	/* The bus owner waits for creators before unregistering the registry. */
	if (!ret && stopping)
		put_device(&rpdev->dev);
	return ret || stopping ? NULL : mdev;
}


static int
mtk_rpmsg_create_ccd_rpmsgdev(struct mtk_rpmsg_rproc_subdev *mtk_subdev,
			      struct rpmsg_channel_info *info)
{
	struct mtk_rpmsg_device *mdev =
		mtk_rpmsg_create_rpmsgdev(mtk_subdev, info);

	if (!mdev)
		return -ENOMEM;

	mtk_subdev->rpdev = &mdev->rpdev;
	put_device(&mdev->rpdev.dev);
	return 0;
}

struct mtk_rpmsg_device *
mtk_create_client_msgdevice(struct rproc_subdev *subdev,
			    struct rpmsg_channel_info *info)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = to_mtk_subdev(subdev);
	struct mtk_ccd_bus *bus = to_ccd_bus(mtk_subdev);
	struct mtk_rpmsg_device *mdev = mtk_rpmsg_create_rpmsgdev(mtk_subdev, info);
	struct rpmsg_channel_info channel;
	bool failed = false;
	int ret;

	if (!mdev)
		return NULL;
	/* The creator reference covers all blocking mailbox and error paths. */
	memcpy(channel.name, mdev->rpdev.id.name, RPMSG_NAME_SIZE);
	channel.src = mdev->rpdev.src;
	channel.dst = mdev->rpdev.dst;
	mutex_lock(&mtk_subdev->master_listen_lock);
	while (atomic_read(&mtk_subdev->listen_obj_rdy) == CCD_LISTEN_OBJECT_READY &&
	       !READ_ONCE(bus->stopping) && !READ_ONCE(bus->resetting) &&
	       !READ_ONCE(to_ccd_device(mdev)->removing)) {
		mutex_unlock(&mtk_subdev->master_listen_lock);
		ret = wait_event_interruptible(mtk_subdev->ccd_listen_wq,
			atomic_read(&mtk_subdev->listen_obj_rdy) == CCD_LISTEN_OBJECT_PREPARING ||
			READ_ONCE(bus->stopping) || READ_ONCE(bus->resetting) ||
			READ_ONCE(to_ccd_device(mdev)->removing));
		mutex_lock(&mtk_subdev->master_listen_lock);
		if (ret) {
			failed = true;
			break;
		}
	}
	mutex_lock(&mtk_subdev->endpoints_lock);
	if (bus->stopping || bus->resetting ||
	    idr_find(&mtk_subdev->endpoints, channel.src) != mdev)
		failed = true;
	mutex_unlock(&mtk_subdev->endpoints_lock);
	if (!failed) {
		memcpy(mtk_subdev->listen_obj.name, channel.name, RPMSG_NAME_SIZE);
		mtk_subdev->listen_obj.src = channel.src;
		mtk_subdev->listen_obj.cmd = CCD_MASTER_CMD_CREATE;
		atomic_set(&mtk_subdev->listen_obj_rdy, CCD_LISTEN_OBJECT_READY);
		wake_up_all(&mtk_subdev->master_listen_wq);
	}
	mutex_unlock(&mtk_subdev->master_listen_lock);
	if (failed) {
		bool claimed = false;

		/* A recycled address must not retire a replacement device. */
		mutex_lock(&mtk_subdev->endpoints_lock);
		if (idr_find(&mtk_subdev->endpoints, channel.src) == mdev) {
			WRITE_ONCE(to_ccd_device(mdev)->removing, true);
			idr_remove(&mtk_subdev->endpoints, channel.src);
			claimed = true;
		}
		mutex_unlock(&mtk_subdev->endpoints_lock);
		if (claimed)
			ccd_unregister_device(mdev);
	}
	if (failed)
		put_device(&mdev->rpdev.dev);
	return failed ? NULL : mdev;
}

EXPORT_SYMBOL_GPL(mtk_create_client_msgdevice);

int mtk_rpmsg_subdev_probe(struct rproc_subdev *subdev)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = to_mtk_subdev(subdev);

	dev_info(&mtk_subdev->pdev->dev, "%s: %p\n", __func__, mtk_subdev);
	return 0;
}

void mtk_rpmsg_subdev_remove(struct rproc_subdev *subdev)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = to_mtk_subdev(subdev);
	struct mtk_rpmsg_device *mdev;
	struct rpmsg_endpoint *ept;
	int id;

	/* Quiesce before the daemon gate waits for blocked readers to finish. */
	mutex_lock(&mtk_subdev->endpoints_lock);
	WRITE_ONCE(to_ccd_bus(mtk_subdev)->stopping, true);
	idr_for_each_entry(&mtk_subdev->endpoints, mdev, id) {
		if (!mdev)
			continue;
		ept = to_ccd_device(mdev)->endpoint;
		if (ept)
			wake_up_all(&to_mtk_rpmsg_endpoint(ept)->worker_readwq);
	}
	mutex_unlock(&mtk_subdev->endpoints_lock);
	wake_up_all(&mtk_subdev->master_listen_wq);
	wake_up_all(&mtk_subdev->ccd_listen_wq);
}
EXPORT_SYMBOL_GPL(mtk_rpmsg_subdev_remove);

struct rproc_subdev *
mtk_rpmsg_create_rproc_subdev(struct platform_device *pdev,
			      struct mtk_ccd_rpmsg_ops *ops)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev;
	struct mtk_ccd_bus *bus;
	struct rpmsg_channel_info rp_info;
	int ret = 0;

	strncpy(rp_info.name, "mtk_ccd_msgdev", RPMSG_NAME_SIZE);
	rp_info.src = MTK_CCD_MSGDEV_ADDR;
	rp_info.dst = RPMSG_ADDR_ANY;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return NULL;
	mtk_subdev = &bus->subdev;
	kref_init(&bus->refcount);
	init_waitqueue_head(&bus->register_wq);
	mutex_init(&bus->reset_lock);
	get_device(&pdev->dev);

	dev_info(&pdev->dev, "%s mtk_ccd_msgdev addr: %d\n",
		 __func__, rp_info.src);

	mtk_subdev->pdev = pdev;
	mtk_subdev->ops = ops;
	mtk_subdev->ccd_msgdev_addr = rp_info.src;

	idr_init(&mtk_subdev->endpoints);
	mutex_init(&mtk_subdev->endpoints_lock);

	mutex_init(&mtk_subdev->master_listen_lock);
	atomic_set(&mtk_subdev->listen_obj_rdy, CCD_LISTEN_OBJECT_PREPARING);
	init_waitqueue_head(&mtk_subdev->master_listen_wq);
	init_waitqueue_head(&mtk_subdev->ccd_listen_wq);

	ret = ccd_msgdev_init();
	if (ret) {
		kref_put(&bus->refcount, ccd_bus_release);
		return NULL;
	}
	ret = mtk_rpmsg_create_ccd_rpmsgdev(mtk_subdev, &rp_info);
	if (ret) {
		ccd_msgdev_exit();
		kref_put(&bus->refcount, ccd_bus_release);
		return NULL;
	}

	return &mtk_subdev->subdev;
}
EXPORT_SYMBOL_GPL(mtk_rpmsg_create_rproc_subdev);

void mtk_rpmsg_destroy_rproc_subdev(struct rproc_subdev *subdev)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev = to_mtk_subdev(subdev);
	struct mtk_ccd_bus *bus = to_ccd_bus(mtk_subdev);

	mtk_rpmsg_subdev_remove(subdev);
	wait_event(bus->register_wq, !READ_ONCE(bus->registering));
	mtk_ccd_remove_clients(mtk_subdev, true);
	ccd_msgdev_exit();
	kref_put(&bus->refcount, ccd_bus_release);
}

EXPORT_SYMBOL_GPL(mtk_rpmsg_destroy_rproc_subdev);

static int ccd_msgdev_cb(struct rpmsg_device *rpdev, void *data,
			 int len, void *priv, u32 src)
{
	struct mtk_rpmsg_rproc_subdev *subdev = to_mtk_rpmsg_device(rpdev)->mtk_subdev;
	struct rpmsg_endpoint *ept = mtk_ccd_endpoint_get(subdev, src);
	int ret = -ENOENT;

	if (!ept)
		return ret;
	/* The message-device must not forward to itself recursively. */
	if (ept->rpdev != rpdev) {
		mutex_lock(&ept->cb_lock);
		if (ept->cb && atomic_read(&to_mtk_rpmsg_endpoint(ept)->ccd_mep_state) ==
		    CCD_MENDPOINT_CREATED)
			ret = ept->cb(ept->rpdev, data, len, ept->priv, src);
		mutex_unlock(&ept->cb_lock);
	}
	kref_put(&ept->refcount, __ept_release);
	return ret;
}


static int ccd_msgdev_probe(struct rpmsg_device *rpmsg_device)
{
	struct rpmsg_driver *rpdrv = to_rpmsg_driver(rpmsg_device->dev.driver);
	struct mtk_rpmsg_device *mdev = to_mtk_rpmsg_device(rpmsg_device);

	mdev->mtk_subdev->rpdev = rpmsg_device;

	dev_info(&mdev->mtk_subdev->pdev->dev, "%s : %s\n", __func__,
		 rpdrv->id_table->name);
	return 0;
}

static void ccd_msgdev_remove(struct rpmsg_device *rpmsg_device)
{
	/*
	 * struct rpmsg_driver *rpdrv =
	 *	to_rpmsg_driver(rpmsg_device->dev.driver);
	 */
	pr_debug("ccd bus rpmsg_dev_remove: %p\n", rpmsg_device);
}

static struct rpmsg_device_id ccd_msgdev_id_table[] = {
	{.name = "mtk_ccd_msgdev"},
	{},
};

static struct rpmsg_driver ccd_msgdev_driver = {
	.drv = {.name = KBUILD_MODNAME},
	.id_table = ccd_msgdev_id_table,
	.probe = ccd_msgdev_probe,
	.callback = ccd_msgdev_cb,
	.remove = ccd_msgdev_remove,
};

static DEFINE_MUTEX(ccd_driver_lock);
static unsigned int ccd_driver_users;

int ccd_msgdev_init(void)
{
	int ret = 0;

	mutex_lock(&ccd_driver_lock);
	if (!ccd_driver_users)
		ret = register_rpmsg_driver(&ccd_msgdev_driver);
	if (!ret)
		ccd_driver_users++;
	mutex_unlock(&ccd_driver_lock);
	return ret;
}

void ccd_msgdev_exit(void)
{
	mutex_lock(&ccd_driver_lock);
	if (!--ccd_driver_users)
		unregister_rpmsg_driver(&ccd_msgdev_driver);
	mutex_unlock(&ccd_driver_lock);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek ccd rpmsg driver");
