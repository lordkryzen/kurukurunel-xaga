// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2018 MediaTek Inc.

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <uapi/linux/mtk_ccd_controls.h>
#include <linux/platform_data/mtk_ccd.h>
#include <linux/rpmsg/mtk_ccd_rpmsg.h>

#include "mtk_ccd_rpmsg_internal.h"

int ccd_ipi_register(struct platform_device *pdev,
		     enum ccd_ipi_id id,
		     ccd_ipi_handler_t handler,
		     void *priv)
{
	struct mtk_ccd *ccd = platform_get_drvdata(pdev);

	if (!ccd) {
		dev_dbg(&pdev->dev, "ccd device is not ready\n");
		return -EPROBE_DEFER;
	}

	dev_info(ccd->dev, "ipi id: %d\n", id);
	return 0;
}
EXPORT_SYMBOL_GPL(ccd_ipi_register);

void ccd_ipi_unregister(struct platform_device *pdev, enum ccd_ipi_id id)
{
	struct mtk_ccd *ccd = platform_get_drvdata(pdev);

	if (!ccd)
		return;

	if (WARN_ON(id < 0) || WARN_ON(id >= CCD_IPI_MAX))
		return;
}
EXPORT_SYMBOL_GPL(ccd_ipi_unregister);

int rpmsg_ccd_ipi_send(struct mtk_rpmsg_rproc_subdev *mtk_subdev,
		       struct mtk_ccd_rpmsg_endpoint *mept,
		       void *buf, unsigned int len, unsigned int wait)
{
	struct mtk_ccd_params *ccd_params;
	int ret = 0;

	if (len > BUF_MAX_SIZE || (len && !buf))
		return -EINVAL;
	ccd_params = kzalloc(sizeof(*ccd_params), GFP_KERNEL);
	if (!ccd_params)
		return -ENOMEM;
	ccd_params->worker_obj.src = mept->mchinfo.chinfo.src;
	ccd_params->worker_obj.id = mept->mchinfo.id;

	/* TODO: Allocate shared memory for additional buffer
	 * If no buffer ready now, wait or not depending on parameter
	 */

	if (len)
		memcpy(ccd_params->worker_obj.sbuf, buf, len);

	ccd_params->worker_obj.len = len;

	/* No need to use spin_lock_irqsave for all non-irq context */
	spin_lock(&mept->pending_sendq.queue_lock);
	if (atomic_read(&mept->ccd_mep_state) != CCD_MENDPOINT_CREATED ||
	    READ_ONCE(to_ccd_bus(mtk_subdev)->stopping)) {
		spin_unlock(&mept->pending_sendq.queue_lock);
		kfree(ccd_params);
		return -ESHUTDOWN;
	}
	list_add_tail(&ccd_params->list_entry, &mept->pending_sendq.queue);
	atomic_inc(&mept->ccd_cmd_sent);
	spin_unlock(&mept->pending_sendq.queue_lock);

	wake_up_all(&mept->worker_readwq);

	dev_dbg(&mtk_subdev->pdev->dev, "%s: id: %d\n",
		__func__, mept->mchinfo.id);

	return ret;
}
EXPORT_SYMBOL_GPL(rpmsg_ccd_ipi_send);

void ccd_master_destroy(struct mtk_ccd *ccd,
			struct ccd_master_status_item *master_obj)
{
	mtk_ccd_remove_clients(to_mtk_subdev(ccd->rpmsg_subdev), false);
}

EXPORT_SYMBOL_GPL(ccd_master_destroy);

void ccd_master_listen(struct mtk_ccd *ccd,
		       struct ccd_master_listen_item *listen_obj)
{
	int ret;
	u32 listen_obj_rdy;
	struct mtk_rpmsg_rproc_subdev *mtk_subdev =
		to_mtk_subdev(ccd->rpmsg_subdev);

	mutex_lock(&mtk_subdev->master_listen_lock);

	listen_obj_rdy = atomic_read(&mtk_subdev->listen_obj_rdy);
	while (listen_obj_rdy == CCD_LISTEN_OBJECT_PREPARING &&
	       !READ_ONCE(to_ccd_bus(mtk_subdev)->stopping)) {
		mutex_unlock(&mtk_subdev->master_listen_lock);
		ret = wait_event_interruptible
			(mtk_subdev->master_listen_wq,
			 (atomic_read(&mtk_subdev->listen_obj_rdy) ==
			 CCD_LISTEN_OBJECT_READY) ||
		 READ_ONCE(to_ccd_bus(mtk_subdev)->stopping));
		if (ret != 0) {
			dev_dbg(ccd->dev,
				"master listen wait error: %d\n", ret);
			return;
		}
		mutex_lock(&mtk_subdev->master_listen_lock);
		listen_obj_rdy = atomic_read(&mtk_subdev->listen_obj_rdy);
	}
	if (READ_ONCE(to_ccd_bus(mtk_subdev)->stopping)) {
		mutex_unlock(&mtk_subdev->master_listen_lock);
		return;
	}

	/* Could be memory copied directly */
	memcpy(listen_obj->name, mtk_subdev->listen_obj.name,
	       sizeof(listen_obj->name));
	listen_obj->src = mtk_subdev->listen_obj.src;
	listen_obj->cmd = mtk_subdev->listen_obj.cmd;

	atomic_set(&mtk_subdev->listen_obj_rdy, CCD_LISTEN_OBJECT_PREPARING);
	wake_up(&mtk_subdev->ccd_listen_wq);
	mutex_unlock(&mtk_subdev->master_listen_lock);

	dev_dbg(ccd->dev, "%s, src: %d\n", __func__,
		 mtk_subdev->listen_obj.src);
}
EXPORT_SYMBOL_GPL(ccd_master_listen);

void ccd_worker_read(struct mtk_ccd *ccd,
		     struct ccd_worker_item *read_obj)
{
	int ret;
	struct rpmsg_endpoint *ept;
	struct mtk_ccd_params *ccd_params;
	struct mtk_ccd_rpmsg_endpoint *mept;
	struct mtk_rpmsg_rproc_subdev *mtk_subdev =
		to_mtk_subdev(ccd->rpmsg_subdev);

	dev_dbg(ccd->dev, "%s, src: %d, %p\n", __func__,
		 read_obj->src, mtk_subdev);

	ept = mtk_ccd_endpoint_get(mtk_subdev, read_obj->src);
	if (!ept)
		return;
	mept = to_mtk_rpmsg_endpoint(ept);
	dev_dbg(ccd->dev, "mept: %p src: %d id: %d\n",
		 mept, mept->mchinfo.chinfo.src, mept->mchinfo.id);

	if (atomic_read(&mept->ccd_mep_state) == CCD_MENDPOINT_DESTROY ||
	    READ_ONCE(to_ccd_bus(mtk_subdev)->stopping)) {
		dev_info(ccd->dev, "mept: %p src: %d is destroyed\n",
			 mept, mept->mchinfo.chinfo.src);
		goto err_ret;
	}

	if (atomic_read(&mept->ccd_cmd_sent) == 0) {
		atomic_set(&mept->worker_read_rdy, 1);
		ret = wait_event_interruptible
			(mept->worker_readwq,
			 (atomic_read(&mept->ccd_cmd_sent) > 0) ||
			 (atomic_read(&mept->ccd_mep_state) !=
			 CCD_MENDPOINT_CREATED) ||
			 READ_ONCE(to_ccd_bus(mtk_subdev)->stopping));

		atomic_set(&mept->worker_read_rdy, 0);
		if (ret != 0) {
			dev_dbg(ccd->dev,
				"worker read wait error: %d\n", ret);
			goto err_ret;
		}
	}

	if (atomic_read(&mept->ccd_mep_state) == CCD_MENDPOINT_DESTROY ||
	    READ_ONCE(to_ccd_bus(mtk_subdev)->stopping)) {
		dev_info(ccd->dev, "mept: %p src: %d would destroy\n",
			 mept, mept->mchinfo.chinfo.src);
		goto err_ret;
	}

	spin_lock(&mept->pending_sendq.queue_lock);
	ccd_params = list_first_entry_or_null(&mept->pending_sendq.queue,
				      struct mtk_ccd_params,
				      list_entry);
	if (!ccd_params) {
		spin_unlock(&mept->pending_sendq.queue_lock);
		dev_info(ccd->dev, "%s: get MULL ccd_params, ccd_cmd_sent(%d)\n",
			 __func__, atomic_read(&mept->ccd_cmd_sent));
		goto err_ret;
	}
	list_del(&ccd_params->list_entry);
	atomic_dec(&mept->ccd_cmd_sent);
	spin_unlock(&mept->pending_sendq.queue_lock);

	memcpy(read_obj, &ccd_params->worker_obj, sizeof(*read_obj));
	kfree(ccd_params);
err_ret:
	kref_put(&mept->ept.refcount, __ept_release);
}
EXPORT_SYMBOL_GPL(ccd_worker_read);

void ccd_worker_write(struct mtk_ccd *ccd,
		      struct ccd_worker_item *write_obj)
{
	struct mtk_rpmsg_rproc_subdev *mtk_subdev =
		to_mtk_subdev(ccd->rpmsg_subdev);
	struct rpmsg_endpoint *ept;
	struct mtk_ccd_rpmsg_endpoint *mept;

	if (write_obj->len > sizeof(write_obj->sbuf))
		return;
	ept = mtk_ccd_endpoint_get(mtk_subdev, write_obj->src);
	if (!ept)
		return;
	mept = to_mtk_rpmsg_endpoint(ept);
	dev_dbg(ccd->dev, "mept: %p src: %d id: %d\n",
		 mept, mept->mchinfo.chinfo.src, mept->mchinfo.id);

	if (atomic_read(&mept->ccd_mep_state) == CCD_MENDPOINT_DESTROY ||
	    READ_ONCE(to_ccd_bus(mtk_subdev)->stopping)) {
		dev_info(ccd->dev, "mept: %p src: %d is destroyed\n",
			 mept, mept->mchinfo.chinfo.src);
		goto err_ret;
	}


	dev_dbg(ccd->dev, "%s, src: %d, ept: %p\n", __func__,
		 write_obj->src, ept);

	mutex_lock(&ept->cb_lock);

	if (ept->cb && atomic_read(&mept->ccd_mep_state) == CCD_MENDPOINT_CREATED)
		ept->cb(ept->rpdev, write_obj->sbuf, write_obj->len, ept->priv,
			write_obj->src);

	mutex_unlock(&ept->cb_lock);

err_ret:
	kref_put(&mept->ept.refcount, __ept_release);
	/* TBD: Free shared memory for additional buffer
	 * If no buffer ready now, wait or not depending on parameter
	 */
}
EXPORT_SYMBOL_GPL(ccd_worker_write);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek ccd IPI interface");
