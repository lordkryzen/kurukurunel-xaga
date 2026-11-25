#include "../mtk-imgsys-dma.h"
#include <linux/hashtable.h>
#include <linux/vmalloc.h>
#include <linux/of_platform.h>
#include <linux/hashtable.h>
#include <linux/iosys-map.h>
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
 *
 * Author: Frederic Chen <frederic.chen@mediatek.com>
 *
 */

#include <linux/dma-buf.h>
#include <linux/overflow.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-event.h>
#include "mtk_imgsys-dev.h"
#include "mtkdip.h"
#include "mtk-img-ipi.h"
#include "mtk_header_desc.h"
#include "mtk_imgsys-trace.h"

struct fd_kva_list_t fd_kva_info_list = {
	.mymutex = __MUTEX_INITIALIZER(fd_kva_info_list.mymutex),
	.mylist = LIST_HEAD_INIT(fd_kva_info_list.mylist)
};

unsigned int nodes_num;

int mtk_imgsys_pipe_init(struct mtk_imgsys_dev *imgsys_dev,
				struct mtk_imgsys_pipe *pipe,
				const struct mtk_imgsys_pipe_desc *setting)
{
	int ret;
	u32 i;
	size_t nodes_size;

	pipe->imgsys_dev = imgsys_dev;
	pipe->desc = setting;
	pipe->nodes_enabled = 0ULL;
	pipe->nodes_streaming = 0ULL;
	pipe->pipeline_started = false;
	atomic_set(&pipe->callbacks, 0);
	INIT_WORK(&pipe->drain_work, mtk_imgsys_drain_work);

	atomic_set(&pipe->pipe_job_sequence, 0);
	INIT_LIST_HEAD(&pipe->pipe_job_running_list);
	INIT_LIST_HEAD(&pipe->pipe_job_pending_list);
	INIT_LIST_HEAD(&pipe->iova_cache.list);
	spin_lock_init(&pipe->iova_cache.lock);
	hash_init(pipe->iova_cache.hlists);
	//spin_lock_init(&pipe->job_lock);
	//mutex_init(&pipe->job_lock);
	spin_lock_init(&pipe->pending_job_lock);
	spin_lock_init(&pipe->running_job_lock);
	mutex_init(&pipe->lock);

	nodes_num = pipe->desc->total_queues;
	nodes_size = sizeof(*pipe->nodes) * nodes_num;
	pipe->nodes = devm_kzalloc(imgsys_dev->dev, nodes_size, GFP_KERNEL);
	if (!pipe->nodes)
		return -ENOMEM;

	for (i = 0; i < nodes_num; i++) {
		pipe->nodes[i].desc = &pipe->desc->queue_descs[i];
		pipe->nodes[i].started = false;
		pipe->nodes[i].flags = pipe->nodes[i].desc->flags;
		spin_lock_init(&pipe->nodes[i].buf_list_lock);
		INIT_LIST_HEAD(&pipe->nodes[i].buf_list);

		pipe->nodes[i].crop.left = 0;
		pipe->nodes[i].crop.top = 0;
		pipe->nodes[i].crop.width =
			pipe->nodes[i].desc->frmsizeenum->stepwise.max_width;
		pipe->nodes[i].crop.height =
			pipe->nodes[i].desc->frmsizeenum->stepwise.max_height;
		pipe->nodes[i].compose.left = 0;
		pipe->nodes[i].compose.top = 0;
		pipe->nodes[i].compose.width =
			pipe->nodes[i].desc->frmsizeenum->stepwise.max_width;
		pipe->nodes[i].compose.height =
			pipe->nodes[i].desc->frmsizeenum->stepwise.max_height;
	}

	ret = mtk_imgsys_pipe_v4l2_register(pipe, &imgsys_dev->mdev,
					 &imgsys_dev->v4l2_dev);
	if (ret) {
		dev_info(pipe->imgsys_dev->dev,
			"%s: failed(%d) to create V4L2 devices\n",
			pipe->desc->name, ret);

		goto err_destroy_pipe_lock;
	}

	return 0;

err_destroy_pipe_lock:
	mutex_destroy(&pipe->lock);

	return ret;
}

int mtk_imgsys_pipe_release(struct mtk_imgsys_pipe *pipe)
{
	mtk_imgsys_pipe_v4l2_unregister(pipe);
	mutex_destroy(&pipe->lock);

	return 0;
}

int mtk_imgsys_pipe_next_job_id(struct mtk_imgsys_pipe *pipe)
{
	int global_job_id = atomic_inc_return(&pipe->pipe_job_sequence);

	return (global_job_id & 0x0000FFFF) | (pipe->desc->id << 16);
}

#ifdef BATCH_MODE_V3
int mtk_imgsys_pipe_next_job_id_batch_mode(struct mtk_imgsys_pipe *pipe,
		unsigned short user_id)
{
	int global_job_id = atomic_inc_return(&pipe->pipe_job_sequence);

	return (global_job_id & 0x0000FFFF) | (user_id << 16);
}
#endif

void mtk_imgsys_job_init(struct mtk_imgsys_request *req)
{
	mutex_init(&req->job_lock);
	atomic_set(&req->callback_refs, 0);
	INIT_LIST_HEAD(&req->active_link);
	INIT_LIST_HEAD(&req->list);
}

int mtk_imgsys_job_register(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	unsigned long flags;

	mutex_lock(&req->job_lock);
	if (req->job_owner || atomic_read(&req->callback_refs) || READ_ONCE(pipe->draining)) {
		mutex_unlock(&req->job_lock);
		return -EBUSY;
	}
	/* Admission owns execution before any asynchronous producer is queued.
	 * JOB_RUN was a debug switch, not a valid lifetime/registry boundary.
	 */
	media_request_get(&req->req);
	req->job_owner = true;
	req->buffers_cancelled = false;
	spin_lock_irqsave(&pipe->running_job_lock, flags);
	list_add_tail(&req->active_link, &pipe->pipe_job_running_list);
	pipe->num_jobs++;
	atomic_inc(&pipe->imgsys_dev->num_composing);
	spin_unlock_irqrestore(&pipe->running_job_lock, flags);
	mutex_unlock(&req->job_lock);
	return 0;
}

/* A pending job owns no submitted hardware work. Permanent vb2 cancellation
 * may retire that execution ref, unlike a temporary QUEUED start rollback.
 * Queue locks nest outside pipe.lock, then job_lock, then node/registry locks.
 */
void mtk_imgsys_request_cancel(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct mtk_imgsys_dev_buffer *buffer, *next;
	unsigned long flags;
	bool retire = false;
	int i;

	media_request_get(&req->req);
	mutex_lock(&pipe->lock);
	mutex_lock(&req->job_lock);
	req->buffers_cancelled = true;
	spin_lock_irqsave(&pipe->pending_job_lock, flags);
	if (!list_empty(&req->list)) {
		list_del_init(&req->list);
		pipe->num_pending_jobs--;
		retire = req->job_owner;
	}
	spin_unlock_irqrestore(&pipe->pending_job_lock, flags);
	if (retire) {
		/* Walk owned node lists, not buf_map: another queue may already
		 * have returned and freed one of this request's buffers.
		 */
		for (i = 0; i < pipe->desc->total_queues; i++) {
			struct mtk_imgsys_video_device *node = &pipe->nodes[i];

			/* A different queue can be between buf_queue and its first
			 * start callback. Let that callback return QUEUED on failure,
			 * rather than injecting ERROR into its core done_list.
			 */
			if (!node->started)
				continue;
			spin_lock(&node->buf_list_lock);
			list_for_each_entry_safe(buffer, next, &node->buf_list, list) {
				if (buffer->vbb.vb2_buf.request != &req->req)
					continue;
				list_del_init(&buffer->list);
				if (buffer->vbb.vb2_buf.state == VB2_BUF_STATE_ACTIVE)
					vb2_buffer_done(&buffer->vbb.vb2_buf, VB2_BUF_STATE_ERROR);
			}
			spin_unlock(&node->buf_list_lock);
		}
		mtk_imgsys_pipe_remove_job(req);
		req->job_owner = false;
		complete(&req->done);
	}
	mutex_unlock(&req->job_lock);
	mutex_unlock(&pipe->lock);
	if (retire) {
		wake_up_all(&pipe->imgsys_dev->flushing_waitq);
		media_request_put(&req->req);
	}
	media_request_put(&req->req);
}

/* The untrusted firmware/command-queue cookie is compared, never dereferenced.
 * A successful lookup owns a media-request reference, released by its caller.
 */
struct mtk_imgsys_request *mtk_imgsys_job_get(struct mtk_imgsys_pipe *pipe,
					 int id, const void *cookie)
{
	struct mtk_imgsys_request *req, *result = NULL;
	unsigned long flags;

	spin_lock_irqsave(&pipe->running_job_lock, flags);
	list_for_each_entry(req, &pipe->pipe_job_running_list, active_link) {
		if (req->id == id && (!cookie || req == cookie)) {
			media_request_get(&req->req);
			atomic_inc(&pipe->callbacks);
			atomic_inc(&req->callback_refs);
			result = req;
			break;
		}
	}
	spin_unlock_irqrestore(&pipe->running_job_lock, flags);
	return result;
}

void mtk_imgsys_job_put(struct mtk_imgsys_request *req, struct mtk_imgsys_pipe *pipe)
{
	atomic_dec(&req->callback_refs);
	atomic_dec(&pipe->callbacks);
	wake_up_all(&pipe->imgsys_dev->flushing_waitq);
	if (READ_ONCE(pipe->draining))
		schedule_work(&pipe->drain_work);
	media_request_put(&req->req);
}

void mtk_imgsys_pipe_remove_job(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	unsigned long flags;

	lockdep_assert_held(&req->job_lock);
	spin_lock_irqsave(&pipe->running_job_lock, flags);
	if (!list_empty(&req->active_link)) {
		list_del_init(&req->active_link);
		pipe->num_jobs--;
		atomic_dec(&pipe->imgsys_dev->num_composing);
	}
	spin_unlock_irqrestore(&pipe->running_job_lock, flags);
}

struct mtk_imgsys_request *mtk_imgsys_pipe_get_running_job(
			struct mtk_imgsys_pipe *pipe, int id)
{
	return mtk_imgsys_job_get(pipe, id, NULL);
}

void mtk_imgsys_pipe_debug_job(struct mtk_imgsys_pipe *pipe,
			    struct mtk_imgsys_request *req)
{
	int i;

	dev_dbg(pipe->imgsys_dev->dev, "%s:%s: pipe-job(%p),id(%d)\n",
		__func__, pipe->desc->name, req, req->id);

	for (i = 0; i < pipe->desc->total_queues ; i++) {
		if (req->buf_map[i])
			dev_dbg(pipe->imgsys_dev->dev, "%s:%s:buf(%p)\n",
				pipe->desc->name, pipe->nodes[i].desc->name,
				req->buf_map[i]);
	}
}

#ifdef BATCH_MODE_V3
bool is_batch_mode(struct mtk_imgsys_request *req)
{
	return req->is_batch_mode;
}

void mtk_imgsys_frame_done(struct mtk_imgsys_request *req)
{
	bool found = false;
	const int user_id = mtk_imgsys_pipe_get_pipe_from_job_id(req->id);
	struct mtk_imgsys_user *user;
	struct mtk_imgsys_dev *imgsys_dev = req->imgsys_pipe->imgsys_dev;
	unsigned long flags;

	mutex_lock(&imgsys_dev->imgsys_users.user_lock);
	list_for_each_entry(user, &imgsys_dev->imgsys_users.list, entry) {
		if (user == NULL)
			continue;

		if (user->id == user_id) {
			found = true;
			break;
		}
	}
	mutex_unlock(&imgsys_dev->imgsys_users.user_lock);

	// add into user's done_list and wake it up
	if (found) {
		spin_lock_irqsave(&user->lock, flags);

		list_add_tail(&req->done_pack->done_entry, &user->done_list);

		user->dqdonestate = true;


		pr_info("%s: user id(%x) req(%p) add done_pack\n", __func__,
			user->id, req);

		wake_up(&user->done_wq);
		wake_up(&user->enque_wq);
		spin_unlock_irqrestore(&user->lock, flags);

		if (!IS_ERR(req)) {
			pr_info("%s free req(%p)\n", __func__, req);
			vfree(req);
		}
	}
}
#endif

void mtk_imgsys_pipe_job_finish(struct mtk_imgsys_request *req,
			     enum vb2_buffer_state vbf_state)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct mtk_imgsys_dev_buffer *in_buf;
	int i;
	int req_id = req->id;
	unsigned int vb2_buffer_index;

#ifdef BATCH_MODE_V3
	// batch mode
	if (is_batch_mode(req)) {
		mtk_imgsys_frame_done(req);
		return;
	}
#endif
	lockdep_assert_held(&req->job_lock);
	if (req->buffers_cancelled)
		goto done;
	dev_dbg(pipe->imgsys_dev->dev, "%s: req %d 0x%p state(%d)",
			__func__, req->tstate.req_fd, &req->req, req->req.state);
	if (req->req.state != MEDIA_REQUEST_STATE_QUEUED) {
		dev_info(pipe->imgsys_dev->dev, "%s: req %d 0x%p flushed in state(%d)", __func__,
					req->tstate.req_fd, &req->req, req->req.state);
		goto done;
	}

	i = is_singledev_mode(req);
	if (!i)
		i = MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT;

	in_buf = req->buf_map[i];

	for (i = 0; i < pipe->desc->total_queues; i++) {
		struct mtk_imgsys_dev_buffer *dev_buf = req->buf_map[i];
		struct mtk_imgsys_video_device *node;

		if (!dev_buf)
			continue;

		if (in_buf && dev_buf != in_buf)
			dev_buf->vbb.vb2_buf.timestamp =
				in_buf->vbb.vb2_buf.timestamp;

		vb2_buffer_index = dev_buf->vbb.vb2_buf.index;
		node = mtk_imgsys_vbq_to_node(dev_buf->vbb.vb2_buf.vb2_queue);

		spin_lock(&node->buf_list_lock);
		if (list_empty(&dev_buf->list)) {
			spin_unlock(&node->buf_list_lock);
			continue;
		}
		list_del_init(&dev_buf->list);
		spin_unlock(&node->buf_list_lock);


		/*  vb2_buf is changed after below function  */
		vb2_buffer_done(&dev_buf->vbb.vb2_buf, vbf_state);

		dev_dbg(pipe->imgsys_dev->dev,
			"%s:%s:%s: return buf, idx(%d), state(%d)\n",
			__func__, pipe->desc->name, node->desc->name,
			vb2_buffer_index, vbf_state);
	}
done:
	req->tstate.time_notify2vb2done = ktime_get_boottime_ns()/1000;
	complete(&req->done);
		dev_dbg(pipe->imgsys_dev->dev,
			"[KT]%s:%d:%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld\n",
			__func__, req->tstate.req_fd,
			//req->tstate.time_hwenq,
			req->tstate.time_composingStart,
			req->tstate.time_composingEnd,
			req->tstate.time_qw2composer,
			req->tstate.time_compfuncStart,
			req->tstate.time_ipisendStart,
			req->tstate.time_reddonescpStart,
			req->tstate.time_doframeinfo,
			req->tstate.time_qw2runner,
			req->tstate.time_runnerStart,
			req->tstate.time_send2cmq,
			req->tstate.time_mdpcbStart,
			req->tstate.time_notifyStart,
			req->tstate.time_unmapiovaEnd,
			req->tstate.time_notify2vb2done);

	dev_dbg(pipe->imgsys_dev->dev,
		"%s:%s: finished job id(%d), vbf_state(%d)\n",
		__func__, pipe->desc->name, req_id, vbf_state);
}

static u32 dip_pass1_fmt_get_stride(struct v4l2_pix_format_mplane *mfmt,
				    const struct mtk_imgsys_dev_format *fmt,
				    unsigned int plane)
{
	unsigned int width = ALIGN(mfmt->width, 4);
	unsigned int pixel_bits = fmt->row_depth[0];
	unsigned int bpl, ppl;

	ppl = DIV_ROUND_UP(width * fmt->depth[0], fmt->row_depth[0]);
	bpl = DIV_ROUND_UP(ppl * pixel_bits, 8);

	return ALIGN(bpl, fmt->pass_1_align);
}

/* Stride that is accepted by MDP HW */
static u32 dip_mdp_fmt_get_stride(struct v4l2_pix_format_mplane *mfmt,
				  const struct mtk_imgsys_dev_format *fmt,
				  unsigned int plane)
#ifdef MDP_COLOR
{
	enum mdp_color c = fmt->mdp_color;
	u32 bytesperline = (mfmt->width * fmt->row_depth[plane]) / 8;
	u32 stride = (bytesperline * DIP_MCOLOR_BITS_PER_PIXEL(c))
		/ fmt->row_depth[0];

	if (plane == 0)
		return stride;

	if (plane < DIP_MCOLOR_GET_PLANE_COUNT(c)) {
		if (DIP_MCOLOR_IS_BLOCK_MODE(c))
			stride = stride / 2;
		return stride;
	}

	return 0;
}
#else
{
	return 0;
}
#endif

static int mtk_imgsys_pipe_get_stride(struct mtk_imgsys_pipe *pipe,
				   struct v4l2_pix_format_mplane *mfmt,
				   const struct mtk_imgsys_dev_format *dfmt,
				   int plane, char *buf_name)
{
	int bpl;

	if (dfmt->pass_1_align)
		bpl = dip_pass1_fmt_get_stride(mfmt, dfmt, plane);
	else
		bpl = dip_mdp_fmt_get_stride(mfmt, dfmt, plane);

	dev_dbg(pipe->imgsys_dev->dev,
		"%s:%s:%s: plane(%d), pixelformat(%x) width(%d), stride(%d)",
		__func__, pipe->desc->name, buf_name, plane, mfmt->pixelformat,
		mfmt->width, bpl);

	return bpl;
}

void mtk_imgsys_pipe_try_fmt(struct mtk_imgsys_pipe *pipe,
			  struct mtk_imgsys_video_device *node,
			  struct v4l2_format *fmt,
			  const struct v4l2_format *ufmt,
			  const struct mtk_imgsys_dev_format *dfmt)
{
	int i;

	fmt->type = ufmt->type;
	fmt->fmt.pix_mp.width =
		clamp_val(ufmt->fmt.pix_mp.width,
			  node->desc->frmsizeenum->stepwise.min_width,
			  node->desc->frmsizeenum->stepwise.max_width);
	fmt->fmt.pix_mp.height =
		clamp_val(ufmt->fmt.pix_mp.height,
			  node->desc->frmsizeenum->stepwise.min_height,
			  node->desc->frmsizeenum->stepwise.max_height);
	fmt->fmt.pix_mp.num_planes = ufmt->fmt.pix_mp.num_planes;
	fmt->fmt.pix_mp.quantization = ufmt->fmt.pix_mp.quantization;
	fmt->fmt.pix_mp.colorspace = ufmt->fmt.pix_mp.colorspace;
	fmt->fmt.pix_mp.field = V4L2_FIELD_NONE;

	/* Only MDP 0 and MDP 1 allow the color space change */
#ifdef MDP_PORT
	if (node->desc->id != MTK_IMGSYS_VIDEO_NODE_ID_WROT_A_CAPTURE &&
	    node->desc->id != MTK_IMGSYS_VIDEO_NODE_ID_WROT_B_CAPTURE) {
		fmt->fmt.pix_mp.quantization = V4L2_QUANTIZATION_FULL_RANGE;
		fmt->fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
	}
#endif
	fmt->fmt.pix_mp.pixelformat = dfmt->format;
	fmt->fmt.pix_mp.num_planes = dfmt->num_planes;
	fmt->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->fmt.pix_mp.field = V4L2_FIELD_NONE;

	for (i = 0; i < fmt->fmt.pix_mp.num_planes; ++i) {
		unsigned int stride;
		unsigned int sizeimage;

		if (is_desc_fmt(dfmt)) {
			fmt->fmt.pix_mp.plane_fmt[i].sizeimage =
							dfmt->buffer_size;
		} else {
			stride = mtk_imgsys_pipe_get_stride(pipe,
							&fmt->fmt.pix_mp, dfmt,
							i, node->desc->name);
			if (dfmt->pass_1_align)
				sizeimage = stride * fmt->fmt.pix_mp.height;
			else
				sizeimage = DIV_ROUND_UP(stride *
							 fmt->fmt.pix_mp.height
							 * dfmt->depth[i],
							 dfmt->row_depth[i]);

			fmt->fmt.pix_mp.plane_fmt[i].sizeimage = sizeimage;
			fmt->fmt.pix_mp.plane_fmt[i].bytesperline = stride;
		}
	}

}

static void set_meta_fmt(struct mtk_imgsys_pipe *pipe,
			 struct v4l2_meta_format *fmt,
			 const struct mtk_imgsys_dev_format *dev_fmt)
{
	fmt->dataformat = dev_fmt->format;

	if (dev_fmt->buffer_size <= 0) {
		dev_dbg(pipe->imgsys_dev->dev,
			"%s: Invalid meta buf size(%u), use default(%u)\n",
			pipe->desc->name, dev_fmt->buffer_size,
			MTK_DIP_DEV_META_BUF_DEFAULT_SIZE);

		fmt->buffersize =
			MTK_DIP_DEV_META_BUF_DEFAULT_SIZE;
	} else {
		fmt->buffersize = dev_fmt->buffer_size;
	}
}

void mtk_imgsys_pipe_load_default_fmt(struct mtk_imgsys_pipe *pipe,
				   struct mtk_imgsys_video_device *node,
				   struct v4l2_format *fmt)
{
	int idx = node->desc->default_fmt_idx;

	if (idx >= node->desc->num_fmts) {
		dev_info(pipe->imgsys_dev->dev,
			"%s:%s: invalid idx(%d), must < num_fmts(%d)\n",
			__func__, node->desc->name, idx, node->desc->num_fmts);

		idx = 0;
	}

	fmt->type = node->desc->buf_type;
	if (mtk_imgsys_buf_is_meta(node->desc->buf_type)) {
		set_meta_fmt(pipe, &fmt->fmt.meta,
			     &node->desc->fmts[idx]);
	} else {
		fmt->fmt.pix_mp.width = node->desc->default_width;
		fmt->fmt.pix_mp.height = node->desc->default_height;
		mtk_imgsys_pipe_try_fmt(pipe, node, fmt, fmt,
				     &node->desc->fmts[idx]);
	}
}

const struct mtk_imgsys_dev_format*
mtk_imgsys_pipe_find_fmt(struct mtk_imgsys_pipe *pipe,
		      struct mtk_imgsys_video_device *node,
		      u32 format)
{
	int i;

	for (i = 0; i < node->desc->num_fmts; i++) {
		if (node->desc->fmts[i].format == format)
			return &node->desc->fmts[i];
	}

	return NULL;
}

bool is_desc_mode(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_dev_buffer *buffer =
		req->buf_map[MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT];

	return buffer && (buffer->dev_fmt->format == V4L2_META_FMT_MTISP_DESC ||
		buffer->dev_fmt->format == V4L2_META_FMT_MTISP_DESC_NORM);
}

int is_singledev_mode(struct mtk_imgsys_request *req)
{
	int ret = 0;

	if (req->buf_map[MTK_IMGSYS_VIDEO_NODE_SIGDEV_OUT])
		ret = MTK_IMGSYS_VIDEO_NODE_SIGDEV_OUT;
	else if (req->buf_map[MTK_IMGSYS_VIDEO_NODE_SIGDEV_NORM_OUT])
		ret = MTK_IMGSYS_VIDEO_NODE_SIGDEV_NORM_OUT;

	return ret;
}

bool is_desc_fmt(const struct mtk_imgsys_dev_format *dev_fmt)
{

	bool std_fmt;

	switch (dev_fmt->format) {
	case V4L2_META_FMT_MTISP_DESC:
	case V4L2_META_FMT_MTISP_SD:
	case V4L2_META_FMT_MTISP_DESC_NORM:
	case V4L2_META_FMT_MTISP_SDNORM:
		std_fmt = false;
		break;
	default:
		std_fmt = true;
		break;
	}

	return !std_fmt;
}
static bool mtk_imgsys_iova_span_valid(struct sg_table *sgt, size_t size)
{
	struct scatterlist *sg;
	dma_addr_t next, start;
	size_t total = 0;
	unsigned int i;

	if (!sgt->nents || !size)
		return false;
	start = sg_dma_address(sgt->sgl);
	next = start;
	for_each_sgtable_dma_sg(sgt, sg, i) {
		if (sg_dma_address(sg) != next || !sg_dma_len(sg) ||
		    check_add_overflow(next, (dma_addr_t)sg_dma_len(sg), &next) ||
		    check_add_overflow(total, (size_t)sg_dma_len(sg), &total))
			return false;
	}
	return total >= size;
}

static void mtk_imgsys_iova_release(struct kref *ref)
{
	struct mtk_imgsys_dma_buf_iova_get_info *entry = container_of(ref,
		struct mtk_imgsys_dma_buf_iova_get_info, refcount);

	if (entry->cached) {
		kref_put(&entry->cached->refcount, mtk_imgsys_iova_release);
		vfree(entry);
		return;
	}
	if (!IS_ERR_OR_NULL(entry->attach)) {
		if (!IS_ERR_OR_NULL(entry->sgt))
			dma_buf_unmap_attachment_unlocked(entry->attach, entry->sgt,
				DMA_BIDIRECTIONAL);
		dma_buf_detach(entry->dma_buf, entry->attach);
	}
	dma_buf_put(entry->dma_buf);
	put_device(entry->owner);
	vfree(entry);
}

void mtk_imgsys_iova_put(struct mtk_imgsys_dma_buf_iova_get_info *entry)
{
	kref_put(&entry->refcount, mtk_imgsys_iova_release);
}

/* Consumes the incoming DMA-BUF reference, including every failure path. */
static struct mtk_imgsys_dma_buf_iova_get_info *
mtk_imgsys_iova_create(struct device *dev, struct dma_buf *buffer, int fd)
{
	struct mtk_imgsys_dma_buf_iova_get_info *entry;
	int ret;

	entry = vzalloc(sizeof(*entry));
	if (!entry) {
		dma_buf_put(buffer);
		return ERR_PTR(-ENOMEM);
	}
	INIT_LIST_HEAD(&entry->list_entry);
	kref_init(&entry->refcount);
	entry->owner = get_device(dev);
	entry->ionfd = fd;
	entry->dma_buf = buffer;
	entry->attach = dma_buf_attach(buffer, dev);
	if (IS_ERR(entry->attach)) {
		ret = PTR_ERR(entry->attach);
		goto fail;
	}
	entry->sgt = dma_buf_map_attachment_unlocked(entry->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(entry->sgt)) {
		ret = PTR_ERR(entry->sgt);
		goto fail;
	}
	if (!mtk_imgsys_iova_span_valid(entry->sgt, buffer->size)) {
		ret = -EINVAL;
		goto fail;
	}
	entry->dma_addr = sg_dma_address(entry->sgt->sgl);
	return entry;
fail:
	mtk_imgsys_iova_put(entry);
	return ERR_PTR(ret);
}

static struct mtk_imgsys_dma_buf_iova_get_info *
mtk_imgsys_iova_find(struct mtk_imgsys_pipe *pipe, struct dma_buf *buffer)
{
	struct mtk_imgsys_dma_buf_iova_get_info *entry;

	list_for_each_entry(entry, &pipe->iova_cache.list, list_entry)
		if (entry->dma_buf == buffer)
			return entry;
	return NULL;
}

void mtk_imgsys_flush_iova_cache(struct mtk_imgsys_pipe *pipe)
{
	struct mtk_imgsys_dma_buf_iova_get_info *entry, *next;
	LIST_HEAD(retired);

	spin_lock(&pipe->iova_cache.lock);
	list_splice_init(&pipe->iova_cache.list, &retired);
	list_for_each_entry(entry, &retired, list_entry)
		hash_del(&entry->hnode);
	spin_unlock(&pipe->iova_cache.lock);
	list_for_each_entry_safe(entry, next, &retired, list_entry) {
		list_del(&entry->list_entry);
		mtk_imgsys_iova_put(entry);
	}
}

int mtk_imgsys_add_iova(struct mtk_imgsys_pipe *pipe, struct fd_info *info)
{
	struct mtk_imgsys_dma_buf_iova_get_info *entry, *old, *next;
	struct dma_buf *buffer;
	LIST_HEAD(pending);
	int i, ret = 0;

	if (info->fd_num > ARRAY_SIZE(info->fds))
		return -EINVAL;
	memset(info->fds_size, 0, sizeof(info->fds_size));
	for (i = 0; i < info->fd_num; i++) {
		if (!info->fds[i])
			continue;
		buffer = dma_buf_get(info->fds[i]);
		if (IS_ERR(buffer))
			continue;
		entry = mtk_imgsys_iova_create(pipe->imgsys_dev->dev, buffer, info->fds[i]);
		if (IS_ERR(entry)) {
			if (PTR_ERR(entry) == -ENOMEM) {
				ret = -ENOMEM;
				goto rollback;
			}
			continue;
		}
		info->fds_size[i] = buffer->size;
		list_add_tail(&entry->list_entry, &pending);
	}
	/* Mapping/exporter calls may sleep; only publish and reference operations
	 * belong under the cache spinlock. Resolve concurrent duplicate additions.
	 */
	list_for_each_entry_safe(entry, next, &pending, list_entry) {
		list_del(&entry->list_entry);
		spin_lock(&pipe->iova_cache.lock);
		old = mtk_imgsys_iova_find(pipe, entry->dma_buf);
		if (!old) {
			list_add_tail(&entry->list_entry, &pipe->iova_cache.list);
			hash_add(pipe->iova_cache.hlists, &entry->hnode, entry->ionfd);
		}
		spin_unlock(&pipe->iova_cache.lock);
		if (old)
			mtk_imgsys_iova_put(entry);
	}
	return 0;
rollback:
	list_for_each_entry_safe(entry, next, &pending, list_entry) {
		list_del(&entry->list_entry);
		mtk_imgsys_iova_put(entry);
	}
	memset(info->fds_size, 0, sizeof(info->fds_size));
	return ret;
}

int mtk_imgsys_del_iova(struct mtk_imgsys_pipe *pipe, struct fd_info *info)
{
	struct mtk_imgsys_dma_buf_iova_get_info *entry;
	struct dma_buf *buffer;
	int i;

	if (info->fd_num > ARRAY_SIZE(info->fds))
		return -EINVAL;
	memset(info->fds_size, 0, sizeof(info->fds_size));
	for (i = 0; i < info->fd_num; i++) {
		if (!info->fds[i])
			continue;
		buffer = dma_buf_get(info->fds[i]);
		if (IS_ERR(buffer))
			continue;
		spin_lock(&pipe->iova_cache.lock);
		entry = mtk_imgsys_iova_find(pipe, buffer);
		if (entry) {
			list_del(&entry->list_entry);
			hash_del(&entry->hnode);
			info->fds_size[i] = buffer->size;
		}
		spin_unlock(&pipe->iova_cache.lock);
		if (entry)
			mtk_imgsys_iova_put(entry);
		dma_buf_put(buffer);
	}
	return 0;
}

static u64 mtk_imgsys_get_iova(struct dma_buf *buffer, s32 fd,
			     struct mtk_imgsys_pipe *pipe,
			     struct mtk_imgsys_dev_buffer *dev_buf)
{
	struct mtk_imgsys_dma_buf_iova_get_info *entry, *cached;
	struct mtk_imgsys_request *req;
	dma_addr_t address;

	if (IS_ERR_OR_NULL(buffer))
		return 0;
	if (!dev_buf->vbb.vb2_buf.request) {
		dma_buf_put(buffer);
		return 0;
	}
	req = mtk_imgsys_media_req_to_imgsys_req(dev_buf->vbb.vb2_buf.request);
	spin_lock(&pipe->iova_cache.lock);
	cached = mtk_imgsys_iova_find(pipe, buffer);
	if (cached)
		kref_get(&cached->refcount);
	spin_unlock(&pipe->iova_cache.lock);
	if (cached) {
		/* Every cache hit owns an explicit frame lease. DEL/streamoff only
		 * drops cache ownership; the attachment survives until frame release.
		 */
		entry = vzalloc(sizeof(*entry));
		dma_buf_put(buffer);
		if (!entry) {
			kref_put(&cached->refcount, mtk_imgsys_iova_release);
			return 0;
		}
		kref_init(&entry->refcount);
		entry->cached = cached;
		entry->dma_addr = cached->dma_addr;
	} else {
		entry = mtk_imgsys_iova_create(pipe->imgsys_dev->dev, buffer, fd);
		if (IS_ERR(entry))
			return 0;
	}
	address = entry->dma_addr;
	kref_get(&entry->refcount);
	mutex_lock(&req->mapping_lock);
	list_add_tail(&entry->request_link, &req->iova_leases);
	mutex_unlock(&req->mapping_lock);
	spin_lock(&dev_buf->iova_map_table.lock);
	list_add_tail(&entry->list_entry, &dev_buf->iova_map_table.list);
	spin_unlock(&dev_buf->iova_map_table.lock);
	return address;
}

void mtk_imgsys_release_iova_table(struct mtk_imgsys_dev_buffer *buffer)
{
	struct mtk_imgsys_dma_buf_iova_get_info *entry, *next;
	LIST_HEAD(retired);

	spin_lock(&buffer->iova_map_table.lock);
	list_splice_init(&buffer->iova_map_table.list, &retired);
	spin_unlock(&buffer->iova_map_table.lock);
	list_for_each_entry_safe(entry, next, &retired, list_entry) {
		list_del(&entry->list_entry);
		mtk_imgsys_iova_put(entry);
	}
}

void mtk_imgsys_put_dma_buf(struct dma_buf *dma_buf,
				struct dma_buf_attachment *attach,
				struct sg_table *sgt)
{
	if (!IS_ERR(dma_buf)) {
		dma_buf_unmap_attachment_unlocked(attach, sgt,
			DMA_BIDIRECTIONAL);
		dma_buf_detach(dma_buf, attach);
		dma_buf_put(dma_buf);
	}
}

/* Cache ownership and vb2 ownership are independent. Cache retirement must not
 * invalidate descriptor mappings still leased by queued/active buffers.
 */
static void put_kva(struct buf_va_info_t *entry)
{
	struct dma_buf *buffer = entry->dma_buf_putkva;

	if (IS_ERR_OR_NULL(buffer))
		return;
	if (entry->kva) {
		mtk_imgsys_vunmap(buffer, (void *)entry->kva);
		dma_buf_end_cpu_access(buffer, DMA_BIDIRECTIONAL);
	}
	if (!IS_ERR_OR_NULL(entry->attach)) {
		if (!IS_ERR_OR_NULL(entry->sgt))
			dma_buf_unmap_attachment_unlocked(entry->attach, entry->sgt,
				DMA_BIDIRECTIONAL);
		dma_buf_detach(buffer, entry->attach);
	}
	dma_buf_put(buffer);
	entry->dma_buf_putkva = NULL;
	entry->kva = 0;
	entry->attach = NULL;
	entry->sgt = NULL;
}

static void mtk_imgsys_kva_release(struct kref *ref)
{
	struct buf_va_info_t *entry = container_of(ref, struct buf_va_info_t, refcount);

	put_kva(entry);
	put_device(entry->owner);
	vfree(entry);
}

void mtk_imgsys_kva_put(struct buf_va_info_t *entry)
{
	if (entry)
		kref_put(&entry->refcount, mtk_imgsys_kva_release);
}

/* Consumes buffer on both success and failure. The returned reference belongs
 * to a pending/cache list; callers take additional references for vb2 leases.
 */
static struct buf_va_info_t *mtk_imgsys_kva_create(struct device *dev,
			struct dma_buf *buffer, int fd, bool attach)
{
	struct buf_va_info_t *entry;
	void *address;
	int ret;

	entry = vzalloc(sizeof(vlist_type(struct buf_va_info_t)));
	if (!entry) {
		dma_buf_put(buffer);
		return ERR_PTR(-ENOMEM);
	}
	INIT_LIST_HEAD(vlist_link(entry, struct buf_va_info_t));
	kref_init(&entry->refcount);
	entry->owner = get_device(dev);
	entry->buf_fd = fd;
	entry->dma_buf_putkva = buffer;
	ret = dma_buf_begin_cpu_access(buffer, DMA_BIDIRECTIONAL);
	if (ret)
		goto fail;
	address = mtk_imgsys_vmap(buffer);
	if (IS_ERR_OR_NULL(address)) {
		ret = IS_ERR(address) ? PTR_ERR(address) : -ENOMEM;
		dma_buf_end_cpu_access(buffer, DMA_BIDIRECTIONAL);
		goto fail;
	}
	entry->kva = (unsigned long)address;
	if (attach) {
		entry->attach = dma_buf_attach(buffer, dev);
		if (IS_ERR(entry->attach)) {
			ret = PTR_ERR(entry->attach);
			goto fail;
		}
		entry->sgt = dma_buf_map_attachment_unlocked(entry->attach,
			DMA_BIDIRECTIONAL);
		if (IS_ERR(entry->sgt)) {
			ret = PTR_ERR(entry->sgt);
			goto fail;
		}
		entry->dma_addr = sg_dma_address(entry->sgt->sgl);
	}
	return entry;
fail:
	mtk_imgsys_kva_put(entry);
	return ERR_PTR(ret);
}

static struct buf_va_info_t *mtk_imgsys_kva_find(struct list_head *list,
			struct device *owner, struct dma_buf *buffer)
{
	struct list_head *ptr;
	struct buf_va_info_t *entry;

	list_for_each(ptr, list) {
		entry = vlist_node_of(ptr, struct buf_va_info_t);
		if (entry->owner == owner && entry->dma_buf_putkva == buffer)
			return entry;
	}
	return NULL;
}

void flush_fd_kva_list(struct mtk_imgsys_dev *imgsys_dev)
{
	struct list_head *ptr, *next;
	struct buf_va_info_t *entry;
	LIST_HEAD(retired);

	mutex_lock(&fd_kva_info_list.mymutex);
	list_for_each_safe(ptr, next, &fd_kva_info_list.mylist) {
		entry = vlist_node_of(ptr, struct buf_va_info_t);
		if (entry->owner != imgsys_dev->dev)
			continue;
		list_move_tail(ptr, &retired);
	}
	mutex_unlock(&fd_kva_info_list.mymutex);
	while (!list_empty(&retired)) {
		entry = vlist_node_of(retired.next, struct buf_va_info_t);
		list_del_init(vlist_link(entry, struct buf_va_info_t));
		mtk_imgsys_kva_put(entry);
	}
}

struct buf_va_info_t *get_first_sd_buf(struct mtk_imgsys_dev *imgsys_dev)
{
	struct buf_va_info_t *entry, *result = NULL;
	struct dma_buf *buffer;
	struct list_head *ptr;

	mutex_lock(&fd_kva_info_list.mymutex);
	list_for_each(ptr, &fd_kva_info_list.mylist) {
		entry = vlist_node_of(ptr, struct buf_va_info_t);
		if (entry->owner != imgsys_dev->dev)
			continue;
		/* This legacy init record sends an FD, not a kernel pointer. Do not
		 * report an FD recycled in the caller's namespace as the old object.
		 */
		buffer = dma_buf_get(entry->buf_fd);
		if (IS_ERR(buffer))
			continue;
		if (entry->dma_buf_putkva == buffer) {
			kref_get(&entry->refcount);
			result = entry;
		}
		dma_buf_put(buffer);
		if (result)
			break;
	}
	mutex_unlock(&fd_kva_info_list.mymutex);
	return result;
}

int mtk_imgsys_add_kva(struct mtk_imgsys_pipe *pipe, struct fd_info *info)
{
	struct device *dev = pipe->imgsys_dev->dev;
	struct buf_va_info_t *entry, *old;
	struct dma_buf *buffer;
	struct list_head *ptr;
	LIST_HEAD(pending);
	LIST_HEAD(retired);
	int i, ret = 0;

	if (info->fd_num > ARRAY_SIZE(info->fds))
		return -EINVAL;
	memset(info->fds_size, 0, sizeof(info->fds_size));
	mutex_lock(&fd_kva_info_list.mymutex);
	for (i = 0; i < info->fd_num; i++) {
		/* Preserve the HAL's zero-FD unused-slot sentinel. */
		if (!info->fds[i])
			continue;
		buffer = dma_buf_get(info->fds[i]);
		if (IS_ERR(buffer))
			continue;
		entry = mtk_imgsys_kva_find(&pending, dev, buffer);
		if (!entry)
			entry = mtk_imgsys_kva_find(&fd_kva_info_list.mylist, dev, buffer);
		if (entry && entry->attach) {
			entry->buf_fd = info->fds[i];
			info->fds_size[i] = buffer->size;
			dma_buf_put(buffer);
			continue;
		}
		entry = mtk_imgsys_kva_create(dev, buffer, info->fds[i], true);
		if (IS_ERR(entry)) {
			if (PTR_ERR(entry) == -ENOMEM) {
				ret = -ENOMEM;
				goto rollback;
			}
			continue;
		}
		info->fds_size[i] = buffer->size;
		list_add_tail(vlist_link(entry, struct buf_va_info_t), &pending);
	}
	/* Upgrade CPU-only cache entries transactionally. Existing vb2 leases
	 * retain the retired mapping until their own finish/cleanup callback.
	 */
	list_for_each(ptr, &pending) {
		entry = vlist_node_of(ptr, struct buf_va_info_t);
		old = mtk_imgsys_kva_find(&fd_kva_info_list.mylist, dev,
			entry->dma_buf_putkva);
		if (old)
			list_move_tail(vlist_link(old, struct buf_va_info_t), &retired);
	}
	list_splice_tail_init(&pending, &fd_kva_info_list.mylist);
	mutex_unlock(&fd_kva_info_list.mymutex);
	goto release;
rollback:
	list_splice_tail_init(&pending, &retired);
	mutex_unlock(&fd_kva_info_list.mymutex);
	memset(info->fds_size, 0, sizeof(info->fds_size));
release:
	while (!list_empty(&retired)) {
		entry = vlist_node_of(retired.next, struct buf_va_info_t);
		list_del_init(vlist_link(entry, struct buf_va_info_t));
		mtk_imgsys_kva_put(entry);
	}
	return ret;
}

int mtk_imgsys_del_kva(struct mtk_imgsys_pipe *pipe, struct fd_info *info)
{
	struct buf_va_info_t *entry;
	struct dma_buf *buffer;
	int i;

	if (info->fd_num > ARRAY_SIZE(info->fds))
		return -EINVAL;
	memset(info->fds_size, 0, sizeof(info->fds_size));
	for (i = 0; i < info->fd_num; i++) {
		if (!info->fds[i])
			continue;
		buffer = dma_buf_get(info->fds[i]);
		if (IS_ERR(buffer))
			continue;
		mutex_lock(&fd_kva_info_list.mymutex);
		entry = mtk_imgsys_kva_find(&fd_kva_info_list.mylist,
			pipe->imgsys_dev->dev, buffer);
		if (entry) {
			list_del_init(vlist_link(entry, struct buf_va_info_t));
			info->fds_size[i] = buffer->size;
		}
		mutex_unlock(&fd_kva_info_list.mymutex);
		mtk_imgsys_kva_put(entry);
		dma_buf_put(buffer);
	}
	return 0;
}

void mtk_imgsys_kva_finish(struct mtk_imgsys_dev_buffer *buffer)
{
	struct buf_va_info_t *entry;

	mutex_lock(&fd_kva_info_list.mymutex);
	entry = buffer->kva_entry;
	buffer->kva_entry = NULL;
	if (entry) {
		buffer->va_daddr[0] = 0;
		buffer->dma_buf_putkva = NULL;
	}
	mutex_unlock(&fd_kva_info_list.mymutex);
	mtk_imgsys_kva_put(entry);
}

/* Raw descriptor pointers are firmware ABI fields, not reference owners. */
struct mtk_imgsys_dmabuf_lease {
	struct list_head link;
	struct dma_buf *buffer;
};

/* Called only while validating in the submitting task's FD namespace. */
static struct dma_buf *mtk_imgsys_desc_pin(struct mtk_imgsys_request *req, int fd)
{
	struct mtk_imgsys_dmabuf_lease *lease;
	struct dma_buf *buffer = dma_buf_get(fd);

	if (IS_ERR(buffer))
		return buffer;
	mutex_lock(&req->mapping_lock);
	list_for_each_entry(lease, &req->dmabuf_leases, link) {
		if (lease->buffer == buffer) {
			mutex_unlock(&req->mapping_lock);
			dma_buf_put(buffer);
			return buffer;
		}
	}
	lease = kzalloc(sizeof(*lease), GFP_KERNEL);
	if (!lease) {
		mutex_unlock(&req->mapping_lock);
		dma_buf_put(buffer);
		return ERR_PTR(-ENOMEM);
	}
	lease->buffer = buffer;
	list_add_tail(&lease->link, &req->dmabuf_leases);
	mutex_unlock(&req->mapping_lock);
	return buffer;
}

/* Compare a shared descriptor cookie without dereferencing it first. */
static struct dma_buf *mtk_imgsys_desc_get(struct mtk_imgsys_request *req, u64 cookie)
{
	struct mtk_imgsys_dmabuf_lease *lease;
	struct dma_buf *buffer = NULL;

	mutex_lock(&req->mapping_lock);
	list_for_each_entry(lease, &req->dmabuf_leases, link) {
		if ((u64)lease->buffer == cookie) {
			buffer = lease->buffer;
			get_dma_buf(buffer);
			break;
		}
	}
	mutex_unlock(&req->mapping_lock);
	return buffer;
}

static void mtk_imgsys_desc_fill_dmabuf(struct mtk_imgsys_pipe *pipe,
				struct frameparams *fparams,
				bool isMENode,
				struct mtk_imgsys_request *req)
{
	unsigned int i, j;
	struct dma_buf *buffer;
	struct v4l2_ext_plane *plane;

	/* ME's borrowed-pointer ABI needs the same lifetime as mapped DMA nodes. */
	(void)pipe;
	(void)isMENode;
	for (i = 0; i < FRAME_BUF_MAX; i++) {
		for (j = 0; j < IMGBUF_MAX_PLANES; j++) {
			plane = &fparams->bufs[i].buf.planes[j];
			plane->reserved[0] = 0;
			plane->reserved[1] = 0;
			if (!plane->m.dma_buf.fd)
				continue;
			buffer = mtk_imgsys_desc_pin(req, plane->m.dma_buf.fd);
			if (IS_ERR(buffer)) {
				if (!req->descriptor_error)
					req->descriptor_error = PTR_ERR(buffer);
				continue;
			}
			if (plane->m.dma_buf.offset >= buffer->size) {
				req->descriptor_error = -EINVAL;
				continue;
			}
			plane->reserved[0] = (u64)buffer;
			plane->reserved[1] = buffer->size;
		}
	}
}

struct mtk_imgsys_kva_lease {
	struct list_head link;
	struct buf_va_info_t *entry;
};

void mtk_imgsys_request_mappings_init(struct mtk_imgsys_request *req)
{
	mutex_init(&req->mapping_lock);
	INIT_LIST_HEAD(&req->kva_leases);
	INIT_LIST_HEAD(&req->iova_leases);
	INIT_LIST_HEAD(&req->dmabuf_leases);
	req->descriptor_error = 0;
}

static bool mtk_imgsys_kva_request_get(struct mtk_imgsys_request *req,
				     struct buf_va_info_t *entry)
{
	struct mtk_imgsys_kva_lease *lease;

	mutex_lock(&req->mapping_lock);
	list_for_each_entry(lease, &req->kva_leases, link) {
		if (lease->entry == entry) {
			mutex_unlock(&req->mapping_lock);
			return true;
		}
	}
	lease = kzalloc(sizeof(*lease), GFP_KERNEL);
	if (!lease) {
		mutex_unlock(&req->mapping_lock);
		return false;
	}
	kref_get(&entry->refcount);
	lease->entry = entry;
	list_add_tail(&lease->link, &req->kva_leases);
	mutex_unlock(&req->mapping_lock);
	return true;
}

void mtk_imgsys_request_mappings_release(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_kva_lease *lease, *next;
	struct mtk_imgsys_dmabuf_lease *raw, *raw_next;
	struct mtk_imgsys_dma_buf_iova_get_info *iova, *tmp;
	LIST_HEAD(kvas);
	LIST_HEAD(iovas);
	LIST_HEAD(raws);

	/* Release cache/DMA references outside the request lock. A request may
	 * outlive vb2 cancellation while a driver/daemon completion holds it.
	 * Reinit/free, after request completion, retires these final safety refs.
	 */
	mutex_lock(&req->mapping_lock);
	list_splice_init(&req->kva_leases, &kvas);
	list_splice_init(&req->iova_leases, &iovas);
	list_splice_init(&req->dmabuf_leases, &raws);
	mutex_unlock(&req->mapping_lock);
	list_for_each_entry_safe(lease, next, &kvas, link) {
		list_del(&lease->link);
		mtk_imgsys_kva_put(lease->entry);
		kfree(lease);
	}
	list_for_each_entry_safe(iova, tmp, &iovas, request_link) {
		list_del(&iova->request_link);
		mtk_imgsys_iova_put(iova);
	}
	list_for_each_entry_safe(raw, raw_next, &raws, link) {
		list_del(&raw->link);
		dma_buf_put(raw->buffer);
		kfree(raw);
	}
}

static void mtk_imgsys_kva_cache(struct mtk_imgsys_dev_buffer *dev_buf,
			       struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_pipe *pipe = vb2_get_drv_priv(dev_buf->vbb.vb2_buf.vb2_queue);
	struct dma_buf *buffer = dev_buf->vbb.vb2_buf.planes[0].dbuf;
	struct buf_va_info_t *entry;

	/* vb2 imported this exact object in the submitting task. Never resolve
	 * the userspace FD again in an asynchronous kernel worker's files table.
	 */
	dev_buf->va_daddr[0] = 0;
	dev_buf->dma_buf_putkva = NULL;
	if (IS_ERR_OR_NULL(buffer) || dev_buf->dataofst >= buffer->size)
		return;
	mutex_lock(&fd_kva_info_list.mymutex);
	entry = dev_buf->kva_entry;
	if (entry && (entry->dma_buf_putkva != buffer ||
		      entry->owner != pipe->imgsys_dev->dev)) {
		dev_buf->kva_entry = NULL;
		mtk_imgsys_kva_put(entry);
		entry = NULL;
	}
	if (!entry) {
		entry = mtk_imgsys_kva_find(&fd_kva_info_list.mylist,
			pipe->imgsys_dev->dev, buffer);
		if (!entry) {
			get_dma_buf(buffer);
			entry = mtk_imgsys_kva_create(pipe->imgsys_dev->dev, buffer,
				dev_buf->vbb.vb2_buf.planes[0].m.fd, false);
			if (IS_ERR(entry))
				goto unlock;
			list_add_tail(vlist_link(entry, struct buf_va_info_t),
				&fd_kva_info_list.mylist);
		}
		kref_get(&entry->refcount);
		dev_buf->kva_entry = entry;
	}
	if (!mtk_imgsys_kva_request_get(req, entry))
		goto unlock;
	dev_buf->dma_buf_putkva = buffer;
	dev_buf->va_daddr[0] = entry->kva + dev_buf->dataofst;
unlock:
	mutex_unlock(&fd_kva_info_list.mymutex);
}

static void mtk_imgsys_desc_iova(struct mtk_imgsys_pipe *pipe,
				struct frameparams *fparams,
				struct mtk_imgsys_dev_buffer *dev_buf)
{
	struct device *dev = pipe->imgsys_dev->dev;
	unsigned int i = 0, j = 0;
	struct dma_buf *dmabuf;
	struct mtk_imgsys_request *req = mtk_imgsys_media_req_to_imgsys_req(
		dev_buf->vbb.vb2_buf.request);

	IMGSYS_SYSTRACE_BEGIN("%s\n", __func__);

	for (i = 0; i < FRAME_BUF_MAX; i++) {
		for (j = 0; j < IMGBUF_MAX_PLANES; j++) {
			if (fparams->bufs[i].buf.planes[j].m.dma_buf.fd == 0)
				continue;

			dmabuf = mtk_imgsys_desc_get(req,
			fparams->bufs[i].buf.planes[j].reserved[0]);
			if (IS_ERR_OR_NULL(dmabuf)) {
				req->descriptor_error = -EINVAL;
				fparams->bufs[i].buf.planes[j].reserved[0] = 0;
				continue;
			}
			if (fparams->bufs[i].buf.planes[j].m.dma_buf.offset >= dmabuf->size) {
				dma_buf_put(dmabuf);
				req->descriptor_error = -EINVAL;
				continue;
			}
			fparams->bufs[i].buf.planes[j].reserved[0] =
			mtk_imgsys_get_iova(dmabuf,
			fparams->bufs[i].buf.planes[j].m.dma_buf.fd,
					pipe, dev_buf);
			if (!fparams->bufs[i].buf.planes[j].reserved[0])
				req->descriptor_error = -EIO;
			dev_dbg(dev,
				"%s - bufs[%d].buf.planes[%d]: fd(%d), iova(%llx)\n",
				__func__, i, j,
			fparams->bufs[i].buf.planes[j].m.dma_buf.fd,
				fparams->bufs[i].buf.planes[j].reserved[0]);
		}
	}

	IMGSYS_SYSTRACE_END();

}

/* ME metadata keeps kernel pointers rather than converting them to IOVAs. */
static void mtk_imgsys_desc_me(struct mtk_imgsys_dev_buffer *dev_buf,
			       struct frameparams *fparams)
{
	struct mtk_imgsys_request *req = mtk_imgsys_media_req_to_imgsys_req(
		dev_buf->vbb.vb2_buf.request);
	unsigned int i, j;

	for (i = 0; i < FRAME_BUF_MAX; i++) {
		for (j = 0; j < IMGBUF_MAX_PLANES; j++) {
			struct v4l2_ext_plane *plane = &fparams->bufs[i].buf.planes[j];
			struct dma_buf *buffer;

			if (!plane->m.dma_buf.fd) {
				plane->reserved[0] = 0;
				plane->reserved[1] = 0;
				continue;
			}
			buffer = mtk_imgsys_desc_get(req, plane->reserved[0]);
			if (!buffer) {
				req->descriptor_error = -EINVAL;
				plane->reserved[0] = 0;
				plane->reserved[1] = 0;
				continue;
			}
			if (plane->m.dma_buf.offset >= buffer->size)
				req->descriptor_error = -EINVAL;
			plane->reserved[1] = buffer->size;
			dma_buf_put(buffer);
		}
	}
}

static bool mtk_imgsys_desc_size_valid(struct mtk_imgsys_request *req,
				      struct mtk_imgsys_dev_buffer *buffer)
{
	struct dma_buf *dbuf = buffer->vbb.vb2_buf.planes[0].dbuf;
	size_t size;

	switch (buffer->dev_fmt->format) {
	case V4L2_META_FMT_MTISP_DESC: size = sizeof(struct header_desc); break;
	case V4L2_META_FMT_MTISP_DESC_NORM: size = sizeof(struct header_desc_norm); break;
	case V4L2_META_FMT_MTISP_SD: size = sizeof(struct singlenode_desc); break;
	case V4L2_META_FMT_MTISP_SDNORM: size = sizeof(struct singlenode_desc_norm); break;
	default: req->descriptor_error = -EINVAL; return false;
	}
	if (IS_ERR_OR_NULL(dbuf) || buffer->dataofst > dbuf->size ||
	    size > dbuf->size - buffer->dataofst) {
		req->descriptor_error = -EINVAL;
		return false;
	}
	return true;
}

static void mtk_imgsys_desc_fill_ipi_param(struct mtk_imgsys_pipe *pipe,
					struct mtk_imgsys_dev_buffer *dev_buf,
					struct mtk_imgsys_request *req,
					bool isMENode)
{
	struct device *dev = pipe->imgsys_dev->dev;
	struct header_desc *desc_dma = NULL;
	struct header_desc_norm *desc_norm = NULL;
	unsigned int i = 0, j = 0, tnum, tmax;

	if (dev_buf->vbb.vb2_buf.memory == VB2_MEMORY_DMABUF) {
		{
			dev_dbg(dev,
			"%s : fd(%d)\n",
				__func__, dev_buf->vbb.vb2_buf.planes[0].m.fd);

			mtk_imgsys_kva_cache(dev_buf, req);
		}
	}
	/* TODO:  MMAP imgbuffer path vaddr = 0 */

	dev_dbg(dev,
		"%s : scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
		__func__,
		dev_buf->scp_daddr[0],
		dev_buf->isp_daddr[0],
		dev_buf->va_daddr[0]);

	if (!mtk_imgsys_desc_size_valid(req, dev_buf))
		return;

	/*getKVA*/
	if (!dev_buf->va_daddr[0]) {
		req->descriptor_error = -ENOMEM;
		return;
	}
	//init map table list for each devbuf

	switch (dev_buf->dev_fmt->format) {
	/* For SMVR */
	default:
	case V4L2_META_FMT_MTISP_DESC:
		desc_dma = (struct header_desc *)dev_buf->va_daddr[0];
		tnum = desc_dma->fparams_tnum;
		tmax = TIME_MAX;
		break;
	/* For Normal */
	case V4L2_META_FMT_MTISP_DESC_NORM:
		desc_norm = (struct header_desc_norm *)dev_buf->va_daddr[0];
		tnum = desc_norm->fparams_tnum;
		tmax = TMAX;
		break;
	}

	/*fd2iova : need 2D array parsing*/
	if (tnum >= tmax) {
		tnum = tmax;
		pr_debug("%s: %d bufinfo enqueued exceeds %d\n", __func__,
			tnum, tmax);
	}

	if (desc_norm) {
		for (i = 0; i < tnum ; i++) {
			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&desc_norm->fparams[i][j], isMENode, req);
		}
	} else {
		for (i = 0; i < tnum ; i++) {
			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&desc_dma->fparams[i][j], isMENode, req);
		}
	}

}

void mtk_imgsys_desc_ipi_params_config(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct device *dev = pipe->imgsys_dev->dev;
	struct mtk_imgsys_dev_buffer *buf_dma;
	int i = 0;
	bool isMENode = false;
	dev_dbg(dev, "%s:%s: pipe-job id(%d)\n", __func__,
		pipe->desc->name, req->id);

	for (i = 0; i < pipe->desc->total_queues; i++) {
		buf_dma = req->buf_map[i];

		if (i == MTK_IMGSYS_VIDEO_NODE_TUNING_OUT) {
			if (!buf_dma) {
				dev_dbg(dev, "No enqueued tuning buffer\n");
				continue;
			}

		} else if (i == MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT)
			isMENode = true;

		if (buf_dma) {

			mtk_imgsys_desc_fill_ipi_param(pipe,
				buf_dma,
				req, isMENode);
		}
	}

}

static void mtk_imgsys_desc_set(struct mtk_imgsys_pipe *pipe,
				void *src,
				struct header_desc *dst,
				bool iova_need,
				struct mtk_imgsys_dev_buffer *dev_buf)
{

	unsigned int tmax, tnum, t, j;
	struct header_desc *input_smvr = NULL;
	struct header_desc_norm *input_norm = NULL;

	switch (dev_buf->dev_fmt->format) {
	case V4L2_META_FMT_MTISP_SD:
	case V4L2_META_FMT_MTISP_DESC:
		input_smvr = (struct header_desc *)src;
		tmax = TIME_MAX;
		tnum = input_smvr->fparams_tnum;
		break;
	case V4L2_META_FMT_MTISP_SDNORM:
	case V4L2_META_FMT_MTISP_DESC_NORM:
	default:
		input_norm = (struct header_desc_norm *)src;
		tmax = TMAX;
		tnum = input_norm->fparams_tnum;
		break;
	}
	/*fd2iova : need 2D array parsing*/
	if (tnum >= tmax) {
		tnum = tmax;
		pr_debug("%s: %d bufinfo enqueued exceeds %d\n", __func__,
			tnum, tmax);
	}

	if (input_smvr) {
		for (t = 0; t < tnum; t++) {
			for (j = 0; j < SCALE_MAX; j++) {
				if (iova_need)
					mtk_imgsys_desc_iova(pipe,
					&input_smvr->fparams[t][j], dev_buf);
				else
					mtk_imgsys_desc_me(dev_buf,
					&input_smvr->fparams[t][j]);
				/* TODO: disable memcpy for imgstream fd */
				if (!dst)
					continue;
				memcpy(&dst->fparams[t][j],
						&input_smvr->fparams[t][j],
						sizeof(struct frameparams));
			}
		}
		if (dst)
			dst->fparams_tnum = tnum;
	} else {
		for (t = 0; t < tnum; t++) {
			for (j = 0; j < SCALE_MAX; j++) {
				if (iova_need)
					mtk_imgsys_desc_iova(pipe,
					&input_norm->fparams[t][j], dev_buf);
				else
					mtk_imgsys_desc_me(dev_buf,
					&input_norm->fparams[t][j]);
				/* TODO: disable memcpy for imgstream fd */
				if (!dst)
					continue;
				memcpy(&dst->fparams[t][j],
						&input_norm->fparams[t][j],
						sizeof(struct frameparams));
			}
		}
		if (dst)
			dst->fparams_tnum = tnum;
	}
}

static void mtk_imgsys_desc_set_skip(struct mtk_imgsys_pipe *pipe,
				int dma,
				void *src,
				struct header_desc *dst,
				bool iova_need,
				struct mtk_imgsys_dev_buffer *dev_buf)
{

	unsigned int tmax, tnum, t, j;
	struct header_desc *input_smvr = NULL;
	struct header_desc_norm *input_norm = NULL;
	struct singlenode_desc *desc_sd = NULL;
	struct singlenode_desc_norm *desc_sd_norm = NULL;

	switch (dev_buf->dev_fmt->format) {
	case V4L2_META_FMT_MTISP_SD:
		desc_sd = (struct singlenode_desc *)dev_buf->va_daddr[0];
		input_smvr = (struct header_desc *)src;
		tmax = TIME_MAX;
		tnum = input_smvr->fparams_tnum;
		break;
	case V4L2_META_FMT_MTISP_SDNORM:
	default:
		desc_sd_norm =
			(struct singlenode_desc_norm *)dev_buf->va_daddr[0];
		input_norm = (struct header_desc_norm *)src;
		tmax = TMAX;
		tnum = input_norm->fparams_tnum;
		break;
	}
	/*fd2iova : need 2D array parsing*/
	if (tnum >= tmax) {
		tnum = tmax;
		pr_debug("%s: %d bufinfo enqueued exceeds %d\n", __func__,
			tnum, tmax);
	}

	if (input_smvr) {
		for (t = 0; t < tnum; t++) {
			if (!desc_sd->dmas_enable[dma][t])
				continue;
			for (j = 0; j < SCALE_MAX; j++) {
				if (iova_need)
					mtk_imgsys_desc_iova(pipe,
					&input_smvr->fparams[t][j], dev_buf);
				else
					mtk_imgsys_desc_me(dev_buf,
					&input_smvr->fparams[t][j]);
				/* TODO: disable memcpy for imgstream fd */
				if (!dst)
					continue;
				memcpy(&dst->fparams[t][j],
						&input_smvr->fparams[t][j],
						sizeof(struct frameparams));
			}
		}
		if (dst)
			dst->fparams_tnum = tnum;
	} else {
		for (t = 0; t < tnum; t++) {
			if (!desc_sd_norm->dmas_enable[dma][t])
				continue;

			for (j = 0; j < SCALE_MAX; j++) {
				if (iova_need)
					mtk_imgsys_desc_iova(pipe,
					&input_norm->fparams[t][j], dev_buf);
				else
					mtk_imgsys_desc_me(dev_buf,
					&input_norm->fparams[t][j]);
				/* TODO: disable memcpy for imgstream fd */
				if (!dst)
					continue;
				memcpy(&dst->fparams[t][j],
						&input_norm->fparams[t][j],
						sizeof(struct frameparams));
			}
		}
		if (dst)
			dst->fparams_tnum = tnum;
	}
}


void mtk_imgsys_desc_map_iova(struct mtk_imgsys_request *req)
{
	unsigned int i;
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct mtk_imgsys_dev_buffer *buf_dma;
	struct img_ipi_frameparam *dip_param = req->working_buf->frameparam.vaddr;
	struct header_desc *desc;
	void *desc_dma = NULL;
	bool need_iova = true;
	unsigned int s, e;

	s = ktime_get_boottime_ns()/1000;
	for (i = 0; i < pipe->desc->total_queues; i++) {

		buf_dma = req->buf_map[i];
		if (!buf_dma)
			continue;

		if (!buf_dma->va_daddr[0]) {
			pr_info("%s fail!! desc_dma is NULL !", __func__);
			return;
		}

		desc = NULL;
		/* TODO: desc mode */
		/* if (dip_param) */
		/*	dip_param->dmas_enable[i] = 1; */

		if (i == MTK_IMGSYS_VIDEO_NODE_TUNING_OUT) {
			if (dip_param)
				desc = &dip_param->tuning_meta;
			need_iova = true;
		} else if (i == MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT) {
			if (dip_param)
				desc = &dip_param->ctrl_meta;
			need_iova = false;
		} else {
			if (dip_param)
				desc = &dip_param->dmas[i];
			need_iova = true;
		}

		desc_dma = (void *)buf_dma->va_daddr[0];
		mtk_imgsys_desc_set(pipe, desc_dma, desc, need_iova, buf_dma);
	}
	e = ktime_get_boottime_ns()/1000;
	pr_debug("%s takes %d ms\n", __func__, (e - s));
}

void mtk_imgsys_sd_desc_map_iova(struct mtk_imgsys_request *req)
{
	unsigned int i;
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct mtk_imgsys_dev_buffer *buf_sd;
	struct img_ipi_frameparam *dip_param = req->working_buf->frameparam.vaddr;
	struct singlenode_desc *desc_sd = NULL;
	struct singlenode_desc_norm *desc_sd_norm = NULL;
	void *src;
	struct header_desc *dst;
	int b;

	b = is_singledev_mode(req);
	buf_sd = req->buf_map[b];
	if (!buf_sd)
		return;

	if (!buf_sd->va_daddr[0]) {
		pr_info("%s fail!! desc_sd is NULL !", __func__);
		return;
	}

	switch (buf_sd->dev_fmt->format) {
	default:
	case V4L2_META_FMT_MTISP_SD:
		desc_sd = (struct singlenode_desc *)buf_sd->va_daddr[0];
		break;
	case V4L2_META_FMT_MTISP_SDNORM:
		desc_sd_norm =
			(struct singlenode_desc_norm *)buf_sd->va_daddr[0];
		break;
	}

	for (i = 0; i < IMG_MAX_HW_DMAS; i++) {

		if (desc_sd)
			src = (void *)&desc_sd->dmas[i];
		else
			src = (void *)&desc_sd_norm->dmas[i];

		if (!dip_param)
			dst = NULL;
		else {
			dst = &dip_param->dmas[i];
			/* TODO: desc mode */
			/* dip_param->dmas_enable[i] = 1; */
		}

		mtk_imgsys_desc_set_skip(pipe, i, src, dst, 1, buf_sd);
	}

	/* tuning */
	if (desc_sd)
		src = (void *)&desc_sd->tuning_meta;
	else
		src = (void *)&desc_sd_norm->tuning_meta;

	if (!dip_param)
		dst = NULL;
	else
		dst = &dip_param->tuning_meta;
	mtk_imgsys_desc_set_skip(pipe,
		MTK_IMGSYS_VIDEO_NODE_TUNING_OUT, src, dst, 1, buf_sd);

	/* ctrl_meta */
	if (desc_sd)
		src = (void *)&desc_sd->ctrl_meta;
	else
		src = (void *)&desc_sd_norm->ctrl_meta;

	if (!dip_param)
		dst = NULL;
	else
		dst = &dip_param->ctrl_meta;
	mtk_imgsys_desc_set(pipe, src, dst, 0, buf_sd);

}


static void mtk_imgsys_std2desc_fill_bufinfo(struct mtk_imgsys_pipe *pipe,
					struct header_desc *ipidma,
					struct mtk_imgsys_dev_buffer *dev_buf)
{
	int i = 0;
#ifndef USE_V4L2_FMT
	struct v4l2_plane_pix_format *vfmt;
	struct plane_pix_format *bfmt;

	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.width =
						dev_buf->fmt.fmt.pix_mp.width;
	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.height =
						dev_buf->fmt.fmt.pix_mp.height;
	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.pixelformat =
					dev_buf->fmt.fmt.pix_mp.pixelformat;

	for (i = 0; i < IMGBUF_MAX_PLANES; i++) {
		vfmt = &dev_buf->fmt.fmt.pix_mp.plane_fmt[i];
		bfmt =
		&ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.plane_fmt[i];
		bfmt->sizeimage = vfmt->sizeimage;
		bfmt->bytesperline = vfmt->sizeimage;
	}
#else
	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp = dev_buf->fmt.fmt.pix_mp;
#endif
	ipidma->fparams[0][0].bufs[0].crop = dev_buf->crop;
	/* ipidma->fparams[0][0].bufs[0].compose = dev_buf->compose;*/
	ipidma->fparams[0][0].bufs[0].rotation = dev_buf->rotation;
	ipidma->fparams[0][0].bufs[0].hflip = dev_buf->hflip;
	ipidma->fparams[0][0].bufs[0].vflip = dev_buf->vflip;

	dev_dbg(pipe->imgsys_dev->dev,
		"[%s] rotat(%d), hflip(%d), vflip(%d)\n",
		__func__,
		ipidma->fparams[0][0].bufs[0].rotation,
		ipidma->fparams[0][0].bufs[0].hflip,
		ipidma->fparams[0][0].bufs[0].vflip);

	for (i = 0; i < dev_buf->fmt.fmt.pix_mp.num_planes; i++) {
		dev_dbg(pipe->imgsys_dev->dev,
			"[%s] multi-planes : width(%d), width(%d), pixelformat(%d), sizeimage(%d), bytesperline(%d)\n",
			__func__,
		ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.width,
		ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.height,
		ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.pixelformat,
	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.plane_fmt[i].sizeimage,
	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.plane_fmt[i].bytesperline);
	}
}

static void mtk_imgsys_meta_std2desc_fill_extbuf(struct mtk_imgsys_pipe *pipe,
					struct header_desc *ipidma,
					struct mtk_imgsys_dev_buffer *dev_buf)
{
	struct device *dev = pipe->imgsys_dev->dev;

	dev_dbg(dev,
		"%s : scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
		__func__,
		dev_buf->scp_daddr[0],
		dev_buf->isp_daddr[0],
		dev_buf->va_daddr[0]);

	ipidma->fparams[0][0].bufs[0].buf.planes[0].m.dma_buf.fd =
		dev_buf->vbb.vb2_buf.planes[0].m.fd;
	ipidma->fparams[0][0].bufs[0].buf.planes[0].m.dma_buf.offset =
		dev_buf->vbb.vb2_buf.planes[0].m.offset;
	ipidma->fparams[0][0].bufs[0].buf.planes[0].reserved[0] =
		(uint32_t)dev_buf->isp_daddr[0];
	dev_dbg(dev,
		"[%s]multi-plane[%d]:fd(%d),offset(%d),iova(0x%llx)\n",
		__func__, 0,
	ipidma->fparams[0][0].bufs[0].buf.planes[0].m.dma_buf.fd,
	ipidma->fparams[0][0].bufs[0].buf.planes[0].m.dma_buf.offset,
	ipidma->fparams[0][0].bufs[0].buf.planes[0].reserved[0]);
}


static void mtk_imgsys_std2desc_fill_extbuf(struct mtk_imgsys_pipe *pipe,
					struct header_desc *ipidma,
					struct mtk_imgsys_dev_buffer *dev_buf)
{
	struct device *dev = pipe->imgsys_dev->dev;
	int i = 0;

	dev_dbg(dev,
		"%s : scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
		__func__,
		dev_buf->scp_daddr[0],
		dev_buf->isp_daddr[0],
		dev_buf->va_daddr[0]);

	for (i = 0; i < dev_buf->fmt.fmt.pix_mp.num_planes; i++) {
		ipidma->fparams[0][0].bufs[0].buf.planes[i].m.dma_buf.fd =
			dev_buf->vbb.vb2_buf.planes[i].m.fd;
		ipidma->fparams[0][0].bufs[0].buf.planes[i].m.dma_buf.offset =
			dev_buf->vbb.vb2_buf.planes[i].m.offset;
		ipidma->fparams[0][0].bufs[0].buf.planes[i].reserved[0] =
			(uint32_t)dev_buf->isp_daddr[i];
		dev_dbg(dev,
			"[%s]multi-plane[%d]:fd(%d),offset(%d),iova(0x%llx)\n",
			__func__, i,
		ipidma->fparams[0][0].bufs[0].buf.planes[i].m.dma_buf.fd,
		ipidma->fparams[0][0].bufs[0].buf.planes[i].m.dma_buf.offset,
		ipidma->fparams[0][0].bufs[0].buf.planes[i].reserved[0]);
	}

}

void mtk_imgsys_std_ipi_params_config(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct device *dev = pipe->imgsys_dev->dev;
	struct img_ipi_frameparam *dip_param = req->working_buf->frameparam.vaddr;
	struct mtk_imgsys_dev_buffer *buf_dma;
	struct mtk_imgsys_dev_buffer *buf_tuning;
	struct mtk_imgsys_dev_buffer *buf_ctrlmeta;
	int i = 0;

	dev_dbg(dev, "%s:%s: pipe-job id(%d)\n", __func__,
		pipe->desc->name, req->id);

	memset(dip_param, 0, sizeof(*dip_param));

	/* Tuning buffer */
	buf_tuning =
		req->buf_map[MTK_IMGSYS_VIDEO_NODE_TUNING_OUT];
	if (buf_tuning) {
		dev_dbg(dev,
			"Tuning buf queued: scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
			buf_tuning->scp_daddr[0],
			buf_tuning->isp_daddr[0],
			buf_tuning->va_daddr[0]);

		mtk_imgsys_meta_std2desc_fill_extbuf(pipe,
			&dip_param->tuning_meta,
			buf_tuning);

	} else {
		dev_dbg(dev,
			"No enqueued tuning buffer\n");
	}

	buf_ctrlmeta = req->buf_map[MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT];
	if (buf_ctrlmeta) {
		/* TODO: kva */
		dev_dbg(dev,
			"Ctrlmeta buf queued: scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
			buf_ctrlmeta->scp_daddr[0],
			buf_ctrlmeta->isp_daddr[0],
			buf_ctrlmeta->va_daddr[0]);

		mtk_imgsys_meta_std2desc_fill_extbuf(pipe,
			&dip_param->ctrl_meta,
			buf_ctrlmeta);

	}
	for (i = 0; i < pipe->desc->total_queues; i++) {
		if ((i == MTK_IMGSYS_VIDEO_NODE_TUNING_OUT) ||
			(i == MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT))
			continue;

		buf_dma = req->buf_map[i];
		if (buf_dma) {
			mtk_imgsys_std2desc_fill_extbuf(pipe,
				&dip_param->dmas[i],
				buf_dma);
			mtk_imgsys_std2desc_fill_bufinfo(pipe,
				&dip_param->dmas[i],
				buf_dma);
		}
	}
}

static void mtk_imgsys_singledevice_fill_ipi_param(struct mtk_imgsys_pipe *pipe,
					void *sd_dma,
					struct mtk_imgsys_dev_buffer *dev_buf,
					bool isMENode)
{
	struct mtk_imgsys_request *req = mtk_imgsys_media_req_to_imgsys_req(
		dev_buf->vbb.vb2_buf.request);
	//struct device *dev = pipe->imgsys_dev->dev;
	unsigned int i = 0, j = 0, tnum, tmax;
	struct header_desc *input_smvr = NULL;
	struct header_desc_norm *input_norm = NULL;

	switch (dev_buf->dev_fmt->format) {
	case V4L2_META_FMT_MTISP_SD:
		input_smvr = (struct header_desc *)sd_dma;
		tmax = TIME_MAX;
		tnum = input_smvr->fparams_tnum;
		break;
	default:
	case V4L2_META_FMT_MTISP_SDNORM:
		input_norm = (struct header_desc_norm *)sd_dma;
		tmax = TMAX;
		tnum = input_norm->fparams_tnum;
		break;
	}
	/*fd2iova : need 2D array parsing*/
	if (tnum >= tmax) {
		tnum = tmax;
		pr_debug("%s: %d bufinfo enqueued exceeds %d\n", __func__,
			tnum, tmax);
	}

	if (input_norm) {
		for (i = 0; i < tnum ; i++) {
			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&input_norm->fparams[i][j], isMENode, req);
		}
	} else {
		for (i = 0; i < tnum ; i++) {
			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&input_smvr->fparams[i][j], isMENode, req);
		}
	}
}

static void mtk_imgsys_sd_fill_dmas(struct mtk_imgsys_pipe *pipe, int dma,
					void *sd_dma,
					struct mtk_imgsys_dev_buffer *dev_buf,
					bool isMENode)
{
	struct mtk_imgsys_request *req = mtk_imgsys_media_req_to_imgsys_req(
		dev_buf->vbb.vb2_buf.request);
	//struct device *dev = pipe->imgsys_dev->dev;
	unsigned int i = 0, j = 0, tnum, tmax;
	struct header_desc *input_smvr = NULL;
	struct header_desc_norm *input_norm = NULL;
	struct singlenode_desc *sd_desc = NULL;
	struct singlenode_desc_norm *sd_desc_norm = NULL;

	switch (dev_buf->dev_fmt->format) {
	case V4L2_META_FMT_MTISP_SD:
		sd_desc =
			(struct singlenode_desc *)dev_buf->va_daddr[0];
		input_smvr = (struct header_desc *)sd_dma;
		tmax = TIME_MAX;
		tnum = input_smvr->fparams_tnum;
		break;
	default:
	case V4L2_META_FMT_MTISP_SDNORM:
		sd_desc_norm =
			(struct singlenode_desc_norm *)dev_buf->va_daddr[0];
		input_norm = (struct header_desc_norm *)sd_dma;
		tmax = TMAX;
		tnum = input_norm->fparams_tnum;
		break;
	}
	/*fd2iova : need 2D array parsing*/
	if (tnum >= tmax) {
		tnum = tmax;
		pr_debug("%s: %d bufinfo enqueued exceeds %d\n", __func__,
			tnum, tmax);
	}

	if (input_norm) {
		for (i = 0; i < tnum ; i++) {
			if (!sd_desc_norm->dmas_enable[dma][i])
				continue;
			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&input_norm->fparams[i][j], isMENode, req);
		}
	} else {
		for (i = 0; i < tnum ; i++) {
			if (!sd_desc->dmas_enable[dma][i])
				continue;

			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&input_smvr->fparams[i][j], isMENode, req);
		}
	}
}


void mtk_imgsys_singledevice_ipi_params_config(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct device *dev = pipe->imgsys_dev->dev;
	struct mtk_imgsys_dev_buffer *buf_in;
	struct singlenode_desc *singledevice_desc_dma = NULL;
	struct singlenode_desc_norm *singledevice_desc_norm = NULL;
	void *tuning_meta, *ctrl_meta;
	int i = 0;
	bool isMENode = false;

	dev_dbg(dev, "%s:%s: pipe-job id(%d)\n", __func__,
		pipe->desc->name, req->id);

	i = is_singledev_mode(req);
	buf_in = req->buf_map[i];
	if (!buf_in)
		return;

	if (buf_in->vbb.vb2_buf.memory == VB2_MEMORY_DMABUF)
		mtk_imgsys_kva_cache(buf_in, req);
	/* TODO:  MMAP imgbuffer path vaddr = 0 */

	dev_dbg(dev,
		"%s : scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
		__func__,
		buf_in->scp_daddr[0],
		buf_in->isp_daddr[0],
		buf_in->va_daddr[0]);

	if (!mtk_imgsys_desc_size_valid(req, buf_in))
		return;

	/*getKVA*/
	if (!buf_in->va_daddr[0]) {
		req->descriptor_error = -ENOMEM;
		return;
	}

	//init map table list for each devbuf

	//tuning_meta
	switch (buf_in->dev_fmt->format) {
	/* SMVR */
	case V4L2_META_FMT_MTISP_SD:
		singledevice_desc_dma =
			(struct singlenode_desc *)buf_in->va_daddr[0];
		tuning_meta = (void *) &singledevice_desc_dma->tuning_meta;
		ctrl_meta = (void *) &singledevice_desc_dma->ctrl_meta;
		break;
	/* NORM */
	default:
	case V4L2_META_FMT_MTISP_SDNORM:
		singledevice_desc_norm =
			(struct singlenode_desc_norm *)buf_in->va_daddr[0];
		tuning_meta = (void *) &singledevice_desc_norm->tuning_meta;
		ctrl_meta = (void *) &singledevice_desc_norm->ctrl_meta;
		break;
	}
	mtk_imgsys_sd_fill_dmas(pipe,
			MTK_IMGSYS_VIDEO_NODE_TUNING_OUT,
			tuning_meta,
			buf_in, isMENode); //ME's tuning need fd2iova????

	mtk_imgsys_singledevice_fill_ipi_param(pipe,
			ctrl_meta,
			buf_in, 1);

	for (i = 0; i < IMG_MAX_HW_DMAS; i++) {

		if (singledevice_desc_dma)
			mtk_imgsys_sd_fill_dmas(pipe, i,
				(void *)&singledevice_desc_dma->dmas[i],
				buf_in, isMENode);
		else
			mtk_imgsys_sd_fill_dmas(pipe, i,
				(void *)&singledevice_desc_norm->dmas[i],
				buf_in, isMENode);
	}
}

void mtk_imgsys_pipe_try_enqueue(struct mtk_imgsys_pipe *pipe)
{
	struct mtk_imgsys_request *req;
	unsigned long flags;

	lockdep_assert_held(&pipe->lock);
	while (pipe->streaming) {
		spin_lock_irqsave(&pipe->pending_job_lock, flags);
		if (list_empty(&pipe->pipe_job_pending_list)) {
			spin_unlock_irqrestore(&pipe->pending_job_lock, flags);
			return;
		}
		req = list_first_entry(&pipe->pipe_job_pending_list,
			struct mtk_imgsys_request, list);
		/* Keep FIFO ordering, but never submit an incompletely queued job. */
		if (atomic_read(&req->buf_count) || READ_ONCE(req->buffers_cancelled)) {
			spin_unlock_irqrestore(&pipe->pending_job_lock, flags);
			return;
		}
		list_del_init(&req->list);
		pipe->num_pending_jobs--;
		spin_unlock_irqrestore(&pipe->pending_job_lock, flags);
		mtk_imgsys_hw_enqueue(pipe->imgsys_dev, req);
	}
}

/* Called after vb2 preparation, while still in the request submitter's task. */
int mtk_imgsys_request_prepare_descriptors(struct mtk_imgsys_request *req)
{
	req->descriptor_error = 0;
	if (is_singledev_mode(req))
		mtk_imgsys_singledevice_ipi_params_config(req);
	else if (is_desc_mode(req))
		mtk_imgsys_desc_ipi_params_config(req);
	return req->descriptor_error;
}
