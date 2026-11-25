/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MTK_VCODEC_ENC_METADATA_H_
#define _MTK_VCODEC_ENC_METADATA_H_

#include "mtk_vcodec_dmabuf.h"

/* The header ABI is shared by both generations. Keep the mapped container and
 * separate QP buffer pinned until DQBUF/cleanup, rather than retaining stale
 * IOVAs after dma_buf_unmap_attachment(). No firmware structure is changed.
 */
static inline void mtk_codec_enc_release_metadata(struct venc_frm_buf *frame)
{
	if (frame->metabuffer_dma) {
		mtk_codec_unmap_dmabuf(frame->metabuffer_dma, DMA_TO_DEVICE,
			&frame->buf_att, &frame->sgt, &frame->metabuffer_addr);
		dma_buf_put(frame->metabuffer_dma);
	} else if (frame->meta_dma) {
		mtk_codec_unmap_dmabuf(frame->meta_dma, DMA_TO_DEVICE,
			&frame->buf_att, &frame->sgt, &frame->meta_addr);
		dma_buf_put(frame->meta_dma);
	}
	if (frame->qpmap_dma) {
		mtk_codec_unmap_dmabuf(frame->qpmap_dma, DMA_TO_DEVICE,
			&frame->qpmap_dma_att, &frame->qpmap_sgt,
			&frame->qpmap_dma_addr);
		dma_buf_put(frame->qpmap_dma);
	}
	frame->meta_dma = NULL;
	frame->metabuffer_dma = NULL;
	frame->qpmap_dma = NULL;
	frame->meta_addr = 0;
	frame->metabuffer_addr = 0;
	frame->qpmap_dma_addr = 0;
	frame->meta_offset = 0;
	frame->has_meta = 0;
	frame->has_qpmap = 0;
}

static inline int mtk_codec_enc_prepare_metadata(struct venc_frm_buf *frame,
		u32 flags, u32 fd, struct device *general_dev,
		struct device *hdr_dev)
{
	const u32 mask = V4L2_BUF_FLAG_HDR_META | V4L2_BUF_FLAG_QP_META |
		V4L2_BUF_FLAG_HAS_META;
	struct metadata_info snapshot;
	struct dma_buf *buffer;
	struct iosys_map map;
	u32 kind = flags & mask;
	unsigned int i;
	int ret;

	if (frame->meta_dma || frame->metabuffer_dma || frame->qpmap_dma)
		return -EBUSY;
	if (!kind)
		return 0;
	if ((kind & (kind - 1)) || fd > INT_MAX)
		return -EINVAL;

	buffer = dma_buf_get((int)fd);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);
	if (kind == V4L2_BUF_FLAG_QP_META) {
		frame->qpmap_dma = buffer;
		ret = mtk_codec_map_dmabuf(general_dev, buffer, DMA_TO_DEVICE,
			&frame->qpmap_dma_att, &frame->qpmap_sgt,
			&frame->qpmap_dma_addr);
		if (ret)
			goto fail;
		frame->has_qpmap = 1;
		return 0;
	}
	if (kind == V4L2_BUF_FLAG_HDR_META) {
		frame->meta_dma = buffer;
		ret = mtk_codec_map_dmabuf(hdr_dev, buffer, DMA_TO_DEVICE,
			&frame->buf_att, &frame->sgt, &frame->meta_addr);
		if (ret)
			goto fail;
		frame->has_meta = 1;
		return 0;
	}

	frame->metabuffer_dma = buffer;
	if (buffer->size < sizeof(snapshot) || buffer->size > ~(u32)0) {
		ret = -EINVAL;
		goto fail;
	}
	ret = dma_buf_begin_cpu_access(buffer, DMA_FROM_DEVICE);
	if (ret)
		goto fail;
	ret = dma_buf_vmap(buffer, &map);
	if (ret) {
		dma_buf_end_cpu_access(buffer, DMA_FROM_DEVICE);
		goto fail;
	}
	/* Parse a stable header snapshot, not a repeatedly changing DMA mapping. */
	iosys_map_memcpy_from(&snapshot, &map, 0, sizeof(snapshot));
	dma_buf_vunmap(buffer, &map);
	ret = dma_buf_end_cpu_access(buffer, DMA_FROM_DEVICE);
	if (ret)
		goto fail;
	ret = mtk_codec_map_dmabuf(general_dev, buffer, DMA_TO_DEVICE,
		&frame->buf_att, &frame->sgt, &frame->metabuffer_addr);
	if (ret)
		goto fail;

	for (i = 0; i < MTK_MAX_METADATA_NUM; i++) {
		const struct meta_describe *item = &snapshot.metadata_dsc[i];
		u64 bytes = (u64)item->size * sizeof(u32);

		if (!item->invalid)
			break;
		if (item->invalid != 1 || item->fd_flag > 1) {
			ret = -EINVAL;
			goto fail;
		}
		if (item->type == METADATA_HDR && !item->fd_flag) {
			if (frame->has_meta || item->value > buffer->size ||
			    bytes > buffer->size - item->value || !bytes) {
				ret = -EINVAL;
				goto fail;
			}
			frame->meta_dma = buffer; /* Alias, not a second owned ref. */
			frame->meta_addr = frame->metabuffer_addr + item->value;
			frame->meta_offset = item->value;
			frame->has_meta = 1;
		} else if (item->type == METADATA_QPMAP && item->fd_flag) {
			if (frame->has_qpmap || item->value > INT_MAX) {
				ret = -EINVAL;
				goto fail;
			}
			frame->qpmap_dma = dma_buf_get((int)item->value);
			if (IS_ERR(frame->qpmap_dma)) {
				ret = PTR_ERR(frame->qpmap_dma);
				frame->qpmap_dma = NULL;
				goto fail;
			}
			if (bytes > frame->qpmap_dma->size) {
				ret = -EINVAL;
				goto fail;
			}
			ret = mtk_codec_map_dmabuf(general_dev, frame->qpmap_dma,
				DMA_TO_DEVICE, &frame->qpmap_dma_att,
				&frame->qpmap_sgt, &frame->qpmap_dma_addr);
			if (ret)
				goto fail;
			frame->has_qpmap = 1;
		} else {
			/* New stats/dynamic/ROI descriptors need actual firmware paths. */
			ret = -EOPNOTSUPP;
			goto fail;
		}
	}
	return 0;
fail:
	mtk_codec_enc_release_metadata(frame);
	return ret;
}

#endif
