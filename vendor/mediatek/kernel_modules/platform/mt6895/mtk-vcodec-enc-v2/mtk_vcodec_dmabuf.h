/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _MTK_VCODEC_DMABUF_H_
#define _MTK_VCODEC_DMABUF_H_

#include <linux/dma-buf.h>
#include <linux/scatterlist.h>
#include <linux/dma-direction.h>
#include <linux/err.h>

/* Caller owns the dma_buf reference. Publish attachment/table only after the
 * complete mapping succeeds; every failed attempt leaves empty outputs.
 */
static inline int mtk_codec_map_dmabuf(struct device *dev, struct dma_buf *buf,
		enum dma_data_direction direction,
		struct dma_buf_attachment **attachment, struct sg_table **table,
		dma_addr_t *address)
{
	struct dma_buf_attachment *a;
	struct sg_table *t;
	struct scatterlist *sg;
	u64 start, contiguous = 0;
	unsigned int i;
	int ret;

	*attachment = NULL;
	*table = NULL;
	*address = 0;
	if (!dev || !buf || !buf->size || buf->size > ~(__u32)0)
		return -EINVAL;
	a = dma_buf_attach(buf, dev);
	if (IS_ERR_OR_NULL(a))
		return a ? PTR_ERR(a) : -ENOMEM;
	t = dma_buf_map_attachment(a, direction);
	if (IS_ERR_OR_NULL(t)) {
		ret = t ? PTR_ERR(t) : -ENOMEM;
		goto detach;
	}
	if (!t->sgl || !t->nents) {
		ret = -EINVAL;
		goto unmap;
	}
	start = sg_dma_address(t->sgl);
	for_each_sg(t->sgl, sg, t->nents, i) {
		u64 size = sg_dma_len(sg);

		if (!size || contiguous > ~(u64)0 - start ||
		    sg_dma_address(sg) != start + contiguous ||
		    size - 1 > ~(u64)0 - sg_dma_address(sg) ||
		    size > ~(u64)0 - contiguous)
			break;
		contiguous += size;
		if (contiguous >= buf->size)
			break;
	}
	if (contiguous < buf->size) {
		ret = -EINVAL;
		goto unmap;
	}
	*attachment = a;
	*table = t;
	*address = (dma_addr_t)start;
	return 0;

unmap:
	dma_buf_unmap_attachment(a, t, direction);
detach:
	dma_buf_detach(buf, a);
	return ret;
}

static inline void mtk_codec_unmap_dmabuf(struct dma_buf *buf,
		enum dma_data_direction direction,
		struct dma_buf_attachment **attachment, struct sg_table **table,
		dma_addr_t *address)
{
	if (*table && *attachment)
		dma_buf_unmap_attachment(*attachment, *table, direction);
	if (*attachment)
		dma_buf_detach(buf, *attachment);
	*attachment = NULL;
	*table = NULL;
	*address = 0;
}

#endif
