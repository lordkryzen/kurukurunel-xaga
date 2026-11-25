// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include "vdec_fmt_dmabuf.h"
#include "vdec_fmt_utils.h"

int fmt_dmabuf_get_iova(struct dma_buf *dbuf, u64 *iova,
	struct device *dev, struct dma_buf_attachment **attach, struct sg_table **sgt,
	bool cache_sync)
{
	struct scatterlist *sg;
 u64 start, length = 0;
 unsigned int i;
 *attach = NULL; *sgt = NULL; *iova = 0;
 if (!dbuf || !dbuf->size) return -EINVAL;
 *attach = dma_buf_attach(dbuf, dev);
	if (IS_ERR(*attach)) {
		fmt_debug(0, "attach fail, return\n");
		*attach = NULL;
		return -1;
	}

	if (!cache_sync)
		(*attach)->dma_map_attrs |= DMA_ATTR_SKIP_CPU_SYNC;

	*sgt = dma_buf_map_attachment_unlocked(*attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(*sgt)) {
		fmt_debug(0, "map failed, detach and return\n");
		dma_buf_detach(dbuf, *attach);
		*sgt = NULL;
		*attach = NULL;
		return -1;
	}

 if (!(*sgt)->sgl || !(*sgt)->nents) goto invalid;
 start = sg_dma_address((*sgt)->sgl);
 for_each_sg((*sgt)->sgl, sg, (*sgt)->nents, i) {
  u64 n = sg_dma_len(sg);
  if (!n || length > U64_MAX - start || sg_dma_address(sg) != start + length ||
      n > U64_MAX - length || n - 1 > U64_MAX - sg_dma_address(sg)) goto invalid;
  length += n;
  if (length >= dbuf->size) break;
 }
 if (length < dbuf->size) goto invalid;
	*iova = sg_dma_address((*sgt)->sgl);
	fmt_debug(1, "dbuf %p attach %p sgt %p iova 0x%llx\n", dbuf, *attach, *sgt, *iova);
	return 0;
invalid:
 dma_buf_unmap_attachment_unlocked(*attach, *sgt, DMA_BIDIRECTIONAL);
 dma_buf_detach(dbuf, *attach);
 *sgt = NULL; *attach = NULL; return -EINVAL;

}

void fmt_dmabuf_free_iova(struct dma_buf *dbuf,
	struct dma_buf_attachment *attach, struct sg_table *sgt)
{
	if (attach == NULL || sgt == NULL) {
		fmt_debug(0, "attach or sgt null, not need to free iova");
		return;
	}
	fmt_debug(1, "dbuf %p attach %p sgt %p\n", dbuf, attach, sgt);
	dma_buf_unmap_attachment_unlocked(attach, sgt, DMA_BIDIRECTIONAL);
	dma_buf_detach(dbuf, attach);
}

struct dma_buf *fmt_dmabuf_get(int fd)
{
	struct dma_buf *dbuf;

	dbuf = dma_buf_get(fd);
	if (IS_ERR(dbuf)) {
		fmt_debug(0, "dma_buf_get fail");
		return NULL;
	}
	fmt_debug(1, "dbuf %p fd %d\n", dbuf, fd);

	return dbuf;
}

void fmt_dmabuf_put(struct dma_buf *dbuf)
{
	if (!dbuf) {
		fmt_debug(0, "dma_buf null, no need to put.");
		return;
	}
	fmt_debug(1, "dbuf %p\n", dbuf);
	dma_buf_put(dbuf);
}

/* Each input/output slot owns exactly one mapped buffer until wait/close.
 * Repeated plane/address commands reuse that owner, including FD aliases.
 */
u64 fmt_translate_fd(u64 fd, u32 offset, struct dmabufmap map[], struct device *dev,
 struct dma_buf **dbuf, struct dma_buf_attachment **attach, struct sg_table **sgt,
 bool cache_sync)
{
 struct dma_buf *buf;
 u64 addr;
 int ret;
 (void)map;
 if (fd > INT_MAX) return 0;
 buf = dma_buf_get((int)fd);
 if (IS_ERR(buf)) return 0;
 if (offset >= buf->size) {dma_buf_put(buf); return 0;}
 if (*dbuf) {
  if (*dbuf != buf) {dma_buf_put(buf); return 0;}
  dma_buf_put(buf);
  addr = sg_dma_address((*sgt)->sgl);
 } else {
  ret = fmt_dmabuf_get_iova(buf, &addr, dev, attach, sgt, cache_sync);
  if (ret) {dma_buf_put(buf); return 0;}
  *dbuf = buf;
 }
 if (addr > U64_MAX - offset) return 0;
 return addr + offset;
}

MODULE_IMPORT_NS(DMA_BUF);
