/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MTK_IMGSYS_DMA_H__
#define __MTK_IMGSYS_DMA_H__
#include <linux/dma-buf.h>
#include <linux/err.h>
#include <linux/iosys-map.h>

static inline void *mtk_imgsys_vmap(struct dma_buf *buffer)
{
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(NULL);
	int ret = dma_buf_vmap_unlocked(buffer, &map);

	if (ret)
		return ERR_PTR(ret);
	if (map.is_iomem) {
		dma_buf_vunmap_unlocked(buffer, &map);
		return ERR_PTR(-EOPNOTSUPP);
	}
	return map.vaddr;
}

static inline void mtk_imgsys_vunmap(struct dma_buf *buffer, void *address)
{
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(address);

	/* vmap accepts only CPU mappings; reconstruct the identical map value. */
	dma_buf_vunmap_unlocked(buffer, &map);
}
#endif
