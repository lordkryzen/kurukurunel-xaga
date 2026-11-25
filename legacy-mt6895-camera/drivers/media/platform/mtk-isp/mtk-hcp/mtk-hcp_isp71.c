#include "../mtk-imgsys-dma.h"
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */
#include <linux/slab.h>
#include <linux/file.h>
#include <linux/mutex.h>
#include <linux/kref.h>
#include "mtk_heap.h"
#include "mtk-hcp_isp71.h"

static struct mtk_hcp_reserve_mblock *mb;
static DEFINE_MUTEX(isp71_working_lock);
static bool isp71_working_active;
static unsigned int isp71_block_count;
static unsigned int isp71_gce_users;
static bool isp71_cpu_access[NUMS_MEM_ID];
static bool isp71_reserved_fd[NUMS_MEM_ID];

static struct mtk_hcp_reserve_mblock isp71_smvr_mblock[] = {
	{
		/*share buffer for frame setting, to be sw usage*/
		.name = "IMG_MEM_FOR_HW_ID",
		.num = IMG_MEM_FOR_HW_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x400000,   /*need more than 4MB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "WPE_MEM_C_ID",
		.num = WPE_MEM_C_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x300000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "WPE_MEM_T_ID",
		.num = WPE_MEM_T_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x500000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "TRAW_MEM_C_ID",
		.num = TRAW_MEM_C_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0xB00000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "TRAW_MEM_T_ID",
		.num = TRAW_MEM_T_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x3000000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "DIP_MEM_C_ID",
		.num = DIP_MEM_C_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0xF00000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "DIP_MEM_T_ID",
		.num = DIP_MEM_T_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x3000000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "PQDIP_MEM_C_ID",
		.num = PQDIP_MEM_C_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x200000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "PQDIP_MEM_T_ID",
		.num = PQDIP_MEM_T_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x200000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "ADL_MEM_C_ID",
		.num = ADL_MEM_C_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x100000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL
	},
	{
		.name = "ADL_MEM_T_ID",
		.num = ADL_MEM_T_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x200000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL
	},
	{
		.name = "IMG_MEM_G_ID",
		.num = IMG_MEM_G_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x2200000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
};


struct mtk_hcp_reserve_mblock isp71_reserve_mblock[] = {
	{
		/*share buffer for frame setting, to be sw usage*/
		.name = "IMG_MEM_FOR_HW_ID",
		.num = IMG_MEM_FOR_HW_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x400000,   /*need more than 4MB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "WPE_MEM_C_ID",
		.num = WPE_MEM_C_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0xE1000,   /*900KB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "WPE_MEM_T_ID",
		.num = WPE_MEM_T_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x196000,   /*1MB + 600KB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "TRAW_MEM_C_ID",
		.num = TRAW_MEM_C_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x4C8000,   /*4MB + 800KB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "TRAW_MEM_T_ID",
		.num = TRAW_MEM_T_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x1AC8000,   /*26MB + 800KB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "DIP_MEM_C_ID",
		.num = DIP_MEM_C_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x5C8000,   /*5MB + 800KB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "DIP_MEM_T_ID",
		.num = DIP_MEM_T_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x1FAF000,   /*31MB + 700KB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "PQDIP_MEM_C_ID",
		.num = PQDIP_MEM_C_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x100000,   /*1MB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "PQDIP_MEM_T_ID",
		.num = PQDIP_MEM_T_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x170000,   /*1MB + 500KB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
	{
		.name = "ADL_MEM_C_ID",
		.num = ADL_MEM_C_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x100000,   /*1MB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL
	},
	{
		.name = "ADL_MEM_T_ID",
		.num = ADL_MEM_T_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x200000,   /*2MB*/
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL
	},
	{
		.name = "IMG_MEM_G_ID",
		.num = IMG_MEM_G_ID,
		.start_phys = 0x0,
		.start_virt = 0x0,
		.start_dma  = 0x0,
		.size = 0x1800000,
		.is_dma_buf = true,
		.mmap_cnt = 0,
		.mem_priv = NULL,
		.d_buf = NULL,
		.fd = -1,
		.pIonHandle = NULL,
		.attach = NULL,
		.sgt = NULL
	},
};

phys_addr_t isp71_get_reserve_mem_phys(unsigned int id)
{
	if (!mb || id >= NUMS_MEM_ID) {
		pr_info("[HCP] no reserve memory for %d", id);
		return 0;
	} else {
		return mb[id].start_phys;
	}
}
EXPORT_SYMBOL(isp71_get_reserve_mem_phys);

void *isp71_get_reserve_mem_virt(unsigned int id)
{
	if (!mb || id >= NUMS_MEM_ID) {
		pr_info("[HCP] no reserve memory for %d", id);
		return 0;
	} else
		return mb[id].start_virt;
}
EXPORT_SYMBOL(isp71_get_reserve_mem_virt);

phys_addr_t isp71_get_reserve_mem_dma(unsigned int id)
{
	if (!mb || id >= NUMS_MEM_ID) {
		pr_info("[HCP] no reserve memory for %d", id);
		return 0;
	} else {
		return mb[id].start_dma;
	}
}
EXPORT_SYMBOL(isp71_get_reserve_mem_dma);

phys_addr_t isp71_get_reserve_mem_size(unsigned int id)
{
	if (!mb || id >= NUMS_MEM_ID) {
		pr_info("[HCP] no reserve memory for %d", id);
		return 0;
	} else {
		return mb[id].size;
	}
}
EXPORT_SYMBOL(isp71_get_reserve_mem_size);

uint32_t isp71_get_reserve_mem_fd(unsigned int id)
{
	if (!mb || id >= NUMS_MEM_ID) {
		pr_info("[HCP] no reserve memory for %d", id);
		return 0;
	} else
		return mb[id].fd;
}
EXPORT_SYMBOL(isp71_get_reserve_mem_fd);

void *isp71_get_gce_virt(void)
{
	return mb ? mb[IMG_MEM_G_ID].start_virt : NULL;
}
EXPORT_SYMBOL(isp71_get_gce_virt);

void *isp71_get_hwid_virt(void)
{
	return mb ? mb[DIP_MEM_FOR_HW_ID].start_virt : NULL;
}
EXPORT_SYMBOL(isp71_get_hwid_virt);


/* All allocation, teardown and GCE-reference changes hold working_lock. */
static void isp71_free_block(struct mtk_hcp_reserve_mblock *block,
	unsigned int id)
{
	enum dma_data_direction direction = id == IMG_MEM_G_ID ?
		DMA_BIDIRECTIONAL : DMA_TO_DEVICE;

	if (id == IMG_MEM_FOR_HW_ID)
		return; /* DT/probe owns this reservation. */
	if (isp71_reserved_fd[id]) {
		put_unused_fd(block->fd);
		isp71_reserved_fd[id] = false;
	}
	if (block->is_dma_buf && !IS_ERR_OR_NULL(block->d_buf)) {
		if (!IS_ERR_OR_NULL(block->start_virt))
			mtk_imgsys_vunmap(block->d_buf, block->start_virt);
		if (isp71_cpu_access[id]) {
			dma_buf_end_cpu_access(block->d_buf, direction);
			isp71_cpu_access[id] = false;
		}
		if (!IS_ERR_OR_NULL(block->attach)) {
			if (!IS_ERR_OR_NULL(block->sgt))
				dma_buf_unmap_attachment_unlocked(block->attach,
					block->sgt, direction);
			dma_buf_detach(block->d_buf, block->attach);
		}
		dma_buf_put(block->d_buf);
	} else if (!block->is_dma_buf) {
		kfree(block->start_virt);
	}
	block->mem_priv = NULL;
	block->mmap_cnt = 0;
	block->start_dma = 0;
	block->start_virt = NULL;
	block->start_phys = 0;
	block->d_buf = NULL;
	block->fd = -1;
	block->pIonHandle = NULL;
	block->attach = NULL;
	block->sgt = NULL;
}

static bool isp71_contiguous_iova(struct sg_table *table, size_t size)
{
	struct scatterlist *sg;
	dma_addr_t expected;
	size_t span = 0;
	unsigned int i;

	if (!table->nents)
		return false;
	expected = sg_dma_address(table->sgl);
	for_each_sgtable_dma_sg(table, sg, i) {
		if (sg_dma_address(sg) != expected)
			break;
		expected += sg_dma_len(sg);
		span += sg_dma_len(sg);
		if (span >= size)
			return true;
	}
	return false;
}

int isp71_allocate_working_buffer(struct mtk_hcp *hcp_dev, unsigned int mode)
{
	struct mtk_hcp_reserve_mblock *blocks = mode ?
		hcp_dev->data->smblock : hcp_dev->data->mblock;
	unsigned int count = hcp_dev->data->block_num;
	unsigned int id;
	int ret = 0;

	if (!blocks || count <= IMG_MEM_G_ID || count > NUMS_MEM_ID)
		return -EINVAL;
	mutex_lock(&isp71_working_lock);
	if (isp71_working_active || isp71_gce_users) {
		ret = -EBUSY;
		goto unlock;
	}
	for (id = 0; id < count; id++) {
		struct mtk_hcp_reserve_mblock *block = &blocks[id];
		enum dma_data_direction direction = id == IMG_MEM_G_ID ?
			DMA_BIDIRECTIONAL : DMA_TO_DEVICE;
		struct dma_heap *heap;

		if (id == IMG_MEM_FOR_HW_ID)
			continue;
		if (!block->is_dma_buf) {
			block->start_virt = kzalloc(block->size, GFP_KERNEL);
			if (!block->start_virt) {
				ret = -ENOMEM;
				goto rollback;
			}
			continue;
		}
		heap = dma_heap_find(id == IMG_MEM_G_ID ? "mtk_mm" : "mtk_mm-uncached");
		if (!heap) {
			ret = -ENODEV;
			goto rollback;
		}
		block->d_buf = dma_heap_buffer_alloc(heap, block->size,
			O_RDWR | O_CLOEXEC, DMA_HEAP_VALID_HEAP_FLAGS);
		dma_heap_put(heap);
		if (IS_ERR(block->d_buf)) {
			ret = PTR_ERR(block->d_buf);
			block->d_buf = NULL;
			goto rollback;
		}
		mtk_dma_buf_set_name(block->d_buf, block->name);
		block->attach = dma_buf_attach(block->d_buf, hcp_dev->dev);
		if (IS_ERR(block->attach)) {
			ret = PTR_ERR(block->attach);
			goto rollback;
		}
		block->sgt = dma_buf_map_attachment_unlocked(block->attach, direction);
		if (IS_ERR(block->sgt)) {
			ret = PTR_ERR(block->sgt);
			goto rollback;
		}
		if (!isp71_contiguous_iova(block->sgt, block->size)) {
			ret = -EINVAL;
			goto rollback;
		}
		block->start_phys = sg_dma_address(block->sgt->sgl);
		block->start_dma = block->start_phys;
		if (id == IMG_MEM_G_ID) {
			ret = dma_buf_begin_cpu_access(block->d_buf, direction);
			if (ret)
				goto rollback;
			isp71_cpu_access[id] = true;
		}
		block->start_virt = mtk_imgsys_vmap(block->d_buf);
		if (IS_ERR_OR_NULL(block->start_virt)) {
			ret = block->start_virt ? PTR_ERR(block->start_virt) : -ENOMEM;
			goto rollback;
		}
		block->fd = get_unused_fd_flags(O_CLOEXEC);
		if (block->fd < 0) {
			ret = block->fd;
			goto rollback;
		}
		isp71_reserved_fd[id] = true;
	}
	/* Reserve every FD before publication: rollback never has to close a
	 * userspace-owned descriptor, and the driver keeps its own DMA-BUF ref.
	 */
	for (id = 0; id < count; id++) {
		if (!isp71_reserved_fd[id])
			continue;
		get_dma_buf(blocks[id].d_buf);
		fd_install(blocks[id].fd, blocks[id].d_buf->file);
		isp71_reserved_fd[id] = false;
	}
	mb = blocks;
	isp71_block_count = count;
	isp71_working_active = true;
	kref_init(&blocks[IMG_MEM_G_ID].kref);
	goto unlock;
rollback:
	/* The failing block may already own a prefix of its resources. */
	do {
		isp71_free_block(&blocks[id], id);
	} while (id--);
unlock:
	mutex_unlock(&isp71_working_lock);
	return ret;
}
EXPORT_SYMBOL(isp71_allocate_working_buffer);

static void gce_release(struct kref *ref)
{
	struct mtk_hcp_reserve_mblock *block =
		container_of(ref, struct mtk_hcp_reserve_mblock, kref);

	isp71_free_block(block, IMG_MEM_G_ID);
}


int isp71_release_working_buffer(struct mtk_hcp *hcp_dev)
{
	unsigned int id;

	mutex_lock(&isp71_working_lock);
	if (!isp71_working_active)
		goto unlock;
	isp71_working_active = false;
	for (id = isp71_block_count; id-- > 0;) {
		if (id == IMG_MEM_G_ID)
			kref_put(&mb[id].kref, gce_release);
		else
			isp71_free_block(&mb[id], id);
	}
unlock:
	mutex_unlock(&isp71_working_lock);
	return 0;
}
EXPORT_SYMBOL(isp71_release_working_buffer);

int isp71_get_init_info(struct img_init_info *info)
{

	if (!info) {
		pr_info("%s:NULL info\n", __func__);
		return -1;
	}

	info->hw_buf = isp71_get_reserve_mem_phys(DIP_MEM_FOR_HW_ID);
	/*WPE:0, ADL:1, TRAW:2, DIP:3, PQDIP:4 */
	info->module_info[0].c_wbuf =
				isp71_get_reserve_mem_phys(WPE_MEM_C_ID);
	info->module_info[0].c_wbuf_dma =
				isp71_get_reserve_mem_dma(WPE_MEM_C_ID);
	info->module_info[0].c_wbuf_sz =
				isp71_get_reserve_mem_size(WPE_MEM_C_ID);
	info->module_info[0].c_wbuf_fd =
				isp71_get_reserve_mem_fd(WPE_MEM_C_ID);
	info->module_info[0].t_wbuf =
				isp71_get_reserve_mem_phys(WPE_MEM_T_ID);
	info->module_info[0].t_wbuf_dma =
				isp71_get_reserve_mem_dma(WPE_MEM_T_ID);
	info->module_info[0].t_wbuf_sz =
				isp71_get_reserve_mem_size(WPE_MEM_T_ID);
	info->module_info[0].t_wbuf_fd =
				isp71_get_reserve_mem_fd(WPE_MEM_T_ID);

  // ADL
	info->module_info[1].c_wbuf =
				isp71_get_reserve_mem_phys(ADL_MEM_C_ID);
	info->module_info[1].c_wbuf_dma =
				isp71_get_reserve_mem_dma(ADL_MEM_C_ID);
	info->module_info[1].c_wbuf_sz =
				isp71_get_reserve_mem_size(ADL_MEM_C_ID);
	info->module_info[1].c_wbuf_fd =
				isp71_get_reserve_mem_fd(ADL_MEM_C_ID);
	info->module_info[1].t_wbuf =
				isp71_get_reserve_mem_phys(ADL_MEM_T_ID);
	info->module_info[1].t_wbuf_dma =
				isp71_get_reserve_mem_dma(ADL_MEM_T_ID);
	info->module_info[1].t_wbuf_sz =
				isp71_get_reserve_mem_size(ADL_MEM_T_ID);
	info->module_info[1].t_wbuf_fd =
				isp71_get_reserve_mem_fd(ADL_MEM_T_ID);

	// TRAW
	info->module_info[2].c_wbuf =
				isp71_get_reserve_mem_phys(TRAW_MEM_C_ID);
	info->module_info[2].c_wbuf_dma =
				isp71_get_reserve_mem_dma(TRAW_MEM_C_ID);
	info->module_info[2].c_wbuf_sz =
				isp71_get_reserve_mem_size(TRAW_MEM_C_ID);
	info->module_info[2].c_wbuf_fd =
				isp71_get_reserve_mem_fd(TRAW_MEM_C_ID);
	info->module_info[2].t_wbuf =
				isp71_get_reserve_mem_phys(TRAW_MEM_T_ID);
	info->module_info[2].t_wbuf_dma =
				isp71_get_reserve_mem_dma(TRAW_MEM_T_ID);
	info->module_info[2].t_wbuf_sz =
				isp71_get_reserve_mem_size(TRAW_MEM_T_ID);
	info->module_info[2].t_wbuf_fd =
				isp71_get_reserve_mem_fd(TRAW_MEM_T_ID);

		// DIP
	info->module_info[3].c_wbuf =
				isp71_get_reserve_mem_phys(DIP_MEM_C_ID);
	info->module_info[3].c_wbuf_dma =
				isp71_get_reserve_mem_dma(DIP_MEM_C_ID);
	info->module_info[3].c_wbuf_sz =
				isp71_get_reserve_mem_size(DIP_MEM_C_ID);
	info->module_info[3].c_wbuf_fd =
				isp71_get_reserve_mem_fd(DIP_MEM_C_ID);
	info->module_info[3].t_wbuf =
				isp71_get_reserve_mem_phys(DIP_MEM_T_ID);
	info->module_info[3].t_wbuf_dma =
				isp71_get_reserve_mem_dma(DIP_MEM_T_ID);
	info->module_info[3].t_wbuf_sz =
				isp71_get_reserve_mem_size(DIP_MEM_T_ID);
	info->module_info[3].t_wbuf_fd =
				isp71_get_reserve_mem_fd(DIP_MEM_T_ID);

	// PQDIP
	info->module_info[4].c_wbuf =
				isp71_get_reserve_mem_phys(PQDIP_MEM_C_ID);
	info->module_info[4].c_wbuf_dma =
				isp71_get_reserve_mem_dma(PQDIP_MEM_C_ID);
	info->module_info[4].c_wbuf_sz =
				isp71_get_reserve_mem_size(PQDIP_MEM_C_ID);
	info->module_info[4].c_wbuf_fd =
			isp71_get_reserve_mem_fd(PQDIP_MEM_C_ID);
	info->module_info[4].t_wbuf =
				isp71_get_reserve_mem_phys(PQDIP_MEM_T_ID);
	info->module_info[4].t_wbuf_dma =
				isp71_get_reserve_mem_dma(PQDIP_MEM_T_ID);
	info->module_info[4].t_wbuf_sz =
				isp71_get_reserve_mem_size(PQDIP_MEM_T_ID);
	info->module_info[4].t_wbuf_fd =
				isp71_get_reserve_mem_fd(PQDIP_MEM_T_ID);

	/*common*/
	/* info->g_wbuf_fd = isp71_get_reserve_mem_fd(IMG_MEM_G_ID); */
	info->g_wbuf_fd = isp71_get_reserve_mem_fd(IMG_MEM_G_ID);
	info->g_wbuf = isp71_get_reserve_mem_phys(IMG_MEM_G_ID);
	/*info->g_wbuf_sw = isp71_get_reserve_mem_virt(IMG_MEM_G_ID);*/
	info->g_wbuf_sz = isp71_get_reserve_mem_size(IMG_MEM_G_ID);

	return 0;
}

static int isp71_put_gce(void)
{
	int ret = 0;

	mutex_lock(&isp71_working_lock);
	if (!mb || !isp71_gce_users) {
		ret = -EINVAL;
	} else {
		isp71_gce_users--;
		kref_put(&mb[IMG_MEM_G_ID].kref, gce_release);
	}
	mutex_unlock(&isp71_working_lock);
	return ret;
}

static int isp71_get_gce(void)
{
	int ret = 0;

	mutex_lock(&isp71_working_lock);
	if (!mb || (!isp71_working_active && !isp71_gce_users)) {
		ret = -ENODEV;
	} else {
		kref_get(&mb[IMG_MEM_G_ID].kref);
		isp71_gce_users++;
	}
	mutex_unlock(&isp71_working_lock);
	return ret;
}

/* Snapshot address and actual extent under the same lock as its lease. */
static int isp71_acquire_gce_region(void **base, size_t *size)
{
	struct mtk_hcp_reserve_mblock *block;
	int ret = -ENODEV;

	if (!base || !size)
		return -EINVAL;
	*base = NULL;
	*size = 0;
	mutex_lock(&isp71_working_lock);
	if (!mb || (!isp71_working_active && !isp71_gce_users))
		goto unlock;
	block = &mb[IMG_MEM_G_ID];
	if (IS_ERR_OR_NULL(block->start_virt) || IS_ERR_OR_NULL(block->d_buf) ||
	    !block->size || block->size > block->d_buf->size)
		goto unlock;
	kref_get(&block->kref);
	isp71_gce_users++;
	*base = block->start_virt;
	*size = block->size;
	ret = 0;
unlock:
	mutex_unlock(&isp71_working_lock);
	return ret;
}

struct mtk_hcp_data isp71_hcp_data = {
	.mblock = isp71_reserve_mblock,
	.block_num = ARRAY_SIZE(isp71_reserve_mblock),
	.smblock = isp71_smvr_mblock,
	.allocate = isp71_allocate_working_buffer,
	.release = isp71_release_working_buffer,
	.get_init_info = isp71_get_init_info,
	.get_gce_virt = isp71_get_gce_virt,
	.get_gce = isp71_get_gce,
	.put_gce = isp71_put_gce,
	.get_hwid_virt = isp71_get_hwid_virt,
	.acquire_gce_region = isp71_acquire_gce_region,
};
