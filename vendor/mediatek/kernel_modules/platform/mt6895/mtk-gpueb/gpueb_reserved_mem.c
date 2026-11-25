// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

/**
 * @file    gpueb_reserved_mem.c
 * @brief   Reserved memory info init for GPUEB
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <linux/random.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>
#include <mboot_params.h>
#include <linux/of_reserved_mem.h>
#include <linux/vmalloc.h>

#include "gpueb_helper.h"
#include "gpueb_reserved_mem.h"
#include "gpueb_memory_contract.h"
#include "gpueb_lifecycle.h"

phys_addr_t gpueb_mem_base_phys;
phys_addr_t gpueb_mem_base_virt;
phys_addr_t gpueb_mem_size;
int gpueb_mem_num;

struct gpueb_reserve_mblock *gpueb_reserve_mblock_ary;
const char *gpueb_reserve_mblock_ary_name[20];

int gpueb_reserve_mem_of_init(struct reserved_mem *rmem)
{
	gpueb_log_d(GPUEB_TAG, "%pa %pa", &rmem->base, &rmem->size);
	gpueb_mem_base_phys = (phys_addr_t) rmem->base;
	gpueb_mem_size = (phys_addr_t) rmem->size;

	return 0;
}
RESERVEDMEM_OF_DECLARE(gpueb_reserve_mem_init,
		GPUEB_MEM_RESERVED_KEY, gpueb_reserve_mem_of_init);

phys_addr_t gpueb_get_reserve_mem_phys(unsigned int id)
{
	if (id >= (unsigned int)smp_load_acquire(&gpueb_mem_num)) {
		gpueb_log_d(GPUEB_TAG, "no reserve memory for %d", id);
		return 0;
	} else
		return gpueb_reserve_mblock_ary[id].start_phys;
}
EXPORT_SYMBOL_GPL(gpueb_get_reserve_mem_phys);

phys_addr_t gpueb_get_reserve_mem_virt(unsigned int id)
{
	if (id >= (unsigned int)smp_load_acquire(&gpueb_mem_num)) {
		gpueb_log_d(GPUEB_TAG, "no reserve memory for %d", id);
		return 0;
	} else
		return gpueb_reserve_mblock_ary[id].start_virt;
}
EXPORT_SYMBOL_GPL(gpueb_get_reserve_mem_virt);

phys_addr_t gpueb_get_reserve_mem_size(unsigned int id)
{
	if (id >= (unsigned int)smp_load_acquire(&gpueb_mem_num)) {
		gpueb_log_d(GPUEB_TAG, "no reserve memory for %d", id);
		return 0;
	} else
		return gpueb_reserve_mblock_ary[id].size;
}
EXPORT_SYMBOL_GPL(gpueb_get_reserve_mem_size);

phys_addr_t gpueb_get_reserve_mem_phys_by_name(char *mem_id_name)
{
	int id = -1;
	int i;
	int count = smp_load_acquire(&gpueb_mem_num);

	if (!mem_id_name)
		return 0;
	for (i = 0; i < count; i++) {
		if (!strcmp(gpueb_reserve_mblock_ary_name[i], mem_id_name))
			id = i;
	}

	if (id < 0 || id >= count) {
		gpueb_log_d(GPUEB_TAG, "no reserve memory for %d (%s)", id, mem_id_name);
		return 0;
	} else
		return gpueb_reserve_mblock_ary[id].start_phys;
}
EXPORT_SYMBOL_GPL(gpueb_get_reserve_mem_phys_by_name);

phys_addr_t gpueb_get_reserve_mem_virt_by_name(char *mem_id_name)
{
	int id = -1;
	int i;
	int count = smp_load_acquire(&gpueb_mem_num);

	if (!mem_id_name)
		return 0;
	for (i = 0; i < count; i++) {
		if (!strcmp(gpueb_reserve_mblock_ary_name[i], mem_id_name))
			id = i;
	}

	if (id < 0 || id >= count) {
		gpueb_log_d(GPUEB_TAG, "no reserve memory for %d (%s)", id, mem_id_name);
		return 0;
	} else
		return gpueb_reserve_mblock_ary[id].start_virt;
}
EXPORT_SYMBOL_GPL(gpueb_get_reserve_mem_virt_by_name);

phys_addr_t gpueb_get_reserve_mem_size_by_name(char *mem_id_name)
{
	int id = -1;
	int i;
	int count = smp_load_acquire(&gpueb_mem_num);

	if (!mem_id_name)
		return 0;
	for (i = 0; i < count; i++) {
		if (!strcmp(gpueb_reserve_mblock_ary_name[i], mem_id_name))
			id = i;
	}

	if (id < 0 || id >= count) {
		gpueb_log_d(GPUEB_TAG, "no reserve memory for %d (%s)", id, mem_id_name);
		return 0;
	} else
		return gpueb_reserve_mblock_ary[id].size;
}
EXPORT_SYMBOL_GPL(gpueb_get_reserve_mem_size_by_name);

int gpueb_reserved_mem_init(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct gpueb_memory_keys keys;
	struct gpueb_reserve_mblock *blocks;
	const char *names[20] = {};
	void __iomem *mapping;
	u64 base, bytes, offset = 0;
	u32 id, block_size;
	int count, i, ret;

	/* Boot-owned DT is immutable; publish nothing until all parsing succeeds. */
	if (smp_load_acquire(&gpueb_mem_num))
		return -EBUSY;
	ret = gpueb_validate_boot_memory(pdev);
	if (ret)
		return ret;
	ret = gpueb_memory_property_keys(node, &keys);
	if (ret)
		return ret;
	ret = of_property_read_u64(node, "gpueb_mem_addr", &base);
	if (ret)
		return ret;
	ret = of_property_read_u64(node, "gpueb_mem_size", &bytes);
	if (ret)
		return ret;
	count = of_property_count_u32_elems(node, keys.table);
	if (count <= 0 || count % MEMORY_TBL_ELEM_NUM)
		return -EINVAL;
	count /= MEMORY_TBL_ELEM_NUM;
	if ((unsigned int)count > ARRAY_SIZE(names))
		return -EINVAL;
	blocks = vzalloc(array_size(count, sizeof(*blocks)));
	if (!blocks)
		return -ENOMEM;
	for (i = 0; i < count; i++) {
		ret = of_property_read_string_index(node, keys.names, i, &names[i]);
		if (ret)
			goto free_blocks;
		ret = of_property_read_u32_index(node, keys.table,
			i * MEMORY_TBL_ELEM_NUM, &id);
		if (ret)
			goto free_blocks;
		/* Bound before the first write indexed by a firmware-supplied ID. */
		if (id >= (unsigned int)count) {
			ret = -EINVAL;
			goto free_blocks;
		}
		ret = of_property_read_u32_index(node, keys.table,
			i * MEMORY_TBL_ELEM_NUM + 1, &block_size);
		if (ret)
			goto free_blocks;
		blocks[id].num = id;
		blocks[id].size = block_size;
	}
	for (i = 0; i < count; i++) {
		if (!blocks[i].size || blocks[i].size > bytes - offset) {
			ret = -EINVAL;
			goto free_blocks;
		}
		blocks[i].start_phys = base + offset;
		/* Temporary offset, not an exported virtual address. */
		blocks[i].start_virt = offset;
		offset += blocks[i].size;
	}
	mapping = ioremap_wc(base, bytes);
	if (!mapping) {
		ret = -ENOMEM;
		goto free_blocks;
	}
	for (i = 0; i < count; i++)
		blocks[i].start_virt += (unsigned long)mapping;
	memcpy(gpueb_reserve_mblock_ary_name, names, sizeof(names));
	gpueb_mem_base_phys = base;
	gpueb_mem_base_virt = (unsigned long)mapping;
	gpueb_mem_size = bytes;
	gpueb_reserve_mblock_ary = blocks;
	/* All exported getters acquire this publication before dereferencing. */
	smp_store_release(&gpueb_mem_num, count);
	return 0;

free_blocks:
	vfree(blocks);
	return ret;
}

/* Only after provider clients and transport producers have been quiesced. */
void gpueb_reserved_mem_exit(void)
{
	void __iomem *mapping = (void __iomem *)(unsigned long)gpueb_mem_base_virt;
	struct gpueb_reserve_mblock *blocks = gpueb_reserve_mblock_ary;

	smp_store_release(&gpueb_mem_num, 0);
	gpueb_reserve_mblock_ary = NULL;
	memset(gpueb_reserve_mblock_ary_name, 0, sizeof(gpueb_reserve_mblock_ary_name));
	gpueb_mem_base_phys = 0;
	gpueb_mem_base_virt = 0;
	gpueb_mem_size = 0;
	if (mapping)
		iounmap(mapping);
	vfree(blocks);
}
