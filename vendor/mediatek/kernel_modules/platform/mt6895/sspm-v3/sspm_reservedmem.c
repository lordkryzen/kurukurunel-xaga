// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2011-2015 MediaTek Inc.
 */

#include <linux/module.h>       /* needed by all modules */
#include <linux/init.h>         /* needed by module macros */
#include <linux/fs.h>           /* needed by file_operations* */
#include <linux/miscdevice.h>   /* needed by miscdevice* */
#include <linux/device.h>       /* needed by device_* */
#include <linux/vmalloc.h>      /* needed by kmalloc */
#include <linux/uaccess.h>      /* needed by copy_to_user */
#include <linux/fs.h>           /* needed by file_operations* */
#include <linux/slab.h>         /* needed by kmalloc */
#include <linux/poll.h>         /* needed by poll */
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_fdt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include "sspm_define.h"
#include "sspm_checked.h"
#include "sspm_helper.h"
#include "sspm_reservedmem.h"
#define _SSPM_INTERNAL_
#include "sspm_reservedmem_define.h"


#define SSPM_MEM_RESERVED_KEY "mediatek,reserve-memory-sspm_share"
#define SSPM_MEM_TBL_UNIT 2

static phys_addr_t sspm_mem_base_phys;
static phys_addr_t sspm_mem_base_virt;
static phys_addr_t sspm_mem_size;

static int sspm_reserve_memory_ioremap(struct platform_device *pdev)
{
	struct device_node *rmem_node;
	struct reserved_mem *rmem;
	u32 start = 0, size = 0, legacy_start = 0, legacy_size = 0;

	of_property_read_u32(pdev->dev.of_node, "sspm-res-ram-start", &start);
	of_property_read_u32(pdev->dev.of_node, "sspm-res-ram-size", &size);
	/* Legacy LK fills these names only. Never supply a captured PA. */
	of_property_read_u32(pdev->dev.of_node, "sspm_res_ram_start", &legacy_start);
	of_property_read_u32(pdev->dev.of_node, "sspm_res_ram_size", &legacy_size);
	if (legacy_start) {
		if (start && (start != legacy_start || size != legacy_size))
			return -EINVAL;
		start = legacy_start;
		size = legacy_size;
	}

	rmem_node = of_find_compatible_node(NULL, NULL, SSPM_MEM_RESERVED_KEY);
	if (!rmem_node)
		rmem_node = of_find_compatible_node(NULL, NULL,
			"mediatek,sspm_ap-shared");
	if (!rmem_node)
		return -ENODEV;
	if (!of_property_read_bool(rmem_node, "no-map")) {
		of_node_put(rmem_node);
		return -EINVAL;
	}
	rmem = of_reserved_mem_lookup(rmem_node);
	of_node_put(rmem_node);
	if (!rmem || !rmem->size)
		return -EINVAL;
	if (start) {
		if (!sspm_range_fits(start, size, rmem->base, rmem->size))
			return -EINVAL;
	} else {
		/* A real AP shared reservation is also a valid modern DT source. */
		sspm_mem_base_phys = rmem->base;
		sspm_mem_size = rmem->size;
		return 0;
	}
	sspm_mem_base_phys = start;
	sspm_mem_size = size;
	return 0;
}

phys_addr_t sspm_reserve_mem_get_phys(unsigned int id)
{
	if (id >= NUMS_MEM_ID) {
		pr_err("[SSPM] no reserve memory for 0x%x", id);
		return 0;
	} else
		return sspm_reserve_mblock[id].start_phys;
}
EXPORT_SYMBOL_GPL(sspm_reserve_mem_get_phys);

phys_addr_t sspm_reserve_mem_get_virt(unsigned int id)
{
	if (id >= NUMS_MEM_ID) {
		pr_err("[SSPM] no reserve memory for 0x%x", id);
		return 0;
	} else
		return sspm_reserve_mblock[id].start_virt;
}
EXPORT_SYMBOL_GPL(sspm_reserve_mem_get_virt);

phys_addr_t sspm_reserve_mem_get_size(unsigned int id)
{
	if (id >= NUMS_MEM_ID) {
		pr_err("[SSPM] no reserve memory for 0x%x", id);
		return 0;
	} else
		return sspm_reserve_mblock[id].size;
}
EXPORT_SYMBOL_GPL(sspm_reserve_mem_get_size);

int sspm_reserve_memory_init(void)
{
	unsigned int id;
	phys_addr_t accumlate_memory_size;
	int ret;
	unsigned int sspm_mem_num, m_idx, m_size;

	if (NUMS_MEM_ID == 0)
		return 0;

	ret = sspm_reserve_memory_ioremap(sspm_pdev);
	if (ret)
		return ret;

	if (!sspm_mem_base_phys)
		return -1;

	/* Get reserved memory table from dts */
	ret = of_property_count_u32_elems(sspm_pdev->dev.of_node,
					"sspm-mem-tbl");

	if (ret <= 0 ) {
		pr_info("[SSPM] sspm-mem-tbl is not defined, skip read\n");
		ret = 0;
		/* ret = 0, skip read reserved mem tbl from dts */
	}
	sspm_mem_num = ret / SSPM_MEM_TBL_UNIT;

	for (id = 0; id < sspm_mem_num; id ++) {
		ret = of_property_read_u32_index(sspm_pdev->dev.of_node,
				"sspm-mem-tbl",
				id * SSPM_MEM_TBL_UNIT,
				&m_idx);
		if (ret) {
			pr_err("[SSPM] cannot get memory index(%d)\n", id);
			return -1;
		}

		if (m_idx >= NUMS_MEM_ID) {
			pr_err("[SSPM] unexpected index: %d\n", m_idx);
			return -1;
		}

		ret = of_property_read_u32_index(sspm_pdev->dev.of_node,
				"sspm-mem-tbl",
				(id * SSPM_MEM_TBL_UNIT) + 1,
				&m_size);
		if (ret) {
			pr_err("[SSPM] cannot get memory size index(%d)\n", id);
			return -1;
		}

		sspm_reserve_mblock[m_idx].num = m_idx;
		sspm_reserve_mblock[m_idx].size = m_size;
#ifdef DEBUG
		pr_info("[SSPM] reserved: <%d  0x%x>\n", m_idx, m_size);
#endif
	}

    /* Phy memory */
	accumlate_memory_size = 0;
	for (id = 0; id < NUMS_MEM_ID; id++) {
		if (!sspm_budget_fits(accumlate_memory_size,
		    sspm_reserve_mblock[id].size, sspm_mem_size))
			return -EINVAL;
		sspm_reserve_mblock[id].start_phys = sspm_mem_base_phys +
							accumlate_memory_size;

		accumlate_memory_size += sspm_reserve_mblock[id].size;
	}

    /* Virt memory */
	accumlate_memory_size = 0;
	sspm_mem_base_virt = (phys_addr_t)(uintptr_t)
			ioremap_wc(sspm_mem_base_phys, sspm_mem_size);
	if (!sspm_mem_base_virt)
		return -ENOMEM;

#ifdef DEBUG
	pr_info("[SSPM]reserve mem: virt:0x%llx - 0x%llx (0x%llx)\n",
			sspm_mem_base_virt,
			sspm_mem_base_virt + sspm_mem_size,
			sspm_mem_size);
#endif

	for (id = 0; id < NUMS_MEM_ID; id++) {
		sspm_reserve_mblock[id].start_virt = sspm_mem_base_virt +
							accumlate_memory_size;
		accumlate_memory_size += sspm_reserve_mblock[id].size;
	}
	/* the reserved memory should be larger then expected memory
	 * or sspm_reserve_mblock does not match dts
	 */

	if (accumlate_memory_size > sspm_mem_size)
		return -EINVAL;

#ifdef DEBUG
	for (id = 0; id < NUMS_MEM_ID; id++) {
		pr_info("[SSPM][mem_reserve-%d] ", id);
		pr_info("phys:0x%llx, virt:0x%llx, size:0x%llx\n",
			(unsigned long long)sspm_reserve_mem_get_phys(id),
			(unsigned long long)sspm_reserve_mem_get_virt(id),
			(unsigned long long)sspm_reserve_mem_get_size(id));
	}
#endif

	return 0;
}

#ifdef SSPM_SHARE_BUFFER_SUPPORT
void __iomem *sspm_base;
unsigned int sspm_share_region_base, sspm_share_region_size;
phys_addr_t sspm_sbuf_get(unsigned int offset)
{
	if (!is_sspm_ready()) {
		pr_notice("[SSPM] device resource is not ready\n");
		return 0;
	}

	if (offset < sspm_share_region_base ||
		offset > sspm_share_region_base + sspm_share_region_size) {
		pr_notice("[SSPM] illegal sbuf request: 0x%x\n", offset);
		return 0;
	} else {
		return (phys_addr_t)(sspm_base + offset);
	}
}
EXPORT_SYMBOL_GPL(sspm_sbuf_get);

int sspm_sbuf_init(void)
{
	struct device *dev = &sspm_pdev->dev;
	struct resource *res;
	u32 ret;

	if (sspm_pdev) {
		res = platform_get_resource_byname(sspm_pdev,
			IORESOURCE_MEM, "sspm_base");
		sspm_base = devm_ioremap_resource(dev, res);

		if (IS_ERR((void const *) sspm_base))
			return -1;
	} else {
		return -1;
	}

	ret = of_property_read_u32(sspm_pdev->dev.of_node, "sspm-share-region-base",
						&sspm_share_region_base);
	if (ret) {
		pr_notice("[SSPM] sspm_share_region_base is not defined.\n");
		return -1;
	}

	ret = of_property_read_u32(sspm_pdev->dev.of_node, "sspm-share-region-size",
						 &sspm_share_region_size);
	if (ret) {
		pr_notice("[SSPM] sspm_share_region_size is not defined.\n");
		return -1;
	}

	return 0;
}
#endif
