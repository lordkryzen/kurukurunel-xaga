// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

/**
 * @file    gpueb.c
 * @brief   GPUEB driver init and probe
 */

/**
 * ===============================================
 * SECTION : Include files
 * ===============================================
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/random.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/arm-smccc.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <mboot_params.h>

#include "gpueb_common.h"
#include "gpueb_helper.h"
#include "gpueb_ipi.h"
#include "gpueb_logger.h"
#include "gpueb_reserved_mem.h"
#include "gpueb_memory_contract.h"
#include "gpueb_lifecycle.h"
#include "gpueb_plat_service.h"
#include "gpueb_hwvoter_dbg.h"
#include "gpueb_debug.h"
#include "gpueb_timesync.h"
#include "gpueb_common.h"
#include "ghpm_wrapper.h"

/*
 * ===============================================
 * SECTION : Local functions declaration
 * ===============================================
 */

static int __gpueb_pdrv_probe(struct platform_device *pdev);
static void gpueb_pdrv_remove(struct platform_device *pdev);

/*
 * ===============================================
 * SECTION : Local variables definition
 * ===============================================
 */

static DEFINE_MUTEX(gpueb_lifecycle_lock);
static struct platform_device *g_bound_device;
static bool g_legacy_profile;
static bool g_memory_started, g_ipi_started, g_logger_started, g_misc_started;
static bool g_attr_started, g_debug_started, g_timesync_started, g_hwvoter_started;
static void __iomem *g_gpueb_gpr_base;
static void __iomem *g_gpueb_cfgreg_base;
static void __iomem *g_mfg0_pwr_con;

#if IS_ENABLED(CONFIG_PM)
static int gpueb_suspend(struct device *dev)
{
	gpueb_timesync_suspend();
	return 0;
}

static int gpueb_resume(struct device *dev)
{
	gpueb_timesync_resume();
	return 0;
}

static const struct dev_pm_ops gpueb_dev_pm_ops = {
	.suspend = gpueb_suspend,
	.resume  = gpueb_resume,
};
#endif

static const struct of_device_id g_gpueb_of_match[] = {
	{ .compatible = "mediatek,gpueb" },
	{ /* sentinel */ }
};

static struct platform_driver g_gpueb_pdrv = {
	.probe = __gpueb_pdrv_probe,
	.remove = gpueb_pdrv_remove,
	.driver = {
		.name = "gpueb",
		.suppress_bind_attrs = true,
		.owner = THIS_MODULE,
		.of_match_table = g_gpueb_of_match,
#if IS_ENABLED(CONFIG_PM)
		.pm = &gpueb_dev_pm_ops,
#endif
	},
};

const struct file_operations gpueb_log_file_ops = {
	.owner = THIS_MODULE,
	.read = gpueb_log_if_read,
	.open = gpueb_log_if_open,
	.poll = gpueb_log_if_poll,
};

static struct miscdevice gpueb_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gpueb",
	.fops = &gpueb_log_file_ops
};

static int gpueb_create_files(void)
{
	int ret = 0;

	ret = misc_register(&gpueb_device);
	if (unlikely(ret != 0)) {
		gpueb_log_i(GPUEB_TAG, "misc register failed");
		return ret;
	}

	g_misc_started = true;
	ret = device_create_file(gpueb_device.this_device,
			&dev_attr_gpueb_mobile_log);
	if (unlikely(ret != 0))
		return ret;
	g_attr_started = true;
	return 0;
}

void __iomem *gpueb_get_gpr_base(void)
{
	return g_gpueb_gpr_base;
}
EXPORT_SYMBOL(gpueb_get_gpr_base);

void __iomem *gpueb_get_gpr_addr(enum gpueb_sram_gpr_id gpr_id)
{
	if (!g_gpueb_gpr_base || (unsigned int)gpr_id > GPUEB_SRAM_GPR63)
		return NULL;
	return g_gpueb_gpr_base + gpr_id * SRAM_GPR_SIZE_4B;
}
EXPORT_SYMBOL(gpueb_get_gpr_addr);

void __iomem *gpueb_get_cfgreg_base(void)
{
	return g_gpueb_cfgreg_base;
}
EXPORT_SYMBOL(gpueb_get_cfgreg_base);

int get_mfg0_pwr_con(void)
{
	if (!g_mfg0_pwr_con)
		return -EOPNOTSUPP;
	return readl(g_mfg0_pwr_con);
}
EXPORT_SYMBOL(get_mfg0_pwr_con);

int mfg0_pwr_sta(void)
{
	if (!g_mfg0_pwr_con)
		return -EOPNOTSUPP;
	return ((readl(g_mfg0_pwr_con) & MFG0_PWR_ACK_BITS) == MFG0_PWR_ACK_BITS)?
		MFG0_PWR_ON : MFG0_PWR_OFF;
}
EXPORT_SYMBOL(mfg0_pwr_sta);

void mfg0_pwr_reset(void)
{
	struct arm_smccc_res res;

	if (!g_mfg0_pwr_con)
		return;
	gpueb_log_d(GPUEB_TAG, "trigger smc");
	arm_smccc_smc(
		MTK_SIP_KERNEL_GPUEB_CONTROL,  /* a0 */
		GPUEB_PWR_RESET_MFG0,          /* a1 */
		GPUEB_MFG0_POWER_RESET_MAGIC_NUMBER,
		0, 0, 0, 0, 0, &res);

	if (res.a0)
		gpueb_log_e(GPUEB_TAG, "failed, ret = %ld", res.a0);
	else
		gpueb_log_d(GPUEB_TAG, "done, ret = %ld", res.a0);
}
EXPORT_SYMBOL(mfg0_pwr_reset);

int is_gpueb_wfi(void)
{
	if (!g_gpueb_cfgreg_base)
		return -EOPNOTSUPP;
	return ((readl(GPUEB_CFGREG_MDSP_CFG) & GPUEB_ON_WFI_BITS) == GPUEB_ON_WFI_BITS)? 1: 0;
}
EXPORT_SYMBOL(is_gpueb_wfi);

/*
 * GPUEB driver probe
 */
bool gpueb_legacy_profile(void)
{
	return g_legacy_profile;
}

/* Called with lifecycle lock; remove users before producers before mappings. */
static int gpueb_probe_cleanup(void)
{
	int ret;

	WRITE_ONCE(g_ghpm_ready, 0);
	if (g_attr_started) {
		device_remove_file(gpueb_device.this_device, &dev_attr_gpueb_mobile_log);
		g_attr_started = false;
	}
	if (g_misc_started) {
		misc_deregister(&gpueb_device);
		g_misc_started = false;
	}
	if (g_debug_started) {
		gpueb_debug_exit();
		g_debug_started = false;
	}
	if (g_hwvoter_started) {
		gpueb_hw_voter_dbg_exit();
		g_hwvoter_started = false;
	}
	ghpm_debug_exit();
	if (g_timesync_started) {
		gpueb_timesync_exit();
		g_timesync_started = false;
	}
	if (g_logger_started) {
		gpueb_logger_exit();
		g_logger_started = false;
	}
	if (g_ipi_started) {
		ret = gpueb_ipi_exit();
		if (ret)
			return ret;
		g_ipi_started = false;
	}
	if (g_memory_started) {
		gpueb_reserved_mem_exit();
		g_memory_started = false;
	}
	g_gpueb_gpr_base = NULL;
	g_gpueb_cfgreg_base = NULL;
	g_mfg0_pwr_con = NULL;
	g_bound_device = NULL;
	g_legacy_profile = false;
	return 0;
}

static int __gpueb_pdrv_probe(struct platform_device *pdev)
{
	struct gpueb_memory_keys keys;
	struct resource *res;
	u32 support = 0, logger = 0, ghpm = 0;
	int ret, cleanup_ret;

	of_property_read_u32(pdev->dev.of_node, "gpueb-support", &support);
	if (!support)
		return 0;
	/* Validate boot-owned firmware memory before any MMIO or IPI publication. */
	ret = gpueb_validate_boot_memory(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "GPUEB boot-memory contract unavailable\n");
	ret = gpueb_memory_property_keys(pdev->dev.of_node, &keys);
	if (ret)
		return ret;
	mutex_lock(&gpueb_lifecycle_lock);
	if (g_bound_device) {
		ret = -EBUSY;
		goto unlock;
	}
	g_bound_device = pdev;
	g_legacy_profile = !strcmp(keys.table, "gpueb_mem_table");
	if (g_legacy_profile) {
		/* The legacy firmware has no native GPR magic/GHPM resource contract. */
		of_property_read_u32(pdev->dev.of_node, "ghpm-support", &ghpm);
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gpueb_base");
		if (!res || ghpm) {
			ret = -EOPNOTSUPP;
			goto cleanup;
		}
	} else {
#define MAP_GPUEB_RESOURCE(name, destination) do { \
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name); \
	if (!res) { ret = -ENODEV; goto cleanup; } \
	destination = devm_ioremap(&pdev->dev, res->start, resource_size(res)); \
	if (!destination) { ret = -ENOMEM; goto cleanup; } \
} while (0)
		MAP_GPUEB_RESOURCE("gpueb_gpr_base", g_gpueb_gpr_base);
		MAP_GPUEB_RESOURCE("gpueb_cfgreg_base", g_gpueb_cfgreg_base);
		MAP_GPUEB_RESOURCE("mfg0_pwr_con", g_mfg0_pwr_con);
#undef MAP_GPUEB_RESOURCE
	}
	ret = gpueb_reserved_mem_init(pdev);
	if (ret)
		goto cleanup;
	g_memory_started = true;
	ret = gpueb_ipi_init(pdev);
	if (ret)
		goto cleanup;
	g_ipi_started = true;
	of_property_read_u32(pdev->dev.of_node, "gpueb-logger-support", &logger);
	if (logger) {
		ret = gpueb_logger_init(pdev, gpueb_get_reserve_mem_virt_by_name("MEM_ID_LOG"),
			gpueb_get_reserve_mem_size_by_name("MEM_ID_LOG"));
		if (ret < 0)
			goto cleanup;
		g_logger_started = true;
		ret = gpueb_create_files();
		if (ret)
			goto cleanup;
	}
	/* Old register-debug protocols are not silently replaced by new SMCs. */
	gpueb_debug_init(pdev);
	g_debug_started = true;
	if (!g_legacy_profile) {
		ret = gpueb_hw_voter_dbg_init();
		if (ret)
			goto cleanup;
		g_hwvoter_started = true;
	}
	ret = gpueb_timesync_init();
	if (ret)
		goto cleanup;
	g_timesync_started = true;
	ghpm_wrapper_init(pdev);
	ret = 0;
	goto unlock;
cleanup:
	cleanup_ret = gpueb_probe_cleanup();
	if (cleanup_ret)
		dev_err(&pdev->dev, "GPUEB quiescence failed: %d\n", cleanup_ret);
unlock:
	mutex_unlock(&gpueb_lifecycle_lock);
	return ret;
}

static void gpueb_pdrv_remove(struct platform_device *pdev)
{
	int ret;

	mutex_lock(&gpueb_lifecycle_lock);
	if (g_bound_device == pdev) {
		ret = gpueb_probe_cleanup();
		if (ret)
			dev_err(&pdev->dev, "GPUEB quiescence failed during remove: %d\n", ret);
	}
	mutex_unlock(&gpueb_lifecycle_lock);
}

/*
 * Register the GPUEB driver
 */
static int __init __gpueb_init(void)
{
	int ret = 0;

	gpueb_log_d(GPUEB_TAG, "start to initialize gpueb driver");

	// Register platform driver
	ret = platform_driver_register(&g_gpueb_pdrv);
	if (ret)
		gpueb_log_i(GPUEB_TAG, "fail to register gpueb driver");

	return ret;
}

/*
 * Unregister the GPUEB driver
 */
static void __exit __gpueb_exit(void)
{
	platform_driver_unregister(&g_gpueb_pdrv);
}

module_init(__gpueb_init);
module_exit(__gpueb_exit);

MODULE_DEVICE_TABLE(of, g_gpueb_of_match);
MODULE_DESCRIPTION("MediaTek GPUEB-PLAT driver");
MODULE_LICENSE("GPL");
