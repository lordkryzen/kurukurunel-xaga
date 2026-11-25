// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include <mtk_cpupm_dbg.h>
#include <lpm_dbg_common_v1.h>
#include <lpm_module.h>

#include <lpm_dbg_fs_common.h>
#include <lpm_dbg_logger.h>

#include <mt6895_lpm_trace_event.h>
#include <mt6895_dbg_fs.h>
#include <mt6895_dbg_power_gs.h>
#include <mt6895_lpm_logger.h>

extern void mtk_lp_sysfs_remove_all(void);
extern void mtk_lp_sysfs_error_reset(void);
extern int mtk_lp_sysfs_error_get(void);
static bool commonfs,devicefs,cpupm,tracefs,logger,pm_driver;
static bool debug_ready;
module_param(debug_ready,bool,0444);
void mt6895_dbg_exit(void)
{
 struct lpm_dbg_plat_ops empty={0};
 mtk_lp_sysfs_remove_all();
 if(pm_driver){lpm_dbg_pm_exit();pm_driver=false;}
 if(logger){lpm_logger_deinit();logger=false;}
 if(tracefs){mt6895_lpm_trace_deinit();tracefs=false;}
 if(cpupm){mtk_cpupm_dbg_exit();cpupm=false;}
 if(devicefs){mt6895_dbg_fs_exit();devicefs=false;}
 if(commonfs){lpm_dbg_common_fs_exit();commonfs=false;}
 lpm_dbg_plat_ops_register(&empty);debug_ready=false;
}
int __init mt6895_dbg_init(void)
{
 int ret;
 mtk_lp_sysfs_error_reset();
 mt6895_dbg_ops_register();
 ret=lpm_dbg_common_fs_init();if(ret)goto fail;commonfs=true;
 /* Flags include partially constructed owned subtrees: root removal drains them. */
 devicefs=true;ret=mt6895_dbg_fs_init();if(ret)goto fail;
 ret=mtk_lp_sysfs_error_get();if(ret)goto fail;
 ret=mtk_cpupm_dbg_init();if(ret)goto fail;cpupm=true;
 tracefs=true;ret=mt6895_lpm_trace_init();if(ret)goto fail;
 ret=lpm_logger_init();if(ret)goto fail;logger=true;
 ret=mtk_lp_sysfs_error_get();if(ret)goto fail;
 ret=lpm_dbg_pm_init();if(ret)goto fail;pm_driver=true;
 debug_ready=true;return 0;
fail:mt6895_dbg_exit();return ret;
}

module_init(mt6895_dbg_init);
module_exit(mt6895_dbg_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mt6895 low power debug module");
MODULE_AUTHOR("MediaTek Inc.");
