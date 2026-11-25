// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/fs.h>

#include <lpm_dbg_fs_common.h>

#include <lpm_spm_comm.h>
#include <gs/lpm_pwr_gs.h>
#include <gs/v1/lpm_power_gs.h>

void mt6895_dbg_fs_exit(void)
{
	lpm_cpuidle_fs_deinit();
	lpm_spm_fs_deinit();
	lpm_rc_fs_deinit();
	lpm_dbg_deinit();
}

int __init mt6895_dbg_fs_init(void)
{int ret;ret=lpm_dbg_init();if(ret)return ret;ret=lpm_rc_fs_init();if(ret)return ret;ret=lpm_spm_fs_init();if(ret)return ret;return lpm_cpuidle_fs_init();}
