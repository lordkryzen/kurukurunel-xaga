// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <asm/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpuidle.h>
#include <linux/suspend.h>
#include <linux/cpu.h>

#include <lpm_plat_apmcu.h>
#include <lpm_module.h>
#include "lpm_plat.h"
#include "lpm_plat_suspend.h"

static unsigned int lpm_pwr_state;

/*
 * Please make sure the race condition protection upfront
 * when calling mtk_lp_plat_do_mcusys_prepare_pdn()
 */
int lpm_do_mcusys_prepare_pdn(unsigned int status,
					   unsigned int *resource_req)
{
	lpm_pwr_state |= (status | PLAT_GIC_MASKED
				| PLAT_MCUSYSOFF_PREPARED);
	return 0;
}

/*
 * Please make sure the race condition protection upfront
 * when calling mtk_lp_plat_do_mcusys_prepare_on()
 */
static int __lpm_do_mcusys_prepare_on(unsigned int clr_status)
{
	lpm_pwr_state &= ~(clr_status | PLAT_GIC_MASKED
				  | PLAT_MCUSYSOFF_PREPARED);

	return 0;
}

int lpm_do_mcusys_prepare_on(void)
{
	unsigned int status = lpm_pwr_state;

	return __lpm_do_mcusys_prepare_on(status);
}

static bool platform_ready;
module_param(platform_ready,bool,0444);
static int __init lpm_plat_init(void)
{
 int ret;long fw;unsigned int sleep_flags=lock_system_sleep();
 ret=lpm_plat_apmcu_early_init();if(ret)goto fail_apmcu;
 ret=lpm_plat_apmcu_init();if(ret)goto fail_apmcu;
 ret=lpm_model_suspend_init();if(ret)goto fail_apmcu;
 cpus_read_lock();cpuidle_pause_and_lock();
 fw=lpm_smc_cpu_pm(VALIDATE_PWR_STATE_CTRL,MT_LPM_SMC_ACT_SET,PSCI_E_SUCCESS,0);
 cpuidle_resume_and_unlock();cpus_read_unlock();
 if(fw<0){ret=fw;goto fail_model;}
 platform_ready=true;unlock_system_sleep(sleep_flags);return 0;
fail_model:lpm_model_suspend_exit();
fail_apmcu:lpm_plat_apmcu_exit();unlock_system_sleep(sleep_flags);return ret;
}
static void __exit lpm_plat_exit(void)
{
 unsigned int sleep_flags=lock_system_sleep();platform_ready=false;
 lpm_model_suspend_exit();lpm_plat_apmcu_exit();unlock_system_sleep(sleep_flags);
}

module_init(lpm_plat_init);
module_exit(lpm_plat_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MTK low power platform module");
MODULE_AUTHOR("MediaTek Inc.");
