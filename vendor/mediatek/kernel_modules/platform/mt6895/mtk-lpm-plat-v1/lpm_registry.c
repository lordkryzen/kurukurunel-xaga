// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/cpuidle.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include "lpm_registry.h"
#include "lpm_internal.h"

int lpm_do_work(int type, blockcall call, void *priv)
{
 int cpu,ret=0;
 if(!call || (type!=LPM_REG_PER_CPU && type!=LPM_REG_ALL_ONLINE))return -EINVAL;
 cpus_read_lock();cpuidle_pause_and_lock();
 if(type==LPM_REG_ALL_ONLINE)ret=call(cpumask_first(cpu_online_mask),priv);
 else for_each_possible_cpu(cpu){ret=call(cpu,priv);if(ret)break;}
 cpuidle_resume_and_unlock();cpus_read_unlock();return ret;
}
