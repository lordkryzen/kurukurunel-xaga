// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */


#include <linux/of.h>
#include <linux/ioport.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/cpu.h>
#include <linux/pm_qos.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/time.h>

#include <lpm.h>

#include <lpm_plat_apmcu.h>
#include <lpm_module.h>

void __iomem *cpu_pm_mcusys_base;
void __iomem *cpu_pm_syssram_base;

#define plat_node_ready()       (cpu_pm_mcusys_base && cpu_pm_syssram_base)

/* qos */
static struct pm_qos_request lpm_plat_qos_req;
static bool apmcu_qos_owned;
static int bp_state=-1, ap_state=-1;

#define lpm_plat_qos_init()\
	cpu_latency_qos_add_request(&lpm_plat_qos_req,\
		PM_QOS_DEFAULT_VALUE)
#define lpm_cpu_off_block()\
	cpu_latency_qos_update_request(&lpm_plat_qos_req, 2)
#define lpm_cpu_off_allow()\
	cpu_latency_qos_update_request(&lpm_plat_qos_req, PM_QOS_DEFAULT_VALUE)
#define lpm_plat_qos_uninit()\
	cpu_latency_qos_remove_request(&lpm_plat_qos_req)

#define CHK(cond) WARN_ON(cond)

#define PWR_LVL_BIT(v)  BIT(4 * v)

#define PWR_LVL_MASK(v) (0xF << (4 * (v)))

#define PWR_LVL0        (PWR_LVL_BIT(0))
#define PWR_LVL1        (PWR_LVL_BIT(1) | PWR_LVL0)
#define PWR_LVL2        (PWR_LVL_BIT(2) | PWR_LVL1)
#define PWR_LVL3        (PWR_LVL_BIT(3) | PWR_LVL2)

#define CORE_LVL	PWR_LVL0
#define CLUSTER_LVL	PWR_LVL1
#define MCUSYS_LVL	PWR_LVL2
#define BUS26M_LVL	PWR_LVL3

#define MAX_PWR_LVL     MCUSYS_LVL

struct lpm_device {
	unsigned int pwr_lvl_mask;
	union {
		struct {
			u32 core:4;
			u32 cluster:4;
			u32 mcusys:4;
			u32 bus26m:4;
		};
		u32 lvl;
	} pwr_on;
	struct lpm_device *parent;
};

static struct lpm_device lp_dev_cpu[NR_CPUS];
static struct lpm_device lp_dev_cluster[nr_cluster_ids];
static struct lpm_device lp_dev_mcusys;

static void _lpm_plat_inc_pwr_cnt(struct lpm_device *dev,
					unsigned int lvl)
{
	unsigned int lvl_cnt;

	lvl_cnt = lvl & dev->pwr_lvl_mask;

	if (!lvl_cnt)
		return;

	/**
	 * Exclusive read-modify-write is ensured by
	 *     lpm_mod_locker in idle flow
	 *     lpm_plat_qos_req in hot-plug flow
	 */
	dev->pwr_on.lvl += lvl_cnt;

	CHK(dev->pwr_on.core >= nr_cpu_ids);

	if (dev->parent)
		_lpm_plat_inc_pwr_cnt(dev->parent, lvl);
}

static void _lpm_plat_dec_pwr_cnt(struct lpm_device *dev,
					unsigned int lvl)
{
	unsigned int lvl_cnt;

	lvl_cnt = lvl & dev->pwr_lvl_mask;

	if (!lvl_cnt)
		return;

	/**
	 * Exclusive read-modify-write is ensured by
	 *     lpm_mod_locker in idle flow
	 *     lpm_plat_qos_req in hot-plug flow
	 */
	dev->pwr_on.lvl -= lvl_cnt;

	CHK((dev->pwr_on.lvl & BIT(31)));

	if (dev->parent)
		_lpm_plat_dec_pwr_cnt(dev->parent, lvl);
}

#define lpm_plat_set_off_lvl(cpu, lvl)				\
do {									\
	if (likely(cpu >= 0 && cpu < nr_cpu_ids))					\
		_lpm_plat_dec_pwr_cnt(&lp_dev_cpu[cpu], lvl);	\
} while (0)

#define lpm_plat_clr_off_lvl(cpu, lvl)				\
do {									\
	if (likely(cpu >= 0 && cpu < nr_cpu_ids))					\
		_lpm_plat_inc_pwr_cnt(&lp_dev_cpu[cpu], lvl);	\
} while (0)

void lpm_plat_set_mcusys_off(int cpu)
{
	lpm_plat_set_off_lvl(cpu, MCUSYS_LVL);
}

void lpm_plat_clr_mcusys_off(int cpu)
{
	lpm_plat_clr_off_lvl(cpu, MCUSYS_LVL);
}

void lpm_plat_set_cluster_off(int cpu)
{
	lpm_plat_set_off_lvl(cpu, CLUSTER_LVL);
}

void lpm_plat_clr_cluster_off(int cpu)
{
	lpm_plat_clr_off_lvl(cpu, CLUSTER_LVL);
}

static inline void lpm_dev_get_cpus_online(void)
{
	int i;

	for_each_online_cpu(i)
		lpm_plat_clr_off_lvl(i, MAX_PWR_LVL);
}

static void lpm_dev_set_cpus_off(void)
{
	int i;

	lp_dev_mcusys.pwr_on.lvl = 0;

	for (i = 0; i < nr_cluster_ids; i++)
		lp_dev_cluster[i].pwr_on.lvl = 0;

	for (i = 0; i < nr_cpu_ids; i++)
		lp_dev_cpu[i].pwr_on.lvl = 0;
}

bool lpm_plat_is_mcusys_off(void)
{
	return !lp_dev_mcusys.pwr_on.mcusys;
}

bool lpm_plat_is_cluster_off(int cpu)
{
	if (unlikely(cpu < 0 || cpu >= nr_cpu_ids || !lp_dev_cpu[cpu].parent))
		return false;

	return !lp_dev_cpu[cpu].parent->pwr_on.cluster;
}

static int lpm_cpuhp_notify_enter(unsigned int cpu)
{
	lpm_cpu_off_block();

	return 0;
}

static int lpm_cpuhp_notify_leave(unsigned int cpu)
{
	lpm_dev_set_cpus_off();
	lpm_dev_get_cpus_online();
	lpm_cpu_off_allow();

	return 0;
}

static int lpm_plat_cpuhp_init(void)
{
 int ret;
 bp_state=cpuhp_setup_state_nocalls_cpuslocked(CPUHP_BP_PREPARE_DYN,"lpm:prepare",
   lpm_cpuhp_notify_enter,lpm_cpuhp_notify_leave);
 if(bp_state<0){ret=bp_state;bp_state=-1;return ret;}
 ap_state=cpuhp_setup_state_nocalls_cpuslocked(CPUHP_AP_ONLINE_DYN,"lpm:online",
   lpm_cpuhp_notify_leave,lpm_cpuhp_notify_enter);
 if(ap_state<0){ret=ap_state;ap_state=-1;cpuhp_remove_state_nocalls_cpuslocked(bp_state);bp_state=-1;return ret;}
 return 0;
}

/*
 * Timespec interfaces utilizing the ktime based ones
 */
static inline void get_monotonic_boottime(struct timespec64 *ts)
{
	*ts = ktime_to_timespec64(ktime_get_boottime());
}

static void lpm_plat_pwr_dev_init(void)
{
	struct lpm_device *dev;
	int i, cluster_id;

	for (i = 0; i < nr_cpu_ids; i++) {

		dev = &lp_dev_cpu[i];
		dev->pwr_lvl_mask = PWR_LVL_MASK(0);

		cluster_id = get_physical_cluster_id(i);

		if (cluster_id < nr_cluster_ids)
			dev->parent = &lp_dev_cluster[cluster_id];
	}

	for (i = 0; i < nr_cluster_ids; i++) {
		dev = &lp_dev_cluster[i];
		dev->pwr_lvl_mask = PWR_LVL_MASK(1);
		dev->parent = &lp_dev_mcusys;
	}

	lp_dev_mcusys.pwr_lvl_mask = PWR_LVL_MASK(2);

	lpm_dev_get_cpus_online();
}

static int lpm_plat_mcusys_ctrl_init(void)
{
 struct device_node *node;struct resource r;int ret;
 node=of_find_compatible_node(NULL,NULL,"mediatek,mcusys-ctrl");
 if(!node)return -ENODEV;
 ret=of_address_to_resource(node,0,&r);
 if(!ret && resource_size(&r)>=4)cpu_pm_mcusys_base=of_iomap(node,0);
 of_node_put(node);
 if(!cpu_pm_mcusys_base)return ret?:-ENOMEM;
 node=of_find_compatible_node(NULL,NULL,"mediatek,cpupm-sysram");
 if(!node){ret=-ENODEV;goto fail;}
 ret=of_address_to_resource(node,0,&r);
 if(!ret && resource_size(&r)<LP_PM_SYSRAM_SIZE)ret=-EINVAL;
 if(!ret)cpu_pm_syssram_base=of_iomap(node,0);
 of_node_put(node);
 if(!cpu_pm_syssram_base){ret=ret?:-ENOMEM;goto fail;}
 return 0;
fail:iounmap(cpu_pm_mcusys_base);cpu_pm_mcusys_base=NULL;return ret;
}

int lpm_plat_apmcu_init(void)
{
 int ret;
 if(!plat_node_ready() || !apmcu_qos_owned)return -ENODEV;
 cpus_read_lock();cpuidle_pause_and_lock();
 lpm_dev_set_cpus_off();lpm_plat_pwr_dev_init();
 ret=lpm_plat_cpuhp_init();
 lpm_cpu_off_allow();
 cpuidle_resume_and_unlock();cpus_read_unlock();return ret;
}

int lpm_plat_apmcu_early_init(void)
{
 int ret=lpm_plat_mcusys_ctrl_init();
 if(ret)return ret;
 lpm_plat_qos_init();apmcu_qos_owned=true;lpm_cpu_off_block();
 cpus_read_lock();cpuidle_pause_and_lock();
 memset_io(cpu_pm_syssram_base+LP_PM_SYSRAM_INFO_OFS,0,LP_PM_SYSRAM_SIZE-LP_PM_SYSRAM_INFO_OFS);
 cpuidle_resume_and_unlock();cpus_read_unlock();return 0;
}
void lpm_plat_apmcu_exit(void)
{
 cpus_read_lock();cpuidle_pause_and_lock();
 if(ap_state>=0){cpuhp_remove_state_nocalls_cpuslocked(ap_state);ap_state=-1;}
 if(bp_state>=0){cpuhp_remove_state_nocalls_cpuslocked(bp_state);bp_state=-1;}
 cpuidle_resume_and_unlock();cpus_read_unlock();
 if(apmcu_qos_owned){lpm_plat_qos_uninit();apmcu_qos_owned=false;}
 if(cpu_pm_syssram_base){iounmap(cpu_pm_syssram_base);cpu_pm_syssram_base=NULL;}
 if(cpu_pm_mcusys_base){iounmap(cpu_pm_mcusys_base);cpu_pm_mcusys_base=NULL;}
}

