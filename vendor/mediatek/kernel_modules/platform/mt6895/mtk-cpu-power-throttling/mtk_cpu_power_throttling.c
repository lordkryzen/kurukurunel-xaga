// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Samuel Hsieh <samuel.hsieh@mediatek.com>
 */
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/arm-smccc.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/nvmem-consumer.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include "mtk_battery_oc_throttling.h"
#include "mtk_low_battery_throttling.h"
#include "mtk_bp_thl.h"
#include "mtk_cpu_power_throttling.h"
#include "core_ctl.h"

#if !IS_BUILTIN(CONFIG_MTK_CPU_POWER_THROTTLING)
#define CREATE_TRACE_POINTS
#endif
#include "mtk_low_battery_throttling_trace.h"

#define CPU_LIMIT_FREQ 900000
#define CPU_UNLIMIT_FREQ 2147483647
#define CLUSTER_NUM 3
#define CORE_NUM 8
#define NAME_LENGTH 32
#define DISABLE_MPMM 3

static int cpu_pt_table_idx;
static unsigned int system_boot_completed;
static bool bootup_pt_support;
static bool switch_pt;
static int lbat_tb_num;
static unsigned int mpmm_gear;

enum cpu_pt_table_num {
	CPU_PT_TABLE0,
	CPU_PT_TABLE1,
	CPU_PT_TABLE2,
	CPU_PT_TABLE3,
	CPU_PT_TABLE4,
	CPU_PT_TABLE5,
	CPU_PT_TABLE6,
	CPU_PT_TABLE_MAX
};

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

struct cpu_pt_priv {
	char max_lv_name[NAME_LENGTH];
	char freq_limit_name[NAME_LENGTH];
	char core_active_name[NAME_LENGTH];
	u32 max_lv;
	u32 cur_lv;
	u32 *freq_limit;
	u32 *core_active;
	u32 mpmm_enable;
	u32 mpmm_activate_lv;
	u32 *soc_limit_cmep_lv;
};

struct cpu_bootup_pt_priv {
	char freq_limit_booting_name[NAME_LENGTH];
	u32 max_lv;
	u32 bootmode;
	u32 *freq_limit_booting;
};

struct cpu_pt_table {
	char freq_limit_name[NAME_LENGTH];
	u32 max_tb_num;
	u32 *freq_limit;
};

struct cpu_pause_tbl {
	int (*pause_func)(unsigned int cpu, bool is_pause, unsigned int request_mask);
};

static struct cpu_pt_priv cpu_pt_info[POWER_THROTTLING_TYPE_MAX] = {
	[LBAT_POWER_THROTTLING] = {
		.max_lv_name = "lbat-max-level",
		.freq_limit_name = "lbat-limit-freq-lv",
		.core_active_name = "",
		.max_lv = LOW_BATTERY_LEVEL_NUM - 1,
	},
	[OC_POWER_THROTTLING] = {
		.max_lv_name = "oc-max-level",
		.freq_limit_name = "oc-limit-freq-lv",
		.core_active_name = "",
		.max_lv = BATTERY_OC_LEVEL_NUM - 1,
	},
	[SOC_POWER_THROTTLING] = {
		.max_lv_name = "soc-max-level",
		.freq_limit_name = "soc-limit-freq-lv",
		.core_active_name = "soc-core-active-lv",
		.max_lv = BATTERY_PERCENT_LEVEL_NUM - 1,
	},
};

static struct cpu_pt_table cpu_pt_tb[CPU_PT_TABLE_MAX] = {
	[CPU_PT_TABLE0] = {
		.freq_limit_name = "lbat-limit-freq-lv",
	},
	[CPU_PT_TABLE1] = {
		.freq_limit_name = "lbat-limit-freq-tb1-lv",
	},
	[CPU_PT_TABLE2] = {
		.freq_limit_name = "lbat-limit-freq-tb2-lv",
	},
	[CPU_PT_TABLE3] = {
		.freq_limit_name = "lbat-limit-freq-tb3-lv",
	},
	[CPU_PT_TABLE4] = {
		.freq_limit_name = "lbat-limit-freq-tb4-lv",
	},
	[CPU_PT_TABLE5] = {
		.freq_limit_name = "lbat-limit-freq-tb5-lv",
	},
	[CPU_PT_TABLE6] = {
		.freq_limit_name = "lbat-limit-freq-tb6-lv",
	},
};

static struct cpu_bootup_pt_priv cpu_bootup_pt_info = {
	.freq_limit_booting_name = "limit-freq-bootmode",
	.max_lv = 1,
	.bootmode = 0,
};

static DEFINE_MUTEX(cpu_thr_lock);
static DEFINE_MUTEX(cpu_freq_lock);
static unsigned int cur_core_active[CORE_NUM];
static struct cpu_pause_tbl cicb = {
	.pause_func = NULL,
};

static LIST_HEAD(pt_policy_list);
static LIST_HEAD(bootup_pt_policy_list);

#if IS_ENABLED(CONFIG_MTK_BATTERY_PERCENT_THROTTLING)
static void __iomem *cmep_addr;
static void __iomem *swpt_freq_limit_B_addr;

static int cmep_init(void)
{
	struct device_node *dev_node;
	struct platform_device *pdev_temp;
	struct resource *sram_res;
	int ret = 0;

	/* init cmep */
	dev_node = of_find_node_by_name(NULL, "cmep");
	if (!dev_node) {
		pr_info("failed to find node cmep @ %s\n", __func__);
		return -ENODEV;
	}

	pdev_temp = of_find_device_by_node(dev_node);
	of_node_put(dev_node); // release reference

	if (!pdev_temp) {
		pr_info("failed to find cmep pdev @ %s\n", __func__);
		return -EINVAL;
	}

	sram_res = platform_get_resource(pdev_temp, IORESOURCE_MEM, 0);
	if (sram_res) {
		cmep_addr = ioremap(sram_res->start, resource_size(sram_res));
		if (!cmep_addr) {
			pr_info("%s ioremap failed\n", __func__);
			cmep_addr = NULL;
			ret = -ENOMEM; goto out_put;
		}
	} else {
		pr_info("%s can't find cmep resource\n", __func__);
		cmep_addr = NULL;
		ret = -ENODEV; goto out_put;
	}
	sram_res = platform_get_resource(pdev_temp, IORESOURCE_MEM, 1);
	if (sram_res) {
		swpt_freq_limit_B_addr = ioremap(sram_res->start, resource_size(sram_res));
		if (!swpt_freq_limit_B_addr) {
			pr_info("%s ioremap failed\n", __func__);
			swpt_freq_limit_B_addr = NULL;
			ret = -ENOMEM; goto out_put;
		}
	} else {
		pr_info("%s can't find swpt_freq_limit_B_addr resource\n", __func__);
		swpt_freq_limit_B_addr = NULL;
		ret = -ENODEV; goto out_put;
	}
	iowrite8(1, cmep_addr); // set init cmep_addr default value
	iowrite32(CPU_UNLIMIT_FREQ, swpt_freq_limit_B_addr); // set init swpt_freq_limit_B_addr default value
out_put:
	put_device(&pdev_temp->dev);
	if (ret) {
		if (cmep_addr) { iounmap(cmep_addr); cmep_addr = NULL; }
		if (swpt_freq_limit_B_addr) { iounmap(swpt_freq_limit_B_addr); swpt_freq_limit_B_addr = NULL; }
	}
	return ret;
}

static void cmep_exit(void)
{
	if (cmep_addr) {
		iounmap(cmep_addr);
		pr_info("cmep_addr unmapped\n");
		cmep_addr = NULL;
	} else
		pr_info("cmep_addr already unmapped\n");

	if (swpt_freq_limit_B_addr) {
		iounmap(swpt_freq_limit_B_addr);
		pr_info("swpt_freq_limit_B_addr unmapped\n");
		swpt_freq_limit_B_addr = NULL;
	} else
		pr_info("swpt_freq_limit_B_addr already unmapped\n");

}
#endif

static int pt_set_cpu_active(unsigned int cpu, bool active)
{
	int ret = -EPERM;
	bool pause;

	if (cur_core_active[cpu] == active)
		return 0;

	if (cicb.pause_func) {
		pause = (active == false) ? true : false;
		ret = cicb.pause_func(cpu, pause, POWER_THROTTLE_FORCE_PAUSE);
	}

	if (ret >= 0) {
		cur_core_active[cpu] = active;
		pr_info("%s: PT success to set cpu%d active=%d, ret=%d\n", __func__, cpu, active, ret);
	} else {
		pr_info("%s: PT failed to set cpu%d active=%d, ret=%d\n", __func__, cpu, active, ret);
	}
	return ret;
}

#if IS_ENABLED(CONFIG_MTK_LOW_BATTERY_POWER_THROTTLING)
static void cpu_pt_low_battery_cb(enum LOW_BATTERY_LEVEL_TAG level, void *data)
{
	struct cpu_pt_policy *pt_policy;
	int cpu;
	s32 freq_limit;
	u32 limit_val[3] = {0};
	int i = 0;

	mutex_lock(&cpu_freq_lock);
	cpu_pt_info[LBAT_POWER_THROTTLING].cur_lv = level;

	if (level > cpu_pt_info[LBAT_POWER_THROTTLING].max_lv) {
		mutex_unlock(&cpu_freq_lock);
		return;
	}
	list_for_each_entry(pt_policy, &pt_policy_list, cpu_pt_list) {
		if (pt_policy->pt_type == LBAT_POWER_THROTTLING) {
			if (level != LOW_BATTERY_LEVEL_0)
				freq_limit = pt_policy->freq_limit[level-1];
			else
				freq_limit = FREQ_QOS_MAX_DEFAULT_VALUE;
			freq_qos_update_request(&pt_policy->qos_req, freq_limit);
			cpu = pt_policy->cpu;
			trace_low_battery_throttling_cpu_freq(cpu, freq_limit);
			if (i < (sizeof(limit_val)/sizeof(u32))) {
				limit_val[i] = freq_limit;
				i++;
			}
		}
	}
	mutex_unlock(&cpu_freq_lock);
	pr_info("[PT][LV] throttle level=%d, cpub_limit_freq=%d cpum_limit_freq=%d cpul_limit_freq=%d\n",
		level, limit_val[0], limit_val[1], limit_val[2]);
}
#endif

static void cpu_bootup_pt_trigger(void)
{
	struct cpu_bootup_pt_policy *bootup_pt_policy;
	s32 freq_limit;

	list_for_each_entry(bootup_pt_policy, &bootup_pt_policy_list, cpu_bootup_pt_list) {
		freq_limit = bootup_pt_policy->freq_limit_booting[0];
		pr_info("%s: freq_limit=%d\n", __func__, freq_limit);
		freq_qos_update_request(&bootup_pt_policy->qos_req, freq_limit);
	}
}

static void mpmm_enable (unsigned int gear)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_KERNEL_PT_CONTROL, PT_OP_SET_MPMM, gear, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		pr_info("%s, SMC call fail", __func__);
}

#if IS_ENABLED(CONFIG_MTK_BATTERY_OC_POWER_THROTTLING)
static void cpu_pt_over_current_cb(enum BATTERY_OC_LEVEL_TAG level, void *data)
{
	struct cpu_pt_policy *pt_policy;
	s32 freq_limit;
	u32 limit_val[3] = {0};
	int i = 0;

	if (level > cpu_pt_info[OC_POWER_THROTTLING].max_lv)
		return;
	list_for_each_entry(pt_policy, &pt_policy_list, cpu_pt_list) {
		if (pt_policy->pt_type == OC_POWER_THROTTLING) {
			if (level != BATTERY_OC_LEVEL_0)
				freq_limit = pt_policy->freq_limit[level-1];
			else
				freq_limit = FREQ_QOS_MAX_DEFAULT_VALUE;
			freq_qos_update_request(&pt_policy->qos_req, freq_limit);
			if (i < (sizeof(limit_val)/sizeof(u32))) {
				limit_val[i] = freq_limit;
				i++;
			}
		}
	}
	pr_info("[PT][OC] throttle level=%d, cpub_limit_freq=%d cpum_limit_freq=%d cpul_limit_freq=%d\n",
		level, limit_val[0], limit_val[1], limit_val[2]);
}
#endif
#if IS_ENABLED(CONFIG_MTK_BATTERY_PERCENT_THROTTLING)
static void cpu_pt_battery_percent_cb(enum BATTERY_PERCENT_LEVEL_TAG level)
{
	struct cpu_pt_policy *pt_policy;
	struct cpu_pt_priv *pt_info_p = &cpu_pt_info[SOC_POWER_THROTTLING];
	s32 freq_limit;
	int idx = 0, i = 0, active;
	u32 limit_val[3] = {0};

	if (level > cpu_pt_info[SOC_POWER_THROTTLING].max_lv)
		return;

	list_for_each_entry(pt_policy, &pt_policy_list, cpu_pt_list) {
		if (pt_policy->pt_type == SOC_POWER_THROTTLING) {
			if (level != BATTERY_PERCENT_LEVEL_0)
				freq_limit = pt_policy->freq_limit[level-1];
			else
				freq_limit = FREQ_QOS_MAX_DEFAULT_VALUE;
			freq_qos_update_request(&pt_policy->qos_req, freq_limit);
			if (i < (sizeof(limit_val)/sizeof(u32))) {
				limit_val[i] = freq_limit;
				i++;
			}
		}
	}
	pr_info("[PT][BP] throttle level=%d, cpub_limit_freq=%d cpum_limit_freq=%d cpul_limit_freq=%d\n",
		level, limit_val[0], limit_val[1], limit_val[2]);

	i = 0;
	mutex_lock(&cpu_thr_lock);
	for_each_possible_cpu(i) {
		if (level == BATTERY_PERCENT_LEVEL_0)
			pt_set_cpu_active(i, true);
		else {
			idx = (level - 1) * CORE_NUM + i;
			active = (pt_info_p->core_active[idx] > 0) ? true : false;
			pt_set_cpu_active(i, active);
		}
	}
	pt_info_p->cur_lv = level;
	mutex_unlock(&cpu_thr_lock);

	if (pt_info_p->mpmm_enable != 0 && level >= pt_info_p->mpmm_activate_lv)
		mpmm_enable(mpmm_gear);
	else if (pt_info_p->mpmm_enable != 0 && level < pt_info_p->mpmm_activate_lv)
		mpmm_enable (DISABLE_MPMM);

	// CME policy trigger by battery percentage
	if (!cmep_addr) {
		pr_info("cmep_addr is NULL, skip write!\n");
		return;
	}

	u8 tcm_val = 1;
	s32 freq_limit_B = CPU_UNLIMIT_FREQ;
	int lv_idx;

	if (!cmep_addr || !swpt_freq_limit_B_addr) {
		pr_info("%s: addr is NULL, skip tcm control (cmep:%p, B:%p)\n",
			__func__, cmep_addr, swpt_freq_limit_B_addr);
		return;
	}

	if (level < 0) {
		pr_info("%s: invalid level %d, skip tcm control\n", __func__, level);
		goto write_reg;
	}

	if (level == BATTERY_PERCENT_LEVEL_0)
		goto write_reg;

	lv_idx = level - 1;
	if (lv_idx < 0 || lv_idx >= pt_info_p->max_lv) {
		pr_info("%s: invalid lv_idx %d (level=%d, max_lv=%u), set default\n",
			__func__, lv_idx, level, pt_info_p->max_lv);
		goto write_reg;
	}

	tcm_val = (pt_info_p->soc_limit_cmep_lv[lv_idx] == 0) ? 0 : 1;
	freq_limit_B = pt_info_p->freq_limit[lv_idx * CLUSTER_NUM + 2];

write_reg:
	iowrite8(tcm_val, cmep_addr);
	iowrite32(freq_limit_B, swpt_freq_limit_B_addr);

	// pr_info("%s: level=%d, tcm_val=%u, B=%d\n", __func__, level, tcm_val, freq_limit_B);
}
#endif
static int __used cpu_limit_default_setting(struct device *dev, enum cpu_pt_type type)
{
	struct device_node *np = dev->of_node;
	int i, max_lv, ret;
	struct cpu_pt_priv *pt_info_p;

	pt_info_p = &cpu_pt_info[type];
	if (type == LBAT_POWER_THROTTLING)
		max_lv = 2;
	else if (type == OC_POWER_THROTTLING)
		max_lv = 1;
	else
		max_lv = 0;
	pt_info_p->max_lv = max_lv;
	if (!pt_info_p->max_lv)
		return 0;

	pt_info_p->freq_limit = kzalloc(sizeof(u32) * pt_info_p->max_lv * CLUSTER_NUM, GFP_KERNEL);
	pt_info_p->core_active = kzalloc(sizeof(u32) * pt_info_p->max_lv * CORE_NUM, GFP_KERNEL);
	if (!pt_info_p->freq_limit || !pt_info_p->core_active)
		return -ENOMEM;
	for (i = 0; i < max_lv * CORE_NUM; i++) pt_info_p->core_active[i] = 1;
	for (i = 0; i < CLUSTER_NUM; i++) {
		pt_info_p->freq_limit[i] = CPU_LIMIT_FREQ;
		pt_info_p->core_active[i] = 1;
	}

	if (type == LBAT_POWER_THROTTLING) {
		ret = of_property_read_u32_array(np, "lbat_cpu_limit",
			&pt_info_p->freq_limit[0], 3);
		if (ret < 0)
			pr_notice("%s: get lbat cpu limit fail %d\n", __func__, ret);
	} else if (type == OC_POWER_THROTTLING) {
		ret = of_property_read_u32_array(np, "oc_cpu_limit",
			&pt_info_p->freq_limit[0], 3);
		if (ret < 0)
			pr_notice("%s: get oc cpu limit fail %d\n", __func__, ret);
	}
	for (i = 1; i < pt_info_p->max_lv; i++) {
		memcpy(&pt_info_p->freq_limit[i * CLUSTER_NUM], &pt_info_p->freq_limit[0],
			sizeof(u32) * CLUSTER_NUM);
		memcpy(&pt_info_p->core_active[i * CORE_NUM], &pt_info_p->core_active[0],
			sizeof(u32) * CORE_NUM);
	}
	return 0;
}

static unsigned int delay_time_s;
static bool parse_bootup_pt_table(struct device_node *np, struct tag_bootmode *tag)
{
	struct cpu_bootup_pt_priv *bootup_pt_info_p;
	int ret, k, bootup_temp, bootup_voltage;
	int bootup_temp_count, bootup_volts_count, num;
	int temp_stage, volt_stage, val, index;
	char buf[NAME_LENGTH];
	int *temp_threshold = NULL, *volt_threshold = NULL;

	bootup_pt_info_p = &cpu_bootup_pt_info;
	bootup_pt_info_p->max_lv = 1;
	bootup_pt_info_p->freq_limit_booting = kzalloc(sizeof(u32) * bootup_pt_info_p->max_lv *
		CLUSTER_NUM, GFP_KERNEL);
	if (!bootup_pt_info_p->freq_limit_booting)
		return false;

	memset(buf, 0, sizeof(buf));

	bootup_temp_count = of_property_count_u32_elems(np, "bootup-temperature-stage-threshold");
	bootup_volts_count = of_property_count_u32_elems(np, "bootup-voltage-threshold");
	num = of_property_count_u32_elems(np, "bootup-throttle-level");

	if (num != (bootup_temp_count + 1)*(bootup_volts_count + 1))
		return false;

	temp_threshold = kcalloc(bootup_temp_count, sizeof(u32), GFP_KERNEL);
	if (!temp_threshold)
		return false;
	volt_threshold = kcalloc(bootup_volts_count, sizeof(u32), GFP_KERNEL);
	if (!temp_threshold || !volt_threshold) {
		kfree(temp_threshold);
		return false;
	}
	ret = of_property_read_u32(np, "bootup-temp", &bootup_temp);
	ret |= of_property_read_u32(np, "bootup-voltage", &bootup_voltage);
	ret |= of_property_read_u32_array(np, "bootup-temperature-stage-threshold", temp_threshold, bootup_temp_count);
	ret |= of_property_read_u32_array(np, "bootup-voltage-threshold", volt_threshold, bootup_volts_count);
	if (ret < 0) {
		kfree(temp_threshold);
		kfree(volt_threshold);
		return false;
	}

	for (temp_stage = 0; temp_stage < bootup_temp_count; temp_stage++) {
		if (bootup_temp > temp_threshold[temp_stage])
			break;
	}
	for (volt_stage = 0; volt_stage < bootup_volts_count; volt_stage++) {
		if (bootup_voltage > volt_threshold[volt_stage])
			break;
	}

	index = temp_stage * (bootup_volts_count + 1) + volt_stage;
	if(index >= num){
		kfree(temp_threshold);
		kfree(volt_threshold);
		return false;
	}

	ret = of_property_read_u32_index(np, "bootup-throttle-level", index, &val);
	if (ret < 0) {
		kfree(temp_threshold);
		kfree(volt_threshold);
		return false;
	}
	if (val != 0){
		ret = snprintf(buf, sizeof(buf), "%s%d-lv%d", bootup_pt_info_p->freq_limit_booting_name,
				tag->bootmode, val);
		if (ret < 0){
			pr_notice("can't merge %s %d\n", bootup_pt_info_p->freq_limit_booting_name, tag->bootmode);
			for (k = 0; k < CLUSTER_NUM; k++)
				bootup_pt_info_p->freq_limit_booting[k] = CPU_UNLIMIT_FREQ;
			kfree(temp_threshold);
			kfree(volt_threshold);
			return false;
		}
		ret = of_property_read_u32_array(np, buf,
			&bootup_pt_info_p->freq_limit_booting[0], CLUSTER_NUM);
		if (ret < 0) {
			pr_notice("%s: get %s fail %d\n", __func__, buf, ret);
			for (k = 0; k < CLUSTER_NUM; k++)
				bootup_pt_info_p->freq_limit_booting[k] = CPU_UNLIMIT_FREQ;
			kfree(temp_threshold);
			kfree(volt_threshold);
			return false;
		}
	} else if (val == 0){
		pr_notice("%s: bootup unlimited", __func__);
		for (k = 0; k < CLUSTER_NUM; k++)
			bootup_pt_info_p->freq_limit_booting[k] = CPU_UNLIMIT_FREQ;
		kfree(temp_threshold);
		kfree(volt_threshold);
		return false;
	}
	ret = of_property_read_u32(np,
		"bootup-delay-release-time", &delay_time_s);
	if (ret < 0) {
		delay_time_s = 0;
		pr_info("[%s] read bootup-delay-release-time fail, ret = %d\n",
			__func__, ret);
	}
	pr_notice("%s: bootmode:0x%x, bootup_temp:%d, bootup_voltage:%d\n",
		__func__, tag->bootmode, bootup_temp, bootup_voltage);
	kfree(temp_threshold);
	kfree(volt_threshold);
	return true;

}

static bool parse_switchpt_table(struct device_node *np)
{
	struct cpu_pt_table *cpu_pt_table;
	int i, j, k, ret;
	char buf[NAME_LENGTH];

	for (i = 0; i < lbat_tb_num; i++) {
		cpu_pt_table = &cpu_pt_tb[i];
		cpu_pt_table->max_tb_num = lbat_tb_num;
		cpu_pt_table->freq_limit = kzalloc(sizeof(u32) * cpu_pt_info[LBAT_POWER_THROTTLING].max_lv *
			CLUSTER_NUM, GFP_KERNEL);
		if (!cpu_pt_table->freq_limit)
			return false;

		for (j = 0; j < cpu_pt_info[LBAT_POWER_THROTTLING].max_lv; j++) {
			memset(buf, 0, sizeof(buf));
			ret = snprintf(buf, sizeof(buf), "%s%d", cpu_pt_table->freq_limit_name, j+1);
			if (ret < 0) {
				pr_notice("can't merge %s %d\n", cpu_pt_table->freq_limit_name, j+1);
				for (k = 0; k < CLUSTER_NUM; k++)
					cpu_pt_table->freq_limit[j * CLUSTER_NUM + k] = CPU_UNLIMIT_FREQ;
				return false;
			}

			ret = of_property_read_u32_array(np, buf,
				&cpu_pt_table->freq_limit[j * CLUSTER_NUM], CLUSTER_NUM);
			if (ret < 0) {
				pr_notice("%s: get %s fail %d\n", __func__, buf, ret);
				for (k = 0; k < CLUSTER_NUM; k++)
					cpu_pt_table->freq_limit[j * CLUSTER_NUM + k] = CPU_UNLIMIT_FREQ;
				return false;
			}
		}
	}
	return true;
}

static int __used parse_cpu_limit_table(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *np_bootmode;
	int i, j, k, num, ret;
	struct cpu_pt_priv *pt_info_p;
	struct tag_bootmode *tag;

	char buf[NAME_LENGTH];

	for (i = 0; i < POWER_THROTTLING_TYPE_MAX; i++) {
		pt_info_p = &cpu_pt_info[i];
		ret = of_property_read_u32(np, pt_info_p->max_lv_name, &num);
		if (ret < 0) {
			ret = cpu_limit_default_setting(dev, i);
			if (ret) return ret;
			continue;
		} else if (num <= 0 || num > pt_info_p->max_lv) {
			pt_info_p->max_lv = 0;
			continue;
		}

		pt_info_p->max_lv = num;
		pt_info_p->freq_limit = kzalloc(sizeof(u32) * pt_info_p->max_lv * CLUSTER_NUM, GFP_KERNEL);
		if (!pt_info_p->freq_limit)
			return -ENOMEM;

		pt_info_p->core_active = kzalloc(sizeof(u32) * pt_info_p->max_lv * CORE_NUM, GFP_KERNEL);
		if (!pt_info_p->core_active)
			return -ENOMEM;

		if (i == SOC_POWER_THROTTLING) {
			pt_info_p->soc_limit_cmep_lv = kzalloc(sizeof(u32) * pt_info_p->max_lv , GFP_KERNEL);
			if (!pt_info_p->soc_limit_cmep_lv) return -ENOMEM;
		}

		for (j = 0; j < pt_info_p->max_lv; j++) {
			memset(buf, 0, sizeof(buf));
			ret = snprintf(buf, sizeof(buf), "%s%d", pt_info_p->freq_limit_name, j+1);
			if (ret < 0)
				pr_notice("can't merge %s %d\n", pt_info_p->freq_limit_name, j+1);

			ret = of_property_read_u32_array(np, buf,
				&pt_info_p->freq_limit[j * CLUSTER_NUM], CLUSTER_NUM);
			if (ret < 0) {
				pr_notice("%s: get %s fail %d\n", __func__, buf, ret);
				for (k = 0; k < CLUSTER_NUM; k++)
					pt_info_p->freq_limit[j * CLUSTER_NUM + k] = CPU_UNLIMIT_FREQ;
			}

			if (i == SOC_POWER_THROTTLING) {
				memset(buf, 0, sizeof(buf));
				ret = snprintf(buf, sizeof(buf), "%s%d", pt_info_p->core_active_name, j+1);
				if (ret < 0)
					pr_notice("can't merge %s %d\n", pt_info_p->core_active_name, j+1);
				ret = of_property_read_u32_array(np, buf,
					&pt_info_p->core_active[j * CORE_NUM], CORE_NUM);
				if (ret < 0) {
					pr_notice("%s: get %s fail %d set core limit to 1\n", __func__, buf, ret);
					for (k = 0; k < CORE_NUM; k++)
						pt_info_p->core_active[j * CORE_NUM + k] = 1;
				}
				// For cmep
				if (cmep_addr != NULL) {
					memset(buf, 0, sizeof(buf));
					if (snprintf(buf, sizeof(buf), "soc-limit-cmep-lv%d", j + 1) < 0) {
						pr_info("%s: snprintf fail at lv%d\n", __func__, j + 1);
						break;
					}
					ret = of_property_read_u32(np, buf, &pt_info_p->soc_limit_cmep_lv[j]);
					if (ret) {
						pr_info("%s: get %s fail (%d)at cmep-lv\n", __func__, buf, ret);
						break;
					}
					pr_info("%s: %s = %u\n", __func__, buf, pt_info_p->soc_limit_cmep_lv[j]);
				}
				ret = of_property_read_u32(np, "mpmm-enable", &pt_info_p->mpmm_enable);
				if (ret) {
					pt_info_p->mpmm_enable = 0;
					pr_info("Failed to read mpmm-enable property\n");
				}
				if (pt_info_p->mpmm_enable == 1) {
					ret = of_property_read_u32(np, "mpmm-gear", &mpmm_gear);
					if (ret) {
						pt_info_p->mpmm_enable = 0;
						pr_info("Failed to read mpmm-gear property\n");
					} else if (mpmm_gear > 2) {
						pt_info_p->mpmm_enable = 0;
						pr_info("mpmm-gear value out of range: %u\n", mpmm_gear);
					}
					ret = of_property_read_u32(np, "soc-limit-mpmm-lv",
						&pt_info_p->mpmm_activate_lv);
					if (ret || pt_info_p->mpmm_activate_lv > pt_info_p->max_lv) {
						pt_info_p->mpmm_activate_lv = pt_info_p->max_lv;
						pr_info("Failed to read soc-limit-mpmm-lv property\n");
					}
				}
			}
		}
	}
	/*cpu bootup pt*/
	np_bootmode = of_parse_phandle(dev->of_node, "bootmode", 0);
	if (!np_bootmode){
		pr_notice("%s: get bootmode fail\n", __func__);
		bootup_pt_support = false;
	} else {
		tag = (struct tag_bootmode *)of_get_property(np_bootmode, "atag,boot", NULL);
		if (!tag) {
			pr_notice("failed to get atag,boot\n");
			bootup_pt_support = false;
		} else {
			bootup_pt_support = parse_bootup_pt_table(np, tag);
		}
		of_node_put(np_bootmode);
	}
	// cpu pt table switch
	ret = of_property_read_u32(np, "lbat-max-tb-num", &lbat_tb_num);
	if (ret || lbat_tb_num < 0 || lbat_tb_num > CPU_PT_TABLE_MAX) {
		pr_notice("%s: get lbat_tb_num %d fail set to default %d\n", __func__, lbat_tb_num, ret);
		lbat_tb_num = 1;
		switch_pt = false;
	} else {
		switch_pt = parse_switchpt_table(np);
	}
	return 0;
}

/*****************************************************************************
 * modify cpu throttle freq
 ******************************************************************************/
static ssize_t modify_cpu_throttle_freq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;
	struct cpu_pt_table *pt_info_p;
	u32 max_tb_num = 0;
	u32 max_lv = 0;
	int i = 0,j = 0;

	if (switch_pt == false) {
		dev_info(dev, "not support switch_pt\n");
		return -EINVAL;
	}
	max_tb_num = cpu_pt_tb[CPU_PT_TABLE0].max_tb_num;
	max_lv = cpu_pt_info[LBAT_POWER_THROTTLING].max_lv;

	if (max_tb_num > CPU_PT_TABLE_MAX)
		max_tb_num = 1;
	if (max_lv > LOW_BATTERY_LEVEL_NUM - 1)
		max_lv = 3;

	for (i = 0; i < max_tb_num; i++ ) {
		pt_info_p = &cpu_pt_tb[i];
		len += snprintf(buf + len, PAGE_SIZE - len, "cpu table%d limit freq:", i);
		for (j = 0; j < max_lv * CLUSTER_NUM; j++)
			len += snprintf(buf + len, PAGE_SIZE - len, " %d", pt_info_p->freq_limit[j]);

		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}
	return len;
}

static ssize_t modify_cpu_throttle_freq_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int i, j, k = 0, len = 0;
	u32 freq_limit[LOW_BATTERY_LEVEL_NUM * CLUSTER_NUM];
	u32 table_idx;
	struct cpu_pt_policy *pt_policy;
	u32 max_tb_num = 0;
	u32 max_lv = 0;

	if (switch_pt == false) {
		dev_info(dev, "not support switch_pt\n");
		return -EINVAL;
	}
	max_tb_num = cpu_pt_tb[CPU_PT_TABLE0].max_tb_num;
	max_lv = cpu_pt_info[LBAT_POWER_THROTTLING].max_lv;

	if (sscanf(buf, "%u%n", &table_idx, &len)!=1) {
		dev_info(dev, "parameter number not correct\n");
		return -EINVAL;
	}
	buf += len;
	if (table_idx > max_tb_num - 1) {
		dev_info(dev, "Invalid table_idx: %u\n", table_idx);
		return -EINVAL;
	}
	for (i = 0; i < max_lv * CLUSTER_NUM; i++) {
		if (sscanf(buf, "%u%n", &freq_limit[i], &len) != 1) {
			dev_info(dev, "Failed to read freq_limit[%d]\n", i);
			return -EINVAL;
		}
		buf += len;

		if (freq_limit[i] <= 0 || freq_limit[i] > CPU_UNLIMIT_FREQ) {
			dev_info(dev, "Invalid freq_limit[%d]: %u\n", i, freq_limit[i]);
			return -EINVAL;
		}
	}

	mutex_lock(&cpu_freq_lock);
	for (i = 0; i < max_lv * CLUSTER_NUM; i++)
		cpu_pt_tb[table_idx].freq_limit[i] = freq_limit[i];
	if (table_idx==cpu_pt_table_idx){
		list_for_each_entry(pt_policy, &pt_policy_list, cpu_pt_list) {
			if (pt_policy->pt_type == LBAT_POWER_THROTTLING) {
				for (j = 0; j < cpu_pt_info[LBAT_POWER_THROTTLING].max_lv; j++) {
					pt_policy->freq_limit[j] = cpu_pt_tb[table_idx].freq_limit[j * CLUSTER_NUM + k];
					dev_notice (dev, "i:%d, k:%d, pt_policy->freq_limit: %d, cpu_pt_info[i].freq_limit %d\n",
					i, k, pt_policy->freq_limit[j],	cpu_pt_tb[table_idx].freq_limit[j * CLUSTER_NUM
					+ k]);
				}
				k++;
			}
		}
	}
	mutex_unlock(&cpu_freq_lock);

	cpu_pt_low_battery_cb(cpu_pt_info[LBAT_POWER_THROTTLING].cur_lv, NULL);

	for (i = 0; i < max_lv * CLUSTER_NUM; i++)
		dev_notice(dev, "freq_limit[%d]: %d\n", i, cpu_pt_tb[table_idx].freq_limit[i]);
	return size;
}
static DEVICE_ATTR_RW(modify_cpu_throttle_freq);
static ssize_t boot_notify_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;

	if (bootup_pt_support == false){
		dev_info(dev, "not support cpu bootup\n");
		return -EINVAL;
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "system_boot_completed: %u\n", system_boot_completed);
	return len;
}

static void release_cpu_performance_work_func(struct work_struct *work)
{
	struct cpu_bootup_pt_policy *bootup_pt_policy;
	s32 freq_limit;

	list_for_each_entry(bootup_pt_policy, &bootup_pt_policy_list, cpu_bootup_pt_list) {
		freq_limit = FREQ_QOS_MAX_DEFAULT_VALUE;
		pr_info("%s: freq_limit=%d\n", __func__, freq_limit);
		freq_qos_update_request(&bootup_pt_policy->qos_req, freq_limit);
	}
}

static ssize_t boot_notify_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned int boot_completed = 0;
	static struct delayed_work work_struct;
	static int only_do_one_time;

	if (only_do_one_time == 0)
		INIT_DELAYED_WORK(&work_struct, release_cpu_performance_work_func);

	if (bootup_pt_support == false){
		dev_info(dev, "not support cpu bootup\n");
		return -EINVAL;
	}
	if (sscanf(buf, "%u\n", &boot_completed) != 1) {
		dev_info(dev, "parameter number not correct\n");
		return -EINVAL;
	}
	if (boot_completed > 1) {
		dev_info(dev, "invalid input %u\n", boot_completed);
		return -EINVAL;
	}
	system_boot_completed = boot_completed;
	if (only_do_one_time == 0) {
		schedule_delayed_work(&work_struct, delay_time_s * HZ);
		only_do_one_time = 1;
	}

	return size;
}

static DEVICE_ATTR_RW(boot_notify);

static ssize_t cpu_pt_table_idx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;

	if (switch_pt == false) {
		dev_info(dev, "not support switch_pt\n");
		return -EINVAL;
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "cpu_pt_table_idx: %d\n", cpu_pt_table_idx);

	return len;
}

static ssize_t cpu_pt_table_idx_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	char cmd[20];
	int k = 0;
	struct cpu_pt_policy *pt_policy;

	if (switch_pt == false) {
		dev_info(dev, "not support switch_pt\n");
		return -EINVAL;
	}
	if ((sscanf(buf, "%9s %u\n", cmd, &cpu_pt_table_idx) != 2) || (strncmp(cmd, "table_idx", 9) != 0)) {
		dev_info(dev, "parameter number not correct\n");
		return -EINVAL;
	}
	if (cpu_pt_table_idx < 0 || cpu_pt_table_idx >= lbat_tb_num) {
		pr_info("Invalid voltage table index.\n");
		return -EINVAL;
	}
	mutex_lock(&cpu_freq_lock);

	list_for_each_entry(pt_policy, &pt_policy_list, cpu_pt_list) {
		if (pt_policy->pt_type == LBAT_POWER_THROTTLING) {
			for (int j = 0; j < cpu_pt_info[LBAT_POWER_THROTTLING].max_lv; j++) {
				pt_policy->freq_limit[j] = cpu_pt_tb[cpu_pt_table_idx].freq_limit[j * CLUSTER_NUM + k];
				dev_notice(dev, "k:%d, pt_policy->freq_limit: %d, cpu_pt_tb[i].freq_limit %d\n", k,
				pt_policy->freq_limit[j], cpu_pt_tb[cpu_pt_table_idx].freq_limit[j * CLUSTER_NUM + k]);
			}
			k++;
		}
	}

	mutex_unlock(&cpu_freq_lock);

	cpu_pt_low_battery_cb(cpu_pt_info[LBAT_POWER_THROTTLING].cur_lv, NULL);

	return size;
}
static DEVICE_ATTR_RW(cpu_pt_table_idx);

static ssize_t set_mpmm_show(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,"%d\n", mpmm_gear);
}

static ssize_t set_mpmm_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t size)
{
	unsigned int val;
	char cmd[9];

	if (sscanf(buf, "%8s %u\n", cmd, &val) != 2) {
		dev_info(dev, "parameter number not correct\n");
		return -EINVAL;
	}
	if (strncmp(cmd, "set_mpmm", 8))
		return -EINVAL;
	if (val < 4)
		mpmm_gear = val;

	mpmm_enable(mpmm_gear);
	pr_info("[%s] smc send", __func__);
	return size;
}
static DEVICE_ATTR_RW(set_mpmm);


int register_pt_isolate_cb(cpu_isolate_cb cb_func)
{
	int ret = 0, idx;
	unsigned int i = 0;
	bool active;
	struct cpu_pt_priv *pt_info_p = &cpu_pt_info[SOC_POWER_THROTTLING];

	if (cb_func) {
		cicb.pause_func = cb_func;

		mutex_lock(&cpu_thr_lock);
		if (pt_info_p->core_active && pt_info_p->cur_lv > 0) {
			for (i = 0; i < CORE_NUM; i++) {
				idx = (pt_info_p->cur_lv - 1) * CORE_NUM + i;
				active = (pt_info_p->core_active[idx] > 0) ? true : false;
				pt_set_cpu_active(i, active);
			}
		}
		mutex_unlock(&cpu_thr_lock);

	} else
		ret = -EINVAL;

	return ret;
}
EXPORT_SYMBOL(register_pt_isolate_cb);

static bool cpu_pt_published;
static const struct attribute_group cpu_pt_group;

static void cpu_pt_release_policies(void)
{
 struct cpu_pt_policy *p, *next;
 struct cpu_bootup_pt_policy *b, *bn;
 list_for_each_entry_safe(p, next, &pt_policy_list, cpu_pt_list) {
  freq_qos_remove_request(&p->qos_req);
  cpufreq_cpu_put(p->policy);
  list_del(&p->cpu_pt_list);
  kfree(p->freq_limit);
  kfree(p);
 }
 list_for_each_entry_safe(b, bn, &bootup_pt_policy_list, cpu_bootup_pt_list) {
  freq_qos_remove_request(&b->qos_req);
  cpufreq_cpu_put(b->policy);
  list_del(&b->cpu_bootup_pt_list);
  kfree(b->freq_limit_booting);
  kfree(b);
 }
}

static int cpu_pt_add_policy(struct cpufreq_policy *policy, unsigned int cluster,
                             unsigned int type, bool boot)
{
 struct cpu_pt_policy *p = NULL;
 struct cpu_bootup_pt_policy *b = NULL;
 struct cpufreq_policy *owned;
 unsigned int i, count;
 int ret;
 if (cluster >= CLUSTER_NUM || type >= POWER_THROTTLING_TYPE_MAX)
  return -EINVAL;
 owned = cpufreq_cpu_get(policy->cpu);
 if (!owned)
  return -EPROBE_DEFER;
 if (boot) {
  b = kzalloc(sizeof(*b), GFP_KERNEL);
  if (!b) { ret = -ENOMEM; goto put; }
  count = cpu_bootup_pt_info.max_lv;
  if (!count || !cpu_bootup_pt_info.freq_limit_booting) { ret = -EINVAL; goto free; }
  b->freq_limit_booting = kcalloc(count, sizeof(u32), GFP_KERNEL);
  if (!b->freq_limit_booting) { ret = -ENOMEM; goto free; }
  b->freq_limit_booting[0] = cpu_bootup_pt_info.freq_limit_booting[cluster];
  b->policy = owned; b->cpu = policy->cpu; b->pt_max_lv = count;
  ret = freq_qos_add_request(&owned->constraints, &b->qos_req,
                            FREQ_QOS_MAX, FREQ_QOS_MAX_DEFAULT_VALUE);
  if (ret < 0) goto free;
  list_add_tail(&b->cpu_bootup_pt_list, &bootup_pt_policy_list);
 } else {
  p = kzalloc(sizeof(*p), GFP_KERNEL);
  if (!p) { ret = -ENOMEM; goto put; }
  count = max(cpu_pt_info[type].max_lv, 1U);
  p->freq_limit = kcalloc(count, sizeof(u32), GFP_KERNEL);
  if (!p->freq_limit) { ret = -ENOMEM; goto free; }
  if (cpu_pt_info[type].max_lv && !cpu_pt_info[type].freq_limit) { ret = -EINVAL; goto free; }
  for (i = 0; i < cpu_pt_info[type].max_lv; i++)
   p->freq_limit[i] = cpu_pt_info[type].freq_limit[i * CLUSTER_NUM + cluster];
  p->policy = owned; p->cpu = policy->cpu; p->pt_type = type;
  p->pt_max_lv = cpu_pt_info[type].max_lv;
  ret = freq_qos_add_request(&owned->constraints, &p->qos_req,
                            FREQ_QOS_MAX, FREQ_QOS_MAX_DEFAULT_VALUE);
  if (ret < 0) goto free;
  list_add_tail(&p->cpu_pt_list, &pt_policy_list);
 }
 return 0;
free:
 if (p) { kfree(p->freq_limit); kfree(p); }
 if (b) { kfree(b->freq_limit_booting); kfree(b); }
put:
 cpufreq_cpu_put(owned);
 return ret;
}

static void cpu_pt_release_tables(void)
{
 unsigned int i;
 for (i = 0; i < POWER_THROTTLING_TYPE_MAX; i++) {
  kfree(cpu_pt_info[i].freq_limit); cpu_pt_info[i].freq_limit = NULL;
  kfree(cpu_pt_info[i].core_active); cpu_pt_info[i].core_active = NULL;
  kfree(cpu_pt_info[i].soc_limit_cmep_lv); cpu_pt_info[i].soc_limit_cmep_lv = NULL;
 }
 for (i = 0; i < CPU_PT_TABLE_MAX; i++) {
  kfree(cpu_pt_tb[i].freq_limit); cpu_pt_tb[i].freq_limit = NULL;
 }
 kfree(cpu_bootup_pt_info.freq_limit_booting);
 cpu_bootup_pt_info.freq_limit_booting = NULL;
 cmep_exit();
}

static struct attribute *cpu_pt_attrs[] = {
 &dev_attr_modify_cpu_throttle_freq.attr,
 &dev_attr_cpu_pt_table_idx.attr,
 &dev_attr_boot_notify.attr,
 &dev_attr_set_mpmm.attr,
 NULL,
};
static const struct attribute_group cpu_pt_group = { .attrs = cpu_pt_attrs };

static int mtk_cpu_power_throttling_probe(struct platform_device *pdev)
{
 struct cpufreq_policy *policy;
 struct cpu_pt_priv saved[POWER_THROTTLING_TYPE_MAX];
 unsigned int cluster = 0, type;
 int cpu, ret;
 if (cpu_pt_published)
  return -EBUSY;
 memcpy(saved, cpu_pt_info, sizeof(saved));
 switch_pt = false;
 bootup_pt_support = false;
 /* Optional SoC SRAM, as in the original driver. Missing hardware is allowed. */
 cmep_init();
 ret = parse_cpu_limit_table(&pdev->dev);
 if (ret) goto fail;
 for_each_possible_cpu(cpu) {
  policy = cpufreq_cpu_get(cpu);
  if (!policy) { ret = -EPROBE_DEFER; goto fail; }
  if (policy->cpu != cpu) { cpufreq_cpu_put(policy); continue; }
  if (cluster >= CLUSTER_NUM) { cpufreq_cpu_put(policy); ret = -E2BIG; goto fail; }
  for (type = 0; type < POWER_THROTTLING_TYPE_MAX; type++) {
   ret = cpu_pt_add_policy(policy, cluster, type, false);
   if (ret) { cpufreq_cpu_put(policy); goto fail; }
  }
  if (bootup_pt_support) {
   ret = cpu_pt_add_policy(policy, cluster, 0, true);
   if (ret) { cpufreq_cpu_put(policy); goto fail; }
  }
  cpufreq_cpu_put(policy);
  cluster++;
 }
 if (cluster != CLUSTER_NUM) { ret = -EPROBE_DEFER; goto fail; }
 ret = devm_device_add_group(&pdev->dev, &cpu_pt_group);
 if (ret) goto fail;
 cpu_pt_table_idx = 0;
 system_boot_completed = 0;
 /* Resident supplier registries have no drained unregister API. Once published,
  * both code and callback-owned state must live until reboot. No later failures. */
 __module_get(THIS_MODULE);
 cpu_pt_published = true;
 if (bootup_pt_support) cpu_bootup_pt_trigger();
 if (cpu_pt_info[LBAT_POWER_THROTTLING].max_lv > 0)
  register_low_battery_notify(cpu_pt_low_battery_cb, LOW_BATTERY_PRIO_CPU_B, NULL);
 if (cpu_pt_info[OC_POWER_THROTTLING].max_lv > 0)
  register_battery_oc_notify(cpu_pt_over_current_cb, BATTERY_OC_PRIO_CPU_B, NULL);
 if (cpu_pt_info[SOC_POWER_THROTTLING].max_lv > 0) {
  for (type = 0; type < CORE_NUM; type++) cur_core_active[type] = 1;
  register_bp_thl_notify(cpu_pt_battery_percent_cb, BATTERY_PERCENT_PRIO_CPU_B);
 }
 dev_info(&pdev->dev, "Native power policy ready: %u clusters; callbacks retained until reboot\n", cluster);
 return 0;
fail:
 cpu_pt_release_policies();
 cpu_pt_release_tables();
 memcpy(cpu_pt_info, saved, sizeof(saved));
 return ret;
}

static const struct of_device_id cpu_power_throttling_of_match[] = {
	{ .compatible = "mediatek,cpu-power-throttling", },
	{},
};
MODULE_DEVICE_TABLE(of, cpu_power_throttling_of_match);
static struct platform_driver cpu_power_throttling_driver = {
	.probe = mtk_cpu_power_throttling_probe,
	.driver = {
		.name = "mtk-cpu_power_throttling",
		.suppress_bind_attrs = true,
		.of_match_table = cpu_power_throttling_of_match,
	},
};
module_platform_driver(cpu_power_throttling_driver);
MODULE_AUTHOR("Samuel Hsieh");
MODULE_DESCRIPTION("MTK cpu power throttling driver");
MODULE_LICENSE("GPL");
