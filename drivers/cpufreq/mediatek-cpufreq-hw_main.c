// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/bitfield.h>
#include <linux/cpufreq.h>
#include <linux/energy_model.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/kdebug.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/panic_notifier.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/sched/clock.h>
#include <linux/topology.h>

#include "mediatek-cpufreq-hw_fdvfs.h"

#define LUT_MAX_ENTRIES			32U
#define LUT_FREQ			GENMASK(11, 0)
#define LUT_FREQ_V2			GENMASK(13, 0)
#define LUT_ROW_SIZE			0x4
#define CPUFREQ_HW_STATUS		BIT(0)
#define SVS_HW_STATUS			BIT(1)
#define POLL_USEC			1000
#define TIMEOUT_USEC			300000
#define REG_FREQ_SCALING		0x4cc/* from csram_base */
#define REG_STOP_CPUDVFS_LOG		0x128 /* from csram_base */
#define REG_QOS_OFF			0x20
#define CHECK_VAL			0X55AA55AA
#define MAX_CONTROL_GROUPS		8
#define MAX_PERF_DOMAINS		3
#define PER_CORE_OFF			0x1380
#define PER_CORE_SIZE			0x20
#define LKG_TBL_SIZE			0xC00
#define LKG_PARA_VER_OFFSET		(LKG_TBL_SIZE - 0x4)

enum {
	REG_FREQ_LUT_TABLE,
	REG_FREQ_ENABLE,
	REG_FREQ_PERF_STATE,
	REG_FREQ_HW_STATE,
	REG_EM_POWER_TBL,
	REG_FREQ_LATENCY,

	REG_ARRAY_SIZE,
};

struct cpufreq_mtk {
	struct cpufreq_frequency_table *table;
	void __iomem *reg_bases[REG_ARRAY_SIZE];
	int nr_opp;
	unsigned int last_index;
	cpumask_t related_cpus;
	long sb_ch;
};

static const u16 cpufreq_mtk_offsets[REG_ARRAY_SIZE] = {
	[REG_FREQ_LUT_TABLE]	= 0x0,
	[REG_FREQ_ENABLE]	= 0x84,
	[REG_FREQ_PERF_STATE]	= 0x88,
	[REG_FREQ_HW_STATE]	= 0x8C,
	[REG_EM_POWER_TBL]	= 0x90,
	[REG_FREQ_LATENCY]	= 0x114,
};

static struct cpufreq_mtk *mtk_freq_domain_map[NR_CPUS];
static bool freq_scaling_disabled = true;
static bool fdvfs_enabled;
static bool per_core_enabled;
static void __iomem *qos_base;
static void __iomem *per_core_base;
static int control_group_num;
static int cpu_control_group_map[MAX_CONTROL_GROUPS];
static int control_group_master[MAX_CONTROL_GROUPS];
static int perf_domain_master[MAX_PERF_DOMAINS];
static unsigned int init_brake_policy0;
static unsigned int init_brake_policy4;
static unsigned int init_brake_policy7;
static unsigned int pwr_tbl_ver;
static DEFINE_MUTEX(cpufreq_owner_lock);
static struct platform_device *cpufreq_owner;
static bool legacy_tables_active = true;
static bool exception_hooks_registered;
static atomic_t first_exception = ATOMIC_INIT(0);

static bool mtk_cpufreq_use_legacy_tables(struct device *dev)
{
	if (!of_property_read_bool(dev->of_node, "mediatek,legacy-dvfs-table"))
		return false;

	/* Original MCU firmware takes OPP indices and 12-bit MHz table entries.
	 * Do not interpret newer CSRAM/USRAM metadata on this interface.
	 */
	freq_scaling_disabled = true;
	fdvfs_enabled = false;
	per_core_enabled = false;
	pwr_tbl_ver = 0;
	qos_base = NULL;
	return true;
}

static int look_up_cpu(struct device *cpu_dev)
{
	unsigned int cpu = 0;

	for_each_possible_cpu(cpu) {
		if (cpu_dev == get_cpu_device(cpu))
			return cpu;
	}

	return 0; /* fallback to cpu0 */
}


static int __maybe_unused
mtk_cpufreq_get_cpu_power(struct device *cpu_dev, unsigned long *uW,
		unsigned long *KHz)
{
	int cpu = look_up_cpu(cpu_dev);
	struct cpufreq_mtk *c = mtk_freq_domain_map[cpu];
	int i;

	if (!c || !c->table || c->nr_opp <= 0 || !uW || !KHz)
		return -EINVAL;

	for (i = 0; i < c->nr_opp; i++) {
		if (c->table[i].frequency < *KHz)
			break;
	}
	if (i)
		i--;

	*KHz = c->table[i].frequency;
	*uW = readl_relaxed(c->reg_bases[REG_EM_POWER_TBL] +
			    i * LUT_ROW_SIZE);

	return 0;
}

int stop_dvfs_log(struct notifier_block *self, unsigned long cmd, void *ptr)
{
	struct cpufreq_mtk *c;
	void __iomem *csram_base_add_0x10;
	void __iomem *stop_dvfs_log;

	/* The original MCU ABI has no documented exception-log register. */
	rcu_read_lock();
	c = smp_load_acquire(&mtk_freq_domain_map[0]);
	if (!READ_ONCE(legacy_tables_active) && c &&
	    atomic_cmpxchg(&first_exception, 0, 1) == 0) {
		csram_base_add_0x10 = c->reg_bases[REG_FREQ_LUT_TABLE];
		stop_dvfs_log = csram_base_add_0x10 + REG_STOP_CPUDVFS_LOG - 0x10;
		writel_relaxed(0x1, stop_dvfs_log);
		pr_info("cpufreq stop DVFS log done\n");
	}
	rcu_read_unlock();
	return NOTIFY_DONE;
}
EXPORT_SYMBOL_GPL(stop_dvfs_log);

static struct notifier_block die_blk = {
	.notifier_call = stop_dvfs_log,
	.priority = INT_MAX,
};

static struct notifier_block panic_blk = {
	.notifier_call = stop_dvfs_log,
	.priority = INT_MAX,
};
static int mtk_cpufreq_hw_target_index(struct cpufreq_policy *policy,
				       unsigned int index)
{
	struct cpufreq_mtk *c = policy->driver_data;

	if (!freq_scaling_disabled) {
		int target_freq = policy->freq_table[index].frequency;

		if (fdvfs_enabled)
			cpufreq_fdvfs_switch(target_freq, policy);
		else
			writel_relaxed(target_freq, c->reg_bases[REG_FREQ_PERF_STATE]);
	} else
		writel_relaxed(index, c->reg_bases[REG_FREQ_PERF_STATE]);

	return 0;
}

static unsigned int mtk_cpufreq_hw_get(unsigned int cpu)
{
	struct cpufreq_mtk *c;
	unsigned int index, nr_opp;

	if (cpu >= NR_CPUS)
		return UINT_MAX;
	c = mtk_freq_domain_map[cpu];
	if (!c || !c->table || c->nr_opp <= 0)
		return UINT_MAX;
	nr_opp = c->nr_opp;

	if (per_core_enabled)
		index = readl_relaxed(per_core_base + 0x4 * cpu);
	else
		index = readl_relaxed(c->reg_bases[REG_FREQ_PERF_STATE]);

	if (index <= LUT_MAX_ENTRIES - 1) {
		if (index >= nr_opp)
			return UINT_MAX;
		return c->table[index].frequency;
	} else if (c->table[0].frequency >= index && index >= c->table[nr_opp - 1].frequency) {
		return index;
	}

	return UINT_MAX;
}

static unsigned int mtk_cpufreq_hw_fast_switch(struct cpufreq_policy *policy,
					       unsigned int target_freq)
{
	struct cpufreq_mtk *c = policy->driver_data;
	unsigned int index;
#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	u64 ts[2];

	ts[0] = sched_clock();
#endif

	if (policy->cached_target_freq == target_freq)
		index = policy->cached_resolved_idx;
	else
		index = cpufreq_table_find_index_dl(policy, target_freq, false);

	if (fdvfs_enabled) {
		if(c->sb_ch && c->sb_ch != -1)
			cpufreq_fdvfs_cci_switch(c->sb_ch);
		cpufreq_fdvfs_switch(target_freq, policy);
	}
	else {
		if(qos_base && c->sb_ch) {
			c->sb_ch == -1 ?
			writel_relaxed(0, qos_base + REG_QOS_OFF):
			writel_relaxed(c->sb_ch, qos_base + REG_QOS_OFF);
		}
		if (!freq_scaling_disabled) // Frequency scaling enabled
			writel_relaxed(target_freq, c->reg_bases[REG_FREQ_PERF_STATE]);
		else
			writel_relaxed(index, c->reg_bases[REG_FREQ_PERF_STATE]);
	}

	if (c->last_index == index)
		return 0;
	c->last_index = index;

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[1] = sched_clock();

	if ((ts[1] - ts[0] > 100000ULL) && in_hardirq()) {
		printk_deferred("%s duration %llu, ts[0]=%llu, ts[1]=%llu\n",
			__func__, ts[1] - ts[0], ts[0], ts[1]);
	}
#endif

	return policy->freq_table[index].frequency;
}

static int mtk_cpufreq_hw_cpu_init(struct cpufreq_policy *policy)
{
	struct cpufreq_mtk *c;
	struct pm_qos_request *qos_request;
	int sig, pwr_hw = CPUFREQ_HW_STATUS | SVS_HW_STATUS;
	unsigned int latency;

	qos_request = kzalloc(sizeof(*qos_request), GFP_KERNEL);
	if (!qos_request)
		return -ENOMEM;

	c = mtk_freq_domain_map[policy->cpu];
	if (!c) {
		pr_info("No scaling support for CPU%d\n", policy->cpu);
		kfree(qos_request);
		return -ENODEV;
	}

	cpumask_copy(policy->cpus, &c->related_cpus);

	policy->freq_table = c->table;
	policy->driver_data = c;

	latency = readl_relaxed(c->reg_bases[REG_FREQ_LATENCY]);
	if (!latency)
		latency = CPUFREQ_ETERNAL;

	/* us convert to ns */
	latency = latency & 0xFFFFU;
	policy->cpuinfo.transition_latency = latency * 1000;

	policy->fast_switch_possible = true;

	/* Let CPUs leave idle-off state for SVS CPU initializing */
	cpu_latency_qos_add_request(qos_request, PM_QOS_DEFAULT_VALUE);

	/* HW should be in enabled state to proceed now */
	writel_relaxed(0x1, c->reg_bases[REG_FREQ_ENABLE]);

	if (readl_poll_timeout(c->reg_bases[REG_FREQ_HW_STATE], sig,
			       (sig & pwr_hw) == pwr_hw, POLL_USEC,
			       TIMEOUT_USEC)) {
		if (!(sig & CPUFREQ_HW_STATUS)) {
			pr_info("cpufreq hardware of CPU%d is not enabled\n",
				policy->cpu);
			cpu_latency_qos_remove_request(qos_request);
			kfree(qos_request);
			return -ENODEV;
		}

		pr_info("SVS of CPU%d is not enabled\n", policy->cpu);
	}

	cpu_latency_qos_remove_request(qos_request);
	kfree(qos_request);

	return 0;
}

static void mtk_cpufreq_hw_cpu_exit(struct cpufreq_policy *policy)
{
	struct cpufreq_mtk *c;

	c = mtk_freq_domain_map[policy->cpu];
	if (!c) {
		pr_info("No scaling support for CPU%d\n", policy->cpu);
		return;
	}

	/* HW should be in paused state now */
	writel_relaxed(0x0, c->reg_bases[REG_FREQ_ENABLE]);
}

static void mtk_cpufreq_register_em(struct cpufreq_policy *policy)
{
	struct em_data_callback em_cb = EM_DATA_CB(mtk_cpufreq_get_cpu_power);
	struct cpufreq_mtk *c = policy->driver_data;

	if (!c->nr_opp) {
		pr_info("%s: %d: CPU%d: Get opp failed\n", __func__, __LINE__,
				policy->cpu);
		return;
	}

	em_dev_register_perf_domain(get_cpu_device(policy->cpu), c->nr_opp, &em_cb, policy->cpus,
			true);
}

static void mtk_cpufreq_ready(struct cpufreq_policy *policy)
{
	if (policy->cpu == 0 && init_brake_policy0 > 0) {
		policy->max = init_brake_policy0;
		pr_notice("%s: cpu=0, policy->max set to %u\n", __func__, policy->max);
	} else if (policy->cpu == 4 && init_brake_policy4 > 0) {
		policy->max = init_brake_policy4;
		pr_notice("%s: cpu=4, policy->max set to %u\n", __func__, policy->max);
	} else if (policy->cpu == 7 && init_brake_policy7 > 0) {
		policy->max = init_brake_policy7;
		pr_notice("%s: cpu=7, policy->max set to %u\n", __func__, policy->max);
	}
}

static struct cpufreq_driver cpufreq_mtk_hw_driver = {
	.flags		= CPUFREQ_NEED_INITIAL_FREQ_CHECK |
			  CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
			  CPUFREQ_IS_COOLING_DEV,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= mtk_cpufreq_hw_target_index,
	.get		= mtk_cpufreq_hw_get,
	.init		= mtk_cpufreq_hw_cpu_init,
	.exit		= mtk_cpufreq_hw_cpu_exit,
	.register_em	= mtk_cpufreq_register_em,
	.fast_switch	= mtk_cpufreq_hw_fast_switch,
	.ready		= mtk_cpufreq_ready,
	.name		= "mtk-cpufreq-hw",
	.attr		= cpufreq_generic_attr,
};

static int mtk_cpu_create_freq_table(struct platform_device *pdev,
				     struct cpufreq_mtk *c)
{
	struct device *dev = &pdev->dev;
	void __iomem *base_table;
	u32 data, i, freq, prev_freq = 0;

	c->table = devm_kcalloc(dev, LUT_MAX_ENTRIES + 1,
				sizeof(*c->table), GFP_KERNEL);
	if (!c->table)
		return -ENOMEM;

	base_table = c->reg_bases[REG_FREQ_LUT_TABLE];

	for (i = 0; i < LUT_MAX_ENTRIES; i++) {
		data = readl_relaxed(base_table + (i * LUT_ROW_SIZE));
		if (pwr_tbl_ver == 2)
			freq = FIELD_GET(LUT_FREQ_V2, data) * 1000;
		else
			freq = FIELD_GET(LUT_FREQ, data) * 1000;

		if (freq == prev_freq)
			break;
		if (!freq || (i && freq > prev_freq))
			return -EINVAL;

		c->table[i].frequency = freq;

		dev_dbg(dev, "index=%d freq=%d\n",
			i, c->table[i].frequency);

		prev_freq = freq;
	}

	c->table[i].frequency = CPUFREQ_TABLE_END;
	c->nr_opp = i;

	return i ? 0 : -EINVAL;
}

static int mtk_get_related_cpus(int master_index, struct cpufreq_mtk *c,
				struct device_node *provider)
{
	struct device_node *cpu_np;
	struct of_phandle_args args;
	int control_index;
	int cpu, ret;

	for_each_possible_cpu(cpu) {
		if(!per_core_enabled) {
			cpu_np = of_cpu_device_node_get(cpu);
			if (!cpu_np)
				continue;

			ret = of_parse_phandle_with_args(cpu_np, "performance-domains",
							"#performance-domain-cells", 0,
							&args);
			of_node_put(cpu_np);
			if (ret < 0)
				continue;

			if (args.args_count != 1 || args.np != provider) {
				of_node_put(args.np);
				continue;
			}
			control_index = args.args[0];
			of_node_put(args.np);
		} else
			control_index = cpu_control_group_map[cpu];

		if (control_index == master_index) {
			cpumask_set_cpu(cpu, &c->related_cpus);
			smp_store_release(&mtk_freq_domain_map[cpu], c);
		}
	}

	return 0;
}

static int mtk_cpu_resources_init(struct platform_device *pdev,
				  unsigned int cpu, int index,
				  const u16 *offsets)
{
	struct cpufreq_mtk *c;
	struct device *dev = &pdev->dev;
	int control_index;
	int ret, i;
	void __iomem *base;

	if (mtk_freq_domain_map[cpu])
		return 0;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, index);
	if (IS_ERR(base))
		return PTR_ERR(base);

	for (i = REG_FREQ_LUT_TABLE; i < REG_ARRAY_SIZE; i++)
		c->reg_bases[i] = base + offsets[i];

	ret = mtk_cpu_create_freq_table(pdev, c);
	if (ret) {
		dev_info(dev, "Domain-%d failed to create freq table\n", index);
		return ret;
	}

	/* Do not publish a domain until its table and register bases are valid. */
	control_index = !per_core_enabled ? index : cpu_control_group_map[cpu];
	ret = mtk_get_related_cpus(control_index, c, dev->of_node);
	if (ret)
		return ret;

	return 0;
}

static int mtk_per_core_cpu_resources_init(struct platform_device *pdev,
					unsigned int cpu, int index,
					const u16 *offsets)
{
	struct cpufreq_mtk *c;
	struct cpufreq_mtk *master_c;
	struct device *dev = &pdev->dev;
	int ret, i, master;
	int control_group = cpu_control_group_map[cpu];

	if (mtk_freq_domain_map[cpu]) {
		dev_info(dev, "already has per-core entry: index-%d cpu-%d\n", index, cpu);
		return 0;
	}

	if (cpu == perf_domain_master[index])
		return mtk_cpu_resources_init(pdev, cpu, index, offsets);
	else if (cpu == control_group_master[control_group]) {
		master = perf_domain_master[index];
		if (master < 0 || master >= num_possible_cpus()) {
			dev_info(dev, "master %d invalid for control group %d, index %d\n",
					master, control_group, index);
			return -EINVAL;
		}
		master_c = mtk_freq_domain_map[master];
		if (!master_c || !master_c->table)
			return -EPROBE_DEFER;

		c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
		if (!c)
			return -ENOMEM;

		/* init all other member variables from master_c */
		c->table = master_c->table;
		c->nr_opp = master_c->nr_opp;

		for (i = REG_FREQ_LUT_TABLE; i < REG_ARRAY_SIZE; i++)
			c->reg_bases[i] = master_c->reg_bases[i];

		ret = mtk_get_related_cpus(control_group, c, dev->of_node);
		if (ret) {
			dev_info(dev, "Domain-%d failed to get related CPUs\n", control_group);
			return ret;
		}
	} else {
		/* shall not go in here */
		dev_info(dev, "wrong per-core entry, index-%d cpu-%d\n", index, cpu);
		return -EINVAL;
	}

	return 0;
}

static bool check_per_core_enabled(void)
{
	struct device_node *cpu_np;
	u32 control_group;
	int cpu_cluster_id;
	int i, cpu, ret;

	if (!fdvfs_enabled) {
		pr_notice("%s: per-core requires fdvfs support\n", __func__);
		return false;
	}

	control_group_num = 0;
	for (i = 0; i < MAX_CONTROL_GROUPS; i++)
		control_group_master[i] = -1;

	for (i = 0; i < MAX_PERF_DOMAINS; i++)
		perf_domain_master[i] = -1;

	for_each_possible_cpu(cpu) {
		cpu_np = of_cpu_device_node_get(cpu);
		if (!cpu_np) {
			pr_notice("%s: failed to get cpu device\n", __func__);
			return false;
		}

		ret = of_property_read_u32(cpu_np, "control-group", &control_group);
		of_node_put(cpu_np);
		if (ret < 0) {
			pr_notice("%s: failed to get control-group\n", __func__);
			return false;
		}

		if (control_group >= MAX_CONTROL_GROUPS) {
			pr_notice("%s: control-group incorrect\n", __func__);
			return false;
		}

		if (cpu >= MAX_CONTROL_GROUPS)
			return false;
		cpu_control_group_map[cpu] = control_group;

		if (control_group_master[control_group] == -1)
			control_group_master[control_group] = cpu;

		if (control_group + 1 > control_group_num)
			control_group_num = control_group + 1;

		cpu_cluster_id = topology_cluster_id(cpu);
		if (cpu_cluster_id < 0 || cpu_cluster_id >= MAX_PERF_DOMAINS)
			return false;
		if (perf_domain_master[cpu_cluster_id] == -1)
			perf_domain_master[cpu_cluster_id] = cpu;
	}

	pr_notice("%s: per-core enabled\n", __func__);
	return true;
}

static void mtk_cpufreq_put_node(void *node)
{
	of_node_put(node);
}

static void mtk_cpufreq_put_device(void *dev)
{
	put_device(dev);
}

static int mtk_cpufreq_init_domains(struct platform_device *pdev,
				    const u16 *offsets)
{
	struct device_node *cpu_np;
	struct of_phandle_args args;
	unsigned int cpu;
	int ret;

	for_each_possible_cpu(cpu) {
		cpu_np = of_cpu_device_node_get(cpu);
		if (!cpu_np) {
			dev_info(&pdev->dev, "Failed to get cpu %d device\n",
				cpu);
			return -ENODEV;
		}

		ret = of_parse_phandle_with_args(cpu_np, "performance-domains",
						 "#performance-domain-cells", 0,
						 &args);
		of_node_put(cpu_np);
		if (ret < 0)
			return ret;
		if (args.args_count != 1 || args.np != pdev->dev.of_node) {
			of_node_put(args.np);
			return -EINVAL;
		}
		of_node_put(args.np);
		if (args.args[0] >= MAX_PERF_DOMAINS)
			return -EINVAL;

		if (per_core_enabled) {
			ret = mtk_per_core_cpu_resources_init(pdev, cpu, args.args[0], offsets);
			if (ret) {
				dev_info(&pdev->dev, "per-core CPUFreq resource init failed\n");
				return ret;
			}
		} else {
			/* Get the bases of cpufreq for domains */
			ret = mtk_cpu_resources_init(pdev, cpu, args.args[0], offsets);
			if (ret) {
				dev_info(&pdev->dev, "CPUFreq resource init failed\n");
				return ret;
			}
		}
	}

	return 0;
}

static int mtk_cpufreq_register(struct platform_device *pdev)
{
	int ret;

	ret = cpufreq_register_driver(&cpufreq_mtk_hw_driver);
	if (ret) {
		dev_info(&pdev->dev, "CPUFreq HW driver failed to register\n");
		return ret;
	}
	if (legacy_tables_active)
		return 0;
	ret = register_die_notifier(&die_blk);
	if (ret)
		goto unregister_driver;
	ret = atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	if (ret) {
		unregister_die_notifier(&die_blk);
		goto unregister_driver;
	}
	exception_hooks_registered = true;
	return 0;

unregister_driver:
	cpufreq_unregister_driver(&cpufreq_mtk_hw_driver);
	return ret;
}

static int mtk_cpufreq_hw_probe_locked(struct platform_device *pdev)
{
	struct device_node *hvfs_node, *csram_sys_node, *qos_node;
	struct resource *csram_res;
	struct resource *usram_res;
	struct resource *qos_res;
	struct platform_device *pdev_c;
	struct platform_device *pdev_qos;
	void __iomem *csram_base;
	void __iomem *usram_base;
	const u16 *offsets;
	int ret;

	offsets = of_device_get_match_data(&pdev->dev);
	if (!offsets)
		return -EINVAL;

	WRITE_ONCE(legacy_tables_active, mtk_cpufreq_use_legacy_tables(&pdev->dev));
	if (legacy_tables_active)
		goto init_domains;

	hvfs_node = of_find_node_by_name(NULL, "cpuhvfs");
	if (hvfs_node == NULL) {
		pr_notice("failed to find node @ %s\n", __func__);
		return -ENODEV;
	}

	ret = devm_add_action_or_reset(&pdev->dev, mtk_cpufreq_put_node, hvfs_node);
	if (ret)
		return ret;
	pdev_c = of_find_device_by_node(hvfs_node);
	if (pdev_c == NULL) {
		pr_notice("failed to find pdev @ %s\n", __func__);
		return -EINVAL;
	}

	ret = devm_add_action_or_reset(&pdev->dev, mtk_cpufreq_put_device, &pdev_c->dev);
	if (ret)
		return ret;
	csram_res = platform_get_resource(pdev_c, IORESOURCE_MEM, 1);
	if (!csram_res) {
		pr_notice("failed to get mem resource @ %s\n", __func__);
		return -ENODEV;
	}

	if (!request_mem_region(csram_res->start, resource_size(csram_res),
			csram_res->name)) {
		pr_notice("failed to request resource %pR\n", csram_res);
		return -EBUSY;
	}

	csram_base = ioremap(csram_res->start, resource_size(csram_res));
	if (!csram_base) {
		pr_notice("failed to map csram_base @ %s\n", __func__);
		ret = -ENOMEM;
		goto release_region;
	}

	csram_sys_node = of_parse_phandle(hvfs_node, "csram-sys-base", 0);
	if (csram_sys_node != NULL) {
		void __iomem *csram_sys_base;
		bool enabled;

		csram_sys_base = of_iomap(csram_sys_node, 0);
		of_node_put(csram_sys_node);
		if (!csram_sys_base) {
			ret = -ENOMEM;
			iounmap(csram_base);
			goto release_region;
		}
		enabled = readl_relaxed(csram_sys_base) == CHECK_VAL &&
			readl_relaxed(csram_sys_base + 0x10 + cpufreq_mtk_offsets[3]);
		iounmap(csram_sys_base);
		if (!enabled) {
			pr_notice("%s: cpufreq hardware not enable\n", __func__);
			ret = -ENOMEM;
			iounmap(csram_base);
			goto release_region;
		}
		pr_notice("%s: cpufreq hardware enable\n", __func__);
	}

	if (readl_relaxed(csram_base + REG_FREQ_SCALING))
		freq_scaling_disabled = false;

	iounmap(csram_base);
	release_mem_region(csram_res->start, resource_size(csram_res));

	fdvfs_enabled = check_fdvfs_support() == 1 ? true : false;
	per_core_enabled = check_per_core_enabled();

	if (per_core_enabled) {
		per_core_base = ioremap(csram_res->start + PER_CORE_OFF, PER_CORE_SIZE);
		if (!per_core_base) {
			pr_notice("failed to map per_core_base @ %s\n", __func__);
			return -ENOMEM;
		}
	}

	usram_res = platform_get_resource(pdev_c, IORESOURCE_MEM, 0);
	if (!usram_res) {
		pr_notice("failed to get mem resource @ %s\n", __func__);
		return -ENODEV;
	}
	usram_base = ioremap(usram_res->start, resource_size(usram_res));
	if (!usram_base) {
		pr_notice("failed to map usram_base @ %s\n", __func__);
		return -ENOMEM;
	}
	pwr_tbl_ver = readl_relaxed(usram_base + LKG_PARA_VER_OFFSET);
	pr_notice("%s: pwr_tbl_ver: %d\n", __func__, pwr_tbl_ver);
	iounmap(usram_base);

	qos_node = of_find_node_by_name(NULL, "cpuqos-v3");
	if (!qos_node) {
		pr_info("without cpuqos-v3 @ %s\n", __func__);
		qos_base = NULL;
	} else {
		pdev_qos = of_find_device_by_node(qos_node);
		of_node_put(qos_node);
		if (!pdev_qos) {
			pr_info("without cpuqos_v3 pdev @ %s\n", __func__);
			qos_base = NULL;
		} else {
			ret = devm_add_action_or_reset(&pdev->dev,
					mtk_cpufreq_put_device, &pdev_qos->dev);
			if (ret)
				return ret;
			qos_res = platform_get_resource(pdev_qos,
							IORESOURCE_MEM, 0);
			if (qos_res) {
				qos_base = ioremap(qos_res->start,
						resource_size(qos_res));
			} else {
				pr_info("%s no qos resource\n", __func__);
				qos_base = NULL;
			}
		}
	}

init_domains:
	ret = mtk_cpufreq_init_domains(pdev, offsets);
	if (ret)
		return ret;
	return mtk_cpufreq_register(pdev);

release_region:
	release_mem_region(csram_res->start, resource_size(csram_res));
	return ret;
}

/* Called before devres releases domain tables and register mappings. Exported
 * exception callers may outlive notifier removal, so drain their RCU readers.
 */
static void mtk_cpufreq_retire_domains(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu)
		WRITE_ONCE(mtk_freq_domain_map[cpu], NULL);
	synchronize_rcu();
	if (per_core_base)
		iounmap(per_core_base);
	if (qos_base)
		iounmap(qos_base);
	per_core_base = NULL;
	qos_base = NULL;
	freq_scaling_disabled = true;
	fdvfs_enabled = false;
	per_core_enabled = false;
	WRITE_ONCE(legacy_tables_active, true);
	pwr_tbl_ver = 0;
}

static int mtk_cpufreq_hw_driver_probe(struct platform_device *pdev)
{
	int ret;

	mutex_lock(&cpufreq_owner_lock);
	if (cpufreq_owner) {
		ret = -EBUSY;
		goto out;
	}
	atomic_set(&first_exception, 0);
	ret = mtk_cpufreq_hw_probe_locked(pdev);
	if (ret)
		mtk_cpufreq_retire_domains();
	else
		cpufreq_owner = pdev;
out:
	mutex_unlock(&cpufreq_owner_lock);
	return ret;
}

static void mtk_cpufreq_hw_driver_remove(struct platform_device *pdev)
{
	mutex_lock(&cpufreq_owner_lock);
	if (cpufreq_owner == pdev) {
		if (exception_hooks_registered) {
			atomic_notifier_chain_unregister(&panic_notifier_list, &panic_blk);
			unregister_die_notifier(&die_blk);
			exception_hooks_registered = false;
		}
		cpufreq_unregister_driver(&cpufreq_mtk_hw_driver);
		mtk_cpufreq_retire_domains();
		cpufreq_owner = NULL;
	}
	mutex_unlock(&cpufreq_owner_lock);
}

static const struct of_device_id mtk_cpufreq_hw_match[] = {
	{ .compatible = "mediatek,cpufreq-hw", .data = &cpufreq_mtk_offsets },
	{}
};

static struct platform_driver mtk_cpufreq_hw_driver = {
	.probe = mtk_cpufreq_hw_driver_probe,
	.remove = mtk_cpufreq_hw_driver_remove,
	.driver = {
		.name = "mtk-cpufreq-hw",
		.of_match_table = mtk_cpufreq_hw_match,
	},
};
module_platform_driver(mtk_cpufreq_hw_driver);
module_param(init_brake_policy0, uint, 0444);
module_param(init_brake_policy4, uint, 0444);
module_param(init_brake_policy7, uint, 0444);

MODULE_DESCRIPTION("Mediatek cpufreq-hw driver");
MODULE_LICENSE("GPL");
