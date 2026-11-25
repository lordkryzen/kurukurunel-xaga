// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "fbt_cpu_platform.h"
#include <sched/sched.h>
#include <mt-plat/fpsgo_common.h>
#include <linux/cpumask.h>
#include <linux/interconnect.h>
#include <linux/platform_device.h>
#include "dvfsrc-exp.h"
#include "fpsgo_base.h"

static int mask_int[FPSGO_PREFER_TOTAL];
static struct cpumask mask[FPSGO_PREFER_TOTAL];
static int mask_done;
static struct icc_path *bw_path;
static struct device_node *node;
static unsigned int peak_bw;
static int plat_gcc_enable;
static int plat_cpu_limit;
static bool platform_ready;
static int native_platform_fault;
module_param_named(native_platform_ready, platform_ready, bool, 0444);
module_param(native_platform_fault, int, 0444);

void fbt_notify_CM_limit(int reach_limit)
{
#if IS_ENABLED(CONFIG_MTK_CM_MGR)
	cm_mgr_perf_set_status(reach_limit);
#endif
	fpsgo_systrace_c_fbt_debug(-100, 0, reach_limit, "notify_cm");
}

static int generate_cpu_mask(void);
static int read_scalar_alias(const char *canonical, const char *legacy, int *value)
{
	u32 canonical_value, legacy_value;
	bool have_new = of_property_present(node, canonical);
	bool have_old = of_property_present(node, legacy);
	int ret;

	if (!have_new && !have_old)
		return 0; /* Native optional property: keep its explicit default. */
	ret = of_property_read_u32(node, have_new ? canonical : legacy, &canonical_value);
	if (ret)
		return ret;
	if (have_new && have_old) {
		ret = of_property_read_u32(node, legacy, &legacy_value);
		if (ret)
			return ret;
		if (canonical_value != legacy_value)
			return -EINVAL;
	}
	if (canonical_value > INT_MAX)
		return -ERANGE;
	*value = canonical_value;
	return 0;
}

static int platform_fpsgo_probe(struct platform_device *pdev)
{
	struct icc_path *path;
	int ret;

	if (platform_ready)
		return -EBUSY;
	node = pdev->dev.of_node;
	if (!node) {
		native_platform_fault = -EINVAL;
		return -EINVAL;
	}
	plat_gcc_enable = 0;
	plat_cpu_limit = 0;
	mask_done = 0;
	native_platform_fault = 0;
	path = of_icc_get(&pdev->dev, "fpsgo-perf-bw");
	if (IS_ERR(path)) {
		ret = PTR_ERR(path);
		goto fail;
	}
	if (!path) {
		ret = -ENODEV;
		goto fail;
	}
	peak_bw = dvfsrc_get_required_opp_peak_bw(node, 0);
	if (!peak_bw) {
		ret = -EINVAL;
		goto release;
	}
	ret = read_scalar_alias("gcc-enable", "gcc_enable", &plat_gcc_enable);
	if (ret)
		goto release;
	ret = read_scalar_alias("cpu-limit", "cpu_limit", &plat_cpu_limit);
	if (ret)
		goto release;
	ret = generate_cpu_mask();
	if (ret)
		goto release;
	bw_path = path;
	smp_store_release(&platform_ready, true);
	return 0;
release:
	icc_put(path);
fail:
	native_platform_fault = ret;
	return ret;
}

static void platform_fpsgo_remove(struct platform_device *pdev)
{
	icc_put(bw_path);
}

static const struct of_device_id platform_fpsgo_of_match[] = {
	{ .compatible = "mediatek,fpsgo", },
	{},
};

static const struct platform_device_id platform_fpsgo_id_table[] = {
	{ "fpsgo", 0},
	{ },
};

static struct platform_driver mtk_platform_fpsgo_driver = {
	.probe = platform_fpsgo_probe,
	.remove	= platform_fpsgo_remove,
	.driver = {
		.name = "fpsgo",
		.suppress_bind_attrs = true,
		.owner = THIS_MODULE,
		.of_match_table = platform_fpsgo_of_match,
	},
	.id_table = platform_fpsgo_id_table,
};

int init_fbt_platform(void)
{
	int ret = platform_driver_register(&mtk_platform_fpsgo_driver);

	if (ret)
		return ret;
	if (!smp_load_acquire(&platform_ready))
		return native_platform_fault ? native_platform_fault : -EPROBE_DEFER;
	return 0;
}

void exit_fbt_platform(void)
{
	platform_driver_unregister(&mtk_platform_fpsgo_driver);
}

void fbt_reg_dram_request(int reg)
{
}

void fbt_boost_dram(int boost)
{
	int ret;

	if (!smp_load_acquire(&platform_ready) || IS_ERR_OR_NULL(bw_path))
		return;
	if (boost && READ_ONCE(native_platform_fault))
		return;
	ret = icc_set_bw(bw_path, 0, boost ? peak_bw : 0);
	if (ret)
		WRITE_ONCE(native_platform_fault, ret);
	fpsgo_systrace_c_fbt_debug(-100, 0, boost, "dram_boost");
}

void fbt_set_boost_value(unsigned int base_blc)
{
	base_blc = clamp(base_blc, 1U, 100U);
	fpsgo_sentcmd(FPSGO_SET_BOOST_TA, base_blc, -1);
	fpsgo_systrace_c_fbt_debug(-100, 0, base_blc, "TA_cap");
}

void fbt_clear_boost_value(void)
{
	fpsgo_sentcmd(FPSGO_SET_BOOST_TA, -1, -1);
	fpsgo_systrace_c_fbt_debug(-100, 0, 0, "TA_cap");

	fbt_notify_CM_limit(0);
	fbt_boost_dram(0);
}

void fbt_set_per_task_cap(int pid, unsigned int min_blc, unsigned int max_blc)
{
	int ret = -1;
	unsigned int min_blc_1024;
	unsigned int max_blc_1024;
	struct task_struct *p;
	struct sched_attr attr = {};

	if (!pid)
		return;

	min_blc_1024 = (min_blc << 10) / 100U;
	min_blc_1024 = clamp(min_blc_1024, 1U, 1024U);

	max_blc_1024 = (max_blc << 10) / 100U;
	max_blc_1024 = clamp(max_blc_1024, 1U, 1024U);

	attr.sched_policy = -1;
	attr.sched_flags =
		SCHED_FLAG_KEEP_ALL |
		SCHED_FLAG_UTIL_CLAMP |
		SCHED_FLAG_RESET_ON_FORK;

	if (min_blc == 0 && max_blc == 100) {
		attr.sched_util_min = -1;
		attr.sched_util_max = -1;
	} else {
		attr.sched_util_min = (min_blc_1024 << 10) / 1280;
		attr.sched_util_max = (max_blc_1024 << 10) / 1280;
	}

	if (pid < 0)
		goto out;

	rcu_read_lock();
	p = find_task_by_vpid(pid);

	if (likely(p))
		get_task_struct(p);

	rcu_read_unlock();

	if (likely(p)) {
		attr.sched_policy = p->policy;
		ret = sched_setattr_nocheck(p, &attr);
		put_task_struct(p);
	}

out:
	if (ret != 0) {
		fpsgo_systrace_c_fbt(pid, 0, ret, "uclamp fail");
		fpsgo_systrace_c_fbt(pid, 0, 0, "uclamp fail");
		return;
	}

	fpsgo_systrace_c_fbt_debug(pid, 0, attr.sched_util_min, "min_cap");
	fpsgo_systrace_c_fbt_debug(pid, 0, attr.sched_util_max, "max_cap");
}

static int generate_cpu_mask(void)
{
	/* Newer DT appends B_M and M; the native four indices are unchanged. */
	u32 values[6], legacy_masks[6], allowed = 0;
	bool have_new = of_property_present(node, "fbt-cpu-mask");
	bool have_old = of_property_present(node, "fbt_cpu_mask");
	const char *property = have_new ? "fbt-cpu-mask" : "fbt_cpu_mask";
	int count, i, cpu, ret;

	count = of_property_count_u32_elems(node, property);
	if (count != FPSGO_PREFER_TOTAL && count != 6)
		return -EINVAL;
	ret = of_property_read_u32_array(node, property, values, count);
	if (ret)
		return ret;
	if (have_new && have_old) {
		if (of_property_count_u32_elems(node, "fbt_cpu_mask") != count)
			return -EINVAL;
		ret = of_property_read_u32_array(node, "fbt_cpu_mask", legacy_masks, count);
		if (ret)
			return ret;
		if (memcmp(values, legacy_masks, count * sizeof(*values)))
			return -EINVAL;
	}
	for_each_possible_cpu(cpu) {
		if (cpu >= 32)
			return -E2BIG;
		allowed |= BIT(cpu);
	}
	/* Validate every supplied entry, including the two extended masks. */
	for (i = 0; i < count; i++) {
		u32 bits = values[i];

		if (bits == ~0U)
			bits = allowed;
		if (!bits || (bits & ~allowed))
			return -EINVAL;
		values[i] = bits;
	}
	for (i = 0; i < FPSGO_PREFER_TOTAL; i++) {
		mask_int[i] = values[i];
		cpumask_clear(&mask[i]);
		for_each_possible_cpu(cpu)
			if (values[i] & BIT(cpu))
				cpumask_set_cpu(cpu, &mask[i]);
	}
	mask_done = 1;
	return 0;
}

void fbt_set_affinity(pid_t pid, unsigned int prefer_type)
{
	long ret = 0;

	if (!mask_done || prefer_type >= FPSGO_PREFER_TOTAL) {
		ret = -100;
		goto out;
	}

	ret = fpsgo_sched_setaffinity(pid, &mask[prefer_type]);

out:
	if (ret != 0) {
		fpsgo_systrace_c_fbt(pid, 0, ret, "setaffinity fail");
		fpsgo_systrace_c_fbt(pid, 0, 0, "setaffinity fail");
		return;
	}
	fpsgo_systrace_c_fbt(pid, 0, prefer_type, "set_affinity");
}

void fbt_set_cpu_prefer(int pid, unsigned int prefer_type)
{
}

int fbt_get_L_min_ceiling(void)
{
	return 0;
}

int fbt_get_default_boost_ta(void)
{
	return 0;
}

int fbt_get_default_adj_loading(void)
{
	return 0;
}

int fbt_get_default_adj_count(void)
{
	return 10;
}

int fbt_get_default_adj_tdiff(void)
{
	return 1000000;
}

int fbt_get_cluster_limit(int *cluster, int *freq, int *r_freq, int *cpu)
{
/*
 * Use return value to specify limit on frequency or core
 * 1. when return value is FPSGO_LIMIT_NO_LIMIT -> no limit
 * 2. when return value is FPSGO_LIMIT_FREQ -> frequency limit
 * 2.1 when cluster is set and freq is set -> ceiling limit
 * 2.2 when cluster is set and r_freq is set -> rescue ceiling limit
 * 3. when return value is FPSGO_LIMIT_CPU -> cpu limit
 * 3.1 when cpu is set valid -> cpu isolation
 */
	int limit = plat_cpu_limit;

	if (limit == FPSGO_LIMIT_CPU)
		*cpu = 7;
	else if (limit == FPSGO_LIMIT_FREQ) {
		*cluster = 2;
		*freq = 2600000;
	}

	return limit;
}

int fbt_get_default_uboost(void)
{
	return 75;
}

int fbt_get_default_qr_enable(void)
{
	return 1;
}

int fbt_get_default_gcc_enable(void)
{
	return plat_gcc_enable;
}

int fbt_get_l_min_bhropp(void)
{
	return 0;
}

