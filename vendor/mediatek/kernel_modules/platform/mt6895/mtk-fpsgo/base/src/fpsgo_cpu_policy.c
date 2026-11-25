// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2020 MediaTek Inc. */
#include <linux/slab.h>
#include <linux/cpufreq.h>
#include "fpsgo_cpu_policy.h"
#include "mtk_cpu_capacity.h"

static int policy_num;
static int *opp_count;
static unsigned int **opp_table;

static void free_policy_copy(int count, int *counts, unsigned int **tables)
{
	int i;

	if (tables)
		for (i = 0; i < count; i++)
			kfree(tables[i]);
	kfree(tables);
	kfree(counts);
}

void fpsgo_cpu_policy_exit(void)
{
	free_policy_copy(policy_num, opp_count, opp_table);
	opp_table = NULL;
	opp_count = NULL;
	policy_num = 0;
}

int fpsgo_cpu_policy_init(void)
{
	struct cpufreq_policy *policy;
	int cpu, count, i, nr = 0, num = 0, ret = -ENOMEM;
	int *counts;
	unsigned int **tables;

	if (policy_num)
		return -EBUSY;
	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			return -EPROBE_DEFER;
		nr++;
		cpu = cpumask_last(policy->related_cpus);
		cpufreq_cpu_put(policy);
	}
	if (!nr)
		return -ENODEV;
	counts = kcalloc(nr, sizeof(*counts), GFP_KERNEL);
	tables = kcalloc(nr, sizeof(*tables), GFP_KERNEL);
	if (!counts || !tables)
		goto fail;
	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			ret = -EPROBE_DEFER;
			goto fail;
		}
		count = mtk_cpu_capacity_count(cpu);
		if (num >= nr || count <= 0 || count > NR_FREQ_CPU) {
			cpufreq_cpu_put(policy);
			ret = -EINVAL;
			goto fail;
		}
		counts[num] = count;
		tables[num] = kcalloc(count, sizeof(**tables), GFP_KERNEL);
		if (!tables[num]) {
			cpufreq_cpu_put(policy);
			goto fail;
		}
		for (i = 0; i < count; i++) {
			tables[num][i] = mtk_cpu_opp_frequency(cpu, i);
			if (!tables[num][i]) {
				cpufreq_cpu_put(policy);
				ret = -EPROTO;
				goto fail;
			}
		}
		num++;
		cpu = cpumask_last(policy->related_cpus);
		cpufreq_cpu_put(policy);
	}
	if (num != nr) {
		ret = -EINVAL;
		goto fail;
	}
	opp_count = counts;
	opp_table = tables;
	policy_num = nr;
	return 0;
fail:
	free_policy_copy(nr, counts, tables);
	return ret;
}

int fpsgo_get_cpu_policy_num(void)
{
	return policy_num;
}

int fpsgo_get_cpu_opp_info(int **out_count, unsigned int ***out_table)
{
	int *counts;
	unsigned int **tables;
	int i;

	if (!out_count || !out_table)
		return -EINVAL;
	if (!policy_num)
		return -ENODEV;
	counts = kcalloc(policy_num, sizeof(*counts), GFP_KERNEL);
	tables = kcalloc(policy_num, sizeof(*tables), GFP_KERNEL);
	if (!counts || !tables)
		goto fail;
	for (i = 0; i < policy_num; i++) {
		counts[i] = opp_count[i];
		tables[i] = kmemdup(opp_table[i], counts[i] * sizeof(**tables), GFP_KERNEL);
		if (!tables[i])
			goto fail;
	}
	*out_count = counts;
	*out_table = tables;
	return 0;
fail:
	free_policy_copy(policy_num, counts, tables);
	return -ENOMEM;
}
