// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/energy_model.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/sort.h>
#include "mtk_cpu_capacity.h"
#include "capacity_table.h"

static struct capacity_domain domains[CAP_MAX_DOMAINS];
static int cpu_domain[NR_CPUS];
static int domain_count;
static int first_cpu[CAP_MAX_DOMAINS], opp_count[CAP_MAX_DOMAINS];
static uint max_capacity[CAP_MAX_DOMAINS];
static bool native_ready;
module_param(native_ready, bool, 0444);
module_param_array(first_cpu, int, &domain_count, 0444);
module_param_array(opp_count, int, NULL, 0444);
module_param_array(max_capacity, uint, NULL, 0444);

static const struct capacity_domain *capacity_for_cpu(unsigned int cpu)
{
	int id;

	if (!smp_load_acquire(&native_ready) || cpu >= nr_cpu_ids || !cpu_possible(cpu))
		return NULL;
	id = cpu_domain[cpu];
	if (id < 0 || id >= domain_count)
		return NULL;
	return &domains[id];
}

int mtk_cpu_capacity_count(unsigned int cpu)
{
	const struct capacity_domain *domain = capacity_for_cpu(cpu);

	if (!domain)
		return -ENODEV;
	return domain->count;
}
EXPORT_SYMBOL_GPL(mtk_cpu_capacity_count);

unsigned long mtk_cpu_opp_capacity(unsigned int cpu, int opp)
{
	const struct capacity_domain *domain = capacity_for_cpu(cpu);

	if (!domain)
		return 0;
	/* Preserve native v3's explicit out-of-range -> maximum OPP contract. */
	if (opp < 0 || (unsigned int)opp >= domain->count)
		opp = 0;
	return domain->capacity[opp];
}
EXPORT_SYMBOL_GPL(mtk_cpu_opp_capacity);

unsigned int mtk_cpu_opp_frequency(unsigned int cpu, int opp)
{
	const struct capacity_domain *domain = capacity_for_cpu(cpu);

	if (!domain)
		return 0;
	if (opp < 0 || (unsigned int)opp >= domain->count)
		opp = 0;
	return domain->frequency[opp];
}
EXPORT_SYMBOL_GPL(mtk_cpu_opp_frequency);

static int descending_frequency(const void *left, const void *right)
{
	unsigned int a = *(const unsigned int *)left;
	unsigned int b = *(const unsigned int *)right;

	return a < b ? 1 : a > b ? -1 : 0;
}

static int collect_domains(void)
{
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *entry;
	struct em_perf_domain *pd;
	int cpu, member, id, nr;

	for_each_possible_cpu(cpu)
		cpu_domain[cpu] = -1;
	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			return -EPROBE_DEFER;
		if (cpu != cpumask_first(policy->related_cpus)) {
			cpufreq_cpu_put(policy);
			continue;
		}
		id = domain_count;
		if (id >= CAP_MAX_DOMAINS || !policy->freq_table) {
			cpufreq_cpu_put(policy);
			return -E2BIG;
		}
		nr = 0;
		cpufreq_for_each_valid_entry(entry, policy->freq_table) {
			if (nr >= CAP_MAX_OPPS) {
				cpufreq_cpu_put(policy);
				return -E2BIG;
			}
			domains[id].frequency[nr++] = entry->frequency;
		}
		pd = em_cpu_get(cpu);
		if (!nr || !pd || pd->nr_perf_states != nr ||
		    !cpumask_equal(to_cpumask(pd->cpus), policy->related_cpus)) {
			cpufreq_cpu_put(policy);
			return -EINVAL;
		}
		for_each_cpu(member, policy->related_cpus) {
			if (cpu_domain[member] != -1) {
				cpufreq_cpu_put(policy);
				return -EINVAL;
			}
			cpu_domain[member] = id;
		}
		domains[id].count = nr;
		sort(domains[id].frequency, nr, sizeof(unsigned int), descending_frequency, NULL);
		first_cpu[id] = cpu;
		opp_count[id] = nr;
		domain_count++;
		cpufreq_cpu_put(policy);
	}
	for_each_possible_cpu(cpu)
		if (cpu_domain[cpu] < 0)
			return -EINVAL;
	return domain_count ? 0 : -EPROBE_DEFER;
}

static int __init cpu_capacity_init(void)
{
	struct device_node *node;
	struct platform_device *provider;
	struct resource *res;
	void __iomem *mapped;
	u16 raw[CAPACITY_WORDS], second[CAPACITY_WORDS];
	int ret, i;

	ret = collect_domains();
	if (ret)
		return ret;
	node = of_find_node_by_name(NULL, "cpuhvfs");
	if (!node)
		return -ENODEV;
	provider = of_find_device_by_node(node);
	of_node_put(node);
	if (!provider)
		return -EPROBE_DEFER;
	res = platform_get_resource_byname(provider, IORESOURCE_MEM, "CSRAM");
	if (!res || resource_size(res) < CAPACITY_OFFSET + CAPACITY_BYTES) {
		ret = -EINVAL;
		goto put_provider;
	}
	mapped = ioremap(res->start + CAPACITY_OFFSET, CAPACITY_BYTES);
	if (!mapped) {
		ret = -ENOMEM;
		goto put_provider;
	}
	for (i = 0; i < CAPACITY_WORDS; i++)
		raw[i] = ioread16(mapped + i * sizeof(u16));
	msleep(2);
	for (i = 0; i < CAPACITY_WORDS; i++)
		second[i] = ioread16(mapped + i * sizeof(u16));
	iounmap(mapped);
	if (memcmp(raw, second, sizeof(raw))) {
		ret = -EAGAIN;
		goto put_provider;
	}
	ret = capacity_decode(raw, CAPACITY_WORDS, domains, domain_count);
	if (ret)
		goto put_provider;
	for (i = 0; i < domain_count; i++)
		max_capacity[i] = domains[i].capacity[0];
	smp_store_release(&native_ready, true);
	pr_info("Native CPU capacity provider ready: %d validated firmware domains; no scheduler hooks\n", domain_count);
put_provider:
	put_device(&provider->dev);
	return ret;
}
static void __exit cpu_capacity_exit(void)
{
	smp_store_release(&native_ready, false);
}
module_init(cpu_capacity_init);
module_exit(cpu_capacity_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Validated native firmware CPU capacity and frequency provider");
