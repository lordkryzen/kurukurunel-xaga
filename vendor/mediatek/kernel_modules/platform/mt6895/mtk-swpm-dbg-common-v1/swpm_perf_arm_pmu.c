// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/perf_event.h>
#include <linux/perf/arm_pmuv3.h>
#include <linux/workqueue.h>

#include <swpm_perf_arm_pmu.h>

static struct perf_event_attr l3dc_event_attr = {
	.type           = PERF_TYPE_RAW,
/*	.config         = 0x2B, */
	.config		= ARMV8_PMUV3_PERFCTR_L3D_CACHE, /* 0x2B */
	.size           = sizeof(struct perf_event_attr),
	.pinned         = 1,
/*	.disabled       = 1, */
};

static struct perf_event_attr inst_spec_event_attr = {
	.type           = PERF_TYPE_RAW,
/*	.config         = 0x1B, */
	.config		= ARMV8_PMUV3_PERFCTR_INST_SPEC, /* 0x1B */
	.size           = sizeof(struct perf_event_attr),
	.pinned         = 1,
/*	.disabled       = 1, */
};

static struct perf_event_attr cycle_event_attr = {
	.type           = PERF_TYPE_HARDWARE,
	.config         = PERF_COUNT_HW_CPU_CYCLES,
	.size           = sizeof(struct perf_event_attr),
	.pinned         = 1,
/*	.disabled       = 1, */
};

/* Each immutable published bundle belongs to one possible CPU. */
struct swpm_cpu_events {
	struct perf_event *l3;
	struct perf_event *inst;
	struct perf_event *cycles;
	struct rcu_head rcu;
};

static DEFINE_PER_CPU(struct swpm_cpu_events __rcu *, swpm_events);
static DEFINE_MUTEX(swpm_pmu_lock);
static unsigned int boundary;
static unsigned int pmu_support;
static unsigned int pmu_dsu_support;
static unsigned int pmu_dsu_type;
static bool swpm_pmu_ready;
static bool swpm_pmu_stopping;
static bool swpm_pmu_requested;
static int swpm_pmu_hp_state = -1;

static void swpm_pmu_overflow(struct perf_event *event,
		struct perf_sample_data *data, struct pt_regs *regs)
{
}

static void swpm_release_bundle_events(struct swpm_cpu_events *events)
{
	if (events->cycles)
		perf_event_release_kernel(events->cycles);
	if (events->inst)
		perf_event_release_kernel(events->inst);
	if (events->l3)
		perf_event_release_kernel(events->l3);
}

static void swpm_drop_cpu(unsigned int cpu)
{
	struct swpm_cpu_events *events;

	events = rcu_access_pointer(per_cpu(swpm_events, cpu));
	if (!events)
		return;
	rcu_assign_pointer(per_cpu(swpm_events, cpu), NULL);
	/* Perf retires event storage through RCU; index readers use the same domain. */
	swpm_release_bundle_events(events);
	kfree_rcu(events, rcu);
}

static int swpm_create_counter(struct perf_event_attr *attr, unsigned int cpu,
			       struct perf_event **out)
{
	struct perf_event *event;

	event = perf_event_create_kernel_counter(attr, cpu, NULL,
					 swpm_pmu_overflow, NULL);
	if (IS_ERR(event))
		return PTR_ERR(event);
	perf_event_enable(event);
	if (READ_ONCE(event->hw.idx) < 0) {
		perf_event_release_kernel(event);
		return -ENOSPC;
	}
	*out = event;
	return 0;
}

static int swpm_add_cpu(unsigned int cpu)
{
	struct swpm_cpu_events *events;
	int ret;

	if (rcu_access_pointer(per_cpu(swpm_events, cpu)))
		return 0;
	events = kzalloc(sizeof(*events), GFP_KERNEL);
	if (!events)
		return -ENOMEM;
	ret = swpm_create_counter(&l3dc_event_attr, cpu, &events->l3);
	if (ret)
		goto fail;
	if (cpu >= boundary) {
		ret = swpm_create_counter(&inst_spec_event_attr, cpu, &events->inst);
		if (ret)
			goto fail;
		ret = swpm_create_counter(&cycle_event_attr, cpu, &events->cycles);
		if (ret)
			goto fail;
	}
	rcu_assign_pointer(per_cpu(swpm_events, cpu), events);
	return 0;
fail:
	swpm_release_bundle_events(events);
	kfree(events);
	return ret;
}

static int swpm_pmu_cpu_online(unsigned int cpu)
{
	int ret = 0;

	mutex_lock(&swpm_pmu_lock);
	if (swpm_pmu_ready && !swpm_pmu_stopping && swpm_pmu_requested)
		ret = swpm_add_cpu(cpu);
	mutex_unlock(&swpm_pmu_lock);
	/* Telemetry allocation failure must not prevent a CPU coming online. */
	if (ret)
		pr_warn_ratelimited("SWPM CPU%u PMU unavailable: %d\n", cpu, ret);
	return 0;
}

static int swpm_pmu_cpu_offline(unsigned int cpu)
{
	mutex_lock(&swpm_pmu_lock);
	swpm_drop_cpu(cpu);
	mutex_unlock(&swpm_pmu_lock);
	return 0;
}

int swpm_arm_pmu_get_idx(unsigned int evt_id, unsigned int cpu)
{
	struct swpm_cpu_events *events;
	struct perf_event *event = NULL;
	int idx = -1;

	if (evt_id == DSU_CYCLES_EVT || cpu >= nr_cpu_ids ||
	    !cpu_possible(cpu) || !cpu_online(cpu))
		return -1;
	rcu_read_lock();
	if (!READ_ONCE(swpm_pmu_ready) || READ_ONCE(swpm_pmu_stopping))
		goto out;
	events = rcu_dereference(per_cpu(swpm_events, cpu));
	if (!events)
		goto out;
	switch (evt_id) {
	case L3DC_EVT:
		event = events->l3;
		break;
	case INST_SPEC_EVT:
		event = events->inst;
		break;
	case CYCLES_EVT:
		event = events->cycles;
		break;
	}
	/* Return the current native perf index, not the old 5.10 numbering. */
	if (event)
		idx = READ_ONCE(event->hw.idx);
out:
	rcu_read_unlock();
	return idx;
}
EXPORT_SYMBOL(swpm_arm_pmu_get_idx);

int swpm_arm_dsu_pmu_get_status(void)
{
	/* This selected xaga profile has no DSU perf event. */
	return 0;
}
EXPORT_SYMBOL(swpm_arm_dsu_pmu_get_status);

int swpm_arm_pmu_get_status(void)
{
	unsigned int cpu;
	bool enabled;

	rcu_read_lock();
	enabled = READ_ONCE(swpm_pmu_ready) && !READ_ONCE(swpm_pmu_stopping) &&
		  READ_ONCE(swpm_pmu_requested);
	if (enabled) {
		for_each_online_cpu(cpu) {
			struct swpm_cpu_events *events;

			events = rcu_dereference(per_cpu(swpm_events, cpu));
			if (!events || READ_ONCE(events->l3->hw.idx) < 0 ||
			    (cpu >= boundary &&
			     (READ_ONCE(events->inst->hw.idx) < 0 ||
			      READ_ONCE(events->cycles->hw.idx) < 0))) {
				enabled = false;
				break;
			}
		}
	}
	rcu_read_unlock();
	return (pmu_dsu_support << 24) | (boundary << 20) | enabled;
}
EXPORT_SYMBOL(swpm_arm_pmu_get_status);

int swpm_arm_dsu_pmu_enable(unsigned int enable)
{
	if (!READ_ONCE(swpm_pmu_ready) || READ_ONCE(swpm_pmu_stopping))
		return -ENODEV;
	return enable ? -EOPNOTSUPP : 0;
}
EXPORT_SYMBOL(swpm_arm_dsu_pmu_enable);

int swpm_arm_pmu_enable_all(unsigned int enable)
{
	cpumask_t created;
	unsigned int cpu;
	int ret = 0;

	/* CPU hotplug takes its write lock before the same control mutex. */
	cpus_read_lock();
	mutex_lock(&swpm_pmu_lock);
	if (!swpm_pmu_ready || swpm_pmu_stopping) {
		ret = -ENODEV;
		goto out;
	}
	if (!enable) {
		WRITE_ONCE(swpm_pmu_requested, false);
		for_each_possible_cpu(cpu)
			swpm_drop_cpu(cpu);
		goto out;
	}
	if (!pmu_support) {
		ret = -EOPNOTSUPP;
		goto out;
	}
	cpumask_clear(&created);
	for_each_online_cpu(cpu) {
		if (rcu_access_pointer(per_cpu(swpm_events, cpu)))
			continue;
		ret = swpm_add_cpu(cpu);
		if (ret)
			goto rollback;
		cpumask_set_cpu(cpu, &created);
	}
	WRITE_ONCE(swpm_pmu_requested, true);
	goto out;
rollback:
	for_each_cpu(cpu, &created)
		swpm_drop_cpu(cpu);
out:
	mutex_unlock(&swpm_pmu_lock);
	cpus_read_unlock();
	return ret;
}
EXPORT_SYMBOL(swpm_arm_pmu_enable_all);

static int __init swpm_arm_pmu_init(void)
{
	struct device_node *node;
	int ret;

	node = of_find_compatible_node(NULL, NULL, "mediatek,mtk-swpm");
	if (!node)
		return -ENODEV;
	if (!of_device_is_available(node)) {
		ret = -ENODEV;
		goto put_node;
	}
	pmu_support = 1;
	if (of_find_property(node, "pmu_support", NULL)) {
		ret = of_property_read_u32_index(node, "pmu_support", 0, &pmu_support);
		if (ret)
			goto put_node;
	}
	ret = of_property_read_u32_index(node, "pmu_boundary_num", 0, &boundary);
	if (ret)
		goto put_node;
	ret = of_property_read_u32_index(node, "pmu_dsu_support", 0, &pmu_dsu_support);
	if (ret)
		goto put_node;
	ret = of_property_read_u32_index(node, "pmu_dsu_type", 0, &pmu_dsu_type);
	if (ret)
		goto put_node;
	if (pmu_support > 1 || boundary > nr_cpu_ids || boundary > 15 ||
	    pmu_dsu_support) {
		ret = -EOPNOTSUPP;
		goto put_node;
	}
put_node:
	of_node_put(node);
	if (ret)
		return ret;
	swpm_pmu_hp_state = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,
			"swpm/pmu:online", swpm_pmu_cpu_online, swpm_pmu_cpu_offline);
	if (swpm_pmu_hp_state < 0)
		return swpm_pmu_hp_state;
	WRITE_ONCE(swpm_pmu_ready, true);
	ret = swpm_arm_pmu_enable_all(pmu_support);
	if (ret) {
		WRITE_ONCE(swpm_pmu_ready, false);
		cpuhp_remove_state_nocalls(swpm_pmu_hp_state);
		swpm_pmu_hp_state = -1;
	}
	return ret;
}
module_init(swpm_arm_pmu_init);

static void __exit swpm_arm_pmu_exit(void)
{
	unsigned int cpu;

	cpus_read_lock();
	mutex_lock(&swpm_pmu_lock);
	WRITE_ONCE(swpm_pmu_stopping, true);
	WRITE_ONCE(swpm_pmu_requested, false);
	WRITE_ONCE(swpm_pmu_ready, false);
	mutex_unlock(&swpm_pmu_lock);
	cpus_read_unlock();
	if (swpm_pmu_hp_state >= 0) {
		cpuhp_remove_state_nocalls(swpm_pmu_hp_state);
		swpm_pmu_hp_state = -1;
	}
	cpus_read_lock();
	mutex_lock(&swpm_pmu_lock);
	for_each_possible_cpu(cpu)
		swpm_drop_cpu(cpu);
	mutex_unlock(&swpm_pmu_lock);
	cpus_read_unlock();
}
module_exit(swpm_arm_pmu_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek SWPM ARM PMU Module");
MODULE_AUTHOR("MediaTek Inc.");

#include "swpm_counts.inc"
