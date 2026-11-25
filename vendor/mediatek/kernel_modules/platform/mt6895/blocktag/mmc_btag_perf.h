/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_MMC_BTAG_PERF_H
#define MTK_MMC_BTAG_PERF_H

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpuhotplug.h>
#include <linux/device.h>
#include <linux/interconnect.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#define MMC_PERF_DOMAINS 3
static const int mmc_perf_floor[MMC_PERF_DOMAINS] = { 2000000, 2000000, 1500000 };

struct mmc_perf_vote {
	struct cpufreq_policy *policy;
	struct freq_qos_request request;
};

struct mmc_perf_owner {
	struct mmc_host *mmc;
	struct icc_path *path;
	u32 peak_bw;
	struct work_struct work;
	spinlock_t lock;
	bool closed, desired;
	int error;
	u64 generation;
	struct mmc_perf_vote vote[MMC_PERF_DOMAINS];
	int hp_state;
	struct hlist_node hp_node;
	bool hp_registered, device_ref;
};

static struct mmc_perf_owner __rcu *mmc_perf_active;

static int mmc_perf_domain(unsigned int cpu)
{
	struct of_phandle_args args;
	struct device_node *node;
	int ret;

	if (cpu >= nr_cpu_ids || !cpu_possible(cpu))
		return -EINVAL;
	node = of_get_cpu_node(cpu, NULL);
	if (!node)
		return -ENODEV;
	ret = of_parse_phandle_with_args(node, "performance-domains",
		"#performance-domain-cells", 0, &args);
	of_node_put(node);
	if (ret)
		return ret;
	if (args.args_count != 1 || args.args[0] >= MMC_PERF_DOMAINS ||
	    !of_device_is_compatible(args.np, "mediatek,cpufreq-hw"))
		ret = -EINVAL;
	else
		ret = args.args[0];
	of_node_put(args.np);
	return ret;
}

static void mmc_perf_drop_votes(struct mmc_perf_owner *owner)
{
	int row;

	for (row = MMC_PERF_DOMAINS - 1; row >= 0; row--) {
		struct mmc_perf_vote *vote = &owner->vote[row];

		if (!vote->policy)
			continue;
		freq_qos_remove_request(&vote->request);
		cpufreq_cpu_put(vote->policy);
		vote->policy = NULL;
	}
}

/* Caller owns the CPU read lock; only this owner's worker changes its votes. */
static int mmc_perf_prepare_votes(struct mmc_perf_owner *owner)
{
	struct cpufreq_policy *policy;
	unsigned int cpu, member;
	int row, ret;

	for_each_online_cpu(cpu) {
		row = mmc_perf_domain(cpu);
		if (row < 0) {
			ret = row;
			goto fail;
		}
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			ret = -EPROBE_DEFER;
			goto fail;
		}
		if (!cpumask_test_cpu(cpu, policy->related_cpus)) {
			ret = -EINVAL;
			goto put_policy;
		}
		for_each_cpu(member, policy->related_cpus) {
			ret = mmc_perf_domain(member);
			if (ret != row) {
				if (ret >= 0)
					ret = -EINVAL;
				goto put_policy;
			}
		}
		if (owner->vote[row].policy) {
			if (owner->vote[row].policy != policy) {
				ret = -EINVAL;
				goto put_policy;
			}
			cpufreq_cpu_put(policy);
			continue;
		}
		ret = freq_qos_add_request(&policy->constraints, &owner->vote[row].request,
			FREQ_QOS_MIN, mmc_perf_floor[row]);
		if (ret < 0)
			goto put_policy;
		owner->vote[row].policy = policy;
	}
	return 0;
put_policy:
	cpufreq_cpu_put(policy);
fail:
	mmc_perf_drop_votes(owner);
	return ret;
}

static int mmc_perf_apply(struct mmc_perf_owner *owner, bool boost)
{
	int ret, undo;

	/* Refresh the whole owned cohort under hotplug exclusion. No borrowed QoS. */
	cpus_read_lock();
	mmc_perf_drop_votes(owner);
	ret = icc_set_bw(owner->path, 0, 0);
	if (!ret && boost) {
		ret = mmc_perf_prepare_votes(owner);
		if (!ret)
			ret = icc_set_bw(owner->path, 0, owner->peak_bw);
		if (ret) {
			mmc_perf_drop_votes(owner);
			undo = icc_set_bw(owner->path, 0, 0);
			if (undo)
				dev_warn(owner->mmc->parent, "MMC bandwidth rollback failed: %d\n", undo);
		}
	}
	cpus_read_unlock();
	return ret;
}

static void mmc_perf_work(struct work_struct *work)
{
	struct mmc_perf_owner *owner = container_of(work, struct mmc_perf_owner, work);
	unsigned long flags;
	bool desired;
	u64 generation;
	int ret;

	for (;;) {
		spin_lock_irqsave(&owner->lock, flags);
		if (owner->closed) {
			spin_unlock_irqrestore(&owner->lock, flags);
			return;
		}
		desired = owner->desired;
		generation = owner->generation;
		spin_unlock_irqrestore(&owner->lock, flags);
		ret = mmc_perf_apply(owner, desired);
		if (ret)
			dev_warn_ratelimited(owner->mmc->parent, "MMC performance vote failed: %d\n", ret);
		spin_lock_irqsave(&owner->lock, flags);
		owner->error = ret;
		if (owner->closed || owner->generation == generation) {
			spin_unlock_irqrestore(&owner->lock, flags);
			return;
		}
		spin_unlock_irqrestore(&owner->lock, flags);
	}
}

static void mmc_perf_queue(struct mmc_perf_owner *owner, bool desired, bool refresh)
{
	unsigned long flags;

	spin_lock_irqsave(&owner->lock, flags);
	if (!owner->closed && (refresh || owner->desired != desired || owner->error)) {
		owner->desired = desired;
		owner->generation++;
		queue_work(system_unbound_wq, &owner->work);
	}
	spin_unlock_irqrestore(&owner->lock, flags);
}

static int mmc_perf_cpu_change(unsigned int cpu, struct hlist_node *node)
{
	struct mmc_perf_owner *owner = container_of(node, struct mmc_perf_owner, hp_node);
	unsigned long flags;

	/* CPU-hotplug callbacks only queue; the worker takes CPU locks afterward. */
	spin_lock_irqsave(&owner->lock, flags);
	if (!owner->closed && owner->desired) {
		owner->generation++;
		queue_work(system_unbound_wq, &owner->work);
	}
	spin_unlock_irqrestore(&owner->lock, flags);
	return 0;
}

static void mmc_perf_close(struct mmc_perf_owner *owner)
{
	unsigned long flags;

	if (!owner)
		return;
	spin_lock_irqsave(&owner->lock, flags);
	owner->closed = true;
	spin_unlock_irqrestore(&owner->lock, flags);
	if (owner->hp_registered) {
		cpuhp_state_remove_instance_nocalls(owner->hp_state, &owner->hp_node);
		owner->hp_registered = false;
	}
	if (owner->hp_state >= 0) {
		cpuhp_remove_multi_state(owner->hp_state);
		owner->hp_state = -1;
	}
}

/* Publication/readers must be closed and drained before this final release. */
static void mmc_perf_destroy(struct mmc_perf_owner *owner)
{
	int ret;

	if (!owner)
		return;
	cancel_work_sync(&owner->work);
	ret = mmc_perf_apply(owner, false);
	if (ret)
		dev_warn(owner->mmc->parent, "MMC bandwidth release failed: %d\n", ret);
	if (owner->device_ref)
		put_device(&owner->mmc->class_dev);
	kfree(owner);
}

static struct mmc_perf_owner *mmc_perf_prepare(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_perf_owner *owner;
	unsigned int cpu, domains = 0;
	unsigned long flags;
	int ret;

	if ((mmc->caps2 & MMC_CAP2_NO_SD) || !host->qos_enable)
		return NULL;
	if (IS_ERR_OR_NULL(host->bw_path))
		return IS_ERR(host->bw_path) ? ERR_CAST(host->bw_path) : ERR_PTR(-ENODEV);
	if (num_possible_cpus() != 8)
		return ERR_PTR(-EOPNOTSUPP);
	for_each_possible_cpu(cpu) {
		ret = mmc_perf_domain(cpu);
		if (ret < 0)
			return ERR_PTR(ret);
		domains |= BIT(ret);
	}
	if (domains != GENMASK(MMC_PERF_DOMAINS - 1, 0))
		return ERR_PTR(-EINVAL);
	owner = kzalloc(sizeof(*owner), GFP_KERNEL);
	if (!owner)
		return ERR_PTR(-ENOMEM);
	owner->mmc = mmc;
	owner->path = host->bw_path;
	owner->peak_bw = host->peak_bw;
	owner->closed = true;
	owner->hp_state = -1;
	spin_lock_init(&owner->lock);
	INIT_WORK(&owner->work, mmc_perf_work);
	INIT_HLIST_NODE(&owner->hp_node);
	if (!get_device(&mmc->class_dev)) {
		ret = -ENODEV;
		goto fail;
	}
	owner->device_ref = true;
	ret = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN, "blocktag/mmc-perf:online",
		mmc_perf_cpu_change, mmc_perf_cpu_change);
	if (ret < 0)
		goto fail;
	owner->hp_state = ret;
	ret = cpuhp_state_add_instance_nocalls(owner->hp_state, &owner->hp_node);
	if (ret)
		goto fail;
	owner->hp_registered = true;
	spin_lock_irqsave(&owner->lock, flags);
	owner->closed = false;
	spin_unlock_irqrestore(&owner->lock, flags);
	return owner;
fail:
	mmc_perf_close(owner);
	mmc_perf_destroy(owner);
	return ERR_PTR(ret);
}

#endif
