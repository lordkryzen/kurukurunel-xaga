// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2019 MediaTek Inc.
 */

/*
 * GenieZone (hypervisor-based seucrity platform) enables hardware protected
 * and isolated security execution environment, includes
 * 1. GZ hypervisor
 * 2. Hypervisor-TEE OS (built-in Trusty OS)
 * 3. Drivers (ex: debug, communication and interrupt) for GZ and
 *    hypervisor-TEE OS
 * 4. GZ and hypervisor-TEE and GZ framework (supporting multiple TEE
 *    ecosystem, ex: M-TEE, Trusty, GlobalPlatform, ...)
 */
/*
 * This is interrupt driver
 *
 * GZ does not support virtual interrupt, interrupt forwarding driver is
 * need for passing GZ and hypervisor-TEE interrupts
 */

#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <gz-trusty/smcall.h>
#include <gz-trusty/sm_err.h>
#include <gz-trusty/trusty.h>
#include <linux/irqdomain.h>
#include <linux/of_irq.h>
#include <dt-bindings/interrupt-controller/arm-gic.h>

#define ENABLE_IRQ_DEBUG_LOG (0)
#if ENABLE_IRQ_DEBUG_LOG
#define IRQ_DEBUG_LOG_EN
#endif

#define enable_code 0 /*replace #if 0*/

struct trusty_irq {
	struct trusty_irq_state *is;
	struct hlist_node node;
	unsigned int irq;
	bool percpu;
	bool enable;
	struct trusty_irq __percpu *percpu_ptr;
};

struct trusty_irq_irqset {
	struct hlist_head pending;
	struct hlist_head inactive;
};

struct trusty_irq_state {
	struct device *dev;
	struct device *trusty_dev;
	struct trusty_irq_irqset normal_irqs;
	spinlock_t normal_irqs_lock;
	struct trusty_irq_irqset __percpu *percpu_irqs;
	struct notifier_block trusty_call_notifier;
	struct hlist_node cpuhp_node;
	enum tee_id_t tee_id;
	struct device_node *spi_node;
	struct device_node *ppi_node;
	bool notifier_registered;
	bool cpuhp_added;
	unsigned int normal_count;
	unsigned int percpu_count;
};

static int trusty_irq_cpuhp_slot = -1;


static void trusty_irq_enable_pending_irqs(struct trusty_irq_state *is,
					   struct trusty_irq_irqset *irqset,
					   bool percpu)
{
	struct hlist_node *n;
	struct trusty_irq *trusty_irq;

	hlist_for_each_entry_safe(trusty_irq, n, &irqset->pending, node) {
#ifdef IRQ_DEBUG_LOG_EN
		dev_dbg(is->dev,
			"%s: enable pending irq %d, percpu %d, cpu %d\n",
			__func__, trusty_irq->irq, percpu, smp_processor_id());
#endif
		if (percpu)
			enable_percpu_irq(trusty_irq->irq, IRQ_TYPE_LEVEL_HIGH);
		else
			enable_irq(trusty_irq->irq);

		hlist_del(&trusty_irq->node);
		hlist_add_head(&trusty_irq->node, &irqset->inactive);
	}
}

static void trusty_irq_enable_irqset(struct trusty_irq_state *is,
				     struct trusty_irq_irqset *irqset)
{
	struct trusty_irq *trusty_irq;

	hlist_for_each_entry(trusty_irq, &irqset->inactive, node) {
		if (trusty_irq->enable) {
			dev_info(is->dev,
				 "%s: percpu irq %d already enabled, cpu %d\n",
				 __func__, trusty_irq->irq, smp_processor_id());
			continue;
		}
#ifdef IRQ_DEBUG_LOG_EN
		dev_dbg(is->dev, "%s: enable percpu irq %d, cpu %d\n",
			__func__, trusty_irq->irq, smp_processor_id());
#endif
		enable_percpu_irq(trusty_irq->irq, IRQ_TYPE_LEVEL_HIGH);
		trusty_irq->enable = true;
	}
}

static void trusty_irq_disable_irqset(struct trusty_irq_state *is,
				      struct trusty_irq_irqset *irqset)
{
	struct hlist_node *n;
	struct trusty_irq *trusty_irq;

	hlist_for_each_entry(trusty_irq, &irqset->inactive, node) {
		if (!trusty_irq->enable) {
			dev_info(is->dev,
				 "irq %d already disabled, percpu %d, cpu %d\n",
				 trusty_irq->irq, trusty_irq->percpu,
				 smp_processor_id());
			continue;
		}
#ifdef IRQ_DEBUG_LOG_EN
		dev_dbg(is->dev, "%s: disable irq %d, percpu %d, cpu %d\n",
			__func__, trusty_irq->irq, trusty_irq->percpu,
			smp_processor_id());
#endif
		trusty_irq->enable = false;
		if (trusty_irq->percpu)
			disable_percpu_irq(trusty_irq->irq);
		else
			disable_irq_nosync(trusty_irq->irq);
	}
	hlist_for_each_entry_safe(trusty_irq, n, &irqset->pending, node) {
		if (!trusty_irq->enable) {
			dev_info(is->dev,
				 "pending irq %d already disabled, percpu %d, cpu %d\n",
				 trusty_irq->irq, trusty_irq->percpu,
				 smp_processor_id());
		}
#ifdef IRQ_DEBUG_LOG_EN
		dev_dbg(is->dev,
			"%s: disable pending irq %d, percpu %d, cpu %d\n",
			__func__, trusty_irq->irq, trusty_irq->percpu,
			smp_processor_id());
#endif
		trusty_irq->enable = false;
		hlist_del(&trusty_irq->node);
		hlist_add_head(&trusty_irq->node, &irqset->inactive);
	}
}

static int trusty_irq_call_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct trusty_irq_state *is;

	WARN_ON(!irqs_disabled());

	if (action != TRUSTY_CALL_PREPARE)
		return NOTIFY_DONE;

	is = container_of(nb, struct trusty_irq_state, trusty_call_notifier);

	spin_lock(&is->normal_irqs_lock);
	trusty_irq_enable_pending_irqs(is, &is->normal_irqs, false);
	spin_unlock(&is->normal_irqs_lock);
	trusty_irq_enable_pending_irqs(is, this_cpu_ptr(is->percpu_irqs), true);

	return NOTIFY_OK;
}

irqreturn_t trusty_irq_handler(int irq, void *data)
{
	struct trusty_irq *trusty_irq = data;
	struct trusty_irq_state *is = trusty_irq->is;
	struct trusty_irq_irqset *irqset;

#ifdef IRQ_DEBUG_LOG_EN
	dev_dbg(is->dev, "%s: irq %d, percpu %d, cpu %d, enable %d\n",
		__func__, irq, trusty_irq->irq, smp_processor_id(),
		trusty_irq->enable);
#endif

	if (trusty_irq->percpu) {
		disable_percpu_irq(irq);
		irqset = this_cpu_ptr(is->percpu_irqs);
	} else {
		disable_irq_nosync(irq);
		irqset = &is->normal_irqs;
	}

	spin_lock(&is->normal_irqs_lock);
	if (trusty_irq->enable) {
		hlist_del(&trusty_irq->node);
		hlist_add_head(&trusty_irq->node, &irqset->pending);
	}
	spin_unlock(&is->normal_irqs_lock);

	gz_trusty_enqueue_nop(is->trusty_dev, NULL, smp_processor_id());

#ifdef IRQ_DEBUG_LOG_EN
	dev_dbg(is->dev, "%s: irq %d done\n", __func__, irq);
#endif

	return IRQ_HANDLED;
}

static int trusty_irq_cpu_up(unsigned int cpu, struct hlist_node *node)
{
	unsigned long irq_flags;
	struct trusty_irq_state *is;

	is = container_of(node, struct trusty_irq_state, cpuhp_node);
#ifdef IRQ_DEBUG_LOG_EN
	dev_dbg(is->dev, "%s: cpu %d\n", __func__, cpu);
#endif

	local_irq_save(irq_flags);
	trusty_irq_enable_irqset(is, this_cpu_ptr(is->percpu_irqs));
	local_irq_restore(irq_flags);

	return 0;
}

static int trusty_irq_cpu_down(unsigned int cpu, struct hlist_node *node)
{
	unsigned long irq_flags;
	struct trusty_irq_state *is;

	is = container_of(node, struct trusty_irq_state, cpuhp_node);
#ifdef IRQ_DEBUG_LOG_EN
	dev_dbg(is->dev, "%s: cpu %d\n", __func__, cpu);
#endif

	local_irq_save(irq_flags);
	trusty_irq_disable_irqset(is, this_cpu_ptr(is->percpu_irqs));
	local_irq_restore(irq_flags);

	return 0;
}

#if enable_code /*#if 0*/
static int trusty_irq_create_irq_mapping(struct trusty_irq_state *is, int irq)
{
	int ret;
	int index;
	u32 irq_pos;
	u32 templ_idx;
	u32 range_base;
	u32 range_end;
	struct of_phandle_args oirq;

	/* check if "interrupt-ranges" property is present */
	if (!of_find_property(is->dev->of_node, "interrupt-ranges", NULL)) {
		/* fallback to old behavior to be backward compatible with
		 * systems that do not need IRQ domains.
		 */
		return irq;
	}

	/* find irq range */
	for (index = 0;; index += 3) {
		ret = of_property_read_u32_index(is->dev->of_node,
						 "interrupt-ranges",
						 index, &range_base);
		if (ret)
			return ret;

		ret = of_property_read_u32_index(is->dev->of_node,
						 "interrupt-ranges",
						 index + 1, &range_end);
		if (ret)
			return ret;

		if (irq >= range_base && irq <= range_end)
			break;
	}

	/*  read the rest of range entry: template index and irq_pos */
	ret = of_property_read_u32_index(is->dev->of_node,
					 "interrupt-ranges",
					 index + 2, &templ_idx);
	if (ret)
		return ret;

	/* read irq template */
	ret = of_parse_phandle_with_args(is->dev->of_node,
					 "interrupt-templates",
					 "#interrupt-cells", templ_idx, &oirq);
	if (ret)
		return ret;

	WARN_ON(!oirq.np);
	WARN_ON(!oirq.args_count);

	/*
	 * An IRQ template is a non empty array of u32 values describing group
	 * of interrupts having common properties. The u32 entry with index
	 * zero contains the position of irq_id in interrupt specifier array
	 * followed by data representing interrupt specifier array with irq id
	 * field omitted, so to convert irq template to interrupt specifier
	 * array we have to move down one slot the first irq_pos entries and
	 * replace the resulting gap with real irq id.
	 */
	irq_pos = oirq.args[0];

	if (irq_pos >= oirq.args_count) {
		dev_info(is->dev, "irq pos is out of range: %d\n", irq_pos);
		return -EINVAL;
	}

	for (index = 1; index <= irq_pos; index++)
		oirq.args[index - 1] = oirq.args[index];

	oirq.args[irq_pos] = irq - range_base;

	ret = irq_create_of_mapping(&oirq);

	return (!ret) ? -EINVAL : ret;
}
#endif


static int map_native_irq(struct device_node *parent, unsigned int hwirq, bool percpu)
{
	struct irq_domain *domain = irq_find_host(parent);
	struct of_phandle_args spec = { .np = parent, .args_count = 3 };
	unsigned int irq;

	if (!domain)
		return -EPROBE_DEFER;
	irq = irq_find_mapping(domain, hwirq);
	if (irq)
		return irq;
	spec.args[0] = percpu ? GIC_PPI : GIC_SPI;
	spec.args[1] = hwirq - (percpu ? 16 : 32);
	spec.args[2] = IRQ_TYPE_LEVEL_HIGH;
	irq = irq_create_of_mapping(&spec);
	return irq ? irq : -EINVAL;
}

static int trusty_irq_init_normal_irq(struct trusty_irq_state *is, int tirq)
{
	struct trusty_irq *route;
	int irq, ret;

	if (tirq < 32 || tirq > 1019)
		return -EINVAL;
	irq = map_native_irq(is->spi_node, tirq, false);
	if (irq < 0)
		return irq;
	route = kzalloc(sizeof(*route), GFP_KERNEL);
	if (!route)
		return -ENOMEM;
	route->is = is;
	route->irq = irq;
	hlist_add_head(&route->node, &is->normal_irqs.inactive);
	/* Claim exclusively; leave masked until the complete instance is ready. */
	ret = request_irq(irq, trusty_irq_handler,
			  IRQF_NO_THREAD | IRQF_TRIGGER_HIGH | IRQF_NO_AUTOEN,
			  "trusty", route);
	if (ret) {
		hlist_del_init(&route->node);
		kfree(route);
		return ret;
	}
	is->normal_count++;
	return 0;
}

static int trusty_irq_init_per_cpu_irq(struct trusty_irq_state *is, int tirq)
{
	struct trusty_irq __percpu *data;
	unsigned int cpu;
	int irq, ret;

	/* The module has no registered SGI dispatch hook; never pretend it does. */
	if (tirq >= 0 && tirq < 16)
		return -EOPNOTSUPP;
	if (tirq < 16 || tirq >= 32)
		return -EINVAL;
	irq = map_native_irq(is->ppi_node, tirq, true);
	if (irq < 0)
		return irq;
	data = alloc_percpu(struct trusty_irq);
	if (!data)
		return -ENOMEM;
	for_each_possible_cpu(cpu) {
		struct trusty_irq *route = per_cpu_ptr(data, cpu);
		struct trusty_irq_irqset *set = per_cpu_ptr(is->percpu_irqs, cpu);

		route->is = is;
		route->irq = irq;
		route->percpu = true;
		route->enable = false;
		route->percpu_ptr = data;
		hlist_add_head(&route->node, &set->inactive);
	}
	ret = request_percpu_irq(irq, trusty_irq_handler, "trusty", data);
	if (ret) {
		for_each_possible_cpu(cpu)
			hlist_del_init(&per_cpu_ptr(data, cpu)->node);
		free_percpu(data);
		return ret;
	}
	is->percpu_count++;
	return 0;
}

static int trusty_smc_get_next_irq(struct trusty_irq_state *is,
				   unsigned long min_irq, bool per_cpu)
{
	return gz_trusty_fast_call32(is->trusty_dev,
			MTEE_SMCNR(SMCF_FC_GET_NEXT_IRQ, is->trusty_dev),
			min_irq, per_cpu, is->tee_id);
}

static int trusty_irq_enumerate(struct trusty_irq_state *is, bool percpu)
{
	unsigned int next = 0, count;
	int irq, ret;

	for (count = 0; count < 64; count++) {
		irq = trusty_smc_get_next_irq(is, next, percpu);
		if (irq == SM_ERR_END_OF_INPUT)
			return 0;
		if (irq < 0)
			return irq;
		if (irq < next || irq > 1019)
			return -EPROTO;
		ret = percpu ? trusty_irq_init_per_cpu_irq(is, irq) :
			      trusty_irq_init_normal_irq(is, irq);
		if (ret)
			return ret;
		next = irq + 1;
	}
	return -E2BIG;
}

static void trusty_irq_free_irqs(struct trusty_irq_state *is)
{
	struct trusty_irq *irq;
	struct hlist_node *n;
	unsigned int cpu;

	hlist_for_each_entry_safe(irq, n, &is->normal_irqs.inactive, node) {
#ifdef IRQ_DEBUG_LOG_EN
		dev_dbg(is->dev, "%s: irq %d\n", __func__, irq->irq);
#endif
		free_irq(irq->irq, irq);
		hlist_del(&irq->node);
		kfree(irq);
	}
	hlist_for_each_entry_safe(irq, n,
				  &per_cpu_ptr(is->percpu_irqs, cpumask_first(cpu_possible_mask))->inactive,
				  node) {
		struct trusty_irq __percpu *trusty_irq_handler_data;

#ifdef IRQ_DEBUG_LOG_EN
		dev_dbg(is->dev, "%s: percpu irq %d\n", __func__, irq->irq);
#endif
		trusty_irq_handler_data = irq->percpu_ptr;
		free_percpu_irq(irq->irq, trusty_irq_handler_data);
		for_each_possible_cpu(cpu) {
			struct trusty_irq *irq_tmp;

			irq_tmp = per_cpu_ptr(trusty_irq_handler_data, cpu);
			hlist_del(&irq_tmp->node);
		}
		free_percpu(trusty_irq_handler_data);
	}
}

static int init_irq_nodes(struct trusty_irq_state *is, struct device_node *node)
{
	is->spi_node = of_irq_find_parent(node);
	if (!is->spi_node)
		return -EINVAL;
	is->ppi_node = of_parse_phandle(node, "ppi-interrupt-parent", 0);
	if (!is->ppi_node)
		is->ppi_node = of_node_get(is->spi_node);
	if (!irq_find_host(is->spi_node) || !irq_find_host(is->ppi_node))
		return -EPROBE_DEFER;
	return 0;
}

static void trusty_irq_cleanup(struct trusty_irq_state *is)
{
	unsigned long flags;

	/* Stop all producers before any list, IRQ data or per-CPU storage is freed. */
	if (is->notifier_registered) {
		gz_trusty_call_notifier_unregister(is->trusty_dev,
						 &is->trusty_call_notifier);
		is->notifier_registered = false;
	}
	if (is->cpuhp_added) {
		cpuhp_state_remove_instance(trusty_irq_cpuhp_slot, &is->cpuhp_node);
		is->cpuhp_added = false;
	}
	spin_lock_irqsave(&is->normal_irqs_lock, flags);
	trusty_irq_disable_irqset(is, &is->normal_irqs);
	spin_unlock_irqrestore(&is->normal_irqs_lock, flags);
	if (is->percpu_irqs) {
		trusty_irq_free_irqs(is);
		free_percpu(is->percpu_irqs);
	}
	of_node_put(is->ppi_node);
	of_node_put(is->spi_node);
	kfree(is);
}

static ssize_t requested_routes_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct trusty_irq_state *is = dev_get_drvdata(dev);

	return sysfs_emit(buf, "per_cpu=%u shared=%u ready=%u\n",
		is->percpu_count, is->normal_count, is->cpuhp_added);
}
static DEVICE_ATTR_RO(requested_routes);
static struct attribute *trusty_irq_attrs[] = {
	&dev_attr_requested_routes.attr,
	NULL,
};
ATTRIBUTE_GROUPS(trusty_irq);

static int trusty_irq_probe(struct platform_device *pdev)
{
	struct trusty_irq_state *is;
	struct device_node *node = pdev->dev.of_node;
	struct trusty_irq *route;
	unsigned int cpu;
	u32 tee_id;
	unsigned long flags;
	int ret;

	if (!node || !pdev->dev.parent || !pdev->dev.parent->of_node)
		return -EINVAL;
	ret = of_property_read_u32(pdev->dev.parent->of_node, "tee-id", &tee_id);
	if (ret || !is_tee_id(tee_id))
		return -EINVAL;
	is = kzalloc(sizeof(*is), GFP_KERNEL);
	if (!is)
		return -ENOMEM;
	is->tee_id = tee_id;
	is->dev = &pdev->dev;
	is->trusty_dev = pdev->dev.parent;
	spin_lock_init(&is->normal_irqs_lock);
	INIT_HLIST_HEAD(&is->normal_irqs.pending);
	INIT_HLIST_HEAD(&is->normal_irqs.inactive);
	is->percpu_irqs = alloc_percpu(struct trusty_irq_irqset);
	if (!is->percpu_irqs) {
		ret = -ENOMEM;
		goto err;
	}
	for_each_possible_cpu(cpu) {
		struct trusty_irq_irqset *set = per_cpu_ptr(is->percpu_irqs, cpu);

		INIT_HLIST_HEAD(&set->pending);
		INIT_HLIST_HEAD(&set->inactive);
	}
	ret = init_irq_nodes(is, node);
	if (ret)
		goto err;
	ret = trusty_irq_enumerate(is, true);
	if (ret)
		goto err;
	ret = trusty_irq_enumerate(is, false);
	if (ret)
		goto err;
	is->trusty_call_notifier.notifier_call = trusty_irq_call_notify;
	ret = gz_trusty_call_notifier_register(is->trusty_dev, &is->trusty_call_notifier);
	if (ret)
		goto err;
	is->notifier_registered = true;
	ret = cpuhp_state_add_instance(trusty_irq_cpuhp_slot, &is->cpuhp_node);
	if (ret)
		goto err;
	is->cpuhp_added = true;
	spin_lock_irqsave(&is->normal_irqs_lock, flags);
	hlist_for_each_entry(route, &is->normal_irqs.inactive, node) {
		route->enable = true;
		enable_irq(route->irq);
	}
	spin_unlock_irqrestore(&is->normal_irqs_lock, flags);
	platform_set_drvdata(pdev, is);
	return 0;
err:
	trusty_irq_cleanup(is);
	return ret;
}

static void trusty_irq_remove(struct platform_device *pdev)
{
	struct trusty_irq_state *is = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	trusty_irq_cleanup(is);
}

/* for trusty */
static const struct of_device_id trusty_irq_of_match[] = {
	{ .compatible = "android,trusty-irq-v1", },
	{},
};

static struct platform_driver trusty_irq_driver = {
	.probe = trusty_irq_probe,
	.remove = trusty_irq_remove,
	.driver = {
		   .name = "trusty-irq",
		   .owner = THIS_MODULE,
		   .of_match_table = trusty_irq_of_match,
		   .dev_groups = trusty_irq_groups,
		   },
};

/* for nebula */
static const struct of_device_id nebula_irq_of_match[] = {
	{ .compatible = "android,nebula-irq-v1", },
	{},
};

static struct platform_driver nebula_irq_driver = {
	.probe = trusty_irq_probe,
	.remove = trusty_irq_remove,
	.driver = {
		   .name = "nebula-irq",
		   .owner = THIS_MODULE,
		   .of_match_table = nebula_irq_of_match,
		   .dev_groups = trusty_irq_groups,
		   },
};

static int __init trusty_irq_driver_init(void)
{
	int ret;

	/* allocate dynamic cpuhp state slot */
	ret = cpuhp_setup_state_multi(CPUHP_AP_ONLINE_DYN,
				      "trusty-irq:cpu:online",
				      trusty_irq_cpu_up, trusty_irq_cpu_down);
	if (ret < 0)
		return ret;
	trusty_irq_cpuhp_slot = ret;

	ret = platform_driver_register(&trusty_irq_driver);
	if (ret < 0)
		goto err_trusty_register;

	ret = platform_driver_register(&nebula_irq_driver);
	if (ret < 0)
		goto err_nebula_register;

	return ret;

err_nebula_register:
	platform_driver_unregister(&trusty_irq_driver);
err_trusty_register:
	/* undo cpuhp slot allocation */
	cpuhp_remove_multi_state(trusty_irq_cpuhp_slot);
	trusty_irq_cpuhp_slot = -1;

	return ret;
}

static void __exit trusty_irq_driver_exit(void)
{
	platform_driver_unregister(&nebula_irq_driver);
	platform_driver_unregister(&trusty_irq_driver);

	cpuhp_remove_multi_state(trusty_irq_cpuhp_slot);
	trusty_irq_cpuhp_slot = -1;
}

module_init(trusty_irq_driver_init);
module_exit(trusty_irq_driver_exit);
MODULE_LICENSE("GPL");

