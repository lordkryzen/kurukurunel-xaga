// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
 *
 * Author: Frederic Chen <frederic.chen@mediatek.com>
 *         Holmes Chiou <holmes.chiou@mediatek.com>
 *
 */
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include "mtk-ipesys-me-runtime.h"
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include "mtk_imgsys-engine.h"
#include "mtk-ipesys-me.h"
#ifdef TF_DUMP
#include "mtk_iommu_ext.h"
#include <dt-bindings/memory/mt8195-larb-port.h>
#endif
#include "iommu_debug.h"

#ifdef TF_DUMP_71_1
#include <dt-bindings/memory/mt6983-larb-port.h>
#elif defined(TF_DUMP_71_2)
#include <dt-bindings/memory/mt6879-larb-port.h>
#endif

struct clk_bulk_data imgsys_isp7_me_clks[] = {
	{ .id = "ME_CG_IPE" },
	{ .id = "ME_CG_IPE_TOP" },
	{ .id = "ME_CG" },
	{ .id = "ME_CG_LARB12" },
};

/* Original public/wire headers stay unchanged; ownership is private. */
struct ipesys_me_state {
	struct ipesys_me_device hw;
	struct clk_bulk_data clks[ARRAY_SIZE(imgsys_isp7_me_clks)];
	struct mtk_imgsys_dev *owner;
	struct device *client;
	bool powered;
	bool clocks_enabled;
};

static DEFINE_MUTEX(me_lock);
static DEFINE_SPINLOCK(me_state_lock);
static struct ipesys_me_state *me_state;

/* IRQ-safe: teardown unpublishes under this lock before freeing registers.
 * The conditional PM pin never resumes a suspended device from a fault IRQ.
 */
static void ipesys_me_dump_registers(unsigned int range)
{
	struct ipesys_me_state *state;
	struct device *client;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&me_state_lock, flags);
	state = me_state;
	if (!state || !state->powered)
		goto out;
	/* ME has no independent power-domain binding in xaga's DT. Its active
	 * flag alone cannot prove the imgsys parent domain is still powered.
	 */
	client = state->client;
	if (pm_runtime_get_if_active(client) <= 0)
		goto out;
	if (pm_runtime_get_if_active(state->hw.dev) <= 0) {
		pm_runtime_put_noidle(client);
		goto out;
	}
	for (i = ME_CTL_OFFSET; i <= ME_CTL_OFFSET + range; i += 0x10) {
		pr_info("%s: 0x%08X %08X, %08X, %08X, %08X", __func__,
			(unsigned int)(0x15320000 + i),
			(unsigned int)ioread32(state->hw.regs + i),
			(unsigned int)ioread32(state->hw.regs + i + 0x4),
			(unsigned int)ioread32(state->hw.regs + i + 0x8),
			(unsigned int)ioread32(state->hw.regs + i + 0xC));
	}
	pm_runtime_put_noidle(state->hw.dev);
	pm_runtime_put_noidle(client);
out:
	spin_unlock_irqrestore(&me_state_lock, flags);
}

#if defined(TF_DUMP_71_1) || defined(TF_DUMP_71_2)
int ME_TranslationFault_callback(int port, dma_addr_t mva, void *data)
{
	(void)port;
	(void)mva;
	(void)data;
	ipesys_me_dump_registers(ME_CTL_RANGE_TF);
	return 1;
}

static int ipesys_me_register_faults(void)
{
	int ret;

#if defined(TF_DUMP_71_1)
	ret = mtk_iommu_register_fault_callback(M4U_PORT_L12_IPE_ME_RDMA,
		ME_TranslationFault_callback, NULL, false);
	if (ret)
		return ret;
	ret = mtk_iommu_register_fault_callback(M4U_PORT_L12_IPE_ME_WDMA,
		ME_TranslationFault_callback, NULL, false);
	if (ret)
		mtk_iommu_unregister_fault_callback(M4U_PORT_L12_IPE_ME_RDMA, false);
#elif defined(TF_DUMP_71_2)
	ret = mtk_iommu_register_fault_callback(M4U_LARB12_PORT4,
		ME_TranslationFault_callback, NULL, false);
	if (ret)
		return ret;
	ret = mtk_iommu_register_fault_callback(M4U_LARB12_PORT5,
		ME_TranslationFault_callback, NULL, false);
	if (ret)
		mtk_iommu_unregister_fault_callback(M4U_LARB12_PORT4, false);
#endif
	return ret;
}

static void ipesys_me_unregister_faults(void)
{
#if defined(TF_DUMP_71_1)
	mtk_iommu_unregister_fault_callback(M4U_PORT_L12_IPE_ME_RDMA, false);
	mtk_iommu_unregister_fault_callback(M4U_PORT_L12_IPE_ME_WDMA, false);
#elif defined(TF_DUMP_71_2)
	mtk_iommu_unregister_fault_callback(M4U_LARB12_PORT4, false);
	mtk_iommu_unregister_fault_callback(M4U_LARB12_PORT5, false);
#endif
}
#else
static int ipesys_me_register_faults(void) { return 0; }
static void ipesys_me_unregister_faults(void) { }
#endif

/* me_lock excludes remove and other clients; publication/MMIO uses the IRQ lock. */
int ipesys_me_start(struct mtk_imgsys_dev *imgsys_dev)
{
	struct ipesys_me_state *state;
	unsigned long flags;
	int ret = 0;

	if (!imgsys_dev || !imgsys_dev->dev)
		return -EINVAL;
	mutex_lock(&me_lock);
	state = me_state;
	if (!state) {
		ret = -ENODEV;
		goto out;
	}
	if (state->owner) {
		ret = state->owner == imgsys_dev ? 0 : -EBUSY;
		goto out;
	}
	ret = pm_runtime_resume_and_get(state->hw.dev);
	if (ret < 0)
		goto out;
#ifdef ME_CLK_CTRL
	ret = clk_bulk_prepare_enable(state->hw.me_clk.clk_num, state->hw.me_clk.clks);
	if (ret) {
		pm_runtime_put_sync(state->hw.dev);
		goto out;
	}
	state->clocks_enabled = true;
#endif
	state->client = get_device(imgsys_dev->dev);
	state->owner = imgsys_dev;
	spin_lock_irqsave(&me_state_lock, flags);
	state->powered = true;
	spin_unlock_irqrestore(&me_state_lock, flags);
	ret = 0;
out:
	mutex_unlock(&me_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ipesys_me_start);

void ipesys_me_set_initial_value(struct mtk_imgsys_dev *imgsys_dev)
{
	int ret = ipesys_me_start(imgsys_dev);

	if (ret)
		dev_err(imgsys_dev->dev, "ME initialization failed: %d\n", ret);
}
EXPORT_SYMBOL(ipesys_me_set_initial_value);

static void ipesys_me_stop(struct ipesys_me_state *state)
{
	unsigned long flags;

	if (!state->owner)
		return;
	spin_lock_irqsave(&me_state_lock, flags);
	state->powered = false;
	spin_unlock_irqrestore(&me_state_lock, flags);
	if (state->clocks_enabled) {
		clk_bulk_disable_unprepare(state->hw.me_clk.clk_num, state->hw.me_clk.clks);
		state->clocks_enabled = false;
	}
	pm_runtime_put_sync(state->hw.dev);
	put_device(state->client);
	state->client = NULL;
	state->owner = NULL;
}

void ipesys_me_uninit(struct mtk_imgsys_dev *imgsys_dev)
{
	mutex_lock(&me_lock);
	if (me_state && me_state->owner == imgsys_dev)
		ipesys_me_stop(me_state);
	mutex_unlock(&me_lock);
}
EXPORT_SYMBOL(ipesys_me_uninit);

void ipesys_me_debug_dump(struct mtk_imgsys_dev *imgsys_dev,
			unsigned int engine)
{
	(void)imgsys_dev;
	(void)engine;
	ipesys_me_dump_registers(ME_CTL_RANGE);
}
EXPORT_SYMBOL(ipesys_me_debug_dump);

void ipesys_me_debug_dump_local(void)
{
	ipesys_me_dump_registers(ME_CTL_RANGE);
}
EXPORT_SYMBOL(ipesys_me_debug_dump_local);

/* Returns an owned device reference; a future caller must put_device().
 * The pin does not retain MMIO or driver data past unbind. There are no
 * existing users of this legacy, unexported getter in the port.
 */
struct device *ipesys_me_getdev(void)
{
	struct device *dev = NULL;
	unsigned long flags;

	spin_lock_irqsave(&me_state_lock, flags);
	if (me_state)
		dev = get_device(me_state->hw.dev);
	spin_unlock_irqrestore(&me_state_lock, flags);
	return dev;
}

static void ipesys_me_delete_link(void *data)
{
	device_link_del(data);
}

static int mtk_ipesys_me_probe(struct platform_device *pdev)
{
	struct ipesys_me_state *state;
	struct resource *resource;
	struct device_link *link;
	struct device_node *node;
	struct platform_device *larb;
	unsigned long flags;
	int count, i, ret;

	state = devm_kzalloc(&pdev->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;
	state->hw.dev = &pdev->dev;
	memcpy(state->clks, imgsys_isp7_me_clks, sizeof(state->clks));
	state->hw.me_clk.clks = state->clks;
	state->hw.me_clk.clk_num = ARRAY_SIZE(state->clks);
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource || resource_size(resource) < ME_CTL_OFFSET + ME_CTL_RANGE + 0x10)
		return -EINVAL;
	state->hw.regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(state->hw.regs))
		return PTR_ERR(state->hw.regs);
	ret = devm_clk_bulk_get(&pdev->dev, state->hw.me_clk.clk_num, state->clks);
	if (ret)
		return ret;

	/* The reference xaga DT omits this legacy property; IOMMU bindings remain. */
	count = of_count_phandle_with_args(pdev->dev.of_node, "mediatek,larb", NULL);
	if (count < 0 && count != -ENOENT)
		return count;
	for (i = 0; i < count; i++) {
		node = of_parse_phandle(pdev->dev.of_node, "mediatek,larb", i);
		if (!node)
			return -EINVAL;
		larb = of_find_device_by_node(node);
		of_node_put(node);
		if (!larb)
			return -EPROBE_DEFER;
		link = device_link_add(&pdev->dev, &larb->dev,
			DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
		put_device(&larb->dev);
		if (!link)
			return -ENOMEM;
		ret = devm_add_action_or_reset(&pdev->dev, ipesys_me_delete_link, link);
		if (ret)
			return ret;
	}

	mutex_lock(&me_lock);
	if (me_state) {
		ret = -EBUSY;
		goto out;
	}
	ret = ipesys_me_register_faults();
	if (ret)
		goto out;
	pm_runtime_enable(&pdev->dev);
	platform_set_drvdata(pdev, state);
	spin_lock_irqsave(&me_state_lock, flags);
	me_state = state;
	spin_unlock_irqrestore(&me_state_lock, flags);
out:
	mutex_unlock(&me_lock);
	return ret;
}

static void mtk_ipesys_me_remove(struct platform_device *pdev)
{
	struct ipesys_me_state *state = platform_get_drvdata(pdev);
	unsigned long flags;

	mutex_lock(&me_lock);
	spin_lock_irqsave(&me_state_lock, flags);
	if (me_state == state)
		me_state = NULL;
	spin_unlock_irqrestore(&me_state_lock, flags);
	/* Waits for fault callbacks before module code and devres can disappear. */
	ipesys_me_unregister_faults();
	ipesys_me_stop(state);
	pm_runtime_disable(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	mutex_unlock(&me_lock);
}

static int __maybe_unused mtk_ipesys_me_runtime_suspend(struct device *dev)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int __maybe_unused mtk_ipesys_me_runtime_resume(struct device *dev)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int __maybe_unused mtk_ipesys_me_pm_suspend(struct device *dev)
{
	pr_info("%s\n", __func__);
	return 0;
}

static int __maybe_unused mtk_ipesys_me_pm_resume(struct device *dev)
{
	pr_info("%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops mtk_ipesys_me_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(
		mtk_ipesys_me_pm_suspend,
		mtk_ipesys_me_pm_resume
	)
	SET_RUNTIME_PM_OPS(
		mtk_ipesys_me_runtime_suspend,
		mtk_ipesys_me_runtime_resume,
		NULL
	)
};

static const struct of_device_id mtk_ipesys_me_of_match[] = {
	{ .compatible = "mediatek,ipesys-me", },
	{}
};
MODULE_DEVICE_TABLE(of, mtk_ipesys_me_of_match);

static struct platform_driver mtk_ipesys_me_driver = {
	.probe   = mtk_ipesys_me_probe,
	.remove  = mtk_ipesys_me_remove,
	.driver  = {
		.name = "camera-me",
		.owner	= THIS_MODULE,
		.pm = &mtk_ipesys_me_pm_ops,
		.of_match_table = mtk_ipesys_me_of_match,
	}
};

module_platform_driver(mtk_ipesys_me_driver);

MODULE_AUTHOR("Marvin Lin <marvin.lin@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Mediatek ME driver");

