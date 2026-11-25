// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

extern struct provider_clk *get_all_provider_clks(bool is_internal);

struct clk_bringup_state {
	struct clk_bulk_data *clks;
	int count;
};

static void clk_bringup_disable(void *data)
{
	struct clk_bringup_state *state = data;

	clk_bulk_disable_unprepare(state->count, state->clks);
}

static int __bring_up_enable(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_bringup_state *state;
	int ret;

	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->count = devm_clk_bulk_get_all(dev, &state->clks);
	if (state->count < 0)
		return dev_err_probe(dev, state->count, "Failed to get bring-up clocks\n");

	ret = clk_bulk_prepare_enable(state->count, state->clks);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable bring-up clocks\n");

	ret = devm_add_action_or_reset(dev, clk_bringup_disable, state);
	if (ret)
		return ret;

	/* Populate the original diagnostic provider cache after clocks are ready. */
	get_all_provider_clks(true);
	return 0;
}

static int clk_bring_up_probe(struct platform_device *pdev)
{
	return __bring_up_enable(pdev);
}

static int clk_post_ao_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	u32 enabled = 0;

	of_property_read_u32(node, "mediatek,post_ao", &enabled);

	if (enabled != 1) {
		pr_notice("btypass_clk_post_ao\n");
		return 0;
	}

	return __bring_up_enable(pdev);
}

static const struct of_device_id bring_up_id_table[] = {
	{
		.compatible = "mediatek,clk-bring-up",
		.data = clk_bring_up_probe,
	}, {
		.compatible = "mediatek,clk-post-ao",
		.data = clk_post_ao_probe,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, bring_up_id_table);

static int bring_up_probe(struct platform_device *pdev)
{
	int (*clk_probe)(struct platform_device *pd);
	int r;

	clk_probe = of_device_get_match_data(&pdev->dev);
	if (!clk_probe)
		return -EINVAL;

	r = clk_probe(pdev);
	if (r)
		dev_err(&pdev->dev,
			"could not register clock provider: %s: %d\n",
			pdev->name, r);

	return r;
}

static struct platform_driver bring_up = {
	.probe		= bring_up_probe,
	.driver		= {
		.name	= "bring_up",
		.owner	= THIS_MODULE,
		.of_match_table = bring_up_id_table,
	},
};

module_platform_driver(bring_up);
MODULE_LICENSE("GPL");
