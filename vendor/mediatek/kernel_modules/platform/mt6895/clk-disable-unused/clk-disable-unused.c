// SPDX-License-Identifier: GPL-2.0
/* Native unused-clock cleanup with explicit ownership and PM error handling. */
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

static bool dry_run;
module_param(dry_run, bool, 0444);
MODULE_PARM_DESC(dry_run, "Resolve native DT clock contracts without powering domains or changing gates");
static char *target;
module_param(target, charp, 0444);
MODULE_PARM_DESC(target, "Optional exact DT node name for a bounded RAM test");
static unsigned int completed_devices, clocks_seen, clocks_processed, clocks_changed;
module_param(completed_devices, uint, 0444);
module_param(clocks_seen, uint, 0444);
module_param(clocks_processed, uint, 0444);
module_param(clocks_changed, uint, 0444);

struct cleanup_state {
	bool pm_enabled;
};

static int cleanup_one_clock(struct clk *clk, bool *changed)
{
	struct clk *parent = clk_get_parent(clk);
	int ret, before, after;

	*changed = false;
	if (IS_ERR(parent))
		return PTR_ERR(parent);
	/* clk_get_parent() returns a borrowed handle in this CCF; no clk_put.
	 * Hold the parent while accessing a potentially power-gated child.
	 */
	ret = clk_prepare_enable(parent);
	if (ret)
		return ret;
	before = __clk_is_enabled(clk);
	ret = clk_prepare_enable(clk);
	if (!ret) {
		/* CCF retains any other consumer's prepare/enable references. */
		clk_disable_unprepare(clk);
		after = __clk_is_enabled(clk);
		*changed = before != after;
	}
	clk_disable_unprepare(parent);
	return ret;
}

static int disable_unused_probe_impl(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cleanup_state *state;
	struct clk **clocks;
	int count, i, acquired = 0, ret = 0, pmret;
	bool changed;

	if (target && strcmp(dev->of_node->name, target))
		return -ENODEV;
	count = of_count_phandle_with_args(dev->of_node, "clocks", "#clock-cells");
	if (count < 0)
		return count;
	if (!count)
		return 0;
	if (count > 4096)
		return -E2BIG;
	clocks = kcalloc(count, sizeof(*clocks), GFP_KERNEL);
	if (!clocks)
		return -ENOMEM;
	/* Resolve the entire cohort before any PM or gate operation. Missing
	 * providers must defer this device, never be silently skipped.
	 */
	for (i = 0; i < count; i++) {
		clocks[i] = of_clk_get(dev->of_node, i);
		if (IS_ERR_OR_NULL(clocks[i])) {
			ret = clocks[i] ? PTR_ERR(clocks[i]) : -ENOENT;
			goto put_clocks;
		}
		acquired++;
	}
	clocks_seen += count;
	if (dry_run) {
		completed_devices++;
		goto put_clocks;
	}
	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state) {
		ret = -ENOMEM;
		goto put_clocks;
	}
	if (!pm_runtime_enabled(dev)) {
		pm_runtime_enable(dev);
		state->pm_enabled = true;
	}
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		goto disable_pm;
	ret = 0;
	for (i = 0; i < count; i++) {
		ret = cleanup_one_clock(clocks[i], &changed);
		if (ret)
			break;
		clocks_processed++;
		if (changed) {
			clocks_changed++;
			dev_info(dev, "disabled unused clock %s\n", __clk_get_name(clocks[i]));
		}
	}
	pmret = pm_runtime_put_sync(dev);
	if (!ret && pmret < 0)
		ret = pmret;
	if (ret)
		goto disable_pm;
	platform_set_drvdata(pdev, state);
	completed_devices++;
	goto put_clocks;

disable_pm:
	if (state->pm_enabled)
		pm_runtime_disable(dev);
put_clocks:
	while (acquired)
		clk_put(clocks[--acquired]);
	kfree(clocks);
	return ret;
}

static DEFINE_MUTEX(cleanup_lock);
static int disable_unused_probe(struct platform_device *pdev)
{
	int ret;

	mutex_lock(&cleanup_lock);
	ret = disable_unused_probe_impl(pdev);
	mutex_unlock(&cleanup_lock);
	return ret;
}

static void disable_unused_remove(struct platform_device *pdev)
{
	struct cleanup_state *state = platform_get_drvdata(pdev);

	if (state && state->pm_enabled)
		pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id disable_unused_id_table[] = {
	{ .compatible = "mediatek,clk-disable-unused" },
	{ }
};
MODULE_DEVICE_TABLE(of, disable_unused_id_table);
static struct platform_driver disable_unused = {
	.probe = disable_unused_probe,
	.remove_new = disable_unused_remove,
	.driver = {
		.name = "disable_unused",
		.of_match_table = disable_unused_id_table,
	},
};
module_platform_driver(disable_unused);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Native DT unused-clock gate cleanup");
