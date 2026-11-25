// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Kuan-Hsin Lee <Kuan-Hsin.Lee@mediatek.com>
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "clkbuf-api.h"
#include "clkbuf-util.h"
#include "clkbuf-ctrl.h"
#include "clkbuf-pmic.h"
#include "srclken.h"

static bool _inited;
static struct device *clkbuf_owner;
static DEFINE_MUTEX(clkbuf_api_lock);
struct device *clkbuf_owner_device(void) { return clkbuf_owner; }
void __iomem *clkbuf_devm_iomap(struct device_node *node, int index)
{
 struct resource res;
 if (!clkbuf_owner || of_address_to_resource(node, index, &res)) return NULL;
 return devm_ioremap(clkbuf_owner, res.start, resource_size(&res));
}
static void clkbuf_free_array(void *data)
{
 struct clkbuf_dts *a = data;
 int i;
 for (i=0; i<a->nums; i++) kfree(a[i].init_dts_cmd.xo_dts_cmd);
 kfree(a);
}


static struct clkbuf_dts *_array;
static void set_dts_array(struct clkbuf_dts *array)
{
	_array = array;
}
static struct clkbuf_dts *get_dts_array(void)
{
	return _array;
}

static struct clkbuf_dts *(*parse_sub_dts[])(struct clkbuf_dts *array,
					     struct device_node *clkbuf_node,
					     int nums) = {
	&parse_pmic_dts,
	&parse_srclken_dts,
	&parse_pmif_dts,
	NULL,
};

static int (*sub_init[])(struct clkbuf_dts *array, struct device *dev) = {
	&clkbuf_pmic_init,
	&clkbuf_srclken_init,
	&clkbuf_pmif_init,
	&clkbuf_platform_init,
#if IS_ENABLED(CONFIG_DEBUG_FS)
	&clkbuf_debug_init,
#endif
	NULL,
};

static const struct of_device_id clkbuf_of_match[] = {
	{ .compatible = "mediatek,mt6761-clkbuf" },
	{ .compatible = "mediatek,mt6765-clkbuf" },
	{ .compatible = "mediatek,mt6768-clkbuf" },
	{ .compatible = "mediatek,mt6789-clkbuf" },
	{ .compatible = "mediatek,mt6833-clkbuf" },
	{ .compatible = "mediatek,mt6855-clkbuf" },
	{ .compatible = "mediatek,mt6858-clkbuf" },
	{ .compatible = "mediatek,mt6877-clkbuf" },
	{ .compatible = "mediatek,mt6881-clkbuf" },
	{ .compatible = "mediatek,mt6885-clkbuf" },
	{ .compatible = "mediatek,mt6895-clkbuf" },
	{ .compatible = "mediatek,mt6897-clkbuf" },
	{ .compatible = "mediatek,mt6985-clkbuf" },
	{ .compatible = "mediatek,mt6989-clkbuf" },
	{ .compatible = "mediatek,mt6991-clkbuf" },
	{ .compatible = "mediatek,mt6993-clkbuf" },
	{ .compatible = "mediatek,mt8786-clkbuf" },
	{ .compatible = "mediatek,mt8792-clkbuf" },
	{ .compatible = "mediatek,mt8781-clkbuf" },
	{ .compatible = "mediatek,mt8910-clkbuf" },
	{}
};

static const char *const xo_api_cmd[] = {
	[SET_XO_MODE] = "SET_XO_MODE",
	[SET_XO_EN_M] = "SET_XO_EN_M",
	[SET_XO_IMPEDANCE] = "SET_XO_IMPEDANCE",
	[SET_XO_DESENSE] = "SET_XO_DESENSE",
	[SET_XO_VOTER] = "SET_XO_VOTER",
	[SET_XO_EXT_VOTER] = "SET_XO_EXT_VOTER",
};

static const char *const rc_api_cmd[] = {
	[RC_NONE_REQ] = "RC_NONE_REQ",
	[RC_FPM_REQ] = "RC_FPM_REQ",
	[RC_LPM_REQ] = "RC_LPM_REQ",
};

/* API usage example */
/* clkbuf_srclken_ctrl("RC_FPM_REQ", 6); */
/* clkbuf_srclken_ctrl("RC_FPM_REQ", 13); */
static int clkbuf_srclken_ctrl_inner(char *cmd, int sub_id)
{
	struct clkbuf_dts *array = get_dts_array();
	struct plat_rcdata *pd;
	struct clkbuf_hdlr *hdlr;
	struct clkbuf_hw hw;
	int ret = 0, nums = 0, i;
	unsigned int cmd_idx;
	int dts_api_perms = 0;

	if (!_inited || !array)
		return -ENODEV;

	nums = array->nums;

	for (cmd_idx = 1; cmd_idx < RC_MAX_REQ; cmd_idx <<=1) {
		if (!strcmp(rc_api_cmd[cmd_idx], cmd)) {
			/*loop all object---step1*/
			for (i = 0; i < nums; i++, array++) {
				hw = array->hw;
				hdlr = array->hdlr;
				/*filter RC object---step2*/
				if (IS_RC_HW(hw.hw_type) && (array->sub_id == sub_id)) {
					if (!hdlr->ops->srclken_subsys_ctrl) {
						CLKBUF_DBG("Error: srclken_subsys_ctrl is null\n");
						return ret;
					}
					dts_api_perms =	(array->perms & 0xffff0000) >> 16;
					pd = (struct plat_rcdata *)hdlr->data;
					ret = hdlr->ops->srclken_subsys_ctrl(
						pd, cmd_idx, array->sub_id,
						dts_api_perms);
					if (ret) {
						CLKBUF_DBG("Error: %d\n", ret);
						break;
					}
					return ret;
				}
			}
			CLKBUF_DBG("can not find RC HW and sub_id %d\n", sub_id);
			return -EINVAL;
		}
	}
	CLKBUF_DBG("can not find cmd %s\n", cmd);
	return -EINVAL;
}
int clkbuf_srclken_ctrl(char *cmd, int sub_id)
{
 int ret;
 mutex_lock(&clkbuf_api_lock);
 ret = clkbuf_srclken_ctrl_inner(cmd, sub_id);
 mutex_unlock(&clkbuf_api_lock);
 return ret;
}
EXPORT_SYMBOL(clkbuf_srclken_ctrl);

/* API usage example */
/* clkbuf_xo_ctrl("SET_XO_MODE", 12, 0); */
/* clkbuf_xo_ctrl("SET_XO_EN_M", 11, 1); */
static int clkbuf_xo_ctrl_inner(char *cmd, int xo_id, u32 input)
{
	struct clkbuf_dts *array = get_dts_array();
	struct clkbuf_hdlr *hdlr;
	struct plat_xodata *pd;
	struct clkbuf_hw hw;
	int dts_api_perms;
	int ret = 0, nums = 0, i;
	unsigned int cmd_idx;

	if (!_inited || !array)
		return -ENODEV;

	nums = array->nums;

	for (cmd_idx = 1; cmd_idx < MAX_XO_CMD; cmd_idx <<=1) {
		if (!strcmp(xo_api_cmd[cmd_idx], cmd)) {
			/*loop all object---step1*/
			for (i = 0; i < nums; i++, array++) {
				hw = array->hw;
				hdlr = array->hdlr;
				/*filter PMIC object---step2*/
				if ((IS_PMIC_HW(hw.hw_type)) && (array->xo_id == xo_id)) {
					if (!hdlr->ops->set_xo_cmd_hdlr) {
						CLKBUF_DBG("Error: set_xo_cmd_hdlr is null\n");
						return ret;
					}
					pd = (struct plat_xodata *)hdlr->data;
					dts_api_perms =	(array->perms & 0xffff0000) >> 16;
					ret = hdlr->ops->set_xo_cmd_hdlr(
						pd, cmd_idx, xo_id, input,
						dts_api_perms);
					if (ret) {
						CLKBUF_DBG("Error: %d\n", ret);
						break;
					}
					return ret;
				}
			}
			CLKBUF_DBG("can not find PMIC HW and sub_id %d\n", xo_id);
			return -EINVAL;
		}
	}
	return -EINVAL;
}
int clkbuf_xo_ctrl(char *cmd, int xo_id, u32 input)
{
 int ret;
 mutex_lock(&clkbuf_api_lock);
 ret = clkbuf_xo_ctrl_inner(cmd, xo_id, input);
 mutex_unlock(&clkbuf_api_lock);
 return ret;
}
EXPORT_SYMBOL(clkbuf_xo_ctrl);

static struct clkbuf_dts *parse_dt(struct platform_device *pdev)
{
	struct device_node *clkbuf_node;
	struct clkbuf_dts *array = NULL, *head;
	const struct of_device_id *match;
	int size = 0, nums = 0, i, ret, count;
	struct clkbuf_dts *(**parse_dts_call)(
		struct clkbuf_dts *, struct device_node *, int) = parse_sub_dts;

	clkbuf_node = pdev->dev.of_node;
	match = of_match_node(pdev->dev.driver->of_match_table, clkbuf_node);
	if (!match) {
		CLKBUF_ERR("CLKBUF probe fail\n");
		return ERR_PTR(-ENODEV);
	}

	count = count_pmic_node(clkbuf_node);
	if (count < 0 || count > 64) return ERR_PTR(-EINVAL);
	nums += count;

	count = count_src_node(clkbuf_node);
	if (count < 0 || count > 64) return ERR_PTR(-EINVAL);
	nums += count;

	count = count_pmif_node(clkbuf_node);
	if (count < 0 || count > 64) return ERR_PTR(-EINVAL);
	nums += count;

	size = sizeof(*array) * (nums + 1);
	array = kzalloc(size, GFP_KERNEL);

	if (!array) {
		CLKBUF_ERR("-ENOMEM\n");
		return ERR_PTR(-ENOMEM);
	}

	head = array;

	while (*parse_dts_call != NULL) {
		array = (*parse_dts_call)(array, clkbuf_node, nums);
		if (IS_ERR_OR_NULL(array)) {
			ret = array ? PTR_ERR(array) : -EINVAL;
			for (i = 0; i < nums; i++)
				kfree(head[i].init_dts_cmd.xo_dts_cmd);
			kfree(head);
			return ERR_PTR(ret);
		}
		parse_dts_call++;
	}

	if (array != head + nums) {
		for (i = 0; i < nums; i++) kfree(head[i].init_dts_cmd.xo_dts_cmd);
		kfree(head);
		return ERR_PTR(-EINVAL);
	}
	set_dts_array(head);

	return head;
}

static int clkbuf_probe_inner(struct platform_device *pdev)
{
	struct clkbuf_dts *array;
	struct device *dev = &pdev->dev;
	int nums = 0, i, ret;

	int (**init_call)(struct clkbuf_dts *, struct device *) = sub_init;

	CLKBUF_DBG("\n");

	array = parse_dt(pdev);
	if (IS_ERR(array))
		return PTR_ERR(array);
	ret = devm_add_action_or_reset(dev, clkbuf_free_array, array);
	if (ret) { set_dts_array(NULL); return ret; }

	while (*init_call != NULL) {
		ret = (*init_call)(array, dev);
		if (ret) {
			set_dts_array(NULL);
			return ret;
		}
		init_call++;
	}

	nums = array->nums;
	//make sure array is complete
	for (i = 0; i < nums; i++, array++) {
		struct clkbuf_hdlr *hdlr = array->hdlr;

		if (!hdlr) {
			CLKBUF_DBG("array<%lx>, index: %d, type:%d, nums:%d, perms:<%x>\n",
				(unsigned long)array, i, array->hw.hw_type, array->nums, array->perms);

			return -EINVAL;
		}
	}
	/* set _inited is the last step */
	mb();
	_inited = true;

	return 0;
}

static int clkbuf_probe(struct platform_device *pdev)
{
 int ret;
 /* This port retains the native MT6895 provider and register contract. */
 if (!of_device_is_compatible(pdev->dev.of_node, "mediatek,mt6895-clkbuf")) return -ENODEV;
 mutex_lock(&clkbuf_api_lock);
 if (clkbuf_owner) { mutex_unlock(&clkbuf_api_lock); return -EBUSY; }
 clkbuf_owner = &pdev->dev;
 ret = clkbuf_probe_inner(pdev);
 if (ret) { _inited = false; set_dts_array(NULL); clkbuf_owner = NULL; }
 else __module_get(THIS_MODULE); /* exported hardware callbacks: reboot to unload */
 mutex_unlock(&clkbuf_api_lock);
 return ret;
}
static void clkbuf_remove(struct platform_device *pdev)
{
 mutex_lock(&clkbuf_api_lock);
 _inited = false;
 mutex_unlock(&clkbuf_api_lock);
 clkbuf_debug_exit(); /* waits for active sysfs readers before devres frees maps */
 mutex_lock(&clkbuf_api_lock);
 set_dts_array(NULL);
 clkbuf_owner = NULL;
 mutex_unlock(&clkbuf_api_lock);
}

#define DUMP_DONE 1
#define WAIT_TO_DUMP 0
static int dump_all(struct device *dev)
{
	struct clkbuf_dts *array = get_dts_array();
	struct clkbuf_hdlr *hdlr;
	struct clkbuf_hw hw;
	struct plat_xodata *xopd;
	struct plat_rcdata *rcpd;
	int pmic_dump = WAIT_TO_DUMP, rc_dump = WAIT_TO_DUMP, pmif_dump = WAIT_TO_DUMP;
	int nums, i;
	char *buf = NULL;

	buf = vmalloc(CLKBUF_STATUS_INFO_SIZE);
	if (!buf)
		return -ENOMEM;

	if (!array) {
		vfree(buf);
		return -ENODEV;
	}

	nums = array->nums;

	for (i = 0; i < nums; i++, array++) {
		hdlr = array->hdlr;
		hw = array->hw;

		switch (hw.hw_type) {
		case PMIC:
			if ((pmic_dump == DUMP_DONE) | !(hdlr->ops->get_pmrcen))
				break;
			xopd = (struct plat_xodata *)hdlr->data;
			hdlr->ops->get_pmrcen(xopd, buf, 0);
			CLKBUF_DBG("%s\n", buf);
			pmic_dump = DUMP_DONE;
			break;
		case SRCLKEN_STA:
		case SRCLKEN_CFG:
			if ((rc_dump == DUMP_DONE) | !(hdlr->ops->dump_srclken_trace))
				break;
			rcpd = (struct plat_rcdata *)hdlr->data;
			hdlr->ops->dump_srclken_trace(rcpd, buf, 0);
			CLKBUF_DBG("%s\n", buf);
			rc_dump = DUMP_DONE;
			break;
		case PMIF_M:
		case PMIF_P:
			if ((pmif_dump == DUMP_DONE) | !(hdlr->ops->spmi_dump_pmif_record))
				break;
			hdlr->ops->spmi_dump_pmif_record();
			pmif_dump = DUMP_DONE;
			break;
		default:
			break;
		}
	}

	vfree(buf);

	return 0;
}

static int clk_buf_dev_pm_suspend(struct device *dev)
{
	return dump_all(dev);
}

static int clk_buf_dev_pm_resume(struct device *dev)
{
	return dump_all(dev);
}

const struct dev_pm_ops clk_buf_suspend_ops = {
	.suspend_noirq = clk_buf_dev_pm_suspend,
	.resume_noirq = clk_buf_dev_pm_resume,
};

static struct platform_driver clkbuf_driver = {
	.probe = clkbuf_probe,
	.remove = clkbuf_remove,
	.driver = {
			.name = "clkbuf",
			.suppress_bind_attrs = true,
			.owner = THIS_MODULE,
			.of_match_table = clkbuf_of_match,
			.pm = &clk_buf_suspend_ops,
	},
};
/* The native DT names the MFD clock-buffer child as a supplier.  Publish
 * its real parent regmap before registering the consuming controller. */
static int clkbuf_pmic_provider_probe(struct platform_device *pdev)
{
 struct regmap *map;
 if (!pdev->dev.parent) return -EPROBE_DEFER;
 map = dev_get_regmap(pdev->dev.parent, NULL);
 if (IS_ERR(map)) return PTR_ERR(map);
 if (!map) return -EPROBE_DEFER;
 platform_set_drvdata(pdev, map);
 return 0;
}
static const struct of_device_id clkbuf_pmic_provider_match[] = {
 { .compatible = "mediatek,mt6685-clkbuf" }, {}
};
MODULE_DEVICE_TABLE(of, clkbuf_pmic_provider_match);
static struct platform_driver clkbuf_pmic_provider_driver = {
 .probe = clkbuf_pmic_provider_probe,
 .driver = { .name = "clkbuf-pmic-provider",
             .of_match_table = clkbuf_pmic_provider_match,
             .suppress_bind_attrs = true },
};
static int __init clkbuf_init(void)
{
 int ret = platform_driver_register(&clkbuf_pmic_provider_driver);
 if (ret) return ret;
 ret = platform_driver_register(&clkbuf_driver);
 if (ret) platform_driver_unregister(&clkbuf_pmic_provider_driver);
 return ret;
}
static void __exit clkbuf_exit(void)
{
 platform_driver_unregister(&clkbuf_driver);
 platform_driver_unregister(&clkbuf_pmic_provider_driver);
}
module_init(clkbuf_init);
module_exit(clkbuf_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek CLKBUF CTRL Driver");
MODULE_AUTHOR("Kuan-Hsin Lee <kuan-hsin.lee@mediatek.com>");
MODULE_SOFTDEP("pre: mt6685-core");
MODULE_SOFTDEP("pre: mt6687-core");
MODULE_SOFTDEP("pre: mt6397");
