// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Samuel Hsieh <samuel.hsieh@mediatek.com>
 */
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include "mtk_ccci_common.h"
#include "mtk_md_power_throttling.h"
#include "mtk_battery_oc_throttling.h"
#include "mtk_low_battery_throttling.h"
#include "mtk_bp_thl.h"
#include "pmic_lbat_service.h"

#define MAX_INT 0x7FFF
#define MD_TX_REDUCE 6
#define MAX_VALUE 0x7FFF

static int *temp_thresholds;
static int temp_stage_size;
static bool md_pt_published;

struct md_pt_priv {
	char max_lv_name[32];
	char limit_name[32];
	u32 max_lv;
	u32 cur_temp_stage;
	u32 cur_thl_lv;
	u32 *reduce_tx;
	u32 *threshold;
	struct work_struct psy_work;
	struct power_supply *psy;
};

static struct md_pt_priv md_pt_info[POWER_THROTTLING_TYPE_MAX] = {
	[LBAT_POWER_THROTTLING] = {
		.max_lv_name = "lbat-max-level",
		.limit_name = "lbat-reduce-tx-lv",
		.max_lv = LOW_BATTERY_LEVEL_NUM - 1,
	},
	[OC_POWER_THROTTLING] = {
		.max_lv_name = "oc-max-level",
		.limit_name = "oc-reduce-tx-lv",
		.max_lv = BATTERY_OC_LEVEL_NUM - 1,
	}
};

static int get_temp_thresholds(struct device *dev)
{
 struct device_node *node = dev->of_node, *owned = NULL;
 const char *property = "lbat-dedicate-temperature";
 int count, ret;
 count = of_property_count_u32_elems(node, property);
 if (count <= 0) {
  owned = of_find_node_by_name(NULL, "low-battery-throttling");
  if (!owned) return -ENODEV;
  node = owned; property = "temperature-stage-threshold";
  count = of_property_count_u32_elems(node, property);
 }
 if (count < 0 || count > 15) { ret = -EINVAL; goto out; }
 if (!count) { ret = 0; goto out; }
 temp_thresholds = kcalloc(count, sizeof(*temp_thresholds), GFP_KERNEL);
 if (!temp_thresholds) { ret = -ENOMEM; goto out; }
 ret = of_property_read_u32_array(node, property, temp_thresholds, count);
 if (ret) { kfree(temp_thresholds); temp_thresholds = NULL; goto out; }
 temp_stage_size = count;
out:
 of_node_put(owned);
 return ret;
}

static struct notifier_block md_pt_nb;
static DEFINE_MUTEX(md_temp_lock);
static DEFINE_MUTEX(md_thl_lock);

#if IS_ENABLED(CONFIG_MTK_LOW_BATTERY_POWER_THROTTLING)
static void md_pt_low_battery_cb(enum LOW_BATTERY_LEVEL_TAG level, void *data)
{
	unsigned int md_throttle_cmd;
	int ret, intensity;
	struct md_pt_priv *priv;

	priv = &md_pt_info[LBAT_POWER_THROTTLING];
	if (level > md_pt_info[LBAT_POWER_THROTTLING].max_lv)
		return;
	if (priv->cur_thl_lv != level) {
		mutex_lock(&md_thl_lock);
		priv->cur_thl_lv = level;
		mutex_unlock(&md_thl_lock);
	}
	if (level != LOW_BATTERY_LEVEL_0)
		intensity = md_pt_info[LBAT_POWER_THROTTLING].reduce_tx[level-1];
	else
		intensity = 0;
	md_throttle_cmd = TMC_CTRL_CMD_TX_POWER | level << 8 | priv->cur_temp_stage << 12 |
		PT_LOW_BATTERY_VOLTAGE_WITH_TEMP << 16 | intensity << 24;
	ret = exec_ccci_kern_func(ID_THROTTLING_CFG,
		(char *)&md_throttle_cmd, 4);

	pr_notice("%s: send cmd to CCCI ret=%d, cmd=0x%x\n", __func__, ret,
					md_throttle_cmd);

	if (ret)
		pr_notice("%s: error, ret=%d, cmd=0x%x l=%d\n", __func__, ret,
			md_throttle_cmd, level);
}

static void md_lbat_dedicate_callback(unsigned int thd)
{
	int i = 0, lv = -1;
	struct md_pt_priv *priv;

	priv = &md_pt_info[LBAT_POWER_THROTTLING];

	for (i = 0; i <= priv->max_lv; i++) {
		if (thd == priv->threshold[i]) {
			lv = i;
			break;
		}
	}

	if (lv >= 0)
		md_pt_low_battery_cb(lv, NULL);
}

#endif

#if IS_ENABLED(CONFIG_MTK_BATTERY_PERCENT_THROTTLING)
static void md_bp_cb(enum BATTERY_PERCENT_LEVEL_TAG bp_level)
{
	unsigned int md_throttle_cmd;
	int ret, val = 0;

	if (bp_level == 0)
		val = TMC_CTRL_LOW_POWER_RECHARGE_BATTERY_EVENT;
	else
		val = TMC_CTRL_LOW_POWER_LOW_BATTERY_EVENT;

	md_throttle_cmd = TMC_CTRL_CMD_LOW_POWER_IND | val << 8;

	ret = exec_ccci_kern_func(ID_THROTTLING_CFG, (char *)&md_throttle_cmd, 4);

	pr_notice("%s: send cmd to CCCI ret=%d, cmd=0x%x\n", __func__, ret, md_throttle_cmd);
}

static void md_bp_uisoc_chg_cb(unsigned int chg_state, unsigned int soc)
{
	unsigned int md_throttle_cmd;
	int ret;

	md_throttle_cmd = TMC_CTRL_CMD_LOW_POWER_IND | soc << 8 | chg_state << 16;
	ret = exec_ccci_kern_func(ID_THROTTLING_CFG, (char *)&md_throttle_cmd, 4);
	if (ret) {
		pr_info("%s: error, ret=%d, cmd=0x%x\n", __func__, ret, md_throttle_cmd);
		return;
	}
	pr_info("[%s] notify md soc:%d, chg state:%d\n", __func__, soc, chg_state);
	pr_info("[%s] send cmd to CCCI ret=%d, cmd=0x%x\n", __func__, ret, md_throttle_cmd);
}
#endif

#if IS_ENABLED(CONFIG_MTK_BATTERY_OC_POWER_THROTTLING)
static void md_pt_over_current_cb(enum BATTERY_OC_LEVEL_TAG level, void *data)
{
	unsigned int md_throttle_cmd;
	int ret, intensity;

	if (level > md_pt_info[OC_POWER_THROTTLING].max_lv)
		return;
	if (level < BATTERY_OC_LEVEL_NUM) {
		if (level != BATTERY_OC_LEVEL_0)
			intensity = md_pt_info[OC_POWER_THROTTLING].reduce_tx[level-1];
		else
			intensity = 0;
		md_throttle_cmd = TMC_CTRL_CMD_TX_POWER | level << 8 | PT_OVER_CURRENT << 16 |
			intensity << 24;
		ret = exec_ccci_kern_func(ID_THROTTLING_CFG,
			(char *)&md_throttle_cmd, 4);

		pr_notice("%s: send cmd to CCCI ret=%d, cmd=0x%x\n", __func__, ret,
						md_throttle_cmd);

		if (ret)
			pr_notice("%s: error, ret=%d, cmd=0x%x l=%d\n", __func__, ret,
				md_throttle_cmd, level);
	}
}
#endif
static void md_pt_psy_handler(struct work_struct *work)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret, temp, temp_thd, temp_stage;
	static int last_temp = MAX_INT;
	struct md_pt_priv *priv;
	bool loop;

	priv = &md_pt_info[LBAT_POWER_THROTTLING];

	if (!priv->psy)
		return;

	psy = priv->psy;

	if (strcmp(psy->desc->name, "battery") != 0)
		return;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret)
		return;

	temp = val.intval / 10;
	temp_stage = priv->cur_temp_stage;

	do {
		loop = false;
		if (temp < last_temp) {
			if (temp_stage < temp_stage_size) {
				temp_thd = temp_thresholds[temp_stage];
				if (temp < temp_thd) {
					temp_stage++;
					loop = true;
				}
			}
		} else if (temp > last_temp) {
			if (temp_stage > 0) {
				temp_thd = temp_thresholds[temp_stage-1];
				if (temp >= temp_thd) {
					temp_stage--;
					loop = true;
				}
			}
		}
	} while (loop);

	last_temp = temp;

	if (temp_stage <= temp_stage_size && temp_stage != priv->cur_temp_stage) {
		mutex_lock(&md_temp_lock);
		priv->cur_temp_stage = temp_stage;
		mutex_unlock(&md_temp_lock);
		md_pt_low_battery_cb(priv->cur_thl_lv, NULL);
	}
}

static int md_pt_psy_event(struct notifier_block *nb, unsigned long event, void *v)
{
	struct md_pt_priv *priv;

	priv = &md_pt_info[LBAT_POWER_THROTTLING];
	if (v == priv->psy)
		schedule_work(&priv->psy_work);
	return NOTIFY_DONE;
}

static int __used md_limit_default_setting(struct device *dev, enum md_pt_type type)
{
	struct device_node *np = dev->of_node;
	int i, max_lv, ret;
	struct md_pt_priv *pt_info_p;

	pt_info_p = &md_pt_info[type];
	if (type == LBAT_POWER_THROTTLING)
		max_lv = LOW_BATTERY_LEVEL_NUM - 1;
	else if (type == OC_POWER_THROTTLING)
		max_lv = 1;
	else
		max_lv = 0;
	pt_info_p->max_lv = max_lv;
	if (!pt_info_p->max_lv)
		return 0;

	pt_info_p->reduce_tx = kcalloc(pt_info_p->max_lv, sizeof(u32), GFP_KERNEL);
	if (!pt_info_p->reduce_tx) {
		pt_info_p->max_lv = 0;
		return -ENOMEM;
	}

	pt_info_p->reduce_tx[0] = MD_TX_REDUCE;
	if (type == LBAT_POWER_THROTTLING) {
		ret = of_property_read_u32(np, "lbat_md_reduce_tx", &pt_info_p->reduce_tx[0]);
		if (ret < 0)
			pr_notice("%s: get lbat cpu limit fail %d\n", __func__, ret);
	} else if (type == OC_POWER_THROTTLING) {
		ret = of_property_read_u32(np, "oc_md_reduce_tx", &pt_info_p->reduce_tx[0]);
		if (ret < 0)
			pr_notice("%s: get oc cpu limit fail %d\n", __func__, ret);
	}
	for (i = 1; i < pt_info_p->max_lv; i++)
		memcpy(&pt_info_p->reduce_tx[i], &pt_info_p->reduce_tx[0], sizeof(u32));

	return 0;
}
static int __used parse_md_limit_table(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int i, j, num, ret;
	struct md_pt_priv *pt_info_p;
	char buf[32];

	for (i = 0; i < POWER_THROTTLING_TYPE_MAX; i++) {
		pt_info_p = &md_pt_info[i];
		ret = of_property_read_u32(np, pt_info_p->max_lv_name, &num);
		if (ret < 0) {
			ret = md_limit_default_setting(dev, i);
			if (ret)
				return ret;
			continue;
		} else if (num <= 0 || num > pt_info_p->max_lv) {
			pt_info_p->max_lv = 0;
			continue;
		}
		pt_info_p->max_lv = num;
		pt_info_p->reduce_tx = kcalloc(pt_info_p->max_lv, sizeof(u32), GFP_KERNEL);
		if (!pt_info_p->reduce_tx)
			return -ENOMEM;
		for (j = 0; j < pt_info_p->max_lv; j++) {
			memset(buf, 0, sizeof(buf));
			ret = snprintf(buf, sizeof(buf), "%s%d", pt_info_p->limit_name, j+1);
			if (ret < 0)
				pr_notice("can't merge %s %d\n", pt_info_p->limit_name, j+1);
			ret = of_property_read_u32(np, buf, &pt_info_p->reduce_tx[j]);
			if (ret < 0) return ret;
			if (pt_info_p->reduce_tx[j] > 255) return -ERANGE;
		}
		if (i == LBAT_POWER_THROTTLING) {
			num = of_property_count_elems_of_size(np, "lbat-dedicate-volts",
				sizeof(u32));
			if (num != pt_info_p->max_lv + 1) {
				pr_info("lbat-dedicate-volts num error: volt array=%d, max_lv=%d\n",
					num, pt_info_p->max_lv);
				continue;
			}

			pt_info_p->threshold = kcalloc(num, sizeof(u32), GFP_KERNEL);
			if (!pt_info_p->threshold) {
				pt_info_p->max_lv = 0;
				kfree(pt_info_p->reduce_tx);
				pt_info_p->reduce_tx = NULL;
				return -ENOMEM;
			}

			ret = of_property_read_u32_array(np, "lbat-dedicate-volts",
				pt_info_p->threshold, num);
			if (ret) {
				pr_notice("get lbat-dedicate-volts error %d\n", ret);
				pt_info_p->max_lv = 0;
				kfree(pt_info_p->reduce_tx);
				pt_info_p->reduce_tx = NULL;
				kfree(pt_info_p->threshold);
				pt_info_p->threshold = NULL;
				continue;
			}
		}
	}

	return 0;
}
static void md_pt_free_tables(void)
{
 unsigned int i;
 for (i = 0; i < POWER_THROTTLING_TYPE_MAX; i++) {
  kfree(md_pt_info[i].reduce_tx); md_pt_info[i].reduce_tx = NULL;
  kfree(md_pt_info[i].threshold); md_pt_info[i].threshold = NULL;
 }
 kfree(temp_thresholds); temp_thresholds = NULL; temp_stage_size = 0;
}

static int mtk_md_power_throttling_probe(struct platform_device *pdev)
{
 struct md_pt_priv saved[POWER_THROTTLING_TYPE_MAX];
 struct md_pt_priv *priv = &md_pt_info[LBAT_POWER_THROTTLING];
 struct lbat_user *lbat;
 bool notifier_registered = false;
 int ret;
 if (md_pt_published) return -EBUSY;
 memcpy(saved, md_pt_info, sizeof(saved));
 INIT_WORK(&priv->psy_work, md_pt_psy_handler);
 ret = parse_md_limit_table(&pdev->dev);
 if (ret) goto fail;
 ret = get_temp_thresholds(&pdev->dev);
 if (ret == -ENODEV) ret = 0; /* Original optional temperature-stage input. */
 if (ret) goto fail;
 if (temp_stage_size > 0) {
  priv->psy = power_supply_get_by_name("battery");
  if (!priv->psy) { ret = -EPROBE_DEFER; goto fail; }
  md_pt_nb.notifier_call = md_pt_psy_event;
  ret = power_supply_reg_notifier(&md_pt_nb);
  if (ret) goto fail;
  notifier_registered = true;
 }
 /* The PMIC API returns errors before publication. Register it as the last
  * fallible step; after success both its callback and its tables are permanent. */
 if (priv->max_lv > 0 && priv->threshold) {
  lbat = lbat_user_register_ext("md pa dedicate throttle", priv->threshold,
                              priv->max_lv + 1, md_lbat_dedicate_callback);
  if (IS_ERR(lbat)) { ret = PTR_ERR(lbat); goto fail; }
 }
 /* Callback registries cannot safely unregister. No failures after this point. */
 __module_get(THIS_MODULE);
 md_pt_published = true;
 if (priv->max_lv > 0 && !priv->threshold)
  register_low_battery_notify(md_pt_low_battery_cb, LOW_BATTERY_PRIO_MD, NULL);
 if (md_pt_info[OC_POWER_THROTTLING].max_lv > 0)
  register_battery_oc_notify(md_pt_over_current_cb, BATTERY_OC_PRIO_MD, NULL);
 register_bp_thl_md_notify(md_bp_cb, BATTERY_PERCENT_PRIO_MD);
 register_bp_thl_md_uisoc_notify(md_bp_uisoc_chg_cb, BATTERY_PERCENT_PRIO_MD);
 if (priv->psy) schedule_work(&priv->psy_work);
 dev_info(&pdev->dev, "Native modem power policy ready; callbacks retained until reboot\n");
 return 0;
fail:
 if (notifier_registered) power_supply_unreg_notifier(&md_pt_nb);
 cancel_work_sync(&priv->psy_work);
 if (priv->psy) { power_supply_put(priv->psy); priv->psy = NULL; }
 md_pt_free_tables();
 memcpy(md_pt_info, saved, sizeof(saved));
 return ret;
}

static const struct of_device_id md_power_throttling_of_match[] = {
	{ .compatible = "mediatek,md-power-throttling", },
	{},
};
MODULE_DEVICE_TABLE(of, md_power_throttling_of_match);
static struct platform_driver md_power_throttling_driver = {
	.probe = mtk_md_power_throttling_probe,
	.driver = {
		.name = "mtk-md_power_throttling",
		.suppress_bind_attrs = true,
		.of_match_table = md_power_throttling_of_match,
	},
};
module_platform_driver(md_power_throttling_driver);
MODULE_AUTHOR("Samuel Hsieh");
MODULE_DESCRIPTION("MTK modem power throttling driver");
MODULE_LICENSE("GPL");
