// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/property.h>
#include <linux/of_platform.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_dp.h>
#include <linux/phy/phy.h>

#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
#include "mux_switch.h"
#endif

#define CHECK_HPD_DELAY 2000

/* MT6983 uds_V1 offset = [11]
 * MT6985 uds_V2 offset = [18] dp 4-lanes offset = [19]
 * Reserved for future platform.
 */

enum usb_dp_sw_vers {
	uds_V1 = 1,
	uds_V2,
};

struct usb_dp_selector {
	struct device *dev;
	struct typec_switch_dev *sw;
	struct typec_mux_dev *mux;
	struct mutex lock;
	struct phy *phy;
	enum typec_orientation orientation;
	void __iomem *selector_reg_address;
	u32 uds_ver;
	bool native_mt6895;
	struct list_head list;
	bool published;
	bool is_dp;

	int hdp_state;
	bool dp_sw_connect;
	struct delayed_work check_wk;

};

/* Resolve opaque core handles without accessing retired private data.
 * Lock order: selector_registry_lock -> uds->lock. Registry writers and
 * unregister must never be entered from a selector callback.
 */
static LIST_HEAD(selector_list);
static DEFINE_MUTEX(selector_registry_lock);

static struct usb_dp_selector *selector_lock(const void *handle, bool mux)
{
	struct usb_dp_selector *uds;

	mutex_lock(&selector_registry_lock);
	list_for_each_entry(uds, &selector_list, list) {
		if ((mux && uds->mux == handle) || (!mux && uds->sw == handle)) {
			mutex_lock(&uds->lock);
			mutex_unlock(&selector_registry_lock);
			return uds;
		}
	}
	mutex_unlock(&selector_registry_lock);
	return NULL;
}

static inline void uds_setbits(void __iomem *base, u32 bits)
{
	void __iomem *addr = base;
	u32 tmp = readl(addr);

	writel((tmp | (bits)), addr);
}

static inline void uds_clrbits(void __iomem *base, u32 bits)
{
	void __iomem *addr = base;
	u32 tmp = readl(addr);

	writel((tmp & ~(bits)), addr);
}

#define PHY_MODE_NORMAL	7
#define PHY_MODE_FLIP	8

static int notify_tcpc_orientation(struct usb_dp_selector *uds,
				enum typec_orientation orientation)
{
	int mode = 0;
	int ret = 0;

	if (orientation == TYPEC_ORIENTATION_NORMAL)
		mode = PHY_MODE_NORMAL;
	else if (orientation == TYPEC_ORIENTATION_REVERSE)
		mode = PHY_MODE_FLIP;

	if (uds->phy) {
		ret = phy_set_mode_ext(uds->phy, PHY_MODE_USB_DEVICE, mode);
		if (ret)
			dev_info(uds->dev, "failed to set phy ext mode\n");
	}

	return ret;
}

/* Native MT6895 register0x10005600 uses bit11 with the opposite
 * polarity to the later selector template. Preserve every unrelated bit.
 */
static int native_selector_orientation(struct usb_dp_selector *uds,
				       enum typec_orientation orientation)
{
	switch (orientation) {
	case TYPEC_ORIENTATION_NONE:
		break;
	case TYPEC_ORIENTATION_NORMAL:
		uds_setbits(uds->selector_reg_address, BIT(11));
		break;
	case TYPEC_ORIENTATION_REVERSE:
		uds_clrbits(uds->selector_reg_address, BIT(11));
		break;
	default:
		return -EINVAL;
	}
	uds->orientation = orientation;
	return 0;
}

static int usb_dp_selector_switch_locked(struct usb_dp_selector *uds,
			      enum typec_orientation orientation)
{
	int ret;

	dev_info(uds->dev, "%s %d\n", __func__, orientation);

	if (orientation < TYPEC_ORIENTATION_NONE || orientation > TYPEC_ORIENTATION_REVERSE)
		return -EINVAL;
	if (uds->native_mt6895)
		return native_selector_orientation(uds, orientation);
	ret = notify_tcpc_orientation(uds, orientation);
	if (ret)
		return ret;

	switch (orientation) {
	case TYPEC_ORIENTATION_NONE:
		/* Disconnect HPD */
		if (uds->is_dp == true) {
			/* We should clr this bit, since dp-4lane can not work with U3  */
			if (uds->uds_ver == uds_V2)
				uds_clrbits(uds->selector_reg_address, (1 << 19));
#if IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK_DPTX_AUTO)
			mtk_dp_SWInterruptSet(0x2);
#endif
			uds->dp_sw_connect = false;
			uds->is_dp = false;
		}

		/* switch mux to u3 off state */
		if (uds->orientation == TYPEC_ORIENTATION_NORMAL) {
			if (uds->uds_ver == uds_V1)
				uds_setbits(uds->selector_reg_address, (1 << 11));
			else if (uds->uds_ver == uds_V2)
				uds_setbits(uds->selector_reg_address, (1 << 18));
		} else if (uds->orientation == TYPEC_ORIENTATION_REVERSE) {
			if (uds->uds_ver == uds_V1)
				uds_clrbits(uds->selector_reg_address, (1 << 11));
			else if (uds->uds_ver == uds_V2)
				uds_clrbits(uds->selector_reg_address, (1 << 18));
		}

		break;
	case TYPEC_ORIENTATION_NORMAL:
		/* USB NORMAL TX1, DP TX2 */
		switch (uds->uds_ver) {
		case uds_V1:
			uds_clrbits(uds->selector_reg_address, (1 << 11));
#if IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK_DPTX_AUTO)
			mtk_dp_aux_swap_enable(true);
#endif
			break;
		case uds_V2:
			uds_clrbits(uds->selector_reg_address, (1 << 18));
#if IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK_DPTX_AUTO)
			mtk_dp_aux_swap_enable(true);
#endif
			break;
		default:
			break;
		}
		break;
	case TYPEC_ORIENTATION_REVERSE:
		/* USB FLIP TX2, DP TX1 */
		switch (uds->uds_ver) {
		case uds_V1:
			uds_setbits(uds->selector_reg_address, (1 << 11));
#if IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK_DPTX_AUTO)
			mtk_dp_aux_swap_enable(false);
#endif
			break;
		case uds_V2:
			uds_setbits(uds->selector_reg_address, (1 << 18));
#if IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK_DPTX_AUTO)
			mtk_dp_aux_swap_enable(false);
#endif
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	uds->orientation = orientation;
	return 0;
}

/*
 * case
 *  4 Pin Assignment C 4-lans
 * 16 Pin Assignment E 4-lans
 *  8 Pin Assignment D 2-lans
 * 32 Pin Assignment F 2-lans
 */

static int usb_dp_selector_mux_locked(struct usb_dp_selector *uds,
				struct typec_mux_state *state)
{
	struct typec_displayport_data *dp_data;
	int ret = 0;

	if (!state || !state->data)
		return -EINVAL;
	dp_data = state->data;
	if (!IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK_DPTX_AUTO) &&
	    (dp_data->conf || dp_data->status))
		return -EOPNOTSUPP;
	if (dp_data->conf && dp_data->conf != 4 && dp_data->conf != 16 &&
	    dp_data->conf != 8 && dp_data->conf != 32)
		return -EINVAL;

	dev_info(uds->dev, "dp_data->config = %d\n", dp_data->conf);
	dev_info(uds->dev, "dp_data->status = %d\n", dp_data->status);
	dev_info(uds->dev, "state->mode = %lu\n", state->mode);


	if (dp_data->conf) {
		uds->is_dp = true;

		/*if (!data->ama_dp_state.active) {
		 *	dev_info(uds->dev, "%s Not active\n", __func__);
		 *	return ret;
		 *}
		 */

		switch (dp_data->conf) {
		/* 4-lanes */
		case 4:
		case 16:
			switch (uds->uds_ver) {
			case uds_V1:
				break;
			case uds_V2:
				uds_setbits(uds->selector_reg_address, (1 << 19));
#if IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK_DPTX_AUTO)
				mtk_dp_set_pin_assign(dp_data->conf);
#endif
				break;
			default:
				break;
			}
			break;
		/* 2-lanes */
		case 8:
		case 32:
			switch (uds->uds_ver) {
			case uds_V1:
				break;
			case uds_V2:
				uds_clrbits(uds->selector_reg_address, (1 << 19));
#if IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK_DPTX_AUTO)
				mtk_dp_set_pin_assign(dp_data->conf);
#endif
				break;
			default:
				break;
			}
			break;
		default:
			dev_info(uds->dev, "%s Pin Assignment not support\n", __func__);
			break;
		}
		uds->hdp_state = 0;
		/* Mark force HPD WA by DP Team Request. */
		/* schedule_delayed_work(&uds->check_wk, msecs_to_jiffies(CHECK_HPD_DELAY)); */

	} else {
		u32 irq = dp_data->status & DP_STATUS_IRQ_HPD;
		u32 state = dp_data->status & DP_STATUS_HPD_STATE;

		dev_info(uds->dev, "TCP Notify DP HPD irq:%x state:%x\n",
			irq, state);

		uds->hdp_state = state;

		/* Call DP API */
		dev_info(uds->dev, "[%s][%d]\n", __func__, __LINE__);
#if IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK_DPTX_AUTO)
		if (state) {
			if (irq) {
				if (uds->dp_sw_connect == false) {
					dev_info(uds->dev, "Force connect\n");
					mtk_dp_SWInterruptSet(0x4);
					uds->dp_sw_connect = true;
				}
				mtk_dp_SWInterruptSet(0x8);
			} else {
				mtk_dp_SWInterruptSet(0x4);
				uds->dp_sw_connect = true;
			}
		} else {
			mtk_dp_SWInterruptSet(0x2);
			uds->dp_sw_connect = false;
		}
#endif
	}

	return ret;
}

static int usb_dp_selector_switch_set(struct typec_switch_dev *sw,
				      enum typec_orientation orientation)
{
	struct usb_dp_selector *uds = selector_lock(sw, false);
	int ret;

	if (!uds)
		return -ENODEV;
	ret = usb_dp_selector_switch_locked(uds, orientation);
	mutex_unlock(&uds->lock);
	return ret;
}

static int usb_dp_selector_mux_set(struct typec_mux_dev *mux,
				   struct typec_mux_state *state)
{
	struct usb_dp_selector *uds = selector_lock(mux, true);
	int ret;

	if (!uds)
		return -ENODEV;
	ret = usb_dp_selector_mux_locked(uds, state);
	mutex_unlock(&uds->lock);
	return ret;
}

static void check_hpd(struct work_struct *work)
{
	struct delayed_work *check_wk = to_delayed_work(work);
	struct usb_dp_selector *uds = container_of(check_wk,
					struct usb_dp_selector, check_wk);

	mutex_lock(&uds->lock);
	if (uds->published && uds->hdp_state == 0) {
		dev_info(uds->dev, "%s: force hpd\n", __func__);
#if IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK_DPTX_AUTO)
		mtk_dp_SWInterruptSet(0x4);
#endif
	}
	mutex_unlock(&uds->lock);
}

/* Devres action is registered before any callback handle and runs before
 * the managed PHY/MMIO/context are released, including failed probe.
 */
static void usb_dp_selector_cleanup(void *data)
{
	struct usb_dp_selector *uds = data;

	mutex_lock(&selector_registry_lock);
	mutex_lock(&uds->lock);
	if (uds->published) {
		list_del(&uds->list);
		uds->published = false;
	}
	mutex_unlock(&uds->lock);
	mutex_unlock(&selector_registry_lock);
#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
	mtk_typec_mux_unregister(uds->mux);
	mtk_typec_switch_unregister(uds->sw);
#else
	if (!IS_ERR_OR_NULL(uds->mux))
		typec_mux_unregister(uds->mux);
	if (!IS_ERR_OR_NULL(uds->sw))
		typec_switch_unregister(uds->sw);
#endif
	uds->mux = NULL;
	uds->sw = NULL;
	cancel_delayed_work_sync(&uds->check_wk);
}

static int usb_dp_selector_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct usb_dp_selector *uds;
	struct typec_switch_desc sw_desc = { };
	struct typec_mux_desc mux_desc = { };
	int ret;

	uds = devm_kzalloc(&pdev->dev, sizeof(*uds), GFP_KERNEL);
	if (!uds)
		return -ENOMEM;

	uds->dev = dev;
	mutex_init(&uds->lock);
	INIT_LIST_HEAD(&uds->list);

	uds->selector_reg_address = devm_platform_ioremap_resource_byname(pdev, "usb_dp_reg");
	if (IS_ERR(uds->selector_reg_address))
		return PTR_ERR(uds->selector_reg_address);

	uds->native_mt6895 = of_machine_is_compatible("mediatek,MT6895");
	uds->uds_ver = uds_V1;
	if (device_property_present(dev, "mediatek,uds-ver")) {
		ret = device_property_read_u32(dev, "mediatek,uds-ver", &uds->uds_ver);
		if (ret)
			return ret;
	}
	if (uds->uds_ver != uds_V1 && uds->uds_ver != uds_V2)
		return -EINVAL;

	/* xaga has no phys property. A declared supplier must resolve strictly;
	 * malformed and deferred named PHY lookups must not become success.
	 */
	if (device_property_present(dev, "phys")) {
		uds->phy = devm_phy_get(dev, "usb3-phy");
		if (IS_ERR(uds->phy))
			return PTR_ERR(uds->phy);
	}

	uds->is_dp = false;
	uds->dp_sw_connect = false;
	uds->orientation = TYPEC_ORIENTATION_NONE;

	INIT_DEFERRABLE_WORK(&uds->check_wk, check_hpd);
	ret = devm_add_action_or_reset(dev, usb_dp_selector_cleanup, uds);
	if (ret)
		return ret;

	/* Setting Switch callback */
	sw_desc.drvdata = uds;
	sw_desc.fwnode = dev->fwnode;
	sw_desc.set = usb_dp_selector_switch_set;

#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
	uds->sw = mtk_typec_switch_register(dev, &sw_desc);
#else
	uds->sw = typec_switch_register(dev, &sw_desc);
#endif
	if (IS_ERR(uds->sw)) {
		dev_info(dev, "error registering typec switch: %ld\n",
			PTR_ERR(uds->sw));
		return PTR_ERR(uds->sw);
	}

	/* Setting MUX callback */
	mux_desc.drvdata = uds;
	mux_desc.fwnode = dev->fwnode;
	mux_desc.set = usb_dp_selector_mux_set;
#if IS_ENABLED(CONFIG_MTK_USB_TYPEC_MUX)
	uds->mux = mtk_typec_mux_register(dev, &mux_desc);
#else
	uds->mux = typec_mux_register(dev, &mux_desc);
#endif
	if (IS_ERR(uds->mux)) {
		dev_info(dev, "error registering typec mux: %ld\n",
			PTR_ERR(uds->mux));
		return PTR_ERR(uds->mux);
	}

	mutex_lock(&selector_registry_lock);
	mutex_lock(&uds->lock);
	list_add_tail(&uds->list, &selector_list);
	uds->published = true;
	mutex_unlock(&uds->lock);
	mutex_unlock(&selector_registry_lock);
	platform_set_drvdata(pdev, uds);

	dev_info(dev, "probe done\n");
	return 0;
}

static void usb_dp_selector_remove(struct platform_device *pdev)
{
	struct usb_dp_selector *uds = platform_get_drvdata(pdev);

	if (!uds)
		return;
	devm_release_action(&pdev->dev, usb_dp_selector_cleanup, uds);
	platform_set_drvdata(pdev, NULL);
}

static const struct of_device_id usb_dp_selector_ids[] = {
	{.compatible = "mediatek,usb_dp_selector",},
	{},
};

MODULE_DEVICE_TABLE(of, usb_dp_selector_ids);

static struct platform_driver usb_dp_selector_driver = {
	.driver = {
		.name = "usb_dp_selector",
		.of_match_table = usb_dp_selector_ids,
	},
	.probe = usb_dp_selector_probe,
	.remove = usb_dp_selector_remove,
};

module_platform_driver(usb_dp_selector_driver);

MODULE_DESCRIPTION("Type-C DP/USB selector");
MODULE_LICENSE("GPL v2");
