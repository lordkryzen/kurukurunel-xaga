/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

struct mtk_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;
	struct usb_role_switch *role_sw;
	unsigned int c_role; /* current data role */
	struct workqueue_struct *extcon_wq;
	struct regulator *vbus;
	unsigned int vbus_vol;
	unsigned int vbus_cur;
	bool vbus_on;
	bool vbus_owned;
	bool otg_controlled;
	struct power_supply *usb_psy;
	struct notifier_block psy_nb;
	struct delayed_work wq_psy;
#if IS_ENABLED(CONFIG_TCPC_CLASS)
	struct tcpc_device *tcpc_dev;
	struct notifier_block tcpc_nb;
#endif
	bool bypss_typec_sink;
	bool psy_registered;
	bool tcpc_registered;
	bool id_irq_registered;
	bool stopping;
	unsigned int applying_role;
	/* id gpio */
	struct gpio_desc *id_gpiod;
	unsigned int id_irq;
	struct delayed_work wq_detcable;
};

struct usb_role_info {
	struct mtk_extcon_info *extcon;
	struct delayed_work dwork;
	unsigned int d_role; /* desire data role */
};

enum {
	DUAL_PROP_MODE_UFP = 0,
	DUAL_PROP_MODE_DFP,
	DUAL_PROP_MODE_NONE,
};

enum {
	DUAL_PROP_PR_SRC = 0,
	DUAL_PROP_PR_SNK,
	DUAL_PROP_PR_NONE,
};
