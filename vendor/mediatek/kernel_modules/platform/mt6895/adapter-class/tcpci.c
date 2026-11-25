// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include "inc/tcpci.h"
#include "inc/tcpc_operation_lifetime.h"
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include "inc/tcpc_notify_lifetime.h"
#include <linux/sched/clock.h>

#include "inc/tcpci_typec.h"
#include "inc/std_tcpci_v10.h"

#define TCPC_NOTIFY_OVERTIME	(20) /* ms */

/* Serialize queue admission against detachment. The caller already owns
 * tcpc; this lock protects its queue, not the lifetime of the containing object.
 */
static DEFINE_MUTEX(tcpc_notify_queue_lock);

void tcpc_notify_queue_stop(struct tcpc_device *tcpc)
{
	struct workqueue_struct *queue;

	mutex_lock(&tcpc_notify_queue_lock);
	queue = tcpc->evt_wq;
	tcpc->evt_wq = NULL;
	mutex_unlock(&tcpc_notify_queue_lock);
	/* Do not hold admission lock while a callback drains: it may itself
	 * submit a notification, which must return ENODEV rather than deadlock.
	 */
	if (queue)
		destroy_workqueue(queue);
}

#if CONFIG_TCPC_NOTIFICATION_NON_BLOCKING
struct tcp_notify_work {
	struct work_struct work;
	struct tcpc_device *tcpc;
	struct tcp_notify tcp_noti;
	struct pd_status status;
	uint8_t type;
	uint8_t state;
};

static void tcp_notify_func(struct work_struct *work)
{
	struct tcp_notify_work *tn_work =
		container_of(work, struct tcp_notify_work, work);
	struct tcpc_device *tcpc = tn_work->tcpc;
	struct tcp_notify *tcp_noti = &tn_work->tcp_noti;
	uint8_t type = tn_work->type;
	uint8_t state = tn_work->state;
#if CONFIG_PD_BEGUG_ON
	u64 t = 0;

	t = local_clock();
	srcu_notifier_call_chain(&tcpc->evt_nh[type], state, tcp_noti);
	t = local_clock() - t;
	do_div(t, NSEC_PER_MSEC);
	PD_WARN_ON(t > TCPC_NOTIFY_OVERTIME);
#else
	srcu_notifier_call_chain(&tcpc->evt_nh[type], state, tcp_noti);
#endif

	kfree(tn_work);
}

static int tcpc_check_notify_time(struct tcpc_device *tcpc,
	struct tcp_notify *tcp_noti, uint8_t type, uint8_t state)
{
	struct tcp_notify_work *tn_work;
	int ret;

	if (!tcpc || !tcp_noti || type >= TCP_NOTIFY_IDX_NR)
		return -EINVAL;
	if (state == TCP_NOTIFY_STATUS &&
	    (type != TCP_NOTIFY_IDX_MISC || !tcp_noti->status_msg.sdb))
		return -EINVAL;

	tn_work = kzalloc(sizeof(*tn_work), GFP_KERNEL);
	if (!tn_work)
		return -ENOMEM;

	INIT_WORK(&tn_work->work, tcp_notify_func);
	tn_work->tcpc = tcpc;
	tn_work->tcp_noti = *tcp_noti;
	if (state == TCP_NOTIFY_STATUS) {
		/* Status originates outside this queued event's lifetime. */
		tn_work->status = *tcp_noti->status_msg.sdb;
		tn_work->tcp_noti.status_msg.sdb = &tn_work->status;
	}
	tn_work->type = type;
	tn_work->state = state;

	mutex_lock(&tcpc_notify_queue_lock);
	if (!tcpc->evt_wq)
		ret = -ENODEV;
	else
		ret = queue_work(tcpc->evt_wq, &tn_work->work) ? 0 : -EAGAIN;
	mutex_unlock(&tcpc_notify_queue_lock);
	if (ret)
		kfree(tn_work);
	return ret;
}
#else
static int tcpc_check_notify_time(struct tcpc_device *tcpc,
	struct tcp_notify *tcp_noti, uint8_t type, uint8_t state)
{
	int ret;
#if CONFIG_PD_BEGUG_ON
	u64 t = 0;

	t = local_clock();
	ret = srcu_notifier_call_chain(&tcpc->evt_nh[type], state, tcp_noti);
	t = local_clock() - t;
	do_div(t, NSEC_PER_MSEC);
	PD_WARN_ON(t > TCPC_NOTIFY_OVERTIME);
#else
	ret = srcu_notifier_call_chain(&tcpc->evt_nh[type], state, tcp_noti);
#endif
	return ret;
}
#endif /* CONFIG_TCPC_NOTIFICATION_NON_BLOCKING */

bool tcpci_check_vbus_valid_from_ic(struct tcpc_device *tcpc)
{
	int vbus_level = tcpc->vbus_level;

	if (tcpci_get_power_status(tcpc) == 0) {
		if (vbus_level != tcpc->vbus_level) {
			TCPC_INFO("[Warning] ps_changed %d -> %d\n",
				vbus_level, tcpc->vbus_level);
			tcpc->ps_changed = true;
		}
	}

	return tcpci_check_vbus_valid(tcpc);
}
EXPORT_SYMBOL(tcpci_check_vbus_valid_from_ic);

static int __tcpc_hw_tcpci_set_auto_dischg_discnt(struct tcpc_device *tcpc, bool en, struct tcpc_ops *ops)
{
	int ret = 0;

	if (en == tcpc->typec_auto_dischg_discnt)
		goto out;
	if (ops->set_auto_dischg_discnt)
		ret = ops->set_auto_dischg_discnt(tcpc, en);
	if (ret < 0)
		goto out;
	tcpc->typec_auto_dischg_discnt = en;
out:
	return ret;
}

int tcpci_set_auto_dischg_discnt(struct tcpc_device *tcpc, bool en)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_auto_dischg_discnt(tcpc, en, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_auto_dischg_discnt);

static int __tcpc_hw_tcpci_get_vbus_voltage(struct tcpc_device *tcpc, u32 *vbus, struct tcpc_ops *ops)
{
	int ret = 0;

	if (ops->get_vbus_voltage)
		ret = ops->get_vbus_voltage(tcpc, vbus);

	return ret;
}

int tcpci_get_vbus_voltage(struct tcpc_device *tcpc, u32 *vbus)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_get_vbus_voltage(tcpc, vbus, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_get_vbus_voltage);

bool tcpci_check_vsafe0v(struct tcpc_device *tcpc)
{
	return tcpc->vbus_level == TCPC_VBUS_SAFE0V;
}
EXPORT_SYMBOL(tcpci_check_vsafe0v);

#ifdef CONFIG_SUPPORT_SOUTHCHIP_PDPHY
static int __tcpc_hw_tcpci_get_chip_id(struct tcpc_device *tcpc,uint32_t *chip_id, struct tcpc_ops *ops)
{
	if (ops->get_chip_id == NULL) {
		return -ENOTSUPP;
	}
	return ops->get_chip_id(tcpc,chip_id);
}

int tcpci_get_chip_id(struct tcpc_device *tcpc,uint32_t *chip_id)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_get_chip_id(tcpc, chip_id, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}

static int __tcpc_hw_tcpci_get_chip_pid(struct tcpc_device *tcpc,uint32_t *chip_pid, struct tcpc_ops *ops)
{
	if (ops->get_chip_pid == NULL) {
		return -ENOTSUPP;
	}
	return ops->get_chip_pid(tcpc,chip_pid);
}

int tcpci_get_chip_pid(struct tcpc_device *tcpc,uint32_t *chip_pid)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_get_chip_pid(tcpc, chip_pid, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}

static int __tcpc_hw_tcpci_get_chip_vid(struct tcpc_device *tcpc,uint32_t *chip_vid, struct tcpc_ops *ops)
{
	if (ops->get_chip_vid == NULL) {
		return -ENOTSUPP;
	}
	return ops->get_chip_vid(tcpc,chip_vid);
}

int tcpci_get_chip_vid(struct tcpc_device *tcpc,uint32_t *chip_vid)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_get_chip_vid(tcpc, chip_vid, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
#endif /* CONFIG_SUPPORT_SOUTHCHIP_PDPHY */

static int __tcpc_hw_tcpci_alert_status_clear(
	struct tcpc_device *tcpc, uint32_t mask, struct tcpc_ops *ops)
{
	PD_WARN_ON(ops->alert_status_clear == NULL);

	return ops->alert_status_clear(tcpc, mask);
}

int tcpci_alert_status_clear(
	struct tcpc_device *tcpc, uint32_t mask)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_alert_status_clear(tcpc, mask, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_alert_status_clear);

static int __tcpc_hw_tcpci_fault_status_clear(
	struct tcpc_device *tcpc, uint8_t status, struct tcpc_ops *ops)
{
	PD_WARN_ON(ops->fault_status_clear == NULL);

	return ops->fault_status_clear(tcpc, status);
}

int tcpci_fault_status_clear(
	struct tcpc_device *tcpc, uint8_t status)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_fault_status_clear(tcpc, status, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_fault_status_clear);

static int __tcpc_hw_tcpci_set_alert_mask(struct tcpc_device *tcpc, uint32_t mask, struct tcpc_ops *ops)
{
	int ret = 0;

	if (ops->set_alert_mask)
		ret = ops->set_alert_mask(tcpc, mask);

	return ret;
}

int tcpci_set_alert_mask(struct tcpc_device *tcpc, uint32_t mask)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_alert_mask(tcpc, mask, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_alert_mask);

static int __tcpc_hw_tcpci_get_alert_mask(
	struct tcpc_device *tcpc, uint32_t *mask, struct tcpc_ops *ops)
{
	PD_WARN_ON(ops->get_alert_mask == NULL);

	return ops->get_alert_mask(tcpc, mask);
}

int tcpci_get_alert_mask(
	struct tcpc_device *tcpc, uint32_t *mask)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_get_alert_mask(tcpc, mask, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_get_alert_mask);

static int __tcpc_hw_tcpci_get_alert_status_and_mask(
	struct tcpc_device *tcpc, uint32_t *alert, uint32_t *mask, struct tcpc_ops *ops)
{
	int ret = 0;

	if (ops->get_alert_status_and_mask)
		ret = ops->get_alert_status_and_mask(tcpc, alert, mask);

	return ret;
}

int tcpci_get_alert_status_and_mask(
	struct tcpc_device *tcpc, uint32_t *alert, uint32_t *mask)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_get_alert_status_and_mask(tcpc, alert, mask, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_get_alert_status_and_mask);

static int __tcpc_hw_tcpci_get_fault_status(
	struct tcpc_device *tcpc, uint8_t *fault, struct tcpc_ops *ops)
{
	if (ops->get_fault_status)
		return ops->get_fault_status(tcpc, fault);

	*fault = 0;
	return 0;
}

int tcpci_get_fault_status(
	struct tcpc_device *tcpc, uint8_t *fault)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_get_fault_status(tcpc, fault, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_get_fault_status);

static int __tcpc_hw_tcpci_get_power_status(struct tcpc_device *tcpc, struct tcpc_ops *ops)
{
	int ret;

	PD_WARN_ON(ops->get_power_status == NULL);

	ret = ops->get_power_status(tcpc);
	if (ret < 0)
		return ret;

	tcpci_vbus_level_refresh(tcpc);
	return 0;
}

int tcpci_get_power_status(struct tcpc_device *tcpc)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_get_power_status(tcpc, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_get_power_status);

static int __tcpc_hw_tcpci_init(struct tcpc_device *tcpc, bool sw_reset, struct tcpc_ops *ops)
{
	int ret;

	PD_WARN_ON(ops->init == NULL);

	ret = ops->init(tcpc, sw_reset);
	if (ret < 0)
		return ret;

	return tcpci_get_power_status(tcpc);
}

int tcpci_init(struct tcpc_device *tcpc, bool sw_reset)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_init(tcpc, sw_reset, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_init);

static int __tcpc_hw_tcpci_init_alert_mask(struct tcpc_device *tcpc, struct tcpc_ops *ops)
{
	if (ops->init_alert_mask)
		return ops->init_alert_mask(tcpc);
	return 0;
}

int tcpci_init_alert_mask(struct tcpc_device *tcpc)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_init_alert_mask(tcpc, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_init_alert_mask);

static int __tcpc_hw_tcpci_get_cc(struct tcpc_device *tcpc, struct tcpc_ops *ops)
{
	int ret;
	int cc1, cc2;

	PD_WARN_ON(ops->get_cc == NULL);

	ret = ops->get_cc(tcpc, &cc1, &cc2);
	if (ret < 0)
		return ret;

	if ((cc1 == tcpc->typec_remote_cc[0]) &&
	    (cc2 == tcpc->typec_remote_cc[1]))
		return 0;

	tcpc->typec_remote_cc[0] = cc1;
	tcpc->typec_remote_cc[1] = cc2;

	return 1;
}

int tcpci_get_cc(struct tcpc_device *tcpc)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_get_cc(tcpc, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_get_cc);

int tcpci_is_plugged_in(struct tcpc_device *tcpc)
{
	if (tcpci_check_vbus_valid_from_ic(tcpc))
		return 1;

	if (tcpci_get_cc(tcpc) < 0)
		return -1;

	if ((tcpc->typec_remote_cc[0] == TYPEC_CC_VOLT_OPEN ||
	     tcpc->typec_remote_cc[0] == TYPEC_CC_DRP_TOGGLING) &&
	    (tcpc->typec_remote_cc[1] == TYPEC_CC_VOLT_OPEN ||
	     tcpc->typec_remote_cc[1] == TYPEC_CC_DRP_TOGGLING))
		return 0;

	return 1;
}
EXPORT_SYMBOL(tcpci_is_plugged_in);

static int __tcpc_hw_tcpci_set_cc(struct tcpc_device *tcpc, int pull, struct tcpc_ops *ops)
{
#if CONFIG_TYPEC_LEGACY3_ALWAYS_LOCAL_RP
	uint8_t rp_lvl = TYPEC_CC_PULL_GET_RP_LVL(pull);
	uint8_t res = TYPEC_CC_PULL_GET_RES(pull);

	if (rp_lvl == TYPEC_RP_DFT) {
		if (tcpc->typec_state == typec_unattached_snk ||
		    tcpc->typec_state == typec_unattached_src)
			rp_lvl = TYPEC_RP_1_5;
		else
			rp_lvl = tcpc->typec_local_rp_level;
	}
	pull = TYPEC_CC_PULL(rp_lvl, res);
#endif /* CONFIG_TYPEC_LEGACY3_ALWAYS_LOCAL_RP */

	PD_WARN_ON(ops->set_cc == NULL);

	if (pull & TYPEC_CC_DRP) {
		tcpc->typec_remote_cc[0] =
		tcpc->typec_remote_cc[1] =
			TYPEC_CC_DRP_TOGGLING;
	}

	tcpc->typec_local_cc = pull;
	return ops->set_cc(tcpc, pull);
}

int tcpci_set_cc(struct tcpc_device *tcpc, int pull)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_cc(tcpc, pull, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_cc);

static int __tcpc_hw_tcpci_set_polarity(struct tcpc_device *tcpc, int polarity, struct tcpc_ops *ops)
{
	PD_WARN_ON(ops->set_polarity == NULL);

	return ops->set_polarity(tcpc, polarity);
}

int tcpci_set_polarity(struct tcpc_device *tcpc, int polarity)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_polarity(tcpc, polarity, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_polarity);

static int __tcpc_hw_tcpci_set_vconn(struct tcpc_device *tcpc, int enable, struct tcpc_ops *ops)
{
	struct tcp_notify tcp_noti;

	if (tcpc->tcpc_source_vconn == enable)
		return 0;

	tcpc->tcpc_source_vconn = enable;

	tcp_noti.en_state.en = enable != 0;
	tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_VBUS, TCP_NOTIFY_SOURCE_VCONN);

	if (ops->set_vconn)
		return ops->set_vconn(tcpc, enable);

	return 0;
}

int tcpci_set_vconn(struct tcpc_device *tcpc, int enable)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_vconn(tcpc, enable, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_vconn);

static int __tcpc_hw_tcpci_set_low_power_mode(struct tcpc_device *tcpc, bool en, struct tcpc_ops *ops)
{
	int ret = 0, pull = TYPEC_CC_OPEN;

	tcpc->typec_lpm = en;
	if (!tcpc_typec_is_cc_open_state(tcpc)) {
		switch (tcpc->typec_role) {
		case TYPEC_ROLE_SNK:
			pull = TYPEC_CC_RD;
			break;
		case TYPEC_ROLE_SRC:
			pull = TYPEC_CC_RP;
			break;
		default:
			pull = TYPEC_CC_DRP;
			break;
		}
	}

	if (ops->set_low_power_mode)
		ret = ops->set_low_power_mode(tcpc, en, pull);

	return ret;
}

int tcpci_set_low_power_mode(struct tcpc_device *tcpc, bool en)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_low_power_mode(tcpc, en, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_low_power_mode);

static int __tcpc_hw_tcpci_alert_vendor_defined_handler(struct tcpc_device *tcpc, struct tcpc_ops *ops)
{
	int ret = 0;

	if (ops->alert_vendor_defined_handler)
		ret = ops->alert_vendor_defined_handler(tcpc);

	return ret;
}

int tcpci_alert_vendor_defined_handler(struct tcpc_device *tcpc)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_alert_vendor_defined_handler(tcpc, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_alert_vendor_defined_handler);

#ifdef CONFIG_SUPPORT_SOUTHCHIP_PDPHY
static int __tcpc_hw_tcpci_set_watchdog(struct tcpc_device *tcpc, bool en, struct tcpc_ops *ops)
{
	int rv = 0;
	uint32_t chip_vid = 0;
	rv = tcpci_get_chip_vid(tcpc, &chip_vid);
	if (!rv && (chip_vid == SOUTHCHIP_PD_VID)) {
		if (ops->set_watchdog)
			rv = ops->set_watchdog(tcpc, en);
	} else {
	if (tcpc->tcpc_flags & TCPC_FLAGS_WATCHDOG_EN)
		if (ops->set_watchdog)
			rv = ops->set_watchdog(tcpc, en);
	}
	return rv;
}

int tcpci_set_watchdog(struct tcpc_device *tcpc, bool en)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_watchdog(tcpc, en, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_watchdog);
#endif /* CONFIG_SUPPORT_SOUTHCHIP_PDPHY */

#if CONFIG_WATER_DETECTION
static int __tcpc_hw_tcpci_set_water_protection(struct tcpc_device *tcpc, bool en, struct tcpc_ops *ops)
{
	if (ops->set_water_protection)
		return ops->set_water_protection(tcpc, en);
	return 0;
}

int tcpci_set_water_protection(struct tcpc_device *tcpc, bool en)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_water_protection(tcpc, en, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_water_protection);

int tcpci_notify_wd_status(struct tcpc_device *tcpc, bool water_detected)
{
	struct tcp_notify tcp_noti;

	tcp_noti.wd_status.water_detected = water_detected;
	return tcpc_check_notify_time(tcpc, &tcp_noti, TCP_NOTIFY_IDX_MISC,
				      TCP_NOTIFY_WD_STATUS);
}
EXPORT_SYMBOL(tcpci_notify_wd_status);
#endif /* CONFIG_WATER_DETECTION */

int tcpci_notify_fod_status(struct tcpc_device *tcpc)
{
	struct tcp_notify tcp_noti;

	tcp_noti.fod_status.fod = tcpc->typec_fod;
	return tcpc_check_notify_time(tcpc, &tcp_noti, TCP_NOTIFY_IDX_MISC,
				      TCP_NOTIFY_FOD_STATUS);
}
EXPORT_SYMBOL(tcpci_notify_fod_status);

#if CONFIG_CABLE_TYPE_DETECTION
static int __tcpc_hw_tcpci_reset_ctd(struct tcpc_device *tcpc, struct tcpc_ops *ops)
{
	if (ops->reset_ctd)
		return ops->reset_ctd(tcpc);
	return 0;
}

int tcpci_reset_ctd(struct tcpc_device *tcpc)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_reset_ctd(tcpc, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_reset_ctd);

int tcpci_notify_cable_type(struct tcpc_device *tcpc)
{
	struct tcp_notify tcp_noti;

	tcp_noti.cable_type.type = tcpc->typec_cable_type;
	return tcpc_check_notify_time(tcpc, &tcp_noti, TCP_NOTIFY_IDX_MISC,
				      TCP_NOTIFY_CABLE_TYPE);
}
EXPORT_SYMBOL(tcpci_notify_cable_type);
#endif /* CONFIG_CABLE_TYPE_DETECTION */

int tcpci_notify_typec_otp(struct tcpc_device *tcpc)
{
	struct tcp_notify tcp_noti;

	tcp_noti.typec_otp.otp = tcpc->typec_otp;
	return tcpc_check_notify_time(tcpc, &tcp_noti, TCP_NOTIFY_IDX_MISC,
				      TCP_NOTIFY_TYPEC_OTP);
}
EXPORT_SYMBOL(tcpci_notify_typec_otp);

static int __tcpc_hw_tcpci_set_cc_hidet(struct tcpc_device *tcpc, bool en, struct tcpc_ops *ops)
{
	int ret = 0, cc_hi = 0;

	if (en == tcpc->cc_hidet_en)
		goto out;
	if (!ops->set_cc_hidet)
		goto out;
	ret = ops->set_cc_hidet(tcpc, en);
	if (ret < 0)
		goto out;
	tcpc->cc_hidet_en = en;
	if (en) {
		cc_hi = tcpc_typec_get_rp_present_flag(tcpc);
		if (ops->get_cc_hi)
			cc_hi |= ops->get_cc_hi(tcpc);
	}
	ret = tcpci_notify_cc_hi(tcpc, cc_hi);
out:
	return ret;
}

int tcpci_set_cc_hidet(struct tcpc_device *tcpc, bool en)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_cc_hidet(tcpc, en, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_cc_hidet);

#ifdef CONFIG_SUPPORT_SOUTHCHIP_PDPHY
int tcpci_notify_wd0_state(struct tcpc_device *tcpc, bool wd0_state, bool typec_port_stat)
{
	struct tcp_notify tcp_noti;

	tcp_noti.wd0_state.wd0 = wd0_state;
	tcp_noti.wd0_state.is_typec_port0 = typec_port_stat;

	TCPC_DBG("wd0: %d\n", wd0_state);
	return tcpc_check_notify_time(tcpc, &tcp_noti, TCP_NOTIFY_IDX_MISC,
						TCP_NOTIFY_WD0_STATE);
}
#else
int tcpci_notify_wd0_state(struct tcpc_device *tcpc, bool wd0_state)
{
	struct tcp_notify tcp_noti;

	tcp_noti.wd0_state.wd0 = wd0_state;

	TCPC_DBG("wd0: %d\n", wd0_state);
	return tcpc_check_notify_time(tcpc, &tcp_noti, TCP_NOTIFY_IDX_MISC,
				      TCP_NOTIFY_WD0_STATE);
}
#endif /* CONFIG_SUPPORT_SOUTHCHIP_PDPHY */
EXPORT_SYMBOL(tcpci_notify_wd0_state);

static int __tcpc_hw_tcpci_set_vbus_short_cc(struct tcpc_device *tcpc, bool cc1, bool cc2, struct tcpc_ops *ops)
{
	int ret = 0;
	int vsc = (cc1 ? BIT(TCPC_POLARITY_CC1) : 0) |
		  (cc2 ? BIT(TCPC_POLARITY_CC2) : 0);

	if (vsc == tcpc->vbus_short_cc)
		goto out;
	if (ops->set_vbus_short_cc)
		ret = ops->set_vbus_short_cc(tcpc, cc1, cc2);
	if (ret < 0)
		goto out;
	tcpc->vbus_short_cc = vsc;
out:
	return ret;
}

int tcpci_set_vbus_short_cc(struct tcpc_device *tcpc, bool cc1, bool cc2)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_vbus_short_cc(tcpc, cc1, cc2, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_vbus_short_cc);

int tcpci_notify_vbus_short_cc_status(struct tcpc_device *tcpc, int vsc_status)
{
	struct tcp_notify tcp_noti;

	if (vsc_status == tcpc->vsc_status)
		return 0;
	tcpc->vsc_status = vsc_status;
	tcp_noti.vsc_status = vsc_status;
	TCPC_DBG("notify_vbus_short_cc: %d\n", vsc_status);
	return tcpc_check_notify_time(tcpc, &tcp_noti, TCP_NOTIFY_IDX_MISC,
				      TCP_NOTIFY_VBUS_SHORT_CC);
}
EXPORT_SYMBOL(tcpci_notify_vbus_short_cc_status);

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)

static int __tcpc_hw_tcpci_set_msg_header(struct tcpc_device *tcpc,
	uint8_t power_role, uint8_t data_role, struct tcpc_ops *ops)
{
	PD_WARN_ON(ops->set_msg_header == NULL);

	return ops->set_msg_header(tcpc, power_role, data_role);
}

int tcpci_set_msg_header(struct tcpc_device *tcpc,
	uint8_t power_role, uint8_t data_role)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_msg_header(tcpc, power_role, data_role, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_msg_header);

static int __tcpc_hw_tcpci_set_rx_enable(struct tcpc_device *tcpc, uint8_t enable, struct tcpc_ops *ops)
{
	PD_WARN_ON(ops->set_rx_enable == NULL);

	return ops->set_rx_enable(tcpc, enable);
}

int tcpci_set_rx_enable(struct tcpc_device *tcpc, uint8_t enable)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_rx_enable(tcpc, enable, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_rx_enable);

static int __tcpc_hw_tcpci_protocol_reset(struct tcpc_device *tcpc, struct tcpc_ops *ops)
{
	if (ops->protocol_reset)
		return ops->protocol_reset(tcpc);

	return 0;
}

int tcpci_protocol_reset(struct tcpc_device *tcpc)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_protocol_reset(tcpc, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_protocol_reset);

static int __tcpc_hw_tcpci_get_message(struct tcpc_device *tcpc,
	uint32_t *payload, uint16_t *head, enum tcpm_transmit_type *type, struct tcpc_ops *ops)
{
	PD_WARN_ON(ops->get_message == NULL);

	return ops->get_message(tcpc, payload, head, type);
}

int tcpci_get_message(struct tcpc_device *tcpc,
	uint32_t *payload, uint16_t *head, enum tcpm_transmit_type *type)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_get_message(tcpc, payload, head, type, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_get_message);

void tcpc_tx_pending_work_func(struct work_struct *work)
{
	struct tcpc_device *tcpc = container_of(work, struct tcpc_device,
						tx_pending_work.work);

	atomic_dec_if_positive(&tcpc->tx_pending);
	if (!atomic_read(&tcpc->tx_pending))
		wake_up(&tcpc->tx_wait_que);
}

static void tcpc_wait_tx_done(struct tcpc_device *tcpc)
{
#if TCPC_DBG_ENABLE
	long ret = 0;
#endif /* TCPC_DBG_ENABLE */
	const u64 j = jiffies;

	if (time_after_eq64(j, tcpc->tx_jiffies + tcpc->tx_jiffies_max))
		return;

#if TCPC_DBG_ENABLE
	ret = wait_event_timeout(tcpc->tx_wait_que,
				 !atomic_read(&tcpc->tx_pending),
				 tcpc->tx_jiffies + tcpc->tx_jiffies_max - j);
	TCPC_DBG("%s ret = %ld\n", __func__, ret);
#else
	wait_event_timeout(tcpc->tx_wait_que, !atomic_read(&tcpc->tx_pending),
			   tcpc->tx_jiffies + tcpc->tx_jiffies_max - j);
#endif /* TCPC_DBG_ENABLE */
}

static int __tcpc_hw_tcpci_transmit(struct tcpc_device *tcpc,
	enum tcpm_transmit_type type, uint16_t header, const uint32_t *data, struct tcpc_ops *ops)
{
	int ret = 0;
	unsigned int bits = 0;

	PD_WARN_ON(ops->transmit == NULL);

	tcpc_wait_tx_done(tcpc);

	mutex_lock(&tcpc->rxbuf_lock);
	ret = ops->transmit(tcpc, type, header, data);
	mutex_unlock(&tcpc->rxbuf_lock);
	if (ret < 0)
		return ret;

	atomic_inc(&tcpc->tx_pending);
	tcpc->tx_jiffies = jiffies;
	switch (type) {
	case TCPC_TX_HARD_RESET:
	case TCPC_TX_CABLE_RESET:
		/*
		 * 64bits preamble + 20bits k-code
		 * Max_tUI = 3700ns
		 */
		tcpc->tx_jiffies_max = nsecs_to_jiffies((64 + 20) * 3700);
		break;
	case TCPC_TX_BIST_MODE_2:
		/* tBISTContMode = 30ms ~ 60ms */
		tcpc->tx_jiffies_max = msecs_to_jiffies(60);
		flush_delayed_work(&tcpc->tx_pending_work);
		schedule_delayed_work(&tcpc->tx_pending_work,
				      tcpc->tx_jiffies_max);
		break;
	case TCPC_TX_SOP:
	case TCPC_TX_SOP_PRIME:
	case TCPC_TX_SOP_PRIME_PRIME:
	case TCPC_TX_SOP_DEBUG_PRIME:
	case TCPC_TX_SOP_DEBUG_PRIME_PRIME:
	default:
		/*
		 * 64bits preamble + 20bits k-code +
		 * maximum 30bytes data (header + payload) in 4b +
		 * 4bytes CRC in 4b +
		 * 5bits k-code (EOP)
		 * Max_tUI = 3700ns
		 */
		bits = 64 + 20 +
			(2 + 4 * PD_HEADER_CNT(header) + 4) * 8 * 5 / 4 + 5;
		tcpc->tx_jiffies_max = nsecs_to_jiffies(bits * 3700);

		/* tReceive = 0.9ms ~ 1.1ms */
		tcpc->tx_jiffies_max =
			tcpc->tx_jiffies_max * (tcpc->pd_retry_count + 1) +
			usecs_to_jiffies(1100) * tcpc->pd_retry_count;
		break;
	}

	return ret;
}

int tcpci_transmit(struct tcpc_device *tcpc,
	enum tcpm_transmit_type type, uint16_t header, const uint32_t *data)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_transmit(tcpc, type, header, data, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_transmit);

static int __tcpc_hw_tcpci_retransmit(struct tcpc_device *tcpc, struct tcpc_ops *ops)
{
	int ret = 0;

	tcpc_wait_tx_done(tcpc);

	mutex_lock(&tcpc->rxbuf_lock);
	ret = ops->retransmit(tcpc);
	mutex_unlock(&tcpc->rxbuf_lock);
	if (ret < 0)
		return ret;

	atomic_inc(&tcpc->tx_pending);
	tcpc->tx_jiffies = jiffies;

	return ret;
}

int tcpci_retransmit(struct tcpc_device *tcpc)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_retransmit(tcpc, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_retransmit);

static int __tcpc_hw_tcpci_set_bist_test_mode(struct tcpc_device *tcpc, bool en, struct tcpc_ops *ops)
{
	if (ops->set_bist_test_mode)
		return ops->set_bist_test_mode(tcpc, en);

	return 0;
}

int tcpci_set_bist_test_mode(struct tcpc_device *tcpc, bool en)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_set_bist_test_mode(tcpc, en, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_set_bist_test_mode);
#endif	/* CONFIG_USB_POWER_DELIVERY */

#if CONFIG_TYPEC_DIRECT_CHARGE
int tcpci_notify_direct_charge(struct tcpc_device *tcpc, bool en)
{
	mutex_lock(&tcpc->access_lock);
	tcpc->typec_during_direct_charge = en;
	mutex_unlock(&tcpc->access_lock);

	return 0;
}
#endif	/* CONFIG_TYPEC_DIRECT_CHARGE */

int tcpci_notify_typec_state(struct tcpc_device *tcpc)
{
	struct tcp_notify tcp_noti;

	tcp_noti.typec_state.rp_level = tcpc->typec_remote_rp_level;
	tcp_noti.typec_state.local_rp_level = tcpc->typec_local_rp_level;
	tcp_noti.typec_state.polarity = tcpc->typec_polarity;
	tcp_noti.typec_state.old_state = tcpc->typec_attach_old;
	tcp_noti.typec_state.new_state = tcpc->typec_attach_new;

	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_USB, TCP_NOTIFY_TYPEC_STATE);
}
EXPORT_SYMBOL(tcpci_notify_typec_state);

int tcpci_notify_role_swap(
	struct tcpc_device *tcpc, uint8_t event, uint8_t role)
{
	struct tcp_notify tcp_noti;

	tcp_noti.swap_state.new_role = role;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MISC, event);
}
EXPORT_SYMBOL(tcpci_notify_role_swap);

int tcpci_notify_pd_mode(struct tcpc_device *tcpc)
{
	struct tcp_notify tcp_noti;

	memset(&tcp_noti, 0, sizeof(tcp_noti));
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_USB, TCP_NOTIFY_PD_MODE);
}
EXPORT_SYMBOL(tcpci_notify_pd_mode);

int tcpci_notify_pd_state(struct tcpc_device *tcpc, uint8_t connect)
{
	struct tcp_notify tcp_noti;

	tcp_noti.pd_state.connected = connect;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_USB, TCP_NOTIFY_PD_STATE);
}
EXPORT_SYMBOL(tcpci_notify_pd_state);

static void __tcpc_hw_tcpci_set_command(struct tcpc_device *tcpc, u8 cmd, struct tcpc_ops *ops)
{
	if (ops->set_command)
		ops->set_command(tcpc, cmd);
}

static void tcpci_set_command(struct tcpc_device *tcpc, u8 cmd)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);

	if (lease < 0)
		return;
	__tcpc_hw_tcpci_set_command(tcpc, cmd, ops);
	tcpc_operation_put(tcpc, lease);
}

int tcpci_source_vbus(
	struct tcpc_device *tcpc, uint8_t type, int mv, int ma)
{
	struct tcp_notify tcp_noti;

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
	if (type >= TCP_VBUS_CTRL_PD &&
			tcpc->pd_port.pe_data.pd_prev_connected)
		type |= TCP_VBUS_CTRL_PD_DETECT;
#endif	/* CONFIG_USB_POWER_DELIVERY */

	if (ma < 0) {
		if (mv != 0) {
			switch (tcpc->typec_local_rp_level) {
			case TYPEC_RP_3_0:
				ma = 3000;
				break;
			case TYPEC_RP_1_5:
				ma = 1500;
				break;
			case TYPEC_RP_DFT:
			default:
				ma = CONFIG_TYPEC_SRC_CURR_DFT;
				break;
			}
		} else
			ma = 0;
	}

	tcp_noti.vbus_state.mv = mv;
	tcp_noti.vbus_state.ma = ma;
	tcp_noti.vbus_state.type = type;

	TCPC_DBG("source_vbus(0x%02X): %d mV, %d mA\n", type, mv, ma);
	tcpci_set_command(tcpc, mv ? TCPM_CMD_ENABLE_SOURCE_VBUS :
				     TCPM_CMD_DISABLE_SOURCE_VBUS);
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_VBUS, TCP_NOTIFY_SOURCE_VBUS);
}
EXPORT_SYMBOL(tcpci_source_vbus);

int tcpci_sink_vbus(
	struct tcpc_device *tcpc, uint8_t type, int mv, int ma)
{
	struct tcp_notify tcp_noti;

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)
	if (type >= TCP_VBUS_CTRL_PD &&
			tcpc->pd_port.pe_data.pd_prev_connected)
		type |= TCP_VBUS_CTRL_PD_DETECT;
#endif	/* CONFIG_USB_POWER_DELIVERY */

	if (ma < 0) {
		if (mv != 0) {
			switch (tcpc->typec_remote_rp_level) {
			case TYPEC_CC_VOLT_SNK_3_0:
				ma = 3000;
				break;
			case TYPEC_CC_VOLT_SNK_1_5:
				ma = 1500;
				break;
			case TYPEC_CC_VOLT_SNK_DFT:
			default:
				ma = tcpc->typec_usb_sink_curr;
				break;
			}
#if CONFIG_TYPEC_SNK_CURR_LIMIT > 0
			if (ma > CONFIG_TYPEC_SNK_CURR_LIMIT)
				ma = CONFIG_TYPEC_SNK_CURR_LIMIT;
#endif	/* CONFIG_TYPEC_SNK_CURR_LIMIT */
		} else
			ma = 0;
	}

	tcpc->sink_vbus_mv = mv;
	tcpc->sink_vbus_ma = ma;
	tcpc->sink_vbus_type = type;

	tcp_noti.vbus_state.mv = mv;
	tcp_noti.vbus_state.ma = ma;
	tcp_noti.vbus_state.type = type;

	TCPC_DBG("sink_vbus(0x%02X): %d mV, %d mA\n", type, mv, ma);
	tcpci_set_command(tcpc, mv ? TCPM_CMD_ENABLE_SINK_VBUS :
				     TCPM_CMD_DISABLE_SINK_VBUS);
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_VBUS, TCP_NOTIFY_SINK_VBUS);
}
EXPORT_SYMBOL(tcpci_sink_vbus);

int tcpci_disable_vbus_control(struct tcpc_device *tcpc)
{
#if CONFIG_TYPEC_USE_DIS_VBUS_CTRL
	struct tcp_notify tcp_noti;

	TCPC_DBG("disable_vbus\n");

	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_VBUS, TCP_NOTIFY_DIS_VBUS_CTRL);
#else
	tcpci_sink_vbus(tcpc, TCP_VBUS_CTRL_REMOVE, TCPC_VBUS_SINK_0V, 0);
	tcpci_source_vbus(tcpc, TCP_VBUS_CTRL_REMOVE, TCPC_VBUS_SOURCE_0V, 0);
	return 0;
#endif	/* CONFIG_TYPEC_USE_DIS_VBUS_CTRL */
}
EXPORT_SYMBOL(tcpci_disable_vbus_control);

int tcpci_notify_attachwait_state(struct tcpc_device *tcpc, bool as_sink)
{
#if CONFIG_TYPEC_NOTIFY_ATTACHWAIT
	uint8_t notify = 0;
	struct tcp_notify tcp_noti;

#if CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK
	if (as_sink)
		notify = TCP_NOTIFY_ATTACHWAIT_SNK;
#endif	/* CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SNK */

#if CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SRC
	if (!as_sink)
		notify = TCP_NOTIFY_ATTACHWAIT_SRC;
#endif	/* CONFIG_TYPEC_NOTIFY_ATTACHWAIT_SRC */

	if (notify == 0)
		return 0;

	memset(&tcp_noti, 0, sizeof(tcp_noti));
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_VBUS, notify);
#else
	return 0;
#endif	/* CONFIG_TYPEC_NOTIFY_ATTACHWAIT */
}
EXPORT_SYMBOL(tcpci_notify_attachwait_state);

static int __tcpc_hw_tcpci_enable_force_discharge(
	struct tcpc_device *tcpc, bool en, int mv, struct tcpc_ops *ops)
{
	int ret = 0;

#if CONFIG_TYPEC_CAP_FORCE_DISCHARGE
	if (tcpc->typec_force_discharge != en) {
		tcpc->typec_force_discharge = en;
		if (ops->set_force_discharge)
			ret = ops->set_force_discharge(tcpc, en, mv);
	}
#endif	/* CONFIG_TYPEC_CAP_FORCE_DISCHARGE */

	return ret;
}

static inline int tcpci_enable_force_discharge(
	struct tcpc_device *tcpc, bool en, int mv)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_enable_force_discharge(tcpc, en, mv, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}

static inline int tcpci_enable_ext_discharge(
	struct tcpc_device *tcpc, bool en)
{
	int ret = 0;
#if CONFIG_TYPEC_CAP_EXT_DISCHARGE
	struct tcp_notify tcp_noti;

	if (tcpc->typec_ext_discharge != en) {
		tcpc->typec_ext_discharge = en;
		tcp_noti.en_state.en = en;
		TCPC_DBG("EXT-Discharge: %d\n", en);
		ret = tcpc_check_notify_time(tcpc, &tcp_noti,
			TCP_NOTIFY_IDX_VBUS, TCP_NOTIFY_EXT_DISCHARGE);
	}
#endif	/* CONFIG_TYPEC_CAP_EXT_DISCHARGE */

	return ret;
}

int tcpci_enable_discharge(struct tcpc_device *tcpc, bool en, int mv)
{
	int ret = 0;

	ret = tcpci_enable_force_discharge(tcpc, en, mv);
	ret |= tcpci_enable_ext_discharge(tcpc, en);

	return ret;
}

static int __tcpc_hw_tcpci_enable_io_boost(struct tcpc_device *tcpc, bool en, struct tcpc_ops *ops)
{
	if (ops->enable_io_boost)
		return ops->enable_io_boost(tcpc, en);
	return -EOPNOTSUPP;
}

int tcpci_enable_io_boost(struct tcpc_device *tcpc, bool en)
{
	struct tcpc_ops *ops;
	int lease = tcpc_operation_get(tcpc, &ops);
	int ret;

	if (lease < 0)
		return lease;
	ret = __tcpc_hw_tcpci_enable_io_boost(tcpc, en, ops);
	tcpc_operation_put(tcpc, lease);
	return ret;
}
EXPORT_SYMBOL(tcpci_enable_io_boost);

int tcpci_notify_ps_change(struct tcpc_device *tcpc, int vbus_level)
{
	struct tcp_notify tcp_noti;

	tcp_noti.vbus_level = vbus_level;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_PS_CHANGE);
}
EXPORT_SYMBOL(tcpci_notify_ps_change);

int tcpci_notify_cc_hi(struct tcpc_device *tcpc, int cc_hi)
{
	struct tcp_notify tcp_noti;

	if (cc_hi == tcpc->cc_hi)
		return 0;
	tcpc->cc_hi = cc_hi;
	tcp_noti.cc_hi = cc_hi;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_CC_HI);
}
EXPORT_SYMBOL(tcpci_notify_cc_hi);

int tcpci_notify_alert_ratelimited(struct tcpc_device *tcpc, bool limited)
{
	struct tcp_notify tcp_noti;

	if (limited == tcpc->alert_ratelimited)
		return 0;
	tcpc->alert_ratelimited = limited;
	tcp_noti.alert_ratelimited = limited;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_ALERT_RATELIMITED);
}

#if IS_ENABLED(CONFIG_USB_POWER_DELIVERY)

int tcpci_notify_hard_reset_state(struct tcpc_device *tcpc, uint8_t state)
{
	struct tcp_notify tcp_noti;

	tcp_noti.hreset_state.state = state;

	if (state >= TCP_HRESET_SIGNAL_SEND)
		tcpc->pd_wait_hard_reset_complete = true;
	else if (tcpc->pd_wait_hard_reset_complete)
		tcpc->pd_wait_hard_reset_complete = false;
	else
		return 0;

	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_HARD_RESET_STATE);
}
EXPORT_SYMBOL(tcpci_notify_hard_reset_state);

int tcpci_notify_wait_new_cap(struct tcpc_device *tcpc)
{
	struct tcp_notify tcp_noti;

	memset(&tcp_noti, 0, sizeof(tcp_noti));
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_WAIT_NEW_CAP);
}
EXPORT_SYMBOL(tcpci_notify_wait_new_cap);

int tcpci_enter_mode(struct tcpc_device *tcpc,
	uint16_t svid, uint8_t ops, uint32_t mode)
{
	struct tcp_notify tcp_noti;

	tcp_noti.mode_ctrl.svid = svid;
	tcp_noti.mode_ctrl.ops = ops;
	tcp_noti.mode_ctrl.mode = mode;

	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_ENTER_MODE);
}
EXPORT_SYMBOL(tcpci_enter_mode);

int tcpci_exit_mode(struct tcpc_device *tcpc, uint16_t svid)
{
	struct tcp_notify tcp_noti;

	tcp_noti.mode_ctrl.svid = svid;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_EXIT_MODE);

}
EXPORT_SYMBOL(tcpci_exit_mode);

int tcpci_report_hpd_state(struct tcpc_device *tcpc, uint32_t dp_status)
{
	struct tcp_notify tcp_noti;
	struct dp_data *dp_data = pd_get_dp_data(&tcpc->pd_port);

	/* UFP_D to DFP_D only */

	if (PD_DP_CFG_DFP_D(dp_data->local_config)) {
		tcp_noti.ama_dp_hpd_state.irq = PD_VDO_DPSTS_IRQ_HPD(dp_status);
		tcp_noti.ama_dp_hpd_state.state =
					PD_VDO_DPSTS_HPD_STATE(dp_status);
		tcpc_check_notify_time(tcpc, &tcp_noti,
			TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_AMA_DP_HPD_STATE);
	}

	return 0;
}
EXPORT_SYMBOL(tcpci_report_hpd_state);

int tcpci_dp_status_update(struct tcpc_device *tcpc, uint32_t dp_status)
{
	DP_INFO("Status0: 0x%x\n", dp_status);
	tcpci_report_hpd_state(tcpc, dp_status);
	return 0;
}
EXPORT_SYMBOL(tcpci_dp_status_update);

int tcpci_dp_configure(struct tcpc_device *tcpc, uint32_t dp_config)
{
	struct tcp_notify tcp_noti;
	uint8_t active = 1;

	DP_INFO("LocalCFG: 0x%x\n", dp_config);

	switch (PD_DP_CFG_ROLE(dp_config)) {
	case DP_CONFIG_DFP_D:
		tcp_noti.ama_dp_state.sel_config = SW_DFP_D;
		break;
	case DP_CONFIG_UFP_D:
		tcp_noti.ama_dp_state.sel_config = SW_UFP_D;
		break;
	case DP_CONFIG_USB:
	default:
		tcp_noti.ama_dp_state.sel_config = SW_USB;
		active = 0;
		break;
	}
	tcp_noti.ama_dp_state.signal = DP_CFG_SIGNAL(dp_config);
	tcp_noti.ama_dp_state.pin_assignment = DP_CFG_PIN(dp_config);
	DP_INFO("pin assignment: 0x%x\n",
		tcp_noti.ama_dp_state.pin_assignment);
	tcp_noti.ama_dp_state.polarity = tcpc->typec_polarity;
	tcp_noti.ama_dp_state.active = active;

	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_AMA_DP_STATE);
}
EXPORT_SYMBOL(tcpci_dp_configure);

int tcpci_dp_attention(struct tcpc_device *tcpc, uint32_t dp_status)
{
	/* DFP_U : Not call this function during internal flow */
	struct tcp_notify tcp_noti;

	DP_INFO("Attention: 0x%x\n", dp_status);
	tcp_noti.ama_dp_attention.state = dp_status;
	tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_AMA_DP_ATTENTION);
	return tcpci_report_hpd_state(tcpc, dp_status);
}
EXPORT_SYMBOL(tcpci_dp_attention);

int tcpci_dp_notify_status_update_done(
	struct tcpc_device *tcpc, uint32_t dp_status, bool ack)
{
	/* DFP_U : Not call this function during internal flow */
	DP_INFO("Status1: 0x%x, ack=%d\n", dp_status, ack);
	return 0;
}
EXPORT_SYMBOL(tcpci_dp_notify_status_update_done);

int tcpci_dp_notify_config_start(struct tcpc_device *tcpc)
{
	/* DFP_U : Put signal & mux into the Safe State */
	struct tcp_notify tcp_noti;

	DP_INFO("ConfigStart\n");
	memset(&tcp_noti, 0, sizeof(tcp_noti));
	tcp_noti.ama_dp_state.sel_config = SW_USB;
	tcp_noti.ama_dp_state.polarity = tcpc->typec_polarity;
	tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_AMA_DP_STATE);
	return 0;
}
EXPORT_SYMBOL(tcpci_dp_notify_config_start);

int tcpci_dp_notify_config_done(struct tcpc_device *tcpc,
	uint32_t local_cfg, uint32_t remote_cfg, bool ack)
{
	/* DFP_U : If DP success,
	 * internal flow will enter this function finally
	 */
	DP_INFO("ConfigDone, L:0x%x, R:0x%x, ack=%d\n",
		local_cfg, remote_cfg, ack);

	if (ack)
		tcpci_dp_configure(tcpc, local_cfg);

	return 0;
}
EXPORT_SYMBOL(tcpci_dp_notify_config_done);

int tcpci_notify_cvdm(struct tcpc_device *tcpc, bool ack)
{
	struct tcp_notify tcp_noti;
	struct tcp_ny_cvdm *cvdm_msg = &tcp_noti.cvdm_msg;
	struct pd_port *pd_port = &tcpc->pd_port;

	cvdm_msg->ack = ack;
	cvdm_msg->cable = pd_port->cvdm_cable;

	if (ack) {
		cvdm_msg->cnt = pd_port->cvdm_cnt;
		memcpy(cvdm_msg->data, pd_port->cvdm_data,
		       sizeof(cvdm_msg->data[0]) * cvdm_msg->cnt);
	}

	tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MODE, TCP_NOTIFY_CVDM);
	return 0;
}
EXPORT_SYMBOL(tcpci_notify_cvdm);

/* ---- Policy Engine (PD30) ---- */

#if CONFIG_USB_PD_REV30

#if CONFIG_USB_PD_REV30_ALERT_REMOTE
int tcpci_notify_alert(struct tcpc_device *tcpc, uint32_t ado)
{
	struct tcp_notify tcp_noti;

	tcp_noti.alert_msg.ado = ado;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_ALERT);
}
EXPORT_SYMBOL(tcpci_notify_alert);
#endif	/* CONFIG_USB_PD_REV30_ALERT_REMOTE */

#if CONFIG_USB_PD_REV30_STATUS_REMOTE
int tcpci_notify_status(struct tcpc_device *tcpc, struct pd_status *sdb)
{
	struct tcp_notify tcp_noti;

	tcp_noti.status_msg.sdb = sdb;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_STATUS);
}
EXPORT_SYMBOL(tcpci_notify_status);
#endif	/* CONFIG_USB_PD_REV30_STATUS_REMOTE */

#if CONFIG_USB_PD_REV30_BAT_INFO
int tcpci_notify_request_bat_info(
	struct tcpc_device *tcpc, enum pd_battery_reference ref)
{
	struct tcp_notify tcp_noti;

	tcp_noti.request_bat.ref = ref;
	return tcpc_check_notify_time(tcpc, &tcp_noti,
		TCP_NOTIFY_IDX_MISC, TCP_NOTIFY_REQUEST_BAT_INFO);
}
EXPORT_SYMBOL(tcpci_notify_request_bat_info);
#endif	/* CONFIG_USB_PD_REV30_BAT_INFO */
#endif	/* CONFIG_USB_PD_REV30 */
#endif	/* CONFIG_USB_POWER_DELIVERY */
