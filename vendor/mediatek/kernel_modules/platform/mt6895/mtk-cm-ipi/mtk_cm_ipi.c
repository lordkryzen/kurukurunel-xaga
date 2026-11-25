// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <linux/scmi_protocol.h>
#include <linux/module.h>
#include <tinysys-scmi.h>

#include "mtk_cm_ipi.h"

//static phys_addr_t mem_phys_addr, mem_virt_addr;
//static unsigned long long mem_size;
static int cm_sspm_ready;
static int cm_ipi_enable = 1;
static u32 scmi_cm_id;
static DEFINE_MUTEX(cm_transport_lock);
static int native_fault;
module_param(native_fault, int, 0444);
module_param_named(native_ready, cm_sspm_ready, int, 0444);
static struct scmi_tinysys_info_st *_tinfo;




int cm_get_ipi_enable(void)
{
	return cm_ipi_enable;
}
EXPORT_SYMBOL(cm_get_ipi_enable);

void cm_set_ipi_enable(int enable)
{
	cm_ipi_enable = enable;
}
EXPORT_SYMBOL(cm_set_ipi_enable);

void cm_sspm_enable(int enable)
{
	cm_mgr_to_sspm_command(IPI_CM_MGR_ENABLE, enable);
}
EXPORT_SYMBOL(cm_sspm_enable);


unsigned int cm_mgr_to_sspm_command(unsigned int cmd, unsigned int val)
{
	int ret;

	mutex_lock(&cm_transport_lock);
	if (native_fault) {
		ret = native_fault;
		goto out;
	}
	if (!cm_sspm_ready) {
		ret = -EAGAIN;
		goto out;
	}
	/* Keep xaga's native command words and unused zero arguments unchanged. */
	ret = scmi_tinysys_common_set(_tinfo->ph, scmi_cm_id, cmd, val, 0, 0, 0);
	if (ret) {
		if (ret > 0)
			ret = -EIO;
		native_fault = ret;
		cm_sspm_ready = 0;
		pr_err("CM SCMI command %u failed: %d; transport quarantined\n", cmd, ret);
	}
out:
	mutex_unlock(&cm_transport_lock);
	return ret;
}
EXPORT_SYMBOL(cm_mgr_to_sspm_command);

/* Additive preflight API: validate the real provider before publishing users. */
int cm_ipi_prepare(void)
{
	struct scmi_tinysys_info_st *info;
	u32 feature, legacy_feature;
	const char *property;
	int ret = 0;

	mutex_lock(&cm_transport_lock);
	if (native_fault) {
		ret = native_fault;
		goto out;
	}
	if (cm_sspm_ready)
		goto out;
	info = get_scmi_tinysys_info();
	if (IS_ERR(info)) {
		ret = PTR_ERR(info);
		goto out;
	}
	if (!info || IS_ERR_OR_NULL(info->sdev) ||
	    IS_ERR_OR_NULL(info->ph) || !info->sdev->dev.of_node) {
		ret = -EPROBE_DEFER;
		goto out;
	}
	/* New DTs use a dash; retain the native underscore spelling as an alias.
	 * Never invent a feature ID or hide a malformed canonical property. */
	property = of_property_present(info->sdev->dev.of_node, "scmi-cm") ?
		"scmi-cm" : "scmi_cm";
	ret = of_property_read_u32(info->sdev->dev.of_node, property, &feature);
	if (ret)
		goto out;
	if (of_property_present(info->sdev->dev.of_node, "scmi-cm") &&
	    of_property_present(info->sdev->dev.of_node, "scmi_cm")) {
		ret = of_property_read_u32(info->sdev->dev.of_node, "scmi_cm", &legacy_feature);
		if (ret)
			goto out;
		if (feature != legacy_feature) {
			ret = -EINVAL;
			goto out;
		}
	}
	_tinfo = info;
	scmi_cm_id = feature;
	cm_sspm_ready = 1;
out:
	mutex_unlock(&cm_transport_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(cm_ipi_prepare);

void cm_ipi_init(void)
{
	int ret = cm_ipi_prepare();

	if (!ret)
		cm_sspm_enable(cm_ipi_enable);
	else
		pr_err("CM SCMI provider unavailable: %d\n", ret);
}
EXPORT_SYMBOL(cm_ipi_init);

MODULE_DESCRIPTION("CM ipi Driver v0.1");
MODULE_LICENSE("GPL");
