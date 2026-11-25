// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/printk.h>
#include <linux/pm_opp.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include "mtk_ccu_public.h"
#include <clocksource/arm_arch_timer.h>

#define CREATE_TRACE_POINTS
#include "mtk-vmm-trace.h"
#include "internal.h"
#include "mtk-vmm-regulator.h"
#include "mtk-ispdvfs-power-owner.h"


static struct regulator *vmm_reg;
static struct dvfs_driver_data *power_provider;


#define REGULATOR_ID_VMM 0
#define AGING_MARGIN_MICROVOLT 12500
#define PM_QOS_VMM_ID 0x21

int mtk_ispdvfs_fix_dvfs;
EXPORT_SYMBOL(mtk_ispdvfs_fix_dvfs);
module_param(mtk_ispdvfs_fix_dvfs, int, 0644);

int mtk_ispdvfs_dbg_level;
EXPORT_SYMBOL(mtk_ispdvfs_dbg_level);
module_param(mtk_ispdvfs_dbg_level, int, 0644);

#define ISPDVFS_DBG
#ifdef ISPDVFS_DBG
#define ISP_LOGD(fmt, args...) \
	do { \
		if (mtk_ispdvfs_dbg_level & DVFS_DEBUG_LOG) \
			pr_notice("[ISPDVFS] %s(): " fmt "\n",\
				__func__, ##args); \
	} while (0)
#else
#define ISPDVFS_DBG(fmt, args...)
#endif
#define ISP_LOGI(fmt, args...) \
	pr_notice("[ISPDVFS] %s(): " fmt "\n", \
		__func__, ##args)
#define ISP_LOGE(fmt, args...) \
	pr_notice("[ISPDVFS] error %s(),%d: " fmt "\n", \
		__func__, __LINE__, ##args)
#define ISP_LOGF(fmt, args...) \
	pr_notice("[ISPDVFS] fatal %s(),%d: " fmt "\n", \
		__func__, __LINE__, ##args)

#define CREATE_REGULATOR(match, _name)	\
	.desc = {					\
		.name = match,				\
		.of_match = of_match_ptr(match),	\
		.ops = &vmm_apmcu_ops,		\
		.type = REGULATOR_VOLTAGE,		\
		.id = REGULATOR_ID_##_name,		\
		.owner = THIS_MODULE,			\
	}

struct vmm_regulator {
	struct regulator_desc	desc;
	struct dvfs_driver_data *dvfs_data;
	int is_enable;
};

struct ispdvfs_dbg_data {
	struct dvfs_driver_data *drv_data;
	struct regulator *reg;
	int max_voltage;
	bool reg_enable;
};

static struct ispdvfs_dbg_data *dbg_data;
static int fix_force_step(const char *val, const struct kernel_param *kp)
{
	int ret;
	u64 new_force_step;
	struct dvfs_driver_data *drv_data;
	struct dvfs_table *opp_table;
	struct dvfs_info *current_info;
	struct ccu_handle_info *ccu_handle;
	struct dvfs_ipc_info dvfs_ipi = {};

	ret = kstrtou64(val, 0, &new_force_step);
	if (ret) {
		ISP_LOGE("force set step failed: %d\n", ret);
		goto out;
	}

	if (!dbg_data) {
		ISP_LOGE("dbg_data is NULL\n");
		ret = PTR_ERR(dbg_data);
		goto out;
	}

	drv_data = dbg_data->drv_data;
	if (!atomic_read(&drv_data->ccu_power_on)) {
		ISP_LOGE("CCU is not power on before set voltage\n");
		ret = -EINVAL;
		goto out;
	}

	opp_table = &drv_data->opp_table;
	if (new_force_step >= opp_table->opp_num) {
		ISP_LOGE("Force level(%llu) is out of range\n",
				new_force_step);
		ret = -EINVAL;
		goto out;
	}

	current_info = &(drv_data->current_dvfs);
	mutex_lock(&current_info->voltage_mutex);

	dvfs_ipi.minOppIdx = new_force_step;
	dvfs_ipi.maxOppIdx = new_force_step;
	ccu_handle = &(drv_data->ccu_handle);
	ret = mtk_ccu_rproc_ipc_send(
		ccu_handle->ccu_pdev,
		MTK_CCU_FEATURE_ISPDVFS,
		DVFS_VOLTAGE_UPDATE,
		(void *)&dvfs_ipi, sizeof(struct dvfs_ipc_info));
	if (ret) {
		ISP_LOGE("mtk_ccu_rproc_ipc_send(DVFS_VOLTAGE_UPDATE) fail(%llu)\n",
				arch_timer_read_counter());
		WARN_ON(1);
		goto Unlock_Mutex;
	}
	current_info->voltage_target = opp_table->voltage[new_force_step];
	trace_vmm__update_voltage(current_info->voltage_target);

Unlock_Mutex:
	mutex_unlock(&current_info->voltage_mutex);
out:
	return ret;
}

static struct kernel_param_ops set_force_step_ops = {
	.set = fix_force_step,
};
module_param_cb(force_step, &set_force_step_ops, NULL, 0644);
MODULE_PARM_DESC(force_step, "force vmm dvfs to specified step");

#define MAX_BUFFER_SIZE (PAGE_SIZE - 1)
static int show_setting(char *buf, const struct kernel_param *kp)
{
	struct dvfs_driver_data *drv_data;
	struct dvfs_table *table;
	struct dvfs_info *current_info;
	struct ccu_handle_info *ccu_handle;
	struct dvfs_ipc_vb vb_info;
	u32 i, ret;
	int written = 0;
	int improve;

	if (!dbg_data) {
		ISP_LOGE("dbg_data is NULL\n");
		return PTR_ERR(dbg_data);
	}
	drv_data = dbg_data->drv_data;

	written += snprintf(buf + written, MAX_BUFFER_SIZE - written,
		"mux number:%d\n", drv_data->num_muxes);
	written += snprintf(buf + written, MAX_BUFFER_SIZE - written,
		"mux:");
	for (i = 0; i < drv_data->num_muxes; i++) {
		written += snprintf(buf + written, MAX_BUFFER_SIZE - written,
			"%s ", drv_data->muxes[i].mux_name);
	}
	written += snprintf(buf + written, MAX_BUFFER_SIZE - written,
		"\n");

	written += snprintf(buf + written, MAX_BUFFER_SIZE - written,
		"Support voltage:");

	table = &drv_data->opp_table;
	for (i = 0; i < table->opp_num; i++)
		written += snprintf(buf + written, MAX_BUFFER_SIZE - written,
				"%d ", table->voltage[i]);
	written += snprintf(buf + written, MAX_BUFFER_SIZE - written,
		"\n");

	current_info = &(drv_data->current_dvfs);
	written += snprintf(buf + written, MAX_BUFFER_SIZE - written,
			"Current voltage:%d\n", current_info->voltage_target);

	if (atomic_read(&drv_data->ccu_power_on)) {
		ccu_handle = &(drv_data->ccu_handle);
		vb_info.efuseValue = 0;
		ret = mtk_ccu_rproc_ipc_send(
			ccu_handle->ccu_pdev,
			MTK_CCU_FEATURE_ISPDVFS,
			DVFS_CCU_QUERY_VB,
			(void *)&vb_info, sizeof(struct dvfs_ipc_vb));
		if (ret) {
			ISP_LOGE("mtk_ccu_rproc_ipc_send(DVFS_CCU_QUERY_VB) fail(%llu)\n",
					arch_timer_read_counter());
		} else {
			if (drv_data->en_vb)
				written += snprintf(buf + written,
						MAX_BUFFER_SIZE - written,
						"Load supports VB\n");
			else
				written += snprintf(buf + written,
						MAX_BUFFER_SIZE - written,
						"Load does not support VB\n");
			written += snprintf(buf + written,
						MAX_BUFFER_SIZE - written,
						"Efuse reg:(0x%x)\n",
						vb_info.efuseValue);
		}

		if (table->opp_num <= MAX_OPP_STEP) {
			written += snprintf(buf + written, MAX_BUFFER_SIZE - written,
					"Final (After vb) voltage & improve:\n");
			for (i = 0; i < table->opp_num; i++) {
				improve = table->voltage[i] - vb_info.voltage[i];
				improve = (improve * 100)/table->voltage[i];
				written += snprintf(buf + written,
						MAX_BUFFER_SIZE - written,
						"volt(%d), improve(%d %%)\n",
						vb_info.voltage[i], improve);
			}
		}
	}

	return written;
}
static struct kernel_param_ops dump_param_ops = {.get = show_setting};
module_param_cb(show_setting, &dump_param_ops, NULL, 0444);
MODULE_PARM_DESC(show_setting, "dump vmm dvfs current setting");

static int regulator_trace_consumers(struct regulator_dev *rdev)
{
	struct regulator *regulator;
	struct regulator_voltage *voltage;
	const char *devname;

	list_for_each_entry(regulator, &rdev->consumer_list, list) {
		voltage = &regulator->voltage[PM_SUSPEND_ON];
		devname = regulator->dev ?
				dev_name(regulator->dev) : "deviceless";
		trace_mtk_pm_qos_update_request(PM_QOS_VMM_ID,
				voltage->min_uV, devname);
	}

	return 0;
}

static int force_voltage(void *data, u64 val)
{
	struct ispdvfs_dbg_data *dbg_data = (struct ispdvfs_dbg_data *)data;
	int voltage = (int)val;

	ISP_LOGI("Force votage(%d)", voltage);

	if (!dbg_data->reg_enable) {
		int ret = regulator_enable(dbg_data->reg);
		if (ret)
			return ret;
		dbg_data->reg_enable = true;
	}

	/* Disable vmm regulator when voltage is 0 */
	if (dbg_data->reg_enable && !voltage) {
		regulator_disable(dbg_data->reg);
		dbg_data->reg_enable = false;
	}

	if (IS_ERR(dbg_data->reg)) {
		ISP_LOGE("can't get dvfs regulator\n");
		return PTR_ERR(dbg_data->reg);
	}

	regulator_set_voltage(dbg_data->reg, voltage, INT_MAX);

	return 0;
}

static int force_opp_level(void *data, u64 val)
{
	struct ispdvfs_dbg_data *dbg_data = (struct ispdvfs_dbg_data *)data;
	struct dvfs_driver_data *drv_data;
	struct dvfs_table *opp_table;

	ISP_LOGI("Force opp level(%llu)", val);

	if (IS_ERR(dbg_data)) {
		ISP_LOGE("dbg_data is NULL\n");
		return PTR_ERR(dbg_data);
	}

	if (IS_ERR(dbg_data->reg)) {
		ISP_LOGE("can't get dvfs regulator\n");
		return PTR_ERR(dbg_data->reg);
	}

	drv_data = dbg_data->drv_data;
	if (IS_ERR(drv_data)) {
		ISP_LOGE("dbg_data is NULL\n");
		return -ENODEV;
	}
	opp_table = &(drv_data->opp_table);

	if (!dbg_data->reg_enable) {
		int ret = regulator_enable(dbg_data->reg);
		if (ret)
			return ret;
		dbg_data->reg_enable = true;
	}

	if (val < opp_table->opp_num)
		regulator_set_voltage(dbg_data->reg, opp_table->voltage[val], INT_MAX);
	else {
		ISP_LOGI("Opp level is not in range.\n");
		if (dbg_data->reg_enable) {
			regulator_disable(dbg_data->reg);
			dbg_data->reg_enable = false;
		}
	}

	return 0;
}

static inline struct device *to_vmm_dev(struct regulator_dev *rdev)
{
	return rdev_get_dev(rdev)->parent;
}

static void set_all_muxes(struct dvfs_driver_data *drv_data, u32 opp_level)
{
	u32 num_muxes = drv_data->num_muxes;
	u32 i;
	struct clk *mux, *clk_src;
	s32 err;

	for (i = 0; i < num_muxes; i++) {
		mux = drv_data->muxes[i].mux;
		clk_src = drv_data->muxes[i].clk_src[opp_level];
		err = clk_prepare_enable(mux);
		if (err) {
			ISP_LOGE("prepare mux(%s) fail:%d opp_level:%d\n",
			drv_data->muxes[i].mux_name, err, opp_level);
			continue;
		}
		err = clk_set_parent(mux, clk_src);
		if (err)
			ISP_LOGE("set parent(%s) fail:%d opp_level:%d\n",
					drv_data->muxes[i].mux_name, err, opp_level);
		clk_disable_unprepare(mux);
	}
}

static int enable_all_muxes(struct dvfs_driver_data *drv_data)
{
	u32 num_muxes;
	u32 i, j;
	struct clk *mux;
	s32 err;

	if (!drv_data) {
		ISP_LOGE("drv_data is NULL");
		return -ENODEV;
	}
	num_muxes = drv_data->num_muxes;

	for (i = 0; i < num_muxes; i++) {
		mux = drv_data->muxes[i].mux;
		err = clk_prepare_enable(mux);
		if (err) {
			ISP_LOGE("prepare mux(%s) fail:%d\n",
				drv_data->muxes[i].mux_name, err);

			for (j = 0; j < i; j++) {
				mux = drv_data->muxes[j].mux;
				clk_disable_unprepare(mux);
			}

			return err;
		}
	}

	return err;
}

static int disable_all_muxes(struct dvfs_driver_data *drv_data)
{
	u32 num_muxes;
	u32 i;
	struct clk *mux;

	if (!drv_data) {
		ISP_LOGE("drv_data is NULL");
		return -ENODEV;
	}
	num_muxes = drv_data->num_muxes;

	for (i = 0; i < num_muxes; i++) {
		mux = drv_data->muxes[i].mux;
		clk_disable_unprepare(mux);
	}

	return 0;
}

static void ccu_ipc_update_dvfs(uint32_t data, uint32_t len, void *priv)
{
	ISP_LOGD("Current VMM voltage(%d)", data);
	trace_vmm__update_voltage(data);
}

static void release_ccu(struct ccu_handle_info *h)
{
	if (h->ccu_pdev) {
		mtk_ccu_ipc_unregister(h->ccu_pdev, MTK_CCU_MSG_TO_APMCU_DVFS_STATUS);
	}
	if (h->proc) {
		rproc_shutdown(h->proc);
		rproc_put(h->proc);
	}
	if (h->ccu_pdev)
		put_device(&h->ccu_pdev->dev);
	memset(h, 0, sizeof(*h));
}

static int power_on_ccu(struct ccu_handle_info *h)
{
	struct device_node *node, *remote;
	struct platform_device *pdev;
	struct rproc *proc;
	u32 handle;
	int ret;

	node = of_find_compatible_node(NULL, NULL, "mediatek,ispdvfs");
	if (!node)
		return -ENODEV;
	ret = of_property_read_u32(node, "mediatek,ccu_rproc", &handle);
	of_node_put(node);
	if (ret)
		return ret;
	remote = of_find_node_by_phandle(handle);
	if (!remote)
		return -ENODEV;
	pdev = of_find_device_by_node(remote);
	of_node_put(remote);
	if (!pdev)
		return -EPROBE_DEFER;
	if (!pdev->dev.driver) {
		put_device(&pdev->dev);
		return -EPROBE_DEFER;
	}
	proc = rproc_get_by_phandle(handle);
	if (!proc) {
		put_device(&pdev->dev);
		return -EPROBE_DEFER;
	}
	ret = rproc_boot(proc);
	if (ret) {
		rproc_put(proc);
		put_device(&pdev->dev);
		return ret;
	}
	h->handle = handle;
	h->proc = proc;
	h->ccu_pdev = pdev;
	mtk_ccu_ipc_register(pdev, MTK_CCU_MSG_TO_APMCU_DVFS_STATUS,
			     ccu_ipc_update_dvfs, NULL);
	return 0;
}

static int get_idx_by_voltage(
		int voltage,
		const struct dvfs_table *table,
		u32 *min_volt,
		u32 *max_volt)
{
	int i = 0;

	if (!table || table->opp_num <= 0 || table->opp_num > MAX_OPP_STEP ||
	    !min_volt || !max_volt) {
		ISP_LOGE("some arguments are NULL\n");
		return -EINVAL;
	}

	for (i = 0; i < table->opp_num; i++) {
		if (voltage == table->voltage[i]) {
			*min_volt = i;
			*max_volt = i;
			break;
		} else if (voltage < table->voltage[i]) {
			if (i >= 1) {
				*min_volt = i - 1;
				*max_volt = i;
			} else {
				*min_volt = i;
				*max_volt = i;
			}
			break;
		}
	}

	if (i == table->opp_num) {
		*min_volt = i - 1;
		*max_volt = i - 1;
	}

	return 0;
}

static int ccu_set_voltage(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned int *selector)
{
	struct vmm_regulator *reg = rdev_get_drvdata(rdev);
	struct dvfs_driver_data *data;
	struct dvfs_ipc_info msg = {};
	int ret, token;

	if (!reg || !reg->dvfs_data)
		return -ENODEV;
	data = reg->dvfs_data;
	if (min_uV > max_uV || min_uV < data->opp_table.voltage[0] ||
	    min_uV > data->opp_table.voltage[data->opp_table.opp_num - 1])
		return -EINVAL;
	token = srcu_read_lock(&data->power_readers);
	if (!atomic_read_acquire(&data->power_ready) || READ_ONCE(data->power_uncertain)) {
		ret = READ_ONCE(data->power_uncertain) ? -EIO : -EAGAIN;
		goto out;
	}
	regulator_trace_consumers(rdev);
	mutex_lock(&data->current_dvfs.voltage_mutex);
	if (data->current_dvfs.voltage_target == min_uV) {
		ret = 0;
		goto unlock_voltage;
	}
	ret = get_idx_by_voltage(min_uV, &data->opp_table,
			&msg.minOppIdx, &msg.maxOppIdx);
	if (ret)
		goto unlock_voltage;
	ret = mtk_ccu_rproc_ipc_send(data->ccu_handle.ccu_pdev,
			MTK_CCU_FEATURE_ISPDVFS, DVFS_VOLTAGE_UPDATE,
			&msg, sizeof(msg));
	if (ret) {
		WRITE_ONCE(data->power_uncertain, true);
		goto unlock_voltage;
	}
	data->current_dvfs.voltage_target = min_uV;
	if (selector)
		*selector = msg.maxOppIdx;
	if (msg.maxOppIdx == msg.minOppIdx)
		trace_vmm__update_voltage(min_uV);
unlock_voltage:
	mutex_unlock(&data->current_dvfs.voltage_mutex);
out:
	srcu_read_unlock(&data->power_readers, token);
	return ret;
}

static int apmcu_set_voltage(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned int *selector)
{
	int ret = 0;
	struct vmm_regulator *regulator;
	struct dvfs_driver_data *drv_data;
	struct dvfs_info *current_info;
	struct dvfs_table *table;
	int target_voltage = min_uV;
	u32 cur_opp_idx = 0, prev_opp_idx = 0;
	u32 i;

	trace_vmm__update_voltage(min_uV);

	regulator = rdev_get_drvdata(rdev);
	if (!regulator) {
		ISP_LOGE("rdev_get_drvdata ptr is null");
		return -ENODEV;
	}

	drv_data = regulator->dvfs_data;
	if (!drv_data) {
		ISP_LOGE("drv_data ptr is null");
		return -ENODEV;
	}
	table = &(drv_data->opp_table);

	current_info = &(drv_data->current_dvfs);
	mutex_lock(&current_info->voltage_mutex);

	for (i = 0; i < table->opp_num; i++) {
		if (min_uV <= table->voltage[i]) {
			cur_opp_idx = i;
			break;
		}
	}
	prev_opp_idx = current_info->opp_level;

	if (drv_data->simulate_aging)
		target_voltage -= AGING_MARGIN_MICROVOLT;

	if (cur_opp_idx < prev_opp_idx) {
		/* Upldate Frequency firstly */
		set_all_muxes(drv_data, cur_opp_idx);

		/* Then update voltage */
		regulator_set_voltage(vmm_reg, target_voltage, INT_MAX);
		current_info->voltage_target = min_uV;
		current_info->opp_level = cur_opp_idx;
		mutex_unlock(&current_info->voltage_mutex);
	} else {
		/* Update voltage firstly */
		regulator_set_voltage(vmm_reg, target_voltage, INT_MAX);
		current_info->voltage_target = min_uV;
		current_info->opp_level = cur_opp_idx;
		mutex_unlock(&current_info->voltage_mutex);

		/* Then update frequency */
		set_all_muxes(drv_data, cur_opp_idx);
	}

	ISP_LOGD("VMM update voltage(%d)", target_voltage);

	return ret;
}

static int vmm_get_voltage(struct regulator_dev *rdev)
{
	struct vmm_regulator *regulator;
	struct dvfs_driver_data *drv_data;
	struct dvfs_info *current_info;
	int current_voltage = DEFAULT_VOLTAGE;

	regulator = rdev_get_drvdata(rdev);
	if (!regulator) {
		ISP_LOGE("rdev_get_drvdata ptr is null");
		goto out;
	}

	drv_data = regulator->dvfs_data;
	if (!drv_data) {
		ISP_LOGE("drv_data ptr is null");
		goto out;
	}

	current_info = &(drv_data->current_dvfs);
	mutex_lock(&current_info->voltage_mutex);
	current_voltage = current_info->voltage_target;
	mutex_unlock(&current_info->voltage_mutex);

out:
	return current_voltage;
}

static int vmm_init_opp_table(struct dvfs_driver_data *data)
{
	struct dvfs_table *table = &data->opp_table;
	struct dev_pm_opp *opp;
	unsigned long freq = 0;
	int ret, count, i;

	ret = devm_pm_opp_of_add_table(data->dev);
	if (ret)
		return ret;
	count = dev_pm_opp_get_opp_count(data->dev);
	if (count <= DEFAULT_VOLTAGE_LEVEL || count > MAX_OPP_STEP)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		opp = dev_pm_opp_find_freq_ceil(data->dev, &freq);
		if (IS_ERR(opp))
			return PTR_ERR(opp);
		table->frequency[i] = freq;
		table->voltage[i] = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);
		if (table->voltage[i] <= 0 ||
		    (i && table->voltage[i] < table->voltage[i - 1]))
			return -EINVAL;
		if (freq == ULONG_MAX)
			return -EOVERFLOW;
		freq++;
	}
	table->opp_num = count;
	return 0;
}

static int power_control(struct dvfs_driver_data *data, bool enable)
{
	struct ccu_handle_info *h = &data->ccu_handle;
	struct dvfs_ipc_init init = {
		.needVoltageBin = data->en_vb,
		.needSimAging = data->simulate_aging,
		.needCbFromMicroP = mtk_ispdvfs_dbg_level & DVFS_DEBUG_MICROP,
	};
	int ret, exit = 1;

	if (data->power_uncertain)
		return -EIO;
	if (enable) {
		ret = power_on_ccu(h);
		if (ret)
			return ret;
		ret = mtk_ccu_rproc_ipc_send(h->ccu_pdev, MTK_CCU_FEATURE_ISPDVFS,
				DVFS_CCU_INIT, &init, sizeof(init));
		if (ret) {
			/* IPC failure may follow firmware acceptance. Keep power and
			 * references; do not free an owner still visible to firmware. */
			WRITE_ONCE(data->power_uncertain, true);
			return ret;
		}
		atomic_set(&data->ccu_power_on, true);
		return 0;
	}
	ret = mtk_ccu_rproc_ipc_send(h->ccu_pdev, MTK_CCU_FEATURE_ISPDVFS,
			DVFS_CCU_UNINIT, &exit, sizeof(exit));
	if (ret) {
		WRITE_ONCE(data->power_uncertain, true);
		return ret;
	}
	release_ccu(h);
	atomic_set(&data->ccu_power_on, false);
	return 0;
}

/* Transition mutex is used ONLY outside regulator-core callbacks. */
static int vmm_power_get(struct dvfs_driver_data *data)
{
	int ret = 0;

	mutex_lock(&data->power_lock);
	if (READ_ONCE(data->power_uncertain)) {
		ret = -EIO;
		goto out;
	}
	if (data->power_users == UINT_MAX) {
		ret = -EOVERFLOW;
		goto out;
	}
	if (!data->power_users) {
		ret = enable_all_muxes(data);
		if (ret)
			goto out;
		data->mux_is_enable = true;
		ret = power_control(data, true);
		if (ret) {
			if (!READ_ONCE(data->power_uncertain)) {
				disable_all_muxes(data);
				data->mux_is_enable = false;
			}
			goto out;
		}
		atomic_set_release(&data->power_ready, 1);
	}
	data->power_users++;
out:
	mutex_unlock(&data->power_lock);
	return ret;
}

static int vmm_power_put(struct dvfs_driver_data *data)
{
	int ret = 0;

	mutex_lock(&data->power_lock);
	if (!data->power_users) {
		ret = -EINVAL;
		goto out;
	}
	if (--data->power_users)
		goto out;
	/* Prevent new callback readers, then wait for in-flight IPC to finish.
	 * No regulator-core lock is held across this grace period or rproc_stop.
	 */
	atomic_set_release(&data->power_ready, 0);
	synchronize_srcu(&data->power_readers);
	if (READ_ONCE(data->power_uncertain)) {
		ret = -EIO;
		goto out;
	}
	ret = power_control(data, false);
	if (ret)
		goto out;
	if (data->mux_is_enable) {
		disable_all_muxes(data);
		data->mux_is_enable = false;
	}
	mutex_lock(&data->current_dvfs.voltage_mutex);
	data->current_dvfs.voltage_target = DEFAULT_VOLTAGE;
	mutex_unlock(&data->current_dvfs.voltage_mutex);
out:
	mutex_unlock(&data->power_lock);
	return ret;
}

int mtk_ispdvfs_regulator_enable(struct regulator *reg)
{
	struct dvfs_driver_data *data = READ_ONCE(power_provider);
	int ret, release_ret;

	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;
	if (!data)
		return -EPROBE_DEFER;
	ret = vmm_power_get(data);
	if (ret)
		return ret;
	ret = regulator_enable(reg);
	if (ret) {
		release_ret = vmm_power_put(data);
		if (release_ret)
			dev_err(data->dev, "power rollback failed: %d (enable %d)\n",
				release_ret, ret);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_ispdvfs_regulator_enable);

int mtk_ispdvfs_regulator_disable(struct regulator *reg)
{
	struct dvfs_driver_data *data = READ_ONCE(power_provider);
	int ret;

	if (IS_ERR_OR_NULL(reg))
		return -EINVAL;
	if (!data)
		return -ENODEV;
	ret = regulator_disable(reg);
	if (ret)
		return ret;
	return vmm_power_put(data);
}
EXPORT_SYMBOL_GPL(mtk_ispdvfs_regulator_disable);

static int vmm_enable_regulator(struct regulator_dev *rdev)
{
	struct vmm_regulator *reg = rdev_get_drvdata(rdev);
	struct dvfs_driver_data *data;
	int token, ret = 0;

	if (!reg || !reg->dvfs_data)
		return -ENODEV;
	data = reg->dvfs_data;
	token = srcu_read_lock(&data->power_readers);
	if (READ_ONCE(data->power_uncertain))
		ret = -EIO;
	else if (!atomic_read_acquire(&data->power_ready))
		ret = -EAGAIN;
	else
		reg->is_enable = 1;
	srcu_read_unlock(&data->power_readers, token);
	return ret;
}

static int vmm_disable_regulator(struct regulator_dev *rdev)
{
	struct vmm_regulator *reg = rdev_get_drvdata(rdev);

	if (!reg || !reg->dvfs_data)
		return -ENODEV;
	/* Firmware/clock release belongs to the outer consumer, after core locks. */
	if (READ_ONCE(reg->dvfs_data->power_uncertain))
		return -EIO;
	reg->is_enable = 0;
	return 0;
}

static int vmm_is_enabled(struct regulator_dev *rdev)
{
	struct vmm_regulator *regulator = rdev_get_drvdata(rdev);

	return regulator->is_enable;
}

static const struct regulator_ops vmm_apmcu_ops = {
	.list_voltage = regulator_list_voltage_table,
	.set_voltage = apmcu_set_voltage,
	.get_voltage = vmm_get_voltage,
	.enable = vmm_enable_regulator,
	.disable = vmm_disable_regulator,
	.is_enabled = vmm_is_enabled,
};

static const struct regulator_ops vmm_ccu_ops = {
	.list_voltage = regulator_list_voltage_table,
	.set_voltage = ccu_set_voltage,
	.get_voltage = vmm_get_voltage,
	.enable = vmm_enable_regulator,
	.disable = vmm_disable_regulator,
	.is_enabled = vmm_is_enabled,
};

static struct vmm_regulator platform_regulators = {
	CREATE_REGULATOR("vmm-proxy", VMM),
};

static const struct of_device_id mtk_vmm_regulator_match[] = {
	{
		.compatible = "mediatek,ispdvfs",
		.data = &platform_regulators,
	}, {
		/* sentinel */
	},
};

DEFINE_SIMPLE_ATTRIBUTE(force_voltage_ops, NULL, force_voltage, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(force_opp_level_ops, NULL, force_opp_level, "%llu\n");
static void vmm_cleanup_readers(void *arg)
{
	struct dvfs_driver_data *data = arg;
	cleanup_srcu_struct(&data->power_readers);
}

static int vmm_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = {};
	struct regulator_dev *rdev;
	struct vmm_regulator *reg;
	struct dvfs_driver_data *data;
	struct property *prop, *src_prop;
	const char *name, *src_name;
	char key[64];
	u32 micro = 0, i = 0, j;
	int ret;

	if (vmm_reg && !IS_ERR(vmm_reg))
		return -EBUSY;
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	reg = devm_kmemdup(dev, &platform_regulators, sizeof(*reg), GFP_KERNEL);
	if (!data || !reg)
		return -ENOMEM;
	data->dev = dev;
	ret = vmm_init_opp_table(data);
	if (ret)
		return ret;
	ret = init_srcu_struct(&data->power_readers);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, vmm_cleanup_readers, data);
	if (ret)
		return ret;
	mutex_init(&data->power_lock);
	mutex_init(&data->current_dvfs.voltage_mutex);
	data->current_dvfs.opp_level = DEFAULT_VOLTAGE_LEVEL;
	data->current_dvfs.voltage_target = data->opp_table.voltage[DEFAULT_VOLTAGE_LEVEL];
	of_property_for_each_string(dev->of_node, "mediatek,support_mux", prop, name) {
		if (i >= MAX_MUX_NUM)
			return -EINVAL;
		data->muxes[i].mux = devm_clk_get(dev, name);
		if (IS_ERR(data->muxes[i].mux))
			return PTR_ERR(data->muxes[i].mux);
		data->muxes[i].mux_name = name;
		if (snprintf(key, sizeof(key), "mediatek,mux_%s", name) >= sizeof(key))
			return -EINVAL;
		j = 0;
		of_property_for_each_string(dev->of_node, key, src_prop, src_name) {
			if (j >= MAX_OPP_STEP)
				return -EINVAL;
			data->muxes[i].clk_src[j] = devm_clk_get(dev, src_name);
			if (IS_ERR(data->muxes[i].clk_src[j]))
				return PTR_ERR(data->muxes[i].clk_src[j]);
			j++;
		}
		if (j != data->opp_table.opp_num)
			return -EINVAL;
		i++;
	}
	if (!i)
		return -EINVAL;
	data->num_muxes = i;
	/* Only the observed xaga CCU-owned policy is qualified by this port. */
	ret = of_property_read_u32(dev->of_node, "mediatek,support_micro_processor", &micro);
	if (ret || micro != 1)
		return -EOPNOTSUPP;
	ret = of_property_read_u32(dev->of_node, "mediatek,disable_dvfs", &data->disable_dvfs);
	if (ret || data->disable_dvfs)
		return -EOPNOTSUPP;
	of_property_read_u32(dev->of_node, "simulate_aging", &data->simulate_aging);
	of_property_read_u32(dev->of_node, "en_vb", &data->en_vb);
	/* This native provider is also used by the preserved diagnostic code. */
	vmm_reg = devm_regulator_get(dev, "buck-vmm");
	if (IS_ERR(vmm_reg)) {
		ret = PTR_ERR(vmm_reg);
		vmm_reg = NULL;
		return ret;
	}
	reg->dvfs_data = data;
	reg->desc.ops = &vmm_ccu_ops;
	reg->desc.n_voltages = data->opp_table.opp_num;
	reg->desc.volt_table = data->opp_table.voltage;
	config.dev = dev;
	config.driver_data = reg;
	rdev = devm_regulator_register(dev, &reg->desc, &config);
	if (IS_ERR(rdev)) {
		vmm_reg = NULL;
		return PTR_ERR(rdev);
	}
	/* Debug force interfaces are not published by this bring-up profile.
	 * Preserve their code for review; no voltage-forcing interface is needed
	 * to provide the real regulator to native camera consumers. */
	platform_set_drvdata(pdev, data);
	WRITE_ONCE(power_provider, data);
	__module_get(THIS_MODULE);
	return 0;
}

static struct platform_driver mtk_vmm_regulator_driver = {
	.driver = {
		.name  = "mtk-vmm-regulator",
		.owner = THIS_MODULE,
		.suppress_bind_attrs = true,
		.of_match_table = mtk_vmm_regulator_match,
	},
	.probe = vmm_regulator_probe,
};

static int __init mtk_vmm_regulator_init(void)
{
	s32 status;

	status = platform_driver_register(&mtk_vmm_regulator_driver);
	if (status) {
		ISP_LOGE("Failed to register VMM driver(%d)\n", status);
		return -ENODEV;
	}
	return 0;
}
subsys_initcall(mtk_vmm_regulator_init);

static void __exit mtk_vmm_regulator_exit(void)
{
	platform_driver_unregister(&mtk_vmm_regulator_driver);
}
module_exit(mtk_vmm_regulator_exit);

MODULE_AUTHOR("Yuan Jung Kuo <yuan-jung.kuo@mediatek.com>");
MODULE_DESCRIPTION("VMM regulator driver");
MODULE_LICENSE("GPL v2");
