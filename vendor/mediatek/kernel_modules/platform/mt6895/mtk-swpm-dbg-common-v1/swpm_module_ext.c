/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/ktime.h>
#include <linux/srcu.h>
#include <linux/mutex.h>

#include <swpm_module.h>
#include <swpm_module_ext.h>

#define SWPM_EXT_OPS (swpm_ext_m.plat_ops)

struct swpm_ext_manager {
	bool func_ready;
	struct swpm_internal_ops *plat_ops;
};

static struct swpm_ext_manager swpm_ext_m;
static DEFINE_MUTEX(swpm_ext_lock);
DEFINE_STATIC_SRCU(swpm_ext_srcu);

int32_t sync_latest_data(void)
{
 struct swpm_internal_ops *ops;
 int idx = srcu_read_lock(&swpm_ext_srcu);
 int32_t ret = -ENODEV;
 ops = READ_ONCE(swpm_ext_m.plat_ops);
 if (ops) {
  ops->cmd(SYNC_DATA, 0); ret = 0;
 }
 srcu_read_unlock(&swpm_ext_srcu, idx);
 return ret;
}
EXPORT_SYMBOL(sync_latest_data);
int32_t get_ddr_act_times(int32_t freq_num, struct ddr_act_times *ddr_times)
{
 struct swpm_internal_ops *ops;
 int idx = srcu_read_lock(&swpm_ext_srcu);
 int32_t ret = -ENODEV;
 ops = READ_ONCE(swpm_ext_m.plat_ops);
 if (ops) {
  ret = ops->ddr_act_times_get(freq_num, ddr_times);
 }
 srcu_read_unlock(&swpm_ext_srcu, idx);
 return ret;
}
EXPORT_SYMBOL(get_ddr_act_times);
int32_t get_ddr_sr_pd_times(struct ddr_sr_pd_times *ddr_times)
{
 struct swpm_internal_ops *ops;
 int idx = srcu_read_lock(&swpm_ext_srcu);
 int32_t ret = -ENODEV;
 ops = READ_ONCE(swpm_ext_m.plat_ops);
 if (ops) {
  ret = ops->ddr_sr_pd_times_get(ddr_times);
 }
 srcu_read_unlock(&swpm_ext_srcu, idx);
 return ret;
}
EXPORT_SYMBOL(get_ddr_sr_pd_times);
int32_t get_ddr_data_ip_num(void)
{
 struct swpm_internal_ops *ops;
 int idx = srcu_read_lock(&swpm_ext_srcu);
 int32_t ret = -ENODEV;
 ops = READ_ONCE(swpm_ext_m.plat_ops);
 if (ops) {
  ret = ops->num_get(DDR_DATA_IP);
 }
 srcu_read_unlock(&swpm_ext_srcu, idx);
 return ret;
}
EXPORT_SYMBOL(get_ddr_data_ip_num);
int32_t get_ddr_freq_num(void)
{
 struct swpm_internal_ops *ops;
 int idx = srcu_read_lock(&swpm_ext_srcu);
 int32_t ret = -ENODEV;
 ops = READ_ONCE(swpm_ext_m.plat_ops);
 if (ops) {
  ret = ops->num_get(DDR_FREQ);
 }
 srcu_read_unlock(&swpm_ext_srcu, idx);
 return ret;
}
EXPORT_SYMBOL(get_ddr_freq_num);
int32_t get_ddr_freq_data_ip_stats(int32_t data_ip_num, int32_t freq_num, void *stats)
{
 struct swpm_internal_ops *ops;
 int idx = srcu_read_lock(&swpm_ext_srcu);
 int32_t ret = -ENODEV;
 ops = READ_ONCE(swpm_ext_m.plat_ops);
 if (ops) {
  ret = ops->ddr_freq_data_ip_stats_get(data_ip_num, freq_num, stats);
 }
 srcu_read_unlock(&swpm_ext_srcu, idx);
 return ret;
}
EXPORT_SYMBOL(get_ddr_freq_data_ip_stats);
int32_t get_vcore_ip_num(void)
{
 struct swpm_internal_ops *ops;
 int idx = srcu_read_lock(&swpm_ext_srcu);
 int32_t ret = -ENODEV;
 ops = READ_ONCE(swpm_ext_m.plat_ops);
 if (ops) {
  ret = ops->num_get(CORE_IP);
 }
 srcu_read_unlock(&swpm_ext_srcu, idx);
 return ret;
}
EXPORT_SYMBOL(get_vcore_ip_num);
int32_t get_vcore_vol_num(void)
{
 struct swpm_internal_ops *ops;
 int idx = srcu_read_lock(&swpm_ext_srcu);
 int32_t ret = -ENODEV;
 ops = READ_ONCE(swpm_ext_m.plat_ops);
 if (ops) {
  ret = ops->num_get(CORE_VOL);
 }
 srcu_read_unlock(&swpm_ext_srcu, idx);
 return ret;
}
EXPORT_SYMBOL(get_vcore_vol_num);
int32_t get_vcore_ip_vol_stats(int32_t ip_num, int32_t vol_num, void *stats)
{
 struct swpm_internal_ops *ops;
 int idx = srcu_read_lock(&swpm_ext_srcu);
 int32_t ret = -ENODEV;
 ops = READ_ONCE(swpm_ext_m.plat_ops);
 if (ops) {
  ret = ops->vcore_ip_vol_stats_get(ip_num, vol_num, stats);
 }
 srcu_read_unlock(&swpm_ext_srcu, idx);
 return ret;
}
EXPORT_SYMBOL(get_vcore_ip_vol_stats);
int32_t get_vcore_vol_duration(int32_t vol_num, struct vol_duration *duration)
{
 struct swpm_internal_ops *ops;
 int idx = srcu_read_lock(&swpm_ext_srcu);
 int32_t ret = -ENODEV;
 ops = READ_ONCE(swpm_ext_m.plat_ops);
 if (ops) {
  ret = ops->vcore_vol_duration_get(vol_num, duration);
 }
 srcu_read_unlock(&swpm_ext_srcu, idx);
 return ret;
}
EXPORT_SYMBOL(get_vcore_vol_duration);
int mtk_register_swpm_ops(struct swpm_internal_ops *ops)
{
 int ret = 0;
 if (!ops || !ops->cmd || !ops->ddr_act_times_get || !ops->ddr_sr_pd_times_get ||
     !ops->ddr_freq_data_ip_stats_get || !ops->vcore_ip_vol_stats_get ||
     !ops->vcore_vol_duration_get || !ops->num_get) return -EINVAL;
 mutex_lock(&swpm_ext_lock);
 if (swpm_ext_m.plat_ops) ret = -EBUSY;
 else { swpm_ext_m.func_ready = true; WRITE_ONCE(swpm_ext_m.plat_ops, ops); }
 mutex_unlock(&swpm_ext_lock);
 return ret;
}
EXPORT_SYMBOL(mtk_register_swpm_ops);
void mtk_unregister_swpm_ops(struct swpm_internal_ops *ops)
{
 mutex_lock(&swpm_ext_lock);
 if (swpm_ext_m.plat_ops == ops) {
  WRITE_ONCE(swpm_ext_m.plat_ops, NULL);
  swpm_ext_m.func_ready = false;
 }
 mutex_unlock(&swpm_ext_lock);
 synchronize_srcu(&swpm_ext_srcu);
}
EXPORT_SYMBOL_GPL(mtk_unregister_swpm_ops);
