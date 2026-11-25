// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "emi_mpu.h"
#include "emi-mpu-owner.h"
#include "emi-mpu-md-callback.h"
#include <linux/cleanup.h>
#include <linux/arm-smccc.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <mt-plat/aee.h>
#include <soc/mediatek/emi.h>

#define EMI_MPUS_OFFSET	(0x1F0)
#define EMI_MPUT_OFFSET	(0x1F8)
#define EMI_MPUT_2ND_OFFSET	(0x1FC)

/* global pointer for exported functions */
struct emi_mpu *global_emi_mpu;
EXPORT_SYMBOL_GPL(global_emi_mpu);

DEFINE_FREE(emimpu_reader, struct emimpu_owner *,
	if (_T) mtk_emimpu_owner_put(_T));








static void set_regs(
	struct reg_info_t *reg_list, unsigned int reg_cnt,
	void __iomem *emi_cen_base)
{
	unsigned int i, j;

	for (i = 0; i < reg_cnt; i++)
		for (j = 0; j < reg_list[i].leng; j++)
			writel(reg_list[i].value, emi_cen_base +
				reg_list[i].offset + 4 * j);

	/*
	 * Use the memory barrier to make sure the interrupt signal is
	 * de-asserted (by programming registers) before exiting the
	 * ISR and re-enabling the interrupt.
	 */
	mb();
}






/**
 * Check whether devmpu.
 *		CONFIG_MTK_DEVMPU_EMI is only on for mt6893 & mt6885.
 *		Check 0x1fc[21] || 0x1fc[22]; Suppose 0x1fc at mpu->dump_reg[2];
 */


/**
 * Clear devmpu violation at emi directly.
 *		clear DEVMPU related registers,
 *		common reigsters will be cleared in clear_violation().
 */






int emimpu_ap_region_init(void)
{
	struct emi_mpu *mpu;
	struct emimpu_owner *reader __free(emimpu_reader) = mtk_emimpu_owner_get();

	mpu = mtk_emimpu_owner_device(reader);
	if (!mpu)
		return 0;

	if (!(mpu->ap_rg_info))
		return 0;

	pr_info("%s: enable AP region\n", __func__);

	mtk_emimpu_set_protection(mpu->ap_rg_info);
	mtk_emimpu_free_region(mpu->ap_rg_info);

	kfree(mpu->ap_rg_info);
	mpu->ap_rg_info = NULL;

	return 0;
}
EXPORT_SYMBOL(emimpu_ap_region_init);

/*
 * mtk_emimpu_init_region - init rg_info's apc data with default forbidden
 * @rg_info:	the target region for init
 * @rg_num:	the region id for the rg_info
 *
 * Returns 0 on success, -EINVAL if rg_info or rg_num is invalid,
 * -ENODEV if the emi-mpu driver is not probed successfully,
 * -ENOMEM if out of memory
 */
int mtk_emimpu_init_region(
	struct emimpu_region_t *rg_info, unsigned int rg_num)
{
	struct emi_mpu *mpu;
	struct emimpu_owner *reader __free(emimpu_reader) = mtk_emimpu_owner_get();
	unsigned int i;

	if (rg_info)
		memset(rg_info, 0, sizeof(struct emimpu_region_t));
	else
		return -EINVAL;

	mpu = mtk_emimpu_owner_device(reader);
	if (!mpu)
		return -ENODEV;

	if (!mpu->domain_cnt || mpu->domain_cnt > 2048 ||
	    mpu->domain_cnt % 8 || mpu->addr_align >= 32 || mpu->region_cnt > 256)
		return -EINVAL;

	if (rg_num >= mpu->region_cnt) {
		pr_info("%s: fail, out-of-range region\n", __func__);
		return -EINVAL;
	}

	rg_info->apc = kmalloc_array(mpu->domain_cnt, sizeof(*rg_info->apc), GFP_KERNEL);
	if (!(rg_info->apc))
		return -ENOMEM;
	for (i = 0; i < mpu->domain_cnt; i++)
		rg_info->apc[i] = MTK_EMIMPU_FORBIDDEN;

	rg_info->rg_num = rg_num;
	rg_info->owner = reader;
	reader = NULL;

	return 0;
}
EXPORT_SYMBOL(mtk_emimpu_init_region);

/*
 * mtk_emi_mpu_free_region - free the apc data in rg_info
 * @rg_info:	the target region for free
 *
 * Returns 0 on success, -EINVAL if rg_info is invalid
 */
int mtk_emimpu_free_region(struct emimpu_region_t *rg_info)
{
	struct emimpu_owner *owner;

	if (!rg_info || !rg_info->apc || !rg_info->owner)
		return -EINVAL;
	owner = rg_info->owner;
	kfree(rg_info->apc);
	rg_info->apc = NULL;
	rg_info->owner = NULL;
	mtk_emimpu_owner_put(owner);
	return 0;
}
EXPORT_SYMBOL(mtk_emimpu_free_region);

/*
 * mtk_emimpu_set_addr - set the address space
 * @rg_info:	the target region for address setting
 * @start:	the start address
 * @end:	the end address
 *
 * Returns 0 on success, -EINVAL if rg_info is invalid
 */
int mtk_emimpu_set_addr(struct emimpu_region_t *rg_info,
	unsigned long long start, unsigned long long end)
{
	if (rg_info) {
		rg_info->start = start;
		rg_info->end = end;
		return 0;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL(mtk_emimpu_set_addr);

/*
 * mtk_emimpu_set_apc - set access permission for target domain
 * @rg_info:	the target region for apc setting
 * @d_num:	the target domain id
 * @apc:	the access permission setting
 *
 * Returns 0 on success, -EINVAL if rg_info or d_num is invalid,
 * -ENODEV if the emi-mpu driver is not probed successfully
 */
int mtk_emimpu_set_apc(struct emimpu_region_t *rg_info,
	unsigned int d_num, unsigned int apc)
{
	struct emi_mpu *mpu;

	if (!rg_info || !rg_info->apc)
		return -EINVAL;

	mpu = mtk_emimpu_owner_device(rg_info->owner);
	if (!mpu)
		return -ENODEV;

	if (d_num >= mpu->domain_cnt) {
		pr_info("%s: fail, out-of-range domain\n", __func__);
		return -EINVAL;
	}

	rg_info->apc[d_num] = apc & 0x7;

	return 0;
}
EXPORT_SYMBOL(mtk_emimpu_set_apc);

/*
 * mtk_emimpu_lock_region - set lock for target region
 * @rg_info:	the target region for lock
 * @lock:	enable/disable lock
 *
 * Returns 0 on success, -EINVAL if rg_info is invalid
 */
int mtk_emimpu_lock_region(struct emimpu_region_t *rg_info, bool lock)
{
	if (rg_info) {
		rg_info->lock = lock;
		return 0;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL(mtk_emimpu_lock_region);

/*
 * mtk_emimpu_set_protection - set emimpu protect into device
 * @rg_info:	the target region information
 *
 * Returns 0 on success, -EINVAL if rg_info is invalid,
 * -ENODEV if the emi-mpu driver is not probed successfully,
 * -EPERM if the SMC call returned failure
 */
int mtk_emimpu_set_protection(struct emimpu_region_t *rg_info)
{
	struct emi_mpu *mpu;
	unsigned int start, end;
	unsigned int group_apc;
	unsigned int d_group;
	unsigned int align_mask;
	struct arm_smccc_res smc_res;
	int i, j;

	if (!rg_info) {
		pr_info("%s: fail, invalid emimpu_region_t.\n", __func__);
		return -EINVAL;
	}

	if (!(rg_info->apc)) {
		pr_info("%s: fail, protect without init\n", __func__);
		return -EINVAL;
	}

	mpu = mtk_emimpu_owner_device(rg_info->owner);
	if (!mpu)
		return -ENODEV;

	/**
	 * Check alignment of start & size.
	 *	Take 64KB align as example (start: 0x40010000, size: 0x00020000)
	 *		=> [start, end] = [0x40010000, 0x4002FFFF]
	 */
	pr_info("%s: rg(%u, [%#llx, %#llx]), align_bit(%u).\n", __func__, rg_info->rg_num,
		rg_info->start, rg_info->end, mpu->addr_align);
	if (rg_info->rg_num >= mpu->region_cnt || mpu->addr_align >= 32 ||
	    rg_info->start > rg_info->end ||
	    (rg_info->start >> mpu->addr_align) > 0xffffff ||
	    (rg_info->end >> mpu->addr_align) > 0xffffff)
		return -EINVAL;
	align_mask = (0x1u << mpu->addr_align) - 1;
	if ((0x0 != (align_mask & rg_info->start)) ||
		(align_mask != (align_mask & rg_info->end))) {
		pr_err("%s: region(%u, [%#llx, %#llx]) is not align!",
			__func__, rg_info->rg_num, rg_info->start, rg_info->end);
		return -EINVAL;
	}

	d_group = mpu->domain_cnt / 8;

	start = (unsigned int)
		(rg_info->start >> (mpu->addr_align)) |
		(rg_info->rg_num << 24);

	for (i = d_group - 1; i >= 0; i--) {
		end = (unsigned int)
			(rg_info->end >> (mpu->addr_align)) |
			(i << 24);

		for (group_apc = 0, j = 0; j < 8; j++)
			group_apc |=
				((rg_info->apc[i * 8 + j]) & 0x7) << (3 * j);
		if ((i == 0) && rg_info->lock)
			group_apc |= 0x80000000;

		arm_smccc_smc(MTK_SIP_EMIMPU_CONTROL, MTK_EMIMPU_SET,
			start, end, group_apc, 0, 0, 0, &smc_res);
		if (smc_res.a0) {
			pr_info("%s:%d failed to set region permission, ret=0x%lx\n",
				__func__, __LINE__, smc_res.a0);
			return -EPERM;
		}

	}

	return 0;
}
EXPORT_SYMBOL(mtk_emimpu_set_protection);

/*
 * mtk_emimpu_clear_protection - clear emimpu protection
 * @rg_info:	the target region information
 *
 * Returns 0 on success, -EINVAL if rg_info is invalid,
 * -ENODEV if the emi-mpu driver is not probed successfully,
 * -EPERM if the SMC call returned failure
 */
int mtk_emimpu_clear_protection(struct emimpu_region_t *rg_info)
{
	struct emi_mpu *mpu;
	struct arm_smccc_res smc_res;

	if (!rg_info)
		return -EINVAL;

	mpu = mtk_emimpu_owner_device(rg_info->owner);
	if (!mpu)
		return -ENODEV;

	if (rg_info->rg_num >= mpu->region_cnt) {
		pr_info("%s: region %u overflow\n", __func__, rg_info->rg_num);
		return -EINVAL;
	}

	arm_smccc_smc(MTK_SIP_EMIMPU_CONTROL, MTK_EMIMPU_CLEAR,
		rg_info->rg_num, 0, 0, 0, 0, 0, &smc_res);
	if (smc_res.a0) {
		pr_info("%s:%d failed to clear region permission, ret=0x%lx\n",
			__func__, __LINE__, smc_res.a0);
		return -EPERM;
	}
	return 0;
}
EXPORT_SYMBOL(mtk_emimpu_clear_protection);

/*
 * mtk_emimpu_prehandle_register - register callback for irq prehandler
 * @bypass_func:	function point for prehandler
 *
 * Return 0 for success, -EINVAL for fail
 */
int mtk_emimpu_prehandle_register(emimpu_pre_handler bypass_func)
{
	struct emi_mpu *mpu;
	struct emimpu_owner *reader __free(emimpu_reader) = mtk_emimpu_owner_get();

	mpu = mtk_emimpu_owner_device(reader);
	if (!mpu)
		return -EINVAL;

	if (!bypass_func) {
		pr_info("%s: bypass_func is NULL\n", __func__);
		return -EINVAL;
	}

	mpu->pre_handler = bypass_func;

	return 0;
}
EXPORT_SYMBOL(mtk_emimpu_prehandle_register);

/*
 * mtk_emimpu_postclear_register - register callback for clear posthandler
 * @clear_func:	function point for posthandler
 *
 * Return 0 for success, -EINVAL for fail
 */
int mtk_emimpu_postclear_register(emimpu_post_clear clear_func)
{
	struct emi_mpu *mpu;
	struct emimpu_owner *reader __free(emimpu_reader) = mtk_emimpu_owner_get();

	mpu = mtk_emimpu_owner_device(reader);
	if (!mpu)
		return -EINVAL;

	if (!clear_func) {
		pr_info("%s: clear_func is NULL\n", __func__);
		return -EINVAL;
	}

	mpu->post_clear = clear_func;

	return 0;
}
EXPORT_SYMBOL(mtk_emimpu_postclear_register);

/*
 * mtk_emimpu_tmem_handler_register - register callback for tmem handling
 * @tmem_handler_func:   function point for tmem handling
 *
 * Return 0 for success, -EINVAL or -ENOMEN for fail
 *
 */
int mtk_emimpu_tmem_handler_register(emimpu_tmem_handler tmem_handler_func)
{
	struct emi_mpu *mpu;
	struct emimpu_owner *reader __free(emimpu_reader) = mtk_emimpu_owner_get();

	mpu = mtk_emimpu_owner_device(reader);
	if(!mpu)
		return -EINVAL;

	if(!tmem_handler_func) {
		pr_info("%s: tmem_handling_func is NULL\n", __func__);
		return -EINVAL;
	}

	mpu->tmem_handler = tmem_handler_func;
	pr_info("%s: tmem_handling_func registered!!\n", __func__);

	return 0;
}
EXPORT_SYMBOL(mtk_emimpu_tmem_handler_register);

/*
 * mtk_emimpu_md_handling_register - register callback for md handling
 * @md_handling_func:	function point for md handling
 *
 * Return 0 for success, -EINVAL for fail
 */
int mtk_emimpu_md_handling_register(
		emimpu_md_handler callback, struct module *owner)
{
	return mtk_emimpu_md_callback_register(callback, owner);
}
EXPORT_SYMBOL(mtk_emimpu_md_handling_register);

/*
 * mtk_clear_md_violation - clear irq for md violation
 *
 * No return
 */
void mtk_clear_md_violation(void)
{
	struct emi_mpu *mpu;
	struct emimpu_owner *reader __free(emimpu_reader) = mtk_emimpu_owner_get();
	void __iomem *emi_cen_base;
	unsigned int emi_id;
	struct arm_smccc_res smc_res;

	mpu = mtk_emimpu_owner_device(reader);
	if (!mpu)
		return;
	if (mpu->version == EMIMPUVER2) {
		arm_smccc_smc(MTK_SIP_EMIMPU_CONTROL, MTK_EMIMPU_CLEAR_MD,
			0, 0, 0, 0, 0, 0, &smc_res);
		if (smc_res.a0) {
			pr_info("%s:%d failed to clear md violation, ret=0x%lx\n",
				__func__, __LINE__, smc_res.a0);
			return;
		}
	} else {
		for (emi_id = 0; emi_id < mpu->emi_cen_cnt; emi_id++) {
			emi_cen_base = mpu->emi_cen_base[emi_id];

			set_regs(mpu->clear_md_reg,
				mpu->clear_md_reg_cnt, emi_cen_base);
		}
	}

	pr_info("%s:version %d\n", __func__, mpu->version);
}
EXPORT_SYMBOL(mtk_clear_md_violation);

/*
 * mtk_emimpu_debugdump_register - register callback for debug info dump
 * @debug_func:	function point for debug info dump
 *
 * Return 0 for success, -EINVAL for fail
 */
int mtk_emimpu_debugdump_register(emimpu_debug_dump debug_func)
{
	struct emimpu_dbg_cb *targ_dbg_cb;
	struct emimpu_dbg_cb *curr_dbg_cb;
	struct emi_mpu *mpu;
	struct emimpu_owner *reader __free(emimpu_reader) = mtk_emimpu_owner_get();

	mpu = mtk_emimpu_owner_device(reader);
	if (!mpu)
		return -EINVAL;

	if (!debug_func) {
		pr_info("%s: debug_func is NULL\n", __func__);
		return -EINVAL;
	}

	targ_dbg_cb = kmalloc(sizeof(struct emimpu_dbg_cb), GFP_KERNEL);
	if (!targ_dbg_cb)
		return -ENOMEM;

	targ_dbg_cb->func = debug_func;
	targ_dbg_cb->next_dbg_cb = NULL;

	if (!(mpu->dbg_cb_list)) {
		mpu->dbg_cb_list = targ_dbg_cb;
		return 0;
	}

	for (curr_dbg_cb = mpu->dbg_cb_list; curr_dbg_cb;
		curr_dbg_cb = curr_dbg_cb->next_dbg_cb) {
		if (!(curr_dbg_cb->next_dbg_cb)) {
			curr_dbg_cb->next_dbg_cb = targ_dbg_cb;
			break;
		}
	}

	return 0;
}
EXPORT_SYMBOL(mtk_emimpu_debugdump_register);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Isolated xaga v2 common EMI helper ownership profile");
