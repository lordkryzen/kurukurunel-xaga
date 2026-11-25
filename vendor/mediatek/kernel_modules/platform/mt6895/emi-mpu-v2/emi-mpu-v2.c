// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include "emi_mpu.h"
#include "emi-mpu-owner.h"
#include "emi-mpu-md-callback.h"
#include <linux/cleanup.h>

DEFINE_FREE(emimpu_reader, struct emimpu_owner *, if (_T) mtk_emimpu_owner_put(_T));
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


struct emimpu_v2_context {
	struct emi_mpu mpu;
	struct work_struct work;
};

static void emimpu_put_node(void *data)
{
	of_node_put(data);
}

static struct device_node *emimpu_get_node(struct device *dev,
		struct device_node *np, const char *property)
{
	struct device_node *node = of_parse_phandle(np, property, 0);
	int ret;

	if (!node)
		return ERR_PTR(-ENODEV);
	ret = devm_add_action_or_reset(dev, emimpu_put_node, node);
	if (ret)
		return ERR_PTR(ret);
	return node;
}

static void __iomem *emimpu_map_node(struct device *dev,
		struct device_node *node, unsigned int index)
{
	struct resource resource;
	void __iomem *base;
	int ret;

	ret = of_address_to_resource(node, index, &resource);
	if (ret)
		return ERR_PTR(ret);
	/* Shared EMI windows are also read by the central EMI provider. */
	base = devm_ioremap(dev, resource.start, resource_size(&resource));
	return base ? base : ERR_PTR(-ENOMEM);
}

static int emimpu_check_list(struct device_node *node, const char *name,
		resource_size_t bytes, bool tuples)
{
	int count = of_property_count_u32_elems(node, name);
	unsigned int stride = tuples ? 3 : 1;
	unsigned int i;
	u32 offset, length;

	if (count <= 0 || count % stride)
		return -EINVAL;
	for (i = 0; i < count; i += stride) {
		if (of_property_read_u32_index(node, name, i, &offset))
			return -EINVAL;
		length = 1;
		if (tuples && of_property_read_u32_index(node, name, i + 2, &length))
			return -EINVAL;
		if ((offset & 3) || !length || offset >= bytes ||
		    length > (bytes - offset) / sizeof(u32))
			return -EINVAL;
	}
	return 0;
}

static const char *emimpu_md_clear_name(struct device_node *node)
{
	return of_find_property(node, "clear_md", NULL) ? "clear_md" : "clear-md";
}

static int emimpu_check_resources(struct device_node *node,
		struct device_node *central, struct device_node *kp,
		struct device_node *secure)
{
	struct device_node *nodes[] = { central, kp, secure };
	struct resource resource;
	int count, i, j, ret;
	u32 hp_offset;

	count = of_property_count_elems_of_size(central, "reg", 4 * sizeof(u32));
	if (count <= 0 || of_property_count_elems_of_size(node, "reg",
			4 * sizeof(u32)) != count)
		return -EINVAL;
	if (of_property_count_u32_elems(node, "dump") < 3 ||
	    of_property_read_u32_index(node, "dump", 2, &hp_offset) || hp_offset != 0x1fc)
		return -EINVAL;
	for (j = 0; j < ARRAY_SIZE(nodes); j++) {
		if (of_property_count_elems_of_size(nodes[j], "reg", 4 * sizeof(u32)) != count)
			return -EINVAL;
		for (i = 0; i < count; i++) {
			ret = of_address_to_resource(nodes[j], i, &resource);
			if (ret)
				return ret;
			if (!resource_size(&resource))
				return -EINVAL;
			ret = emimpu_check_list(j ? nodes[j] : node, "dump",
				resource_size(&resource), false);
			if (ret)
				return ret;
			ret = emimpu_check_list(j ? nodes[j] : node, "clear",
				resource_size(&resource), true);
			if (ret)
				return ret;
			if (!j) {
				ret = emimpu_check_list(node, "clear_hp", resource_size(&resource), true);
				if (ret)
					return ret;
				ret = emimpu_check_list(node, emimpu_md_clear_name(node), resource_size(&resource), true);
				if (ret)
					return ret;
			}
		}
	}
	return 0;
}

static void emimpu_cancel_work(void *data)
{
	struct emimpu_v2_context *context = data;

	cancel_work_sync(&context->work);
}

static void emimpu_unpublish(void *data)
{
	cmpxchg(&global_emi_mpu, data, NULL);
	mtk_emimpu_owner_withdraw(data);
}

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

/*
 * mtk_emimpu_iommu_handling_register - register callback for iommu handling
 * @iommu_handling_func:	function point for md handling
 *
 * Return 0 for success, -EINVAL for fail
 */
int mtk_emimpu_iommu_handling_register(emimpu_iommu_handler iommu_handling_func)
{
	struct emi_mpu *mpu_v2;
	struct emimpu_owner *reader __free(emimpu_reader) = mtk_emimpu_owner_get();

	mpu_v2 = mtk_emimpu_owner_device(reader);

	if (!mpu_v2)
		return -EINVAL;

	if (!iommu_handling_func) {
		pr_info("%s: iommu_handling_func is NULL\n", __func__);
		return -EINVAL;
	}

	mpu_v2->iommu_handler = iommu_handling_func;

	pr_info("%s: iommu_handling_func registered!!\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(mtk_emimpu_iommu_handling_register);

/*
 * mtk_emimpu_isr_hook_register - register the by-platform hook function for ISR
 * @hook: the by-platform hook function
 *
 * Return 0 for success, -EINVAL for fail
 */
int mtk_emimpu_isr_hook_register(emimpu_isr_hook hook)
{
	struct emi_mpu *mpu_v2;
	struct emimpu_owner *reader __free(emimpu_reader) = mtk_emimpu_owner_get();

	mpu_v2 = mtk_emimpu_owner_device(reader);

	if (!mpu_v2)
		return -EINVAL;

	if (!hook) {
		pr_info("%s: hook is NULL\n", __func__);
		return -EINVAL;
	}

	mpu_v2->by_plat_isr_hook = hook;

	return 0;
}
EXPORT_SYMBOL_GPL(mtk_emimpu_isr_hook_register);

static void clear_violation(
	struct emi_mpu *mpu, unsigned int emi_id)
{
	void __iomem *emi_cen_base;
	void __iomem *miu_kp_base;
	void __iomem *miu_mpu_base;

	emi_cen_base = mpu->emi_cen_base[emi_id];
	miu_kp_base = mpu->miu_kp_base[emi_id];
	miu_mpu_base = mpu->miu_mpu_base[emi_id];

	set_regs(mpu->clear_reg,
		mpu->clear_reg_cnt, emi_cen_base);

	set_regs(mpu->clear_hp_reg,
		mpu->clear_hp_reg_cnt, emi_cen_base);

	set_regs(mpu->clear_miukp_reg,
		mpu->clear_miukp_reg_cnt, miu_kp_base);

	set_regs(mpu->clear_miumpu_reg,
		mpu->clear_miumpu_reg_cnt, miu_mpu_base);
}

static void emimpu_vio_dump(struct work_struct *work)
{
	struct emi_mpu *mpu;

	mpu = &container_of(work, struct emimpu_v2_context, work)->mpu;

	if (mpu->vio_msg)
		aee_kernel_exception("EMIMPU_V2", mpu->vio_msg);

	mpu->in_msg_dump = 0;
}


static irqreturn_t emimpu_violation_irq(int irq, void *dev_id)
{
	struct emi_mpu *mpu = (struct emi_mpu *)dev_id;
	struct reg_info_t *dump_reg = mpu->dump_reg;
	struct reg_info_t *miumpu_dump_reg = mpu->miumpu_dump_reg;
	void __iomem *emi_cen_base;
	void __iomem *miu_mpu_base;
	unsigned int emi_id, i;
	ssize_t msg_len;
	int nr_vio;
	bool violation, miu_violation;
	irqreturn_t irqret;
	const unsigned int hp_mask = 0x600000;
	char md_str[MTK_EMI_MAX_CMD_LEN + 13] = {'\0'};

	nr_vio = 0;
	msg_len = 0;

	for (emi_id = 0; emi_id < mpu->emi_cen_cnt; emi_id++) {
		violation = false;
		miu_violation = false;
		emi_cen_base = mpu->emi_cen_base[emi_id];
		for (i = 0; i < mpu->dump_cnt; i++) {
			dump_reg[i].value = readl(
				emi_cen_base + dump_reg[i].offset);
			if (dump_reg[i].value)
				violation = true;
		}

		if (!violation)
			continue;

		/*
		 * The hardware interrupt to service will be triggered by
		 * two sources: one is the new MIUMPU violation, another is
		 * the legacy EMIMPU violation.
		 * Determine whether this is a MIUMPU violation or a EMIMPU
		 * violation by checking HP_MODE.
		 * If this is a MIUMPU violation, the info to dump is from
		 * another set of registers.
		 */
		miu_violation = (dump_reg[2].value & hp_mask) ? true : false;
		if (miu_violation) {
			violation = false;
			miu_mpu_base = mpu->miu_mpu_base[emi_id];
			for (i = 0; i < mpu->miumpu_dump_cnt; i++) {
				miumpu_dump_reg[i].value = readl(
				miu_mpu_base + miumpu_dump_reg[i].offset);

				if (miumpu_dump_reg[i].value)
					violation = true;
			}

			if (!violation) {
				pr_info("%s: emi:%d smpu = 0", __func__, emi_id);
				goto clear_violation;
			}
		}

		/*
		 * Have one hook function for one platform for handling
		 * by-platform problems. For example, bypass this violation
		 * since it is a false alarm.
		 * If the violation needs to be bypassed, clear the violation
		 * but do NOT write logs in the vio_msg buffer to avoid generate
		 * a violation report.
		 */
		if (mpu->by_plat_isr_hook) {
			irqret = mpu->by_plat_isr_hook(emi_id,
			(miu_violation) ? miumpu_dump_reg : dump_reg,
			(miu_violation) ? mpu->miumpu_dump_cnt : mpu->dump_cnt);

			if (irqret == IRQ_HANDLED)
				goto clear_violation;
		}

		nr_vio++;

		if (miu_violation) {
			/* Dump MIUMPU violation info */
			if (msg_len < MTK_EMI_MAX_CMD_LEN)
				msg_len += scnprintf(mpu->vio_msg + msg_len,
					MTK_EMI_MAX_CMD_LEN - msg_len,
					"\n[MIUMPU]emiid%d\n", emi_id);
			for (i = 0; i < mpu->miumpu_dump_cnt; i++)
				if (msg_len < MTK_EMI_MAX_CMD_LEN)
					msg_len += scnprintf(mpu->vio_msg + msg_len,
						MTK_EMI_MAX_CMD_LEN - msg_len,
						"[%x]%x;",
						miumpu_dump_reg[i].offset,
						miumpu_dump_reg[i].value);
		} else {
			/* Dump EMIMPU violation info */
			if (msg_len < MTK_EMI_MAX_CMD_LEN)
				msg_len += scnprintf(mpu->vio_msg + msg_len,
						MTK_EMI_MAX_CMD_LEN - msg_len,
						"\n[EMIMPU]emiid%d\n", emi_id);
			for (i = 0; i < mpu->dump_cnt; i++)
				if (msg_len < MTK_EMI_MAX_CMD_LEN)
					msg_len += scnprintf(mpu->vio_msg + msg_len,
						MTK_EMI_MAX_CMD_LEN - msg_len,
						"%s(%d),%s(%x),%s(%x);\n",
						"emi", emi_id,
						"off", dump_reg[i].offset,
						"val", dump_reg[i].value);
		}

		/*
		 * Whenever there is an EMI MPU violation, the Modem
		 * software would like to be notified immediately.
		 * This is because the Modem software wants to do
		 * its coredump as earlier as possible for debugging
		 * and analysis.
		 * (Even if the violated master is not Modem, it
		 *  may still need coredump for clarification.)
		 * Have a hook function in the EMI MPU ISR for this
		 * purpose.
		 */
		strscpy(md_str, "emi-mpu-v2.c", 13);
		strncat(md_str, mpu->vio_msg, sizeof(md_str) - strlen(md_str) - 1);
		mtk_emimpu_md_callback_notify(md_str);

		/*
		 * IOMMU IRQ not connected on mt6983, mt6879 and mt6895.
		 * IOMMU uses SRINFO=3 to make SMPU violation when
		 * translation fault. This behavior is security spec
		 * for IOMMU. Throuh having hook function, IOMMU will
		 * be nodified when violoation occurred.
		 */
		if (mpu->iommu_handler) {
			mpu->iommu_handler(emi_id,
				dump_reg, mpu->dump_cnt);
		}

clear_violation:
		clear_violation(mpu, emi_id);

	}

	if (nr_vio) {
		pr_info("%s: %s", __func__, mpu->vio_msg);
		mpu->in_msg_dump = 1;
		schedule_work(&container_of(mpu, struct emimpu_v2_context, mpu)->work);
	}

	return IRQ_HANDLED;
}

static const struct of_device_id emimpu_of_ids[] = {
	{.compatible = "mediatek,mt6983-emimpu",},
	{}
};
MODULE_DEVICE_TABLE(of, emimpu_of_ids);

static int emimpu_probe(struct platform_device *pdev)
{
	struct device_node *emimpu_node = pdev->dev.of_node;
	struct device_node *emicen_node, *miukp_node, *miumpu_node;
	struct emimpu_v2_context *context;
	struct emi_mpu *mpu;
	int ret, size, i;
	struct resource *res;
	unsigned int *dump_list, *miukp_dump_list, *miumpu_dump_list;

	dev_info(&pdev->dev, "driver probed\n");

	emicen_node = emimpu_get_node(&pdev->dev, emimpu_node, "mediatek,emi-reg");
	if (IS_ERR(emicen_node)) {
		dev_info(&pdev->dev, "No emi-reg\n");
		return PTR_ERR(emicen_node);
	}

	miukp_node = emimpu_get_node(&pdev->dev, emimpu_node, "mediatek,miukp-reg");
	if (IS_ERR(miukp_node)) {
		dev_info(&pdev->dev, "No miukp-reg\n");
		return PTR_ERR(miukp_node);
	}

	miumpu_node = emimpu_get_node(&pdev->dev, emimpu_node, "mediatek,miumpu-reg");
	if (IS_ERR(miumpu_node)) {
		dev_info(&pdev->dev, "No miumpu-reg\n");
		return PTR_ERR(miumpu_node);
	}

	ret = emimpu_check_resources(emimpu_node, emicen_node, miukp_node, miumpu_node);
	if (ret)
		return ret;
	context = devm_kzalloc(&pdev->dev, sizeof(*context), GFP_KERNEL);
	if (!context)
		return -ENOMEM;
	mpu = &context->mpu;
	INIT_WORK(&context->work, emimpu_vio_dump);

//dump emi start
	size = of_property_count_elems_of_size(emimpu_node,
		"dump", sizeof(char));
	if (size <= 0) {
		dev_info(&pdev->dev, "No dump\n");
		return -ENXIO;
	}
	dump_list = devm_kmalloc(&pdev->dev, size, GFP_KERNEL);
	if (!dump_list)
		return -ENOMEM;
	size >>= 2;
	mpu->dump_cnt = size;
	ret = of_property_read_u32_array(emimpu_node, "dump",
		dump_list, size);
	if (ret) {
		dev_info(&pdev->dev, "No emi dump\n");
		return -ENXIO;
	}
	mpu->dump_reg = devm_kmalloc(&pdev->dev,
		size * sizeof(struct reg_info_t), GFP_KERNEL);
	if (!(mpu->dump_reg))
		return -ENOMEM;
	for (i = 0; i < mpu->dump_cnt; i++) {
		mpu->dump_reg[i].offset = dump_list[i];
		mpu->dump_reg[i].value = 0;
		mpu->dump_reg[i].leng = 0;
	}
//dump emi end
//dump miu kp start
	size = of_property_count_elems_of_size(miukp_node,
		"dump", sizeof(char));
	if (size <= 0) {
		dev_info(&pdev->dev, "No miu kp dump\n");
		return -ENXIO;
	}
	miukp_dump_list = devm_kmalloc(&pdev->dev, size, GFP_KERNEL);
	if (!miukp_dump_list)
		return -ENOMEM;
	size >>= 2;
	mpu->miukp_dump_cnt = size;
	ret = of_property_read_u32_array(miukp_node, "dump",
		miukp_dump_list, size);
	if (ret) {
		dev_info(&pdev->dev, "No  miu kp dump\n");
		return -ENXIO;
	}
	mpu->miukp_dump_reg = devm_kmalloc(&pdev->dev,
		size * sizeof(struct reg_info_t), GFP_KERNEL);
	if (!(mpu->miukp_dump_reg))
		return -ENOMEM;
	for (i = 0; i < mpu->miukp_dump_cnt; i++) {
		mpu->miukp_dump_reg[i].offset = miukp_dump_list[i];
		mpu->miukp_dump_reg[i].value = 0;
		mpu->miukp_dump_reg[i].leng = 0;
	}
//dump miu kp end
//dump miu mpu start
	size = of_property_count_elems_of_size(miumpu_node,
		"dump", sizeof(char));
	if (size <= 0) {
		dev_info(&pdev->dev, "No miu mpu dump\n");
		return -ENXIO;
	}
	miumpu_dump_list = devm_kmalloc(&pdev->dev, size, GFP_KERNEL);
	if (!miumpu_dump_list)
		return -ENOMEM;
	size >>= 2;
	mpu->miumpu_dump_cnt = size;
	ret = of_property_read_u32_array(miumpu_node, "dump",
		miumpu_dump_list, size);
	if (ret) {
		dev_info(&pdev->dev, "No miu mpu dump\n");
		return -ENXIO;
	}
	mpu->miumpu_dump_reg = devm_kmalloc(&pdev->dev,
		size * sizeof(struct reg_info_t), GFP_KERNEL);
	if (!(mpu->miumpu_dump_reg))
		return -ENOMEM;
	for (i = 0; i < mpu->miumpu_dump_cnt; i++) {
		mpu->miumpu_dump_reg[i].offset = miumpu_dump_list[i];
		mpu->miumpu_dump_reg[i].value = 0;
		mpu->miumpu_dump_reg[i].leng = 0;
	}
//dump miu mpu end
//clear start
	size = of_property_count_elems_of_size(emimpu_node,
		"clear", sizeof(char));
	if (size <= 0) {
		dev_info(&pdev->dev, "No clear\n");
		return  -ENXIO;
	}
	mpu->clear_reg = devm_kmalloc(&pdev->dev,
		size, GFP_KERNEL);
	if (!(mpu->clear_reg))
		return -ENOMEM;
	mpu->clear_reg_cnt = size / sizeof(struct reg_info_t);
	size >>= 2;
	ret = of_property_read_u32_array(emimpu_node, "clear",
		(unsigned int *)(mpu->clear_reg), size);
	if (ret) {
		dev_info(&pdev->dev, "No clear\n");
		return -ENXIO;
	}
//clear end

//clear md start
	size = of_property_count_elems_of_size(emimpu_node,
		emimpu_md_clear_name(emimpu_node), sizeof(char));
	if (size <= 0) {
		dev_info(&pdev->dev, "No clear_md\n");
		return -ENXIO;
	}
	mpu->clear_md_reg = devm_kmalloc(&pdev->dev,
		size, GFP_KERNEL);
	if (!(mpu->clear_md_reg))
		return -ENOMEM;
	mpu->clear_md_reg_cnt = size / sizeof(struct reg_info_t);
	size >>= 2;
	ret = of_property_read_u32_array(emimpu_node, emimpu_md_clear_name(emimpu_node),
		(unsigned int *)(mpu->clear_md_reg), size);
	if (ret) {
		dev_info(&pdev->dev, "No clear_md\n");
		return -ENXIO;
	}
//clear md end
//clear hp start
	size = of_property_count_elems_of_size(emimpu_node,
		"clear_hp", sizeof(char));
	if (size <= 0) {
		dev_info(&pdev->dev, "No clear_hp\n");
		return -ENXIO;
	}
	mpu->clear_hp_reg = devm_kmalloc(&pdev->dev,
		size, GFP_KERNEL);
	if (!(mpu->clear_hp_reg))
		return -ENOMEM;
	mpu->clear_hp_reg_cnt = size / sizeof(struct reg_info_t);
	size >>= 2;
	ret = of_property_read_u32_array(emimpu_node, "clear_hp",
		(unsigned int *)(mpu->clear_hp_reg), size);
	if (ret) {
		dev_info(&pdev->dev, "No clear_hp\n");
		return -ENXIO;
	}
//clear hp end
//clear miukp start
	size = of_property_count_elems_of_size(miukp_node,
		"clear", sizeof(char));
	if (size <= 0) {
		dev_info(&pdev->dev, "No clear miu kp\n");
		return -ENXIO;
	}
	mpu->clear_miukp_reg = devm_kmalloc(&pdev->dev,
		size, GFP_KERNEL);
	if (!(mpu->clear_miukp_reg))
		return -ENOMEM;
	mpu->clear_miukp_reg_cnt = size / sizeof(struct reg_info_t);
	size >>= 2;
	ret = of_property_read_u32_array(miukp_node, "clear",
		(unsigned int *)(mpu->clear_miukp_reg), size);
	if (ret) {
		dev_info(&pdev->dev, "No clear miu kp\n");
		return -ENXIO;
	}
//clear miukp end
//clear miumpu start
	size = of_property_count_elems_of_size(miumpu_node,
		"clear", sizeof(char));
	if (size <= 0) {
		dev_info(&pdev->dev, "No clear miu mpu\n");
		return -ENXIO;
	}
	mpu->clear_miumpu_reg = devm_kmalloc(&pdev->dev,
		size, GFP_KERNEL);
	if (!(mpu->clear_miumpu_reg))
		return -ENOMEM;
	mpu->clear_miumpu_reg_cnt = size / sizeof(struct reg_info_t);
	size >>= 2;
	ret = of_property_read_u32_array(miumpu_node, "clear",
		(unsigned int *)(mpu->clear_miumpu_reg), size);
	if (ret) {
		dev_info(&pdev->dev, "No clear miu mpu\n");
		return -ENXIO;
	}
//clear miumpu end
//reg base start
	mpu->emi_cen_cnt = of_property_count_elems_of_size(
			emicen_node, "reg", sizeof(unsigned int) * 4);
	if (mpu->emi_cen_cnt <= 0) {
		dev_info(&pdev->dev, "No reg\n");
		return -ENXIO;
	}

	mpu->emi_cen_base = devm_kmalloc_array(&pdev->dev,
		mpu->emi_cen_cnt, sizeof(void __iomem *), GFP_KERNEL);
	if (!(mpu->emi_cen_base))
		return -ENOMEM;

	mpu->emi_mpu_base = devm_kmalloc_array(&pdev->dev,
		mpu->emi_cen_cnt, sizeof(void __iomem *), GFP_KERNEL);
	if (!(mpu->emi_mpu_base))
		return -ENOMEM;

	mpu->miu_kp_base = devm_kmalloc_array(&pdev->dev,
		mpu->emi_cen_cnt, sizeof(void __iomem *), GFP_KERNEL);
	if (!(mpu->miu_kp_base))
		return -ENOMEM;

	mpu->miu_mpu_base = devm_kmalloc_array(&pdev->dev,
		mpu->emi_cen_cnt, sizeof(void __iomem *), GFP_KERNEL);
	if (!(mpu->miu_mpu_base))
		return -ENOMEM;

	for (i = 0; i < mpu->emi_cen_cnt; i++) {
		mpu->emi_cen_base[i] = emimpu_map_node(&pdev->dev, emicen_node, i);
		if (IS_ERR(mpu->emi_cen_base[i])) {
			dev_info(&pdev->dev, "Failed to map EMI%d CEN base\n",
				i);
			return -EIO;
		}

		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		mpu->emi_mpu_base[i] =
			devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(mpu->emi_mpu_base[i])) {
			dev_info(&pdev->dev, "Failed to map EMI%d MPU base\n",
				i);
			return -EIO;
		}

		mpu->miu_kp_base[i] = emimpu_map_node(&pdev->dev, miukp_node, i);
		if (IS_ERR(mpu->miu_kp_base[i])) {
			dev_info(&pdev->dev, "Failed to map MIU%d kernel protection base\n",
				i);
			return -EIO;
		}

		mpu->miu_mpu_base[i] = emimpu_map_node(&pdev->dev, miumpu_node, i);
		if (IS_ERR(mpu->miu_mpu_base[i])) {
			dev_info(&pdev->dev, "Failed to map MIU%d secure range base\n",
				i);
			return -EIO;
		}
	}
//reg base end
	mpu->vio_msg = devm_kmalloc(&pdev->dev,
		MTK_EMI_MAX_CMD_LEN, GFP_KERNEL);
	if (!(mpu->vio_msg))
		return -ENOMEM;

	ret = devm_add_action_or_reset(&pdev->dev, emimpu_cancel_work, context);
	if (ret)
		return ret;
	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;
	mpu->irq = ret;
	mpu->version = EMIMPUVER2;
	ret = devm_request_irq(&pdev->dev, mpu->irq, emimpu_violation_irq,
		IRQF_NO_AUTOEN, "emimpu", mpu);
	if (ret)
		return ret;
	if (cmpxchg(&global_emi_mpu, NULL, mpu))
		return -EBUSY;
	ret = mtk_emimpu_owner_publish(mpu);
	if (ret) {
		cmpxchg(&global_emi_mpu, mpu, NULL);
		return ret;
	}
	ret = devm_add_action_or_reset(&pdev->dev, emimpu_unpublish, mpu);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, mpu);
	enable_irq(mpu->irq);

	devm_kfree(&pdev->dev, dump_list);
	devm_kfree(&pdev->dev, miukp_dump_list);
	devm_kfree(&pdev->dev, miumpu_dump_list);

	return 0;
}


static struct platform_driver emimpu_driver = {
	.probe = emimpu_probe,
	.driver = {
		.name = "emimpu_v2_driver",
		.owner = THIS_MODULE,
		.of_match_table = emimpu_of_ids,
	},
};

static __init int emimpu_init(void)
{
	int ret;

	pr_info("emimpu_v2 was loaded\n");

	ret = platform_driver_register(&emimpu_driver);
	if (ret) {
		pr_info("emimpu_v2: failed to register driver\n");
		return ret;
	}

	return 0;
}

module_init(emimpu_init);

MODULE_DESCRIPTION("MediaTek EMI MPU V2 Driver");
MODULE_LICENSE("GPL");
