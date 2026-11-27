// SPDX-License-Identifier: GPL-2.0-only
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/unaligned.h>
#include <ufs/ufshcd.h>
#include "ufshcd-priv.h"

static struct proc_dir_entry *mv_proc;
static u32 dram_vendor, dram_gib;
static u16 ufs_vendor;
static u32 ufs_gib;
static char ufs_model[18], ufs_revision[6];

static int read_dram_info(void)
{
	struct device_node *node;
	u32 count, value, total = 0, index;
	int ret;

	node = of_find_node_by_name(NULL, "dramc");
	if (!node)
		return -ENODEV;
	ret = of_property_read_u32(node, "rk_cnt", &count);
	if (ret || !count || count > 8) {
		ret = -EINVAL;
		goto out;
	}
	for (index = 0; index < count; index++) {
		ret = of_property_read_u32_index(node, "rk_size", index, &value);
		if (ret)
			goto out;
		/* MediaTek encodes rank capacity in bits [24:3], in GiB. */
		value = (value >> 3) & GENMASK(21, 0);
		if (!value || check_add_overflow(total, value, &total)) {
			ret = -EINVAL;
			goto out;
		}
	}
	ret = of_property_read_u32(node, "mr_cnt", &count);
	if (ret || !count || count > 32) {
		ret = -EINVAL;
		goto out;
	}
	for (index = 0; index < count; index++) {
		ret = of_property_read_u32_index(node, "mr", index * 2, &value);
		if (ret)
			goto out;
		if (value != 5)
			continue;
		ret = of_property_read_u32_index(node, "mr", index * 2 + 1, &value);
		if (!ret) {
			dram_vendor = value & 0xff;
			dram_gib = total;
		}
		goto out;
	}
	ret = -ENODATA;
out:
	of_node_put(node);
	return ret;
}

static int match_ufs_host(struct device *dev, const void *unused)
{
	return dev->driver && !strcmp(dev->driver->name, "ufshcd-mtk");
}

static int read_string(struct ufs_hba *hba, u8 index, char *result, size_t size)
{
	u8 *descriptor;
	int length = QUERY_DESC_MAX_SIZE, ret;
	size_t i, used = 0;

	if (!index)
		return -ENODATA;
	descriptor = kzalloc(length, GFP_KERNEL);
	if (!descriptor)
		return -ENOMEM;
	ret = ufshcd_query_descriptor_retry(hba, UPIU_QUERY_OPCODE_READ_DESC,
					  QUERY_DESC_IDN_STRING, index, 0,
					  descriptor, &length);
	if (ret)
		goto out;
	if (length < 2 || descriptor[0] < 2 || descriptor[0] > length ||
	    descriptor[1] != QUERY_DESC_IDN_STRING || (descriptor[0] & 1)) {
		ret = -EINVAL;
		goto out;
	}
	for (i = 2; i < descriptor[0] && used + 1 < size; i += 2) {
		u16 character = get_unaligned_be16(descriptor + i);

		if (!character)
			break;
		result[used++] = character >= 0x21 && character <= 0x7e ? character : '_';
	}
	result[used] = 0;
	if (!used)
		ret = -ENODATA;
out:
	kfree(descriptor);
	return ret;
}

static int read_ufs_info(void)
{
	struct device *dev;
	struct ufs_hba *hba;
	u8 *descriptor;
	__be64 capacity;
	u64 gib;
	int ret, length = QUERY_DESC_MAX_SIZE;

	dev = bus_find_device(&platform_bus_type, NULL, NULL, match_ufs_host);
	if (!dev)
		return -ENODEV;
	descriptor = kzalloc(length, GFP_KERNEL);
	if (!descriptor) {
		ret = -ENOMEM;
		goto put;
	}
	device_lock(dev);
	hba = dev_get_drvdata(dev);
	if (!dev->driver || !hba || hba->dev != dev) {
		ret = -ENODEV;
		goto unlock_device;
	}
	down(&hba->host_sem);
	if (!ufshcd_is_user_access_allowed(hba)) {
		ret = -EBUSY;
		goto unlock_host;
	}
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		goto unlock_host;
	ret = ufshcd_query_descriptor_retry(hba, UPIU_QUERY_OPCODE_READ_DESC,
					  QUERY_DESC_IDN_DEVICE, 0, 0,
					  descriptor, &length);
	if (ret)
		goto put_pm;
	if (length <= DEVICE_DESC_PARAM_PRDCT_REV ||
	    descriptor[0] <= DEVICE_DESC_PARAM_PRDCT_REV || descriptor[0] > length ||
	    descriptor[1] != QUERY_DESC_IDN_DEVICE) {
		ret = -EINVAL;
		goto put_pm;
	}
	ufs_vendor = get_unaligned_be16(descriptor + DEVICE_DESC_PARAM_MANF_ID);
	if (!ufs_vendor) {
		ret = -ENODATA;
		goto put_pm;
	}
	ret = read_string(hba, descriptor[DEVICE_DESC_PARAM_PRDCT_NAME],
			  ufs_model, sizeof(ufs_model));
	if (ret)
		goto put_pm;
	ret = read_string(hba, descriptor[DEVICE_DESC_PARAM_PRDCT_REV],
			  ufs_revision, sizeof(ufs_revision));
	if (ret)
		goto put_pm;
	ret = ufshcd_read_desc_param(hba, QUERY_DESC_IDN_GEOMETRY, 0,
				    GEOMETRY_DESC_PARAM_DEV_CAP,
				    (u8 *)&capacity, sizeof(capacity));
	if (ret)
		goto put_pm;
	/* Geometry capacity is in 512-byte sectors; retain Xiaomi's size bucket. */
	gib = be64_to_cpu(capacity) >> 21;
	if (gib > 512 && gib <= 1024)
		ufs_gib = 1024;
	else if (gib > 256 && gib <= 512)
		ufs_gib = 512;
	else if (gib > 128 && gib <= 256)
		ufs_gib = 256;
	else if (gib > 64 && gib <= 128)
		ufs_gib = 128;
	else if (gib > 32 && gib <= 64)
		ufs_gib = 64;
	else if (gib > 16 && gib <= 32)
		ufs_gib = 32;
	else
		ret = -ERANGE;
put_pm:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
unlock_host:
	up(&hba->host_sem);
unlock_device:
	device_unlock(dev);
	kfree(descriptor);
put:
	put_device(dev);
	return ret;
}

static int mv_show(struct seq_file *file, void *unused)
{
	seq_printf(file, "D: 0x%02x %u\nU: 0x%04x %u %s %s\n",
		   dram_vendor, dram_gib, ufs_vendor, ufs_gib,
		   ufs_model, ufs_revision);
	return 0;
}
DEFINE_PROC_SHOW_ATTRIBUTE(mv);

static int __init memory_info_init(void)
{
	int ret = read_dram_info();

	if (ret)
		return ret;
	ret = read_ufs_info();
	if (ret)
		return ret;
	mv_proc = proc_create("mv", 0444, NULL, &mv_proc_ops);
	return mv_proc ? 0 : -ENOMEM;
}

static void __exit memory_info_exit(void)
{
	proc_remove(mv_proc);
}

module_init(memory_info_init);
module_exit(memory_info_exit);
MODULE_DESCRIPTION("DDR and UFS identity for Xiaomi platform services");
MODULE_LICENSE("GPL");
