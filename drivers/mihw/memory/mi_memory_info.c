// SPDX-License-Identifier: GPL-2.0-only
#include <linux/device.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_modeset_lock.h>
#include "mtk_drm_drv.h"
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/unaligned.h>
#include <ufs/ufshcd.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_common.h>
#include "ufshcd-priv.h"

static struct proc_dir_entry *mv_proc;
static u32 dram_vendor, dram_gib, dram_type;
static struct class *memory_class;
static struct device *memory_device;
static struct kobject *hwconf_kobj;
static char *hwconf_name = "hwconf";
module_param(hwconf_name, charp, 0444);
static char *interface_name = "mi_memory";
module_param(interface_name, charp, 0444);
static char *proc_name = "mv";
module_param(proc_name, charp, 0444);
static u16 ufs_vendor, ufs_spec;
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
	ret = of_property_read_u32(node, "dram_type", &dram_type);
	if (ret)
		goto out;
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
	ufs_spec = get_unaligned_be16(descriptor + DEVICE_DESC_PARAM_SPEC_VER);
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


static ssize_t ddr_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "0x%02x\n", dram_vendor);
}
static DEVICE_ATTR_RO(ddr_id);

static ssize_t ddr_size_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", dram_gib);
}
static DEVICE_ATTR_RO(ddr_size);

static ssize_t ddr_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "0x%x\n", dram_type);
}
static DEVICE_ATTR_RO(ddr_type);

static ssize_t ddr_vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const char *name;

	switch (dram_vendor) {
	case 1: name = "SAMSUNG"; break;
	case 6: name = "HYNIX"; break;
	case 255: name = "MICRON"; break;
	default: name = "UNKNOWN"; break;
	}
	return sysfs_emit(buf, "%s\n", name);
}
static DEVICE_ATTR_RO(ddr_vendor);

/* Standard READ_DESC queries only; no vendor mode, password or buffer commands. */
static int memory_read_descriptor(enum desc_idn id, u8 index, u8 *descriptor, int *length)
{
	struct device *dev;
	struct ufs_hba *hba;
	int ret;

	dev = bus_find_device(&platform_bus_type, NULL, NULL, match_ufs_host);
	if (!dev)
		return -ENODEV;
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
					  id, index, 0, descriptor, length);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	if (!ret && (*length < 2 || descriptor[0] < 2 ||
		     descriptor[0] > *length || descriptor[1] != id))
		ret = -EINVAL;
unlock_host:
	up(&hba->host_sem);
unlock_device:
	device_unlock(dev);
	put_device(dev);
	return ret;
}

struct descriptor_field {
	const char *name;
	u8 offset, width;
};

static const struct descriptor_field device_fields[] = {
	{ "bLength", 0x00, 1 }, { "bDescriptorType", 0x01, 1 },
	{ "bDevice", 0x02, 1 }, { "bDeviceClass", 0x03, 1 },
	{ "bDeviceSubClass", 0x04, 1 }, { "bProtocol", 0x05, 1 },
	{ "bNumberLU", 0x06, 1 }, { "bNumberWLU", 0x07, 1 },
	{ "bBootEnable", 0x08, 1 }, { "bDescrAccessEn", 0x09, 1 },
	{ "bInitPowerMode", 0x0a, 1 }, { "bHighPriorityLUN", 0x0b, 1 },
	{ "bSecureRemovalType", 0x0c, 1 }, { "bSecurityLU", 0x0d, 1 },
	{ "Reserved", 0x0e, 1 }, { "bInitActiveICCLevel", 0x0f, 1 },
	{ "wSpecVersion", 0x10, 2 }, { "wManufactureDate", 0x12, 2 },
	{ "iManufactureName", 0x14, 1 }, { "iProductName", 0x15, 1 },
	{ "iSerialNumber", 0x16, 1 }, { "iOemID", 0x17, 1 },
	{ "wManufactureID", 0x18, 2 }, { "bUD0BaseOffset", 0x1a, 1 },
	{ "bUDConfigPLength", 0x1b, 1 }, { "bDeviceRTTCap", 0x1c, 1 },
	{ "wPeriodicRTCUpdate", 0x1d, 2 }, { "bUFSFeaturesSupport", 0x1f, 1 },
	{ "bFFUTimeout", 0x20, 1 }, { "bQueueDepth", 0x21, 1 },
	{ "wDeviceVersion", 0x22, 2 }, { "bNumSecureWpArea", 0x24, 1 },
	{ "dPSAMaxDataSize", 0x25, 4 }, { "bPSAStateTimeout", 0x29, 1 },
};
static const struct descriptor_field health_fields[] = {
	{ "bLength", 0, 1 }, { "bDescriptorType", 1, 1 },
	{ "bPreEOLInfo", 2, 1 }, { "bDeviceLifeTimeEstA", 3, 1 },
	{ "bDeviceLifeTimeEstB", 4, 1 },
};

static ssize_t descriptor_show(char *buf, enum desc_idn id,
			       const struct descriptor_field *fields, size_t count)
{
	u8 descriptor[QUERY_DESC_MAX_SIZE] = { 0 };
	int length = sizeof(descriptor), ret;
	ssize_t used = 0;
	size_t i;

	ret = memory_read_descriptor(id, 0, descriptor, &length);
	if (ret)
		return ret;
	for (i = 0; i < count; i++) {
		const struct descriptor_field *field = &fields[i];
		u32 value;

		if (field->offset + field->width > descriptor[0])
			return -EINVAL;
		if (field->width == 4)
			value = get_unaligned_be32(descriptor + field->offset);
		else if (field->width == 2)
			value = get_unaligned_be16(descriptor + field->offset);
		else
			value = descriptor[field->offset];
		used += sysfs_emit_at(buf, used,
			"Device Descriptor[Byte offset 0x%x]: %s = 0x%x\n",
			field->offset, field->name, value);
	}
	return used;
}

static ssize_t dump_device_desc_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return descriptor_show(buf, QUERY_DESC_IDN_DEVICE,
			       device_fields, ARRAY_SIZE(device_fields));
}
static DEVICE_ATTR_RO(dump_device_desc);

static ssize_t dump_health_desc_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return descriptor_show(buf, QUERY_DESC_IDN_HEALTH,
			       health_fields, ARRAY_SIZE(health_fields));
}
static DEVICE_ATTR_RO(dump_health_desc);

static ssize_t show_hba_show(struct device *unused,
			     struct device_attribute *attr, char *buf)
{
	static const char * const event_names[UFS_EVT_CNT] = {
		[UFS_EVT_PA_ERR] = "pa_error", [UFS_EVT_DL_ERR] = "dl_error",
		[UFS_EVT_NL_ERR] = "nl_error", [UFS_EVT_TL_ERR] = "tl_error",
		[UFS_EVT_DME_ERR] = "dme_error", [UFS_EVT_AUTO_HIBERN8_ERR] = "auto_hibern8",
		[UFS_EVT_FATAL_ERR] = "fatal_error", [UFS_EVT_LINK_STARTUP_FAIL] = "link_startup_fail",
		[UFS_EVT_RESUME_ERR] = "resume_error", [UFS_EVT_SUSPEND_ERR] = "suspend_error",
		[UFS_EVT_WL_SUSP_ERR] = "wl_suspend_error", [UFS_EVT_WL_RES_ERR] = "wl_resume_error",
		[UFS_EVT_DEV_RESET] = "device_reset", [UFS_EVT_HOST_RESET] = "host_reset",
		[UFS_EVT_ABORT] = "abort",
	};
	struct device *dev;
	struct ufs_hba *hba;
	unsigned long flags;
	ssize_t used;
	unsigned int i;

	dev = bus_find_device(&platform_bus_type, NULL, NULL, match_ufs_host);
	if (!dev)
		return -ENODEV;
	device_lock(dev);
	hba = dev_get_drvdata(dev);
	if (!dev->driver || !hba || hba->dev != dev) {
		used = -ENODEV;
		goto out;
	}
	spin_lock_irqsave(hba->host->host_lock, flags);
	used = sysfs_emit(buf,
		"hba->outstanding_reqs = 0x%lx\nhba->capabilities = 0x%x\n"
		"hba->nutrs = %d\nhba->nutmrs = %d\nhba->ufs_version = 0x%x\n"
		"hba->irq = 0x%x\nhba->auto_bkops_enabled = %d\n"
		"hba->ufshcd_state = 0x%x\nhba->eh_flags = 0x%x\n"
		"hba->intr_mask = 0x%x\nhba->ee_ctrl_mask = 0x%x\n"
		"hba->errors = 0x%x\nhba->uic_error = 0x%x\n"
		"hba->saved_err = 0x%x\nhba->saved_uic_err = 0x%x\n"
		"hibern8_exit_cnt = %u\n",
		READ_ONCE(hba->outstanding_reqs), hba->capabilities, hba->nutrs,
		hba->nutmrs, hba->ufs_version, hba->irq, hba->auto_bkops_enabled,
		hba->ufshcd_state, hba->eh_flags, hba->intr_mask, hba->ee_ctrl_mask,
		hba->errors, hba->uic_error, hba->saved_err, hba->saved_uic_err,
		hba->ufs_stats.hibern8_exit_cnt);
	for (i = 0; i < UFS_EVT_CNT; i++)
		used += sysfs_emit_at(buf, used, "ufs_event_%s_cnt = 0x%llx\n",
				      event_names[i], hba->ufs_stats.event[i].cnt);
	spin_unlock_irqrestore(hba->host->host_lock, flags);
out:
	device_unlock(dev);
	put_device(dev);
	return used;
}
static DEVICE_ATTR_RO(show_hba);


static ssize_t dump_string_desc_serial_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 descriptor[QUERY_DESC_MAX_SIZE] = { 0 }, index;
	int length = sizeof(descriptor), ret;
	ssize_t used;
	unsigned int i;

	ret = memory_read_descriptor(QUERY_DESC_IDN_DEVICE, 0, descriptor, &length);
	if (ret)
		return ret;
	if (descriptor[0] <= DEVICE_DESC_PARAM_SN)
		return -EINVAL;
	index = descriptor[DEVICE_DESC_PARAM_SN];
	if (!index)
		return -ENODATA;
	length = sizeof(descriptor);
	ret = memory_read_descriptor(QUERY_DESC_IDN_STRING, index, descriptor, &length);
	if (ret)
		return ret;
	if (descriptor[0] & 1)
		return -EINVAL;
	used = sysfs_emit(buf, "serial:");
	for (i = 2; i < descriptor[0]; i++)
		used += sysfs_emit_at(buf, used, "%02x", descriptor[i]);
	return used + sysfs_emit_at(buf, used, "\n");
}
static DEVICE_ATTR_RO(dump_string_desc_serial);

static const char *memory_error_reason(unsigned int event, u32 value)
{
	switch (event) {
	case UFS_EVT_AUTO_HIBERN8_ERR: return "auto_hibern8_err\n";
	case UFS_EVT_FATAL_ERR: return "fatal_err\n";
	case UFS_EVT_LINK_STARTUP_FAIL:
		return value ? "link_startup_fail\n" : "link_startup_fail_not_present\n";
	case UFS_EVT_RESUME_ERR: return "ufs_resume_fail\n";
	case UFS_EVT_SUSPEND_ERR: return "ufs_suspend_fail\n";
	case UFS_EVT_WL_SUSP_ERR: return "wlun_suspend_fail\n";
	case UFS_EVT_WL_RES_ERR: return "wlun_resume_fail\n";
	case UFS_EVT_HOST_RESET: return "host_reset\n";
	case UFS_EVT_ABORT: return "task_abort\n";
	default: return NULL;
	}
}

static ssize_t error_report(char *buf, bool reasons)
{
	struct error_entry { u64 time; const char *reason; } newest[10] = { 0 };
	struct device *dev;
	struct ufs_hba *hba;
	unsigned long flags;
	u64 count = 0;
	unsigned int i, j, k, n;
	ssize_t used = 0;

	dev = bus_find_device(&platform_bus_type, NULL, NULL, match_ufs_host);
	if (!dev)
		return -ENODEV;
	device_lock(dev);
	hba = dev_get_drvdata(dev);
	if (!dev->driver || !hba || hba->dev != dev) {
		used = -ENODEV;
		goto out;
	}
	spin_lock_irqsave(hba->host->host_lock, flags);
	for (i = 0; i < UFS_EVT_CNT; i++) {
		const struct ufs_event_hist *event = &hba->ufs_stats.event[i];

		if (!memory_error_reason(i, 1))
			continue;
		count = count > U64_MAX - event->cnt ? U64_MAX : count + event->cnt;
		for (j = 0; j < UFS_EVENT_HIST_LENGTH; j++) {
			if (!event->tstamp[j])
				continue;
			for (k = 0; k < ARRAY_SIZE(newest); k++) {
				if (newest[k].time >= event->tstamp[j])
					continue;
				for (n = ARRAY_SIZE(newest) - 1; n > k; n--)
					newest[n] = newest[n - 1];
				newest[k].time = event->tstamp[j];
				newest[k].reason = memory_error_reason(i, event->val[j]);
				break;
			}
		}
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);
	if (!reasons)
		used = sysfs_emit(buf, "%llu\n", count);
	else
		for (i = 0; i < ARRAY_SIZE(newest) && newest[i].reason; i++)
			used += sysfs_emit_at(buf, used, "%s", newest[i].reason);
out:
	device_unlock(dev);
	put_device(dev);
	return used;
}

static ssize_t err_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return error_report(buf, false);
}
static DEVICE_ATTR_RO(err_state);
static ssize_t err_reason_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return error_report(buf, true);
}
static DEVICE_ATTR_RO(err_reason);

static DEFINE_MUTEX(hr_lock);
static int hr_vendor_status = -EAGAIN;

static ssize_t hr_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = READ_ONCE(hr_vendor_status);

	if (status == -EAGAIN)
		return sysfs_emit(buf, "vendor_report: not queried\n");
	if (status == -EOPNOTSUPP)
		return sysfs_emit(buf, "vendor_report: unsupported; hr contains standard health only\n");
	if (status)
		return sysfs_emit(buf, "vendor_report: error %d\n", status);
	return sysfs_emit(buf, "vendor_report: available\n");
}
static DEVICE_ATTR_RO(hr_status);

static int hynix_hr_read(struct scsi_device *sdev, u32 version, u8 *buf)
{
	u8 cdb[16] = { 0xd0 };
	struct request *req;
	struct scsi_cmnd *cmd;
	unsigned int size = version < 0x400 ? 0x52 : 0x6e;
	int ret;

	cdb[1] = version < 0x400 ? 3 : 0x0f;
	cdb[2] = version < 0x400 ? 0x58 : 0x53;
	cdb[11] = size;
	req = scsi_alloc_request(sdev->request_queue, REQ_OP_DRV_IN, BLK_MQ_REQ_PM);
	if (IS_ERR(req))
		return PTR_ERR(req);
	ret = blk_rq_map_kern(sdev->request_queue, req, buf, size, GFP_NOIO);
	if (ret)
		goto out;
	cmd = blk_mq_rq_to_pdu(req);
	cmd->cmd_len = sizeof(cdb);
	memcpy(cmd->cmnd, cdb, sizeof(cdb));
	cmd->allowed = 3;
	req->timeout = msecs_to_jiffies(7500);
	req->rq_flags |= RQF_PM | RQF_QUIET;
	blk_execute_rq(req, true);
	if (cmd->result || cmd->resid_len) {
		struct scsi_sense_hdr sense = { 0 };

		scsi_normalize_sense(cmd->sense_buffer, cmd->sense_len, &sense);
		pr_warn("mi_memory: Hynix hr result=0x%x residual=%u sense=%x/%x/%x\n",
			cmd->result, cmd->resid_len, sense.sense_key, sense.asc, sense.ascq);
	}
	ret = cmd->result || cmd->resid_len ? -EIO : 0;
	if (cmd->result) {
		struct scsi_sense_hdr sense = { 0 };

		if (scsi_normalize_sense(cmd->sense_buffer, cmd->sense_len, &sense) &&
		    sense.sense_key == ILLEGAL_REQUEST &&
		    (sense.asc == 0x20 || sense.asc == 0x24) && !sense.ascq)
			ret = -EOPNOTSUPP;
	}
out:
	blk_mq_free_request(req);
	return ret;
}

static ssize_t hr_show(struct device *unused, struct device_attribute *attr, char *buf)
{
	struct device *dev;
	struct ufs_hba *hba;
	struct scsi_device *sdev;
	u8 *data, descriptor[QUERY_DESC_MAX_SIZE];
	int length = sizeof(descriptor), ret;
	u32 version;
	ssize_t used = 0;
	unsigned int i;

	if (ufs_vendor != 0x1ad)
		return -EOPNOTSUPP;
	data = kzalloc(512, GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	mutex_lock(&hr_lock);
	ret = memory_read_descriptor(QUERY_DESC_IDN_HEALTH, 0, descriptor, &length);
	if (ret)
		goto unlock;
	memcpy(data, descriptor, min_t(unsigned int, descriptor[0], 0x25));
	if (READ_ONCE(hr_vendor_status) == -EOPNOTSUPP)
		goto unlock;
	dev = bus_find_device(&platform_bus_type, NULL, NULL, match_ufs_host);
	if (!dev) {
		ret = -ENODEV;
		goto unlock;
	}
	device_lock(dev);
	hba = dev_get_drvdata(dev);
	if (!dev->driver || !hba || hba->dev != dev) {
		ret = -ENODEV;
		goto put_dev;
	}
	version = ufs_spec;
	sdev = scsi_device_lookup(hba->host, 0, 0, 0);
	device_unlock(dev);
	put_device(dev);
	if (!sdev) {
		ret = -ENODEV;
		goto unlock;
	}
	if (strncmp(sdev->vendor, "SKhynix", 7) || !scsi_device_online(sdev)) {
		ret = -EOPNOTSUPP;
		goto put_scsi;
	}
	ret = scsi_autopm_get_device(sdev);
	if (ret)
		goto put_scsi;
	ret = hynix_hr_read(sdev, version, data + 0x80);
	WRITE_ONCE(hr_vendor_status, ret);
	/* Keep the native 512-byte layout; hr_status identifies an absent extension. */
	if (ret == -EOPNOTSUPP)
		ret = 0;
	scsi_autopm_put_device(sdev);
put_scsi:
	scsi_device_put(sdev);
	goto unlock;
put_dev:
	device_unlock(dev);
	put_device(dev);
unlock:
	mutex_unlock(&hr_lock);
	if (!ret) {
		for (i = 0; i < 512; i++)
			used += sysfs_emit_at(buf, used, "%02x", data[i]);
		used += sysfs_emit_at(buf, used, "\n");
	}
	kfree(data);
	return ret ? ret : used;
}
static DEVICE_ATTR_RO(hr);


static int match_display(struct device *dev, const void *unused)
{
	return dev->driver && !strcmp(dev->driver->name, "mediatek-drm");
}

static ssize_t hw_mon_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct device *dev;
	struct mtk_drm_private *private;
	struct drm_device *drm;
	struct drm_connector_list_iter iter;
	struct drm_connector *connector;
	unsigned int connected = 0, active = 0, width = 0, height = 0, refresh = 0;
	u8 health[QUERY_DESC_MAX_SIZE] = { 0 };
	int length = sizeof(health), ret, index;

	ret = memory_read_descriptor(QUERY_DESC_IDN_HEALTH, 0, health, &length);
	if (ret)
		return ret;
	if (health[0] < 5)
		return -EINVAL;
	dev = bus_find_device(&platform_bus_type, NULL, NULL, match_display);
	if (!dev)
		return -ENODEV;
	device_lock(dev);
	private = dev_get_drvdata(dev);
	if (!dev->driver || !private || !private->drm) {
		ret = -ENODEV;
		goto out;
	}
	drm = private->drm;
	if (!drm_dev_enter(drm, &index)) {
		ret = -ENODEV;
		goto out;
	}
	drm_modeset_lock_all(drm);
	drm_connector_list_iter_begin(drm, &iter);
	drm_for_each_connector_iter(connector, &iter) {
		struct drm_crtc *crtc;

		if (connector->connector_type != DRM_MODE_CONNECTOR_DSI)
			continue;
		connected = connector->status == connector_status_connected;
		crtc = connector->state ? connector->state->crtc : NULL;
		if (crtc && crtc->state) {
			active = crtc->state->active;
			width = crtc->state->adjusted_mode.hdisplay;
			height = crtc->state->adjusted_mode.vdisplay;
			refresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);
		}
		break;
	}
	drm_connector_list_iter_end(&iter);
	drm_modeset_unlock_all(drm);
	drm_dev_exit(index);
	ret = sysfs_emit(buf,
		"{\"display\":{\"connected\":\"%u\",\"active\":\"%u\","
		"\"width\":\"%u\",\"height\":\"%u\",\"refresh_hz\":\"%u\"},"
		"\"memory\":{\"ddr_id\":\"0x%02x\",\"ddr_type\":\"%u\","
		"\"ddr_size\":\"%u\",\"ufs_id\":\"0x%04x\",\"ufs_size\":\"%u\","
		"\"pre_eol\":\"%u\",\"life_a\":\"%u\",\"life_b\":\"%u\"}}\n",
		connected, active, width, height, refresh, dram_vendor, dram_type,
		dram_gib, ufs_vendor, ufs_gib, health[2], health[3], health[4]);
out:
	device_unlock(dev);
	put_device(dev);
	return ret;
}
static struct kobj_attribute hw_mon_attr = __ATTR_RO(hw_mon);

static struct attribute *ddr_attrs[] = {
	&dev_attr_ddr_id.attr, &dev_attr_ddr_size.attr, &dev_attr_ddr_type.attr,
	&dev_attr_ddr_vendor.attr, NULL,
};
static const struct attribute_group ddr_group = { .name = "ddr", .attrs = ddr_attrs };
static struct attribute *ufs_attrs[] = {
	&dev_attr_dump_device_desc.attr, &dev_attr_dump_health_desc.attr,
	&dev_attr_show_hba.attr, &dev_attr_dump_string_desc_serial.attr,
	&dev_attr_hr.attr, &dev_attr_hr_status.attr, &dev_attr_err_state.attr, &dev_attr_err_reason.attr, NULL,
};
static const struct attribute_group ufs_group = { .name = "ufshcd0", .attrs = ufs_attrs };
static const struct attribute_group *memory_groups[] = { &ddr_group, &ufs_group, NULL };

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
	int ret;

	if (!hwconf_name[0] || strchr(hwconf_name, '/') || strlen(hwconf_name) > 63)
		return -EINVAL;
	ret = read_dram_info();

	if (ret)
		return ret;
	ret = read_ufs_info();
	if (ret)
		return ret;
	memory_class = class_create(interface_name);
	if (IS_ERR(memory_class))
		return PTR_ERR(memory_class);
	memory_device = device_create_with_groups(memory_class, NULL, 0, NULL,
						 memory_groups, "mi_memory_device");
	if (IS_ERR(memory_device)) {
		ret = PTR_ERR(memory_device);
		goto destroy_class;
	}
	mv_proc = proc_create(proc_name, 0444, NULL, &mv_proc_ops);
	if (!mv_proc) {
		ret = -ENOMEM;
		goto destroy_device;
	}
	hwconf_kobj = kobject_create_and_add(hwconf_name, NULL);
	if (!hwconf_kobj) {
		ret = -ENOMEM;
		goto remove_proc;
	}
	ret = sysfs_create_file(hwconf_kobj, &hw_mon_attr.attr);
	if (!ret)
		return 0;
	kobject_put(hwconf_kobj);
remove_proc:
	proc_remove(mv_proc);
destroy_device:
	device_unregister(memory_device);
destroy_class:
	class_destroy(memory_class);
	return ret;
}

static void __exit memory_info_exit(void)
{
	sysfs_remove_file(hwconf_kobj, &hw_mon_attr.attr);
	kobject_put(hwconf_kobj);
	proc_remove(mv_proc);
	device_unregister(memory_device);
	class_destroy(memory_class);
}

module_init(memory_info_init);
module_exit(memory_info_exit);
MODULE_DESCRIPTION("DDR and UFS identity for Xiaomi platform services");
MODULE_LICENSE("GPL");
