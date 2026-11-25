// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/errno.h>
#include "drivers/misc/mediatek/vcp/include/vcp.h"
#include "drivers/misc/mediatek/vcp/rv_v2/vcp_status.h"
#include "bridge.h"

static bool registered;
module_param(registered, bool, 0444);
static unsigned long gce_memory_bytes;
module_param(gce_memory_bytes, ulong, 0444);
static unsigned int notify_errors;
module_param(notify_errors, uint, 0444);

static phys_addr_t bridge_phys(enum vcp_reserve_mem_id_t id)
{
	return native_bridge_memory(id, 0);
}
static phys_addr_t bridge_virt(enum vcp_reserve_mem_id_t id)
{
	return native_bridge_memory(id, 1);
}
static phys_addr_t bridge_size(enum vcp_reserve_mem_id_t id)
{
	return native_bridge_memory(id, 2);
}
static int bridge_on(enum feature_id id)
{
	return native_bridge_feature(id, true);
}
static int bridge_off(enum feature_id id)
{
	return native_bridge_feature(id, false);
}
static unsigned int bridge_ready(enum feature_id id)
{
	return native_bridge_ready(id);
}
static void bridge_notify_on(enum feature_id id, struct notifier_block *nb)
{
	if (id != GCE_FEATURE_ID || native_bridge_notify(nb, true))
		notify_errors++;
}
static void bridge_notify_off(enum feature_id id, struct notifier_block *nb)
{
	if (id != GCE_FEATURE_ID || native_bridge_notify(nb, false))
		notify_errors++;
}
static struct vcp_status_fp modern_ops = {
	.vcp_get_reserve_mem_phys = bridge_phys,
	.vcp_get_reserve_mem_virt = bridge_virt,
	.vcp_get_reserve_mem_size = bridge_size,
	.vcp_register_feature = bridge_on,
	.vcp_deregister_feature = bridge_off,
	.is_vcp_ready = bridge_ready,
	.vcp_A_register_notify = bridge_notify_on,
	.vcp_A_unregister_notify = bridge_notify_off,
};

static int __init native_bridge_init(void)
{
	/* This board retains native firmware. Do not replace an active provider. */
	if (vcp_get_reserve_mem_virt_ex(GCE_MEM_ID) ||
	    vcp_get_reserve_mem_phys_ex(GCE_MEM_ID) ||
	    vcp_get_reserve_mem_size_ex(GCE_MEM_ID))
		return -EBUSY;
	gce_memory_bytes = bridge_size(GCE_MEM_ID);
	if (!bridge_phys(GCE_MEM_ID) || !bridge_virt(GCE_MEM_ID) ||
	    gce_memory_bytes < 0x100000)
		return -ENODEV;
	/* vcp_set_fp has no unregister API. Both this table and its native
	 * supplier remain pinned for this boot, matching native VCP lifetime.
	 * Never advertise hot unloading of a published global callback table.
	 */
	__module_get(THIS_MODULE);
	vcp_set_fp(&modern_ops);
	registered = true;
	pr_info("VCP native bridge: GCE shared memory and feature callbacks online\n");
	return 0;
}
module_init(native_bridge_init);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Native VCP GCE provider for the modern status interface");
