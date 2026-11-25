// SPDX-License-Identifier: GPL-2.0
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include "vcp.h"
#include "bridge.h"

static DEFINE_MUTEX(feature_lock);
static unsigned int gce_users;

phys_addr_t native_bridge_memory(unsigned int id, unsigned int kind)
{
	if (id >= NUMS_MEM_ID)
		return 0;
	switch (kind) {
	case 0: return vcp_get_reserve_mem_phys(id);
	case 1: return vcp_get_reserve_mem_virt(id);
	case 2: return vcp_get_reserve_mem_size(id);
	default: return 0;
	}
}

unsigned int native_bridge_ready(unsigned int id)
{
	/* The new API accepts a feature ID; the native API accepts a core ID. */
	return id == GCE_FEATURE_ID ? is_vcp_ready(VCP_A_ID) : 0;
}

int native_bridge_feature(unsigned int id, bool enable)
{
	int ret = 0, i;

	if (id != GCE_FEATURE_ID)
		return -EOPNOTSUPP;
	mutex_lock(&feature_lock);
	if (enable) {
		if (gce_users == UINT_MAX) {
			ret = -EOVERFLOW;
			goto out;
		}
		vcp_register_feature(GCE_FEATURE_ID);
		for (i = 0; i < 100 && !is_vcp_ready(VCP_A_ID); i++)
			msleep(5);
		if (!is_vcp_ready(VCP_A_ID)) {
			vcp_deregister_feature(GCE_FEATURE_ID);
			ret = -ETIMEDOUT;
			goto out;
		}
		gce_users++;
	} else {
		if (!gce_users) {
			ret = -EINVAL;
			goto out;
		}
		vcp_deregister_feature(GCE_FEATURE_ID);
		gce_users--;
	}
out:
	mutex_unlock(&feature_lock);
	return ret;
}

int native_bridge_notify(struct notifier_block *nb, bool enable)
{
	if (!nb || !nb->notifier_call)
		return -EINVAL;
	if (enable)
		return vcp_A_register_notify(nb);
	vcp_A_unregister_notify(nb);
	return 0;
}
