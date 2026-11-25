// SPDX-License-Identifier: GPL-2.0
/* Native kernel clients and retained firmware have different memory ID 13. */
#include <linux/module.h>
#include "scp.h"

#undef scp_get_reserve_mem_phys
#undef scp_get_reserve_mem_virt
#undef scp_get_reserve_mem_size

/* IDs 0..12 retain their numeric identity. Native AOV/DBI/AOD and newer
 * regions are absent in this firmware. Never expose Elliptic memory as AOV.
 */
phys_addr_t scp_get_reserve_mem_phys(enum scp_reserve_mem_id_t id)
{
	if ((unsigned int)id > SENS_CUSTOM_R_MEM_ID)
		return 0;
	return scp_legacy_get_reserve_mem_phys(id);
}
EXPORT_SYMBOL_GPL(scp_get_reserve_mem_phys);

phys_addr_t scp_get_reserve_mem_virt(enum scp_reserve_mem_id_t id)
{
	if ((unsigned int)id > SENS_CUSTOM_R_MEM_ID)
		return 0;
	return scp_legacy_get_reserve_mem_virt(id);
}
EXPORT_SYMBOL_GPL(scp_get_reserve_mem_virt);

phys_addr_t scp_get_reserve_mem_size(enum scp_reserve_mem_id_t id)
{
	if ((unsigned int)id > SENS_CUSTOM_R_MEM_ID)
		return 0;
	return scp_legacy_get_reserve_mem_size(id);
}
EXPORT_SYMBOL_GPL(scp_get_reserve_mem_size);
