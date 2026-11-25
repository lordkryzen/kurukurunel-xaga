/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_HANGDET_RESOURCE_H
#define MTK_HANGDET_RESOURCE_H

#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_address.h>

/* Keep the watchdog resource reservation intact. MT6895's stock connectivity
 * node declares a shared covering RGU window, including the watchdog counter.
 * Mapping that declared alias preserves the existing physical register/page.
 */
static void __iomem *hangdet_map_toprgu(struct device_node *watchdog)
{
	struct resource primary, shared;
	struct device_node *connectivity;
	void __iomem *mapping = NULL;
	const resource_size_t needed = WDT_COUNTER + sizeof(u32);
	unsigned int index;

	if (!watchdog || of_address_to_resource(watchdog, 0, &primary) ||
	    !(primary.flags & IORESOURCE_MEM) || primary.end < primary.start)
		return NULL;
	if (resource_size(&primary) >= needed)
		return of_iomap(watchdog, 0);
	if (!of_device_is_compatible(watchdog, "mediatek,mt6895-wdt"))
		return NULL;
	connectivity = of_find_compatible_node(NULL, NULL, "mediatek,mt6895-consys");
	if (!connectivity)
		return NULL;
	for (index = 0; index < 64; index++) {
		if (of_address_to_resource(connectivity, index, &shared))
			break;
		if (!(shared.flags & IORESOURCE_MEM) || shared.end < shared.start ||
		    shared.start != primary.start || resource_size(&shared) < needed)
			continue;
		mapping = of_iomap(connectivity, index);
		break;
	}
	of_node_put(connectivity);
	return mapping;
}

#endif
