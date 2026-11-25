/* SPDX-License-Identifier: GPL-2.0 */
#ifndef GPUEB_MEMORY_CONTRACT_H
#define GPUEB_MEMORY_CONTRACT_H

struct gpueb_memory_keys {
	const char *table;
	const char *names;
};

/* Do not mix families or silently choose between duplicate definitions. */
static int gpueb_memory_property_keys(struct device_node *node,
	struct gpueb_memory_keys *keys)
{
	bool modern_table = of_property_present(node, "gpueb-mem-table");
	bool modern_names = of_property_present(node, "gpueb-mem-name-table");
	bool legacy_table = of_property_present(node, "gpueb_mem_table");
	bool legacy_names = of_property_present(node, "gpueb_mem_name_table");

	if (!keys)
		return -EINVAL;
	if (modern_table && modern_names && !legacy_table && !legacy_names) {
		keys->table = "gpueb-mem-table";
		keys->names = "gpueb-mem-name-table";
	} else if (legacy_table && legacy_names && !modern_table && !modern_names) {
		keys->table = "gpueb_mem_table";
		keys->names = "gpueb_mem_name_table";
	} else {
		return -EINVAL;
	}
	return 0;
}

/* Validate the firmware-owned region before any mailbox/IRQ publication. */
static int gpueb_validate_boot_memory(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct gpueb_memory_keys keys;
	u64 base, size, total = 0;
	u32 id, bytes, seen = 0;
	const char *name;
	int cells, count, i, j, ret;

	ret = of_property_read_u64(node, "gpueb_mem_addr", &base);
	if (ret)
		return ret;
	ret = of_property_read_u64(node, "gpueb_mem_size", &size);
	if (ret)
		return ret;
	if (!base || !size)
		return -ENODATA;
	if (base < 0x40000000ULL || base >= 0x800000000ULL ||
	    size > 0x800000000ULL - base || (base & 0xfff) || (size & 0xfff))
		return -ERANGE;

	ret = gpueb_memory_property_keys(node, &keys);
	if (ret)
		return ret;
	cells = of_property_count_u32_elems(node, keys.table);
	if (cells <= 0 || cells % 2)
		return -EINVAL;
	count = cells / 2;
	/* The native provider publishes a fixed twenty-entry name array. */
	if (count > 20 ||
	    of_property_count_strings(node, keys.names) != count)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		ret = of_property_read_u32_index(node, keys.table, i * 2, &id);
		if (ret)
			return ret;
		ret = of_property_read_u32_index(node, keys.table, i * 2 + 1, &bytes);
		if (ret)
			return ret;
		if (id >= (u32)count || (seen & (1U << id)) || !bytes || bytes > size - total)
			return -EINVAL;
		seen |= 1U << id;
		total += bytes;
		ret = of_property_read_string_index(node, keys.names, i, &name);
		if (ret)
			return ret;
		if (!name[0])
			return -EINVAL;
		for (j = 0; j < i; j++) {
			const char *previous;

			ret = of_property_read_string_index(node, keys.names, j, &previous);
			if (ret)
				return ret;
			if (!strcmp(name, previous))
				return -EINVAL;
		}
	}
	return 0;
}
#endif
