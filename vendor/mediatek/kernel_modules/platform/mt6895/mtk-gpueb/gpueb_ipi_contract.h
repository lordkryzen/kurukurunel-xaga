/* SPDX-License-Identifier: GPL-2.0 */
#ifndef GPUEB_IPI_CONTRACT_H
#define GPUEB_IPI_CONTRACT_H

#define GPUEB_MAX_CHANNELS 20
#define GPUEB_MAX_MAILBOXES 16
struct gpueb_ipi_pin_layout {
	u32 channel, mailbox, words, receive_option, callback_context;
	const char *name;
};
struct gpueb_ipi_layout {
	u32 mailbox_count, mailbox_words, slot_bytes, timesync_mailbox;
	u32 send_count, receive_count;
	struct gpueb_ipi_pin_layout send[GPUEB_MAX_CHANNELS];
	struct gpueb_ipi_pin_layout receive[GPUEB_MAX_CHANNELS];
};

/* Parse the wire schema, not just the spelling of its property names. */
/* Keep the candidate and caller output in separate bounded stack frames. */
static noinline int gpueb_read_ipi_layout(struct device_node *node,
	struct gpueb_ipi_layout *output)
{
	static const char * const keys[2][8] = {
		{"mbox-count", "mbox-size", "slot-size", "ts-mbox",
		 "send-table", "recv-table", "send-name-table", "recv-name-table"},
		{"mbox_count", "mbox_size", "slot_size", "ts_mbox",
		 "send_table", "recv_table", "send_name_table", "recv_name_table"}
	};
	struct gpueb_ipi_layout layout = {};
	u32 used[GPUEB_MAX_MAILBOXES] = {};
	u32 *scalars[] = {&layout.mailbox_count, &layout.mailbox_words,
		&layout.slot_bytes, &layout.timesync_mailbox};
	u32 seen, i, j, side, count, stride;
	int family, ret, cells;
	bool legacy;

	if (!output)
		return -EINVAL;
	legacy = of_property_present(node, keys[1][0]);
	family = legacy ? 1 : 0;
	for (i = 0; i < 8; i++) {
		if (!of_property_present(node, keys[family][i]) ||
		    of_property_present(node, keys[!family][i]))
			return -EINVAL;
	}
	for (i = 0; i < 4; i++) {
		ret = of_property_read_u32(node, keys[family][i], scalars[i]);
		if (ret)
			return ret;
	}
	/* Mailbox is a four-bit field; offsets are twenty-bit word offsets. */
	if (!layout.mailbox_count || layout.mailbox_count > GPUEB_MAX_MAILBOXES ||
	    !layout.mailbox_words || layout.mailbox_words > (1U << 20) ||
	    layout.slot_bytes != sizeof(u32) ||
	    layout.timesync_mailbox >= layout.mailbox_count)
		return -ERANGE;
	for (side = 0; side < 2; side++) {
		struct gpueb_ipi_pin_layout *pins = side ? layout.receive : layout.send;
		const char *table = keys[family][4 + side];
		const char *names = keys[family][6 + side];

		/* Old receive rows omit the callback field: zero means ISR context,
		 * matching the zero-initialized legacy mtk_mbox_pin_recv structure.
		 */
		stride = side ? (legacy ? 4 : 5) : 3;
		cells = of_property_count_u32_elems(node, table);
		if (cells <= 0 || cells % stride)
			return -EINVAL;
		count = cells / stride;
		if (count > GPUEB_MAX_CHANNELS ||
		    of_property_count_strings(node, names) != (int)count)
			return -EINVAL;
		seen = 0;
		for (i = 0; i < count; i++) {
			struct gpueb_ipi_pin_layout *pin = &pins[i];
			u32 *fields[] = {&pin->channel, &pin->mailbox, &pin->words,
				&pin->receive_option, &pin->callback_context};

			for (j = 0; j < stride; j++) {
				ret = of_property_read_u32_index(node, table, i * stride + j, fields[j]);
				if (ret)
					return ret;
			}
			if (pin->channel >= count || (seen & (1U << pin->channel)) ||
			    pin->mailbox >= layout.mailbox_count || !pin->words ||
			    pin->receive_option > 1 || pin->callback_context > 1)
				return -EINVAL;
			seen |= 1U << pin->channel;
			if (pin->words > layout.mailbox_words - used[pin->mailbox])
				return -EOVERFLOW;
			used[pin->mailbox] += pin->words;
			ret = of_property_read_string_index(node, names, i, &pin->name);
			if (ret)
				return ret;
			if (!pin->name[0])
				return -EINVAL;
			for (j = 0; j < i; j++)
				if (!strcmp(pin->name, pins[j].name))
					return -EINVAL;
		}
		if (side)
			layout.receive_count = count;
		else
			layout.send_count = count;
	}
	/* mtk_ipi_device_register registers one channel count for both directions. */
	if (layout.send_count != layout.receive_count)
		return -EINVAL;
	for (i = 0; i < layout.send_count; i++)
		for (j = 0; j < layout.receive_count; j++)
			if (layout.send[i].channel == layout.receive[j].channel &&
			    strcmp(layout.send[i].name, layout.receive[j].name))
				return -EINVAL;
	*output = layout;
	return 0;
}
#endif
