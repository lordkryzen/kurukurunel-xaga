/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_CAPACITY_TABLE_H
#define MTK_CAPACITY_TABLE_H
#define CAP_MAX_DOMAINS 3
#define CAP_MAX_OPPS 32
#define CAPACITY_OFFSET 0xfa0
#define CAPACITY_BYTES 0x100
#define CAPACITY_WORDS (CAPACITY_BYTES / sizeof(u16))
struct capacity_domain {
	unsigned int count;
	unsigned int frequency[CAP_MAX_OPPS];
	u16 capacity[CAP_MAX_OPPS];
};

/* Decode xaga's native firmware table. The final capacity is repeated as
 * a domain delimiter. Validate the entire snapshot before publishing data. */
static int capacity_decode(const u16 *raw, size_t words,
		struct capacity_domain *domains, unsigned int count)
{
	u16 decoded[CAP_MAX_DOMAINS][CAP_MAX_OPPS] = {};
	size_t offset = 0;
	unsigned int i, j;

	if (!raw || !domains || !count || count > CAP_MAX_DOMAINS)
		return -EINVAL;
	if (words > CAPACITY_WORDS)
		return -E2BIG;
	for (i = 0; i < count; i++) {
		unsigned int nr = domains[i].count;

		if (!nr || nr > CAP_MAX_OPPS)
			return -EINVAL;
		if (offset > words || nr + 1 > words - offset)
			return -EOVERFLOW;
		for (j = 0; j < nr; j++) {
			u16 cap = raw[offset + j];

			if (!cap || cap > 1024 ||
			    (j && cap > decoded[i][j - 1]))
				return -EINVAL;
			if (!domains[i].frequency[j] ||
			    (j && domains[i].frequency[j] >= domains[i].frequency[j - 1]))
				return -EINVAL;
			decoded[i][j] = cap;
		}
		if (raw[offset + nr] != decoded[i][nr - 1])
			return -EINVAL;
		offset += nr + 1;
	}
	for (i = 0; i < count; i++)
		memcpy(domains[i].capacity, decoded[i], sizeof(decoded[i]));
	return 0;
}
#endif
