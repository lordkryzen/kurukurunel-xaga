/* SPDX-License-Identifier: GPL-2.0 */
/* MT6895 OS3.0.5 Android16: offsets verified against the matching donor KO.
 * Do not cast this region to either generic gpufreq_shared_status structure.
 */
#ifndef GPUFREQ_MT6895_A16_WIRE_H
#define GPUFREQ_MT6895_A16_WIRE_H
#define A16_SHARED_SIZE 16384U
#define A16_OPP_CAPACITY 70U
#define A16_WORKING_GPU 0x47cU
#define A16_WORKING_STACK 0xb0cU
#define A16_ROW_BYTES 24U

/* Caller supplies an immutable CPU snapshot, never an MMIO pointer. */
static inline int a16_u32(const unsigned char *b, unsigned int size,
    unsigned int offset, unsigned int *out)
{
    if (!b || !out || size < 4 || offset > size - 4 || (offset & 3))
        return -EINVAL;
    *out = (unsigned int)b[offset] | ((unsigned int)b[offset + 1] << 8) |
        ((unsigned int)b[offset + 2] << 16) | ((unsigned int)b[offset + 3] << 24);
    return 0;
}

static inline int a16_validate_snapshot(const unsigned char *b, unsigned int size)
{
    unsigned int domain, count, index, offset, row, col, value, previous;
    if (!b || size != A16_SHARED_SIZE)
        return -EINVAL;
    for (domain = 0; domain < 2; domain++) {
        if (a16_u32(b, size, 12 + domain * 4, &count) ||
            count != (domain ? 40U : 6U) ||
            a16_u32(b, size, 4 + domain * 4, &index) || index >= count)
            return -EPROTO;
        offset = domain ? A16_WORKING_STACK : A16_WORKING_GPU;
        previous = ~0U;
        for (row = 0; row < count; row++) {
            for (col = 0; col < 6; col++) {
                if (a16_u32(b, size, offset + row * A16_ROW_BYTES + col * 4, &value))
                    return -ERANGE;
                /* Ordinary xaga operating envelope, not signed test OPPs. */
                if (!col && (!value || value > (domain ? 852000U : 950000U) ||
                    value >= previous)) return -ERANGE;
                if (!col) previous = value;
                if ((col == 1 || col == 2) && (value < 40000 || value > 100000))
                    return -ERANGE;
                if (col == 3 && value > 4) return -ERANGE;
            }
        }
    }
    if (a16_u32(b, size, 0xb4, &index) || index >= 40 ||
        a16_u32(b, size, 0xb8, &count) || count >= 40 || index > count)
        return -ERANGE;
    return 0;
}
#endif
