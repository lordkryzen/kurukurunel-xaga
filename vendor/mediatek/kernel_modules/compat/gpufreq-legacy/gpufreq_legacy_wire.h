/* SPDX-License-Identifier: GPL-2.0 */
#ifndef GPUFREQ_LEGACY_WIRE_H
#define GPUFREQ_LEGACY_WIRE_H
#include <linux/build_bug.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/stddef.h>

/* MT6895 firmware uses the pre-magic, eight-word mailbox layout. */
struct gpufreq_legacy_ipi {
	u32 command;
	u32 target;
	union {
		s32 value;
		u32 unsigned_value;
		struct { u64 status_base, debug_base; u32 status_size, debug_size; } memory;
		struct { u32 frequency, voltage; } custom;
		struct { u32 limiter; s32 ceiling, floor; } limit;
	} payload;
};
struct gpufreq_legacy_memory {
	u64 base;
	u32 size;
};
static_assert(sizeof(struct gpufreq_legacy_ipi) == 32);
static_assert(offsetof(struct gpufreq_legacy_ipi, payload) == 8);
static_assert(offsetof(struct gpufreq_legacy_ipi, payload.memory.debug_base) == 16);

static inline int gpufreq_legacy_limiter(u32 native, u32 *legacy)
{
	switch (native) {
	case LIMIT_SEGMENT: *legacy = 0; break;
	case LIMIT_DEBUG: *legacy = 1; break;
	case LIMIT_THERMAL_AP: *legacy = 2; break;
	case LIMIT_THERMAL_EB: *legacy = 3; break;
	case LIMIT_SRAMRC: *legacy = 4; break;
	case LIMIT_BATT_OC: *legacy = 5; break;
	case LIMIT_BATT_PERCENT: *legacy = 6; break;
	case LIMIT_LOW_BATT: *legacy = 7; break;
	case LIMIT_PBM: *legacy = 8; break;
	case LIMIT_APIBOOST: *legacy = 9; break;
	/* GED ceiling/floor entry points renamed FPSGO to POWERHAL. */
	case LIMIT_POWERHAL: *legacy = 10; break;
	default: return -EOPNOTSUPP;
	}
	return 0;
}

static inline int gpufreq_native_limiter(u32 legacy, u32 *native)
{
	switch (legacy) {
	case 0: *native = LIMIT_SEGMENT; break;
	case 1: *native = LIMIT_DEBUG; break;
	case 2: *native = LIMIT_THERMAL_AP; break;
	case 3: *native = LIMIT_THERMAL_EB; break;
	case 4: *native = LIMIT_SRAMRC; break;
	case 5: *native = LIMIT_BATT_OC; break;
	case 6: *native = LIMIT_BATT_PERCENT; break;
	case 7: *native = LIMIT_LOW_BATT; break;
	case 8: *native = LIMIT_PBM; break;
	case 9: *native = LIMIT_APIBOOST; break;
	case 10: *native = LIMIT_POWERHAL; break;
	default: return -EOPNOTSUPP;
	}
	return 0;
}

/* On errors the caller's output remains untouched; no message may be sent. */
static inline int gpufreq_legacy_encode(const struct gpufreq_ipi_data *native,
	const struct gpufreq_legacy_memory *memory, struct gpufreq_legacy_ipi *output)
{
	struct gpufreq_legacy_ipi wire = {};
	u32 limiter;
	int ret;

	if (!native || !output || native->target >= TARGET_INVALID)
		return -EINVAL;
	wire.target = native->target;
	switch (native->cmd_id) {
	case CMD_INIT_SHARED_MEM:
		if (!memory || !memory->base || memory->size <= 0x400)
			return -ENODATA;
		if (memory->base < 0x40000000ULL || memory->base >= 0x800000000ULL ||
		    memory->size > 0x800000000ULL - memory->base ||
		    (memory->base & 3) || (memory->size & 3))
			return -ERANGE;
		wire.command = 0;
		wire.payload.memory.status_base = memory->base;
		wire.payload.memory.debug_base = memory->base + 0x400;
		wire.payload.memory.status_size = 0x400;
		wire.payload.memory.debug_size = memory->size - 0x400;
		break;
	case CMD_GET_FREQ_BY_IDX: wire.command = 1; wire.payload.value = native->u.oppidx; break;
	case CMD_GET_POWER_BY_IDX: wire.command = 2; wire.payload.value = native->u.oppidx; break;
	case CMD_GET_OPPIDX_BY_FREQ: wire.command = 3; wire.payload.unsigned_value = native->u.freq; break;
	case CMD_GET_LEAKAGE_POWER: wire.command = 4; wire.payload.unsigned_value = native->u.volt; break;
	case CMD_SET_LIMIT:
	case CMD_SWITCH_LIMIT:
		ret = gpufreq_legacy_limiter(native->u.set_limit.limiter, &limiter);
		if (ret)
			return ret;
		wire.command = native->cmd_id == CMD_SET_LIMIT ? 5 : 13;
		wire.payload.limit.limiter = limiter;
		wire.payload.limit.ceiling = native->u.set_limit.ceiling_info;
		wire.payload.limit.floor = native->u.set_limit.floor_info;
		break;
	case CMD_POWER_CONTROL: wire.command = 6; wire.payload.unsigned_value = native->u.power_state; break;
	case CMD_COMMIT: wire.command = 7; wire.payload.value = native->u.oppidx; break;
	case CMD_FIX_TARGET_OPPIDX: wire.command = 14; wire.payload.value = native->u.oppidx; break;
	case CMD_FIX_CUSTOM_FREQ_VOLT:
		wire.command = 15;
		wire.payload.custom.frequency = native->u.custom.freq;
		wire.payload.custom.voltage = native->u.custom.volt;
		break;
	default:
		return -EOPNOTSUPP;
	}
	memcpy(output, &wire, sizeof(wire));
	return 0;
}

static inline int gpufreq_legacy_decode(const struct gpufreq_ipi_data *request,
	const struct gpufreq_legacy_ipi *sent, const struct gpufreq_legacy_ipi *reply,
	struct gpufreq_ipi_data *output)
{
	struct gpufreq_ipi_data native = {};
	u32 expected;

	if (!request || !sent || !reply || !output)
		return -EINVAL;
	switch (request->cmd_id) {
	case CMD_INIT_SHARED_MEM: expected = 0; break;
	case CMD_GET_FREQ_BY_IDX: expected = 1; break;
	case CMD_GET_POWER_BY_IDX: expected = 2; break;
	case CMD_GET_OPPIDX_BY_FREQ: expected = 3; break;
	case CMD_GET_LEAKAGE_POWER: expected = 4; break;
	case CMD_SET_LIMIT: expected = 5; break;
	case CMD_POWER_CONTROL: expected = 6; break;
	case CMD_COMMIT: expected = 7; break;
	case CMD_SWITCH_LIMIT: expected = 13; break;
	case CMD_FIX_TARGET_OPPIDX: expected = 14; break;
	case CMD_FIX_CUSTOM_FREQ_VOLT: expected = 15; break;
	default: return -EOPNOTSUPP;
	}
	if (sent->command != expected || sent->target != request->target ||
	    (reply->command && reply->command != sent->command))
		return -EPROTO;
	/* Live stock debug queries acknowledge with a zero header, not an echo. */
	native.cmd_id = request->cmd_id;
	native.target = request->target;
	/* Stock INIT checks IPI completion only. Firmware may retain the prior
	 * query's payload; shared-status validation establishes initialization.
	 */
	native.u.return_value = request->cmd_id == CMD_INIT_SHARED_MEM ?
		0 : reply->payload.value;
	memcpy(output, &native, sizeof(native));
	return 0;
}
#endif
