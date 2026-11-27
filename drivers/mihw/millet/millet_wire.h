/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _MILLET_WIRE_H
#define _MILLET_WIRE_H

#include <linux/types.h>

#define MILLET_KERNEL_PORT 0x12341234ULL
#define MILLET_USER_PORT 0xabcddcbaULL
#define MILLET_CHANNELS 7

enum millet_channel {
	MILLET_SIGNAL = 1,
	MILLET_BINDER,
	MILLET_BINDER_STAT,
	MILLET_RESERVED,
	MILLET_PACKET,
	MILLET_HANDSHAKE,
};

enum millet_kind {
	MILLET_LOOPBACK = 1,
	MILLET_EVENT,
	MILLET_COMMAND,
};

struct millet_command {
	__u32 channel;
	__u32 kind;
	__u64 source;
	__u64 destination;
	__u8 reserved[48];
	__u32 command;
	__u32 uid;
};

struct millet_event {
	__u32 channel;
	__u32 monitor;
	__u32 kind;
	__u32 reserved0;
	__u64 source;
	__u64 destination;
	__u32 uid;
	__u32 reserved1;
	__u64 seconds;
	__u64 nanoseconds;
	__u64 binder_event;
	__u8 reserved2[40];
	__u64 reserved_pointer0;
	__u64 reserved_pointer1;
	__u32 caller_uid;
	__u32 caller_pid;
	__u32 caller_tid;
	__u32 target_pid;
	__u8 oneway;
	__u8 reserved3[3];
	__u32 code;
};

#endif
