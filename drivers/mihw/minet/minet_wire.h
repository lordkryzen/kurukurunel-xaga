/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _MINET_WIRE_H
#define _MINET_WIRE_H
#include <linux/types.h>
struct minet_command {
	__u16 version;
	__u16 operation;
	__u32 count;
	__u32 uids[];
};
struct minet_flow {
	__u8 protocol;
	__u8 reserved;
	__u16 family;
	__u16 remote_port;
	__u16 local_port;
	__u8 remote_address[16];
	__u8 local_address[16];
	__u32 rx_packets;
	__u32 rx_bytes;
	__u32 tx_packets;
	__u32 tx_bytes;
};
struct minet_report {
	__u16 version;
	__u16 reserved;
	__u32 uid;
	__u16 count;
	__u16 reserved2;
	struct minet_flow flows[];
};
#endif
