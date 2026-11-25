/* SPDX-License-Identifier: GPL-2.0 */
#ifndef CCCI_LK_MODEM_INFO_H
#define CCCI_LK_MODEM_INFO_H
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>
/* Canonical kernel state; never copy an LK wire record directly into it. */
struct _modem_info {
 unsigned long long base_addr;
 unsigned int size;
 char md_id;
 char errno;
 unsigned int md_type;
 char ver;
 unsigned int reserved[2];
};
#define CCCI_LK_MODEM_MAX_RECORDS 4U
#define CCCI_LK_MODEM_MAX_BYTES (CCCI_LK_MODEM_MAX_RECORDS * 32U)
static int ccci_lk_decode_modem_info(const u8 *raw, unsigned int bytes,
                                  unsigned int count, struct _modem_info *out)
{
 struct _modem_info value = {0};
 unsigned int stride, i, off;
 bool found = false;
 if (!raw || !out || !count || count > CCCI_LK_MODEM_MAX_RECORDS)
  return -EINVAL;
 if (bytes != count * 24U && bytes != count * 32U)
  return -EINVAL;
 stride = bytes / count;
 for (i = 0; i < count; i++) {
  off = i * stride;
  /* The single-modem kernel consumes MD1 (wire ID0), not array position0. */
  if (raw[off + 12] != 0)
   continue;
  if (found)
   return -EINVAL;
  found = true;
  value.base_addr = get_unaligned_le64(raw + off);
  value.size = get_unaligned_le32(raw + off + 8);
  value.md_id = raw[off + 12];
  value.errno = raw[off + 13];
  if (stride == 24U) {
   value.md_type = raw[off + 14];
   value.ver = raw[off + 15];
   value.reserved[0] = get_unaligned_le32(raw + off + 16);
   value.reserved[1] = get_unaligned_le32(raw + off + 20);
  } else {
   value.md_type = get_unaligned_le32(raw + off + 16);
   value.ver = raw[off + 20];
   value.reserved[0] = get_unaligned_le32(raw + off + 24);
   value.reserved[1] = get_unaligned_le32(raw + off + 28);
  }
 }
 if (!found)
  return -ENOENT;
 *out = value;
 return 0;
}
#endif
