/* Generated native MT6895 diagnostic tables; see board-profile.json. */
#ifndef NATIVE_VIDEO_BOARD_PROFILE_H
#define NATIVE_VIDEO_BOARD_PROFILE_H
#include <linux/of.h>
#include <linux/string.h>
#include <linux/errno.h>
static const u32 native_table_0[] = {0x0, 0x0, 0x12, 0x2, 0x1, 0x2, 0x3, 0x1, 0x1, 0x9, 0x2, 0x12, 0xb, 0x2, 0x4, 0xd, 0x3, 0x2, 0xe, 0x3, 0x1, 0xf, 0x3, 0x6, 0x10, 0x3, 0x2, 0x15, 0x4, 0x4};
static const u32 native_table_1[] = {0x1, 0x0, 0x12, 0x0, 0x2, 0x1, 0x1, 0x1, 0x4, 0x1, 0xa, 0x0, 0x5, 0x1, 0x1, 0x0, 0x6, 0x1, 0x2, 0x0, 0xa, 0x2, 0x12, 0x0, 0xc, 0x2, 0x4, 0x0, 0xd, 0x3, 0x1, 0x1, 0x11, 0x3, 0xa, 0x0, 0x12, 0x3, 0x6, 0x0, 0x13, 0x3, 0x1, 0x0, 0x14, 0x3, 0x2, 0x0, 0x16, 0x4, 0x4, 0x0};
static const u32 native_table_2[] = {0x0, 0x78000, 0x1, 0x8000, 0x2, 0x180000, 0x3, 0x400, 0x4, 0x400, 0x5, 0x400, 0x6, 0x400, 0x7, 0x100000};
static const u32 native_table_3[] = {0xa};
struct native_video_table {const char *path, *key; const u32 *values; size_t count;};
static const struct native_video_table native_video_tables[] = {
{"/soc/vcp@1ec00000", "send_table", native_table_0, ARRAY_SIZE(native_table_0)},
{"/soc/vcp@1ec00000", "recv_table", native_table_1, ARRAY_SIZE(native_table_1)},
{"/soc/vcp@1ec00000", "vcp_mem_tbl", native_table_2, ARRAY_SIZE(native_table_2)},
{"/soc/vcp@1ec00000", "femter_ck", native_table_3, ARRAY_SIZE(native_table_3)},
};
static inline const struct native_video_table *native_video_find(const struct device_node *node, const char *key)
{
 size_t i;
 if (!node) return NULL;
 for (i=0; i<ARRAY_SIZE(native_video_tables); i++)
  if (!strcmp(key,native_video_tables[i].key)) {
   struct device_node *target = of_find_node_by_path(native_video_tables[i].path);
   bool match = target == node;
   of_node_put(target);
   if (match) return &native_video_tables[i];
  }
 return NULL;
}
static inline int native_video_count(const struct device_node *node,const char *key)
{
 const struct native_video_table *p=native_video_find(node,key);
 return p ? (int)p->count : of_property_count_u32_elems(node,key);
}
static inline int native_video_index(const struct device_node *node,const char *key,u32 index,u32 *value)
{
 const struct native_video_table *p=native_video_find(node,key);
 if (!p) return of_property_read_u32_index(node,key,index,value);
 if (!value || index>=p->count) return -EINVAL;
 *value=p->values[index];return 0;
}
static inline int native_video_read(const struct device_node *node,const char *key,u32 *value)
{
 return native_video_index(node,key,0,value);
}
#define of_property_count_u32_elems native_video_count
#define of_property_read_u32_index native_video_index
#define of_property_read_u32 native_video_read
#endif
