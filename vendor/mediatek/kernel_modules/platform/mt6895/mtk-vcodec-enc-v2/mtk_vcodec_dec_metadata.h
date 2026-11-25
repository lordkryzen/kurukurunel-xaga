/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_DEC_NATIVE_METADATA_H
#define MTK_DEC_NATIVE_METADATA_H
/* Native C2 scans eight16-byte descriptors. Color type5 reads18 U32 words.
 * Legacy decoder firmware instead writes a raw16-byte frame record at byte0.
 * Publish only a color description queried from the real firmware, never
 * reinterpret legacy frame statistics as descriptor flags/type/offset.
 */
#define MTK_DEC_META_HEADER 128U
#define MTK_DEC_META_COLOR_WORDS 18U
#define MTK_DEC_META_COLOR_BYTES (MTK_DEC_META_COLOR_WORDS * sizeof(u32))
static inline int mtk_dec_native_metadata(void *dst, size_t size,
 const u32 *color, bool have_color)
{
 u32 *p = dst;
 if (!dst || size < MTK_DEC_META_HEADER + MTK_DEC_META_COLOR_BYTES)
  return -EINVAL;
 memset(dst, 0, size);
 if (!have_color)
  return 0;
 memcpy((char *)dst + MTK_DEC_META_HEADER, color, 17 * sizeof(u32));
 p[1] = 5; /* Color-aspects metadata, as consumed by native C2. */
 p[2] = MTK_DEC_META_COLOR_WORDS; /* Descriptor sizes use32-bit words. */
 p[3] = MTK_DEC_META_HEADER;
 p[0] = 1; /* inline payload, valid; all other descriptors remain unavailable. */
 return 0;
}
#endif
