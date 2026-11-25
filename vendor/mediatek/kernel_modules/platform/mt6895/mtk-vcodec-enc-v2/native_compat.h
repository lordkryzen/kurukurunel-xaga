#include <linux/iosys-map.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/pm_runtime.h>
static inline size_t native_codec_strlcpy(char *dest, const char *src, size_t size)
{
 size_t length = strlen(src);
 if (size) {size_t copy = min(length, size - 1); memcpy(dest, src, copy); dest[copy] = 0;}
 return length;
}
#define strlcpy native_codec_strlcpy
