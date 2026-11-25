/* Native 5.10 decoder port list; newer DT appends ports absent on this board. */
#include <linux/of.h>
#include <linux/string.h>
static const u32 native_decoder_ports[] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x10087, 0x10088, 0x89, 0x8a, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0x100a5, 0xa6, 0xa7, 0xa8};
static const char *const native_decoder_names[] = {"M4U_PORT_VDEC_MC", "M4U_PORT_VDEC_UFO", "M4U_PORT_VDEC_PP", "M4U_PORT_VDEC_PRED_RD", "M4U_PORT_VDEC_PRED_WR", "M4U_PORT_VDEC_PPWRAP", "M4U_PORT_VDEC_TILE", "M4U_PORT_VDEC_VLD", "M4U_PORT_VDEC_VLD2", "M4U_PORT_VDEC_AVC_MV", "M4U_PORT_VDEC_UFO_ENC", "M4U_PORT_VDEC_LAT0_VLD", "M4U_PORT_VDEC_LAT0_VLD2", "M4U_PORT_VDEC_LAT0_AVC_MV", "M4U_PORT_VDEC_LAT0_PRED_RD", "M4U_PORT_VDEC_LAT0_TILE", "M4U_PORT_VDEC_LAT0_WDMA", "M4U_PORT_VDEC_LAT0_UFO_ENC", "M4U_PORT_VDEC_LAT0_UFO_ENC_C", "M4U_PORT_VDEC_LAT0_MC"};
static inline bool native_decoder_target(const struct device_node *node)
{
 struct device_node *target = of_find_node_by_path("/soc/vdec@16000000");
 bool match = target == node;
 of_node_put(target);
 return match;
}
static inline int native_decoder_read_port(const struct device_node *node, const char *key, u32 index, u32 *value)
{
 if (!native_decoder_target(node) || strcmp(key,"m4u-ports")) return of_property_read_u32_index(node,key,index,value);
 if (!value || index>=ARRAY_SIZE(native_decoder_ports)) return -EINVAL;
 *value=native_decoder_ports[index];return 0;
}
static inline int native_decoder_read_port_name(const struct device_node *node, const char *key, int index, const char **value)
{
 if (!native_decoder_target(node) || strcmp(key,"m4u-port-names")) return of_property_read_string_index(node,key,index,value);
 if (!value || index<0 || index>=ARRAY_SIZE(native_decoder_names)) return -ENODATA;
 *value=native_decoder_names[index];return 0;
}
#define of_property_read_u32_index native_decoder_read_port
#define of_property_read_string_index native_decoder_read_port_name
