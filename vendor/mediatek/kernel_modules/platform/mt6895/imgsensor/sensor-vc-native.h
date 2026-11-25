/* Native MT6895 sensor VC UAPI: DT remapping adds one u16 per entry.
 * Keep the legacy in-kernel sensor/ISP structures unchanged.
 */
struct sensor_vc_native_entry {
 u16 feature, id, datatype, width, height, dt_remap;
 u16 bytes, output, pad;
};
struct sensor_vc_native_info {
 u16 count, pixels, mode, ratio, od, rg;
 struct sensor_vc_native_entry entry[9];
 u16 updated;
};
static_assert(sizeof(struct sensor_vc_native_entry) == 18);
static_assert(sizeof(struct sensor_vc_native_info) == 176);

static void sensor_vc_to_native(const struct SENSOR_VC_INFO2_STRUCT *old,
                               struct sensor_vc_native_info *wire)
{
 unsigned int i;
 memset(wire, 0, sizeof(*wire));
 wire->count = old->VC_Num;
 wire->pixels = old->VC_PixelNum;
 wire->mode = old->ModeSelect;
 wire->ratio = old->EXPO_Ratio;
 wire->od = old->ODValue;
 wire->rg = old->RG_STATSMODE;
 wire->updated = old->updated;
 for (i = 0; i < ARRAY_SIZE(wire->entry); i++) {
  const struct SINGLE_VC_INFO2 *a = &old->vc_info[i];
  struct sensor_vc_native_entry *b = &wire->entry[i];
  b->feature = a->VC_FEATURE;
  b->id = a->VC_ID;
  b->datatype = a->VC_DataType;
  b->width = a->VC_SIZEH_PIXEL;
  b->height = a->VC_SIZEV;
  /* Legacy xaga descriptors have no DT remapping request. Native NONE is0. */
  b->dt_remap = 0;
  b->bytes = a->VC_SIZEH_BYTE;
  b->output = a->VC_OUTPUT_FORMAT;
  b->pad = a->pad;
 }
}
