/* Native HAL copies0x324B at constructor157f5c; QBUFconfigparam+84.
 * Private wire type; do not change native register or user headers. */
struct dpe_v4l2_config_wire {
	enum DPEMODE Dpe_engineSelect;
	unsigned int Dpe_is16BitMode;
	struct DVS_Settings	Dpe_DVSSettings;
	struct DVP_Settings	Dpe_DVPSettings;
	struct DPE_Config_map DPE_DMapSettings;
	u32 Dpe_InBuf_SrcImg_Y_L;
	u32 Dpe_InBuf_SrcImg_Y_R;
	u32 Dpe_InBuf_SrcImg_Y;
	u32 Dpe_InBuf_SrcImg_C;
	u32 Dpe_InBuf_ValidMap_L;
	u32 Dpe_InBuf_ValidMap_R;
	u32 Dpe_OutBuf_CONF;
	u32 Dpe_OutBuf_OCC;
	u32 Dpe_OutBuf_OCC_Ext;
	u32 Dpe_InBuf_OCC;
	u32 Dpe_InBuf_OCC_Ext;
	u32 Dpe_OutBuf_CRM;
	u32 Dpe_OutBuf_ASF_RM;
	u32 Dpe_OutBuf_ASF_RM_Ext;
	u32 Dpe_OutBuf_ASF_RD;
	u32 Dpe_OutBuf_ASF_RD_Ext;
	u32 Dpe_OutBuf_ASF_HF;
	u32 Dpe_OutBuf_ASF_HF_Ext;
	u32 Dpe_OutBuf_WMF_HF;
	u32 Dpe_OutBuf_WMF_FILT;
	u32 DVP_SRC_18_ASF_RMDV;
	u32 DVP_SRC_24_WMF_HFDV;
	u32 DVP_EXT_SRC_18_ASF_RMDV;
	u32 Dpe_InBuf_SrcImg_Y_L_Pre;
	u32 Dpe_InBuf_SrcImg_Y_R_Pre;
	u32 Dpe_InBuf_P4_L_DV;
	u32 Dpe_InBuf_P4_R_DV;
	struct DPE_feedback	Dpe_feedback;
};
static_assert(sizeof(struct dpe_v4l2_config_wire) == 804);
static_assert(offsetof(struct dpe_v4l2_config_wire, Dpe_InBuf_SrcImg_Y_L) == 688);
static_assert(offsetof(struct dpe_v4l2_config_wire, Dpe_feedback) == 796);
static_assert(offsetof(struct DPE_Config, Dpe_InBuf_SrcImg_Y_L) == 688);
static_assert(sizeof(struct DPE_Config) == 912);
static void dpe_unpack_config(struct DPE_Config *out,
		const struct dpe_v4l2_config_wire *wire)
{
	memset(out, 0, sizeof(*out));
	memcpy(out, wire, offsetof(struct dpe_v4l2_config_wire, Dpe_InBuf_SrcImg_Y_L));
	out->Dpe_InBuf_SrcImg_Y_L = wire->Dpe_InBuf_SrcImg_Y_L;
	out->Dpe_InBuf_SrcImg_Y_R = wire->Dpe_InBuf_SrcImg_Y_R;
	out->Dpe_InBuf_SrcImg_Y = wire->Dpe_InBuf_SrcImg_Y;
	out->Dpe_InBuf_SrcImg_C = wire->Dpe_InBuf_SrcImg_C;
	out->Dpe_InBuf_ValidMap_L = wire->Dpe_InBuf_ValidMap_L;
	out->Dpe_InBuf_ValidMap_R = wire->Dpe_InBuf_ValidMap_R;
	out->Dpe_OutBuf_CONF = wire->Dpe_OutBuf_CONF;
	out->Dpe_OutBuf_OCC = wire->Dpe_OutBuf_OCC;
	out->Dpe_OutBuf_OCC_Ext = wire->Dpe_OutBuf_OCC_Ext;
	out->Dpe_InBuf_OCC = wire->Dpe_InBuf_OCC;
	out->Dpe_InBuf_OCC_Ext = wire->Dpe_InBuf_OCC_Ext;
	out->Dpe_OutBuf_CRM = wire->Dpe_OutBuf_CRM;
	out->Dpe_OutBuf_ASF_RM = wire->Dpe_OutBuf_ASF_RM;
	out->Dpe_OutBuf_ASF_RM_Ext = wire->Dpe_OutBuf_ASF_RM_Ext;
	out->Dpe_OutBuf_ASF_RD = wire->Dpe_OutBuf_ASF_RD;
	out->Dpe_OutBuf_ASF_RD_Ext = wire->Dpe_OutBuf_ASF_RD_Ext;
	out->Dpe_OutBuf_ASF_HF = wire->Dpe_OutBuf_ASF_HF;
	out->Dpe_OutBuf_ASF_HF_Ext = wire->Dpe_OutBuf_ASF_HF_Ext;
	out->Dpe_OutBuf_WMF_HF = wire->Dpe_OutBuf_WMF_HF;
	out->Dpe_OutBuf_WMF_FILT = wire->Dpe_OutBuf_WMF_FILT;
	out->DVP_SRC_18_ASF_RMDV = wire->DVP_SRC_18_ASF_RMDV;
	out->DVP_SRC_24_WMF_HFDV = wire->DVP_SRC_24_WMF_HFDV;
	out->DVP_EXT_SRC_18_ASF_RMDV = wire->DVP_EXT_SRC_18_ASF_RMDV;
	out->Dpe_InBuf_SrcImg_Y_L_Pre = wire->Dpe_InBuf_SrcImg_Y_L_Pre;
	out->Dpe_InBuf_SrcImg_Y_R_Pre = wire->Dpe_InBuf_SrcImg_Y_R_Pre;
	out->Dpe_InBuf_P4_L_DV = wire->Dpe_InBuf_P4_L_DV;
	out->Dpe_InBuf_P4_R_DV = wire->Dpe_InBuf_P4_R_DV;
	out->Dpe_feedback = wire->Dpe_feedback;
}
static int dpe_pack_config(struct dpe_v4l2_config_wire *wire,
		const struct DPE_Config *in)
{
	/* Validate all narrowing before modifying destination. */
	if (in->Dpe_InBuf_SrcImg_Y_L > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_SrcImg_Y_R > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_SrcImg_Y > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_SrcImg_C > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_ValidMap_L > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_ValidMap_R > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_CONF > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_OCC > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_OCC_Ext > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_OCC > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_OCC_Ext > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_CRM > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_ASF_RM > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_ASF_RM_Ext > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_ASF_RD > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_ASF_RD_Ext > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_ASF_HF > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_ASF_HF_Ext > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_WMF_HF > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_OutBuf_WMF_FILT > U32_MAX) return -EOVERFLOW;
	if (in->DVP_SRC_18_ASF_RMDV > U32_MAX) return -EOVERFLOW;
	if (in->DVP_SRC_24_WMF_HFDV > U32_MAX) return -EOVERFLOW;
	if (in->DVP_EXT_SRC_18_ASF_RMDV > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_SrcImg_Y_L_Pre > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_SrcImg_Y_R_Pre > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_P4_L_DV > U32_MAX) return -EOVERFLOW;
	if (in->Dpe_InBuf_P4_R_DV > U32_MAX) return -EOVERFLOW;
	memcpy(wire, in, offsetof(struct dpe_v4l2_config_wire, Dpe_InBuf_SrcImg_Y_L));
	wire->Dpe_InBuf_SrcImg_Y_L = in->Dpe_InBuf_SrcImg_Y_L;
	wire->Dpe_InBuf_SrcImg_Y_R = in->Dpe_InBuf_SrcImg_Y_R;
	wire->Dpe_InBuf_SrcImg_Y = in->Dpe_InBuf_SrcImg_Y;
	wire->Dpe_InBuf_SrcImg_C = in->Dpe_InBuf_SrcImg_C;
	wire->Dpe_InBuf_ValidMap_L = in->Dpe_InBuf_ValidMap_L;
	wire->Dpe_InBuf_ValidMap_R = in->Dpe_InBuf_ValidMap_R;
	wire->Dpe_OutBuf_CONF = in->Dpe_OutBuf_CONF;
	wire->Dpe_OutBuf_OCC = in->Dpe_OutBuf_OCC;
	wire->Dpe_OutBuf_OCC_Ext = in->Dpe_OutBuf_OCC_Ext;
	wire->Dpe_InBuf_OCC = in->Dpe_InBuf_OCC;
	wire->Dpe_InBuf_OCC_Ext = in->Dpe_InBuf_OCC_Ext;
	wire->Dpe_OutBuf_CRM = in->Dpe_OutBuf_CRM;
	wire->Dpe_OutBuf_ASF_RM = in->Dpe_OutBuf_ASF_RM;
	wire->Dpe_OutBuf_ASF_RM_Ext = in->Dpe_OutBuf_ASF_RM_Ext;
	wire->Dpe_OutBuf_ASF_RD = in->Dpe_OutBuf_ASF_RD;
	wire->Dpe_OutBuf_ASF_RD_Ext = in->Dpe_OutBuf_ASF_RD_Ext;
	wire->Dpe_OutBuf_ASF_HF = in->Dpe_OutBuf_ASF_HF;
	wire->Dpe_OutBuf_ASF_HF_Ext = in->Dpe_OutBuf_ASF_HF_Ext;
	wire->Dpe_OutBuf_WMF_HF = in->Dpe_OutBuf_WMF_HF;
	wire->Dpe_OutBuf_WMF_FILT = in->Dpe_OutBuf_WMF_FILT;
	wire->DVP_SRC_18_ASF_RMDV = in->DVP_SRC_18_ASF_RMDV;
	wire->DVP_SRC_24_WMF_HFDV = in->DVP_SRC_24_WMF_HFDV;
	wire->DVP_EXT_SRC_18_ASF_RMDV = in->DVP_EXT_SRC_18_ASF_RMDV;
	wire->Dpe_InBuf_SrcImg_Y_L_Pre = in->Dpe_InBuf_SrcImg_Y_L_Pre;
	wire->Dpe_InBuf_SrcImg_Y_R_Pre = in->Dpe_InBuf_SrcImg_Y_R_Pre;
	wire->Dpe_InBuf_P4_L_DV = in->Dpe_InBuf_P4_L_DV;
	wire->Dpe_InBuf_P4_R_DV = in->Dpe_InBuf_P4_R_DV;
	wire->Dpe_feedback = in->Dpe_feedback;
	return 0;
}
static int dpe_get_v4l2_configs(struct DPE_Config *out,
		const void __user *arg, unsigned int count)
{
	struct dpe_v4l2_config_wire *wire __free(kfree) = NULL;
	unsigned int i;
	if (!count || count > 3) return -EINVAL;
	wire = kcalloc(count, sizeof(*wire), GFP_KERNEL);
	if (!wire) return -ENOMEM;
	if (copy_from_user(wire, arg, count * sizeof(*wire))) return -EFAULT;
	for (i = 0; i < count; i++) dpe_unpack_config(&out[i], &wire[i]);
	return 0;
}
static int dpe_put_v4l2_configs(void __user *arg,
		const struct DPE_Config *in, unsigned int count)
{
	struct dpe_v4l2_config_wire *wire __free(kfree) = NULL;
	unsigned int i;
	int ret;
	if (!count || count > 3) return -EINVAL;
	wire = kcalloc(count, sizeof(*wire), GFP_KERNEL);
	if (!wire) return -ENOMEM;
	for (i = 0; i < count; i++) {
		ret = dpe_pack_config(&wire[i], &in[i]);
		if (ret) return ret;
	}
	return copy_to_user(arg, wire, count * sizeof(*wire)) ? -EFAULT : 0;
}
