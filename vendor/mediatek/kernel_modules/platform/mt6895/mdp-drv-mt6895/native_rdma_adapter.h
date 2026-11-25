/* SPDX-License-Identifier: GPL-2.0 */
/* Modern RDMA metadata on MT6895, without changing the ATF CPR program ABI.
 * Register offsets match the donor cache and the target's mtk-mml-rdma.c.
 * Each submitted job owns its pipe caches; reselection preserves unchanged
 * words just like the firmware CPR image, without borrowing another job's DMA.
 * The existing prebuilt lock remains held; execute the supplied register image
 * in this task instead of handing a 50-entry image to the old 20-entry firmware.
 */
#define MDP_A16_RDMA_REGS 50
#define MDP_A16_RDMA_WAIT 680
#define MDP_A16_RDMA_DONE 681

struct mdp_a16_rdma_state {
	u32 value[MDP_A16_RDMA_REGS];
	u32 mask[MDP_A16_RDMA_REGS];
	u64 valid;
	u16 engine;
	bool selected, active, completed;
};

static const u16 mdp_a16_rdma_offset[MDP_A16_RDMA_REGS] = {
	0x118, 0x120, 0x128, 0x148, 0x150, 0x200,
	0xf00, 0xf08, 0xf10, 0xf20, 0xf28,
	0xf30, 0xf34, 0xf38, 0xf3c, 0xf40,
	0xf44, 0xf48, 0xf4c, 0xf50,
	0x000, 0x020, 0x024, 0x028, 0x030, 0x038,
	0x060, 0x068, 0x070, 0x078, 0x080, 0x090, 0x098,
	0x240, 0x244, 0x248, 0x250,
	0x254, 0x258, 0x260, 0x264,
	0x268, 0x270, 0x274, 0x278,
	0x280, 0x284, 0x288, 0x290, 0x2a0,
};

static int mdp_a16_rdma_store(struct mdp_a16_rdma_state *state,
			     u8 index, u32 value, u32 mask)
{
	if (index >= MDP_A16_RDMA_REGS)
		return -EINVAL;
	state->value[index] = value;
	state->mask[index] = mask;
	state->valid |= BIT_ULL(index);
	state->active = true;
	return 0;
}

static int mdp_a16_rdma_emit(struct mdp_a16_rdma_state *state,
	struct cmdqRecStruct *handle, struct cmdq_command_buffer *cmd_buf)
{
	u32 addr;
	int i, ret;

	/* A fresh job/pipe requires a complete image. Later selections may
	 * update its cached image; state is never borrowed from another job.
	 */
	if (state->completed ||
	    state->valid != GENMASK_ULL(MDP_A16_RDMA_REGS - 1, 0))
		return -EINVAL;
	for (i = 0; i < MDP_A16_RDMA_REGS; i++) {
		addr = cmdq_mdp_get_hw_reg(state->engine, mdp_a16_rdma_offset[i]);
		if (!addr)
			return -EINVAL;
		ret = cmdq_op_write_reg_ex(handle, cmd_buf, addr,
			state->value[i], state->mask[i]);
		if (ret)
			return ret;
	}
	state->active = false;
	state->completed = true;
	return 0;
}
