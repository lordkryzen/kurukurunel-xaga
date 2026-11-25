// SPDX-License-Identifier: GPL-2.0
/* Read-only xaga SWPM provider/PMU survey: no firmware commands or memory writes. */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cpu.h>
#include <linux/spinlock.h>
#include <sspm_reservedmem_define.h>
#include <swpm_perf_arm_pmu.h>
#include <swpm_v6895.h>
#include <swpm_v6895_ext.h>
extern int swpm_arm_pmu_read_count(unsigned int id, unsigned int cpu,
		u64 *count, u64 *enabled, u64 *running);
extern int swpm_native_calibration_init(void);
extern int get_devinfo(int index);
static struct proc_dir_entry *entry;
static int survey_show(struct seq_file *m, void *unused)
{
	phys_addr_t phys = sspm_reserve_mem_get_phys(SWPM_MEM_ID);
	phys_addr_t virt = sspm_reserve_mem_get_virt(SWPM_MEM_ID);
	phys_addr_t size = sspm_reserve_mem_get_size(SWPM_MEM_ID);
	unsigned int cpu, id;
	seq_printf(m, "native_core_static_ma=%d (zero selects native96mA fallback)\n", get_devinfo(7));

	seq_printf(m, "memory_id=%u phys=%pa bytes=%llu mapped=%u record=%zu wrap=%zu index=%zu ctrl=%zu ext_index=%zu ext_ctrl=%zu pmu_status=0x%x\n",
		SWPM_MEM_ID, &phys, (unsigned long long)size, !!virt,
		sizeof(struct swpm_rec_data), sizeof(struct share_wrap),
		sizeof(struct share_index), sizeof(struct share_ctrl),
		sizeof(struct share_index_ext), sizeof(struct share_ctrl_ext),
		swpm_arm_pmu_get_status());
	if (!phys || !virt || size != 8192 || size < sizeof(struct swpm_rec_data))
		return -EINVAL;
	for_each_possible_cpu(cpu) {
		for (id = 0; id < DSU_CYCLES_EVT; id++) {
			u64 count = 0, enabled = 0, running = 0;
			int ret = swpm_arm_pmu_read_count(id, cpu, &count, &enabled, &running);
			seq_printf(m, "cpu=%u event=%u index=%d rc=%d count=%llu enabled=%llu running=%llu\n",
				cpu, id, swpm_arm_pmu_get_idx(id, cpu), ret,
				count, enabled, running);
		}
	}
	return 0;
}
static int survey_open(struct inode *inode, struct file *file)
{
	return single_open(file, survey_show, NULL);
}
static const struct proc_ops survey_ops = {
	.proc_open = survey_open, .proc_read = seq_read,
	.proc_lseek = seq_lseek, .proc_release = single_release,
};
static int __init survey_init(void)
{
	struct device_node *node;
	int ret = swpm_native_calibration_init();
	if (ret) return ret;
	node = of_find_compatible_node(NULL, NULL, "mediatek,mtk-swpm");
	if (!node)
		return -ENODEV;
	if (!of_device_is_available(node)) {
		of_node_put(node);
		return -ENODEV;
	}
	of_node_put(node);
	entry = proc_create("swpm_contract_survey", 0400, NULL, &survey_ops);
	return entry ? 0 : -ENOMEM;
}
static void __exit survey_exit(void) { proc_remove(entry); }
module_init(survey_init);
module_exit(survey_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Read-only MT6895 SWPM contract survey");
