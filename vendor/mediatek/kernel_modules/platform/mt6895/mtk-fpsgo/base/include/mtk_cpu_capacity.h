/* SPDX-License-Identifier: GPL-2.0 */
#ifndef MTK_CPU_CAPACITY_H
#define MTK_CPU_CAPACITY_H
unsigned long mtk_cpu_opp_capacity(unsigned int cpu, int opp);
unsigned int mtk_cpu_opp_frequency(unsigned int cpu, int opp);
int mtk_cpu_capacity_count(unsigned int cpu);
#endif
