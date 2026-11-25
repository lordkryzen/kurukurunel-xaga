/* Read-only diagnostic extension. Same CPU/control lock order as enable/exit. */
int swpm_arm_pmu_read_count(unsigned int id, unsigned int cpu,
		u64 *count, u64 *enabled, u64 *running)
{
	struct swpm_cpu_events *events;
	struct perf_event *event = NULL;
	int ret = -ENODEV;

	if (!count || !enabled || !running || cpu >= nr_cpu_ids || id >= DSU_CYCLES_EVT)
		return -EINVAL;
	cpus_read_lock();
	mutex_lock(&swpm_pmu_lock);
	if (!swpm_pmu_ready || swpm_pmu_stopping || !cpu_online(cpu))
		goto out;
	events = rcu_access_pointer(per_cpu(swpm_events, cpu));
	if (!events)
		goto out;
	switch (id) {
	case L3DC_EVT: event = events->l3; break;
	case INST_SPEC_EVT: event = events->inst; break;
	case CYCLES_EVT: event = events->cycles; break;
	}
	if (event) {
		*count = perf_event_read_value(event, enabled, running);
		ret = 0;
	}
out:
	mutex_unlock(&swpm_pmu_lock);
	cpus_read_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(swpm_arm_pmu_read_count);
