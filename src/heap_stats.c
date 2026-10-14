#include <zephyr/kernel.h>

extern struct k_heap _system_heap;

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(heap_stats, LOG_LEVEL_INF);

void print_heap_stats(void)
{
	int err;
	struct sys_memory_stats stats;

	err = sys_heap_runtime_stats_get(&_system_heap.heap, &stats);
	if (err) {
		LOG_ERR("Failed to read kernel system heap statistics (err %d)", err);
		return;
	}

	LOG_INF("free:           %zu", stats.free_bytes);
	LOG_INF("allocated:      %zu", stats.allocated_bytes);
	LOG_INF("max. allocated: %zu", stats.max_allocated_bytes);
}

