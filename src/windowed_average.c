#include <zephyr/kernel.h>
#include <string.h>
#include "windowed_average.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(windowed_average, CONFIG_LOG_DEFAULT_LEVEL);

// change to 1 to log the store
#define EXTRA_VERBOSE 0

int init_windowed_average(wnd_avg_store *store, int32_t *hist_buffer, int32_t hist_buffer_len, const char *name)
{
	if (!store || !hist_buffer || !hist_buffer_len) {
		return -EINVAL;
	}
	store->name = name;
	store->hist_buffer = hist_buffer;
	store->hist_buffer_len = hist_buffer_len;
	reset_windowed_average(store);

	return 0;
}

void reset_windowed_average(wnd_avg_store *store)
{
	store->depth = 0;
	memset(store->hist_buffer, 0, store->hist_buffer_len * sizeof(store->hist_buffer[0]));
}

int32_t update_windowed_average(wnd_avg_store *store, int32_t new_datum)
{
	int32_t i;

	// throw out oldest sample
	for (i = store->hist_buffer_len - 1; i >= 0; i--) {
		store->hist_buffer[i + 1] = store->hist_buffer[i];
	}

	store->hist_buffer[0] = new_datum;

	store->depth++;
	if (store->depth > store->hist_buffer_len) {
		store->depth = store->hist_buffer_len;
	}

	int32_t sum = 0;
#if EXTRA_VERBOSE
	static char line[1024]; // This is an expeditious value but not optimized in any way.
							// EXTRA_VERBOSE is only for initial debug and can be removed soon.
	size_t len = 0;
#endif

	// TODO: optimize by removing oldest from sum, and adding newest, then divide
	for (i = 0; i < store->depth; i++) {
		sum += store->hist_buffer[i];
#if EXTRA_VERBOSE
		snprintk(&line[len], sizeof(line) - len, "%d, ", store->hist_buffer[i]);
		len += strlen(&line[len]);
#endif
	}
#if EXTRA_VERBOSE
	LOG_INF("%s, new:%d, sum:%d, depth:%d; %s", store->name ? store->name : "<none>", new_datum, sum, store->depth, line);
#endif

	return (int32_t)(sum / store->depth);
}


