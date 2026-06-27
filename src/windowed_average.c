#include <zephyr/kernel.h>
#include <string.h>
#include "windowed_average.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(windowed_average, CONFIG_LOG_DEFAULT_LEVEL);

// change to 1 to log the store
#define EXTRA_VERBOSE 0

void reset_windowed_average(wnd_avg_store *store)
{
	store->depth = 0;
	memset(store->hist, 0, sizeof(store->hist));
}

int32_t update_windowed_average(wnd_avg_store *wasp, int32_t new_datum)
{
	int32_t i;

	// throw out oldest sample
	for (i = NUM_DATA_SAMPLE_PER_AVG - 1; i >= 0; i--) {
		wasp->hist[i + 1] = wasp->hist[i];
	}

	wasp->hist[0] = new_datum;

	wasp->depth++;
	if (wasp->depth > NUM_DATA_SAMPLE_PER_AVG) {
		wasp->depth = NUM_DATA_SAMPLE_PER_AVG;
	}

	int32_t sum = 0;
#if EXTRA_VERBOSE
	static char line[1024];
	size_t len = 0;
#endif

	// TODO: optimize by removing oldest from sum, and adding newest, then divide
	for (i = 0; i < wasp->depth; i++) {
		sum += wasp->hist[i];
#if EXTRA_VERBOSE
		snprintk(&line[len], sizeof(line) - len, "%d, ", wasp->hist[i]);
		len += strlen(&line[len]);
#endif
	}
#if EXTRA_VERBOSE
	LOG_INF("%s, new:%d, sum:%d, depth:%d; %s", wasp->name ? wasp->name : "", new_datum, sum, wasp->depth, line);
#endif

	return (int32_t)(sum / wasp->depth);
}


