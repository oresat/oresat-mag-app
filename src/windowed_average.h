#ifndef _WINDOWED_AVERAGE_H_
#define _WINDOWED_AVERAGE_H_


// TODO: use macro tricks to allow each user set the history size to whatever they need
// make so that IMU_ODR * this is equal or longer than magnetorquer update period
#define NUM_DATA_SAMPLE_PER_AVG 10

typedef struct {
	char *name;
	int32_t hist[NUM_DATA_SAMPLE_PER_AVG + 1];
	int32_t depth;
} wnd_avg_store;

/**
 * @brief Return windowed average to empty state.
 *
 * @param hist_sp   - pointer to wnd_avg_store
 */
void reset_windowed_average(wnd_avg_store *store);

/**
 * @brief Remove oldest history entry when full.
 *
 * Insert newest datum.
 *
 * Update and return average.
 * 
 * @param hist_sp   - pointer to wnd_avg_store
 * @param new_datum - new value to include
 * @return int32_t  - updated average
 */
int32_t update_windowed_average(wnd_avg_store *hist_sp, int32_t new_datum);

#endif
