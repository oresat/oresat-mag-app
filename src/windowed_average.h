#ifndef _WINDOWED_AVERAGE_H_
#define _WINDOWED_AVERAGE_H_

typedef struct {
	const char *name;
	int32_t *hist_buffer;
	int32_t hist_buffer_len;
	int32_t depth;
} wnd_avg_store;

/**
 * @brief Set up windowed average structure.
 *
 * @param hist_sp   - pointer to wnd_avg_store
 * @param hist_buffer - caller's array in which the history is
 * stored.
 * @param hist_buffer_len - number of int32_t entries.
 * @param name - optional string to print in logging
 * @return int - negative error code or 0 on no error.
 */
int init_windowed_average(wnd_avg_store *store, int32_t *hist_buffer, int32_t hist_buffer_len, const char *name);

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
