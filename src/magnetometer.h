#ifndef _MAGNETOMETER_H_
#define _MAGNETOMETER_H_

//#define DEV_MAG_ZEPHYR_ENABLE_MAGB
#define NUM_MAGS 1

int get_mag_reading(int mag_num, int32_t *x, int32_t *y, int32_t *z);

#endif

