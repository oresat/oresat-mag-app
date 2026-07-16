#ifndef _MAGNETOMETER_H_
#define _MAGNETOMETER_H_

//#define DEV_MAG_ZEPHYR_ENABLE_MAGB
#define NUM_MAGS 2

int get_mag_reading(int mag_num, int16_t *x, int16_t *y, int16_t *z);

#endif

