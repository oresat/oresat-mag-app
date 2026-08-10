#ifndef _MAGNETOMETER_H_
#define _MAGNETOMETER_H_

//#define DEV_MAG_ZEPHYR_ENABLE_MAGB
#define NUM_MAGS 2

// TODO [ ] Determine whether the following enum for RM3100 sensor physical
//          positions is going to be not possible to support, in conjunction
//          with device tree "foreach" macros which do not guarantee the order
//          of their generated code fragments:

// The order below matches the order in the object dictionary.
typedef enum {
	EC_MAG_0_PZ_1 = 0,
	EC_MAG_1_PZ_2,
	EC_MAG_2_MZ_1,
	EC_MAG_3_MZ_2,
	EC_MAG_NONE,
} end_card_magnetometer_t;

int get_mag_reading(int mag_num, int32_t *x, int32_t *y, int32_t *z);

#endif

