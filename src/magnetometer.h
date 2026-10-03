#ifndef _MAGNETOMETER_H_
#define _MAGNETOMETER_H_

// Note:
// - EC stands for End Cap, a daughter board to Oresat end cards.
// - PZ stands for positive z-axis.
// - MZ stands for negative z-axis.

// The order below matches the order in the object dictionary.
typedef enum {
	EC_MAG_0_PZ_1 = 0,
	EC_MAG_1_PZ_2,
	EC_MAG_2_MZ_1,
	EC_MAG_3_MZ_2,
	EC_MAG_NONE,
} end_card_magnetometer_t;

int get_mag_reading(int mag_num, int32_t *x, int32_t *y, int32_t *z);

uint32_t num_mags_detected(void);

#endif // _MAGNETOMETER_H_
