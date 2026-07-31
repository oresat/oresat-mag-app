#ifndef _IMU_H_
#define _IMU_H_

bool is_imu_ready(void);

// x, y, and z are in units of milli-degrees per second.
// temp is in centigrade
void get_gyro_data(int16_t *x, int16_t *y, int16_t *z, int16_t *temp);

// x, y, and z are in units of milli-Gs
void get_accel_data(int16_t *x, int16_t *y, int16_t *z);

#endif
