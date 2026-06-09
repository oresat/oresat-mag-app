#ifndef _IMU_H_
#define _IMU_H_

// x, y, and z are in units of: (for GYRO_FS_SEL = 7) 2097.2LSB/(º/s)
// temp is in centigrade
void get_gyro_data(int16_t *x, int16_t *y, int16_t *z, int16_t *temp);

#endif
