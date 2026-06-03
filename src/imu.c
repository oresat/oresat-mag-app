#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "../drivers/sensor/tdk/icm4268x/icm4268x_reg.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu, CONFIG_LOG_DEFAULT_LEVEL);

#define IMU_DEVICE_ADDR     0x68 // I2C 7 bit address
#define IMU_DEVICE_ADDR_ALT 0x69 // I2C 7 bit address

static const struct device *i2c;
static uint8_t imu_addr;

#define IMU_THREAD_STACK_SIZE 2048
#define IMU_THREAD_PRIORITY 0
extern const k_tid_t imu_id;

int init_imu(void)
{
	int ret;

	i2c = DEVICE_DT_GET(DT_NODELABEL(flexcomm0_lpi2c0)); //i2c-0));

	if (!device_is_ready(i2c)) {
		LOG_ERR("I2C bus is not ready");
			return -ENODEV;
	}

	struct i2c_msg msgs[1];
	uint8_t dst;

	/* Send the address to read from */
	msgs[0].buf = &dst;
	msgs[0].len = 0U;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	if (i2c_transfer(i2c, &msgs[0], 1, IMU_DEVICE_ADDR) == 0) {
		LOG_INF("Device detected on i2c bus at 0x%02x", IMU_DEVICE_ADDR);
		imu_addr = IMU_DEVICE_ADDR;
	} else {
		LOG_INF("Device NOT detected on i2c bus at 0x%02x", IMU_DEVICE_ADDR);
		if (i2c_transfer(i2c, &msgs[0], 1, IMU_DEVICE_ADDR_ALT) == 0) {
			LOG_INF("Device detected on i2c bus at 0x%02x", IMU_DEVICE_ADDR_ALT);
			imu_addr = IMU_DEVICE_ADDR_ALT;
		} else {
			LOG_INF("Device NOT detected on i2c bus at 0x%02x", IMU_DEVICE_ADDR_ALT);
			LOG_ERR("No device found at either address");
			return -ENODEV;
		}
	}

	uint8_t reg_addr = REG_WHO_AM_I;
	uint8_t buf[1] = {0};

	ret = i2c_write_read(i2c, imu_addr, &reg_addr, 1, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Error reading IMU WHOAMI register: %d", ret);
		return ret;
	}

	if (buf[0] == WHO_AM_I_ICM42688) {
		LOG_INF("IMU detected.");
		return 0;
	} else {
		LOG_ERR("Register 0x%02x returned 0x%02x, not 0x%02x", reg_addr, buf[0], WHO_AM_I_ICM42688);
		return -ENXIO;
	}
}

static void handle_imu(void *p1, void *p2, void *p3)
{
	int err;

	k_thread_name_set(imu_id, "imu_thread");

	LOG_INF("Starting IMU thread");

	err = init_imu();
	if (err < 0) {
		return;
	}

	for (;;) {
		LOG_INF("Processing imu...");
		k_sleep(K_MSEC(1000));
	}
}

K_THREAD_DEFINE(imu_id, IMU_THREAD_STACK_SIZE, handle_imu, NULL, NULL, NULL, IMU_THREAD_PRIORITY, 0, 0);

