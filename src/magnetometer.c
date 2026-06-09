/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 2026-03-14 Adapted for Portland State Oresat firmware work with RM3100
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>
#include <CO_OD.h>

#include "magnetometer.h"

LOG_MODULE_REGISTER(magnetometer, CONFIG_LOG_DEFAULT_LEVEL);

#define N		(8)
#define M		(N/2)
#define SQ_SZ		(N)
#define CQ_SZ		(N)

#define MAG0_NODE	DT_ALIAS(mag0)
#define MAG1_NODE	DT_ALIAS(mag1)

#define SAMPLE_PERIOD	1.0 / DT_PROP(MAG0_NODE, odr)
#define SAMPLE_SIZE	1

#define PROCESS_TIME	((M - 1) * SAMPLE_PERIOD)

// - DEV 0402 -
#define READINGS_BUFFER_SIZE 256

#define RM3100_DEMO_SLEEP_TIME_MS 1000

/* === GPIO data === */
#define BP_NODE DT_NODELABEL(maggpios)

static const struct gpio_dt_spec n_mag_en = GPIO_DT_SPEC_GET(BP_NODE, n_mag_en_gpios);
static const struct gpio_dt_spec n_mag_fault = GPIO_DT_SPEC_GET(BP_NODE, n_mag_fault_gpios);
static const struct gpio_dt_spec mag_ready = GPIO_DT_SPEC_GET(BP_NODE, mag_ready_gpios);

static struct sensor_three_axis_data mag_data[NUM_MAGS];

#define MAG_THREAD_STACK_SIZE 2048
#define MAG_THREAD_PRIORITY 0
extern const k_tid_t mag_id;

#if 0
// FROM CHIBIOS CODE:
typedef enum {
	EC_MAG_0_MZ_1 = 0,
	EC_MAG_1_MZ_2,
	EC_MAG_2_PZ_1,
	EC_MAG_3_PZ_2,
	EC_MAG_NONE,
} end_card_magnetometoer_t;


static const I2CConfig mmc5983ma_i2ccfg = {
    STM32_TIMINGR_PRESC(0xBU) |
    STM32_TIMINGR_SCLDEL(0x4U) | STM32_TIMINGR_SDADEL(0x2U) |
    STM32_TIMINGR_SCLH(0xFU)  | STM32_TIMINGR_SCLL(0x13U),
    0,
    0
};

static const MMC5983MAConfig mmc5983ma_generic_config = {
	.i2cp = &I2CD1,
	.i2ccfg = &mmc5983ma_i2ccfg
};


typedef struct {
	MMC5983MADriver driver;
	mmc5983ma_data_t data;
	volatile bool is_initialized;
	volatile bool is_working;
} magnetometer_data_struct_t;

typedef struct  {
	bmi088_accelerometer_sample_t accl_data;
	bmi088_gyro_sample_t gyro_sample;
	int16_t temp_c;

	mt_pwm_phase_data_t mt_pwm_data[3];

	magnetometer_data_struct_t magetometer_data[4];
} adcs_data_t;


adcs_data_t g_adcs_data;

static const BMI088Config imucfg = {
    .i2cp = &I2CD1,
    .i2ccfg = &i2ccfg,
    .gyro_saddr = BMI088_GYRO_SADDR,
    .acc_saddr = BMI088_ACC_SADDR,
};

static BMI088Driver imudev;

#endif

//----------------------------------------------------------------------
// - SECTION - routines
//----------------------------------------------------------------------

static int gpios_init(void)
{
    int ret;

    ret = gpio_pin_configure_dt(&n_mag_en, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
    if (ret) {
        return ret;
    }
    ret = gpio_pin_configure_dt(&n_mag_fault, GPIO_INPUT);
    if (ret) {
        return ret;
    }
    ret = gpio_pin_configure_dt(&mag_ready, GPIO_INPUT);
    if (ret) {
        return ret;
    }

    return ret;
}

static void stop_end_cap_magnetometers(void) {
	//Disable power to the end cap magnetometers
	gpio_pin_set_dt(&n_mag_en, false);
}

static void start_end_cap_magnetometers(void) {
	gpio_pin_set_dt(&n_mag_en, true);
	k_sleep(K_MSEC(10));
}

static const struct device *check_rm3100_sensor(const struct device *rm3100_dev)
{
	if (rm3100_dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		LOG_ERR("\nError: no device found.");
		return NULL;
	}

	if (!device_is_ready(rm3100_dev)) {
		LOG_ERR("\nError: Device \"%s\" is not ready; "
			"check the driver initialization logs for errors.",
			rm3100_dev->name);
		return NULL;
	}

	LOG_INF("Found device \"%s\"", rm3100_dev->name);
	return rm3100_dev;
}

// RTIO_DEFINE_WITH_MEMPOOL(ez_io, SQ_SZ, CQ_SZ, N, SAMPLE_SIZE, 4);

SENSOR_DT_READ_IODEV(iodev, DT_COMPAT_GET_ANY_STATUS_OKAY(pni_rm3100),
		{SENSOR_CHAN_MAGN_X, 0},
		{SENSOR_CHAN_MAGN_Y, 0},
		{SENSOR_CHAN_MAGN_Z, 0},
		{SENSOR_CHAN_MAGN_XYZ, 0});

RTIO_DEFINE(ctx, 1, 1);

SENSOR_DT_READ_IODEV(iodev_b, DT_ALIAS(mag1),
		{SENSOR_CHAN_MAGN_X, 0},
		{SENSOR_CHAN_MAGN_Y, 0},
		{SENSOR_CHAN_MAGN_Z, 0},
		{SENSOR_CHAN_MAGN_XYZ, 0});

RTIO_DEFINE(ctx_b, 1, 1);

const struct device *const rm3100a_dev = DEVICE_DT_GET(MAG0_NODE);
const struct device *const rm3100b_dev = DEVICE_DT_GET(MAG1_NODE);

int init_mag(void)
{
	int ret;
	uint32_t count = 1;

	k_msleep(500);
	LOG_INF("Initializing magnetometers");
	ret = gpios_init();
	if (ret < 0) {
		LOG_ERR("Unable to initialize magnetometer gpio pins: %d", ret);
		return -ENODEV;
	}

	for (;;) {
		k_msleep(500);
		LOG_INF("Turning on mag power");
		k_msleep(10);
		gpio_pin_set_dt(&n_mag_en, 1); // enable the MAX892 mag power switch and breaker
		k_msleep(100);
		if (!gpio_pin_get_dt(&n_mag_fault)) {
			LOG_WRN("Enabled MAX892 mag power, but got a fault. Retry %u", count++);
		} else {
			break;
		}
	}

	// We use deferred initialization in the device tree so we can wait until
	// we power up the mags before trying to talk to them. The init function for
	// this sensor tries to read the revision ID register, which it obviously
	// cannot do if the device is powered off.
	ret = device_init(rm3100a_dev);
	if (ret < 0) {
		LOG_ERR("Error initializing rm3100a device driver: %d", ret);
		return ret;
	}
	if (check_rm3100_sensor(rm3100a_dev) == NULL) {
		LOG_ERR("Could not find RM3100 magnetometer instance 'a'");
		ret = -ENODEV;
	}

#if (NUM_MAGS > 1)
	ret = device_init(rm3100b_dev);
	if (ret < 0) {
		LOG_ERR("Error initializing rm3100b device driver: %d", ret);
		return ret;
	}
	if (check_rm3100_sensor(rm3100b_dev) == NULL) {
		LOG_ERR("Could not find RM3100 magnetometer instance 'b'");
		ret = -ENODEV;
	}
#endif

	return ret;
}

int get_mag_reading(int mag_num, int32_t *x, int32_t *y, int32_t *z)
{
	if (mag_num >= NUM_MAGS) {
		return -EINVAL; // we don't support that one yet
	}

	*x = mag_data[mag_num].readings[0].x;
	*y = mag_data[mag_num].readings[0].y;
	*z = mag_data[mag_num].readings[0].z;

	return 0;
}

static void handle_mag(void *p1, void *p2, void *p3)
{
	int err;

	k_thread_name_set(mag_id, "mag_thread");

	LOG_INF("Starting MAG thread");

	err = init_mag();
	if (err < 0) {
		return;
	}

	uint8_t buf[READINGS_BUFFER_SIZE] = {0};
#if (NUM_MAGS > 1)
	uint8_t buf_b[READINGS_BUFFER_SIZE] = {0};
#endif

	static uint32_t loop_count = 1;
	int32_t rc = 0;

	while (true) {
		if (!gpio_pin_get_dt(&n_mag_fault)) {
			LOG_WRN("MAX892 mag power fault!");
		}

		rc = sensor_read(&iodev, &ctx, buf, 128);
		if (rc != 0) {
				LOG_ERR("%s: sensor_read() failed: %d", rm3100a_dev->name, rc);
				break;
		}

		const struct sensor_decoder_api *decoder;

		rc = sensor_get_decoder(rm3100a_dev, &decoder);
		if (rc != 0) {
				LOG_ERR("%s: sensor_get_decode() failed: %d", rm3100a_dev->name, rc);
				break;
		}

		uint32_t mag_fit = 0;

		decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_MAGN_XYZ, 0},
										&mag_fit, 1, &mag_data[0]);

		LOG_INF(PRIsensor_three_axis_data,
			PRIsensor_three_axis_data_arg(mag_data[0], 0));

//------------------------------------------------------
// For magnetometer b:
//------------------------------------------------------

#if (NUM_MAGS > 1)
		rc = sensor_read(&iodev_b, &ctx_b, buf_b, 128);
		if (rc != 0) {
				LOG_ERR("%s: sensor_read() for mag1 failed, err %d", rm3100b_dev->name, rc);
				break;
		}

		const struct sensor_decoder_api *decoder_b;

		rc = sensor_get_decoder(rm3100b_dev, &decoder_b);
		if (rc != 0) {
				LOG_ERR("%s: sensor_get_decode() failed: %d", rm3100b_dev->name, rc);
				break;
		}

		uint32_t mag1_fit = 0;

		decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_MAGN_XYZ, 0},
										&mag_fit, 1, &mag_data[1]);

		LOG_INF(PRIsensor_three_axis_data,
			PRIsensor_three_axis_data_arg(mag_data[1], 0));
#endif

		k_msleep(RM3100_DEMO_SLEEP_TIME_MS);
		loop_count++;
	}
}

K_THREAD_DEFINE(mag_id, MAG_THREAD_STACK_SIZE, handle_mag, NULL, NULL, NULL, MAG_THREAD_PRIORITY, 0, 0);

