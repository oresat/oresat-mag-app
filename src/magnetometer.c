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
#include <zephyr/logging/log_ctrl.h>
#include <canopennode.h>
#include <CO_OD.h>

#include "magnetometer.h"

LOG_MODULE_REGISTER(magnetometer, CONFIG_SENSOR_LOG_LEVEL);

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

#define MAG_STARTUP_DELAY 750
#define RM3100_DEMO_SLEEP_TIME_MS 1000

/* === GPIO data === */
#define BP_NODE DT_NODELABEL(maggpios)

// Copied from zephyr/dsp/utils.h (you end up needing to pull in a bunch of DSP stuff, including a library,
// simply to access this macro, which is overkill.)
// Removed the "Z_" prefix t prevent conflicts in the future.
#define SHIFT_Q31_TO_F32(src, m) ((float32_t)(((int64_t)src) << m) / (float32_t)(1U << 31))

static const struct gpio_dt_spec n_mag_en = GPIO_DT_SPEC_GET(BP_NODE, n_mag_en_gpios);
static const struct gpio_dt_spec n_mag_fault = GPIO_DT_SPEC_GET(BP_NODE, n_mag_fault_gpios);
static const struct gpio_dt_spec mag_ready = GPIO_DT_SPEC_GET(BP_NODE, mag_ready_gpios);

static struct sensor_three_axis_data mag_data[NUM_MAGS];

#define MAG_THREAD_STACK_SIZE 4096
#define MAG_THREAD_PRIORITY 0
extern const k_tid_t mag_id;

static bool mag_0_good;
static bool mag_1_good;

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

#if 0
static void stop_end_cap_magnetometers(void) {
	//Disable power to the end cap magnetometers
	gpio_pin_set_dt(&n_mag_en, false);
}
#endif

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

		start_end_cap_magnetometers(); // enable the MAX892 mag power switch and breaker
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
		mag_0_good = false;
	} else if (check_rm3100_sensor(rm3100a_dev) == NULL) {
		LOG_ERR("Could not find RM3100 magnetometer instance 'a'");
		ret = -ENODEV;
		mag_0_good = false;
	} else {
		mag_0_good = true;
	}

#if (NUM_MAGS > 1)
	ret = device_init(rm3100b_dev);
	if (ret < 0) {
		LOG_ERR("Error initializing rm3100b device driver: %d", ret);
	} else if (check_rm3100_sensor(rm3100b_dev) == NULL) {
		LOG_ERR("Could not find RM3100 magnetometer instance 'b'");
		ret = -ENODEV;
		mag_1_good = false;
	} else {
		mag_1_good = true;
	}
	ret = 0; // run without it
#endif

	return ret;
}

int get_mag_reading(int mag_num, int32_t *x, int32_t *y, int32_t *z)
{
	if (mag_num >= NUM_MAGS) {
		return -EINVAL; // we don't support that one yet
	}
	/*
	The 32 bit value is shifted by the shift amount, but what that means in
	practical terms is hard to figure out. See:
	zephyr/drivers/sensor/pni/rm3100/rm3100_decoder.c line 116 (rm3100_convert_raw_to_q31)
	with the ODR value set in mcxn947_mag_card_mcxn947_cpu0.dtsi, which sets odr to 300 Hz.
	This means the decoder fn above uses shift = 11 and divider = 75 (uT per LSB).
	Further, the decoder scales the data (micro_tesla_scaled) then divides by 100
	to get gauss_scaled, which is the raw output value.
	To extract the integer portion in gauss, Zephyr samples such as 
	zephyr/sensors/sample/stream_fifo/src/main.c use a series of macros:
	PRIsensor_q31_data_arg() from zephyr/include/zephyr/drivers/sensor_data_types.h, which then
	uses macros from zephyr/include/zephyr/dsp/print_format.h: PRIq_arg() etc.

	What we want is to convert the reading to milligauss.
	*/

	*x = (int32_t)(SHIFT_Q31_TO_F32(mag_data[mag_num].readings[0].x, mag_data[mag_num].shift) * 1000.0f);
	*y = (int32_t)(SHIFT_Q31_TO_F32(mag_data[mag_num].readings[0].y, mag_data[mag_num].shift) * 1000.0f);
	*z = (int32_t)(SHIFT_Q31_TO_F32(mag_data[mag_num].readings[0].z, mag_data[mag_num].shift) * 1000.0f);

	return 0;
}

static void handle_mag(void *p1, void *p2, void *p3)
{
	static uint32_t loop_count = 1;
	int32_t rc = 0;

	k_thread_name_set(mag_id, "mag_thread");
	k_sleep(K_MSEC(MAG_STARTUP_DELAY));

	LOG_INF("Starting MAG thread");

	rc = init_mag();
	if (rc < 0) {
		return;
	}

	uint8_t buf[READINGS_BUFFER_SIZE] = {0};
#if (NUM_MAGS > 1)
	uint8_t buf_b[READINGS_BUFFER_SIZE] = {0};
#endif
	LOG_INF("Starting mag loop");

	while (true) {
		if (!gpio_pin_get_dt(&n_mag_fault)) {
			LOG_WRN("MAX892 mag power fault!");
		}

		if (mag_0_good) {
			LOG_DBG("Reading mag 0");
			rc = sensor_read(&iodev, &ctx, buf, 128);
			if (rc != 0) {
					LOG_ERR("%s: sensor_read() failed: %d", rm3100a_dev->name, rc);
					break;
			}

			const struct sensor_decoder_api *decoder;
			uint32_t mag_fit = 0;

			LOG_DBG("Getting mag 0 decoder");
			rc = sensor_get_decoder(rm3100a_dev, &decoder);
			if (rc != 0) {
					LOG_ERR("%s: sensor_get_decode() failed: %d", rm3100a_dev->name, rc);
					break;
			}

			LOG_DBG("Decoding mag 0 into plus Z mag 1");
			decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_MAGN_XYZ, 0},
											&mag_fit, 1, &mag_data[EC_MAG_0_PZ_1]);
		}

//------------------------------------------------------
// For magnetometer b:
//------------------------------------------------------

#if (NUM_MAGS > 1)
		if (mag_1_good) {
			LOG_DBG("Reading mag 1");
			rc = sensor_read(&iodev_b, &ctx_b, buf_b, 128);
			if (rc != 0) {
					LOG_ERR("%s: sensor_read() for mag1 failed, err %d", rm3100b_dev->name, rc);
					break;
			}

			const struct sensor_decoder_api *decoder_b;
			uint32_t mag_fit_b = 0;

			LOG_DBG("Getting mag 1 decoder");
			rc = sensor_get_decoder(rm3100b_dev, &decoder_b);
			if (rc != 0) {
					LOG_ERR("%s: sensor_get_decode() failed: %d", rm3100b_dev->name, rc);
					break;
			}

			LOG_DBG("Decoding mag 1 into plus Z mag 2");
			decoder_b->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_MAGN_XYZ, 0},
											&mag_fit_b, 1, &mag_data[EC_MAG_1_PZ_2]);
		}
#endif

		k_msleep(RM3100_DEMO_SLEEP_TIME_MS);
		loop_count++;
	}
}

K_THREAD_DEFINE(mag_id, MAG_THREAD_STACK_SIZE, handle_mag, NULL, NULL, NULL, MAG_THREAD_PRIORITY, 0, 0);

