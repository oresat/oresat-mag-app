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
#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>

#include <stdio.h>

#ifndef DT_HAS_COMPAT_STATUS_OKAY
#warning "- DEV 0315 - macro `DT_HAS_COMPAT_STATUS_OKAY` not defined"
#else
#warning "- DEV 0315 - about to use `DT_HAS_COMPAT_STATUS_OKAY` . . ."
#endif 

#if !DT_HAS_COMPAT_STATUS_OKAY(pni_rm3100)
#warning "- DEV 0315 - Node `rm3100` does not have status set to 'okay'"
#endif

#if !DT_COMPAT_GET_ANY_STATUS_OKAY(pni_rm3100)
#warning "- DEV 0315 - No pni,rm3100 compatible node found in the device tree"
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define N		(8)
#define M		(N/2)
#define SQ_SZ		(N)
#define CQ_SZ		(N)

#define NODE_ID		DT_COMPAT_GET_ANY_STATUS_OKAY(pni_rm3100)

// #define SAMPLE_PERIOD	DT_PROP(NODE_ID, sample_period)
// #define SAMPLE_SIZE	DT_PROP(NODE_ID, sample_size)

#define SAMPLE_PERIOD	1.0 / DT_PROP(NODE_ID, odr)
#define SAMPLE_SIZE	1

#define PROCESS_TIME	((M - 1) * SAMPLE_PERIOD)

// - DEV 0402 -
#define READINGS_BUFFER_SIZE 256

#define RM3100_DEMO_SLEEP_TIME_MS 1000

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

	LOG_INF("Found device \"%s\", getting sensor data", rm3100_dev->name);
	return rm3100_dev;
}

// RTIO_DEFINE_WITH_MEMPOOL(ez_io, SQ_SZ, CQ_SZ, N, SAMPLE_SIZE, 4);

SENSOR_DT_READ_IODEV(iodev, DT_COMPAT_GET_ANY_STATUS_OKAY(pni_rm3100),
		{SENSOR_CHAN_MAGN_X, 0},
		{SENSOR_CHAN_MAGN_Y, 0},
		{SENSOR_CHAN_MAGN_Z, 0},
		{SENSOR_CHAN_MAGN_XYZ, 0});

RTIO_DEFINE(ctx, 1, 1);

int main(void)
{
	// const struct device *const rm3100_dev = DEVICE_DT_GET(NODE_ID);
	const struct device *const rm3100_dev = DEVICE_DT_GET_ANY(pni_rm3100);
	// struct rtio_iodev *iodev = mag0->data;
	uint8_t buf[READINGS_BUFFER_SIZE] = {0};
	static uint32_t loop_count = 1;
	int32_t rc = 0;

	LOG_INF("Starting . . .");

	if (check_rm3100_sensor(rm3100_dev) == NULL) {
		LOG_ERR("Could not find the RM3100");
		return -ENODEV;
	}

	while (true) {
                rc = sensor_read(&iodev, &ctx, buf, 128);
                if (rc != 0) {
                        LOG_ERR("%s: sensor_read() failed: %d", rm3100_dev->name, rc);
                        break;
                }

                const struct sensor_decoder_api *decoder;

                rc = sensor_get_decoder(rm3100_dev, &decoder);
                if (rc != 0) {
                        LOG_ERR("%s: sensor_get_decode() failed: %d", rm3100_dev->name, rc);
                        break;
                }

                uint32_t mag_x_fit = 0;
                struct sensor_q31_data mag_x_data = {0};

                decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_MAGN_X, 0},
                                                &mag_x_fit, 1, &mag_x_data);

                uint32_t mag_y_fit = 0;
                struct sensor_q31_data mag_y_data = {0};

                decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_MAGN_Y, 0},
                                                &mag_y_fit, 1, &mag_y_data);

                uint32_t mag_z_fit = 0;
                struct sensor_q31_data mag_z_data = {0};

                decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_MAGN_Z, 0},
                                                &mag_z_fit, 1, &mag_z_data);

		// See zephyr/include/zephyr/drivers/sensor_data_types.h
		// for `.value`, `.temperature`, `.humidity` and similar as
		// they appear as members of the `readings` array:
		LOG_INF("RM3100 readings (iter %u):  mag_x %s%d.%d mag_y %s%d.%d mag_z %s%d.%d",
			loop_count,
			PRIq_arg(mag_x_data.readings[0].value, 6, mag_x_data.shift),
			PRIq_arg(mag_y_data.readings[0].value, 6, mag_y_data.shift),
			PRIq_arg(mag_z_data.readings[0].value, 6, mag_z_data.shift));

		k_msleep(RM3100_DEMO_SLEEP_TIME_MS);
		loop_count++;
	}

	// Zephyr requires int main() to return 0
	return 0;
}
