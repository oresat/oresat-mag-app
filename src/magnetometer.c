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

#define MAG_THREAD_STACK_SIZE 4096
#define MAG_THREAD_PRIORITY 0
extern const k_tid_t mag_id;

K_MUTEX_DEFINE(mag_data_mtx);

#define N		(8)
#define M		(N/2)
#define SQ_SZ		(N)
#define CQ_SZ		(N)

#define SAMPLE_PERIOD	1.0 / DT_PROP(MAG0_NODE, odr)
#define SAMPLE_SIZE	1

#define PROCESS_TIME	((M - 1) * SAMPLE_PERIOD)

#define READINGS_BUFFER_SIZE 256

#define MAG_STARTUP_DELAY 750
#define RM3100_DEMO_SLEEP_TIME_MS 1000

#define MAG_GET_READING_TIMEOUT_MS 1000

/* === GPIO data === */
#define BP_NODE DT_NODELABEL(maggpios)

// Copied from zephyr/dsp/utils.h (you end up needing to pull in a bunch of DSP stuff, including a library,
// simply to access this macro, which is overkill.
// Removed the "Z_" prefix t prevent conflicts in the future.
#define SHIFT_Q31_TO_F32(src, m) ((float32_t)(((int64_t)src) << m) / (float32_t)(1U << 31))

static const struct gpio_dt_spec n_mag_en = GPIO_DT_SPEC_GET(BP_NODE, n_mag_en_gpios);
static const struct gpio_dt_spec n_mag_fault = GPIO_DT_SPEC_GET(BP_NODE, n_mag_fault_gpios);
static const struct gpio_dt_spec mag_ready = GPIO_DT_SPEC_GET(BP_NODE, mag_ready_gpios);

/**
 * @brief Zephyr DTS macros, and macros based on them, used to construct code
 *  for each magnetometer node with status equal to "okay".
 *
 * @note Normally used only in Zephyr drivers, here we define DT_DRV_COMPAT to
 *  match our magnetometer's DTS compatible property value.  (Our sensor is the
 *  RM3100.)  We leave this defined long enough to use a device tree "foreach"
 *  type of macro, to generate code constructs for each device tree node with
 *  this sensor enabled.
 */

#define DT_DRV_COMPAT pni_rm3100

// Create the two basic, static-qualified RTIO structs which each magnetometer
// needs, in order to interface with Zephyr's real time I/O sub-system.  Note
// that these structs are described in zephyr/include/zephyr/sensor.h.

#define MAG_CREATE_IODEV_AND_CONTEXT_STRUCT(inst) \
SENSOR_DT_READ_IODEV(iodev_##inst, DT_ALIAS(mag##inst),  \
                {SENSOR_CHAN_MAGN_X, 0},          \
                {SENSOR_CHAN_MAGN_Y, 0},          \
                {SENSOR_CHAN_MAGN_Z, 0},          \
                {SENSOR_CHAN_MAGN_XYZ, 0});       \
RTIO_DEFINE(ctx_##inst, 1, 1);

DT_INST_FOREACH_STATUS_OKAY(MAG_CREATE_IODEV_AND_CONTEXT_STRUCT)

// Create a decoder struct instance for each enabled magnetometer:

#define MAG_CREATE_DECODER(inst) \
const struct sensor_decoder_api decoder_##inst;

DT_INST_FOREACH_STATUS_OKAY(MAG_CREATE_DECODER)

// Sensor context struct, to organize run-time state and connections of a
// sensor to the RTIO sub-system and application code.
//
// See https://github.com/zephyrproject-rtos/zephyr/blob/1f6485eca25431b5ff27ce9a754218c9e559bbbb/include/zephyr/drivers/sensor_data_types.h#L42
// for Zephyr 4.4.1 three-axis data struct details.

struct rm3100_sensor_ctx {
	// Zephyr device handle, pointing to struct of basic device attributes:
	const struct device *const dev;
	// Run time sensor status (as opposed to initialization status):
	bool status_ok;
	// Sensor instance per Zephyr device tree source parsing:
	uint32_t dt_instance;
	// Sensor I2C address:
	uint32_t reg;
	// Param to map order of sensor discovery in device tree with object dictionary order:
	int32_t obj_dict_order;
	// Structs to connect sensor to Zephyr RTIO sub-system:
	const struct rtio_iodev *iodev;
	struct rtio *rtio_ctx;
	// Encoded magntometer readings obtained directly from sensor:
	uint8_t readings[READINGS_BUFFER_SIZE];
	// Decoded magnetometer readings:
	struct sensor_three_axis_data mag_data;
	// TODO [ ] Explain this parameter used in mag reading decoding:
	uint32_t mag_fit;
	// Following pre-refactor code each sensor gets its own decoder instance:
        const struct sensor_decoder_api *decoder;
};

// Macro to create array entry based on sensor context struct:
// Note: the pattern 'mag##inst' must match the form of magnetometer device
//  node aliases in this app's device tree sources.

#define MAG_ADD_SENSOR_TO_TABLE(inst)                 \
{                                                     \
	.dev = DEVICE_DT_GET(DT_ALIAS(mag##inst)),    \
	.status_ok = false,                           \
	.dt_instance = inst,                          \
	/* - 0811 - build time warning about "braces around scalar initializer": */ \
	.reg = DT_PROP(DT_ALIAS(mag##inst), reg),     \
	.obj_dict_order = 0,                          \
	.iodev = &iodev_##inst,                       \
	.rtio_ctx = &ctx_##inst,                      \
	.readings = { 0 },                            \
	.mag_data = {                                 \
		.header = { 0 },                      \
		.shift = 0,                           \
		.readings = {                         \
			{                             \
				.timestamp_delta = 0, \
				.values = { 0 },      \
			},                            \
		},                                    \
	},                                            \
	.mag_fit = 0,                                 \
	.decoder = &decoder_##inst,                   \
},

static struct rm3100_sensor_ctx rm3100_ctx[] = {
DT_INST_FOREACH_STATUS_OKAY(MAG_ADD_SENSOR_TO_TABLE)
};

#undef DT_DRV_COMPAT

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

void num_mags_detected(uint32_t *num_mags)
{
	*num_mags = ARRAY_SIZE(rm3100_ctx);
}

// TODO [ ] Ask whether this commented function is needed or can be removed:
#if 0
static void stop_end_cap_magnetometers(void) {
	//Disable power to the end cap magnetometers
	gpio_pin_set_dt(&n_mag_en, false);
}
#endif

/**
 * @note In the main branch at commit 4bf8bee004, device tree source aliases
 *  mag0 to the magnetometer with I2C device address 0x20, and aliases mag1 to
 *  the device node with I2C device address 0x22.  Further, that code associates
 *  the enum element EC_MAG_0_PZ_1 with mag0, and enum element EC_MAG_1_PZ_2
 *  with mag1.
 *
 *  In order to support calls to get_mag_reading(idx, *x, *y, *z), the
 *  following routine maps magnetometer axis enum values to the corresponding
 *  mag sensor context struct in this module.  The array of these context
 *  structs is populated by a Zephyr 'foreach' device tree macro, which doesn't
 *  guarantee the order of the device nodes it finds at compile time.  For this
 *  reason, a run time look-up function is needed.
 *
 * @param axis, the name position of the caller's magnetometer of interest
 * @param mag_idx, a variable to hold an array index to an enabled magnetometer
 *  sensor.
 *
 * @retval 0 on success.  Sensor array index returned in mag_idx.
 * @retval -ENODEV when magnetometer array holds no sensor contexts, meaning
 *  no sensors were detected at build time.
 */

int32_t mag_axis_to_mag_index(const end_card_magnetometer_t axis, uint32_t *mag_idx)
{
	uint32_t reg = 0;
	uint32_t idx = 0;
	int32_t rc = 0;

	if (ARRAY_SIZE(rm3100_ctx) < 1) {
		return -ENODEV;
	}

	switch(axis)
	{
	// Note these case statement axis-to-i2c-addr associations are taken
	// from mag app main branch, commit hash 4bf8bee004:
	case EC_MAG_0_PZ_1:
		reg = 0x20;
		break;
	case EC_MAG_1_PZ_2:
		reg = 0x22;
		break;
	default:
		// If axis does not match a defined case, that input is invalid.
		rc = -EINVAL;
	}

	if (rc != 0) {
		LOG_ERR("Failed to map mag axis to discovered sensor, err %d",
		        rc);
		goto done;
	}

	// Now we search for a sensor with known I2C device address (reg property) value.
	// Assign return code rc with current state "error no such device":
	rc = -ENODEV;
	for (idx = 0; idx < ARRAY_SIZE(rm3100_ctx); idx++) {
		if (rm3100_ctx[idx].reg == reg) {
			*mag_idx = idx;
			rc = 0;
			LOG_INF("matched mag axis %d with mag sensor array"
				"idx %u", axis, *mag_idx);
			break;
		}
	}

done:
	return rc;
}

// A development time routine, may be removed to prepare for production code:

static int32_t mag_sensor_summary(void)
{
	int32_t rc = 0;

	if (ARRAY_SIZE(rm3100_ctx) < 1) {
		return -ENODEV;
	}

	for (uint32_t i = 0; i < ARRAY_SIZE(rm3100_ctx); i++) {
		LOG_INF("rm3100 dt instance %d has I2C device address %02X",
			rm3100_ctx[i].dt_instance, rm3100_ctx[i].reg);
	}

	return rc;
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
		LOG_ERR("\nError: Device '%s' is not ready; "
			"check the driver initialization logs for errors.",
			rm3100_dev->name);
		return NULL;
	}

	LOG_INF("Found device '%s'", rm3100_dev->name);
	return rm3100_dev;
}

// TODO [ ] Determine whether it makes sense to enable the use of mempool for
//          this RM3100 magnetometer module:
// RTIO_DEFINE_WITH_MEMPOOL(ez_io, SQ_SZ, CQ_SZ, N, SAMPLE_SIZE, 4);

int init_mag(void)
{
	int ret;
	uint32_t count = 1;
	int32_t rc = 0;

	rc = mag_sensor_summary();
	if (rc < 0) {
		LOG_WRN("dev-only magnetometer summary report failed, err %d", rc);
	}

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
	uint32_t idx = 0;
	for (idx = 0; idx < ARRAY_SIZE(rm3100_ctx); idx++) {
		ret = device_init(rm3100_ctx[idx].dev);
		if (ret < 0) {
			LOG_ERR("Error initializing rm3100 device driver: %d", ret);
			rm3100_ctx[idx].status_ok = false;
		} else if (check_rm3100_sensor(rm3100_ctx[idx].dev) == NULL) {
			LOG_ERR("Could not find RM3100 magnetometer, dt instance %u",
				rm3100_ctx[idx].dt_instance);
			ret = -ENODEV;
			rm3100_ctx[idx].status_ok = false;
		} else {
			rm3100_ctx[idx].status_ok = true;
		}
	}

	return ret;
}

int get_mag_reading(int mag_num, int32_t *x, int32_t *y, int32_t *z)
{
	if (mag_num >= ARRAY_SIZE(rm3100_ctx)) {
		return -EINVAL; // we don't support that one yet
	}

	if (k_mutex_lock(&mag_data_mtx, K_MSEC(MAG_GET_READING_TIMEOUT_MS)) == 0) {
		/* mutex successfully locked */
	} else {
		LOG_ERR("Failed to lock mutex in get_mag_reading()");
		return -EAGAIN;
	}

	/*
	Convert magnetometer number to index to array of sensors element.
	Sensors are discovered by device tree macros, which don't guarantee any
	particular ordering of those sensors.
	*/

	uint32_t idx = 0;
	int32_t rc = 0;
	rc = mag_axis_to_mag_index(mag_num, &idx);
	if (rc != 0) {
		LOG_ERR("Failed to get index to sensor with mag axis %d, err %d",
			mag_num, rc);
		goto done;
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

	*x = (int32_t)(SHIFT_Q31_TO_F32(rm3100_ctx[idx].mag_data.readings[0].x,
					rm3100_ctx[idx].mag_data.shift) * 1000.0f);
	*y = (int32_t)(SHIFT_Q31_TO_F32(rm3100_ctx[idx].mag_data.readings[0].y,
					rm3100_ctx[idx].mag_data.shift) * 1000.0f);
	*z = (int32_t)(SHIFT_Q31_TO_F32(rm3100_ctx[idx].mag_data.readings[0].z,
					rm3100_ctx[idx].mag_data.shift) * 1000.0f);

done:
	k_mutex_unlock(&mag_data_mtx);

	return rc;
}

static void handle_mag(void *p1, void *p2, void *p3)
{
	static uint32_t loop_count = 1;
	uint32_t idx = 0;
	int32_t rc = 0;

	k_thread_name_set(mag_id, "mag_thread");
	k_sleep(K_MSEC(MAG_STARTUP_DELAY));

	LOG_INF("Starting MAG thread");

	rc = init_mag();
	if (rc < 0) {
		return;
	}

	LOG_INF("Starting mag loop");

	while (true) {
		if (!gpio_pin_get_dt(&n_mag_fault)) {
			LOG_WRN("MAX892 mag power fault!");
		}

		idx = 0;
		for (idx = 0; idx < ARRAY_SIZE(rm3100_ctx); idx++) {
			if (rm3100_ctx[idx].status_ok) {
				LOG_DBG("Reading mag '%s'", rm3100_ctx[idx].dev->name);
				rc = sensor_read(rm3100_ctx[idx].iodev, rm3100_ctx[idx].rtio_ctx,
						rm3100_ctx[idx].readings, 128);
				if (rc != 0) {
					LOG_ERR("%s: sensor_read() failed: %d",
						rm3100_ctx[idx].dev->name, rc);
					break;
				}

				LOG_DBG("Getting decoder for mag '%s'", rm3100_ctx[idx].dev->name);
				rc = sensor_get_decoder(rm3100_ctx[idx].dev,
						&rm3100_ctx[idx].decoder);
				if (rc != 0) {
					LOG_ERR("Failed sensor_get_decoder() for '%s', err %d",
						rm3100_ctx[idx].dev->name, rc);
					break;
				}

				LOG_DBG("Decoding mag '%s' into plus Z mag 1",
					rm3100_ctx[idx].dev->name);
				rm3100_ctx[idx].decoder->decode(rm3100_ctx[idx].readings,
						(struct sensor_chan_spec) {SENSOR_CHAN_MAGN_XYZ, 0},
						&rm3100_ctx[idx].mag_fit, 1,
						&rm3100_ctx[idx].mag_data);
			}
		}

		k_msleep(RM3100_DEMO_SLEEP_TIME_MS);
		loop_count++;
	}
}

K_THREAD_DEFINE(mag_id, MAG_THREAD_STACK_SIZE, handle_mag, NULL, NULL, NULL, MAG_THREAD_PRIORITY, 0, 0);
