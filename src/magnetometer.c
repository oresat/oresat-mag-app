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

// #define MAG0_NODE	DT_ALIAS(mag0)
// #define MAG1_NODE	DT_ALIAS(mag1)

#define SAMPLE_PERIOD	1.0 / DT_PROP(MAG0_NODE, odr)
#define SAMPLE_SIZE	1

#define PROCESS_TIME	((M - 1) * SAMPLE_PERIOD)

#define READINGS_BUFFER_SIZE 256

#define MAG_STARTUP_DELAY 750
#define RM3100_DEMO_SLEEP_TIME_MS 1000

/* === GPIO data === */
#define BP_NODE DT_NODELABEL(maggpios)

// Copied from zephyr/dsp/utils.h (you end up needing to pull in a bunch of DSP stuff, including a library,
// simply to access this macro, which is overkill.
// Removed the "Z_" prefix t prevent conflicts in the future.
#define SHIFT_Q31_TO_F32(src, m) ((float32_t)(((int64_t)src) << m) / (float32_t)(1U << 31))

static const struct gpio_dt_spec n_mag_en = GPIO_DT_SPEC_GET(BP_NODE, n_mag_en_gpios);
static const struct gpio_dt_spec n_mag_fault = GPIO_DT_SPEC_GET(BP_NODE, n_mag_fault_gpios);
static const struct gpio_dt_spec mag_ready = GPIO_DT_SPEC_GET(BP_NODE, mag_ready_gpios);

static struct sensor_three_axis_data mag_data[NUM_MAGS];

#define MAG_THREAD_STACK_SIZE 4096
#define MAG_THREAD_PRIORITY 0
extern const k_tid_t mag_id;

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



// - DEV 0808 BEGION -

void rm3100_test_function(char* sensor_instance)
{
	LOG_WRN("- DEV 0808 - called for RM3100 node instance %s", sensor_instance);
}

#define RM3100_TEST_FUNCTION(inst) \
rm3100_test_function(STRINGIFY(inst));

// Normally used only in Zephyr drivers, we define DT_DRV_COMPAT for our
// magnetometer sensor model, long enough to use a device tree "foreach" type
// of macro, to generate repeated sensor code constructs for each magnetometer
// expressed in app device tree sources:

#define DT_DRV_COMPAT pni_rm3100

void rm3100_roll_call(void) {
DT_INST_FOREACH_STATUS_OKAY(RM3100_TEST_FUNCTION)
}

#define MAG_CREATE_IODEV_AND_CONTEXT_STRUCT(inst) \
SENSOR_DT_READ_IODEV(iodev_##inst, DT_ALIAS(mag##inst),  \
                {SENSOR_CHAN_MAGN_X, 0},          \
                {SENSOR_CHAN_MAGN_Y, 0},          \
                {SENSOR_CHAN_MAGN_Z, 0},          \
                {SENSOR_CHAN_MAGN_XYZ, 0});       \
RTIO_DEFINE(ctx_##inst, 1, 1);

DT_INST_FOREACH_STATUS_OKAY(MAG_CREATE_IODEV_AND_CONTEXT_STRUCT)

#define MAG_CREATE_DECODER(inst) \
const struct sensor_decoder_api decoder_##inst;

DT_INST_FOREACH_STATUS_OKAY(MAG_CREATE_DECODER)

struct rm3100_sensor_ctx {
	const struct device *const dev;
	bool status_ok;
	uint32_t dt_instance;
	int32_t obj_dict_order;
	// See zephyr/include/zephyr/sensor.h:1152, for the types of iodev and
	// rtio_ctx:
	const struct rtio_iodev *iodev;
	struct rtio *rtio_ctx;
	uint8_t readings[READINGS_BUFFER_SIZE];
	// TODO [ ] Tie in this 'mag_data' with 'mag_data' array references in get_mag_reading():
	struct sensor_three_axis_data mag_data;
	uint32_t mag_fit;
        const struct sensor_decoder_api *decoder;
};

// TEMPORARY NOTE:
/* const struct device *const rm3100a_dev = DEVICE_DT_GET(MAG0_NODE); */
// .dev = DEVICE_DT_GET(DT_ALIAS(mag##inst)),

#define MAG_ADD_SENSOR_TO_TABLE(inst)              \
{                                                  \
	.dev = DEVICE_DT_GET(DT_ALIAS(mag##inst)), \
	.status_ok = false,                        \
	.dt_instance = inst,                       \
	.obj_dict_order = 0,                       \
	.iodev = &iodev_##inst,                    \
	.rtio_ctx = &ctx_##inst,                   \
	.readings = {0},                           \
	/* TODO [ ] initialize struct sensor_three_axis_data mag_data */ \
	.mag_fit = 0,                              \
	.decoder = &decoder_##inst,                \
},

static struct rm3100_sensor_ctx rm3100_ctx[] = {
DT_INST_FOREACH_STATUS_OKAY(MAG_ADD_SENSOR_TO_TABLE)
};

#undef DT_DRV_COMPAT

// - DEV 0808 DEV -



int init_mag(void)
{
	int ret;
	uint32_t count = 1;

	LOG_INF("- DEV 0808 - Check of device tree for RM3100 nodes . . .");
	// TODO [ ] Remove roll call test:
	rm3100_roll_call();

	LOG_INF("- DEV 0808 - From device tree built sensor array of %d elements",
		ARRAY_SIZE(rm3100_ctx));

	LOG_INF("- DEV 0808 - Check done.");



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
			// mag_0_good = false;
			rm3100_ctx[idx].status_ok = false;
		} else if (check_rm3100_sensor(rm3100_ctx[idx].dev) == NULL) {
			LOG_ERR("Could not find RM3100 magnetometer, dt instance %u",
				rm3100_ctx[idx].dt_instance);
			ret = -ENODEV;
			// mag_0_good = false;
			rm3100_ctx[idx].status_ok = false;
		} else {
			// mag_0_good = true;
			rm3100_ctx[idx].status_ok = true;
		}
	}

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
	uint32_t idx = 0;
	int32_t rc = 0;

	k_thread_name_set(mag_id, "mag_thread");
	k_sleep(K_MSEC(MAG_STARTUP_DELAY));

	LOG_INF("Starting MAG thread");

	rc = init_mag();
	if (rc < 0) {
		return;
	}

#if 0
	uint8_t buf[READINGS_BUFFER_SIZE] = {0};
#if (NUM_MAGS > 1)
	uint8_t buf_b[READINGS_BUFFER_SIZE] = {0};
#endif
#endif // 0
	LOG_INF("Starting mag loop");

	while (true) {
		if (!gpio_pin_get_dt(&n_mag_fault)) {
			LOG_WRN("MAX892 mag power fault!");
		}

		idx = 0;
		for (idx = 0; idx < ARRAY_SIZE(rm3100_ctx); idx++) {
			if (rm3100_ctx[idx].status_ok) {
				LOG_DBG("Reading mag '%s'", rm3100_ctx[idx].dev->name);
// TODO [ ] Determine, may we move 'iodev' into rm3100_ctx array of sensor context structs?
//   ANSWER: maybe not directly as iodev struct is static qualified, but maybe by pointer.
				// rc = sensor_read(&iodev, &ctx, buf, 128);
				rc = sensor_read(rm3100_ctx[idx].iodev, rm3100_ctx[idx].rtio_ctx,
						rm3100_ctx[idx].readings, 128);
				if (rc != 0) {
					LOG_ERR("%s: sensor_read() failed: %d",
						rm3100_ctx[idx].dev->name, rc);
					break;
				}

// TODO [ ] Refactor decoder struct and mag_fit variable into rm3100 context:
				// const struct sensor_decoder_api *decoder;
				// uint32_t mag_fit = 0;

				LOG_DBG("Getting mag 0 decoder");
				// rc = sensor_get_decoder(rm3100_ctx[idx].dev, &decoder);
				rc = sensor_get_decoder(rm3100_ctx[idx].dev,
						&rm3100_ctx[idx].decoder);
				if (rc != 0) {
					LOG_ERR("Failed sensor_get_decoder() for '%s', err %d",
						rm3100_ctx[idx].dev->name, rc);
					break;
				}

				LOG_DBG("Decoding mag '%s' into plus Z mag 1",
					rm3100_ctx[idx].dev->name);
				// decoder->decode(buf, (struct sensor_chan_spec) {SENSOR_CHAN_MAGN_XYZ, 0},
				//		&mag_fit, 1, &mag_data[EC_MAG_0_PZ_1]);
				rm3100_ctx[idx].decoder->decode(rm3100_ctx[idx].readings,
						(struct sensor_chan_spec) {SENSOR_CHAN_MAGN_XYZ, 0},
						&rm3100_ctx[idx].mag_fit, 1,
						&rm3100_ctx[idx].mag_data);
			}
		}

//------------------------------------------------------
// For magnetometer b:
//------------------------------------------------------

// #if (NUM_MAGS > 1)
#if 0
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
