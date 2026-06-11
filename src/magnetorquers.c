#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>
#include <canopennode.h>
#include <CO_OD.h>
#include <version.h>
#include <app_version.h>

#include "dac.h"
#include "pwm.h"
#include "adc.h"
#include "imu.h"
#include "magnetometer.h"

LOG_MODULE_REGISTER(magnetorquers, CONFIG_LOG_DEFAULT_LEVEL);

/* size of stack area used by each thread */
#define STACK_SIZE 4096

/* scheduling priority used by each thread */
#define PRIORITY 7

#define ITERATION_PERIOD 5 // ms

extern const k_tid_t magtqr_id;

typedef struct {
	int32_t current_pwm_percent; //0-10000
	int32_t target_pwm_percent; //Negative values indicate the phase should be inverted

	float current_feedback_measurement_V; //Volts, Note: this is the average voltage while the PWM output is high.
	int32_t current_feedback_measurement_uA; //uA. Note: this is the average current flowing while the PWM output is high. It does not represent overall average current.

	bool phase_state;
	uint8_t phase_gpio_pin_number;
	uint8_t pwm_channel_number;

	int64_t last_update_time;
} mt_pwm_phase_data_t;

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;
} three_axis_data;

typedef struct  {
	mt_pwm_phase_data_t mt_pwm_data[3];
	three_axis_data accl_data;
	three_axis_data gyro_data;
	int16_t temp_data;
	three_axis_data magnetometer_data[4];
} adcs_data_t;

static adcs_data_t g_adcs_data;

typedef enum {
	EC_MAG_0_MZ_1 = 0,
	EC_MAG_1_MZ_2,
	EC_MAG_2_PZ_1,
	EC_MAG_3_PZ_2,
	EC_MAG_NONE,
} end_card_magnetometer_t;

/* === GPIO data === */
#define BP_NODE DT_NODELABEL(maggpios)

static const struct gpio_dt_spec mt_en = GPIO_DT_SPEC_GET(BP_NODE, mt_en_gpios);
static const struct gpio_dt_spec n_mt_en_fault = GPIO_DT_SPEC_GET(BP_NODE, n_mt_en_fault_gpios);
static const struct gpio_dt_spec n_mt_stby_rst = GPIO_DT_SPEC_GET(BP_NODE, n_mt_stby_rst_gpios);
static const struct gpio_dt_spec mt_x_phase = GPIO_DT_SPEC_GET(BP_NODE, mt_x_phase_gpios);
static const struct gpio_dt_spec mt_y_phase = GPIO_DT_SPEC_GET(BP_NODE, mt_y_phase_gpios);
static const struct gpio_dt_spec mt_z_phase = GPIO_DT_SPEC_GET(BP_NODE, mt_z_phase_gpios);

/**************************************************/

static int init_gpios(void)
{
	int ret;

	// TODO: figure this out
	// GPIO_LINE_OPEN_DRAIN gives an assertion:
	// ASSERTION FAIL [(flags & (1 << 1)) != 0 || (flags & (1 << 2)) == 0] @ WEST_TOPDIR/zephyr/include/zephyr/drivers/gpio.h:1002

	// this should be GPIO_OPEN_DRAIN, but the MCXN947 gpio driver does not support it
	// instead, set to INPUT to float, or OUTPUT_INACTIVE to drive low
	ret = gpio_pin_configure_dt(&mt_en, GPIO_INPUT);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&n_mt_en_fault, GPIO_INPUT);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&n_mt_stby_rst, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&mt_x_phase, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&mt_y_phase, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}
	ret = gpio_pin_configure_dt(&mt_z_phase, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}

	return ret;
}

/**************************************************/

static int32_t saturate_int32_t(const int32_t v, const int32_t min, const int32_t max) {
	if (v >= max)
		return (max);

	else if (v <= min)
		return (min);

	return (v);
}


/**
 * return value is in the range of 0 to 10000
 */
static int32_t map_current_uA_to_pwm_duty_cycle(const int32_t current_uA, const uint8_t axis) {
	int32_t ret = 0;

	if (axis <= 1) {
		//X and Y axes
		//1700 => 1000000 uA
		//500 => 295000 uA (this is hard/impossible to measure using the ADC
		ret = current_uA / (988000.0 / 1700.0);
		const int32_t pwm_duty_max_value = 1700;
		ret = saturate_int32_t(ret, -pwm_duty_max_value, pwm_duty_max_value);
	} else {
		//Z axis
		//1700 => 344000 uA
		//500 => 102000 uA  (this is hard/impossible to measure using the ADC
		ret = current_uA / (344000.0 / 1700.0);
		const int32_t pwm_duty_max_value = 4940;
		ret = saturate_int32_t(ret, -pwm_duty_max_value, pwm_duty_max_value);
	}

	return(ret);
}

static int16_t raw_to_milligauss(int16_t raw)
{
		float gauss = 1000.0f * ((float) raw) / 4096.0f;
		return (int16_t)gauss;
}

// to simulate the C3 ADCS code setting the setpoints for node id 0x10:
//
// write current_x_setpoint to 291:
// $ cansend can0 610#23.07.40.04.23.01.00.00
// write current_y_setpoint to 4660:
// $ cansend can0 610#23.07.40.05.34.12.00.00
// write current_z_setpoint to 69:
// $ cansend can0 610#23.07.40.06.45.00.00.00
//
// To read an object, send this:
//  			   ID RD COBID --- SUB ---
// $ cansend can0 610#40.00.40.01.00.00.00
// to read fw_version:
// $ cansend can0 610#40.02.30.03.00.00.00

static void handle_can_open_data(void)
{
	size_t ver_size = sizeof(CO_OD_RAM.versions.fw_version);

	strncpy(CO_OD_RAM.versions.fw_version, &APP_VERSION_STRING[6], ver_size);

	g_adcs_data.mt_pwm_data[0].target_pwm_percent = map_current_uA_to_pwm_duty_cycle(CO_OD_RAM.magnetorquer.current_x_setpoint * 100, 0);
	g_adcs_data.mt_pwm_data[1].target_pwm_percent = map_current_uA_to_pwm_duty_cycle(CO_OD_RAM.magnetorquer.current_y_setpoint * 100, 1);
	g_adcs_data.mt_pwm_data[2].target_pwm_percent = map_current_uA_to_pwm_duty_cycle(CO_OD_RAM.magnetorquer.current_z_setpoint * 100, 2);

	CO_OD_RAM.gyroscope.pitch_rate = g_adcs_data.gyro_data.x;
	CO_OD_RAM.gyroscope.yaw_rate = g_adcs_data.gyro_data.y;
	CO_OD_RAM.gyroscope.roll_rate = g_adcs_data.gyro_data.z;
	CO_OD_RAM.gyroscope.pitch_rate_raw = g_adcs_data.gyro_data.x_raw;
	CO_OD_RAM.gyroscope.yaw_rate_raw = g_adcs_data.gyro_data.y_raw;
	CO_OD_RAM.gyroscope.roll_rate_raw = g_adcs_data.gyro_data.z_raw;

	CO_OD_RAM.accelerometer.x = g_adcs_data.accl_data.x;
	CO_OD_RAM.accelerometer.y = g_adcs_data.accl_data.y;
	CO_OD_RAM.accelerometer.z = g_adcs_data.accl_data.z;
	CO_OD_RAM.accelerometer.X_raw = g_adcs_data.accl_data.x_raw;
	CO_OD_RAM.accelerometer.Y_raw = g_adcs_data.accl_data.y_raw;
	CO_OD_RAM.accelerometer.Z_raw = g_adcs_data.accl_data.z_raw;

	CO_OD_RAM.temperature = g_adcs_data.temp_data;

	CO_OD_RAM.magnetorquer.current_x = g_adcs_data.mt_pwm_data[0].current_feedback_measurement_uA;
	CO_OD_RAM.magnetorquer.current_y = g_adcs_data.mt_pwm_data[1].current_feedback_measurement_uA;
	CO_OD_RAM.magnetorquer.current_z = g_adcs_data.mt_pwm_data[2].current_feedback_measurement_uA;

	CO_OD_RAM.magnetorquer.pwm_x = g_adcs_data.mt_pwm_data[0].current_pwm_percent;
	CO_OD_RAM.magnetorquer.pwm_y = g_adcs_data.mt_pwm_data[1].current_pwm_percent;
	CO_OD_RAM.magnetorquer.pwm_z = g_adcs_data.mt_pwm_data[2].current_pwm_percent;

	if (1) { //g_adcs_data.magetometer_data[EC_MAG_2_PZ_1].is_working) {
		CO_OD_RAM.pos_z_magnetometer_1.x = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_2_PZ_1].x);
		CO_OD_RAM.pos_z_magnetometer_1.y = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_2_PZ_1].y);
		CO_OD_RAM.pos_z_magnetometer_1.z = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_2_PZ_1].z);
	} else {
		CO_OD_RAM.pos_z_magnetometer_1.x = INT16_MAX;
		CO_OD_RAM.pos_z_magnetometer_1.y = INT16_MAX;
		CO_OD_RAM.pos_z_magnetometer_1.z = INT16_MAX;
	}

	if (1) { //g_adcs_data.magetometer_data[EC_MAG_3_PZ_2].is_working) {
		CO_OD_RAM.pos_z_magnetometer_2.x = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_3_PZ_2].x);
		CO_OD_RAM.pos_z_magnetometer_2.y = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_3_PZ_2].y);
		CO_OD_RAM.pos_z_magnetometer_2.z = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_3_PZ_2].z);
	} else {
		CO_OD_RAM.pos_z_magnetometer_2.x = INT16_MAX;
		CO_OD_RAM.pos_z_magnetometer_2.y = INT16_MAX;
		CO_OD_RAM.pos_z_magnetometer_2.z = INT16_MAX;
	}

	if (1) { //g_adcs_data.magetometer_data[EC_MAG_0_MZ_1].is_working) {
		CO_OD_RAM.min_z_magnetometer_1.x = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_0_MZ_1].x);
		CO_OD_RAM.min_z_magnetometer_1.y = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_0_MZ_1].y);
		CO_OD_RAM.min_z_magnetometer_1.z = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_0_MZ_1].z);
	} else {
		CO_OD_RAM.min_z_magnetometer_1.x = INT16_MAX;
		CO_OD_RAM.min_z_magnetometer_1.y = INT16_MAX;
		CO_OD_RAM.min_z_magnetometer_1.z = INT16_MAX;
	}

	if (1) { //g_adcs_data.magetometer_data[EC_MAG_1_MZ_2].is_working) {
		CO_OD_RAM.min_z_magnetometer_2.x = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_1_MZ_2].x);
		CO_OD_RAM.min_z_magnetometer_2.y = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_1_MZ_2].y);
		CO_OD_RAM.min_z_magnetometer_2.z = raw_to_milligauss(g_adcs_data.magnetometer_data[EC_MAG_1_MZ_2].z);
	} else {
		CO_OD_RAM.min_z_magnetometer_2.x = INT16_MAX;
		CO_OD_RAM.min_z_magnetometer_2.y = INT16_MAX;
		CO_OD_RAM.min_z_magnetometer_2.z = INT16_MAX;
	}
}

void print_debug_output(void) {
	static int64_t last_print_time = 0;
	int64_t now = k_uptime_get();

	if ((now - last_print_time) > 1500) { //750) {
		last_print_time = now;

		LOG_DBG( "================");
		LOG_DBG( "CANOpen Data:");
		LOG_DBG( "  CO_OD_RAM.versions.fw_version = %.5s", CO_OD_RAM.versions.fw_version);
		LOG_DBG( "  CO_OD_RAM.gyroscope.pitch_rate = %d", CO_OD_RAM.gyroscope.pitch_rate);
		LOG_DBG( "  CO_OD_RAM.gyroscope.yaw_rate = %d", CO_OD_RAM.gyroscope.yaw_rate);
		LOG_DBG( "  CO_OD_RAM.gyroscope.roll_rate = %d", CO_OD_RAM.gyroscope.roll_rate);
		LOG_DBG( "  CO_OD_RAM.gyroscope.pitch_rate_raw = %d", CO_OD_RAM.gyroscope.pitch_rate_raw);
		LOG_DBG( "  CO_OD_RAM.gyroscope.yaw_rate_raw = %d", CO_OD_RAM.gyroscope.yaw_rate_raw);
		LOG_DBG( "  CO_OD_RAM.gyroscope.roll_rate_raw = %d", CO_OD_RAM.gyroscope.roll_rate_raw);

		LOG_DBG( "  CO_OD_RAM.accelerometer.x = %d", CO_OD_RAM.accelerometer.x);
		LOG_DBG( "  CO_OD_RAM.accelerometer.y = %d", CO_OD_RAM.accelerometer.y);
		LOG_DBG( "  CO_OD_RAM.accelerometer.z = %d", CO_OD_RAM.accelerometer.z);
		LOG_DBG( "  CO_OD_RAM.accelerometer.x_raw = %d", CO_OD_RAM.accelerometer.X_raw);
		LOG_DBG( "  CO_OD_RAM.accelerometer.y_raw = %d", CO_OD_RAM.accelerometer.Y_raw);
		LOG_DBG( "  CO_OD_RAM.accelerometer.z_raw = %d", CO_OD_RAM.accelerometer.Z_raw);

		LOG_DBG( "  CO_OD_RAM.temperature = %d", CO_OD_RAM.temperature);

		LOG_DBG( "  CO_OD_RAM.magnetorquer_current_x.current_set = %d", CO_OD_RAM.magnetorquer.current_x_setpoint);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_current_y.current_set = %d", CO_OD_RAM.magnetorquer.current_y_setpoint);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_current_z.current_set = %d", CO_OD_RAM.magnetorquer.current_z_setpoint);

		LOG_DBG( "  CO_OD_RAM.magnetorquer_pwm_percent.x = %d", CO_OD_RAM.magnetorquer.pwm_x);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_pwm_percent.y = %d", CO_OD_RAM.magnetorquer.pwm_y);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_pwm_percent.z = %d", CO_OD_RAM.magnetorquer.pwm_z);

		LOG_DBG( "  CO_OD_RAM.magnetorquer_current.x = %d", CO_OD_RAM.magnetorquer.current_x);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_current.y = %d", CO_OD_RAM.magnetorquer.current_y);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_current.z = %d", CO_OD_RAM.magnetorquer.current_z);

		LOG_DBG( "  CO_OD_RAM.pos_z_magnetometer_1.x = %d", CO_OD_RAM.pos_z_magnetometer_1.x);
		LOG_DBG( "  CO_OD_RAM.pos_z_magnetometer_1.y = %d", CO_OD_RAM.pos_z_magnetometer_1.y);
		LOG_DBG( "  CO_OD_RAM.pos_z_magnetometer_1.z = %d", CO_OD_RAM.pos_z_magnetometer_1.z);

		LOG_DBG( "  CO_OD_RAM.pos_z_magnetometer_2.x = %d", CO_OD_RAM.pos_z_magnetometer_2.x);
		LOG_DBG( "  CO_OD_RAM.pos_z_magnetometer_2.y = %d", CO_OD_RAM.pos_z_magnetometer_2.y);
		LOG_DBG( "  CO_OD_RAM.pos_z_magnetometer_2.z = %d", CO_OD_RAM.pos_z_magnetometer_2.z);

		LOG_DBG( "  CO_OD_RAM.min_z_magnetometer_1.x = %d", CO_OD_RAM.min_z_magnetometer_1.x);
		LOG_DBG( "  CO_OD_RAM.min_z_magnetometer_1.y = %d", CO_OD_RAM.min_z_magnetometer_1.y);
		LOG_DBG( "  CO_OD_RAM.min_z_magnetometer_1.z = %d", CO_OD_RAM.min_z_magnetometer_1.z);

		LOG_DBG( "  CO_OD_RAM.min_z_magnetometer_1.x = %d", CO_OD_RAM.min_z_magnetometer_2.x);
		LOG_DBG( "  CO_OD_RAM.min_z_magnetometer_1.y = %d", CO_OD_RAM.min_z_magnetometer_2.y);
		LOG_DBG( "  CO_OD_RAM.min_z_magnetometer_1.z = %d", CO_OD_RAM.min_z_magnetometer_2.z);

		for (int i = 0; i < 3; i++) {
			mt_pwm_phase_data_t *data = &g_adcs_data.mt_pwm_data[i];
			LOG_DBG( "  measured_i_sense_voltage[%d] = %d uA, %u mV",
							i,
							data->current_feedback_measurement_uA,
							(uint32_t) (data->current_feedback_measurement_V * 1000));
		}
		//LOG_DBG( "  CO_EM_GENERIC_ERROR:  %u", CO_isError(CO->em, CO_EM_GENERIC_ERROR));
	}
}

static int set_pwm_phase(int i, bool level)
{
	const struct gpio_dt_spec *spec;
	int ret;

	switch (i) {
	case 0:
		spec = &mt_x_phase;
		break;
	case 1:
		spec = &mt_y_phase;
		break;
	case 2:
		spec = &mt_z_phase;
		break;
	default:
		LOG_ERR("Incorrect axis selected: %d", i);
		return -EINVAL;
	}
	ret = gpio_pin_set_dt(spec, level);
	if (ret) {
		LOG_ERR("Unable to set phase pin level: %d", ret);
	}
	return ret;
}

static int set_pwm_output(void) {
	int ret = 0;
	int err = 0;

	for (int i = 0; i < 3; i++ ) {
		int32_t t_now = k_uptime_get(); //in milliseconds
		bool new_state;

		//Updates will come in periodically via CANOpen, this will apply those updates to the PWM outputs.
		if ((g_adcs_data.mt_pwm_data[i].last_update_time == 0) ||
			((t_now - g_adcs_data.mt_pwm_data[i].last_update_time) > 10)) {

			if( g_adcs_data.mt_pwm_data[i].current_pwm_percent != g_adcs_data.mt_pwm_data[i].target_pwm_percent ) {
				LOG_DBG("target_pwm_percent = %d", g_adcs_data.mt_pwm_data[i].target_pwm_percent);

				// can't disable on MCXN? pwmDisableChannel(&PWMD1, g_adcs_data.mt_pwm_data[i].pwm_channel_number);

				// TODO: warning -- there may be a delay due to other threads running between changing the phase
				// below, and setting the new PWM. This could cause a glitch and an unintentional pulse in the wrong
				// direction.
				if (g_adcs_data.mt_pwm_data[i].target_pwm_percent < 0) {
					new_state = true;
				} else {
					new_state = false;
				}
				if (g_adcs_data.mt_pwm_data[i].phase_state != new_state) {
					ret = set_pwm_phase(i, true);
					if (ret) {
						err = ret;
						continue; // could cause guidance issues? mismatched phase and pwm value?
					}
					g_adcs_data.mt_pwm_data[i].phase_state = new_state;
				}

				const int32_t pwm_val = abs(g_adcs_data.mt_pwm_data[i].target_pwm_percent);

				set_pwm(i, pwm_val); // set_pwm does the scaling from percentage to width

				//pwmEnableChannel(&PWMD1, g_adcs_data.mt_pwm_data[i].pwm_channel_number, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, pwm_val));

				g_adcs_data.mt_pwm_data[i].current_pwm_percent = g_adcs_data.mt_pwm_data[i].target_pwm_percent;
				g_adcs_data.mt_pwm_data[i].last_update_time = t_now;
			}
		}
	}
	return err;
}

static int reset_magnetorquer(void)
{
	int err;

	err = gpio_pin_configure_dt(&mt_en, GPIO_OUTPUT_INACTIVE); // drive the pin low -- disable power stage
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}
	err = gpio_pin_set_dt(&mt_en, false);
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}

	err = gpio_pin_set_dt(&n_mt_stby_rst, false); // goto low power/reset mode
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}
	k_sleep(K_MSEC(5));
	err = gpio_pin_set_dt(&mt_en, true);
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}
	k_sleep(K_MSEC(5));
	err = gpio_pin_set_dt(&n_mt_stby_rst, true); // activate
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}
	k_sleep(K_MSEC(5));
#if 0
	err = gpio_pin_configure_dt(&mt_en, GPIO_INPUT); // float the pin; enable power stage
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}
	k_sleep(K_MSEC(5));
#endif
	return err;
}

static int init_magnetorquer(void) {
	int err;

	err = init_gpios();
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}

	err = init_dac();
	if (err) {
		LOG_ERR("Error initializing DAC: %d", err);
		return err;
	}
	// ChibiOS version did this:
	//    dacPutChannelX(&DACD1, 0, 3600); //3V
	// It's DAC resolution also 12 bit; the max value is 4095.
	// If it's max output voltage is 3.3V, then
	// 3.3V * 3600 /4095 = 2.90V, not 3V.
	// Leaving for now.
	// TODO: find out if this is OK.

	// R58 was stuffed wrong; need to change output to 0.0284 (value of 35)
	err = write_dac(177U);
	if (err) {
		LOG_ERR("Error writing DAC: %d", err);
		return err;
	}

	err = init_pwm();
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}
	err = init_adc();
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}

	err = gpio_pin_configure_dt(&mt_en, GPIO_OUTPUT_INACTIVE); // drive the pin low
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}
	err = gpio_pin_set_dt(&mt_en, false);
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}
	err = gpio_pin_set_dt(&n_mt_stby_rst, true);
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}

	err = set_pwm_phase(0, false);
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}
	err = set_pwm_phase(1, false);
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}
	err = set_pwm_phase(2, false);
	if (err) {
		LOG_ERR("Error initializing GPIOS: %d", err);
		return err;
	}

	err = reset_magnetorquer();
	return err;
}

static void check_magnetorquer_fault(void)
{
	int fault;

	fault = gpio_pin_get_dt(&n_mt_en_fault);
	if (!fault) {
		LOG_WRN("Fault on magnetorquer driver(s)!");

		// TODO: ask Andrew if this is ok to do. It wasn't in the old code.
		// LOG_INF("Resetting magnetorquer drivers.");
		// (void)reset_magnetorquer();
	}

}

#if 1
static int handle_magnetorquer(void *p1, void *p2, void *p3)
{
	int err;

	k_thread_name_set(magtqr_id, "magtqr_thread");

	LOG_INF("Starting MAGNETORQUER thread");

#if 0
	err = init_dac();
	if (err) {
		return 0;
	}
	err = write_dac(1024U);
	if (err) {
		return 0;
	}

	err = init_pwm();
	if (err) {
		return 0;
	}
	err = init_adc();
	if (err) {
		return 0;
	}
#else
	init_magnetorquer();
#endif

	err = set_pwm(0, 1000);  // --> ADC
	err = set_pwm(1, 1000);  // --> ADC1
	err = set_pwm(2, 1000);  // --> ADC2

	while (true) {
		uint32_t adc_val;

		err = acquire_adc_readings();
		if (err) {
			return 0;
		}

		for (int i = 0; i < get_num_adc_channels(); i++) {
			err = read_adc(i, &adc_val);
			if (err) {
				continue;
			}
			LOG_INF("ADC num %d: %u mV", i, adc_val);
		}

		check_magnetorquer_fault();
		k_msleep(1000);
	}
}

#else
static int handle_magnetorquer(void *p1, void *p2, void *p3)
{
	int err;

	k_thread_name_set(magtqr_id, "magtqr_thread");

	LOG_INF("Starting MAGNETORQUER thread");

#if 0
	err = set_pwm(0, 2500);
	err = set_pwm(1, 5000);
	err = set_pwm(2, 7500);
#endif

	err = init_magnetorquer();
	if (err) {
		LOG_ERR("Unable to control the magnetorquers!");
		return 0;
	}

	int64_t t_start = k_uptime_get(); //in milliseconds
	int64_t t_last = t_start;
	int64_t t_now = t_start;
	uint32_t adc_val;
	uint32_t iterations = 0;
	int32_t sign;
	float measured_i_sense_voltage;
	float microamps;
	int i;

    for (;;) {
        //LOG_DBG("IMU loop iteration %u system time %llu", iterations, t_last);
		iterations++;

		// x, y, and z are in units of: (for GYRO_FS_SEL = 7) 2097.2LSB/(º/s)
		// temp is in units of decicentigrade (degrees C times 10)
		get_gyro_data(&g_adcs_data.gyro_data.x,
					  &g_adcs_data.gyro_data.y,
					  &g_adcs_data.gyro_data.z,
					  &g_adcs_data.temp_data);

		for (i = 0; i < NUM_MAGS; i++) {
			err = get_mag_reading(i,
								  &g_adcs_data.magnetometer_data[i].x,
								  &g_adcs_data.magnetometer_data[i].y,
								  &g_adcs_data.magnetometer_data[i].z);
		}

		// tell ADC to read all channels at once
		err = acquire_adc_readings();
		if (err) {
			return 0;
		}

		// now read the values acquired
		for (i = 0; i < get_num_adc_channels(); i++) {
			err = read_adc(i, &adc_val);
			if (err) {
				continue;
			}
			measured_i_sense_voltage = (((float) adc_val) / 4096.0f) * 3.3f;
			// Based on the circuit design, this should nominally be 3V/amp.
			// This calculation seems to be within 5%-10% accurate when compared to in line bench DMM readings.
			microamps = (measured_i_sense_voltage / 3.0f) * 1000000.0f;

			g_adcs_data.mt_pwm_data[i].current_feedback_measurement_V = measured_i_sense_voltage;

			sign = (g_adcs_data.mt_pwm_data[i].current_pwm_percent < 0) ? -1 : 1;
			g_adcs_data.mt_pwm_data[i].current_feedback_measurement_uA = (int32_t)(microamps * sign);
		}

		err = set_pwm_output();
		if (err) {
			LOG_WRN("One or more PWM channels could not be set: %d", err);
		}

		check_magnetorquer_fault();

		print_debug_output();

		handle_can_open_data();

		t_now = k_uptime_get();
		int32_t t_sleep = (int32_t)(ITERATION_PERIOD - (t_now - t_last));

		if (t_sleep <= 0) {
			t_sleep = 1;
		}
		k_msleep(t_sleep);
		t_last = t_now;
	}
}
#endif

K_THREAD_DEFINE(magtqr_id, STACK_SIZE, handle_magnetorquer, NULL, NULL, NULL, PRIORITY, 0, 0);
