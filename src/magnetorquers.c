#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#if defined(CONFIG_CAN)
#include <canopennode.h>
#endif
#include <CO_OD.h>
#include <version.h>
#include <app_version.h>
#include <unistd.h>

#include "dac.h"
#include "pwm.h"
#include "adc.h"
#include "imu.h"
#include "magnetometer.h"
#include "windowed_average.h"

LOG_MODULE_REGISTER(magnetorquers, LOG_LEVEL_INF);

/* size of stack area used by each thread */
#define STACK_SIZE 4096

/* scheduling priority used by each thread */
#define PRIORITY 7

#define MAGNETORQUER_STARTUP_DELAY 2000			// roughly when all the helper threads are up; TODO: add interthread signalling for this
#define ITERATION_PERIOD 5						// ms
#define DEBUG_PRINT_PERIOD 1500					// ms
#define MAGNETORQUER_UPDATE_PERIOD 100          // ms
#define ISENSE_GAIN 50.0f						// gain of the INA185 op amp
#define ISENSE_R_OHMS 0.030f					// resistance between the op amp + and - inputs
#define VSENSE_INPUT_OFFSET_TYP_UV 5			// typically, the INA185 can have +/- this many microvolts offset on the input (pre-gain)
#define VSENSE_INPUT_OFFSET_MAX_UV 55			// maximum offset in microvolts -- even when no current is flowing through Rsense

#define DAC_RANGE 4096							// TODO: use real value from device tree
#define DAC_VREF 3.3f
#define R1_OHMS 237								// R58 should have been 23.7K
#define R2_OHMS 1000							// R59
#define R_SENSE_TOTAL (2 * ISENSE_R_OHMS)		// shown in schematic, a second ISENSE_R_OHMS is in series from the op amp - input to ground
#define MAGNETORQUER_CURRENT_LIMIT_A 2.0f		// specified in schematic

#define VREF (MAGNETORQUER_CURRENT_LIMIT_A * R_SENSE_TOTAL)				// input to STSPIN250; with a total of 0.060 ohm sense resistance = ISENSE_R_OHMS * 2, this limits output current to 2A
#define VOUT ((VREF * (R1_OHMS + R2_OHMS)) / R2_OHMS)					// calculate needed input V to voltage divider to get out desired Vref
#define MT_ILIM_DAC_VALUE ((uint32_t)((VOUT * DAC_RANGE) / DAC_VREF))	// convert that to a raw DAC value

#define MAX_PWM_DUTY_CYCLE_X 10000
#define MAX_PWM_DUTY_CYCLE_Y 10000
#define MAX_PWM_DUTY_CYCLE_Z 10000

#define OPERATING_VBUSP_MV 8200					// set Kff so that we get maximum possible current at 100% duty cycle for mid-point of battery voltage
#define R_X_MT 15.5								// DC resistance of X axis magnetorquer
#define R_Y_MT 15.5								// DC resistance of Y axis magnetorquer
#define R_Z_MT 60.0								// DC resistance of Z axis magnetorquer

static const int32_t max_i_ua[] = {
	(int32_t)((OPERATING_VBUSP_MV * 1000) / R_X_MT),
	(int32_t)((OPERATING_VBUSP_MV * 1000) / R_Y_MT),
	(int32_t)((OPERATING_VBUSP_MV * 1000) / R_Z_MT)
};

static const float Kff[] = {
	0.95,
	0.95,
	0.95
};

// starting point -- tune these in the lab
static const float Kp[] = {
	0.5,
	0.5,
	0.5
};

static const int32_t max_pwm_duty_cycles[] = {
	MAX_PWM_DUTY_CYCLE_X,
	MAX_PWM_DUTY_CYCLE_Y,
	MAX_PWM_DUTY_CYCLE_Z
};

extern const k_tid_t magtqr_id;

static uint32_t iterations;

typedef struct {
	int32_t target_current_uA;			// ADCS code on C3 requests this over CAN
	bool target_changed;				// true if the C3 has changed the value
	int32_t goal_pwm_percent;			// Negative values indicate the phase should be inverted

	int32_t active_pwm_percent;			// 0-10000

	wnd_avg_store ofs_mv_store;			// used to compute running average of a recent set of offset voltages
	uint32_t ofs_mv;					// compensation for inherent op-amp input voltage offset
	float feedback_measurement_V;		// Volts, Note: this is the average voltage while the PWM output is high.
	int32_t feedback_measurement_uA;	// uA. Note: this is the average current flowing while the PWM output is high. It does not represent overall average current.

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

static adcs_data_t g_adcs_data = {
	.mt_pwm_data[0].ofs_mv_store.name = "adcx",
	.mt_pwm_data[1].ofs_mv_store.name = "adcy",
	.mt_pwm_data[2].ofs_mv_store.name = "adcz"
};

static int32_t *setpoints[] = { // make it easy to index by numeric axis
	&CO_OD_RAM.magnetorquer.current_x_setpoint,
	&CO_OD_RAM.magnetorquer.current_y_setpoint,
	&CO_OD_RAM.magnetorquer.current_z_setpoint
};

static char *axis_names[] = {
	"X", "Y", "Z"
};

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

static int32_t calc_pwm_from_uA_setpoint(const int32_t target_uA, int axis);
static int16_t raw_to_milligauss(int16_t raw);

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
	// NOTE: the device tree should set the x/y/z_phase lines as active low, to fix logical-sense of the remainder of this code.
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
//
// oresat-configs sdo [-h] [--oresat {0,0.5,1}] BUS NODE MODE INDEX SUBINDEX [VALUE]
// so for you oresat-configs sdo can0 battery_1 read <index> <subindex>
// oresat-configs sdo can0 adcs read versions fw_version
// oresat-configs sdo can0 adcs read temperature foo
// oresat-configs sdo can0 adcs read magnetorquer pwm_x
// oresat-configs sdo can0 adcs read magnetorquer current_x
// oresat-configs sdo can0 adcs write magnetorquer current_x_setpoint 1000
// oresat-configs sdo can0 adcs read pos_z_magnetometer_1 x

static void handle_can_open_data(void)
{
	size_t ver_size = sizeof(CO_OD_RAM.versions.fw_version);

	CO_LOCK_OD();

	strncpy(CO_OD_RAM.versions.fw_version, &APP_VERSION_STRING[6], ver_size);

	for (int i = 0; i < 3; i++) {
		if (g_adcs_data.mt_pwm_data[i].target_current_uA != *setpoints[i]) {
			g_adcs_data.mt_pwm_data[i].target_current_uA = *setpoints[i];
			g_adcs_data.mt_pwm_data[i].target_changed = true;
			LOG_DBG("%s target uA now: %d", axis_names[i], g_adcs_data.mt_pwm_data[i].target_current_uA);
		}
	}

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

	CO_OD_RAM.magnetorquer.current_x = g_adcs_data.mt_pwm_data[0].feedback_measurement_uA;
	CO_OD_RAM.magnetorquer.current_y = g_adcs_data.mt_pwm_data[1].feedback_measurement_uA;
	CO_OD_RAM.magnetorquer.current_z = g_adcs_data.mt_pwm_data[2].feedback_measurement_uA;

	CO_OD_RAM.magnetorquer.pwm_x = g_adcs_data.mt_pwm_data[0].active_pwm_percent;
	CO_OD_RAM.magnetorquer.pwm_y = g_adcs_data.mt_pwm_data[1].active_pwm_percent;
	CO_OD_RAM.magnetorquer.pwm_z = g_adcs_data.mt_pwm_data[2].active_pwm_percent;

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
	CO_UNLOCK_OD();
}

static void process_can_open_targets(adcs_data_t *data)
{
	for (int i = 0; i < 3; i++) {
		if (data->mt_pwm_data[i].target_changed) {
			data->mt_pwm_data[i].target_changed = false;
			data->mt_pwm_data[i].goal_pwm_percent = calc_pwm_from_uA_setpoint(*setpoints[i], i);
		}
	}
}

static void print_debug_output(void) {
	static int64_t last_print_time = 0;
	int64_t now = k_uptime_get();

	if ((now - last_print_time) > DEBUG_PRINT_PERIOD) {
		last_print_time = now;

		LOG_INF("Magnetorquer iterations: %u", iterations);
		LOG_DBG( "================");
		LOG_DBG( "CANOpen Data:");
		LOG_DBG( "  CO_OD_RAM.versions.fw_version = %.5s", CO_OD_RAM.versions.fw_version);
		LOG_DBG( "  CO_OD_RAM.gyroscope.pitch_rate = %d", CO_OD_RAM.gyroscope.pitch_rate);
		LOG_DBG( "  CO_OD_RAM.gyroscope.yaw_rate = %d", CO_OD_RAM.gyroscope.yaw_rate);
		LOG_DBG( "  CO_OD_RAM.gyroscope.roll_rate = %d", CO_OD_RAM.gyroscope.roll_rate);
		LOG_DBG( "  CO_OD_RAM.gyroscope.pitch_rate_raw = %d", CO_OD_RAM.gyroscope.pitch_rate_raw);
		LOG_DBG( "  CO_OD_RAM.gyroscope.yaw_rate_raw = %d", CO_OD_RAM.gyroscope.yaw_rate_raw);
		LOG_DBG( "  CO_OD_RAM.gyroscope.roll_rate_raw = %d", CO_OD_RAM.gyroscope.roll_rate_raw);
#if 0
		LOG_DBG( "  CO_OD_RAM.accelerometer.x = %d", CO_OD_RAM.accelerometer.x);
		LOG_DBG( "  CO_OD_RAM.accelerometer.y = %d", CO_OD_RAM.accelerometer.y);
		LOG_DBG( "  CO_OD_RAM.accelerometer.z = %d", CO_OD_RAM.accelerometer.z);
		LOG_DBG( "  CO_OD_RAM.accelerometer.x_raw = %d", CO_OD_RAM.accelerometer.X_raw);
		LOG_DBG( "  CO_OD_RAM.accelerometer.y_raw = %d", CO_OD_RAM.accelerometer.Y_raw);
		LOG_DBG( "  CO_OD_RAM.accelerometer.z_raw = %d", CO_OD_RAM.accelerometer.Z_raw);
#endif
		LOG_DBG( "  CO_OD_RAM.temperature = %d", CO_OD_RAM.temperature);

		LOG_DBG( "  CO_OD_RAM.magnetorquer_current_x.current_setpoint = %d", CO_OD_RAM.magnetorquer.current_x_setpoint);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_current_y.current_setpoint = %d", CO_OD_RAM.magnetorquer.current_y_setpoint);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_current_z.current_setpoint = %d", CO_OD_RAM.magnetorquer.current_z_setpoint);

		LOG_DBG( "  CO_OD_RAM.magnetorquer_pwm_percent.x = %d", CO_OD_RAM.magnetorquer.pwm_x);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_pwm_percent.y = %d", CO_OD_RAM.magnetorquer.pwm_y);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_pwm_percent.z = %d", CO_OD_RAM.magnetorquer.pwm_z);

		LOG_DBG( "  CO_OD_RAM.magnetorquer_current.x = %d", CO_OD_RAM.magnetorquer.current_x);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_current.y = %d", CO_OD_RAM.magnetorquer.current_y);
		LOG_DBG( "  CO_OD_RAM.magnetorquer_current.z = %d", CO_OD_RAM.magnetorquer.current_z);

#if 0
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
#endif

		for (int i = 0; i < 3; i++) {
			mt_pwm_phase_data_t *data = &g_adcs_data.mt_pwm_data[i];
			LOG_DBG( "  i_sense[%d] = %.3f mA, %.3f mV",
					i,
					(double)data->feedback_measurement_uA / 1000.0,
					(double)data->feedback_measurement_V * 1000.0);
		}
		//LOG_DBG( "  CO_EM_GENERIC_ERROR:  %u", CO_isError(CO->em, CO_EM_GENERIC_ERROR));
	}
}

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
static int32_t calc_pwm_from_uA_setpoint(const int32_t target_uA, int axis)
{
	int32_t pwm = 0;

	if (target_uA == 0) {
		return pwm;
	}

	int32_t goal_uA = saturate_int32_t(target_uA, -max_i_ua[axis], +max_i_ua[axis]);
	const int32_t actual_uA = g_adcs_data.mt_pwm_data[axis].feedback_measurement_uA;

	float feed_forward = goal_uA * Kff[axis];
	int32_t error = target_uA - actual_uA;
	float p = error * Kp[axis];

	pwm = (int32_t)(((feed_forward + p) * max_pwm_duty_cycles[axis]) / max_i_ua[axis]);
	pwm = saturate_int32_t(pwm, -max_pwm_duty_cycles[axis], max_pwm_duty_cycles[axis]);

	LOG_DBG("Axis:%d, target_mA:%.3f, goal_mA:%.3f, actual_mA:%.3f, max_pwm:%d, ff:%.3f, error:%d, p:%.3f, pwm:%d",
			axis, target_uA / 1000.0, goal_uA / 1000.0, actual_uA / 1000.0, max_pwm_duty_cycles[axis], 
			(double)feed_forward, error, (double)p, pwm);
	return(pwm);

#if 0
	// ChibiOS stuff

	if (axis <= 1) {
		//X and Y axes
		//1700 => 1000000 uA
		//500 => 295000 uA (this is hard/impossible to measure using the ADC
		pwm = current_uA / (988000.0 / 1700.0);
		const int32_t pwm_duty_max_value = 1700;
		pwm = saturate_int32_t(pwm, -pwm_duty_max_value, pwm_duty_max_value);
	} else {
		//Z axis
		//1700 => 344000 uA
		//500 => 102000 uA  (this is hard/impossible to measure using the ADC
		pwm = current_uA / (344000.0 / 1700.0);
		const int32_t pwm_duty_max_value = 4940;
		pwm = saturate_int32_t(pwm, -pwm_duty_max_value, pwm_duty_max_value);
	}
	return(pwm);
#endif
}

static int16_t raw_to_milligauss(int16_t raw)
{
		float gauss = 1000.0f * ((float) raw) / 4096.0f;
		return (int16_t)gauss;
}

/**
 * @brief get_current_readings()
 * Read current through magnetorquers.
 *
 * @param axes   - pointer to array of the number of ADC
 *  			 channels, which should also be the number of
 *  			 axes; TODO: add assert if not true
 * @return int - non-zero value on error, 0 if none
 */
static int get_current_readings(mt_pwm_phase_data_t *axes)
{
	int err;
	int i;
	uint32_t adc_mv;
	int32_t sign;
	float measured_i_sense_voltage;
	float microamps;

	// tell ADC to read all channels at once
	err = acquire_adc_readings();
	if (err) {
		return 0;
	}

	// now read the values acquired
	for (i = 0; i < get_num_adc_channels(); i++) {
		err = read_adc(i, &adc_mv);
		if (err) {
			continue;
		}

		// TODO: deal with Vin offset pre-gain on op-amp.
		// Post-gain will be +/-5uV * 50 = 0.25 mA typical,
		// +/-55uV * 50 = 2.75 mA maximum on the ADC input, when PWM = 0.
		// This is equivalent to 166 to 1833 uA, below which we cannot measure.
		// This also means pwm values might have a deadband between 0 and 700
		// on the 10,000 range scale.
		// Question: can we measure the at-rest ADC right before turning on the PWM to
		// a non-zero value, and just use that as the ofs_mv below? How much can it
		// drift while the ADCS is actively requesting current? How long will the duration
		// be for the ADCS request? Seconds? Minutes? Hours?

		if (axes[i].active_pwm_percent == 0) {
			axes[i].ofs_mv = update_windowed_average(&axes[i].ofs_mv_store, adc_mv);
		}

		measured_i_sense_voltage = ((float)(adc_mv - axes[i].ofs_mv)) / (1000.0f * ISENSE_GAIN);

		microamps = (measured_i_sense_voltage / ISENSE_R_OHMS) * 1000000.0f;
		sign = (axes[i].active_pwm_percent < 0) ? -1 : 1;

		axes[i].feedback_measurement_V = measured_i_sense_voltage;
		axes[i].feedback_measurement_uA = (int32_t)(microamps * sign);
	}
	return err;
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
	LOG_DBG("set MT_%d_PHASE to %d", i, level);
	// NOTE: the device tree should set the x/y/z_phase lines as active low, to fix logical-sense of the remainder of this code.
	ret = gpio_pin_set_dt(spec, level);
	if (ret) {
		LOG_ERR("Unable to set phase pin level: %d", ret);
	}
	return ret;
}

static int set_pwm_output(mt_pwm_phase_data_t *axes)
{
	int ret = 0;
	int err = 0;

	for (int i = 0; i < 3; i++ ) {
		bool new_state;

		//Updates will come in periodically via CANOpen, this will apply those updates to the PWM outputs.
		if( axes[i].active_pwm_percent != axes[i].goal_pwm_percent ) {
			LOG_DBG("goal_pwm_percent = %d", axes[i].goal_pwm_percent);

			if (axes[i].goal_pwm_percent < 0) {
				new_state = true;
			} else {
				new_state = false;
			}
			const int32_t pwm_val = abs(axes[i].goal_pwm_percent);

			// lock scheduler so we minimize delays between changing phase and setting pwm
			k_sched_lock();
			if (axes[i].phase_state != new_state) {
				ret = set_pwm_phase(i, new_state);
				if (ret) { // this should never be in error, but do the right thing anyway
					err = ret;
					k_sched_unlock();
					continue; // could cause guidance issues? mismatched phase and pwm value?
				}
				axes[i].phase_state = new_state;
			}
			set_pwm(i, pwm_val); // set_pwm does the scaling from percentage to width
			k_sched_unlock();

			axes[i].active_pwm_percent = axes[i].goal_pwm_percent;
		}
	}
	return err;
}

static int reset_magnetorquer(void)
{
	int err;

	err = gpio_pin_configure_dt(&mt_en, GPIO_OUTPUT_INACTIVE); // drive the pin low -- disable power stage
	if (err) {
		LOG_ERR("Error configuring mt_en output low: %d", err);
		return err;
	}
	err = gpio_pin_set_dt(&mt_en, false);
	if (err) {
		LOG_ERR("Error setting mt_en low: %d", err);
		return err;
	}

	err = gpio_pin_set_dt(&n_mt_stby_rst, false); // goto low power/reset mode
	if (err) {
		LOG_ERR("Error setting n_mt_stby_rst low: %d", err);
		return err;
	}
	k_sleep(K_MSEC(5));
	err = gpio_pin_set_dt(&mt_en, true);
	if (err) {
		LOG_ERR("Error setting mt_en high: %d", err);
		return err;
	}
	k_sleep(K_MSEC(5));
	err = gpio_pin_set_dt(&n_mt_stby_rst, true); // activate
	if (err) {
		LOG_ERR("Error setting n_mt_stby_rst high: %d", err);
		return err;
	}
	k_sleep(K_MSEC(5));
#if 0
	err = gpio_pin_configure_dt(&mt_en, GPIO_INPUT); // float the pin; enable power stage
	if (err) {
		LOG_ERR("Error floating mt_en: %d", err);
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

	LOG_DBG("Set MT_ILIM; VREF_MV = %d, VOUT_MV = %d, DAC = %d",
			(int)(VREF * 1000.0f), (int)(VOUT * 1000.0f), MT_ILIM_DAC_VALUE);
	err = write_dac(MT_ILIM_DAC_VALUE);
	if (err) {
		LOG_ERR("Error writing DAC: %d", err);
		return err;
	}

	err = init_pwm();
	if (err) {
		LOG_ERR("Error initializing PWMs: %d", err);
		return err;
	}
	err = init_adc();
	if (err) {
		LOG_ERR("Error initializing ADCs: %d", err);
		return err;
	}

	err = gpio_pin_configure_dt(&mt_en, GPIO_OUTPUT_INACTIVE); // drive the pin low
	if (err) {
		LOG_ERR("Error configuring mt_en output low: %d", err);
		return err;
	}
	err = gpio_pin_set_dt(&mt_en, false);
	if (err) {
		LOG_ERR("Error setting mt_en low: %d", err);
		return err;
	}
	err = gpio_pin_set_dt(&n_mt_stby_rst, true);
	if (err) {
		LOG_ERR("Error setting n_mt_stby_rst high: %d", err);
		return err;
	}

	err = set_pwm_phase(0, false);
	if (err) {
		LOG_ERR("Error setting x_mt_phase low: %d", err);
		return err;
	}
	err = set_pwm_phase(1, false);
	if (err) {
		LOG_ERR("Error setting y_mt_phase low: %d", err);
		return err;
	}
	err = set_pwm_phase(2, false);
	if (err) {
		LOG_ERR("Error setting z_mt_phase low: %d", err);
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

#if defined(CONFIG_MAGNETORQUER_EXPLORE)

static int32_t pwm_pct[3] = {0};
static const char axis_let[3] = "xyz";

typedef enum test_modes {
	TM_OFF,
	TM_ADC,
	TM_CURRENT,
	TM_LOOP,
	TM_LOOP_RAMP,
	TM_MODE_COUNT // number of possible modes
} test_modes;

typedef struct {
	const char *name;
	const char *desc;
} test_mode_info;

static test_mode_info tm_info[] = {
	{"TM_OFF", "only shell"},
	{"TM_ADC", "read and display ADC sense channels"},
	{"TM_CURRENT", "read and display current sense measurements"},
	{"TM_LOOP", "run mt control loop"},
	{"TM_LOOP_RAMP", "run mt control loop while ramping target"}
};

static test_modes mt_test_mode;

typedef struct {
	uint32_t step_ms;
	uint32_t step_ua;
	uint32_t start_ua;
	uint32_t stop_ua;
	int axis;
} mt_ramp;

static mt_ramp mt_ramp_info = {
	.step_ms = 1000,
	.step_ua = 10000,
	.start_ua = 0,
	.stop_ua = 500000
};

static void reset_ramp_mode(void)
{
	*setpoints[mt_ramp_info.axis] = 0;
	g_adcs_data.mt_pwm_data[mt_ramp_info.axis].goal_pwm_percent = 0;
	set_pwm_output(g_adcs_data.mt_pwm_data);
}

// test code
static int handle_magnetorquer(void *p1, void *p2, void *p3)
{
	int err;
	uint32_t adc_val;
	int64_t t_start = k_uptime_get(); //in milliseconds
	int64_t t_now = t_start;
	int64_t t_last = t_start;
	int64_t t_mt_last = t_start;
	int64_t t_mt_loop = 0;
	test_modes active_mode = TM_OFF;
	bool tlr_printed = false;
	bool print_loop = true;

	k_thread_name_set(magtqr_id, "magtqr_thread");

	LOG_INF("Starting MAGNETORQUER thread");

	init_magnetorquer();

	set_pwm_phase(0, pwm_pct[0] < 0);
	set_pwm_phase(1, pwm_pct[1] < 0);
	set_pwm_phase(2, pwm_pct[2] < 0);
	err = set_pwm(0, abs(pwm_pct[0]));
	err = set_pwm(1, abs(pwm_pct[1]));
	err = set_pwm(2, abs(pwm_pct[2]));

	LOG_INF("Testing interactive test loop");
	while (true) {

		if ((t_mt_last == 0) || ((t_now - t_mt_last) > MAGNETORQUER_UPDATE_PERIOD)) {
			t_mt_last = t_now;
			active_mode = mt_test_mode; // use the user-set mode now
		} else {
			active_mode = TM_OFF; // not time to execute user's mode yet
		}

		switch (active_mode) {
		case TM_OFF:
			if ((mt_test_mode == active_mode) && (t_mt_loop > 0)) {
				t_mt_loop = 0; // terminate running loop ramp
				reset_ramp_mode();
				tlr_printed = false;
				print_loop = true;
				LOG_INF("loop ramp terminated");
			}
			break;
		case TM_ADC:
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
			break;
		case TM_CURRENT:
			err = get_current_readings(g_adcs_data.mt_pwm_data);
			if (err) {
				LOG_WRN("One or more ADC channels read in error: %d", err);
			}

			LOG_INF("x V:%.5f, %.3f mA",
					(double)g_adcs_data.mt_pwm_data[0].feedback_measurement_V,
					g_adcs_data.mt_pwm_data[0].feedback_measurement_uA / 1000.0);
			LOG_INF("y V:%.5f, %.3f mA",
					(double)g_adcs_data.mt_pwm_data[1].feedback_measurement_V,
					g_adcs_data.mt_pwm_data[1].feedback_measurement_uA / 1000.0);
			LOG_INF("z V:%.5f, %.3f mA",
					(double)g_adcs_data.mt_pwm_data[2].feedback_measurement_V,
					g_adcs_data.mt_pwm_data[2].feedback_measurement_uA / 1000.0);
			break;
		case TM_LOOP_RAMP:
			/*
			 * Every loop update interval, increment current axis target uA by step size.
			 * If incremented beyond the limit, start over at 0.
			 */
			if (!tlr_printed) {
				tlr_printed = true;
				mt_ramp_info.stop_ua = max_i_ua[mt_ramp_info.axis]; // adjust the upper bounds per axis
				LOG_INF("x pwm, x targ uA, x fb uA, y pwm, y targ uA, y fb uA, z pwm, z targ uA, z fb uA");
			}
			if ((t_now - t_mt_loop) > mt_ramp_info.step_ms) {
				t_mt_loop = t_now;
				*setpoints[mt_ramp_info.axis] += mt_ramp_info.step_ua;
				if (*setpoints[mt_ramp_info.axis] > mt_ramp_info.stop_ua) {
					*setpoints[mt_ramp_info.axis] = mt_ramp_info.start_ua;
				}
				print_loop = true;
			} else {
				print_loop = false;
			}
			// fall through
		case TM_LOOP:
			err = get_current_readings(g_adcs_data.mt_pwm_data);
			if (err) {
				LOG_WRN("One or more ADC channels read in error: %d", err);
			}
			if (print_loop) {
				LOG_INF("%d, %d, %d, %d, %d, %d, %d, %d, %d",
						g_adcs_data.mt_pwm_data[0].active_pwm_percent, g_adcs_data.mt_pwm_data[0].target_current_uA / 1000, g_adcs_data.mt_pwm_data[0].feedback_measurement_uA / 1000,
						g_adcs_data.mt_pwm_data[1].active_pwm_percent, g_adcs_data.mt_pwm_data[1].target_current_uA / 1000, g_adcs_data.mt_pwm_data[1].feedback_measurement_uA / 1000,
						g_adcs_data.mt_pwm_data[2].active_pwm_percent, g_adcs_data.mt_pwm_data[2].target_current_uA / 1000, g_adcs_data.mt_pwm_data[2].feedback_measurement_uA / 1000
						);
			}

			process_can_open_targets(&g_adcs_data);

			err = set_pwm_output(g_adcs_data.mt_pwm_data);
			if (err) {
				LOG_WRN("One or more PWM channels could not be set: %d", err);
			}
			break;
		default:
			break;
		}

		check_magnetorquer_fault();

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

#if defined(CONFIG_SHELL)
static int cmd_mtpwm(const struct shell *sh, size_t argc, char **argv)
{
	char chaxis;
	int axis = -1;
	int pwm;
	int err;

	if (argc < 2) {
		shell_print(sh, "Current pwm values: x=%d, y=%d, z=%d", pwm_pct[0], pwm_pct[1], pwm_pct[2]);
		return 0;
	}

	chaxis = argv[1][0];
	switch (chaxis) {
	case 'x':
		axis = 0;
		break;
	case 'y':
		axis = 1;
		break;
	case 'z':
		axis = 2;
		break;
	default:
		axis = atoi(argv[1]);
	}

	if ((axis < 0) || (axis > 3)) {
		shell_error(sh, "Axis out of range: %d", axis);
		return 0;
	}
	if (argc < 3) {
		shell_print(sh, "PWM on axis %d = %d", axis, pwm_pct[axis]);
		return 0;
	}

	pwm = atoi(argv[2]);
	if ((pwm < -10000) || (pwm > 10000)) {
		shell_error(sh, "PWM out of range: %d", pwm);
		return 0;
	}
	pwm_pct[axis] = pwm;
	g_adcs_data.mt_pwm_data[axis].goal_pwm_percent = pwm;

	set_pwm_phase(axis, pwm_pct[axis] < 0);
	err = set_pwm(axis, abs(pwm_pct[axis]));

	g_adcs_data.mt_pwm_data[axis].active_pwm_percent = pwm;
	shell_print(sh, "Set axis %d (%c) pwm = %d; err: %d", axis, axis_let[axis], pwm, err);

	return 0;
}

static int cmd_frqpwm(const struct shell *sh, size_t argc, char **argv)
{
	char chaxis;
	int axis = -1;
	int freq;
	int err;

	if (argc < 2) {
		shell_print(sh, "Current pwm frequencies: x=%u, y=%u, z=%u",
					get_pwm_frequency(0), get_pwm_frequency(1), get_pwm_frequency(2));
		return 0;
	}

	chaxis = argv[1][0];
	switch (chaxis) {
	case 'x':
		axis = 0;
		break;
	case 'y':
		axis = 1;
		break;
	case 'z':
		axis = 2;
		break;
	default:
		axis = atoi(argv[1]);
	}
	if ((axis < 0) || (axis > 3)) {
		shell_error(sh, "Axis out of range: %d", axis);
		return 0;
	}
	if (argc < 3) {
		shell_print(sh, "Frequency on axis %d = %d", axis, get_pwm_frequency(axis));
		return 0;
	}
	freq = atoi(argv[2]);
	if ((freq < 1) || (freq > 100000)) {
		shell_error(sh, "Frequency out of range: %d", freq);
		return 0;
	}
	err = set_pwm_frequency(axis, freq);
	shell_print(sh, "Set axis %d (%c) pwm frequency = %u; err: %d", axis, axis_let[axis], freq, err);

	return 0;
}

static int cmd_ua2pwm(const struct shell *sh, size_t argc, char **argv)
{
	char chaxis;
	int axis = -1;
	int32_t target_ua;
	int32_t pwm;

	if (argc < 3) {
		shell_error(sh, "Missing parameters");
		return 0;
	}

	chaxis = argv[1][0];
	switch (chaxis) {
	case 'x':
		axis = 0;
		break;
	case 'y':
		axis = 1;
		break;
	case 'z':
		axis = 2;
		break;
	default:
		axis = atoi(argv[1]);
	}
	if ((axis < 0) || (axis > 3)) {
		shell_error(sh, "Axis out of range: %d", axis);
		return 0;
	}
	target_ua = (int32_t)atoi(argv[2]);
	pwm = calc_pwm_from_uA_setpoint(target_ua, axis);

	shell_print(sh, "Axis %d (%c) pwm for %d uA = %d", axis, axis_let[axis], pwm, target_ua);

	return 0;
}

static int cmd_mtmode(const struct shell *sh, size_t argc, char **argv)
{
	int i;
	char c;

	if (argc < 2) {
		shell_print(sh, "Current mt mode: %d (%s: %s)",
					mt_test_mode,
					tm_info[mt_test_mode].name,
					tm_info[mt_test_mode].desc);
		return 0;
	}

	while ((c = getopt(argc, argv, "h")) != -1) {
		switch (c) {
		case 'h':
			for (i = 0; i < TM_MODE_COUNT; i++) {
				shell_print(sh, "Mode %d: %s: %s", i, tm_info[i].name, tm_info[i].desc);
			}
			return 0;
		default:
			break;
		}
		break;
	}

	const char *sel_mode = argv[1];
	const char *name;

	for (i = 0; i < TM_MODE_COUNT; i++) {
		name = tm_info[i].name;
		if (strncmp(sel_mode, name, strlen(name)) == 0) {
			mt_test_mode = (test_modes)i;
		}
	}
	if (i >= TM_MODE_COUNT) {
		int mode = atoi(sel_mode);
		LOG_DBG("Mode not found by name: %s; using atoi: %d", sel_mode, mode);

		if ((mode >= 0) && (mode < TM_MODE_COUNT)) {
			mt_test_mode = (test_modes)mode;
		} else {
			shell_error(sh, "Mode '%s' not found.", sel_mode);
			return 0;
		}
	}
	shell_print(sh, "Selected mode: %s: %s", tm_info[mt_test_mode].name, tm_info[mt_test_mode].desc);

	if (argc > 2) {
		reset_ramp_mode();
		int axis = atoi(argv[2]);
		if ((axis >= 0) && (axis < 3)) {
			mt_ramp_info.axis = axis;
			LOG_INF("Set ramp to use axis %d", axis);
		}
	}

	return 0;
}

SHELL_CMD_ARG_REGISTER(mtpwm, NULL,  "mtpwm  [<axis>] [<new duty cycle value>]", cmd_mtpwm, 1, 2);
SHELL_CMD_ARG_REGISTER(frqpwm, NULL, "frqpwm [<axis>] [<new frequency in Hz>]", cmd_frqpwm, 1, 2);
SHELL_CMD_ARG_REGISTER(ua2pwm, NULL, "ua2pwm [<axis>] [<target current in uA>]", cmd_ua2pwm, 3, 1);
SHELL_CMD_ARG_REGISTER(mtmode, NULL, "mtmode [<num>] [<axis>] | [-h]", cmd_mtmode, 1, 2);
#endif

#else // not CONFIG_MAGNETORQUER_EXPLORE (normal mode)

static int handle_magnetorquer(void *p1, void *p2, void *p3)
{
	int err;

	k_thread_name_set(magtqr_id, "magtqr_thread");
	k_sleep(K_MSEC(MAGNETORQUER_STARTUP_DELAY));

	LOG_INF("Starting MAGNETORQUER thread");

	err = init_magnetorquer();
	if (err) {
		LOG_ERR("Unable to control the magnetorquers!");
		return 0;
	}

	int64_t t_start = k_uptime_get(); //in milliseconds
	int64_t t_now = t_start;
	int64_t t_last = t_start;
	int64_t t_mt_last = t_start;
	int i;

	LOG_INF("Starting magnetorquer loop");
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
			int ret = get_mag_reading(i,
								  &g_adcs_data.magnetometer_data[i].x,
								  &g_adcs_data.magnetometer_data[i].y,
								  &g_adcs_data.magnetometer_data[i].z);
			if (ret) {
				err = ret; // be sure to report any errors, even just 1
			}
		}
		if (err) {
			LOG_WRN("One or more magnetometers could not be read: %d", err);
		}

		//Updates will come in periodically via CANOpen, this will apply those updates to the PWM outputs.
		if ((t_mt_last == 0) || ((t_now - t_mt_last) > MAGNETORQUER_UPDATE_PERIOD)) {
			t_mt_last = t_now;

			err = get_current_readings(g_adcs_data.mt_pwm_data);
			if (err) {
				LOG_WRN("One or more ADC channels read in error: %d", err);
			}

			process_can_open_targets(&g_adcs_data);

			err = set_pwm_output(g_adcs_data.mt_pwm_data);
			if (err) {
				LOG_WRN("One or more PWM channels could not be set: %d", err);
			}
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

#endif // CONFIG_MAGNETORQUER_EXPLORE

K_THREAD_DEFINE(magtqr_id, STACK_SIZE, handle_magnetorquer, NULL, NULL, NULL, PRIORITY, 0, 0);
