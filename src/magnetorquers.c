#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>

#include "dac.h"
#include "pwm.h"
#include "adc.h"

LOG_MODULE_REGISTER(magnetorquers, CONFIG_LOG_DEFAULT_LEVEL);

/* size of stack area used by each thread */
#define STACK_SIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

extern const k_tid_t magtqr_id;

static int handle_magnetorquer(void *p1, void *p2, void *p3)
{
	int err;

	k_thread_name_set(magtqr_id, "magtqr_thread");

	LOG_INF("Starting MAGNETORQUER thread");

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
	err = set_pwm(0, 1000, 100);
	err = set_pwm(1, 1000, 200);
	err = set_pwm(2, 1000, 300);

	err = init_adc();
	if (err) {
		return 0;
	}

	err = acquire_adc_readings();
	if (err) {
		return 0;
	}

	uint32_t adc_val;
	for (int i = 0; i < get_num_adc_channels(); i++) {
		err = read_adc(i, &adc_val);
		if (err) {
			continue;
		}
		LOG_INF("ADC num %d: %u mV", i, adc_val);
	}

	while (true) {
		k_msleep(100);
	}
}

K_THREAD_DEFINE(magtqr_id, STACK_SIZE, handle_magnetorquer, NULL, NULL, NULL, PRIORITY, 0, 0);

#if 0



FROM CHIBIOS CODE:

#define ADC_NUM_CHANNELS           4
#define MY_SAMPLING_NUMBER         32
#define ADC_BUFF_SIZE              (ADC_NUM_CHANNELS * MY_SAMPLING_NUMBER)
static adcsample_t                 adc_sample_buff[ADC_BUFF_SIZE];

static float measured_i_sense_voltage[ADC_NUM_CHANNELS];;

/*
 * Context references for ADC conversion triggered by TIM1
 * http://forum.chibios.org/viewtopic.php?t=2254
 * https://forum.chibios.org/viewtopic.php?t=6036
 * https://forum.chibios.org/viewtopic.php?t=2093
 */


//ADC_SMPR_SMP_1P5
//ADC_SMPR_SMP_7P5
//ADC_SMPR_SMP_13P5
//ADC_SMPR_SMP_71P5
//ADC_SMPR_SMP_239P5


//volatile uint32_t adc_conversion_complete_callback_count = 0;
//void adc_conversion_complete_callback(ADCDriver *adcp) {
//	adc_conversion_complete_callback_count++;
//}


/**
 * This ADC is configured to trigger a single batch of ADC conversions based on the risigion edge of TRIGO from TIM1
 */
static const ADCConversionGroup adcgrpcfg_tim1_trigo = {
  .circular = FALSE,                                             /* Enables the circular buffer mode for the group.  */
  .num_channels = ADC_NUM_CHANNELS,
  .end_cb = NULL,
  .error_cb = NULL,
  .cfgr1 = ADC_CFGR1_RES_12BIT | ADC_CFGR1_EXTEN_RISING, /* CFGR1 */
  .tr = ADC_TR(0, 0),                                    /* TR */
  .smpr = ADC_SMPR_SMP_1P5,                             /* SMPR */
  .chselr = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6 | ADC_CHSELR_CHSEL7     /* CHSELR, note, for continuous conversion mode you must configure 1 or an even number of channels */
};


static const DACConfig dac_config = {
  .init         = 2047u,
  .datamode     = DAC_DHRM_12BIT_RIGHT,
  .cr           = 0
};


#define PWM_TIMER_FREQ	1000000 // Hz
#define PWM_FREQ		2500// periods per sec
#define PWM_PERIOD		PWM_TIMER_FREQ/PWM_FREQ


//FIXME these PWM channel mappings may be wrong? or the mod-wires on the dev board may be wrong. Either way, Z-axis in the firmware is controling X-axis magnetorquer in the hardware on the dev ADCS board
//PB13
#define MT_Z_PWM_PWM_CHANNEL     (1 - 1)
//PB14
#define MT_Y_PWM_PWM_CHANNEL     (2 - 1)
//PB15
#define MT_X_PWM_PWM_CHANNEL     (3 - 1)

/**
 * This PWM block is configured to enable the TRIGO output to start an ADC conversion batch when the PWM edge goes high.
 */
static PWMConfig pwmcfg_1_trigo = {
  .frequency = PWM_TIMER_FREQ,
  .period = PWM_PERIOD,
  .callback = NULL,
  .channels = {
   {PWM_OUTPUT_ACTIVE_LOW | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_ACTIVE_LOW | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_ACTIVE_LOW | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  .cr2 = TIM_CR2_MMS_1,//CR2
 #if STM32_PWM_USE_ADVANCED
   .bdtr = 0, //BDTR
 #endif
   .dier = 0,//DIER
};



typedef enum {
    ADCS_OD_ERROR_INFO_CODE_NONE = 0,
    ADCS_OD_ERROR_INFO_CODE_IMU_COMM_FAILURE,
    ADCS_OD_ERROR_INFO_CODE_ACCL_CHIP_ID_MISMATCH,
    ADCS_OD_ERROR_INFO_CODE_GYRO_CHIP_ID_MISMATCH,
	ADCS_OD_ERROR_INFO_CODE_IMU_INIT_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_IMU_DATA_UPDATE_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_MAGNETOMETER_0_INIT_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_MAGNETOMETER_1_INIT_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_MAGNETOMETER_2_INIT_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_MAGNETOMETER_3_INIT_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_LTZ_PLUS_Z_ENDCAP_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_LTZ_MINUS_Z_ENDCAP_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_MAGNETOMETER_0_COMM_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_MAGNETOMETER_1_COMM_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_MAGNETOMETER_2_COMM_FAILURE,
	ADCS_OD_ERROR_INFO_CODE_MAGNETOMETER_3_COMM_FAILURE,
} adcs_od_error_info_code_t;

typedef struct {
	int32_t current_pwm_percent; //0-10000
	int32_t target_pwm_percent; //Negative values indicate the phase should be inverted

	float current_feedback_measurement_V; //Volts, Note: this is the average voltage while the PWM output is high.
	int32_t current_feedback_measurement_uA; //uA. Note: this is the average current flowing while the PWM output is high. It does not represent overall average current.

	uint8_t phase_gpio_pin_number;
	uint8_t pwm_channel_number;

	systime_t last_update_time;
} mt_pwm_phase_data_t;



void set_pwm_output(void) {
	if( PWMD1.state == PWM_STOP ) {
		pwmStart(&PWMD1, &pwmcfg_1_trigo);
#if 0
		chprintf(DEBUG_SD, "Turning on PWM output...\r\n");
		pwmEnableChannel(&PWMD1, MT_X_PWM_PWM_CHANNEL, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 1000));
		pwmEnableChannel(&PWMD1, MT_Y_PWM_PWM_CHANNEL, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 1000));
		pwmEnableChannel(&PWMD1, MT_Z_PWM_PWM_CHANNEL, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 1000));
#endif
	}

#if 1
	for(int i = 0; i < 3; i++ ) {
		const systime_t now_time = chVTGetSystemTime();

//		if( i == 1 ) {
//			int mod = (now_time / 40000) % 4;
//			switch (mod) {
//				case 0:
//					g_adcs_data.mt_pwm_data[i].target_pwm_percent =	map_current_uA_to_pwm_duty_cycle(500000, i);
//					break;
//				case 1:
//					g_adcs_data.mt_pwm_data[i].target_pwm_percent = 0;
//					break;
//				case 2:
//					g_adcs_data.mt_pwm_data[i].target_pwm_percent =	map_current_uA_to_pwm_duty_cycle(-500000, i);
//					break;
//				case 3:
//					g_adcs_data.mt_pwm_data[i].target_pwm_percent = 0;
//					break;
//			}
//		}

		//Updates will come in periodically via CANOpen, this will apply those updates to the PWM outputs.
		if( g_adcs_data.mt_pwm_data[i].last_update_time == 0 || chTimeDiffX(g_adcs_data.mt_pwm_data[i].last_update_time, now_time) > 10 ) {
			if( g_adcs_data.mt_pwm_data[i].current_pwm_percent != g_adcs_data.mt_pwm_data[i].target_pwm_percent ) {
				chprintf(DEBUG_SD, "target_pwm_percent = %d\r\n", g_adcs_data.mt_pwm_data[i].target_pwm_percent);

				pwmDisableChannel(&PWMD1, g_adcs_data.mt_pwm_data[i].pwm_channel_number);

				if( g_adcs_data.mt_pwm_data[i].target_pwm_percent < 0 ) {
					palSetPad(GPIOB, g_adcs_data.mt_pwm_data[i].phase_gpio_pin_number);
				} else {
					palClearPad(GPIOB, g_adcs_data.mt_pwm_data[i].phase_gpio_pin_number);
				}

				const int32_t pwm_val = ABS(g_adcs_data.mt_pwm_data[i].target_pwm_percent);
				pwmEnableChannel(&PWMD1, g_adcs_data.mt_pwm_data[i].pwm_channel_number, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, pwm_val));

				g_adcs_data.mt_pwm_data[i].current_pwm_percent = g_adcs_data.mt_pwm_data[i].target_pwm_percent;
				g_adcs_data.mt_pwm_data[i].last_update_time = now_time;
			}
		}
	}
#endif
}


int32_t current_feedback_convert_volts_to_microamps(const float volts) {
	//Based on the circuit design, this should nominally be 3V/amp.
	//This calculation seems to be within 5%-10% accurate when compared to in line bench DMM readings.
	float microamps = (volts / 3.0) * 1000000.0;

	return(microamps);
}

void print_debug_output(void) {
	static systime_t last_print_time = 0;
	systime_t now_time = TIME_I2MS(chVTGetSystemTime());
	if (chTimeDiffX(last_print_time, now_time) > 750) {
		last_print_time = now_time;

		chprintf(DEBUG_SD, "================\r\n");
		chprintf(DEBUG_SD, "CANOpen Data:\r\n");
		chprintf(DEBUG_SD, "  OD_RAM.x4000_gyroscope.pitch_rate = %d\r\n", OD_RAM.x4000_gyroscope.pitch_rate);
		chprintf(DEBUG_SD, "  OD_RAM.x4000_gyroscope.yaw_rate = %d\r\n", OD_RAM.x4000_gyroscope.yaw_rate);
		chprintf(DEBUG_SD, "  OD_RAM.x4000_gyroscope.roll_rate = %d\r\n", OD_RAM.x4000_gyroscope.roll_rate);
		chprintf(DEBUG_SD, "  OD_RAM.x4000_gyroscope.pitch_rate_raw = %d\r\n", OD_RAM.x4000_gyroscope.pitch_rate_raw);
		chprintf(DEBUG_SD, "  OD_RAM.x4000_gyroscope.yaw_rate_raw = %d\r\n", OD_RAM.x4000_gyroscope.yaw_rate_raw);
		chprintf(DEBUG_SD, "  OD_RAM.x4000_gyroscope.roll_rate_raw = %d\r\n", OD_RAM.x4000_gyroscope.roll_rate_raw);

		chprintf(DEBUG_SD, "  OD_RAM.x4001_accelerometer.x = %d\r\n", OD_RAM.x4001_accelerometer.x);
		chprintf(DEBUG_SD, "  OD_RAM.x4001_accelerometer.y = %d\r\n", OD_RAM.x4001_accelerometer.y);
		chprintf(DEBUG_SD, "  OD_RAM.x4001_accelerometer.z = %d\r\n", OD_RAM.x4001_accelerometer.z);
		chprintf(DEBUG_SD, "  OD_RAM.x4001_accelerometer.x_raw = %d\r\n", OD_RAM.x4001_accelerometer.x_raw);
		chprintf(DEBUG_SD, "  OD_RAM.x4001_accelerometer.y_raw = %d\r\n", OD_RAM.x4001_accelerometer.y_raw);
		chprintf(DEBUG_SD, "  OD_RAM.x4001_accelerometer.z_raw = %d\r\n", OD_RAM.x4001_accelerometer.z_raw);

		chprintf(DEBUG_SD, "  OD_RAM.x4002_temperature = %d\r\n", OD_RAM.x4002_temperature);

		chprintf(DEBUG_SD, "  OD_RAM.x4007_magnetorquer_current_x.current_set = %d\r\n", OD_RAM.x4007_magnetorquer.current_x_setpoint);
		chprintf(DEBUG_SD, "  OD_RAM.x4008_magnetorquer_current_y.current_set = %d\r\n", OD_RAM.x4007_magnetorquer.current_y_setpoint);
		chprintf(DEBUG_SD, "  OD_RAM.x4009_magnetorquer_current_z.current_set = %d\r\n", OD_RAM.x4007_magnetorquer.current_z_setpoint);

		chprintf(DEBUG_SD, "  OD_RAM.x4010_magnetorquer_pwm_percent.x = %d\r\n", OD_RAM.x4007_magnetorquer.pwm_x);
		chprintf(DEBUG_SD, "  OD_RAM.x4010_magnetorquer_pwm_percent.y = %d\r\n", OD_RAM.x4007_magnetorquer.pwm_y);
		chprintf(DEBUG_SD, "  OD_RAM.x4010_magnetorquer_pwm_percent.z = %d\r\n", OD_RAM.x4007_magnetorquer.pwm_z);

		chprintf(DEBUG_SD, "  OD_RAM.x4007_magnetorquer_current.x = %d\r\n", OD_RAM.x4007_magnetorquer.current_x);
		chprintf(DEBUG_SD, "  OD_RAM.x4007_magnetorquer_current.y = %d\r\n", OD_RAM.x4007_magnetorquer.current_y);
		chprintf(DEBUG_SD, "  OD_RAM.x4007_magnetorquer_current.z = %d\r\n", OD_RAM.x4007_magnetorquer.current_z);

		chprintf(DEBUG_SD, "  OD_RAM.x4003_pos_z_magnetometer_1.x = %d\r\n", OD_RAM.x4003_pos_z_magnetometer_1.x);
		chprintf(DEBUG_SD, "  OD_RAM.x4003_pos_z_magnetometer_1.y = %d\r\n", OD_RAM.x4003_pos_z_magnetometer_1.y);
		chprintf(DEBUG_SD, "  OD_RAM.x4003_pos_z_magnetometer_1.z = %d\r\n", OD_RAM.x4003_pos_z_magnetometer_1.z);

		chprintf(DEBUG_SD, "  OD_RAM.x4004_pos_z_magnetometer_2.x = %d\r\n", OD_RAM.x4004_pos_z_magnetometer_2.x);
		chprintf(DEBUG_SD, "  OD_RAM.x4004_pos_z_magnetometer_2.y = %d\r\n", OD_RAM.x4004_pos_z_magnetometer_2.y);
		chprintf(DEBUG_SD, "  OD_RAM.x4004_pos_z_magnetometer_2.z = %d\r\n", OD_RAM.x4004_pos_z_magnetometer_2.z);

		chprintf(DEBUG_SD, "  OD_RAM.x4005_min_z_magnetometer_1.x = %d\r\n", OD_RAM.x4005_min_z_magnetometer_1.x);
		chprintf(DEBUG_SD, "  OD_RAM.x4005_min_z_magnetometer_1.y = %d\r\n", OD_RAM.x4005_min_z_magnetometer_1.y);
		chprintf(DEBUG_SD, "  OD_RAM.x4005_min_z_magnetometer_1.z = %d\r\n", OD_RAM.x4005_min_z_magnetometer_1.z);

		chprintf(DEBUG_SD, "  OD_RAM.x4005_min_z_magnetometer_1.x = %d\r\n", OD_RAM.x4006_min_z_magnetometer_2.x);
		chprintf(DEBUG_SD, "  OD_RAM.x4005_min_z_magnetometer_1.y = %d\r\n", OD_RAM.x4006_min_z_magnetometer_2.y);
		chprintf(DEBUG_SD, "  OD_RAM.x4005_min_z_magnetometer_1.z = %d\r\n", OD_RAM.x4006_min_z_magnetometer_2.z);



		for(int i = 0; i < 3; i++ ) {
			mt_pwm_phase_data_t *data = &g_adcs_data.mt_pwm_data[i];
			chprintf(DEBUG_SD, "  measured_i_sense_voltage[%d] = %d uA, %u mV\r\n",
							i,
							data->current_feedback_measurement_uA,
							(uint32_t) (data->current_feedback_measurement_V * 1000));
		}


		chprintf(DEBUG_SD, "  CO_EM_GENERIC_ERROR:  %u\r\n", CO_isError(CO->em, CO_EM_GENERIC_ERROR));

	}
}


void process_magnetorquer(void) {
	chprintf(DEBUG_SD, "Entering process_magnetorquer()\r\n");
//	chprintf(DEBUG_SD, "ADCD1.state = %u\r\n", ADCD1.state);

	if( ADCD1.state == ADC_STOP || ADCD1.state == ADC_UNINIT ) {
		adcStart(&ADCD1, NULL);
	}

	if( ADCD1.state == ADC_COMPLETE ) {
		adcStopConversion(&ADCD1);
	}

	if( ADCD1.state == ADC_READY ) {
		chprintf(DEBUG_SD, "ADC ready, reading ADC values...\r\n");

		uint32_t channel_sums[MY_SAMPLING_NUMBER];
		memset(channel_sums, 0, sizeof(channel_sums));

		for(int s = 0; s < MY_SAMPLING_NUMBER; s++) {
//			chprintf(DEBUG_SD, "  adc_sample_buff[%d] = [", s);

			for (int adc_chan_idx = 1; adc_chan_idx < ADC_NUM_CHANNELS; adc_chan_idx++) {
				const int idx = (s * ADC_NUM_CHANNELS) + adc_chan_idx;
				channel_sums[adc_chan_idx] += adc_sample_buff[idx];
//				chprintf(DEBUG_SD, "%d, ", adc_sample_buff[idx]);
			}
//			chprintf(DEBUG_SD, "]\r\n");
		}

		for (int adc_chan_idx = 1; adc_chan_idx < ADC_NUM_CHANNELS; adc_chan_idx++) {
			const uint32_t channel_avg = channel_sums[adc_chan_idx] / MY_SAMPLING_NUMBER;
			measured_i_sense_voltage[adc_chan_idx] = (((float) channel_avg) / 4096.0) * 3.3;
//			chprintf(DEBUG_SD, "avg = %u, voltage[%d] = %u mV\r\n", channel_avg, adc_chan_idx, ((uint32_t) (measured_i_sense_voltage[adc_chan_idx] * 1000.0)));

			uint8_t dest_mt_idx = 0;
			if( adc_chan_idx == 1 ) {
				dest_mt_idx = 0;
			} else if( adc_chan_idx == 2 ) {
				dest_mt_idx = 1;
			} else if( adc_chan_idx == 3 ) {
				dest_mt_idx = 2;
			}

			g_adcs_data.mt_pwm_data[dest_mt_idx].current_feedback_measurement_V = measured_i_sense_voltage[adc_chan_idx];
			g_adcs_data.mt_pwm_data[dest_mt_idx].current_feedback_measurement_uA = current_feedback_convert_volts_to_microamps(measured_i_sense_voltage[adc_chan_idx]);
			if( g_adcs_data.mt_pwm_data[dest_mt_idx].current_pwm_percent < 0 ) {
				g_adcs_data.mt_pwm_data[dest_mt_idx].current_feedback_measurement_uA *= -1;
			}

		}

		chprintf(DEBUG_SD, "Staring conversion...\r\n");// chThdSleepMilliseconds(10);
		adcStartConversion(&ADCD1, &adcgrpcfg_tim1_trigo, adc_sample_buff, ADC_BUFF_SIZE);//Starts an ADC conversion.
	}

//	chprintf(DEBUG_SD, "adc_conv_cb#=%u, ", adc_conversion_complete_callback_count);
//	chprintf(DEBUG_SD, "adc_error_cb#=%u\r\n", adc_conversion_error_callback_count);

	chprintf(DEBUG_SD, "Setting magnetorquer PWM outputs...\r\n");
	set_pwm_output();
	print_debug_output();
	chprintf(DEBUG_SD, "Exiting process_magnetorquer()\r\n");
}


void init_magnetorquer(void) {
	memset(&g_adcs_data.mt_pwm_data, 0, sizeof(g_adcs_data.mt_pwm_data));

	g_adcs_data.mt_pwm_data[0].phase_gpio_pin_number = GPIOB_MT_X_PHASE;
	g_adcs_data.mt_pwm_data[0].pwm_channel_number = MT_X_PWM_PWM_CHANNEL;

	g_adcs_data.mt_pwm_data[1].phase_gpio_pin_number = GPIOB_MT_Y_PHASE;
	g_adcs_data.mt_pwm_data[1].pwm_channel_number = MT_Y_PWM_PWM_CHANNEL;

	g_adcs_data.mt_pwm_data[2].phase_gpio_pin_number = GPIOB_MT_Z_PHASE;
	g_adcs_data.mt_pwm_data[2].pwm_channel_number = MT_Z_PWM_PWM_CHANNEL;

	for(int i = 0; i < 3; i++ ) {
		palSetPad(GPIOB, g_adcs_data.mt_pwm_data[i].phase_gpio_pin_number);
	}



	palSetPadMode(GPIOA, GPIOA_MT_ILIM, PAL_MODE_INPUT_ANALOG);
	dacStart(&DACD1, &dac_config);
//	dacPutChannelX(&DACD1, 0, 650); //0.44V
	dacPutChannelX(&DACD1, 0, 3600); //3V

//	palSetPad(GPIOA, GPIOA_MT_STBY_RST);
//	palSetPad(GPIOB, GPIOB_MT_EN);



	palClearPad(GPIOB, GPIOB_MT_EN);
	palSetPad(GPIOA, GPIOA_MT_STBY_RST);

	palClearPad(GPIOB, GPIOB_MT_X_PWM);
	palClearPad(GPIOB, GPIOB_MT_Y_PWM);
	palClearPad(GPIOB, GPIOB_MT_Z_PWM);

	palClearPad(GPIOB, GPIOB_MT_X_PHASE);
	palClearPad(GPIOB, GPIOB_MT_Y_PHASE);
	palClearPad(GPIOB, GPIOB_MT_Z_PHASE);


	palClearPad(GPIOA, GPIOA_MT_STBY_RST);
	chThdSleepMilliseconds(5);
	palSetPad(GPIOB, GPIOB_MT_EN);
	chThdSleepMilliseconds(5);
	palSetPad(GPIOA, GPIOA_MT_STBY_RST);
	chThdSleepMilliseconds(5);
}

#endif
