#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>

#include "windowed_average.h"
#include "imu.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu_bmi, CONFIG_SENSOR_LOG_LEVEL);

/**
 * From the mag requirements doc, which reigns supreme not this
 * comment block:
 *
 * ICM-42688-P Gyroscope Care and Feeding
 *
 * Use GYRO_FS_SEL = 7 to get a full scale of  ±15.625 º/s
 * That should give us 2097.2 LSB/(º/s) sp that’s 476.8
 * microdps/bit
 *
 * Use ODR < 1kHz in order to get the 5 - 500 Hz bandwidth?
 *
 * Leave the gyro noise notch filter enabled (GYRO_NF_DIS = 0)
 * We probably want the lowest bandwidth through the notch
 * filter possible, which is  GYRO_NF_BW_SEL = 7 which gets us
 * to a bandwidth filter of 10 Hz.
 *
 * Follow the steps to program NF_COSWZ and NF_COSWZ_SEL.
 * 3dB Bandwidth of anti-aliasing filter (AAF) should be as low
 * as possible; 42 Hz GYRO_AAF_DELT = 1, GYRO_AAF_DELTSQR = 1,
 * GYRO_AAF_BITSHIFT = 15.
 *
 * Let’s use a Gyro ODR  of 100 Hz so GYRO_ODR = 0b1000, and
 * then do a running average down to 10 Hz (IIR style).
 *
 * Oh! Hey! We also want the temperature, too!
 *
 * Would be nice: the MCXN generates a 32.768 KHz clock signal
 * on INT2 and we use that as CLKIN. PIN9_FUNCTION should be
 * 0b10 then, otherwise it’s INT2 which is 0b00.
 */

#define IMU_DEVICE_ADDR 0x68 // I2C 7 bit address
#define IMU_DEVICE_ADDR_ALT 0x69 // I2C 7 bit address

#define SOFT_RESET_RETRIES 10
#define SOFT_RESET_READY_TRIES 10
#define DATA_READY_TRIES 500
#define MAX_I2C_RECOVERY_RETRIES 10
#define MAX_RECOVERY_BOOTS 10

#define IMU_STARTUP_DELAY 500
#define IMU_ITERATION_PERIOD 90 // ms -- a bit more fast than the ODR -- will wait for interrupt (polled)
#define IMU_ACCEL_RANGE_G 2
#define IMU_GYRO_RANGE_DPS 125
#define IMU_ODR 100 // Hz
#define DEBUG_PRINT_PERIOD  500 // 1s

#define RAW_ACCEL_OUT_SCALE 1
#define ACCEL_UNIT_SCALE 1

#define RAW_GYRO_OUT_SCALE 1
#define GYRO_UNIT_SCALE 1000.0f // milli-degrees/second

#if defined(CONFIG_SHELL) // calibration mode
#define HIST_LEN 100	// average more data for calibration purposes
#else
#define HIST_LEN 10
#endif

static int32_t ax_hist_buffer[HIST_LEN];
static wnd_avg_store ax_hist;
static int32_t ay_hist_buffer[HIST_LEN];
static wnd_avg_store ay_hist;
static int32_t az_hist_buffer[HIST_LEN];
static wnd_avg_store az_hist;

static int32_t gx_hist_buffer[HIST_LEN];
static wnd_avg_store gx_hist;
static int32_t gy_hist_buffer[HIST_LEN];
static wnd_avg_store gy_hist;
static int32_t gz_hist_buffer[HIST_LEN];
static wnd_avg_store gz_hist;

static int32_t t_hist_buffer[HIST_LEN];
static wnd_avg_store t_hist;

static bool imu_is_ready;

#define IMU_THREAD_STACK_SIZE 2048
#define IMU_THREAD_PRIORITY 0
extern const k_tid_t imu_id;

static int32_t a_cal[3]; // offset when unit is not accelerating
static int32_t a_raw[3];

static int32_t g_cal[3]; // offset when unit is not rotating
static int32_t g_raw[3];

static int32_t gtemp;

static bool reset_cal;

const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bmi270);

K_SEM_DEFINE(imu_data_ready, 0, 1);

bool is_imu_ready(void)
{
	return imu_is_ready;
}

static uint8_t load_recovery_count(void)
{
	int rc;
	uint8_t recovery_count = 0;

	settings_load();

	rc = settings_load_one("recovery_count", (void *)&recovery_count, sizeof(recovery_count));
	if (rc < 0) {
		LOG_ERR("Error loading recovery_count from settings: %d", rc);
	}
	if (!recovery_count) {
		recovery_count = 0;
	}
	return recovery_count;
}

static int store_recovery_count(uint8_t recovery_count)
{
	int rc;

	rc = settings_save_one("recovery_count", &recovery_count, sizeof(recovery_count));
	if (rc < 0) {
		LOG_ERR("Error saving recovery_count to settings: %d", rc);
	}
	return rc;
}

static void load_gyro_calibration(int32_t *gcal)
{
	int rc1;
	int rc2;
	int rc3;

	settings_load();

	rc1 = settings_load_one("gx_cal", (void *)&gcal[0], sizeof(*gcal));
	rc2 = settings_load_one("gy_cal", (void *)&gcal[1], sizeof(*gcal));
	rc3 = settings_load_one("gz_cal", (void *)&gcal[2], sizeof(*gcal));

	if (rc1 || rc2 || rc3) {
		LOG_WRN("No gyroscope calibration found in settings. Rebuild with shell and run gyrocal.");
	}
}

#if defined(CONFIG_SHELL)
static int store_gyro_calibration(int32_t *gcal)
{
	int rc;

	rc = settings_save_one("gx_cal", (void *)&gcal[0], sizeof(*gcal));
	if (rc < 0) {
		LOG_WRN("Error saving gx_cal from settings: %d", rc);
	}
	rc = settings_save_one("gy_cal", (void *)&gcal[1], sizeof(*gcal));
	if (rc < 0) {
		LOG_WRN("Error saving gy_cal from settings: %d", rc);
	}
	rc = settings_save_one("gz_cal", (void *)&gcal[2], sizeof(*gcal));
	if (rc < 0) {
		LOG_WRN("Error saving gz_cal from settings: %d", rc);
	}

	return rc;
}
#endif

static int recover_i2c_bus(void)
{
    int ret;
    int attempt;
	int rec_count = 0;

    ret = settings_subsys_init();
    if (ret) {
        LOG_ERR("settings subsys initialization: fail (err %d)", ret);
    }

    rec_count = load_recovery_count();
    LOG_INF("  I2C recovery count: %d", rec_count);
    if (rec_count >= MAX_RECOVERY_BOOTS) {
        LOG_ERR("Unable to recover i2c bus after 10 resets. Will stop trying.");
    }

	// We use deferred-init in the device tree, which means we need to manually start the driver
	ret = device_init(dev);
	if (ret) {
		LOG_WRN("Device not started: %d", ret);
	}

	if (device_is_ready(dev)) {
        if (rec_count) {
            LOG_INF("Resetting recovery count. Recovery successful");
            store_recovery_count(0); // reset since we're good
        }
        return 0;
    }

    for (attempt = 1; attempt < MAX_I2C_RECOVERY_RETRIES; attempt++) {
        ret = i2c_recover_bus(DEVICE_DT_GET(DT_NODELABEL(flexcomm0_lpi2c0)));
		if (ret == -ENOSYS) {
			LOG_WRN("I2C bus recovery is not implemented. Giving up.");
			ret = 0;
			break;
		} else if (ret) {
            LOG_WRN("I2C bus is stuck (err: %d); recovery failed", ret);
        } else { // do something to verify that it is actually working
			LOG_INF("Bus recovered.");
            if (device_is_ready(dev)) {
                break;
            }
			k_msleep(100);
			ret = device_init(dev);
            if (!ret) {
                LOG_INF("I2C bus recovery successful.");
                break;
            } else {
				LOG_ERR("Error starting BMI270 driver: %d", ret);
            }
        }
        k_sleep(K_MSEC(10 * attempt));
    }

    if (ret < 0) {
        if (rec_count < MAX_RECOVERY_BOOTS) {
            store_recovery_count(rec_count + 1);
            settings_commit();
            k_sleep(K_MSEC(500)); // give settings time to be written to flash
            __ASSERT(ret < 0, "Giving up on soft reset of IMU. Rebooting.");
        } else {
            LOG_ERR("Cannot recover i2c bus after 10 reboot attempts. Giving up.");
        }
    } else if (rec_count) {
        LOG_INF("Resetting recovery count. Recovery successful");
        store_recovery_count(0); // reset since we're good
    }

    return ret;
}

static int reset_imu(void)
{
	return 0;
}

static int init_imu(void)
{
	int ret;

	ret = init_windowed_average(&ax_hist, ax_hist_buffer, HIST_LEN, "ax");
	if (ret) {
		return ret;
	}
	ret = init_windowed_average(&ay_hist, ay_hist_buffer, HIST_LEN, "ay");
	if (ret) {
		return ret;
	}
	ret = init_windowed_average(&az_hist, az_hist_buffer, HIST_LEN, "az");
	if (ret) {
		return ret;
	}
	ret = init_windowed_average(&gx_hist, gx_hist_buffer, HIST_LEN, "gx");
	if (ret) {
		return ret;
	}
	ret = init_windowed_average(&gy_hist, gy_hist_buffer, HIST_LEN, "gy");
	if (ret) {
		return ret;
	}
	ret = init_windowed_average(&gz_hist, gz_hist_buffer, HIST_LEN, "gz");
	if (ret) {
		return ret;
	}
	ret = init_windowed_average(&t_hist, t_hist_buffer, HIST_LEN, "t");
	if (ret) {
		return ret;
	}

	imu_is_ready = false;

	if (!device_is_ready(dev)) {
		LOG_ERR("IMU %s is not ready", dev->name);
		return -ENODEV;
	}

	LOG_INF("IMU detected.");

	imu_is_ready = true;
	return reset_imu();
}

/**
 * From the mag requirements doc, which reigns supreme not this
 * comment block:
 *
 * ICM-42688-P Gyroscope Care and Feeding
 *
 * Use GYRO_FS_SEL = 7 to get a full scale of  ±15.625 º/s
 * That should give us 2097.2 LSB/(º/s) sp that’s 476.8
 * microdps/bit
 *
 * Use ODR < 1kHz in order to get the 5 - 500 Hz bandwidth?
 *
 * Leave the gyro noise notch filter enabled (GYRO_NF_DIS = 0)
 * We probably want the lowest bandwidth through the notch
 * filter possible, which is  GYRO_NF_BW_SEL = 7 which gets us
 * to a bandwidth filter of 10 Hz.
 *
 * Follow the steps to program NF_COSWZ and NF_COSWZ_SEL.
 *
 * fdesired is the desired frequency of the Notch Filter in kHz.
 * The lower bound for fdesired is 1kHz, and the upper bound is
 * 3kHz. Operating the notch filter outside this range is not
 *   supported.
 *
 * Step1:
 *   COSWZ = cos(2*pi*fdesired/32)
 *
 * Step2:
 *   If abs(COSWZ)≤0.875
 *     NF_COSWZ = round[COSWZ*256]
 *     NF_COSWZ_SEL = 0
 *   else
 *     NF_COSWZ_SEL = 1
 *     if COSWZ > 0.875
 *  	 NF_COSWZ = round[8*(1-COSWZ)*256]
 *     else if COSWZ < 0.875
 *  	 NF_COSWZ = round[-8*(1+COSWZ)*256]
 *     end
 *   End
 *
 * See: scripts/calc_coswz.py
 * output:
 * For fdesired:1KHz --> set nf_coswz_sel:1, nf_coswz:39.352
 *
 * 3dB Bandwidth of anti-aliasing filter (AAF) should be as low
 * as possible; 42 Hz GYRO_AAF_DELT = 1, GYRO_AAF_DELTSQR = 1,
 * GYRO_AAF_BITSHIFT = 15.
 *
 * Let’s use a Gyro ODR  of 100 Hz so GYRO_ODR = 0b1000, and
 * then do a running average down to 10 Hz (IIR style).
 *
 * Oh! Hey! We also want the temperature, too!
 *
 * Would be nice: the MCXN generates a 32.768 KHz clock signal
 * on INT2 and we use that as CLKIN. PIN9_FUNCTION should be
 * 0b10 then, otherwise it’s INT2 which is 0b00.
 */
static int configure_imu(void)
{
	int ret = 0;

	LOG_INF("Configuring IMU...");

#if 1
	struct sensor_value full_scale;
	struct sensor_value sampling_freq;
	struct sensor_value oversampling;

	/* Setting scale in g, due to loss of precision if the SI unit m/s^2
	 * is used
	 */
	full_scale.val1 = IMU_ACCEL_RANGE_G;	/* G */
	full_scale.val2 = 0;
	sampling_freq.val1 = IMU_ODR;			/* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1;					/* Normal mode */
	oversampling.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,
			&oversampling);
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change to 0.0Hz before changing
	 * other attributes
	 */
	sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sampling_freq);

	/* Setting scale in degrees/s to match the sensor scale */
	full_scale.val1 = IMU_GYRO_RANGE_DPS;	/* dps */
	full_scale.val2 = 0;
	sampling_freq.val1 = IMU_ODR;			/* Hz. Performance mode */
	sampling_freq.val2 = 0;
	oversampling.val1 = 1;					/* Normal mode */
	oversampling.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE,
			&full_scale);
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING,
			&oversampling);
	/* Set sampling frequency last as this also sets the appropriate
	 * power mode. If already sampling, change sampling frequency to
	 * 0.0Hz before changing other attributes
	 */
	sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&sampling_freq);

#else
	/* Enable gyro in low noise mode */
	ret = imu_write_reg(REG_PWR_MGMT0, (BIT_GYRO_MODE_LNM << 2));
	k_msleep(1); /* must sleep > 200 us before any other register access */
	if (ret < 0) {
		LOG_ERR("Error setting Power Mgmt on IMU: %d", ret);
		return ret;
	}

	/* Set gyro full scale range and output data rate */
	ret = imu_write_reg(REG_GYRO_CONFIG0,
						(GYRO_FULL_SCALE_RANGE_BIT << 5) | BIT_GYRO_ODR_100);
	k_msleep(1); /* must sleep > 200 us before any other register access */
	if (ret < 0) {
		LOG_ERR("Error setting Gyro Config on IMU: %d", ret);
		return ret;
	}

	/* Set gyro bandwidth notch filter to 10Hz */
	/* This is a bank1 register */
	ret = imu_write_reg(REG_GYRO_CONFIG_STATIC10,
						7 << 4); // GYRO_NF_BW_SEL = 10Hz
	if (ret < 0) {
		LOG_ERR("Error setting gyro bandwitch notch filter: %d", ret);
		return ret;
	}

	/* Set gyro notch filter nf_coswz_sel = 1, nf_coswz = 39
	   for each axis X, Y, Z; nf_coswz is split across two regs; bits 0-7
	   in one and bit 8 in another */
	uint8_t nf_coswz = 39;
	uint8_t nf_coswz_bits07 = nf_coswz & 0xff;
	uint8_t nf_coswz_bit8 = (nf_coswz >> 8) & 1;
	uint8_t nf_coswz_sel = 1;
	uint8_t val = (nf_coswz_sel << 5) | (nf_coswz_sel << 4) |
		(nf_coswz_sel << 3) | (nf_coswz_bit8 << 2) |
		(nf_coswz_bit8 << 1) | (nf_coswz_bit8 << 0);

	LOG_DBG("nf_coswz:%u, sel:%u, bits07:0x%02x, bit8:0x%02x, val:0x%02x",
			nf_coswz, nf_coswz_sel, nf_coswz_bits07, nf_coswz_bit8, val);
	ret = imu_write_reg(REG_GYRO_CONFIG_STATIC6, nf_coswz_bits07);
	if (ret < 0) {
		LOG_ERR("Error setting x axis coswz bits07: %d", ret);
		return ret;
	}
	ret = imu_write_reg(REG_GYRO_CONFIG_STATIC7, nf_coswz_bits07);
	if (ret < 0) {
		LOG_ERR("Error setting y axis coswz bits07: %d", ret);
		return ret;
	}
	ret = imu_write_reg(REG_GYRO_CONFIG_STATIC8, nf_coswz_bits07);
	if (ret < 0) {
		LOG_ERR("Error setting z axis coswz bits07: %d", ret);
		return ret;
	}
	ret = imu_write_reg(REG_GYRO_CONFIG_STATIC8, val);
	if (ret < 0) {
		LOG_ERR("Error setting axes x,y,z coswz_sel and bit8: %d", ret);
		return ret;
	}

	/** Set anti-alias filter
	 * 3dB Bandwidth of anti-aliasing filter (AAF) should be as low
	 * as possible; 42 Hz GYRO_AAF_DELT = 1, GYRO_AAF_DELTSQR = 1,
	 * GYRO_AAF_BITSHIFT = 15.
	 */
	ret = imu_write_reg(REG_GYRO_CONFIG_STATIC3, 1);
	if (ret < 0) {
		LOG_ERR("Error setting gyro aaf: %d", ret);
		return ret;
	}
	ret = imu_write_reg(REG_GYRO_CONFIG_STATIC4, 1);
	if (ret < 0) {
		LOG_ERR("Error setting gyro aaf: %d", ret);
		return ret;
	}
	ret = imu_write_reg(REG_GYRO_CONFIG_STATIC5, 15 << 4);
	if (ret < 0) {
		LOG_ERR("Error setting gyro aaf: %d", ret);
		return ret;
	}
#endif
	LOG_INF("Configuration complete.");
	return ret;
}

/**
 *  @brief convert accel value to gs
 *
 *  @param val_mps2 - in: driver value in meters / sec^2
 *  @param range - configured range in gs
 *  @retval int32_t - output value
 */
static int32_t channel_accel_convert(struct sensor_value *val_mps2,
									 uint8_t range)
{
	/* 16 bit accelerometer. 2^15 bits represent the range in g */
	/* Converting to g from m/s^2 */
	int64_t out_val;

	out_val = val_mps2->val1 * 1000000LL + val_mps2->val2;

	// driver does this:
	// out_val = (out_val * SENSOR_G * (int64_t) range) / INT16_MAX;
	out_val = (out_val * INT16_MAX) / (SENSOR_G * (int64_t)range);

	return out_val;
}

/**
 *  @brief convert gyro value to degrees / sec
 *
 *  @param val_radians - in: driver value in radians / sec
 *  @param range - configured range in degrees / sec
 *  @retval int32_t - output value
 */
static int32_t channel_gyro_convert(struct sensor_value *val_rps,
									uint16_t range)
{
	/* 16 bit gyroscope. 2^15 bits represent the range in degrees/s */
	/* Converting to degrees/s from radians/s */
	int64_t out_val;

	out_val = val_rps->val1 * 1000000LL + val_rps->val2;

	// driver does this:
	// out_val = ((out_val * (int64_t) range * SENSOR_PI) / (180LL * INT16_MAX));
	out_val = (out_val * 180LL * INT16_MAX) / (SENSOR_PI * (int64_t)range);

	return out_val;
}

static int read_accel_data(int32_t *aval)
{
	struct sensor_value accel[3];
	int ret;

	ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (!ret) {
		aval[0] = channel_accel_convert(&accel[0], IMU_ACCEL_RANGE_G);
		aval[1] = channel_accel_convert(&accel[1], IMU_ACCEL_RANGE_G);
		aval[2] = channel_accel_convert(&accel[2], IMU_ACCEL_RANGE_G);
	}
	return ret;
}

static int read_gyro_data(int32_t *gval)
{
	struct sensor_value gyro[3];
	int ret;

	ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (!ret) {
		gval[0] = channel_gyro_convert(&gyro[0], IMU_GYRO_RANGE_DPS);
		gval[1] = channel_gyro_convert(&gyro[1], IMU_GYRO_RANGE_DPS);
		gval[2] = channel_gyro_convert(&gyro[2], IMU_GYRO_RANGE_DPS);
	}
	return ret;
}

static int read_temp_data(int32_t *temp)
{
	int ret = 0;

	*temp = 20;
	// The bmi270 driver does not currently support the built-in temperature sensor
	return ret;
}

static int process_data(int32_t *a, int32_t *g, int32_t *acal, int32_t *gcal, int32_t *temp)
{
	int err;
	int32_t accel[3] = {0};
	int32_t gyro[3] = {0};
	int32_t new_temp;

	sensor_sample_fetch(dev);

	err = read_accel_data(accel);
	if (err < 0) {
		return err;
	}
	a[0] = (int32_t)update_windowed_average(&ax_hist, accel[0]) - acal[0]; // remove static offsets found during calibration
	a[1] = (int32_t)update_windowed_average(&ay_hist, accel[1]) - acal[1];
	a[2] = (int32_t)update_windowed_average(&az_hist, accel[2]) - acal[2];

	err = read_gyro_data(gyro);
	if (err < 0) {
		return err;
	}
	g[0] = (int32_t)update_windowed_average(&gx_hist, gyro[0]) - gcal[0]; // remove static offsets found during calibration
	g[1] = (int32_t)update_windowed_average(&gy_hist, gyro[1]) - gcal[1];
	g[2] = (int32_t)update_windowed_average(&gz_hist, gyro[2]) - gcal[2];

	err = read_temp_data(&new_temp);
	if (err < 0) {
		return err;
	}

	*temp = (int32_t)update_windowed_average(&t_hist, new_temp);

	return 0;
}

// external interface
void get_accel_data(int16_t *x, int16_t *y, int16_t *z)
{
	*x = (int16_t)(ACCEL_UNIT_SCALE * a_raw[0] / RAW_ACCEL_OUT_SCALE);  // convert to 0.001 degrees / second (milli-degrees per second)
	*y = (int16_t)(ACCEL_UNIT_SCALE * a_raw[1] / RAW_ACCEL_OUT_SCALE);
	*z = (int16_t)(ACCEL_UNIT_SCALE * a_raw[2] / RAW_ACCEL_OUT_SCALE);
}

// external interface
void get_gyro_data(int16_t *x, int16_t *y, int16_t *z, int16_t *temp)
{
	*x = (int16_t)(GYRO_UNIT_SCALE * g_raw[0] / RAW_GYRO_OUT_SCALE);  // convert to 0.001 degrees / second (milli-degrees per second)
	*y = (int16_t)(GYRO_UNIT_SCALE * g_raw[1] / RAW_GYRO_OUT_SCALE);
	*z = (int16_t)(GYRO_UNIT_SCALE * g_raw[2] / RAW_GYRO_OUT_SCALE);
	*temp = gtemp;
}

static void handle_imu(void *p1, void *p2, void *p3)
{
	int err;
	int rec_count;

	k_thread_name_set(imu_id, "imu_thread");
	k_msleep(IMU_STARTUP_DELAY);

	LOG_INF("Starting IMU thread");

	err = recover_i2c_bus();
	if (err) {
        LOG_ERR("Unable to recover bus. Continuing, but expect issues.");
	}

	rec_count = load_recovery_count();
	LOG_INF("  I2C recovery count: %d", rec_count);
	if (rec_count >= MAX_RECOVERY_BOOTS) {
		LOG_ERR("Unable to recover i2c bus after 10 resets. Will stop trying.");
	}

	load_gyro_calibration(g_cal);
	LOG_INF("Gyro calibration: (%d, %d, %d)", g_cal[0], g_cal[1], g_cal[2]);

	err = init_imu();
	if (err < 0) {
		return;
	}

	err = configure_imu();
	if (err < 0) {
		return;
	}

	int count = 0;
	int samples = 0;
	int32_t temp = 0;
	k_msleep(IMU_STARTUP_DELAY);

	LOG_INF("Starting imu loop");

	for (;;) {
		if (reset_cal) {
			g_cal[0] = 0;
			g_cal[1] = 0;
			g_cal[2] = 0;
			reset_windowed_average(&gx_hist);
			reset_windowed_average(&gy_hist);
			reset_windowed_average(&gz_hist);
			reset_cal = false;
			samples = 0;
		}
		err = process_data(a_raw, g_raw, a_cal, g_cal, &temp);
		if (!err) {
			samples++;
			if (samples > HIST_LEN) {
				samples = 0;
				k_sem_give(&imu_data_ready);
			}
		}

		// Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
		int32_t temp_decicentigrade = (int32_t)(((int)temp * 100) / (1325) + 250);

		gtemp = temp_decicentigrade / 10;

		if (!err) {
			count++;
			if (count >= (DEBUG_PRINT_PERIOD / IMU_ODR) ){
				count = 0;
				LOG_DBG("Accl %d, %d, %d", a_raw[0], a_raw[1], a_raw[2]);
				LOG_DBG("Gyro %d, %d, %d", g_raw[0], g_raw[1], g_raw[2]);
				LOG_DBG("Temp %d", temp_decicentigrade);
			}
		}
		k_msleep(IMU_ITERATION_PERIOD);
	}
}

#if defined(CONFIG_SHELL)

static int cmd_gyrocal(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	if (argc >= 2) {
		if (strncmp(argv[1], "reset", 5) == 0) {
			shell_print(sh, "Resetting gyroscope calibration to 0.");
			k_sem_take(&imu_data_ready, K_SECONDS(10));
			reset_cal = true;
		} else {
			shell_error(sh, "Unknown option: %s", argv[1]);
			return 0;
		}
	}

	shell_print(sh, "Calibrating gyroscope...");

	err = k_sem_take(&imu_data_ready, K_SECONDS(10));
	if (err) {
		shell_error(sh, "Error waiting for data: %d", err);
	} else {
		g_cal[0] = g_raw[0];
		g_cal[1] = g_raw[1];
		g_cal[2] = g_raw[2];
		err = store_gyro_calibration(g_cal);
		shell_print(sh, "New calibration: (%d, %d, %d)", g_cal[0], g_cal[1], g_cal[2]);
		if (err) {
			shell_error(sh, "Unable to store calibration: %d", err);
		}
		shell_print(sh, "Done.");
	}

	return 0;
}

SHELL_CMD_ARG_REGISTER(gyrocal, NULL,  SHELL_HELP("Calibrate the gyroscope", "gyrocal [<reset>]"), cmd_gyrocal, 1, 1);
#endif

K_THREAD_DEFINE(imu_id, IMU_THREAD_STACK_SIZE, handle_imu, NULL, NULL, NULL, IMU_THREAD_PRIORITY, 0, 0);
