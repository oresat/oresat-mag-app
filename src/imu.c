#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include <canopennode.h>
#include <CO_OD.h>

#include "windowed_average.h"
#include "imu.h"

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

#include "../drivers/sensor/tdk/icm4268x/icm4268x_reg.h"

#include "windowed_average.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu, CONFIG_SENSOR_LOG_LEVEL);

#define IMU_DEVICE_ADDR 0x68 // I2C 7 bit address
#define IMU_DEVICE_ADDR_ALT 0x69 // I2C 7 bit address

#define SOFT_RESET_RETRIES 10
#define SOFT_RESET_READY_TRIES 10
#define DATA_READY_TRIES 500
#define MAX_I2C_RECOVERY_RETRIES 10
#define MAX_RECOVERY_BOOTS 10

#define IMU_STARTUP_DELAY 500
#define IMU_ITERATION_PERIOD 90 // ms -- a bit more fast than the ODR -- will wait for interrupt (polled)
#define IMU_ODR 100 // Hz
#define DEBUG_PRINT_PERIOD  100 // 1s

// 15.625 degrees / second full scale range
#define GYRO_FULL_SCALE_RANGE_BIT BIT_GYRO_UI_FS_15_625
// in units of LSB / (degree/s) == (32767 / 15.625) = 2097 counts / degree / second
#define GYRO_OUT_SCALE (32767 / 15.625f)

static const struct device *i2c;
static uint8_t imu_addr;
static uint8_t prev_bank;
static bool imu_is_ready;

#define IMU_THREAD_STACK_SIZE 2048
#define IMU_THREAD_PRIORITY 0
extern const k_tid_t imu_id;

static int16_t gx = 0;
static int16_t gy = 0;
static int16_t gz = 0;
static int16_t gtemp = 0;

static wnd_avg_store x_hist = {.name = "x"};
static wnd_avg_store y_hist = {.name = "y"};
static wnd_avg_store z_hist = {.name = "z"};
static wnd_avg_store temp_hist = {.name = "t"};

static int check_i2c_device_presence(uint8_t addr)
{
	struct i2c_msg msgs[1];
	uint8_t dst;

	/* Send the address to read from */
	msgs[0].buf = &dst;
	msgs[0].len = 0U;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	if (i2c_transfer(i2c, &msgs[0], 1, addr) == 0) {
		LOG_DBG("Device detected on i2c bus at 0x%02x", addr);
		return 0;
	} else {
		LOG_DBG("Device NOT detected on i2c bus at 0x%02x", addr);
		return -ENODEV;
	}
}

static int imu_write_reg(uint16_t full_addr, uint8_t val)
{
	int ret;
	uint8_t bank = full_addr >> 8;
	uint8_t addr = full_addr & 0x0ff;

	if (bank != prev_bank) {
		ret = i2c_reg_write_byte(i2c, imu_addr, REG_BANK_SEL, bank);
		if (ret < 0) {
			LOG_ERR("Unable to write IMU bank register: %d", ret);
			return ret;
		}
		prev_bank = bank;
	}

	ret = i2c_reg_write_byte(i2c, imu_addr, addr, val);
	if (ret < 0) {
		LOG_ERR("Unable to write IMU register bank %u, addr 0x%02x, data 0x%02x: %d",
				bank, addr, val, ret);
	}
	return ret;
}

static int imu_read_reg(uint16_t full_addr, uint8_t *buf)
{
	int ret;
	uint8_t bank = full_addr >> 8;
	uint8_t addr = full_addr & 0x0ff;

	if (bank != prev_bank) {
		ret = i2c_reg_write_byte(i2c, imu_addr, REG_BANK_SEL, bank);
		if (ret < 0) {
			LOG_ERR("Unable to write IMU bank register: %d", ret);
			return ret;
		}
		prev_bank = bank;
	}

	ret = i2c_reg_read_byte(i2c, imu_addr, addr, buf);
	if (ret < 0) {
		LOG_ERR("Unable to read IMU register bank %u, addr 0x%02x: %d",
				bank, addr, ret);
	}
	return ret;
}

bool is_imu_ready(void)
{
	if (!imu_is_ready) {
		char state_str[80] = {0};
		k_thread_state_str(imu_id, state_str, sizeof(state_str) - 1);
		LOG_ERR("IMU thread state: %s", state_str);
	}
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

static int reset_imu(void)
{
	int ret;
	int attempt;
	uint8_t int_status;
	int rec_count;

	rec_count = load_recovery_count();

	for (attempt = 1; attempt < SOFT_RESET_RETRIES; attempt++) {
		ret = imu_write_reg(REG_DEVICE_CONFIG, BIT_SOFT_RESET_CONFIG);
		k_msleep(10 * attempt); /* must sleep > 1 ms before any other register access */
		if (ret < 0) {
			LOG_ERR("Attempt %d: error soft resetting IMU: %d", attempt++, ret);
			continue;
		} else {
			LOG_INF("Soft reset the IMU...");
		}
		for (int i = 0; i < SOFT_RESET_READY_TRIES; i++) {
			ret = imu_read_reg(REG_INT_STATUS, &int_status);
			if (ret < 0) {
				LOG_ERR("Error reading int status: %d", ret);
			} else {
				if (int_status & BIT_RESET_DONE_INT) {
					LOG_INF("Soft reset complete.");
					break;
				}
			}
			k_msleep(50 * (i + 1));
		}
		if (!ret) {
			if ((int_status & BIT_RESET_DONE_INT)) {
				LOG_INF("I2C bus recovery successful.");
				break;
			}
			// TODO: test this, then add sync to data ready int
		}
		LOG_INF("Recovering i2c bus");
		ret = i2c_recover_bus(DEVICE_DT_GET(DT_NODELABEL(flexcomm0_lpi2c0)));
		if (ret) {
			LOG_WRN("I2C bus is stuck (err: %d); recovery failed", ret);
		}
		k_msleep(50 * attempt); // increase the time each loop, just in case that helps recovery
	}

	if (ret < 0) {
		if (rec_count < MAX_RECOVERY_BOOTS) {
			store_recovery_count(rec_count + 1);
			settings_commit();
			k_msleep(500); // give settings time to be written to flash
			__ASSERT(ret < 0, "Giving up on soft reset of IMU. Rebooting.");
		} else {
			LOG_ERR("Cannot recover i2c bus after 10 reboot attempts. Giving up.");
		}
	} else {
		if (rec_count) {
			LOG_INF("Resetting recovery count. Recovery successful");
			store_recovery_count(0); // reset since we're good
		}
		LOG_INF("IMU has been reset.");
	}

	return ret;
}

static int init_imu(void)
{
	int ret;

	imu_is_ready = false;
	i2c = DEVICE_DT_GET(DT_NODELABEL(flexcomm0_lpi2c0)); //i2c-0));

	if (!device_is_ready(i2c)) {
		LOG_ERR("I2C bus is not ready");
			return -ENODEV;
	}

	if (check_i2c_device_presence(IMU_DEVICE_ADDR) == 0) {
		LOG_INF("Device detected on i2c bus at 0x%02x", IMU_DEVICE_ADDR);
		imu_addr = IMU_DEVICE_ADDR;
	} else {
		LOG_INF("Device NOT detected on i2c bus at 0x%02x", IMU_DEVICE_ADDR);
		if (check_i2c_device_presence(IMU_DEVICE_ADDR_ALT) == 0) {
			LOG_INF("Device detected on i2c bus at 0x%02x", IMU_DEVICE_ADDR_ALT);
			imu_addr = IMU_DEVICE_ADDR_ALT;
		} else {
			LOG_INF("Device NOT detected on i2c bus at 0x%02x", IMU_DEVICE_ADDR_ALT);
			LOG_ERR("No device found at either address");
			return -ENODEV;
		}
	}

	uint8_t buf = 0;

	ret = imu_read_reg(REG_WHO_AM_I, &buf);
	if (ret < 0) {
		LOG_ERR("Error reading IMU WHOAMI register: %d", ret);
		return ret;
	}

	if (buf != WHO_AM_I_ICM42688) {
		LOG_ERR("WHOAMI register returned 0x%02x, not 0x%02x", buf, WHO_AM_I_ICM42688);
		return -ENXIO;
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
	int ret;

	LOG_INF("Configuring IMU...");
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
	LOG_INF("Configuration complete.");
	return ret;
}

static int read_gyro_data(int16_t *x, int16_t *y, int16_t *z)
{
	int ret;
	uint8_t b0;
	uint8_t b1;

	ret = imu_read_reg(REG_GYRO_DATA_X0, &b0);
	if (ret < 0) {
		LOG_ERR("Error reading X0: %d", ret);
		return ret;
	}
	ret = imu_read_reg(REG_GYRO_DATA_X1, &b1);
	if (ret < 0) {
		LOG_ERR("Error reading X1: %d", ret);
		return ret;
	}
	*x = (uint16_t)b0 | (((uint16_t)b1) << 8);

	ret = imu_read_reg(REG_GYRO_DATA_Y0, &b0);
	if (ret < 0) {
		LOG_ERR("Error reading Y0: %d", ret);
		return ret;
	}
	ret = imu_read_reg(REG_GYRO_DATA_Y1, &b1);
	if (ret < 0) {
		LOG_ERR("Error reading Y1: %d", ret);
		return ret;
	}
	*y = (uint16_t)b0 | (((uint16_t)b1) << 8);

	ret = imu_read_reg(REG_GYRO_DATA_Z0, &b0);
	if (ret < 0) {
		LOG_ERR("Error reading Z0: %d", ret);
		return ret;
	}
	ret = imu_read_reg(REG_GYRO_DATA_Z1, &b1);
	if (ret < 0) {
		LOG_ERR("Error reading Z1: %d", ret);
		return ret;
	}
	*z = (uint16_t)b0 | (((uint16_t)b1) << 8);

	return ret;
}

static int read_temp_data(int16_t *temp)
{
	int ret;
	uint8_t b0;
	uint8_t b1;

	ret = imu_read_reg(REG_TEMP_DATA0, &b0);
	if (ret < 0) {
		LOG_ERR("Error reading TEMP0: %d", ret);
		return ret;
	}
	ret = imu_read_reg(REG_TEMP_DATA1, &b1);
	if (ret < 0) {
		LOG_ERR("Error reading TEMP1: %d", ret);
		return ret;
	}
	*temp = (uint16_t)b0 | (((uint16_t)b1) << 8);
	return ret;
}

static int process_data(int16_t *x, int16_t *y, int16_t *z, int16_t *temp)
{
	int err;
	int16_t new_x;
	int16_t new_y;
	int16_t new_z;
	int16_t new_temp;

	err = read_gyro_data(&new_x, &new_y, &new_z);
	if (err < 0) {
		return err;
	}

	*x = (int16_t)update_windowed_average(&x_hist, new_x);
	*y = (int16_t)update_windowed_average(&y_hist, new_y);
	*z = (int16_t)update_windowed_average(&z_hist, new_z);

	err = read_temp_data(&new_temp);
	if (err < 0) {
		return err;
	}

	*temp = (int16_t)update_windowed_average(&temp_hist, new_temp);

	return 0;
}

void get_gyro_data(int16_t *x, int16_t *y, int16_t *z, int16_t *temp)
{
	*x = (int16_t)(gx / GYRO_OUT_SCALE);  // convert to degrees / second (we lose a lot of resolution; mDPS might be better)
	*y = (int16_t)(gy / GYRO_OUT_SCALE);
	*z = (int16_t)(gz / GYRO_OUT_SCALE);
	*temp = gtemp;
}

static void handle_imu(void *p1, void *p2, void *p3)
{
	int err;
	int rec_count;

	k_thread_name_set(imu_id, "imu_thread");
	k_msleep(IMU_STARTUP_DELAY);

	LOG_INF("Starting IMU thread");

	err = settings_subsys_init();
	if (err) {
		LOG_ERR("settings subsys initialization: fail (err %d)", err);
	}

	rec_count = load_recovery_count();
	LOG_INF("  I2C recovery count: %d", rec_count);
	if (rec_count >= MAX_RECOVERY_BOOTS) {
		LOG_ERR("Unable to recover i2c bus after 10 resets. Will stop trying.");
	}

	err = init_imu();
	if (err < 0) {
		return;
	}

	err = configure_imu();
	if (err < 0) {
		return;
	}

	int count = 0;
	int i;
	uint8_t int_status;
	int16_t temp = 0;

	LOG_INF("Starting imu loop");
	for (;;) {
		for (i = 0; i < DATA_READY_TRIES; i++) {
			err = imu_read_reg(REG_INT_STATUS, &int_status);
			if (!err && (int_status & BIT_DATA_RDY_INT)) {
				break;
			}
			k_msleep(1);
		}
		if (err) {
			LOG_ERR("Timeout waiting for data ready");
		}
		err = process_data(&gx, &gy, &gz, &temp);

		// Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
		int16_t temp_decicentigrade = (int16_t)(((int)temp * 100) / (1325) + 250);

		gtemp = temp_decicentigrade / 10;

		if (!err) {
			count++;
			if (count >= (DEBUG_PRINT_PERIOD / IMU_ODR) ){
				count = 0;
				LOG_DBG("Ave gyro: (%d, %d, %d)", gx, gy, gz);
				LOG_DBG("Ave temp (dC): %d", temp_decicentigrade);
			}
		}
		k_msleep(IMU_ITERATION_PERIOD);
	}
}

K_THREAD_DEFINE(imu_id, IMU_THREAD_STACK_SIZE, handle_imu, NULL, NULL, NULL, IMU_THREAD_PRIORITY, 0, 0);

