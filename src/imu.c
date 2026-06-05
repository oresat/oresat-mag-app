#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu, CONFIG_LOG_DEFAULT_LEVEL);

#define IMU_DEVICE_ADDR     0x68 // I2C 7 bit address
#define IMU_DEVICE_ADDR_ALT 0x69 // I2C 7 bit address

#define SOFT_RESET_RETRIES 10
#define HIST_SIZE 10

static const struct device *i2c;
static uint8_t imu_addr;
static uint8_t prev_bank;

#define IMU_THREAD_STACK_SIZE 2048
#define IMU_THREAD_PRIORITY 0
extern const k_tid_t imu_id;

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

static int init_imu(void)
{
	int ret;

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

	if (buf == WHO_AM_I_ICM42688) {
		LOG_INF("IMU detected.");
		return 0;
	} else {
		LOG_ERR("WHOAMI register returned 0x%02x, not 0x%02x", buf, WHO_AM_I_ICM42688);
		return -ENXIO;
	}
}

static int reset_imu(void)
{
	int ret;
	int attempt;

	for (attempt = 1; attempt < SOFT_RESET_RETRIES; attempt++) {
		ret = imu_write_reg(REG_DEVICE_CONFIG, BIT_SOFT_RESET_CONFIG);
		k_msleep(1); /* must sleep 1ms before any other register access */
		if (ret < 0) {
			LOG_ERR("Attempt %d: error soft resetting IMU: %d", attempt++, ret);
		} else {
			LOG_INF("Soft reset the IMU");
			break;
		}
	}

	if (ret < 0) {
		LOG_ERR("Giving up on soft reset of IMU.");
	}

	return ret;
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
 *       NF_COSWZ = round[8*(1-COSWZ)*256]
 *     else if COSWZ < 0.875
 *       NF_COSWZ = round[-8*(1+COSWZ)*256]
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
						(BIT_GYRO_UI_FS_15_625 << 5) | BIT_GYRO_ODR_100);
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
		LOG_ERR("Error reading X0: %d", ret);
		return ret;
	}
	*x = b0 | ((uint16_t)b1 << 8);

	ret = imu_read_reg(REG_GYRO_DATA_Y0, &b0);
	if (ret < 0) {
		LOG_ERR("Error reading X0: %d", ret);
		return ret;
	}
	ret = imu_read_reg(REG_GYRO_DATA_Y1, &b1);
	if (ret < 0) {
		LOG_ERR("Error reading X0: %d", ret);
		return ret;
	}
	*y = b0 | ((uint16_t)b1 << 8);

	ret = imu_read_reg(REG_GYRO_DATA_Z0, &b0);
	if (ret < 0) {
		LOG_ERR("Error reading X0: %d", ret);
		return ret;
	}
	ret = imu_read_reg(REG_GYRO_DATA_Z1, &b1);
	if (ret < 0) {
		LOG_ERR("Error reading X0: %d", ret);
		return ret;
	}
	*z = b0 | ((uint16_t)b1 << 8);

	return ret;
}


static int process_gyro_data(int16_t *x, int16_t *y, int16_t *z)
{
	static int hist_depth = 0;
	static int16_t x_hist[HIST_SIZE + 1] = {0};
	static int16_t y_hist[HIST_SIZE + 1] = {0};
	static int16_t z_hist[HIST_SIZE + 1] = {0};
	int err;
	int i;

	// throw out oldest sample
	for (i = MIN(HIST_SIZE - 1, hist_depth - 1); i > 0; i--) {
		x_hist[i + 1] = x_hist[i];
		y_hist[i + 1] = y_hist[i];
		z_hist[i + 1] = z_hist[i];
	}

	err = read_gyro_data(&x_hist[0], &y_hist[0], &z_hist[0]);
	if (err < 0) {
		return err;
	}

	//LOG_DBG("Raw gyro: (%d, %d, %d)", x_hist[0], y_hist[0], z_hist[0]);

	hist_depth++;
	if (hist_depth > HIST_SIZE) {
		hist_depth = HIST_SIZE;
	}

	int32_t sum_x = 0;
	int32_t sum_y = 0;
	int32_t sum_z = 0;

	for (i = 0; i < hist_depth; i++) {
		sum_x += x_hist[i];
		sum_y += y_hist[i];
		sum_z += z_hist[i];
	}
	*x = (sum_x / hist_depth);
	*y = (sum_y / hist_depth);
	*z = (sum_z / hist_depth);

	return 0;
}

static void handle_imu(void *p1, void *p2, void *p3)
{
	int err;

	k_thread_name_set(imu_id, "imu_thread");

	LOG_INF("Starting IMU thread");

	err = init_imu();
	if (err < 0) {
		return;
	}

	err = configure_imu();
	if (err < 0) {
		return;
	}

	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
	int count = 0;

	for (;;) {
		err = process_gyro_data(&x, &y, &z);
		if (!err) {
			count++;
			if (count >= HIST_SIZE) {
				count = 0;
				LOG_INF("Ave gyro: (%d, %d, %d)", x, y, z);
			}
		}
		k_sleep(K_MSEC(10));
	}
}

K_THREAD_DEFINE(imu_id, IMU_THREAD_STACK_SIZE, handle_imu, NULL, NULL, NULL, IMU_THREAD_PRIORITY, 0, 0);

