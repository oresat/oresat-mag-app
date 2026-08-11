#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_CAN)
#include <canopennode.h>
#endif

#include "gpios.h"

LOG_MODULE_REGISTER(gpios, LOG_LEVEL_DBG);

/* === GPIO data === */
#define BP_NODE DT_NODELABEL(maggpios)

const struct gpio_dt_spec mt_en = GPIO_DT_SPEC_GET(BP_NODE, mt_en_gpios);
const struct gpio_dt_spec n_mt_en_fault = GPIO_DT_SPEC_GET(BP_NODE, n_mt_en_fault_gpios);
const struct gpio_dt_spec n_mt_stby_rst = GPIO_DT_SPEC_GET(BP_NODE, n_mt_stby_rst_gpios);
const struct gpio_dt_spec mt_x_phase = GPIO_DT_SPEC_GET(BP_NODE, mt_x_phase_gpios);
const struct gpio_dt_spec mt_y_phase = GPIO_DT_SPEC_GET(BP_NODE, mt_y_phase_gpios);
const struct gpio_dt_spec mt_z_phase = GPIO_DT_SPEC_GET(BP_NODE, mt_z_phase_gpios);
const struct gpio_dt_spec hw_rev_bit_0 = GPIO_DT_SPEC_GET(BP_NODE, hw_rev_bit_0_gpios);
const struct gpio_dt_spec hw_rev_bit_1 = GPIO_DT_SPEC_GET(BP_NODE, hw_rev_bit_1_gpios);
const struct gpio_dt_spec hw_rev_bit_2 = GPIO_DT_SPEC_GET(BP_NODE, hw_rev_bit_2_gpios);
const struct gpio_dt_spec tcan_nfault_oc = GPIO_DT_SPEC_GET(BP_NODE, tcan_nfault_oc_gpios);
const struct gpio_dt_spec tcan_silent = GPIO_DT_SPEC_GET(BP_NODE, tcan_silent_gpios);

static unsigned raw_board_rev;

/**************************************************/

int init_gpios(void)
{
	int ret;
	int err = 0;
	static bool initialized = false;

	if (initialized) {
		return 0;
	}

	initialized = true;

	err = gpio_pin_set_dt(&mt_en, false); // disable the STSPIN250s
	if (err) {
		LOG_ERR("Error setting mt_en low: %d", err);
		return err;
	}
	ret = gpio_pin_configure_dt(&n_mt_en_fault, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Could not configure n_mt_en_fault as input: %d", ret);
		err = ret;
	}
	ret = gpio_pin_configure_dt(&n_mt_stby_rst, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Could not configure n_mt_stby_rst as output inactive: %d", ret);
		err = ret;
	}
	// NOTE: the device tree should set the x/y/z_phase lines as active low, to fix logical-sense of the remainder of this code.
	ret = gpio_pin_configure_dt(&mt_x_phase, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Could not configure mt_x_phase as output inactive: %d", ret);
		err = ret;
	}
	ret = gpio_pin_configure_dt(&mt_y_phase, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Could not configure mt_y_phase as output inactive: %d", ret);
		err = ret;
	}
	ret = gpio_pin_configure_dt(&mt_z_phase, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		LOG_ERR("Could not configure mt_z_phase as output inactive: %d", ret);
		err = ret;
	}

	ret = gpio_pin_configure_dt(&hw_rev_bit_0, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		LOG_ERR("Could not configure hw_rev_bit_0 as input with pull up: %d", ret);
		err = ret;
	}
	ret = gpio_pin_configure_dt(&hw_rev_bit_1, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		LOG_ERR("Could not configure hw_rev_bit_1 as input with pull up: %d", ret);
		err = ret;
	}
	ret = gpio_pin_configure_dt(&hw_rev_bit_2, GPIO_INPUT | GPIO_PULL_UP);
	if (ret) {
		LOG_ERR("Could not configure hw_rev_bit_2 as input with pull up: %d", ret);
		err = ret;
	}

	raw_board_rev = gpio_pin_get_dt(&hw_rev_bit_0) << 0 |
				gpio_pin_get_dt(&hw_rev_bit_1) << 1 |
				gpio_pin_get_dt(&hw_rev_bit_2) << 2;

	LOG_INF("Board Rev %u", get_board_hw_rev());

	ret = gpio_pin_configure_dt(&tcan_nfault_oc, GPIO_INPUT | GPIO_PULL_UP); // goes high on fault; v1.1+ boards only
	if (ret) {
		LOG_ERR("Could not configure tcan_nfault_oc as input with pull up: %d", ret);
		err = ret;
	}

	ret = gpio_pin_configure_dt(&tcan_silent, GPIO_OUTPUT_INACTIVE); // put silent in disabled (normal) mode
	if (ret) {
		LOG_ERR("Could not configure tcan_silent as output inactive: %d", ret);
		err = ret;
	}

	return err;
}

board_hw_rev_t get_board_hw_rev(void)
{
	if (raw_board_rev == 7) { // no rev pins hooked up, so the pullups on the inputs make these all read as 1
		return HW_REV_1_0_0;
	}
	return (board_hw_rev_t)raw_board_rev;	// otherwise, believe the value
}

int set_can_silent(void)
{
	if (get_board_hw_rev() != HW_REV_1_0_0) {
		return gpio_pin_configure_dt(&tcan_silent, GPIO_OUTPUT_ACTIVE); // drive the pin high
	}
	return 0;
}

int set_can_normal(void)
{
	if (get_board_hw_rev() != HW_REV_1_0_0) {
		return gpio_pin_configure_dt(&tcan_silent, GPIO_OUTPUT_INACTIVE); // drive the pin low
	}
	return 0;
}

bool get_can_fault(void)
{
	if (get_board_hw_rev() != HW_REV_1_0_0) {
		return gpio_pin_get_dt(&tcan_nfault_oc) != 0; // high == fault
	}
	return false;
}

bool check_magnetorquer_fault(void)
{
	bool fault;

	fault = gpio_pin_get_dt(&n_mt_en_fault);
	if (!fault) {
		LOG_WRN("Fault on magnetorquer driver(s)!");

		// TODO: ask Andrew if this is ok to do. It wasn't in the old code.
		// LOG_INF("Resetting magnetorquer drivers.");
		// (void)reset_magnetorquer();
	}
	return !fault;
}

