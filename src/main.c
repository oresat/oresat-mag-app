/*
 * main.c
 *
 * Simple main that only logs a bootup message. The remainder
 * of the functionality is implemented as independent threads in
 * blink.c and others.
 *
 * These can be disabled at compile time by adding:
 *   CONFIG_BLINK=n
 * for example, to prj.conf. See Kconfig for the options or run
 * west build -t menuconfig for an interacive configuration
 * editor.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <version.h>
#include <app_version.h>

LOG_MODULE_REGISTER(oresat_mcxn947_mag, LOG_LEVEL_DBG);

int main(void)
{
	LOG_INF("\nOresat MCXN947 Mag Board App");
	LOG_INF("   Oresat   Board: %s", CONFIG_BOARD_TARGET);
	LOG_INF("   App    Version: %s", APP_VERSION_STRING);
	LOG_INF("   Zephyr Version: %s", KERNEL_VERSION_STRING);

	return 0;
}
