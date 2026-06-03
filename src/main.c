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

LOG_MODULE_REGISTER(oresat_mcxn947_mag, LOG_LEVEL_DBG);

int main(void)
{
	LOG_INF("Oresat MCXN947 Mag Board App\n");
	return 0;
}
