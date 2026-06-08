/**
 * pwm.c
 *
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h> 
#include <zephyr/sys/__assert.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/dsp/print_format.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(oresat_pwm, LOG_LEVEL_DBG);

/* PWM node from the devicetree. */
#define PWM_NODE_0 DT_ALIAS(pwm0)
#define PWM_NODE_1 DT_ALIAS(pwm1)

typedef struct pwm_info {
	const struct device *dev;
	int channel;
} pwm_info;

static pwm_info pwm_table[] = {
	{DEVICE_DT_GET(PWM_NODE_0), 0}, // pwm_num = 0
	{DEVICE_DT_GET(PWM_NODE_0), 1}, // pwm_num = 1
	{DEVICE_DT_GET(PWM_NODE_1), 0}  // pwm_num = 2
};
#define NUM_PWMS ARRAY_SIZE(pwm_table)

int init_pwm(void)
{
	pwm_flags_t flags = 0;
	int err;
	pwm_info *p;
	unsigned int i;

	LOG_INF("Initializing PWMs");

	for (i = 0; i < NUM_PWMS; i++) {
		p = &pwm_table[i];
		if (!device_is_ready(p->dev)) {
			LOG_ERR("PWM %s is not ready", p->dev->name);
			return -ENODEV;
		}

		err = pwm_set_cycles(p->dev, p->channel, 0, 0, flags);
		if (err) {
			LOG_ERR("Error setting pwm_num %u, dev %s, ch %u cycles: %d", i, p->dev->name, p->channel, err);
			return 0;
		}
	}
	return 0;
}

int set_pwm(unsigned int pwm_num, uint32_t period, uint32_t pulse)
{
	pwm_flags_t flags = 0;
	int err;
	pwm_info *p;

	if (pwm_num >= NUM_PWMS) {
		LOG_ERR("Incorrect pwm number set: %u; max is %u", pwm_num, NUM_PWMS);
		return -ENODEV;
	}
	p = &pwm_table[pwm_num];

	err = pwm_set_cycles(p->dev, p->channel, period, pulse, flags);
	if (err) {
		LOG_ERR("Error setting pwm_num %u, dev %s, ch %u, per %u, pulse %u: %d", 
				pwm_num, p->dev->name, p->channel, period, pulse, err);
	}
	return err;
}
