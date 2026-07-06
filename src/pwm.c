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

LOG_MODULE_REGISTER(oresat_pwm, CONFIG_PWM_LOG_LEVEL);

/**
 * @brief   Converts from fraction to pulse width.
 * @note    Be careful with rounding errors, this is integer math not magic.
 *          You can specify tenths of thousandth but make sure you have the
 *          proper hardware resolution by carefully choosing the clock source
 *          and prescaler settings, see @p PWM_COMPUTE_PSC.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] denominator denominator of the fraction
 * @param[in] numerator numerator of the fraction
 * @return              The pulse width to be passed to @p pwmEnableChannel().
 *
 * @api
 */
#define PWM_FRACTION_TO_WIDTH(period, denominator, numerator) \
		((uint32_t)((((uint32_t)period) * \
					 (uint32_t)(numerator)) / (uint32_t)(denominator)))

/**
 * @brief   Converts from percentage to pulse width.
 * @note    Be careful with rounding errors, this is integer math not magic.
 *          You can specify tenths of thousandth but make sure you have the
 *          proper hardware resolution by carefully choosing the clock source
 *          and prescaler settings, see @p PWM_COMPUTE_PSC.
 *
 * @param[in] pwmp      pointer to a @p PWMDriver object
 * @param[in] percentage percentage as an integer between 0 and 10000
 * @return              The pulse width to be passed to @p pwmEnableChannel().
 *
 * @api
 */
#define PWM_PERCENTAGE_TO_WIDTH(period, percentage) \
	PWM_FRACTION_TO_WIDTH(period, PERCENT_SCALE, percentage)

/* PWM node from the devicetree. */
#define PWM_NODE_0 DT_ALIAS(pwm0)
#define PWM_NODE_1 DT_ALIAS(pwm1)

#define PWM_FREQUENCY 20000U
#define PERCENT_SCALE 10000U

typedef struct pwm_info {
	const struct device *dev;
	int channel;
	uint64_t cps;
	uint32_t period;
	uint32_t frequency;
} pwm_info;

static pwm_info pwm_table[] = {
	{DEVICE_DT_GET(PWM_NODE_0), 1, 0, 0}, // pwm_num = 0
	{DEVICE_DT_GET(PWM_NODE_0), 0, 0, 0}, // pwm_num = 1
	{DEVICE_DT_GET(PWM_NODE_1), 1, 0, 0}  // pwm_num = 2
};
#define NUM_PWMS ARRAY_SIZE(pwm_table)

int init_pwm(void)
{
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
		err = pwm_get_cycles_per_sec(p->dev, p->channel, &p->cps);
		if (err) {
			LOG_ERR("Error getting cycles per second:%d", err);
			return err;
		}
		p->period = (p->cps + PWM_FREQUENCY / 2) / PWM_FREQUENCY;
		p->frequency = PWM_FREQUENCY;
		LOG_DBG("pwm%d (%s): cycles_per_sec:%lld, period:%d", i, p->dev->name, p->cps, p->period);
	}
	return 0;
}

int set_pwm_frequency(unsigned int pwm_num, uint32_t frequency)
{
	pwm_info *p;

	if (pwm_num >= NUM_PWMS) {
		LOG_ERR("Incorrect pwm number set: %u; max is %u", pwm_num, NUM_PWMS);
		return -ENODEV;
	}
	p = &pwm_table[pwm_num];
	p->period = (p->cps + frequency / 2) / frequency;
	p->frequency = frequency;
	return 0;
}

uint32_t get_pwm_frequency(unsigned int pwm_num)
{
	if (pwm_num >= NUM_PWMS) {
		LOG_ERR("Incorrect pwm number set: %u; max is %u", pwm_num, NUM_PWMS);
		return 0;
	}
	return pwm_table[pwm_num].frequency;
}

int set_pwm(unsigned int pwm_num, uint32_t scaled_percent)
{
	pwm_flags_t flags = 0;
	int err;
	pwm_info *p;
	uint32_t pulse;

	if (pwm_num >= NUM_PWMS) {
		LOG_ERR("Incorrect pwm number set: %u; max is %u", pwm_num, NUM_PWMS);
		return -ENODEV;
	}
	p = &pwm_table[pwm_num];

	pulse = PWM_PERCENTAGE_TO_WIDTH(p->period, scaled_percent);

	LOG_DBG("Setting pwm%d (%s), ch:%d, percent:%d, per:%d, pulse:%d",
			pwm_num, p->dev->name, p->channel, scaled_percent / (PERCENT_SCALE / 100), p->period, pulse);
	err = pwm_set_cycles(p->dev, p->channel, p->period, pulse, flags);
	if (err) {
		LOG_ERR("Error setting pwm_num %u, dev %s, ch %u, per %u, pulse %u: %d", 
				pwm_num, p->dev->name, p->channel, p->period, pulse, err);
	}
	return err;
}
