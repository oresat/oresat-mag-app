#if !defined(_PWM_H_)
#define _PWM_H_

int init_pwm(void);

int set_pwm(unsigned int pwm_num, uint32_t scaled_percent);

#endif

