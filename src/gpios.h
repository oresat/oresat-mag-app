#ifndef _GPIOS_H_
#define _GPIOS_H_

typedef enum {
	HW_REV_1_0_0 = 0,	// board revision 0.0.0
	HW_REV_1_1_0 = 1	// board revision 1.1.0
} board_hw_rev_t;

// TODO: refactor the use of these gpio_dt_specs from magnetorquers.c
extern const struct gpio_dt_spec mt_en;
extern const struct gpio_dt_spec n_mt_en_fault;
extern const struct gpio_dt_spec n_mt_stby_rst;
extern const struct gpio_dt_spec mt_x_phase;
extern const struct gpio_dt_spec mt_y_phase;
extern const struct gpio_dt_spec mt_z_phase;
extern const struct gpio_dt_spec hw_rev_bit_0; 
extern const struct gpio_dt_spec hw_rev_bit_1; 
extern const struct gpio_dt_spec hw_rev_bit_2; 
extern const struct gpio_dt_spec tcan_nfault_oc; 
extern const struct gpio_dt_spec tcan_silent;

int init_gpios(void);

board_hw_rev_t get_board_hw_rev(void);

int set_can_silent(void);

int set_can_normal(void);

bool get_can_fault(void);

bool check_magnetorquer_fault(void);

#endif
