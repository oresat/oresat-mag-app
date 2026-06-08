#if !defined(_ADC_H_)
#define _ADC_H_

int get_num_adc_channels(void);

int init_adc(void);

int acquire_adc_readings(void);

int read_adc(unsigned int adc_num, int32_t *val_mv);

#endif


