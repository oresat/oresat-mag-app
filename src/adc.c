/**
 * adc.c
 *
 * Original code came from zephyr/samples/driver/adc_sequence.
 *
 * Modified slightly to run as a thread and use logging.
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h> 
#include <zephyr/sys/__assert.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/dsp/print_format.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(oresat_adc, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define ADC_SLEEP_TIME_MS 50 /* Use 5 to generate more data to graph samples that follow the DAC output */

#define MAX_ADC_READ_TRIES 10

/* ADC node from the devicetree. */
#define ADC_NODE0 DT_ALIAS(adc0)
#define ADC_NODE1 DT_ALIAS(adc1)

/* Auxiliary macro to obtain channel vref, if available. */
#define CHANNEL_VREF(node_id) DT_PROP_OR(node_id, zephyr_vref_mv, 0)

/* Data of ADC device specified in devicetree. */
static const struct device *adc_devs[] = {
	DEVICE_DT_GET(ADC_NODE0),
	DEVICE_DT_GET(ADC_NODE1)
};
#define ADC_DEV_COUNT 2

/* Data array of ADC channels for the specified ADC. */
static const struct adc_channel_cfg channel_cfgs_dev_0[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE0, ADC_CHANNEL_CFG_DT, (,))
};
static const struct adc_channel_cfg channel_cfgs_dev_1[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE1, ADC_CHANNEL_CFG_DT, (,))
};
#define CHANNEL_COUNT_DEV_0 ARRAY_SIZE(channel_cfgs_dev_0)
#define CHANNEL_COUNT_DEV_1 ARRAY_SIZE(channel_cfgs_dev_1)
#define CHANNEL_COUNT (CHANNEL_COUNT_DEV_0 + CHANNEL_COUNT_DEV_1)
#define MAX_CHANNEL_DEV_COUNT MAX(CHANNEL_COUNT_DEV_0, CHANNEL_COUNT_DEV_1)
static const struct adc_channel_cfg *adc_channel_cfgs[ADC_DEV_COUNT] = {
	channel_cfgs_dev_0, channel_cfgs_dev_1
};
static int adc_channel_counts[ADC_DEV_COUNT] = {
	CHANNEL_COUNT_DEV_0, CHANNEL_COUNT_DEV_1
};

/* Data array of ADC channel voltage references. */
static uint32_t vrefs_mv_dev_0[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE0, CHANNEL_VREF, (,))
};
static uint32_t vrefs_mv_dev_1[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE1, CHANNEL_VREF, (,))
};
#define ADC_VREF_DEV_0_COUNT ARRAY_SIZE(vrefs_mv_dev_0)
#define ADC_VREF_DEV_1_COUNT ARRAY_SIZE(vrefs_mv_dev_1)
#define ADC_VREF_COUNT (ADC_VREF_DEV_0_COUNT + ADC_VREF_DEV_1_COUNT)
#define MAX_ADC_VREF_COUNT MAX(ADC_VREF_DEV_0_COUNT, ADC_VREF_DEV_1_COUNT)
static int adc_vref_counts[ADC_DEV_COUNT] = {
	ADC_VREF_DEV_0_COUNT, ADC_VREF_DEV_1_COUNT
};
static uint32_t *adc_vref_mvs[] = {
	vrefs_mv_dev_0, vrefs_mv_dev_1
};

struct adc_info; // forward decl

// structure to tie together various Zephyr device structures and adc structures
typedef struct adc_dev_info {
	const struct device *dev;
	uint32_t num_vrefs;
	uint32_t *vrefs_mv;						// pointer to an array of num_vrefs uint32_t mV values
	uint32_t num_ch;						// number used in the adc_channel_cfg array
	const struct adc_channel_cfg *config;	// pointer to num_ch adc_channel_cfg structures
	struct adc_sequence sequence;
#ifdef CONFIG_SEQUENCE_32BITS_REGISTERS
	uint32_t channel_reading[CONFIG_SEQUENCE_SAMPLES][CHANNEL_COUNT];
#else
	uint16_t channel_reading[CONFIG_SEQUENCE_SAMPLES][CHANNEL_COUNT];
#endif
} adc_dev_info;

static adc_dev_info adc_dev_infos[ADC_DEV_COUNT];

// structure to map the user interface to this file to the multiple adc devices and their channels
// this allows the user to specify adc_num 0, 1, or 2, and have them map properly to the interals
typedef struct adc_info {
	adc_dev_info *adcdev;
	uint16_t adc_dev_num;	// number of the ADC device (index into adc_devs[])
	uint16_t ch_num;		// number of the channel on that device
} adc_info;

static adc_info adc_info_map[CHANNEL_COUNT];

// Renumber the outside interface to match reality (adc number 0 and 2 seem swapped somewhere.
// TODO: fix.
static int log_to_phys[CHANNEL_COUNT];

/* Options for the sequence sampling. */
static const struct adc_sequence_options options = {
	.extra_samplings = CONFIG_SEQUENCE_SAMPLES - 1,
	.interval_us = 0,
};

static void init_adc_info(void)
{
	LOG_DBG("Mapping ADC info...");
	LOG_DBG("ADC_DEV_COUNT:%d, CH_CNT_D0:%d, CH_CNT_D1:%d, CH_CNT:%d, MAX_CH_CNT:%d",
			ADC_DEV_COUNT, CHANNEL_COUNT_DEV_0, CHANNEL_COUNT_DEV_1, CHANNEL_COUNT, MAX_CHANNEL_DEV_COUNT);
	LOG_DBG("ADC_VREF_D0:%d, ADC_VREF_D1:%d, ADC_VREF_COUNT:%d",
			ADC_VREF_DEV_0_COUNT, ADC_VREF_DEV_1_COUNT, ADC_VREF_COUNT);

	uint32_t channel_bitmask;
	adc_dev_info *padi = adc_dev_infos;
	int adc_index = 0; // index into adc_info_map
	int i;
	int j;

	for (i = 0; i < ADC_DEV_COUNT; i++) {
		padi->dev = adc_devs[i];
		padi->num_vrefs = adc_vref_counts[i];
		padi->vrefs_mv = adc_vref_mvs[i];
		padi->num_ch = adc_channel_counts[i];
		padi->config = adc_channel_cfgs[i];

		channel_bitmask = 0;
		for (j = 0; j < padi->num_ch; j++) {
			channel_bitmask |= BIT(padi->config[j].channel_id);
			adc_info_map[adc_index].adcdev = padi;
			adc_info_map[adc_index].adc_dev_num = i;
			adc_info_map[adc_index].ch_num = j;
			adc_index++;
		}

		padi->sequence.options = &options;
		padi->sequence.channels = channel_bitmask;
		padi->sequence.buffer = padi->channel_reading;
		padi->sequence.buffer_size = sizeof(padi->channel_reading);
		padi->sequence.resolution = CONFIG_SEQUENCE_RESOLUTION;
		padi->sequence.oversampling = CONFIG_SEQUENCE_OVERSAMPLING;
		padi->sequence.calibrate = 0;

		LOG_DBG("ADC%d (%s): num_vrefs:%u, num_ch:%u, bitmask:0x%08x, res:%u, ave:%u, buf:%p, buf_size:%d",
				i, padi->dev->name, padi->num_vrefs, padi->num_ch, padi->sequence.channels,
				padi->sequence.resolution, padi->sequence.oversampling,
				padi->sequence.buffer, padi->sequence.buffer_size);

		padi++;
	}
	if (adc_index == 3) {
		log_to_phys[0] = 2;
		log_to_phys[1] = 1;
		log_to_phys[2] = 0;
	}
	for (i = 0; i < adc_index; i++) {
		LOG_DBG("adc_info_map[%d]: padi:%p, logidx:%d, physidx:%d, dev:%d",
				i, adc_info_map[i].adcdev, i, log_to_phys[i], adc_info_map[i].adc_dev_num);
	}
}

int get_num_adc_channels(void)
{
	return CHANNEL_COUNT;
}

int init_adc(void)
{
	int err = 0;
	const struct device *adc;
	adc_dev_info *padi;
	const struct adc_channel_cfg *config;
	int i;
	int j;

	init_adc_info();

	LOG_INF("Channels: %d, sequence samples: %d, resolution: %d",
		   CHANNEL_COUNT, CONFIG_SEQUENCE_SAMPLES, CONFIG_SEQUENCE_RESOLUTION);

	for (i = 0U; i < ADC_DEV_COUNT; i++) {
		adc = adc_devs[i];
		if (!device_is_ready(adc)) {
			LOG_ERR("ADC controller device %s not ready", adc->name);
			err = -ENODEV;
			continue;
		}
		padi = &adc_dev_infos[i];
		LOG_INF("Init adc%d: %s, num_ch:%d, num_vref:%d", i, adc->name, padi->num_ch, padi->num_vrefs);
	}

	for (i = 0U; i < ADC_DEV_COUNT; i++) {
		adc = adc_devs[i];
		padi = &adc_dev_infos[i];
		for (j = 0; j < padi->num_ch; j++) {
			LOG_INF("  Init ch:%d", j);
			config = &padi->config[j];
			err = adc_channel_setup(adc, &padi->config[j]);
			if (err < 0) {
				LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
				return 0;
			}
			#ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
			LOG_INF("  Channel: %u, gain: %u, acq time: %u, diff: %u, inp_pos: %u, inp_neg: %u",
					config->channel_id, config->gain, config->acquisition_time,
					config->differential, config->input_positive, config->input_negative);
			#else
			LOG_INF("  Channel: %u, gain: %u, acq time: %u, diff: %u",
					config->channel_id, config->gain, config->acquisition_time,
					config->differential);
			#endif
		}

		for (int k = 0; k < padi->num_vrefs; padi++) {
			if ((padi->vrefs_mv[k] == 0) && (padi->config->reference == ADC_REF_INTERNAL)) {
				padi->vrefs_mv[k] = adc_ref_internal(adc);
			}
			LOG_INF("  Vref: %u, vref_mv: %u", k, padi->vrefs_mv[k]);
		}
	}
	return err;
}

int acquire_adc_readings(void)
{
	int err;
	int i;
	int count = 0;
	adc_dev_info *padi = adc_dev_infos;

	for (i = 0; i < ADC_DEV_COUNT; i++) {
		LOG_DBG("Requesting read from adc%d", i);
		do {
			err = adc_read(padi->dev, &padi->sequence);
			if (err < 0) {
				k_sleep(K_MSEC(100));
				//LOG_ERR("Could not read (%d)", err);
			}
		} while (err && (count++ < MAX_ADC_READ_TRIES));
		if (err) {
			LOG_ERR("Failed to read after %d tries; error %d", count, err);
		}
		padi++;
	}
	return err;
}

int read_adc(unsigned int log_adc_num, int32_t *val_mv)
{
	int err;
	int32_t raw;
	int num_samples = 0;
	int32_t val_tmp;
	int32_t val_sum = 0;
	adc_info *info;
	adc_dev_info *padi;
	int phys_adc_num = log_to_phys[log_adc_num];

	if (log_adc_num >= CHANNEL_COUNT) {
		LOG_ERR("Incorrect adc number set:%u; max is:%u", log_adc_num, CHANNEL_COUNT);
		return -ENODEV;
	}
	info = &adc_info_map[phys_adc_num];
	padi = info->adcdev;

	*val_mv = 0;

	LOG_DBG("Reading log_adc_num:%d, phys_num:%d, dev_num:%d (%s), ch_num:%d",
			log_adc_num, phys_adc_num, info->adc_dev_num, padi->dev->name, info->ch_num);

	for (size_t sample_index = 0U; sample_index < CONFIG_SEQUENCE_SAMPLES; sample_index++) {

		raw = padi->channel_reading[sample_index][info->ch_num];
		val_tmp = raw;

		err = adc_raw_to_millivolts(padi->vrefs_mv[info->ch_num],
									padi->config->gain,
									CONFIG_SEQUENCE_RESOLUTION, &val_tmp);
		if (err) {
			LOG_ERR("Error converting raw adc to mV: %d", err);
		} else {
			num_samples++;
			val_sum += val_tmp;
		}
	}
	if (num_samples) {
		*val_mv = val_sum / num_samples;
	} else {
		return -ENODATA;
	}

	/* conversion to mV may not be supported, skip if not */
	if (!err) {
		LOG_DBG("mV: %u", *val_mv);
	}
	return 0;
}
