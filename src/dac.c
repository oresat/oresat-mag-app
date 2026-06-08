/**
 * dac.c
 *
 * Original code came from zephyr/samples/driver/dac.
 *
 */
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h> 
#include <zephyr/sys/__assert.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/dsp/print_format.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(oresat_dac, LOG_LEVEL_DBG);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if (DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac0) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_channel_id) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_resolution))
#define DAC_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac0)
#define DAC_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac_channel_id)
#define DAC_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac_resolution)
#else
#error "Unsupported board: see README and check /zephyr,user node"
#define DAC_NODE DT_INVALID_NODE
#define DAC_CHANNEL_ID 0
#define DAC_RESOLUTION 0
#endif

static const struct device *const dac_dev = DEVICE_DT_GET(DAC_NODE);

int init_dac(void)
{
	int ret;
	const struct dac_channel_cfg dac_ch_cfg = {
		.channel_id  = DAC_CHANNEL_ID,
		.resolution  = DAC_RESOLUTION,
	#if defined(CONFIG_DAC_BUFFER_NOT_SUPPORT)
		.buffered = false,
	#else
		.buffered = true,
	#endif /* CONFIG_DAC_BUFFER_NOT_SUPPORT */
	};

	LOG_INF("Initializing DAC");

	/* Can we use the DAC? */
	if (!device_is_ready(dac_dev)) {
		LOG_ERR("DAC device %s is not ready", dac_dev->name);
		return -1;
	}

	/* Set it up */
	ret = dac_channel_setup(dac_dev, &dac_ch_cfg);
	if (ret != 0) {
		LOG_ERR("Setting up of DAC channel failed with code %d", ret);
	}

	return ret;
}

int write_dac(uint16_t value)
{
	int ret;
	/* Number of valid DAC values, e.g. 4096 for 12-bit DAC */
	//const int dac_values = 1U << DAC_RESOLUTION;

	ret = dac_write_value(dac_dev, DAC_CHANNEL_ID, value);
	if (ret != 0) {
		LOG_ERR("dac_write_value() failed with code %d", ret);
	}
	return ret;
}
