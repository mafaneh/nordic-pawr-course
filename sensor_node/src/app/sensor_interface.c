#include "sensor_interface.h"

// Sensor-related includes and definitions
#include <zephyr/drivers/sensor/sht4x.h>

static const struct device *sht = DEVICE_DT_GET_ANY(sensirion_sht4x);

sensor_data_t sensor_data;

void sensor_capture_data(void)
{
    if (sensor_sample_fetch(sht)) {
        printk("Failed to fetch sample from SHT4X device\n");
        return;
    }

    sensor_channel_get(sht, SENSOR_CHAN_AMBIENT_TEMP, &sensor_data.temp);
    sensor_channel_get(sht, SENSOR_CHAN_HUMIDITY, &sensor_data.humidity);
}