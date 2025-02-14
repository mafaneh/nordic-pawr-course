#include "sensor_interface.h"

// Sensor-related includes and definitions
#include <zephyr/drivers/sensor/sht4x.h>

static const struct device *sht = DEVICE_DT_GET_ANY(sensirion_sht4x);

sensor_data_t sensor_data;

void sensor_capture_data(void)
{
    // char text[200];

    if (sensor_sample_fetch(sht)) {
        printk("Failed to fetch sample from SHT4X device\n");
        return;
    }

    sensor_channel_get(sht, SENSOR_CHAN_AMBIENT_TEMP, &sensor_data.temp);
    sensor_channel_get(sht, SENSOR_CHAN_HUMIDITY, &sensor_data.humidity);

    // sprintf(text, "---- SENSOR DATA ----\n\n Temperature: %.2f °C\n Humidity: %0.2f %%",
    //     sensor_value_to_float(&sensor_data.temp),
    //     sensor_value_to_float(&sensor_data.humidity));  
        
    // printk(text);
}