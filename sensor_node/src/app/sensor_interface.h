#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <zephyr/drivers/sensor.h>
typedef struct sensor_data_s
{
    // Temperature value
    struct sensor_value temp;

    // Humidity value
    struct sensor_value humidity;
} sensor_data_t;

void sensor_capture_data(void);

#endif