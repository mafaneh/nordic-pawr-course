/*
 * Copyright (c) 2024 Novel Bits, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "app/device_id.h"
#include "app/sensor_node_sm.h"
#include "app/sensor_interface.h"
#include "app/ble_interface.h"
#include "app/epaper_display.h"

// Sensor data
extern sensor_data_t sensor_data;

int main(void)
{
    uint32_t counter = 0;

    printk("Initializing System [Sensor Node #%d]\n", DEVICE_ID);

    // Initialize sensor interface
    sensor_capture_data();

    // Initialize ePaper display
    epaper_display_init();

    // Initialize BLE
    ble_init();

    // Register callbacks for periodic adv sync
    ble_register_periodic_adv_sync_callbacks();

    // Initialize State Machine
    sensor_node_sm_init();

    do {

        // Run the state machine
        if (sensor_node_sm_run())
        {
            /* handle return code and terminate state machine */
            printk("Error running state machine\n");
            break;
        }

        // Only update sensor reading every 1 second
        if (counter % 10 == 0)
        {
            // Capture sensor data
            sensor_capture_data();
        }

        // Only update sensor reading and display every 30 seconds
        if (counter % 300 == 0)
        {
            // Update display with sensor data
            epaper_display_update_sensor_data();
        }

        // Sleep for 100 msec
        k_msleep(100);

        // Increment counter
        counter++;
    } while (true);

    return 0;
}
