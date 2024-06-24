/*
 * Copyright (c) 2024 Novel Bits, LLC
 *
 * SPDX-License-Identifier: MIT License
 */

#include "app/device_id.h"
#include "app/sensor_node_sm.h"
#include "app/sensor_interface.h"
#include "app/ble_interface.h"
#include "app/epaper_display.h"

// TODO: UNCOMMENT to enable low power mode
// #define LOW_POWER_MODE

// Define delays for Low Power Mode and Regular Mode
#ifdef LOW_POWER_MODE
#define LOOP_DELAY 10000 // 10 seconds in Low Power Mode
#define SENSOR_UDPATE_DELAY 10000 // 10 seconds in Low Power Mode
#define DISPLAY_UPDATE_DELAY 36000 // 1 hour in Low Power Mode
#else
#define LOOP_DELAY 100 // 100 msec in Regular Mode
#define SENSOR_UDPATE_DELAY 1000 // 1 second in Regular Mode
#define DISPLAY_UPDATE_DELAY 30000 // 30 seconds in Regular Mode
#endif

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

        // Only update sensor reading every N msec (depends on settings at top of file)
        if (counter % (SENSOR_UDPATE_DELAY/LOOP_DELAY) == 0)
        {
            // Capture sensor data
            sensor_capture_data();
        }

        // Only update display every M msec (depends on settings at top of file)
        if (counter % (DISPLAY_UPDATE_DELAY/LOOP_DELAY) == 0)
        {
            // Update display with sensor data
            epaper_display_update_sensor_data();
        }

        // Sleep for X msec (depends on device mode and settings at top of file)
        k_msleep(LOOP_DELAY);

        // Increment counter
        counter++;
    } while (true);

    return 0;
}
