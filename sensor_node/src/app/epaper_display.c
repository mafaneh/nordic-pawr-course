#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <lvgl_input_device.h>
#include <zephyr/pm/device.h>

#include "device_id.h"
#include "sensor_interface.h"
#include "epaper_display.h"

extern sensor_data_t sensor_data;

// Display-related definitions
static lv_style_t style1;
static lv_obj_t * obj;

static const struct device *display_dev;
static const struct device *display_bus_dev;

// Display Power Management
static int suspend_display(const struct device *display_dev,
            const struct device *display_bus_dev)
{
    int ret;

    ret = pm_device_action_run(display_dev, PM_DEVICE_ACTION_SUSPEND);
    if (ret < 0) {
        printk("Could not suspend the display");
        return ret;
    }

    ret = pm_device_action_run(display_bus_dev, PM_DEVICE_ACTION_SUSPEND);
    if (ret < 0) {
        printk("Could not suspend the display bus");
        return ret;
    }

    return 0;
}

static int resume_display(const struct device *display_dev,
           const struct device *display_bus_dev)
{
    int ret;

    ret = pm_device_action_run(display_dev, PM_DEVICE_ACTION_RESUME);
    if (ret < 0) {
        printk("Could not resume the display");
        return ret;
    }

    ret = pm_device_action_run(display_bus_dev, PM_DEVICE_ACTION_RESUME);
    if (ret < 0) {
        printk("Could not resume the display bus");
        return ret;
    }

    return 0;
}

static void initialize_display_style(void)
{
    printk("Display device: %s", display_dev->name);

    obj = lv_label_create(lv_scr_act());
    lv_label_set_text(obj, "Initializing System...");
    lv_style_set_text_font(&style1, &lv_font_montserrat_20);
    lv_obj_add_style(obj, &style1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color( obj, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(obj);
    lv_task_handler();
    suspend_display(display_dev, display_bus_dev);
}

void epaper_display_update_sensor_data(void)
{
    char text[200];

    resume_display(display_dev, display_bus_dev);
    sprintf(text, "---- SENSOR NODE #%d ----\n\n Temperature: %.2f °C\n Humidity: %0.2f %%",
        DEVICE_ID,
        sensor_value_to_double(&sensor_data.temp),
        sensor_value_to_double(&sensor_data.humidity));
    lv_label_set_text(obj, text);
    lv_task_handler();
    suspend_display(display_dev, display_bus_dev);
}

void epaper_display_init(void)
{
    lv_style_init(&style1);

    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    display_bus_dev = DEVICE_DT_GET(DT_PARENT(DT_CHOSEN(zephyr_display)));
    if (!device_is_ready(display_dev)) {
        printk("Device not ready, aborting test");
        return;
    }

    initialize_display_style();
}