/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/util.h>

// State machine includes
#include <zephyr/smf.h>

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

// Sensor-related includes and definitions
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/sht4x.h>

// Change this value for each device
#define DEVICE_ID 1

// Display-related definitions
static lv_style_t style1;
static lv_obj_t * obj;

static const struct device *sht = DEVICE_DT_GET_ANY(sensirion_sht4x);

typedef struct sensor_data_s
{
    // Temperature value
    struct sensor_value temp;

    // Humidity value
    struct sensor_value humidity;
} sensor_data_t;

static sensor_data_t sensor_data;

// Logging
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

// Device Name Length
#define NAME_LEN 30

// Company ID for Novel Bits (used for the manufacturer data)
#define NOVEL_BITS_COMPANY_ID 0x08D3

// Advertising data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, 'N', 'o', 'v', 'e', 'l', ' ', 'B', 'i', 't', 's'),
	BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
                  (NOVEL_BITS_COMPANY_ID & 0xFF),
                  ((NOVEL_BITS_COMPANY_ID >> 8) & 0xFF),
                  DEVICE_ID)
};

// PAwR Definitions
#define PAWR_CMD_REQUEST_TEMP 0x01
#define PAWR_CMD_REQUEST_HUMIDITY 0x02

static struct bt_conn *default_conn;
static struct bt_le_per_adv_sync *default_sync = NULL;

static struct __packed {
	uint8_t subevent;
	uint8_t response_slot;
} pawr_timing;

static struct bt_le_per_adv_sync_transfer_param past_param;

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info);
static void term_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_term_info *info);
static void recv_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_recv_info *info,
					struct net_buf_simple *buf);
static void state_changed_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_state_info *info);

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
    .state_changed = state_changed_cb,
};

// Count of missing indications
static uint8_t missed_indications = 0;

// ----------------- State Machine START ---------------

// Forward declaration of state table
static const struct smf_state endnode_states[];

/* List of end node states */
enum endnode_state { NotSynced, Synced };

/* User defined object */
struct s_object {
        /* This must be first */
        struct smf_ctx ctx;

        /* Other state specific data add here */
} s_obj;

// State NotSynced
static void notsynced_entry(void *o)
{
	int err;

	// // Register callbacks for periodic adv sync
	// bt_le_per_adv_sync_cb_register(&sync_callbacks);

	// // Subscribe to PAST events
	// past_param.skip = 1;
	// past_param.timeout = 1000; /* 10 seconds */
	// past_param.options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_NONE;
	// err = bt_le_per_adv_sync_transfer_subscribe(NULL, &past_param);
	// if (err) {
	// 	printk("PAST subscribe failed (err %d)\n", err);
	// 	return;
	// }	
	// printk("Subscribed to PAST events\n");

    // Start advertising
	err = bt_le_adv_start(
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_ONE_TIME | BT_LE_ADV_OPT_CONNECTABLE,
						BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL), ad, ARRAY_SIZE(ad), NULL, 0);
	if (err && err != -EALREADY) {
		printk("Advertising failed to start (err %d)\n", err);
	}

	printk("Started Advertising... Waiting for periodic sync info..\n");    
}

static void notsynced_run(void *o)
{

}

static void notsynced_exit(void *o)
{
    /* Do something */
}

// State Synced
static void synced_entry(void *o)
{
	printk("Entering Synced State. Now syncing to subevent\n");
}

static void synced_exit(void *o)
{
    int err = bt_le_per_adv_sync_delete(default_sync);
    if (err) {
        printk("Failed to delete sync (err %d)\n", err);
    }
    default_sync = NULL;
    missed_indications = 0;
}

/* Populate state table */
static const struct smf_state endnode_states[] = {

		// NotSynced State
        [NotSynced] = SMF_CREATE_STATE(notsynced_entry, notsynced_run, notsynced_exit),

		// Synced State
        [Synced] = SMF_CREATE_STATE(synced_entry, NULL, synced_exit)
};

// ----------------- State Machine END -----------------

// Display Power Management
int suspend_display(const struct device *display_dev,
		    const struct device *display_bus_dev)
{
	int ret;

	ret = pm_device_action_run(display_dev, PM_DEVICE_ACTION_SUSPEND);
	if (ret < 0) {
		LOG_ERR("Could not suspend the display");
		return ret;
	}

	ret = pm_device_action_run(display_bus_dev, PM_DEVICE_ACTION_SUSPEND);
	if (ret < 0) {
		LOG_ERR("Could not suspend the display bus");
		return ret;
	}

	return 0;
}

int resume_display(const struct device *display_dev,
		   const struct device *display_bus_dev)
{
	int ret;

	ret = pm_device_action_run(display_dev, PM_DEVICE_ACTION_RESUME);
	if (ret < 0) {
		LOG_ERR("Could not resume the display");
		return ret;
	}

	ret = pm_device_action_run(display_bus_dev, PM_DEVICE_ACTION_RESUME);
	if (ret < 0) {
		LOG_ERR("Could not resume the display bus");
		return ret;
	}

	return 0;
}

void sensor_capture_data(void)
{
    if (sensor_sample_fetch(sht)) {
        printk("Failed to fetch sample from SHT4X device\n");
        return;
    }

    sensor_channel_get(sht, SENSOR_CHAN_AMBIENT_TEMP, &sensor_data.temp);
    sensor_channel_get(sht, SENSOR_CHAN_HUMIDITY, &sensor_data.humidity);
}

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	printk("Synced to %s with %d subevents\n", le_addr, info->num_subevents);

	default_sync = sync;

	int err;
	uint8_t subevents[1];
	struct bt_le_per_adv_sync_subevent_params params;	

	params.properties = 0;
	params.num_subevents = 1;
	params.subevents = subevents;
	subevents[0] = pawr_timing.subevent;

    printk("Setting subevent to sync to: %d\n", pawr_timing.subevent);
    printk("Setting response slot to: %d\n", pawr_timing.response_slot);

	err = bt_le_per_adv_sync_subevent(default_sync, &params);
	if (err) {
		printk("Failed to set subevents to sync to (err %d)\n", err);
	}    

	smf_set_state(SMF_CTX(&s_obj), &endnode_states[Synced]);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("Sync terminated (reason %d)\n", info->reason);

	// Go to NotSynced State
	smf_set_state(SMF_CTX(&s_obj), &endnode_states[NotSynced]);
}

static bool print_ad_field(struct bt_data *data, void *user_data)
{
	uint8_t *request_command = ((uint8_t *)user_data);

    if (data->type == BT_DATA_MANUFACTURER_DATA)
    {
        uint16_t company_id = (data->data[1] << 8) | data->data[0];
        *request_command = data->data[2];

        printk("Company ID: 0x%04X, Request Type: %s\n", company_id, (*request_command == PAWR_CMD_REQUEST_TEMP) ? "Temperature" : "Humidity");
    }
	return true;
 }

// Function to set response data
int bt_le_per_adv_set_response_data(struct bt_le_per_adv_sync *per_adv_sync,
				    const struct bt_le_per_adv_response_params *params,
				    const struct net_buf_simple *data);

// Response parameters
static struct bt_le_per_adv_response_params rsp_params;

// Response buffer
NET_BUF_SIMPLE_DEFINE_STATIC(rsp_buf, 247);

// Function to handle state changes
static void state_changed_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_state_info *info)
{
    printk("Recv enabled: %d\n", info->recv_enabled);
}

static void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info, struct net_buf_simple *buf)
{
	int err;

	if (buf && buf->len) {
        uint8_t request_command;
        uint8_t device_id = DEVICE_ID;

        // Clear missing indication count
        missed_indications = 0;

        if (info->subevent != pawr_timing.subevent)
        {
            printk("Received indication for subevent %d, but expected subevent %d\n", info->subevent, pawr_timing.subevent);

            // Sync to the correct subevent
            uint8_t subevents[1];
            struct bt_le_per_adv_sync_subevent_params params;
            params.properties = 0;
            params.num_subevents = 1;
            params.subevents = subevents;
            subevents[0] = pawr_timing.subevent;

            err = bt_le_per_adv_sync_subevent(sync, &params);
            if (err) {
                printk("Failed to set subevents to sync to (err %d)\n", err);
            }
            return;
        }

        printk("Indication: subevent %d, responding in slot %d\n", info->subevent,
		       pawr_timing.response_slot);
		bt_data_parse(buf, print_ad_field, &request_command);

		// Send back temperature or humidity data based on the request type
		net_buf_simple_reset(&rsp_buf);

        // Check for command type (Temperature vs. Humidity)
        if (request_command == PAWR_CMD_REQUEST_TEMP)
        {
            printk("Received request for temperature data. Responding with Temperature = %.2f °C\n", sensor_value_to_double(&sensor_data.temp));
            net_buf_simple_add_mem(&rsp_buf, &device_id, sizeof(device_id));
            net_buf_simple_add_mem(&rsp_buf, &request_command, sizeof(request_command));
            net_buf_simple_add_mem(&rsp_buf, &sensor_data.temp, sizeof(sensor_data.temp));
        }
        else if (request_command == PAWR_CMD_REQUEST_HUMIDITY)
        {
            printk("Received request for humidity data. Responding with Humidity = %0.2f %%\n", sensor_value_to_double(&sensor_data.humidity));
            net_buf_simple_add_mem(&rsp_buf, &device_id, sizeof(device_id));
            net_buf_simple_add_mem(&rsp_buf, &request_command, sizeof(request_command));
            net_buf_simple_add_mem(&rsp_buf, &sensor_data.humidity, sizeof(sensor_data.humidity));
        }
        else
        {
            printk("Received unknown request.\n");
            return;
        }

		rsp_params.request_event = info->periodic_event_counter;
		rsp_params.request_subevent = info->subevent;

		/* Respond in current subevent and assigned response slot */
		rsp_params.response_subevent = info->subevent;
		rsp_params.response_slot = pawr_timing.response_slot;

		err = bt_le_per_adv_set_response_data(sync, &rsp_params, &rsp_buf);
		if (err) {
			printk("Failed to send response (err %d)\n", err);
		}
	} else if (buf) {
		printk("Received empty indication: subevent %d\n", info->subevent);

        // Clear missing indication count
        missed_indications = 0;
	} else {
		printk("Failed to receive %d indication(s): subevent %d\n", missed_indications, info->subevent);

        // increment missed indication count
        missed_indications++;
	}

    // if (missing_indications > 10)
    // {
    //     printk("Too many missing indications. Terminating sync.\n");
    //     // err = bt_le_per_adv_sync_recv_disable(default_sync);
    //     // if (err) {
    //     //     printk("Failed to disable recv (err %d)\n", err);
    //     // }
    //     // bt_le_per_adv_sync_transfer_unsubscribe(NULL);
    //     smf_set_state(SMF_CTX(&s_obj), &endnode_states[NotSynced]);
    // }
}

static struct bt_uuid_128 pawr_svc_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0));
static struct bt_uuid_128 pawr_char_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static ssize_t write_timing(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			    uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(pawr_timing)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (!default_sync)
    {
		// printk("Not synced yet. Delaying the new PAwR Subevent Timing for a bit\n");
        // k_msleep(500);
	}     

	memcpy(&pawr_timing, buf, len);

	printk("New timing: subevent %d, response slot %d\n", pawr_timing.subevent,
	       pawr_timing.response_slot);    

	return len;
}

BT_GATT_SERVICE_DEFINE(pawr_svc, BT_GATT_PRIMARY_SERVICE(&pawr_svc_uuid.uuid),
		       BT_GATT_CHARACTERISTIC(&pawr_char_uuid.uuid, BT_GATT_CHRC_WRITE,
					      BT_GATT_PERM_WRITE, NULL, write_timing,
					      &pawr_timing));

void connected(struct bt_conn *conn, uint8_t err)
{
	printk("Connected (err 0x%02X)\n", err);

	if (err) {
		default_conn = NULL;

		return;
	}

	default_conn = bt_conn_ref(conn);
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
	bt_conn_unref(default_conn);
	default_conn = NULL;

	printk("Disconnected (reason 0x%02X)\n", reason);
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
};

void initialize_display(const struct device *display_dev,
                        const struct device *display_bus_dev)
{
	LOG_INF("Display device: %s", display_dev->name);

    obj = lv_label_create(lv_scr_act());
    lv_label_set_text(obj, "Initializing System...");
	lv_style_set_text_font(&style1, &lv_font_montserrat_20);
	lv_obj_add_style(obj, &style1, LV_PART_MAIN | LV_STATE_DEFAULT);
	lv_obj_set_style_text_color( obj, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_center(obj);
	lv_task_handler();
	suspend_display(display_dev, display_bus_dev);
}

void update_display_with_sensor_data(lv_obj_t * obj, const struct device *display_dev,
    const struct device *display_bus_dev)
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

int main(void)
{
	int err;
	int32_t ret;
    uint32_t counter = 0;

	lv_style_init(&style1);
	const struct device *display_dev;
	const struct device *display_bus_dev;

    sensor_capture_data();

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	display_bus_dev = DEVICE_DT_GET(DT_PARENT(DT_CHOSEN(zephyr_display)));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}

	printk("Starting Periodic Advertising with Responses Synchronization Demo\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	// Register callbacks for periodic adv sync
	bt_le_per_adv_sync_cb_register(&sync_callbacks);

	// Subscribe to PAST events
	past_param.skip = 5;
	past_param.timeout = 1000; /* 10 seconds */
	// past_param.options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_FILTER_DUPLICATES;
    past_param.options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_NONE;
	err = bt_le_per_adv_sync_transfer_subscribe(NULL, &past_param);
	if (err) {
		printk("PAST subscribe failed (err %d)\n", err);
		return 0;
	}	
	printk("Subscribed to PAST events\n");

	/* Set initial state */
	smf_set_initial(SMF_CTX(&s_obj), &endnode_states[NotSynced]);

    printk("Initializing system for Device #%d\n", DEVICE_ID);

    initialize_display(display_dev, display_bus_dev); 

	while (1) {

        // Run the state machine
		ret = smf_run_state(SMF_CTX(&s_obj));
		if (ret) {
			/* handle return code and terminate state machine */
			printk("Error running state machine\n");
			break;
		}

        // Only update sensor reading every 5 seconds
        if (counter % 50 == 0)
        {
            // Capture sensor data
            sensor_capture_data();
        }

        // Only update sensor reading and display every 30 seconds
        if (counter % 300 == 0)
        {
            // Update display with sensor data
            update_display_with_sensor_data(obj, display_dev, display_bus_dev);
        }

        // Sleep for 100 msec
		k_msleep(100);
        counter++;
	}

	return 0;
}
