#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/util.h>

#include "device_id.h"
#include "ble_interface.h"
#include "sensor_node_sm.h"
#include "sensor_interface.h"

extern sensor_data_t sensor_data;

// PAwR Request Type Definitions
#define PAWR_CMD_REQUEST_TEMP 0x01
#define PAWR_CMD_REQUEST_HUMIDITY 0x02

static struct bt_conn *default_conn;
static struct bt_le_per_adv_sync_transfer_param past_param;
static struct bt_le_per_adv_sync *default_sync;

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info);
static void term_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_term_info *info);
static void recv_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_recv_info *info,
                    struct net_buf_simple *buf);

static struct bt_le_per_adv_sync_cb sync_callbacks = {
    .synced = sync_cb,
    .term = term_cb,
    .recv = recv_cb,
};

static struct __packed {
    uint8_t subevent;
    uint8_t response_slot;
} pawr_timing;

// 13 bytes: 1 byte for length, 1 byte for type, 2 bytes for company ID, 1 byte for device ID, 1 byte for command, 8 bytes for sensor value
#define RESPONSE_DATA_SIZE 13

// Response data format
struct response_data {
    uint8_t len;
    uint8_t type;
    uint16_t company_id;
    uint8_t device_id;
    uint8_t command;
    struct sensor_value sensor_reading;
} __packed;

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

void ble_start_advertising(void)
{
    int err;

    err = bt_le_adv_start(
        BT_LE_ADV_PARAM(BT_LE_ADV_OPT_ONE_TIME | BT_LE_ADV_OPT_CONNECTABLE,
                        BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL), ad, ARRAY_SIZE(ad), NULL, 0);
    if (err && err != -EALREADY) {
        printk("Advertising failed to start (err %d)\n", err);
    }
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

    err = bt_le_per_adv_sync_subevent(sync, &params);
    if (err) {
        printk("Failed to set subevents to sync to (err %d)\n", err);
    }    

    sensor_node_sm_set_state(Synced);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
            const struct bt_le_per_adv_sync_term_info *info)
{
    char le_addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

    printk("Sync terminated (reason %d)\n", info->reason);

    // Go to NotSynced State
    sensor_node_sm_set_state(NotSynced);
}

static bool parse_ad_field(struct bt_data *data, void *user_data)
{
    uint8_t *request_command = ((uint8_t *)user_data);

    if (data->type == BT_DATA_MANUFACTURER_DATA)
    {
        uint16_t company_id = (data->data[1] << 8) | data->data[0];
        *request_command = data->data[2];

        if (company_id != NOVEL_BITS_COMPANY_ID)
        {
            printk("Found Unexpected Company ID: 0x%04X\n", company_id);
        }
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
NET_BUF_SIMPLE_DEFINE_STATIC(rsp_buf, RESPONSE_DATA_SIZE+1);

static void recv_cb(struct bt_le_per_adv_sync *sync,
            const struct bt_le_per_adv_sync_recv_info *info, struct net_buf_simple *buf)
{
    int err;

    if (buf && buf->len) {
        uint8_t request_command;
        struct response_data rsp_data;

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

        printk("\n\n[SENSOR NODE: #%d] Indication: subevent %d, responding in slot %d\n", 
                DEVICE_ID,
                info->subevent,
                pawr_timing.response_slot);
        bt_data_parse(buf, parse_ad_field, &request_command);

        // Send back temperature or humidity data based on the request type
        net_buf_simple_reset(&rsp_buf);

        // Set the response data
        rsp_data.len = RESPONSE_DATA_SIZE;
        rsp_data.type = BT_DATA_MANUFACTURER_DATA;
        /* Novel Bits */
        rsp_data.company_id = NOVEL_BITS_COMPANY_ID;
        rsp_data.device_id = DEVICE_ID;
        rsp_data.command = request_command;

        // Check for command type (Temperature vs. Humidity)
        if (request_command == PAWR_CMD_REQUEST_TEMP)
        {
            printk("\tReceived request for temperature data.\n\tResponding with Temperature = %.2f °C\n", sensor_value_to_float(&sensor_data.temp));
            rsp_data.sensor_reading = sensor_data.temp;
        }
        else if (request_command == PAWR_CMD_REQUEST_HUMIDITY)
        {
            printk("\tReceived request for humidity data.\n\tResponding with Humidity = %0.2f %%\n", sensor_value_to_float(&sensor_data.humidity));
            rsp_data.sensor_reading = sensor_data.humidity;
        }
        else
        {
            printk("Received unknown request.\n");
            return;
        }

        // Assign response data to response buffer
        net_buf_simple_add_mem(&rsp_buf, &rsp_data, sizeof(rsp_data));

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
    } else {
        printk("Failed to receive indication: subevent %d\n", info->subevent);
    }
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

// Register callbacks for periodic adv sync
void ble_register_periodic_adv_sync_callbacks(void)
{
    int err;

    bt_le_per_adv_sync_cb_register(&sync_callbacks);

    // Subscribe to PAST events
    past_param.skip = 1;
    past_param.timeout = 1000; /* 10 seconds */
    past_param.options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_NONE;
    err = bt_le_per_adv_sync_transfer_subscribe(NULL, &past_param);
    if (err) {
        printk("PAST subscribe failed (err %d)\n", err);
        return;
    }   
    printk("Subscribed to PAST events\n");
}

void ble_delete_per_adv_sync(void)
{
    int err = bt_le_per_adv_sync_delete(default_sync);
    if (err) {
        printk("Failed to delete sync (err %d)\n", err);
    }
    default_sync = NULL;
}

void ble_init(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
    }
}
