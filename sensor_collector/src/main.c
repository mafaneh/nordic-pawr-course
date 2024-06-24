/*
 * Copyright (c) 2024 Novel Bits, LLC
 *
 * SPDX-License-Identifier: MIT License
 */

#include <zephyr/sys/byteorder.h> // Needed for handling Little Endian data
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/sensor.h>
#include <dk_buttons_and_leds.h>

// UNCOMMENT for low power mode
//#define LOW_POWER_MODE

#ifdef LOW_POWER_MODE
#define PA_INTERVAL 0x320 // 1 second
#define SUBEVENT_INTERVAL 0xF0 // 300 ms  
#else
#define PA_INTERVAL 0xFF // 318.75 ms
#define SUBEVENT_INTERVAL 0x50 // 100 ms
#endif

#define NUM_SENSORS 3

// Sensor variables
static uint32_t sensor_temp_values[NUM_SENSORS] = {0};
static uint32_t sensor_hum_values[NUM_SENSORS] = {0};

// 13 bytes: 1 byte for length, 1 byte for type, 2 bytes for company ID, 1 byte for device ID, 1 byte for command, 8 bytes for sensor value
#define RESPONSE_DATA_SIZE 13

// We will be assigning each sensor node a subevent and response slot
// We will have a maximum of 3 sensor nodes
#define NUM_RSP_SLOTS 1
#define NUM_SUBEVENTS NUM_SENSORS
#define PACKET_SIZE   5
#define NAME_LEN      30

// Device Discovery Definitions
#define NOVEL_BITS_COMPANY_ID 0x08D3

typedef struct adv_data
{
    // Device Name
    char name[NAME_LEN];

    /* data */
    bool novelbits_id_present;
    uint8_t data[30];

} custom_adv_data_t;

// Initialize the currently connected device ID to 0xFF (Invalid ID)
static uint8_t currently_connected_device_id = 0xFF;

// PAwR Definitions
#define PAWR_CMD_REQUEST_TEMP 0x01
#define PAWR_CMD_REQUEST_HUMIDITY 0x02

static uint8_t current_pawr_command = PAWR_CMD_REQUEST_TEMP;

static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_discovered, 0, 1);
static K_SEM_DEFINE(sem_written, 0, 1);
static K_SEM_DEFINE(sem_disconnected, 0, 1);

static struct bt_uuid_128 pawr_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));
static uint16_t pawr_attr_handle;
static const struct bt_le_per_adv_param per_adv_params = {
    .interval_min = PA_INTERVAL,
    .interval_max = PA_INTERVAL,
    .options = 0, // No options
    .num_subevents = NUM_SUBEVENTS, // 3
    .subevent_interval = SUBEVENT_INTERVAL,
    .response_slot_delay = 0x5, // 6.25 ms
    .response_slot_spacing = 0xFA, // 31.25 ms
    .num_response_slots = NUM_RSP_SLOTS, // 1
};

#define DEVICE_NAME "Weather_Station"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

static struct bt_le_per_adv_subevent_data_params subevent_data_params[NUM_SUBEVENTS];
static struct net_buf_simple bufs[NUM_SUBEVENTS];
static uint8_t backing_store[NUM_SUBEVENTS][PACKET_SIZE];

BUILD_ASSERT(ARRAY_SIZE(bufs) == ARRAY_SIZE(subevent_data_params));
BUILD_ASSERT(ARRAY_SIZE(backing_store) == ARRAY_SIZE(subevent_data_params));

uint32_t quick_ieee11073_from_float(float temperature)
{
    uint8_t  exponent = 0xFE; //Exponent is -2
    uint32_t mantissa = (uint32_t)(temperature * 100);

    return (((uint32_t)exponent) << 24) | mantissa;
}

static struct bt_conn *central_conn;
static struct bt_conn *peripheral_conn;

static void request_cb(struct bt_le_ext_adv *adv, const struct bt_le_per_adv_data_request *request);
static void response_cb(struct bt_le_ext_adv *adv, struct bt_le_per_adv_response_info *info,
             struct net_buf_simple *buf);

static const struct bt_le_ext_adv_cb adv_cb = {
    .pawr_data_request = request_cb,
    .pawr_response = response_cb,
};

void connected_cb(struct bt_conn *conn, uint8_t err)
{

    printk("Connected (err 0x%02X)\n", err);

    // Success case
    if (err == 0) {
        if (conn != central_conn)
        {
            // Connected as a Peripheral
            printk("Connected as a Peripheral\n");
            peripheral_conn = bt_conn_ref(conn);
        }
    }
    // Failure Case
    else
    {
        if (conn == central_conn) {
            bt_conn_unref(central_conn);
            central_conn = NULL;
        }
        else if (conn == peripheral_conn) {
            bt_conn_unref(peripheral_conn);
            peripheral_conn = NULL;
        }
    }
}

void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02X)\n", reason);

    if (conn == central_conn) {
        bt_conn_unref(central_conn);
        central_conn = NULL;
        k_sem_give(&sem_disconnected);
        currently_connected_device_id = 0xFF;
    }
    else if (conn == peripheral_conn) {
        bt_conn_unref(peripheral_conn);
        peripheral_conn = NULL;
    }
}

void remote_info_available_cb(struct bt_conn *conn, struct bt_conn_remote_info *remote_info)
{
    /* Need to wait for remote info before initiating PAST  -- only as a Central*/

    if (conn == central_conn) {
        k_sem_give(&sem_connected);
    }
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .remote_info_available = remote_info_available_cb,
};

static bool data_cb(struct bt_data *data, void *user_data)
{
    custom_adv_data_t *adv_data_struct = user_data;
    uint8_t len;

    switch (data->type) {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
        len = MIN(data->data_len, NAME_LEN - 1);
        memcpy(adv_data_struct->name, data->data, len);
        adv_data_struct->name[len] = '\0';
        return true;

    case BT_DATA_MANUFACTURER_DATA:
        if (data->data_len < 3) {
            return true;
        }

        if (sys_get_le16(&(data->data[0])) != NOVEL_BITS_COMPANY_ID) {
            return true;
        }

        printk("Found Novel Bits Company ID\n");

        adv_data_struct->novelbits_id_present = true;
        memcpy(adv_data_struct->data, &data->data[0], data->data_len);

        return false;
    default:
        return true;
    }
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
             struct net_buf_simple *ad)
{
    int err;
    char addr_str[BT_ADDR_LE_STR_LEN];
    custom_adv_data_t device_ad_data;

    if (central_conn) {
        return;
    }

    /* We're only interested in connectable events */
    if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        return;
    }

    (void)memset(&device_ad_data, 0, sizeof(device_ad_data));
    bt_data_parse(ad, data_cb, &device_ad_data);

    if (!device_ad_data.novelbits_id_present) {
        return;
    }

    printk("Device found: %s (RSSI %d)\n", device_ad_data.name, rssi);
    printk("Manufacturer specific data [Novel Bits]. ID = 0x%02X\n", device_ad_data.data[2]);

    if (device_ad_data.data[2] > NUM_SENSORS)
    {
        printk("Invalid device ID\n");
        return;
    }

    if (bt_le_scan_stop()) {
        return;
    }

    err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
                &central_conn);
    if (err) {
        printk("Create conn to %s failed (%u)\n", addr_str, err);
    }

    currently_connected_device_id = device_ad_data.data[2];
}

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                 struct bt_gatt_discover_params *params)
{
    struct bt_gatt_chrc *chrc;
    char str[BT_UUID_STR_LEN];

    printk("Discovery: attr %p\n", attr);

    if (!attr) {
        return BT_GATT_ITER_STOP;
    }

    chrc = (struct bt_gatt_chrc *)attr->user_data;

    bt_uuid_to_str(chrc->uuid, str, sizeof(str));
    printk("UUID %s\n", str);

    if (!bt_uuid_cmp(chrc->uuid, &pawr_char_uuid.uuid)) {
        pawr_attr_handle = chrc->value_handle;

        printk("Characteristic handle: %d\n", pawr_attr_handle);

        k_sem_give(&sem_discovered);
    }

    return BT_GATT_ITER_STOP;
}

static void write_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    if (err) {
        printk("Write failed (err %d)\n", err);

        return;
    }

    k_sem_give(&sem_written);
}

void init_bufs(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(backing_store); i++) {
        backing_store[i][0] = ARRAY_SIZE(backing_store[i]) - 1;
        backing_store[i][1] = BT_DATA_MANUFACTURER_DATA;
        backing_store[i][2] = (NOVEL_BITS_COMPANY_ID & 0xFF); /* Novel Bits */
        backing_store[i][3] = ((NOVEL_BITS_COMPANY_ID >> 8) & 0xFF);

        net_buf_simple_init_with_data(&bufs[i], &backing_store[i],
                          ARRAY_SIZE(backing_store[i]));
    }
}

// GATT Definitions

// Strings for the User Descriptions
#define SENSOR_1_TEMP_DESCRIPTION "Sensor 1 Temperature"
#define SENSOR_1_HUM_DESCRIPTION "Sensor 1 Humidity"
#define SENSOR_2_TEMP_DESCRIPTION "Sensor 2 Temperature"
#define SENSOR_2_HUM_DESCRIPTION "Sensor 2 Humidity"
#define SENSOR_3_TEMP_DESCRIPTION "Sensor 3 Temperature"
#define SENSOR_3_HUM_DESCRIPTION "Sensor 3 Humidity"

static ssize_t read_sensor_1_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_temp_values[0], sizeof(uint32_t));
}

static ssize_t read_sensor_1_hum(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_hum_values[0], sizeof(uint32_t));
}

static ssize_t read_sensor_2_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_temp_values[1], sizeof(uint32_t));
}

static ssize_t read_sensor_2_hum(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_hum_values[1], sizeof(uint32_t));
}

static ssize_t read_sensor_3_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_temp_values[2], sizeof(uint32_t));
}

static ssize_t read_sensor_3_hum(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_hum_values[2], sizeof(uint32_t));
}

// Format for the temperature characteristic
static const struct bt_gatt_cpf temp_cpf = {
    .format =  0x17, // IEEE-11073 32-bit SFLOAT
    .unit = 0x272F,  // Temperature degrees C
};

// Format for the humidity characteristic
static const struct bt_gatt_cpf hum_cpf = {
    .format =  0x17, // IEEE-11073 32-bit SFLOAT
    .unit = 0x27AD,  // Percentage
};

// Service UUID: 12345678-1234-5678-1234-56789abcdef2
static struct bt_uuid_128 ws_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2));

// ----- Characteristic UUIDs -----
// Sensor 1 Temperature: 12345678-1234-5678-1234-56789abcdef3
static struct bt_uuid_128 ws_sensor_1_temp_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3));
// Sensor 1 Humidity: 12345678-1234-5678-1234-56789abcdef4
static struct bt_uuid_128 ws_sensor_1_hum_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4));

// Sensor 2 Temperature: 12345678-1234-5678-1234-56789abcdef5
static struct bt_uuid_128 ws_sensor_2_temp_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef5));
// Sensor 2 Humidity: 12345678-1234-5678-1234-56789abcdef6
static struct bt_uuid_128 ws_sensor_2_hum_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef6));

// Sensor 3 Temperature: 12345678-1234-5678-1234-56789abcdef7
static struct bt_uuid_128 ws_sensor_3_temp_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef7));
// Sensor 3 Humidity: 12345678-1234-5678-1234-56789abcdef8
static struct bt_uuid_128 ws_sensor_3_hum_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef8));

// --- Weather Station Service ---
// The handles are in the following sequence:
// Format: [INDEX INTO THE ATTRIBUTE TABLE] [ATTRIBUTE]
// [0] Weather Station Service
// [1] Sensor 1 Temperature characteristic declaration
// [2] Sensor 1 Temperature characteristic value
// [3] Sensor 1 Temperature characteristic client configuration descriptor (CCC)
// [4] Sensor 1 Temperature characteristic presentation format descriptor (CPF)
// [5] Sensor 1 Temperature characteristic user description (CUD)
// [6] Sensor 1 Humidity characteristic declaration
// [7] Sensor 1 Humidity characteristic value
// [8] Sensor 1 Humidity characteristic client configuration descriptor (CCC)
// [9] Sensor 1 Humidity characteristic presentation format descriptor (CPF)
// [10] Sensor 1 Humidity characteristic user description (CUD)
// [11] Sensor 2 Temperature characteristic declaration
// [12] Sensor 2 Temperature characteristic value
// [13] Sensor 2 Temperature characteristic client configuration descriptor (CCC)
// [14] Sensor 2 Temperature characteristic presentation format descriptor (CPF)
// [15] Sensor 2 Temperature characteristic user description (CUD)
// [16] Sensor 2 Humidity characteristic declaration
// [17] Sensor 2 Humidity characteristic value
// [18] Sensor 2 Humidity characteristic client configuration descriptor (CCC)
// [19] Sensor 2 Humidity characteristic presentation format descriptor (CPF)
// [20] Sensor 2 Humidity characteristic user description (CUD)
// [21] Sensor 3 Temperature characteristic declaration
// [22] Sensor 3 Temperature characteristic value
// [23] Sensor 3 Temperature characteristic client configuration descriptor (CCC)
// [24] Sensor 3 Temperature characteristic presentation format descriptor (CPF)
// [25] Sensor 3 Temperature characteristic user description (CUD)
// [26] Sensor 3 Humidity characteristic declaration
// [27] Sensor 3 Humidity characteristic value
// [28] Sensor 3 Humidity characteristic client configuration descriptor (CCC)
// [29] Sensor 3 Humidity characteristic presentation format descriptor (CPF)
// [30] Sensor 3 Humidity characteristic user description (CUD)
BT_GATT_SERVICE_DEFINE(
    ws_svc,
    
    // Simple Service
    BT_GATT_PRIMARY_SERVICE(&ws_service_uuid.uuid),

    // Sensor 1 Temperature Characteristic  [1,2]
    // Properties: Read
    BT_GATT_CHARACTERISTIC(&ws_sensor_1_temp_char_uuid.uuid,
                    BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_sensor_1_temp,
                    NULL,
                    &sensor_temp_values[0]),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CPF(&temp_cpf),
    BT_GATT_CUD(SENSOR_1_TEMP_DESCRIPTION, BT_GATT_PERM_READ),

    // Sensor 1 Humidity Characteristic  [6,7]
    // Properties: Read
    BT_GATT_CHARACTERISTIC(&ws_sensor_1_hum_char_uuid.uuid,
                    BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_sensor_1_hum,
                    NULL,
                    &sensor_hum_values[0]),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),    
    BT_GATT_CPF(&hum_cpf),
    BT_GATT_CUD(SENSOR_1_HUM_DESCRIPTION, BT_GATT_PERM_READ),

    // Sensor 2 Temperature Characteristic [11, 12]
    // Properties: Read
    BT_GATT_CHARACTERISTIC(&ws_sensor_2_temp_char_uuid.uuid,
                    BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_sensor_2_temp,
                    NULL,
                    &sensor_temp_values[1]),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CPF(&temp_cpf),
    BT_GATT_CUD(SENSOR_2_TEMP_DESCRIPTION, BT_GATT_PERM_READ),

    // Sensor 2 Humidity Characteristic [16,17]
    // Properties: Read
    BT_GATT_CHARACTERISTIC(&ws_sensor_2_hum_char_uuid.uuid,
                    BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_sensor_2_hum,
                    NULL,
                    &sensor_hum_values[1]),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CPF(&hum_cpf),
    BT_GATT_CUD(SENSOR_2_HUM_DESCRIPTION, BT_GATT_PERM_READ),

    // Sensor 3 Temperature Characteristic [21, 22]
    // Properties: Read
    BT_GATT_CHARACTERISTIC(&ws_sensor_3_temp_char_uuid.uuid,
                    BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_sensor_3_temp,
                    NULL,
                    &sensor_temp_values[2]),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CPF(&temp_cpf),
    BT_GATT_CUD(SENSOR_3_TEMP_DESCRIPTION, BT_GATT_PERM_READ),

    // Sensor 3 Humidity Characteristic [26,27]
    // Properties: Read
    BT_GATT_CHARACTERISTIC(&ws_sensor_3_hum_char_uuid.uuid,
                    BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_sensor_3_hum,
                    NULL,
                    &sensor_hum_values[2]),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CPF(&hum_cpf),
    BT_GATT_CUD(SENSOR_3_HUM_DESCRIPTION, BT_GATT_PERM_READ),
);

static void request_cb(struct bt_le_ext_adv *adv, const struct bt_le_per_adv_data_request *request)
{
    int err;
    uint8_t to_send;
    struct net_buf_simple *buf;

    to_send = MIN(request->count, ARRAY_SIZE(subevent_data_params));

    for (size_t i = 0; i < to_send; i++) {
        buf = &bufs[i];
        buf->data[buf->len - 1] = current_pawr_command;

        subevent_data_params[i].subevent =
            (request->start + i) % per_adv_params.num_subevents;
        subevent_data_params[i].response_slot_start = 0;
        subevent_data_params[i].response_slot_count = NUM_RSP_SLOTS;
        subevent_data_params[i].data = buf;
    }

    err = bt_le_per_adv_set_subevent_data(adv, to_send, subevent_data_params);
    if (err) {
        printk("Failed to set subevent data (err %d)\n", err);
    }
}

// Initial temperature characteristic declaration index
#define INITIAL_TEMP_CHAR_DEC_INDEX 1
// Initial humidity characteristic declaration index
#define INITIAL_HUM_CHAR_DEC_INDEX 6
// Offset between each characteristic declaration index
#define CHAR_DEC_INDEX_OFFSET 10

static void response_cb(struct bt_le_ext_adv *adv, struct bt_le_per_adv_response_info *info,
             struct net_buf_simple *buf)
{
    if (buf) {
        const struct sensor_value * sensor_val;

        printk("\n\nReceived Response in Subevent #%d, Response Slot #%d [Data length = %d bytes]\n", info->subevent, info->response_slot, buf->len);

        // Format of data is as follows:
        // [0] Length
        // [1] Type
        // [2] Company ID (LSB)
        // [3] Company ID (MSB)
        // [4] Device ID
        // [5] Command
        // [6-13] Sensor Value

        // Validate the data received:

        // 1. Make sure the length is correct
        if ((buf->len == RESPONSE_DATA_SIZE+1)
        // 2. Make sure the length is as expected
            && (buf->data[0] == RESPONSE_DATA_SIZE) 
        // 3. Make sure the type is Manufacturer Specific Data
            && buf->data[1] == 0xFF // Manufacturer Specific Data type (0xFF)
        // 4. Make sure the Company ID is Novel Bits
            && (buf->data[2] == (NOVEL_BITS_COMPANY_ID & 0xFF)) && (buf->data[3] == (NOVEL_BITS_COMPANY_ID >> 8)))
        {
            uint8_t device_id = buf->data[4];
            uint8_t command = buf->data[5];
            sensor_val = (const struct sensor_value *)&(buf->data[6]);

            // Check if the command is for temperature or humidity
            if (command == PAWR_CMD_REQUEST_TEMP)
            {
                int err;
                float temp_local = sensor_value_to_float(sensor_val);

                printk("\tSensor Data Type: Temperature Reading\n");
                sensor_temp_values[device_id-1] = quick_ieee11073_from_float(temp_local);  

                printk("\t---- SENSOR NODE #%d ----\n\tTemperature: %.2f °C\n", device_id, sensor_value_to_float(sensor_val));

                // Notify the connected device of the new temperature value

                // We then need to do a bit of math to determine where the temperature characteristic declaration is in the Attribute array
                // The index of the first Temperature characteristic declaration is 1, and the offset to the next one is 10
                // So, it would be as follows: Device ID 1 -> 1, Device ID 2 -> 11, Device ID 3 -> 21
                uint8_t temp_char_dec_index = INITIAL_TEMP_CHAR_DEC_INDEX + CHAR_DEC_INDEX_OFFSET*(device_id-1);

                // Check if the connected device is subscribed to the temperature characteristic
                if (peripheral_conn && bt_gatt_is_subscribed(peripheral_conn, 
                                                             &ws_svc.attrs[temp_char_dec_index], 
                                                             BT_GATT_CCC_NOTIFY))
                {
                    // Notify the connected device of the new temperature value
                    err = bt_gatt_notify(peripheral_conn, &ws_svc.attrs[temp_char_dec_index], &sensor_temp_values[device_id-1], sizeof(uint32_t));
                    if (err) {
                        printk("Failed to notify central (err %d)\n", err);
                    }
                }
            }
            else if (command == PAWR_CMD_REQUEST_HUMIDITY)
            {
                int err;
                float hum_local = sensor_value_to_float(sensor_val);

                printk("\tSensor Data Type: Relative Humidity Reading\n");
                sensor_hum_values[device_id-1] = quick_ieee11073_from_float(hum_local);

                printk("\t---- SENSOR NODE #%d ----\n\tHumidity: %0.2f %%\n", device_id, sensor_value_to_float(sensor_val));

                // Notify the connected device of the new humidity value

                // We then need to do a bit of math to determine where the humidity characteristic declaration is in the Attribute array
                // The index of the first Temperature characteristic declaration is 6, and the offset to the next one is 10
                // So, it would be as follows: Device ID 1 -> 6, Device ID 2 -> 16, Device ID 3 -> 26
                uint8_t hum_char_dec_index = INITIAL_HUM_CHAR_DEC_INDEX + CHAR_DEC_INDEX_OFFSET*(device_id-1);

                // Check if the connected device is subscribed to the humidity characteristic
                if (peripheral_conn && bt_gatt_is_subscribed(peripheral_conn, &ws_svc.attrs[hum_char_dec_index], BT_GATT_CCC_NOTIFY))
                {
                    err = bt_gatt_notify(peripheral_conn, &ws_svc.attrs[hum_char_dec_index], &sensor_hum_values[device_id-1], sizeof(uint32_t));
                    if (err) {
                        printk("Failed to notify central (err %d)\n", err);
                    }
                }
            }
            else
            {
                printk("Unknown request type received\n");
            }
        }
        else
        {
            printk("Invalid data format received\n");
        }
    }
    else 
    {
        printk("Failed to receive response: subevent %d, slot %d\n", info->subevent,
               info->response_slot);
    }
}

struct pawr_timing {
    uint8_t subevent;
    uint8_t response_slot;
} __packed;

#define USER_BUTTON DK_BTN1_MSK

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
    if (has_changed & USER_BUTTON) {
        uint32_t user_button_state = button_state & USER_BUTTON;

        if (user_button_state) {
            printk("Button pressed - changing to %s request\n", current_pawr_command == PAWR_CMD_REQUEST_TEMP ? "humidity" : "temperature");

            // Toggle the command between requesting temperature and humidity
            current_pawr_command = (current_pawr_command == PAWR_CMD_REQUEST_TEMP) ? PAWR_CMD_REQUEST_HUMIDITY : PAWR_CMD_REQUEST_TEMP;
        }
    }
}

static int init_button(void)
{
    int err;

    err = dk_buttons_init(button_changed);
    if (err) {
        printk("Cannot init buttons (err: %d)\n", err);
    }

    return err;
}

int main(void)
{
    int err;
    struct bt_le_ext_adv *pawr_adv;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_write_params write_params;
    struct pawr_timing sync_config;

    init_bufs();
    init_button();

    printk("Starting Periodic Advertising Demo\n");

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err && err != -EALREADY) {
        printk("Advertising failed to start (err %d) @ %d\n", err, __LINE__);

        return 0;
    }

    /* Create a non-connectable non-scannable advertising set */
    err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, &adv_cb, &pawr_adv);
    if (err) {
        printk("Failed to create advertising set (err %d)\n", err);
        return 0;
    }

    /* Set periodic advertising parameters */
    err = bt_le_per_adv_set_param(pawr_adv, &per_adv_params);
    if (err) {
        printk("Failed to set periodic advertising parameters (err %d)\n", err);
        return 0;
    }

    /* Enable Periodic Advertising */
    err = bt_le_per_adv_start(pawr_adv);
    if (err) {
        printk("Failed to enable periodic advertising (err %d)\n", err);
        return 0;
    }

    printk("Start Periodic Advertising\n");
    err = bt_le_ext_adv_start(pawr_adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        printk("Failed to start extended advertising (err %d)\n", err);
        return 0;
    }

    while (1) {
        // Continuously scan for devices
        err = bt_le_scan_start(BT_LE_SCAN_PASSIVE_CONTINUOUS, device_found);
        if (err) {
            printk("Scanning failed to start (err %d)\n", err);
            return 0;
        }

        printk("Scanning successfully started\n");

        k_sem_take(&sem_connected, K_FOREVER);

        err = bt_le_per_adv_set_info_transfer(pawr_adv, central_conn, 0);
        if (err) {
            printk("Failed to send PAST (err %d)\n", err);
        }
        else // PAST sent successfully
        {
            printk("PAST sent\n");
            
            discover_params.uuid = &pawr_char_uuid.uuid;
            discover_params.func = discover_func;
            discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
            discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
            discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            err = bt_gatt_discover(central_conn, &discover_params);
            if (err) {
                printk("Discovery failed (err %d)\n", err);
            }
            else // Discovery started successfully
            {
                printk("Discovery started\n");

                err = k_sem_take(&sem_discovered, K_SECONDS(10));
                if (err) {
                    printk("Timed out during GATT discovery\n");
                }
                else // Discovery completed successfully
                {
                    if (currently_connected_device_id == 0xFF) {
                        printk("No device ID found\n");
                    }
                    else // Device ID found
                    {
                        printk("Device ID: %d --> PAST sent for subevent %d and response slot %d\n",
                                currently_connected_device_id,
                                sync_config.subevent,
                                sync_config.response_slot);

                        // Give the device time to process the PAST and sync before writing the PAwR config
                        k_msleep(2*per_adv_params.interval_min);        

                        // Assign the subevent and response slot to the sync_config
                        // [Note: each node will be assigned a subevent based on their device ID. Response slot will be 0 for all nodes]
                        sync_config.subevent = currently_connected_device_id-1;
                        sync_config.response_slot = 0;

                        write_params.func = write_func;
                        write_params.handle = pawr_attr_handle;
                        write_params.offset = 0;
                        write_params.data = &sync_config;
                        write_params.length = sizeof(sync_config);

                        err = bt_gatt_write(central_conn, &write_params);
                        if (err) {
                            printk("Write failed (err %d)\n", err);
                        }
                        else // Write started successfully
                        {
                            printk("Write started\n");

                            err = k_sem_take(&sem_written, K_SECONDS(10));
                            if (err) {
                                printk("Timed out during GATT write\n");
                            }
                            else // Write completed successfully
                            {
                                printk("PAwR config written to device %d, disconnecting\n", currently_connected_device_id);
                            }
                        }
                    }
                }
            }
        }

        // Disconnect from the device
        err = bt_conn_disconnect(central_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        if (err) {
            printk("Failed to disconnect (err %d)\n", err);
            return 0;
        }

        printk("Disconnected\n");
        k_sem_take(&sem_disconnected,  K_SECONDS(30));
    }

    printk("SHOULD NEVER REACH HERE\n");

    return 0;
}
