#ifndef BLE_INTERFACE_H
#define BLE_INTERFACE_H

void ble_init(void);

void ble_start_advertising(void);

// Register callbacks for periodic adv sync
void ble_register_periodic_adv_sync_callbacks(void);

void ble_delete_per_adv_sync(void);

#endif