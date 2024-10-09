#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
// For random power value testing
#include <stdlib.h>

//------------------------- BT SERVICE SETUP ----------------------------------------
/* Cycling Power Service UUID */
#define BT_UUID_CPS BT_UUID_DECLARE_16(0x1818)  // CPS 16-bit UUID
#define BT_UUID_CPS_MEASUREMENT BT_UUID_DECLARE_16(0x2A63)  // CPS Measurement

/* Cycling Power Measurement Value (instantaneous power, cadence, etc.) */
static uint8_t cycling_power_value[5];  // Adjusted to 5 bytes

/* Advertising Data (Include CPS UUID) */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x1818)),  // CPS UUID
};

/* Read function for CPS (can be extended as needed) */
static ssize_t read_cycling_power(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset) {
    const uint8_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(cycling_power_value));
}

/* Define the Cycling Power Service and Characteristics */
BT_GATT_SERVICE_DEFINE(cps_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CPS),
    BT_GATT_CHARACTERISTIC(BT_UUID_CPS_MEASUREMENT, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, read_cycling_power, NULL, cycling_power_value),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),  // CCC for notifications
);

/* Function to send cycling power notifications */
static void send_cycling_power_notification(void) {
    bt_gatt_notify(NULL, &cps_svc.attrs[1], cycling_power_value, sizeof(cycling_power_value));
}

//------------------------------ BT CONNECTION SETUP ----------------------------------------------

/* Callback function for Bluetooth connection */
static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("Connection failed (err 0x%02x)\n", err);
    } else {
        printk("Connected\n");
    }
}

/* Callback function for Bluetooth disconnection */
static void disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("Disconnected (reason 0x%02x)\n", reason);
}

/* Register connection callbacks */
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* Bluetooth ready callback */
static void bt_ready(void) {
    int err;

    printk("Bluetooth initialized\n");

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    printk("Advertising successfully started\n");
}

// ----------------------------- CYCLING POWER DEFINITIONS ------------------------------------------

/* Simulate more realistic, fluctuating power and cadence values */
static void simulate_cycling_power_values(void) {
    static uint16_t power = 100;    // Base power initialized to 100W
    static uint16_t cadence = 80;   // Base cadence initialized to 80 RPM

    /* Generate random variations during runtime */
    power = 100 + (rand() % 20);    // Fluctuate power around 100W, within 20W range
    cadence = 80 + (rand() % 10);   // Fluctuate cadence around 80RPM, within 10RPM range

    /* Cycling Power Measurement Packet Format */
    cycling_power_value[0] = 0x00;  // Flags (set to 0 for simplicity)
    cycling_power_value[1] = (uint8_t)(power & 0xFF);  // Power (LSB)
    cycling_power_value[2] = (uint8_t)((power >> 8) & 0xFF);  // Power (MSB)
    cycling_power_value[3] = (uint8_t)(cadence & 0xFF);  // Cadence (LSB)
    cycling_power_value[4] = (uint8_t)((cadence >> 8) & 0xFF);  // Cadence (MSB)
}

// --------------------------------- MAIN APPLICATION ------------------------------------------------

/* Main function */
void main(void) {
    int err;
    
    /* Initialize RNG with uptime value */
    srand(k_uptime_get());  // Seed RNG with current uptime in milliseconds

    /* Initialize Bluetooth */
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    bt_ready();

    /* Main loop */
    while (1) {
        /* Send cycling power notifications every second */
        k_sleep(K_SECONDS(1));
        
        /* Simulate cycling power values */
        simulate_cycling_power_values();
        
        /* Send power metrics to broadcast channel*/
        send_cycling_power_notification();
    }
}
