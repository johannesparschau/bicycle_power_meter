#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <stdlib.h>

//------------------------- BT SERVICE SETUP ----------------------------------------
#define BT_UUID_CPS BT_UUID_DECLARE_16(0x1818)  // CPS 16-bit UUID
#define BT_UUID_CPS_MEASUREMENT BT_UUID_DECLARE_16(0x2A63)  // CPS Measurement

static uint8_t cycling_power_value[5];
static bool notifications_enabled = false;

/* Advertising Data (Include CPS UUID) */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x1818)),
};

/* CCC configuration change handler (for notifications) */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notifications %s\n", notifications_enabled ? "enabled" : "disabled");
}

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
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),  // CCC for notifications
);

/* Function to send cycling power notifications */
static void send_cycling_power_notification(void) {
    if (notifications_enabled) {
        bt_gatt_notify(NULL, &cps_svc.attrs[1], cycling_power_value, sizeof(cycling_power_value));
    }
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
    static uint16_t power = 100;
    static uint16_t cadence = 80;

    power = 50 + (rand() % 250);
    cadence = 80 + (rand() % 10);

    cycling_power_value[0] = 0x00;  // Flags (set to 0 for simplicity)
    cycling_power_value[1] = (uint8_t)(power & 0xFF);  // Power (LSB)
    cycling_power_value[2] = (uint8_t)((power >> 8) & 0xFF);  // Power (MSB)
    cycling_power_value[3] = (uint8_t)(cadence & 0xFF);  // Cadence (LSB)
    cycling_power_value[4] = (uint8_t)((cadence >> 8) & 0xFF);  // Cadence (MSB)
}

// --------------------------------- VOLTAGE READING -------------------------------------------------
/* SW1_NODE is the devicetree node identifier for the node with alias "sw1"= button 1 */
// TODO change to other pin where we get the voltage from
#define SW1_NODE DT_ALIAS(sw1)
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);

/* Read voltage from input pin, needs to be converted to digital signal and then send via btooth */
int read_voltage(void) {
    // TODO read voltage from pin defined above (for now button 1 = sw1)
    /* Read the status of the button and store it */
	bool val = gpio_pin_get_dt(&button1);
	printk("Reading voltage\n");
    return val;
};

// --------------------------------- INTERRUPT CONFIGS -----------------------------------------------

/* SW0_NODE is the devicetree node identifier for the node with alias "sw0"= button 0 */
#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

/* Define the callback function */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	read_voltage();
}
/* Define a variable of type static struct gpio_callback */
static struct gpio_callback button_cb_data;

// --------------------------------- MAIN APPLICATION ------------------------------------------------

/* Main function */
int main(void) {
    int err;

    srand(k_uptime_get());

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return -1;
    }

    /* Load settings, such as Bluetooth identity address */
    err = settings_load();
    if (err) {
        printk("Settings load failed (err %d)\n", err);
        return -1;
    }

    bt_ready();

    /* Init button for interrupt */
    if (!device_is_ready(button0.port)) {
		return -1;
	}

    err = gpio_pin_configure_dt(&button0, GPIO_INPUT);
	if (err < 0) {
		return -1;
	}

    /* Configure the interrupt on the button's pin */
	err = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);

	/* Initialize the static struct gpio_callback variable   */
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));

	/* Add the callback function by calling gpio_add_callback()   */
	gpio_add_callback(button0.port, &button_cb_data);


    while (1) {
        k_sleep(K_SECONDS(1));
        simulate_cycling_power_values();
        send_cycling_power_notification();
    }
}
