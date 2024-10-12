// basics
#include <zephyr/types.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

// inputs
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

// bluetooth
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>

// Zephyr ADC API
#include <zephyr/drivers/adc.h>

//------------------------- ADC SETUP ----------------------------------------------

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

// --------------------------------- ADC VOLTAGE READING -------------------------------------------------
/* Define a variable of type adc_dt_spec for each channel */
// here only channel 0 so we can use ADC_DT_SPEC_GET() macro to get the io-channels at index 0
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

/* Read voltage from input pin and convert it to digital signal */
int read_voltage(void) {
    printk("Reading voltage\n");
    // Read the status of the button and store it to value
    // TODO read voltage from pin defined above (for now: button 1 = sw1)
    int err;
    int val_mv;
    uint32_t count = 0;

    /* Define a variable of type adc_sequence and a buffer of type uint16_t to specify where the samples are to be written */
	int16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
		// Optional
		//.calibrate = true,
	};

    /* Initialize the ADC sequence */
	err = adc_sequence_init_dt(&adc_channel, &sequence);
	if (err < 0) {
		//LOG_ERR("Could not initalize sequence");
        printk("Could not initalize sequence\n");
		return 0;
	}

    /* Read a sample from the ADC */
    err = adc_read(adc_channel.dev, &sequence);
    if (err < 0) {
        printk("Could not read (%d)\n", err);
        //LOG_ERR("Could not read (%d)", err);
        // continue;
    }

    val_mv = (int)buf;
    //LOG_INF("ADC reading[%u]: %s, channel %d: Raw: %d", count++, adc_channel.dev->name,
    //    adc_channel.channel_id, val_mv);
    printk("ADC reading[%u]: %s, channel %d: Raw: %d", count++, adc_channel.dev->name,
        adc_channel.channel_id, val_mv);
    

    /* Convert raw value to mV*/
    err = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
    /* conversion to mV may not be supported, skip if not */
    if (err < 0) {
        //LOG_WRN(" (value in mV not available)\n");
        printk(" (value in mV not available)\n");
    } else {
        //LOG_INF(" = %d mV", val_mv);
        printk(" = %d mV\n", val_mv);
    }
    return val_mv;
};

void voltage_to_power(int voltage) {
    // convert voltage to power
    // add it to cycling_power_value to be send via btooth
    // needs calibration function (?)
}

// --------------------------------- INTERRUPT CONFIGS -----------------------------------------------

/* SW0_NODE is the devicetree node identifier for the node with alias "sw0"= button 0 */
#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

/* Define the callback function */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	int vol = read_voltage();
    voltage_to_power(vol);
}

/* Define a variable of type static struct gpio_callback */
static struct gpio_callback button_cb_data;

// --------------------------------- MAIN APPLICATION ------------------------------------------------

/* Main function */
int main(void) {
    int err;

    /* -------------- BLUETOOTH -----------------*/
    srand(k_uptime_get());

    err = bt_enable(NULL);
    if (err) {
        //LOG_ERR("Bluetooth init failed (err %d)\n", err);
        printk("Bluetooth init failed (err %d)\n", err);
        return -1;
    }

    /* Load settings, such as Bluetooth identity address */
    err = settings_load();
    if (err) {
        //LOG_ERR("Settings load failed (err %d)\n", err);
        printk("Settings load failed (err %d)\n", err);
        return -1;
    }

    bt_ready();

    /* -------------- ADC -----------------*/

    /* Validate that the ADC peripheral (SAADC) is ready */
	if (!adc_is_ready_dt(&adc_channel)) {
        //LOG_ERR("ADC controller devivce %s not ready", adc_channel.dev->name)
		printk("ADC controller devivce %s not ready", adc_channel.dev->name);
		return -1;
	}

    /*  Setup the ADC channel */
	err = adc_channel_setup_dt(&adc_channel);
	if (err < 0) {
        //LOG_ERR("Could not setup channel #%d (%d)", 0, err);
		printk("Could not setup channel #%d (%d)", 0, err);
		return -1;
	}

    /* -------------- INTERRUPT -----------------*/
    /* Init button for interrupt */
    if (!device_is_ready(button0.port)) {
		return -1;
	}

    /* Configure button as input */
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
    return 0;
}
