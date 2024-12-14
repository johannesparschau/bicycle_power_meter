// basics
#include <zephyr/types.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <math.h>
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

//------------------------- DEFINE ----------------------------------------------
// #define BT_UUID_CPS BT_UUID_DECLARE_16(0x1818)  // CPS 16-bit UUID
#define BT_UUID_CPS_MEASUREMENT BT_UUID_DECLARE_16(0x2A63)  // CPS Measurement

/* KALMAN FILTER: GLOBAL DEFINES FOR EASY ALGORITHM TUNING */
#define CALIB_COEFF 0.00216    // Calibration coeff for mV -> W conversion
#define KALMAN_X 0    // Initial state estimate (W)
#define KALMAN_P 1.0    // Initial covariance estimate
#define KALMAN_Q 0.01    // Process noise covariance
#define KALMAN_R 1.0    // Measurement noise covariance

// Bluetooth threads
#define BLE_THREAD_STACK_SIZE 1024
#define BLE_THREAD_PRIORITY 3

// ADC threads
#define ADC_THREAD_STACK_SIZE 1024
#define ADC_THREAD_PRIORITY 1 // high priority to ensure consistent readings

// Cadence threads
#define CADENCE_THREAD_STACK_SIZE 1024
#define CADENCE_THREAD_PRIORITY -1 // high priority to ensure consistent readings

// TODO: Better comment
#define DEBOUNCE_DELAY_MS 200

/* SW0_NODE is the devicetree node identifier for the node with alias "sw0"= button 0 */
#define SW0_NODE DT_ALIAS(sw0)

//---------------------- K_DEFINE ----------------------------------------------

// Thread stack and priority definitions for ADC
K_THREAD_STACK_DEFINE(adc_thread_stack, ADC_THREAD_STACK_SIZE);

// Thread stack and priority definitions for BLE notification
K_THREAD_STACK_DEFINE(ble_thread_stack, BLE_THREAD_STACK_SIZE);

// Thread stack and priority definitions for Cadence
K_THREAD_STACK_DEFINE(cadence_thread_stack, CADENCE_THREAD_STACK_SIZE);
// K_THREAD_DEFINE(cadence_thread, 1024, cadence_entry, NULL, NULL, NULL, -1, 0, 0);

// Semaphore to signal the thread
K_SEM_DEFINE(adc_semaphore, 0, 1);

K_SEM_DEFINE(cadence_semaphore, 0, 1);


//---------------------- STRUCTS -------------------------------------------------

static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec signal_pin = {
    .dt_flags = GPIO_ACTIVE_HIGH,
    .pin = 5,
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0))
};

/* Advertising Data (Include CPS UUID) */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x1818)),
};

/* Define a variable of type adc_dt_spec for each channel */
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

/* Define a variable of type adc_sequence and a buffer of type uint16_t to specify where the samples are to be written */
int16_t buf;
struct adc_sequence sequence = {
    .buffer = &buf,
    /* buffer size in bytes, not number of samples */
    .buffer_size = sizeof(buf),
    // Optional
    .calibrate = true,
};

//Kalman filter
typedef struct {
    double x;  // State estimate
    double P;  // Estimate covariance
    double Q;  // Process noise covariance
    double R;  // Measurement noise covariance
    double K;  // Kalman gain
} KalmanFilter;

// Thread stack and priority definitions for adc
struct k_thread adc_thread_data;
k_tid_t adc_thread_id;

// Thread stack and priority definitions for BLE notification
struct k_thread ble_thread_data;
k_tid_t ble_thread_id;

// Thread stack and priority definitions for cadence
struct k_thread cadence_thread_data;
k_tid_t cadence_thread_id;

/* Define a variable of type static struct gpio_callback */
static struct gpio_callback button_cb_data;

//TODO: Not used. Check if needed
/* GPIO callback structure for interrupt signal*/
static struct gpio_callback interrupt_cb_data;

static struct k_timer debounce_timer;

//TODO: Find a place for this
volatile bool debounce_flag = false;

//-------------------------- ??? ---------------------------------------------------

//TODO: Find a place for this
/* Enable logs */
LOG_MODULE_REGISTER(cycling_power_meter, LOG_LEVEL_DBG);

//-------------------------- FUNCTION DECLARATIONS ----------------------------------

//BT service setup
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t read_cycling_power(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset);
static void send_cycling_power_notification(void);
void ble_notification_thread(void *arg1, void *arg2, void *arg3);

// BT connection setup
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void bt_ready(void);

// ADC voltage reading and conversion
int read_voltage(void);
void voltage_to_power(int voltage_mv);

// Kalman filtering
void kalman_init(KalmanFilter* kf, double init_x, double init_P, double Q, double R);
void kalman_predict(KalmanFilter* kf);
void kalman_update(KalmanFilter* kf, double measurement);
void adc_thread(void *arg1, void *arg2, void *arg3);

// Interrupt configs
void debounce_timer_handler(struct k_timer *timer_id);
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void cadence_entry(void *arg1, void *arg2, void *arg3);


// --------------------------------- MAIN ------------------------------------------------
int main(void) {
    int err;
    
    // -------------- THREADS --------------------
    // Initialize semaphore
    k_sem_init(&adc_semaphore, 0, 1);

    // Create ADC reading thread
    adc_thread_id = k_thread_create(&adc_thread_data, adc_thread_stack,
                                    K_THREAD_STACK_SIZEOF(adc_thread_stack),
                                    adc_thread,
                                    NULL, NULL, NULL,
                                    ADC_THREAD_PRIORITY, 0, K_NO_WAIT);
                                    
    // Create BLE notification thread
    ble_thread_id = k_thread_create(&ble_thread_data, ble_thread_stack,
                                    K_THREAD_STACK_SIZEOF(ble_thread_stack),
                                    ble_notification_thread,
                                    NULL, NULL, NULL,
                                    BLE_THREAD_PRIORITY, 0, K_NO_WAIT);

    cadence_thread_id = k_thread_create(&cadence_thread_data, cadence_thread_stack,
                                        K_THREAD_STACK_SIZEOF(cadence_thread_stack),
                                         cadence_entry,
                                         NULL, NULL, NULL,
                                         CADENCE_THREAD_STACK_SIZE, 0, K_NO_WAIT);

    /* -------------- BLUETOOTH -----------------*/
    // srand(k_uptime_get());

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return -1;
    }

    /* Load settings, such as Bluetooth identity address */
    err = settings_load();
    if (err) {
        LOG_ERR("Settings load failed (err %d)", err);
        return -1;
    }

    bt_ready();

    /* -------------- ADC -----------------*/

    /* Validate that the ADC peripheral (SAADC) is ready */
    if (!adc_is_ready_dt(&adc_channel)) {
        LOG_ERR("ADC controller device %s not ready", adc_channel.dev->name);
        return -1;
    }

    LOG_INF("ADC peripheral ready");

    /*  Setup the ADC channel */
    err = adc_channel_setup_dt(&adc_channel);
    if (err < 0) {
        LOG_ERR("Could not setup channel #%d (%d)", 0, err);
        return -1;
    }

    LOG_INF("ADC channel set up");

    /* Initialize the ADC sequence */
    err = adc_sequence_init_dt(&adc_channel, &sequence);
    if (err < 0) {
        LOG_ERR("Could not initialize sequence");
        return -1;
    }

    LOG_INF("ADC sequence initialized");

    /* -------------- INTERRUPT -----------------*/
    /* Init button for interrupt */
    if (!device_is_ready(button0.port)) {
        return -1;
    }

    LOG_INF("Button ready");

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
    
    /* Initialize the timer */
    k_timer_init(&debounce_timer, debounce_timer_handler, NULL);

    LOG_INF("GPIO interrupt routine set up");
    
    // Rising edge signal triggered interrupt
    
    if (!device_is_ready(signal_pin.port)) {
        LOG_ERR("Error: GPIO device %s is not ready", signal_pin.port->name);
        return -1;
    }

    /* Configure P0.05 as input with a rising edge interrupt */
    err = gpio_pin_configure_dt(&signal_pin, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    if (err < 0) {
        LOG_ERR("Error configuring GPIO pin");
        return -1;
    }

    err = gpio_pin_interrupt_configure_dt(&signal_pin, GPIO_INT_EDGE_RISING);
    if (err < 0) {
        LOG_ERR("Error configuring GPIO interrupt");
        return -1;
    }

    /* Initialize and add GPIO callback */
    gpio_init_callback(&interrupt_cb_data, interrupt_handler, BIT(signal_pin.pin));
    gpio_add_callback(signal_pin.port, &interrupt_cb_data);

    LOG_INF("Interrupt on P0.05 configured for rising edge");

    LOG_INF("Device completely set up and functional");

    return 0;
}

//------------------------- BT SERVICE SETUP ----------------------------------------
static uint8_t cycling_power_value[5];  // will be secured by mutex
static bool notifications_enabled = false;

/* Mutex for cycling_power_value and cadence */
K_MUTEX_DEFINE(power_val_mutex);  // start value 0, limit 1, same as: K_SEM_DEFINE(power_val_sem, 0, 1);
K_MUTEX_DEFINE(cadence_val_mutex);
K_MUTEX_DEFINE(last_press_time_mutex);

/* CCC configuration change handler (for notifications) */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications %s", notifications_enabled ? "enabled" : "disabled");
}

/* Read function for CPS (can be extended as needed) */
static ssize_t read_cycling_power(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset) {
    const uint8_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(cycling_power_value));
}

// TODO: Find a place for this
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
        if (k_mutex_lock(&power_val_mutex, K_MSEC(50)) != 0) {
            LOG_ERR("Input data not available!");
        } else {
            bt_gatt_notify(NULL, &cps_svc.attrs[1], cycling_power_value, sizeof(cycling_power_value));
            k_mutex_unlock(&power_val_mutex);
        }
    }
}

void ble_notification_thread(void *arg1, void *arg2, void *arg3) {
    while (1) {
        // Sleep for 1 second or adjust the interval as needed for power saving
        k_sleep(K_SECONDS(1));

        // Send BLE notification
        send_cycling_power_notification();
    }
}

//------------------------------ BT CONNECTION SETUP ----------------------------------------------

/* Callback function for Bluetooth connection */
static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
    } else {
        LOG_INF("Connected");
    }
}

/* Callback function for Bluetooth disconnection */
static void disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("Disconnected (reason 0x%02x)", reason);
}

//TODO: Find a place for this
/* Register connection callbacks */
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* Bluetooth ready callback */
static void bt_ready(void) {
    int err;

    LOG_INF("Bluetooth initialized");

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising successfully started");
}

// --------------------------------- ADC VOLTAGE READING AND CONVERSION -------------------------------------------------

/* Read voltage from input pin and convert it to digital signal */
int read_voltage(void) {
    int err;
    int val_mv;
    uint32_t count = 0;

    LOG_INF("Reading voltage");
    
    err = adc_read(adc_channel.dev, &sequence);
    
    if (err < 0) {
        LOG_ERR("Failed to read ADC: %d", err);
    }

    if (err < 0) {
        LOG_ERR("Failed to read ADC after retries, %d", err);
        return err;
    }

    val_mv = (int)buf;
    LOG_INF("ADC reading[%u]: %s, channel %d: Raw: %d", count++, adc_channel.dev->name,
            adc_channel.channel_id, val_mv);

    /* Convert raw value to mV */
    err = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
    /* Conversion to mV may not be supported, skip if not */
    if (err < 0) {
        LOG_WRN(" (value in mV not available)");
    } else {
        LOG_INF(" = %d mV", val_mv);
    }
    return val_mv;
}

// TODO: Figure where this belongs
static uint16_t global_cadence = 0; 

// Convert mV to W
void voltage_to_power(int voltage_mv) {
    uint16_t cadence;

    if (k_mutex_lock(&cadence_val_mutex, K_MSEC(50)) == 0) {
        cadence = global_cadence;  // Copy the global cadence while holding the mutex
        k_mutex_unlock(&cadence_val_mutex);
    } else {
        LOG_ERR("Failed to lock mutex for reading cadence value");
        return;
    }
    uint16_t power = voltage_mv * cadence * CALIB_COEFF;

    if (k_mutex_lock(&power_val_mutex, K_MSEC(50)) == 0) {
        cycling_power_value[0] = 0x00;
        cycling_power_value[1] = (uint8_t)(power & 0xFF);
        cycling_power_value[2] = (uint8_t)((power >> 8) & 0xFF);
        cycling_power_value[3] = (uint8_t)(cadence & 0xFF);
        cycling_power_value[4] = (uint8_t)((cadence >> 8) & 0xFF);
        k_mutex_unlock(&power_val_mutex);
    } else {
        LOG_ERR("Failed to lock mutex for updating power value");
    }
}

//------------------------------ Kalman Filtering -------------------------------------

void kalman_init(KalmanFilter* kf, double init_x, double init_P, double Q, double R) {
    kf->x = init_x;
    kf->P = init_P;
    kf->Q = Q;
    kf->R = R;
}

void kalman_predict(KalmanFilter* kf) {
    kf->P += kf->Q;
}

void kalman_update(KalmanFilter* kf, double measurement) {
    kf->K = kf->P / (kf->P + kf->R);
    kf->x += kf->K * (measurement - kf->x);
    kf->P *= (1 - kf->K);
}

void adc_thread(void *arg1, void *arg2, void *arg3) {
    KalmanFilter kf;
    kalman_init(&kf, KALMAN_X, KALMAN_P, KALMAN_Q, KALMAN_R);  // Initialize the Kalman filter with macros

    while (1) {
        // Wait for the button press signal
        k_sem_take(&adc_semaphore, K_FOREVER);

        // Perform ADC reading
        int voltage_mv = read_voltage();
        if (voltage_mv < 0) {
            voltage_mv = 0;  // convert negative voltage readings to 0
        }

        // Convert voltage to power
        voltage_to_power(voltage_mv);

        // Apply Kalman filter to the power reading
        if (k_mutex_lock(&power_val_mutex, K_MSEC(50)) == 0) {
            // Use current power value from `cycling_power_value` to get a raw power estimate
            uint16_t raw_power = (cycling_power_value[2] << 8) | cycling_power_value[1];
            
            // Predict and update steps for Kalman filter
            kalman_predict(&kf);
            kalman_update(&kf, (double)raw_power);
            
            // Store the filtered power estimate back in `cycling_power_value`
            uint16_t filtered_power = (uint16_t)round(kf.x);
            cycling_power_value[1] = (uint8_t)(filtered_power & 0xFF);
            cycling_power_value[2] = (uint8_t)((filtered_power >> 8) & 0xFF);

            k_mutex_unlock(&power_val_mutex);
        } else {
            LOG_ERR("Failed to lock mutex for updating power value");
        }
    }
}


// --------------------------------- INTERRUPT CONFIGS -----------------------------------------------

/* Timer Callback: Reset debounce flag */
void debounce_timer_handler(struct k_timer *timer_id) {
    debounce_flag = false;
}

// TODO: Move this?
static uint64_t last_button_time = 0;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (debounce_flag) {
        // Ignore if still debouncing
        return;
    }

    debounce_flag = true;
    k_timer_start(&debounce_timer, K_MSEC(DEBOUNCE_DELAY_MS), K_NO_WAIT);
    
    LOG_INF("Rising edge detected on P0.05");
    // Signal the ADC thread to start
    k_sem_give(&cadence_semaphore);
    k_sem_give(&adc_semaphore);
}

/* Interrupt handler function */
void interrupt_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (debounce_flag) {
        // Ignore if still debouncing
        return;
    }

    debounce_flag = true;
    k_timer_start(&debounce_timer, K_MSEC(DEBOUNCE_DELAY_MS), K_NO_WAIT);
    
    LOG_INF("Rising edge detected on P0.05");
    // Signal the ADC thread to start
    k_sem_give(&cadence_semaphore);
    k_sem_give(&adc_semaphore);
}

void cadence_entry(void *arg1, void *arg2, void *arg3)
{
    while(1){
        k_sem_take(&cadence_semaphore, K_FOREVER);
        uint64_t current_time = k_uptime_get();

        // Calculate cadence (RPM) based on time since last button press
        if (last_button_time != 0) {
            uint64_t delta_t = current_time - last_button_time;

            if (delta_t > 0) {
                uint16_t calculated_cadence = (60000 / delta_t);  // Calculating cadence (ms to min conversion)

                // Lock the cadence mutex before updating the global cadence
                if (k_mutex_lock(&cadence_val_mutex, K_MSEC(50)) == 0) {
                    global_cadence = calculated_cadence;
                    LOG_INF("Calculated Cadence: %d RPM", global_cadence);
                    k_mutex_unlock(&cadence_val_mutex);
                } else {
                    LOG_ERR("Failed to lock mutex for cadence update");
                }
            }
        }
        // Update the last button press time
        last_button_time = current_time;
    }
}

