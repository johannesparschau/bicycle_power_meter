// Basics
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

// Zephyr ADC API
#include <zephyr/drivers/adc.h>

#include "../inc/defines.h"


/* Define a variable of type adc_dt_spec for each channel */
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

/* Define a variable of type adc_sequence and a buffer of type uint16_t to specify where the samples are to be written */
int16_t buf;
struct adc_sequence sequence = {
    .buffer = &buf,
    /* buffer size in bytes, not number of samples */
    .buffer_size = sizeof(buf),
    // Optional
    //.calibrate = true,
};


/* ------------------- ADC (VOLTAGE READING and CONVERSION -------------------- */
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

/* Convert mV to W */
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