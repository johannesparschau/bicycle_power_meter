#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <zephyr/types.h>

/* Define a variable of type adc_sequence and a buffer of type uint16_t to specify where the samples are to be written */
int16_t buf;
static uint16_t global_cadence = 0; 

static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

struct adc_sequence sequence = {
    .buffer = &buf,
    /* buffer size in bytes, not number of samples */
    .buffer_size = sizeof(buf),
    // Optional
    .calibrate = true,
};

int read_voltage(void);
void voltage_to_power(int voltage_mv);

#endif