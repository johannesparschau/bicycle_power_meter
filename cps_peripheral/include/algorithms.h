#ifndef ALGORITHMS_H
#define ALGORITHMS_H

/* Define a variable of type adc_sequence and a buffer of type uint16_t to specify where the samples are to be written */
int16_t buf;
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