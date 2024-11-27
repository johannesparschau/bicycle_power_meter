#ifndef ALGORITHMS_H
#define ALGORITHMS_H

// VARIABLES
/* adc_dt_spec for each channel */
static const struct adc_dt_spec adc_channel;

/* adc_sequence and a buffer of type uint16_t to specify where the samples are to be written */
extern struct adc_sequence sequence;

// FUNCTIONS
int read_voltage(void);
void voltage_to_power(int voltage_mv);


#endif