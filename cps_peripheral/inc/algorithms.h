#ifndef ALGORITHMS_H
#define ALGORITHMS_H

// VARIABLES
static const struct adc_dt_spec adc_channel;
struct adc_sequence sequence;

// FUNCTIONS
int read_voltage(void);
void voltage_to_power(int voltage_mv);


#endif