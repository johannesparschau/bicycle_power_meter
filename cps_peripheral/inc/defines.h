#ifndef DEFINES_H
#define DEFINES_H

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

/* ------------------------- GLOBALS -------------------------- */

/* KALMAN FILTER: GLOBAL DEFINES FOR EASY ALGORITHM TUNING */
#define CALIB_COEFF 0.00216    // Calibration coeff for mV -> W conversion
#define KALMAN_X 0    // Initial state estimate (W)
#define KALMAN_P 1.0    // Initial covariance estimate
#define KALMAN_Q 0.01    // Process noise covariance
#define KALMAN_R 1.0    // Measurement noise covariance

static uint16_t global_cadence=0;
static uint8_t cycling_power_value[5];

#endif