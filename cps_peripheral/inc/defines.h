#ifndef DEFINES_H
#define DEFINES_H

#include <stdio.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>

/* ------------------------- GLOBALS -------------------------- */
static uint16_t global_cadence = 0;
static uint8_t cycling_power_value[5];

/* KALMAN FILTER: GLOBAL DEFINES FOR EASY ALGORITHM TUNING */
#define CALIB_COEFF 0.00216    // Calibration coeff for mV -> W conversion
#define KALMAN_X 0    // Initial state estimate (W)
#define KALMAN_P 1.0    // Initial covariance estimate
#define KALMAN_Q 0.01    // Process noise covariance
#define KALMAN_R 1.0    // Measurement noise covariance

/* ------------------------ MUTEXES --------------------------- */
// start value 0, limit 1, same as: K_SEM_DEFINE(power_val_sem, 0, 1);
K_MUTEX_DEFINE(power_val_mutex); 
K_MUTEX_DEFINE(cadence_val_mutex);

/* Define what messages should be printed */
LOG_MODULE_REGISTER(cycling_power_meter, LOG_LEVEL_DBG);

#endif