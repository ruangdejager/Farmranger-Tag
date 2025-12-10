/*
 * Battery_Config.h
 *
 *  Created on: Dec 10, 2025
 *      Author: Ruan de Jager
 */

#ifndef WORKER_BATTERY_BATTERY_CONFIG_H_
#define WORKER_BATTERY_BATTERY_CONFIG_H_

//! Settling time (ms) after measurement enable (bias) before measurement is taken
#define BAT_MEAS_BIAS_SETTLE_TIME	20

//! Measurement avg factor
#define BAT_AVG_FACTOR 8

//! Measurement sample rate (s)
#define BAT_SAMPLE_INTERVAL		10

//! Voltage max delta between samples (mV)
#define BAT_SAMPLE_VALUE_DELTA_MAX	20.

// Might be board-specific
#define BAT_MEAS_ADC_M_NUM      4300
#define BAT_MEAS_ADC_M_DEN      1366  // 4096 / 3.0v
#define BAT_MEAS_ADC_C          0

#endif /* WORKER_BATTERY_BATTERY_CONFIG_H_ */
