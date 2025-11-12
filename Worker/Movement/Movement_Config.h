/*
 * Movement_Config.h
 *
 *  Created on: Nov 11, 2025
 *      Author: Ruan de Jager
 */

#ifndef WORKER_MOVEMENT_MOVEMENT_CONFIG_H_
#define WORKER_MOVEMENT_MOVEMENT_CONFIG_H_

// Sensor health check #1: device ID
// The device ID is checked every 1s; 10 consecutive failures will result in a
// sensor health error alert and set the sensor error flag (cleared on reboot).
#define MOVE_SENSOR_ID_ERR_CNT_ALERT		10

// Sensor health check #2: no sample
// This error counter is increments every 1s, and reset every 1s at the longest,
// In case of no sample error, the counter will increment to 10 over 10s, when it
// will trigger a sensor reset. If the error persists, the counter will increment
// to 20 over the next 20s, when it will trigger an alert and set the sensor error
// flag (cleared on reboot).
#define MOVE_SENSOR_SAMPLE_ERR_CNT_RESET		10
#define MOVE_SENSOR_SAMPLE_ERR_CNT_ALERT		20

// Sensor health check #3: no delta
// In case of error, this counter increments every 40ms (25Hz). When it reaches
// 25*10s ten it will trigger a sensor reset. If the error persists, the counter
// will increment to 25*20s, when it will trigger an alert and set the sensor
// error flag (cleared at reboot)
#define MOVE_SENSOR_DELTA_ERR_CNT_RESET		(25*10)
#define MOVE_SENSOR_DELTA_ERR_CNT_ALERT		(25*20)

// Max number of move sensor alerts
#define MOVE_SENSOR_ALERT_COUNT_MAX		3

//! LIS3DH raw axis scale values
//! Note: it is assumed the LIS3DH is set to +-2g sensitivity
//! The scale factor is calculated so that the same resolution (g/bit) is provided
//! to the higher-level algorithm as with previous Versnelskaap versions
//! i.e. for Versnelskaap 6>:
//! - Freescale MMA7260: +-1.5g between 0V and VDD=3V (zero at VDD/2=1.5V) into
//!   8-bit ADC with Vref=3V
//!   - 11.7mg/bit
//! and for Versnelskaap 7:
//! - ST LIS3DH: +-2g with 16-bit ADC (e.g. +1g = 16384) with result in granularity/
//!   resolution of 64
//!   - 61ug/bit
//! So, for LIS3DH divide with 64 in order to get granularity of 1, yielding
//! 61u * 64 = 3.9mg/bit. Now we can compare MMA7260 and LIS3DH in mg/bit; LIS3DH
//! is 11.7/3.9 = 3 times more sensitive, resulting in 64 * 3 = 192 scale value.
#define MOVE_ACC_RAW_SCALE_VALUE	192

//! Movement algorithm DC buffer size
#define MOVE_ALG_DC_BUF_SIZE 32

//! Movement algorithm envelope buffer size
#define MOVE_ALG_ENV_BUF_SIZE 8

#define MOVE_SENSOR_AXIS_MIN		0b00000000
#define MOVE_SENSOR_AXIS_DEF		0b00000010
#define MOVE_SENSOR_AXIS_MAX		0b00000111

#define MOVE_ALARM_TRACKING_UPDATE_INTERVAL	30

//! Active time of day (min / def / max)
//! 32-bit field with [LSB..MSB] = [SUN_MORN,SUN_MID,SUN_AFT,SUN_NIGHT,MON_MORN..SAT_NIGHT,0,0,0,0]
//! calendar standard?
#define MOVE_ALARM_TIME_OF_DAY_ACT_MIN			UINT32_C(0)
//#define MOVE_ALARM_TIME_OF_DAY_ACT_DEF			UINT32_C(0x08888888)		// Night active for all days
#define MOVE_ALARM_TIME_OF_DAY_ACT_DEF			UINT32_C(0x0CCCCCCC)		// Afternoon + Night active for all days
//#define MOVE_ALARM_TIME_OF_DAY_ACT_DEF			UINT32_C(0b00000000000000000100000000000000)
//#define MOVE_ALARM_TIME_OF_DAY_ACT_DEF			UINT32_C(0b00001111111111111111111111111111)
#define MOVE_ALARM_TIME_OF_DAY_ACT_MAX			UINT32_C(0x0FFFFFFF)

//! Move alarm enabled (min / def / max)
#define MOVE_ALARM_SETTING_ENABLED_MIN		0
#define MOVE_ALARM_SETTING_ENABLED_DEF		40
#define MOVE_ALARM_SETTING_ENABLED_MAX		254

//! Move Window A time [seconds] (min / def / max)
//! Fixed values for now
#define MOVE_ALARM_WINDOW_A_TIME_MIN		300		// 5 mins
#define MOVE_ALARM_WINDOW_A_TIME_DEF		300
#define MOVE_ALARM_WINDOW_A_TIME_MAX		300

//! Move Window B time [minutes] (min / def / max)
//! Fixed values for now
#define MOVE_ALARM_WINDOW_B_TIME_MIN		10		// 10 mins
#define MOVE_ALARM_WINDOW_B_TIME_DEF		10
#define MOVE_ALARM_WINDOW_B_TIME_MAX		10

//! Move Window A max alarms (min / def / max)
//! Fixed values for now
//! Max number of alerts (mc, sms, app) for event
//! 1st and 2nd hold back are included
#define MOVE_HIGH_ACT_ALERTS_MAX_MIN		20
#define MOVE_HIGH_ACT_ALERTS_MAX_DEF		20
#define MOVE_HIGH_ACT_ALERTS_MAX_MAX		20



//#define MOVE_ALARM_ALERT_MIN		MOVE_ALARM_ALERT_HIGH_ACT_MC_SMS
//#define MOVE_ALARM_ALERT_DEF		MOVE_ALARM_ALERT_HIGH_ACT_MC_SMS
//#define MOVE_ALARM_ALERT_MAX		MOVE_ALARM_ALERT_HIGH_ACT_SMS

//! Move alarm max level setting (min / def / max)
#define MOVE_MAX_LEVEL_MIN					5
#define MOVE_MAX_LEVEL_DTZ0_DEF				30		//!< Morning
#define MOVE_MAX_LEVEL_DTZ1_DEF				30		//!< Midday
#define MOVE_MAX_LEVEL_DTZ2_DEF				25		//!< Afternoon
#define MOVE_MAX_LEVEL_DTZ3_DEF				25		//!< Night
#define MOVE_MAX_LEVEL_MAX					254

//! Move alarm max window setting (min / def / max)
#define MOVE_MAX_WINDOW_MIN					1
#define MOVE_MAX_WINDOW_DTZ0_DEF			15		//!< Morning
#define MOVE_MAX_WINDOW_DTZ1_DEF			15		//!< Midday
#define MOVE_MAX_WINDOW_DTZ2_DEF			15		//!< Afternoon
#define MOVE_MAX_WINDOW_DTZ3_DEF			15		//!< Night
#define MOVE_MAX_WINDOW_MAX					254

//! Move alarm hold-first-call setting (min / def / max)
#define MOVE_MAX_HOLD_FIRST_MIN				0
#define MOVE_MAX_HOLD_FIRST_DTZ0_DEF		1		//!< Morning
#define MOVE_MAX_HOLD_FIRST_DTZ1_DEF		1		//!< Midday
#define MOVE_MAX_HOLD_FIRST_DTZ2_DEF		1		//!< Afternoon
#define MOVE_MAX_HOLD_FIRST_DTZ3_DEF	  	1		//!< Night
#define MOVE_MAX_HOLD_FIRST_MAX				1

//! Move alarm hold-second-call setting (min / def / max)
#define MOVE_MAX_HOLD_SECOND_MIN			0
#define MOVE_MAX_HOLD_SECOND_DTZ0_DEF		1		//!< Morning
#define MOVE_MAX_HOLD_SECOND_DTZ1_DEF		1		//!< Midday
#define MOVE_MAX_HOLD_SECOND_DTZ2_DEF		1		//!< Afternoon
#define MOVE_MAX_HOLD_SECOND_DTZ3_DEF	  	0		//!< Night
#define MOVE_MAX_HOLD_SECOND_MAX			1

//! Move alarm min level setting (min / def / max)
#define MOVE_MIN_LEVEL_MIN					0
#define MOVE_MIN_LEVEL_DTZ0_DEF				3		//!< Morning
#define MOVE_MIN_LEVEL_DTZ1_DEF				3		//!< Midday
#define MOVE_MIN_LEVEL_DTZ2_DEF				3		//!< Afternoon
#define MOVE_MIN_LEVEL_DTZ3_DEF				3		//!< Night
// Note: for 9.5.x we misused the two MSBs for MOVE_MIN_LEVEL_MAX to manipulate
// geofence functionality; this turned out to be a massive risk when these units'
// FW was upgrades (ie. move min level was abnormally high, resulting units to
// be stuck in no movement state). For now limit this value to 63, which is a
// safe value; once all 9.5.x. FW has been removed from production units, then
// MOVE_MIN_LEVEL_MAX may be defined as 254
#define MOVE_MIN_LEVEL_MAX					0b00111111		// 63

//! Move alarm min window setting (min / def / max)
#define MOVE_MIN_WINDOW_MIN					1
#define MOVE_MIN_WINDOW_DTZ0_DEF			60		//!< Morning
#define MOVE_MIN_WINDOW_DTZ1_DEF			60		//!< Midday
#define MOVE_MIN_WINDOW_DTZ2_DEF			60		//!< Afternoon
#define MOVE_MIN_WINDOW_DTZ3_DEF			60		//!< Night
#define MOVE_MIN_WINDOW_MAX					254

//! Movement alarm tracking time [seconds]
#define MOVE_TRACKING_TIME_NIGHT		UINT16_C(15*60)		// Default tracking time for night is 15 minutes
#define MOVE_TRACKING_TIME_DAY		UINT16_C(5*60)		// Default tracking time for day is 5 minutes

//! Margin [s] to add to tracking window, allowing tracking extending on final tracking update (before tracking expired event)
#define MOVE_TRACKING_TIME_MARGIN	15


#endif /* WORKER_MOVEMENT_MOVEMENT_CONFIG_H_ */
