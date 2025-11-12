/*
 * Movement.c
 *
 *  Created on: Nov 11, 2025
 *      Author: Ruan de Jager
 *
 * Notes on server missed call:
 * As legacy functionality, units made missed calls directly to users' mobile
 * phones; while this continues to prove to be an effective alarming mechanism,
 * it limits options in terms of SIM connectivity.
 *
 * An initial implementation of a server missed call was prototyped. For this
 * there were no changes unit-side; server-side a new event was created upon
 * movement alarm event + triggered state, which initiated the server missed
 * call (using Infobip). While the server call seemed to work well, an unwated
 * side effect was a long delay from excessive movement detection up until the
 * user receiving the server missed call; this was due to the extra time required
 * for GSM data network registration and GPS lock.
 *
 * It was decided that the unit should generate the server missed call. This would
 * mean:
 * - that the unit "knows" it should either make a direct mc, or a server mc.
 * - there would be a new state required for the movement alarm event.
 *
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hal_bsp.h"
#include "Movement.h"
#include "Movement_Driver.h"

#include "platform.h"

#include "dbg_log.h"
#include "math_func.h"


//! Movement algorithm axis info
typedef struct move_alg_axis_t {
	int8_t ai8DcBuf[MOVE_ALG_DC_BUF_SIZE];
	uint8_t u8DcBufIdx;
	int8_t i8DcLevel;
} move_alg_axis_t;

typedef struct acc_t
{
	move_alg_axis_t X;
	move_alg_axis_t Y;
	move_alg_axis_t Z;
	uint8_t au8EnvBuf[MOVE_ALG_ENV_BUF_SIZE];
	uint8_t u8EnvBufIdx;
	uint8_t u8EnvLevel;
//	uint8_t au8EnvLevelBuf[ACC_ENV_LEVEL_BUF_SIZE];
//	uint8_t u8EnvLevelBufInIdx;
//	uint8_t u8EnvLevelBufOutIdx;
} move_alg_t;
move_alg_t MoveAlg;

//! Scratchpad variables
uint8_t u8MoveTemp;
int16_t i16MoveTemp;

typedef enum _move_alarm_state_t {
	MOVE_ALARM_STATE_IDLE=0,		//! No high movement alarm active
	MOVE_ALARM_STATE_WINDOW_A=1,
	MOVE_ALARM_STATE_WINDOW_B=2		//! High movement tracking alarm (i.e. n/a for basic sub, event for premium sub
} move_alarm_state_t;

//! Health timer
//static ALARMS_alarm_t tMoveSensorHealthRtcAlarm = 0;
static uint8_t u8MoveSensorHealthErrorCnt;
static uint8_t u8MoveSensorHealthAlertCnt;

// Move alarm parameters (up to but excluding alarms)
typedef struct {
//	TIMERS_timer_t tTmr;		// Window A timer
	uint16_t u16TmrHwm;		// Window A timer high water mark
	uint8_t u8HoldBackCnt;		// Hold back setting at the time of the alarm
	uint8_t u8TriggerCnt;		// Trigger cnt, including hold backs
} move_window_a_t;
move_window_a_t MoveWinA;

// Holdback
#define MOVE_HOLDBACK_BUF_SIZE 16
typedef struct _move_holdback
{
	uint8_t u8Idx;
	move_holdback_event_t Events[MOVE_HOLDBACK_BUF_SIZE];
} move_holdback_t;
move_holdback_t MoveHoldback;


//// Make an "tracking update struct!"
//typedef struct {
//	uint32_t u32Id;		// Use timestamp
//	uint8_t u8Count;
//	uint32_t u32NextTimeStamp;
//	uint32_t u32WindowExpirationTime;
//} move_alarm_tracking_update_t;
//move_alarm_tracking_update_t MoveAlarmTrackingUpdate;

// Window timer for movement > MaxLevel
// Timebase is move sensor sample rate (e.g. 25Hz = 40ms)
uint16_t u16MoveMaxLevelWindowTmr;

// Window timer for movement < MinLevel
// Timebase is move sensor sample rate (e.g. 25Hz = 40ms)
uint32_t u32MoveMinLevelWindowTmr;

// Legacy totals to report
uint8_t u8MoveHighActivityAlarmCnt;		// Move alarm triggers
uint8_t u8MoveHighActAlarmSuccessCnt;		// Move alarm alerts successfully reported
uint8_t u8MoveNotActiveAlarmCnt;		// No-movement alarms

//! Macro to map an alarm active time of day to a 28-bit bitfield
#define MOVE_ALARM_TIME_OF_DAY_ACT_IDX(DayOfWeek, TimeOfDay)		((4*DayOfWeek)+(TimeOfDay))

time_of_day_t tTimeOfDayPrev;



//! Abnormal activity alarm buffer
//#define MOVE_ALARM_FIFO_SIZE		8		// Was 4, prior to server missed call
//typedef struct _move_alarm_fifo_t {
//	move_alarm_t Buf[MOVE_ALARM_FIFO_SIZE];
//	uint8_t u8Head;
//	uint8_t u8Tail;
//} move_alarm_fifo_t;
//
//move_alarm_fifo_t xMoveAlarmFifo;
//uint8_t u8MoveAlarmFifoHeadTmp;


//! No movement alarm buffer
#define MOVE_NOT_ALARM_FIFO_SIZE		4		// For set-restore combo
typedef struct _move_not_alarm_fifo_t {
	move_not_alarm_t Buf[MOVE_NOT_ALARM_FIFO_SIZE];
	uint8_t u8Head;
	uint8_t u8Tail;
} move_not_alarm_fifo_t;

move_not_alarm_fifo_t MoveNotAlarmFifo;


//! Scratchpad variables
uint8_t u8MoveTemp;
uint32_t u32MoveTemp;

//! Coroutine context variable
ccrContext ccrMoveTestTask;

// Sensor health error
// We continuously check for 3 types of sensor health errors:
//
// 1) Device ID read failure: Every second we read the device ID; if this fails
//    on (typically) 8 consecutive samples, then we immediately generate an alert.
// 2) Sample delta error: Every sample (25Hz) we compare the current sample with the
//    previous sample; if there is no variation for (typically) 30s (=30*25 samples),
//    then we reset the sensor; if there is no variation for (typically) 60s (=60*25
//    samples), then we generate an alert.
// 3) No sample error: Every second we check for new samples (we expect ~25 sammples
//    per second); if there are no new samples, then we reset the sensor; if there is
//    still no new samples, then we generate an alert.
typedef struct _acc_health_t {
	uint8_t u8DevIdErrorSeqCnt;		// Device ID error sequential counter (test every 1s)
	uint16_t u16SampleDeltaErrorCnt;		// Sample delta error sequential counter (tested every sample)
	uint8_t u8NoSampleErrorCnt;		// No sample error sequential counter (tested every 1s)
	acc_t AccPrevSample;		// Previous sample (used for u16SampleDeltaErrorCnt)
	bool bErrorFlag;		// Combined error flag
	uint8_t u8AlertCnt;		// Cumulative alert count (clears on reboot)
	time_of_day_t tAlertTimeOfDayPrev;		// Store prev time of day; used to trigger alert
	bool bAlertFlag;		// Alert flag
} acc_health_t;
acc_health_t AccHealth;

//! Struct to store previous ACC data sample for data delta check
//lis3dh_acc_t AccMoveDevPrev;

//! Health error counters
// uint16_t u16AccHealthDevIdErrSec;
// uint16_t u16AccHealthDataDeltaErrSamples;

//uint8_t u8AccHealthAlertCnt;

//! Health error alert
//bool bAccHealthErrorAlertFlag;

//! Time of day
//time_of_day_t tAccHealthErrAlertTimeOfDayPrev;

//! Local prototypes
//holdback_event_t xholdbackEvents[HOLDBACK_STORE];
//uint8_t xholdback_index = 0;


//! Descriptions for no movement alarm states
const char acMoveNoActAlarmStateIdle [] = "IDLE";
const char acMoveNoActAlarmStateTriggered [] = "TRIGGERED";
const char acMoveNoActAlarmStateCancelled [] = "CLEARED";
const char * const acMoveNoActAlarmState [] = {acMoveNoActAlarmStateIdle, acMoveNoActAlarmStateTriggered, acMoveNoActAlarmStateCancelled};

bool MOVE_bUpdateMovementLevel(uint8_t * pu8Level);
void MOVE_vEvalMovementLevel(void);
void MOVE_vWindowATick(void);

/**
 * Init module
 *
 */
void MOVE_vInit(void)
{
    //! Movement algorithm
    MoveAlg.X.u8DcBufIdx = 0;
    MoveAlg.Y.u8DcBufIdx = 0;
    MoveAlg.Z.u8DcBufIdx = 0;
    MoveAlg.u8EnvBufIdx = 0;

    ccrMoveTestTask = 0;

    // Abnormal activity alarm buffer
//    xMoveAlarmFifo.u8Head = 0;
//    xMoveAlarmFifo.u8Tail = 0;

    MoveNotAlarmFifo.u8Head = 0;
    MoveNotAlarmFifo.u8Tail = 0;

    // Move eval
    u16MoveMaxLevelWindowTmr = 0;       // 25*(uint16_t)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_WINDOW);
#ifndef SWITCH_POSITION_LOGGER
    // Nominal case: Setup minimum window from setting
    u32MoveMinLevelWindowTmr = 60 * 25 * (uint32_t)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MIN_WINDOW);
#else
    // Switch case: Force minimum window to 60 minutes
    u32MoveMinLevelWindowTmr = 60 * 25 * 60;
#endif

	// HWM
//	u16MoveMaxLevelWindowTmrHwm = 0;

    // Totals
    u8MoveHighActivityAlarmCnt = 0;
    u8MoveHighActAlarmSuccessCnt = 0;
    u8MoveNotActiveAlarmCnt = 0;

    // Window A
	memset(&MoveWinA, 0, sizeof(move_window_a_t));


    tTimeOfDayPrev = TIME_tGetDayTimeZoneNow();

    //! Health timer
//    ALARMS_bAlarmCreate(&tMoveSensorHealthRtcAlarm, TIMER_RTC_TICK);
    u8MoveSensorHealthErrorCnt = 0;
    u8MoveSensorHealthAlertCnt = 0;

	// Sensor health
	memset(&AccHealth, 0, sizeof(acc_health_t));
	AccHealth.tAlertTimeOfDayPrev = TIME_tGetDayTimeZoneNow();
	// Clear any previous error
//	SYSTEM_vClearHealthStatusError(MoMessage_Event_SystemHealthUpdateParameter_SystemHealthUpdateState_MOVE_SENSOR_ERROR);
}



/**
 * @brief	Main module tick.
 *
 * The tick schedules the following functions:
 * - Sample movement and implement movement algorithm (incl HPF); generate alarms
 * - Run state machine for movement alarm windows
 */
bool MOVE_bTick(void)
{
	bool bBusy = false;

	// Sample movement and implement movement algorithm (incl HPF); generate alarms
	MOVE_vEvalMovementLevel();

	// Run state machine for movement alarm windows
	MOVE_vWindowATick();

	// 1s tick
//	if (ALARMS_bAlarmGet(&tMoveSensorHealthRtcAlarm)) {
		// Health check #1: Device ID
		if ( AccHealth.u8DevIdErrorSeqCnt < MOVE_SENSOR_ID_ERR_CNT_ALERT ) {
			// Seq error limit not reached; attempt to read device ID
			if (MOVE_DRIVER_u8GetAccelDeviceId() != ACC_WHO_AM_I_VALUE) {
				// Cannot read ID; incr error cnt
				AccHealth.u8DevIdErrorSeqCnt++;
				//
				if ( AccHealth.u8DevIdErrorSeqCnt==MOVE_SENSOR_ID_ERR_CNT_ALERT && !AccHealth.bErrorFlag ) {
					AccHealth.bErrorFlag = true;
					DBG("+++ MOVE SENSOR ERROR (DEVICE_ID) - alert +++");
				}
			}
			else {
				// Read ID OK; clear seq error cnt
				AccHealth.u8DevIdErrorSeqCnt = 0;
			}
		}

		// Health check #2: No sample error
		if ( AccHealth.u8NoSampleErrorCnt < MOVE_SENSOR_SAMPLE_ERR_CNT_ALERT ) {
			AccHealth.u8NoSampleErrorCnt++;
			if (AccHealth.u8NoSampleErrorCnt == MOVE_SENSOR_SAMPLE_ERR_CNT_RESET) {
				// Reset device
				ACC_vInit();
				DBG("+++ MOVE SENSOR ERROR (NO_SAMPLE) - reset +++");
			}
			// Set error flag if not set yet
			if ( AccHealth.u8NoSampleErrorCnt==MOVE_SENSOR_SAMPLE_ERR_CNT_ALERT && !AccHealth.bErrorFlag ) {
				AccHealth.bErrorFlag = true;
				DBG("+++ MOVE SENSOR ERROR (NO_SAMPLE) - alert +++");
			}
		}

		// Health check #3 is done at sample level

		// Sensor health alerts
		if (
		( AccHealth.bErrorFlag && AccHealth.u8AlertCnt==0 )	// 1st alert
		|| ( AccHealth.u8AlertCnt>0 && TIME_tGetDayTimeZoneNow()==TIME_OF_DAY_MIDDAY && AccHealth.tAlertTimeOfDayPrev==TIME_OF_DAY_MORNING && AccHealth.u8AlertCnt<MOVE_SENSOR_ALERT_COUNT_MAX )		// 2nd+ alert
		) {
			AccHealth.u8AlertCnt++;		// Incr first, so will show 1..3 and not 0..2
			AccHealth.bAlertFlag = true;		// Queue alert
			DBG("+++ MOVE SENSOR HEALTH ALARM (%u of %u) +++", AccHealth.u8AlertCnt, MOVE_SENSOR_ALERT_COUNT_MAX);
		}

		// Update time of day for next time
		AccHealth.tAlertTimeOfDayPrev = TIME_tGetDayTimeZoneNow();
//	}

	// Check for change in dtz, and force no act window update
	if (tTimeOfDayPrev != TIME_tGetDayTimeZoneNow())
	{
		// Only update if not in no move state (else we repeat no move alarm)
		if (u32MoveMinLevelWindowTmr)
		{
#ifndef SWITCH_POSITION_LOGGER
			// Nominal case: Setup minimum window from setting
			u32MoveMinLevelWindowTmr = 60 * 25 * (uint32_t)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MIN_WINDOW);
#else
			// Switch case: Force minimum window to 60 minutes
			u32MoveMinLevelWindowTmr = 60 * 25 * 60;
#endif
		}
		// Update dtz
		tTimeOfDayPrev = TIME_tGetDayTimeZoneNow();
	}

	// Always return false
	return bBusy;
}

/*
move_sensor_error_alert_t MOVE_tSensorHealthAlertPeek(void)
{
	if (u8MoveSensorHealthAlertCnt == MOVE_SENSOR_HEALTH_ALERT_COUNT) {
		return MOVE_SENSOR_ERROR_ALERT_PRIM;
	}
	else if (u8MoveSensorHealthAlertCnt) {
		return MOVE_SENSOR_ERROR_ALERT_SEC;
	}
	else {
		return MOVE_SENSOR_ERROR_ALERT_NONE;
	}
}
*/
bool MOVE_bSensorHealthAlertPeek(void)
{
	return AccHealth.bAlertFlag;
}

/*
move_sensor_error_alert_t MOVE_tSensorHealthAlertPull(void)
{
	move_sensor_error_alert_t tTemp;

	tTemp = MOVE_tSensorHealthAlertPeek();
	if (tTemp!=MOVE_SENSOR_ERROR_ALERT_NONE) {
		u8MoveSensorHealthAlertCnt--;
	}
	return tTemp;
}
*/
bool MOVE_bSensorHealthAlertPull(void)
{
	bool bTemp = AccHealth.bAlertFlag;
	AccHealth.bAlertFlag = false;
	return bTemp;
}

bool MOVE_bSensorHealthOk(void)
{
	return !AccHealth.bErrorFlag;
}



/**
 * Main algorithm to sample movement sensor and update the 3-axis movement level
 *
 * Check & sample the ACC movement sensor, and implement the HPF for movement
 * by calculating moving avg levels.
 *
 * @param	uint8_t *	Ptr to buffer for movement level sample
 * @returns	bool	True if new sample(s) available, false if not
 */
bool MOVE_bUpdateMovementLevel(uint8_t * pu8Level)
{
	uint8_t u8;
	acc_t AccMoveDev;
	//	uint8_t u8NumSamplesInFifo;

	// Read all bytes from FIFO (read until empty)
	if ( ACC_u8NumSamplesInFifo() > 0 ) {
		// Clear counter to detect no samples error
		AccHealth.u8NoSampleErrorCnt = 0;

		// Samples available in ACC FIFO; read next sample
		ACC_vGetAccSample(&AccMoveDev);

		// Scale ACC raw value to be same as previous versions of Versnelskaap
		AccMoveDev.i16.i16OutX /= MOVE_ACC_RAW_SCALE_VALUE;
		AccMoveDev.i16.i16OutY /= MOVE_ACC_RAW_SCALE_VALUE;
		AccMoveDev.i16.i16OutZ /= MOVE_ACC_RAW_SCALE_VALUE;
		// Raw values are now -170..170 (-2g..+2g); limit raw values to -127..+128
		// Note that we can't AND-mask, as this does not map correct
		AccMoveDev.i16.i16OutX = min(AccMoveDev.i16.i16OutX, 127);
		AccMoveDev.i16.i16OutX = max(AccMoveDev.i16.i16OutX, -128);
		AccMoveDev.i16.i16OutY = min(AccMoveDev.i16.i16OutY, 127);
		AccMoveDev.i16.i16OutY = max(AccMoveDev.i16.i16OutY, -128);
		AccMoveDev.i16.i16OutZ = min(AccMoveDev.i16.i16OutZ, 127);
		AccMoveDev.i16.i16OutZ = max(AccMoveDev.i16.i16OutZ, -128);

		// Update DC buffers
		u8 = SETTINGS_u8GetByte(movementAlarm.accelerometerAxes);
		MoveAlg.X.ai8DcBuf[MoveAlg.X.u8DcBufIdx++] = ( (u8 & 0x01) ? (int8_t)AccMoveDev.i16.i16OutX : 0);
		MoveAlg.X.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;
		MoveAlg.Y.ai8DcBuf[MoveAlg.Y.u8DcBufIdx++] = ( (u8 & 0x02) ? (int8_t)AccMoveDev.i16.i16OutY : 0);
		MoveAlg.Y.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;
		MoveAlg.Z.ai8DcBuf[MoveAlg.Z.u8DcBufIdx++] = ( (u8 & 0x04) ? (int8_t)AccMoveDev.i16.i16OutZ : 0);
		MoveAlg.Z.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;
		// Calculate DC value for each axis
		i16MoveTemp = 0;
		for (u8MoveTemp=0; u8MoveTemp<MOVE_ALG_DC_BUF_SIZE; u8MoveTemp++) i16MoveTemp += MoveAlg.X.ai8DcBuf[u8MoveTemp];
		MoveAlg.X.i8DcLevel = (i16MoveTemp/MOVE_ALG_DC_BUF_SIZE);
		i16MoveTemp = 0;
		for (u8MoveTemp=0; u8MoveTemp<MOVE_ALG_DC_BUF_SIZE; u8MoveTemp++) i16MoveTemp += MoveAlg.Y.ai8DcBuf[u8MoveTemp];
		MoveAlg.Y.i8DcLevel = (i16MoveTemp/MOVE_ALG_DC_BUF_SIZE);
		i16MoveTemp = 0;
		for (u8MoveTemp=0; u8MoveTemp<MOVE_ALG_DC_BUF_SIZE; u8MoveTemp++) i16MoveTemp += MoveAlg.Z.ai8DcBuf[u8MoveTemp];
		MoveAlg.Z.i8DcLevel = (i16MoveTemp/MOVE_ALG_DC_BUF_SIZE);
		// Calculate vector displacement from DC
		u8 = SETTINGS_u8GetByte(movementAlarm.accelerometerAxes);
		MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx] = (uint8_t) abs((int8_t) ( (u8 & 0x01) ? (int8_t)AccMoveDev.i16.i16OutX : 0) - (int8_t) MoveAlg.X.i8DcLevel);
		MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx] += (uint8_t) abs((int8_t) ( (u8 & 0x02) ? (int8_t)AccMoveDev.i16.i16OutY : 0) - (int8_t) MoveAlg.Y.i8DcLevel);
		MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx] += (uint8_t) abs((int8_t) ( (u8 & 0x04) ? (int8_t)AccMoveDev.i16.i16OutZ : 0) - (int8_t) MoveAlg.Z.i8DcLevel);
		MoveAlg.u8EnvBufIdx++;
		MoveAlg.u8EnvBufIdx %= MOVE_ALG_ENV_BUF_SIZE;
		// Avg envelope
		i16MoveTemp = 0;
		for (u8MoveTemp=0; u8MoveTemp<MOVE_ALG_ENV_BUF_SIZE; u8MoveTemp++) i16MoveTemp += MoveAlg.au8EnvBuf[u8MoveTemp];
		MoveAlg.u8EnvLevel = (uint8_t) (i16MoveTemp/MOVE_ALG_ENV_BUF_SIZE);

		// Copy to target ptr
		*pu8Level = MoveAlg.u8EnvLevel;

		// Output to USB terminal
//		USB_HID_bPutMove(
//			(int8_t)AccMoveDev.i16.i16OutX,
//			(int8_t)AccMoveDev.i16.i16OutY,
//			(int8_t)AccMoveDev.i16.i16OutZ,
//			MoveAlg.u8EnvLevel,
//			MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_LEVEL),
//			MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MIN_LEVEL)
//		);

		// Sensor health check #3: no delta error
		// Check for no change in data error; do at this point with scaled-down samples, as we saw "noise" with raw samples
		// Compare current sample (3-axis, 16-bit) with previous sample (just run through bytes, break from loop on diff)
		for (u8=0; u8<sizeof(acc_t); u8++) {
			if ( AccHealth.AccPrevSample.au8Data[u8]!=AccMoveDev.au8Data[u8]) {
				memcpy(&AccHealth.AccPrevSample, &AccMoveDev, sizeof(acc_t));		// Store as prev
				AccHealth.u16SampleDeltaErrorCnt = 0;		// Reset error sample counter
				break;
			}
		}
		if ( u8==sizeof(acc_t) && AccHealth.u16SampleDeltaErrorCnt < MOVE_SENSOR_DELTA_ERR_CNT_ALERT ) {
			AccHealth.u16SampleDeltaErrorCnt++;
			if (AccHealth.u16SampleDeltaErrorCnt==MOVE_SENSOR_DELTA_ERR_CNT_RESET) {
				ACC_vInit();
				DBG("+++ MOVE SENSOR ERROR (NO_DELTA) - reset +++");
			}
			else if (AccHealth.u16SampleDeltaErrorCnt==MOVE_SENSOR_DELTA_ERR_CNT_ALERT && !AccHealth.bErrorFlag ) {
				AccHealth.bErrorFlag = true;
				DBG("+++ MOVE SENSOR ERROR (NO_DELTA) - alert +++");
			}
			// else no change in data alert has triggered
		}

		return true;
	}
	else {
		// No samples in ACC FIFO
		return false;
	}
}

#ifdef SWITCH_MOVE_SENSOR_DATA
int8_t MOVE_i8SensorDataX(void)
{
	return (int8_t)AccHealth.AccPrevSample.i16.i16OutX;
}

int8_t MOVE_i8SensorDataY(void)
{
	return (int8_t)AccHealth.AccPrevSample.i16.i16OutY;
}

int8_t MOVE_i8SensorDataZ(void)
{
	return (int8_t)AccHealth.AccPrevSample.i16.i16OutZ;
}

#endif

bool MOVE_bSetSensorAxis(uint8_t u8AxisMask)
{
	if (u8AxisMask & 0x07) {
		SETTINGS_vSetByte(movementAlarm.accelerometerAxes, u8AxisMask);
		return true;
	}
	else {
		return false;
	}
}



uint8_t MOVE_u8HighActivityAlarmCnt(void)
{
	return (uint8_t) u8MoveHighActivityAlarmCnt;
}

uint8_t MOVE_u8HighActAlarmSuccessCnt(void)
{
	return u8MoveHighActAlarmSuccessCnt;
}

void MOVE_vHighActAlarmSuccessIncr(void)
{
	u8MoveHighActAlarmSuccessCnt++;
}


uint8_t MOVE_u8NotActiveAlarmCnt(void)
{
	return (uint8_t) u8MoveNotActiveAlarmCnt;
}

/*
bool MOVE_bAlarmEventStateIdle(void)
{
	return (MoveAlarmEvent.tState == MOVE_ALARM_STATE_IDLE);
}
*/




/**
 * Check movement levels, and queue alarms
 *
 * Call MOVE_bUpdateMovementLevel() to update the movement level, and then eval
 * level in order to generate and queue movement and no-movement alarms.
 *
 * @note	Typically scheduled in module tick
 */
void MOVE_vEvalMovementLevel(void)
{
    uint8_t u8Level;

    // Get acc level (new acc level data becomes available at sample rate of acc,
    // i.e. 25Hz); note that we consume ACC_bGetLevel() output all times of day.
    while (MOVE_bUpdateMovementLevel(&u8Level)) {
        // Check out >MaxLevel and <MinLevel separately
        if ( u8Level >= MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_LEVEL) )
        {
        	// Check if move alarm is active for day time zone
            if ( MOVE_bIsAlarmActiveNow() ) {
                // LED on
//                if ( DIAGNOSTICS_vEE_GetWord(ee_ledsActive)==LEDS_ACTIVE_ENABLE )
//                {
//               		MOVE_DRIVER_vLedOn();
//               	}
                // Increment time in MaxWindow
                u16MoveMaxLevelWindowTmr++;
                // Update potential HWM for Window A
                MoveWinA.u16TmrHwm = max(MoveWinA.u16TmrHwm, u16MoveMaxLevelWindowTmr);
                // Check our >MaxWindow (convert to seconds)
                if (u16MoveMaxLevelWindowTmr >= (25 * (uint16_t)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_WINDOW)) )
                {
                    // Start new MaxWindow
                    u16MoveMaxLevelWindowTmr = 0;
                    // Clear HWM for window
                    MoveWinA.u16TmrHwm = 0;
					// Incr trigger count (will count 1, 2, 3...)
					MoveWinA.u8TriggerCnt++;
					// Lock in hold cnt
					MoveWinA.u8HoldBackCnt = (bool)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_FIRST_HOLD) + (bool)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_SECOND_HOLD);
					// (Re)start timer for Window A
//					TIMERS_vTimerCreate(&MoveWinA.tTmr, TIMER_RTC_TICK);
//					TIMERS_vTimerStart(&MoveWinA.tTmr, SETTINGS_u16GetShort(movementAlarm.windowSensitive));
					// Holding back alerts?
					if (
						((MoveWinA.u8TriggerCnt == 1) && (bool)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_FIRST_HOLD))
					   || ((MoveWinA.u8TriggerCnt == 2) && (bool)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_SECOND_HOLD))
					   )
					{
						// Dbg
						DBG("--- high act alarm | hold (%u/%u) ---", MoveWinA.u8TriggerCnt, MoveWinA.u8HoldBackCnt);

						// Add possible holdback event upon first trigger
						if (MoveHoldback.u8Idx < MOVE_HOLDBACK_BUF_SIZE)
						{
							if (MoveWinA.u8TriggerCnt == 1)
							{
								MoveHoldback.Events[MoveHoldback.u8Idx].u32Ts = RTC_u64GetUTC();
							}
						}
					}
					else
					{
						// Not holding back alerts; queue
//						MOVE_ALARM_bTriggerAlertAdd(MoMessage_Event_MovementAlarmParameter_MovementAlarmState_TRIGGERED, NULL);
						// Incr total count
						u8MoveHighActivityAlarmCnt++;
					}
                }
                // else <MaxLevel
            }
            // else move alarm NOT active for day time zone
        }
        else
        {
            // <MaxLevel
            // LED off
        	MOVE_DRIVER_vLedOff();
            // Decr time in window
            if (u16MoveMaxLevelWindowTmr) u16MoveMaxLevelWindowTmr--;
        }

        // Not-active alarm is always checking (regardless of active time) (note that 0 will disable animal not-active)
        if (u8Level < MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MIN_LEVEL))
        {
            // u8Level < MOVE_ALARM_SETTING_MIN_LEVEL

        	// Check for unexpired window
            if (u32MoveMinLevelWindowTmr)
            {
            	// Decr window & check for expiry
                u32MoveMinLevelWindowTmr--;
                if (u32MoveMinLevelWindowTmr==0)
                {
                	// Queue event (don't worry about queue result)
                	MOVE_bNoActAlertPush(MOVE_NOT_ALARM_STATE_TRIGGERED);
                	// Incr alarm cnt
                	u8MoveNotActiveAlarmCnt++;
                }
            }
            // else previously expired window
        }
        else
        {
            // u8Level >= MOVE_ALARM_SETTING_MIN_LEVEL

        	// Check for expired window with adequate movement
        	if ( u32MoveMinLevelWindowTmr==0 && u8Level>=5 )
        	{
        		// Queue event (don't worry about queue result)
        		MOVE_bNoActAlertPush(MOVE_NOT_ALARM_STATE_CANCELLED);
        	}

        	// Check for unexpired window, or expired window with adequate movement
        	if ( u32MoveMinLevelWindowTmr || ( u32MoveMinLevelWindowTmr==0 && u8Level>=5 ) )
        	{
                // Reset window
#ifndef SWITCH_POSITION_LOGGER
            	// Nominal case: Setup minimum window from setting
            	u32MoveMinLevelWindowTmr = 60 * 25 * (uint32_t)MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MIN_WINDOW);
#else
            	// Switch case: Force minimum window to 60 minutes
            	u32MoveMinLevelWindowTmr = 60 * 25 * 60;
#endif
        	}
        }
    }  // while
}

/**
 * State machine for movement alarm windows (A and B)
 *
 * WinA can be cleared if timeout & no alarm;
 */
void MOVE_vWindowATick(void)
{
	// Check if WinA is expired; note that if stopped then IsExpired()=false
	if ( MoveWinA.u8TriggerCnt/* && TIMERS_bTimerIsExpired(&MoveWinA.tTmr) && !MOVE_ALARM_bIsActive() */)
	{
		if ( MoveWinA.u8TriggerCnt<=MoveWinA.u8HoldBackCnt ) {
			// This was not an alarm
			DBG("--- high act alarm | cancel (hold=%u/%u, hwm=%u/%us) ---", MoveWinA.u8TriggerCnt, MoveWinA.u8HoldBackCnt, MoveWinA.u16TmrHwm/25, MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_WINDOW));
            // Confirm possible holdback event
            if (MoveHoldback.u8Idx < MOVE_HOLDBACK_BUF_SIZE)
            {
            	// Timestamp is already set
				MoveHoldback.Events[MoveHoldback.u8Idx].u8TrigCnt = MoveWinA.u8TriggerCnt;
            	MoveHoldback.Events[MoveHoldback.u8Idx].u8HoldCnt = MoveWinA.u8HoldBackCnt;
            	MoveHoldback.Events[MoveHoldback.u8Idx].u8WinHwm = MoveWinA.u16TmrHwm/25;
            	MoveHoldback.Events[MoveHoldback.u8Idx].u8WinLen = MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_WINDOW);
            	MoveHoldback.u8Idx++;
            }
		}
		else
		{
			// This was an alarm
			DBG("--- high act alarm | done (trig=%u, hold=%u) ---", MoveWinA.u8TriggerCnt, MoveWinA.u8HoldBackCnt);
		}
		// Clear winA
		memset(&MoveWinA, 0, sizeof(move_window_a_t));
//		TIMERS_vTimerStop(&MoveWinA.tTmr);
	}
	// else WinA is NOT expired
}


uint8_t MOVE_u8GetAlarmSettingNow(move_alarm_setting_t tMoveAlarmSetting)
{
	// TIME_tGetDayTimeZoneNow(): 0=morning...3-night
	if (TIME_tGetDayTimeZoneNow() == TIME_OF_DAY_MORNING) {
		switch(tMoveAlarmSetting) {
			case MOVE_ALARM_SETTING_MAX_LEVEL: return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.maxLevel); break;
			case MOVE_ALARM_SETTING_MAX_WINDOW: return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.maxWindow); break;
			case MOVE_ALARM_SETTING_FIRST_HOLD: return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.holdFirst); break;
			case MOVE_ALARM_SETTING_SECOND_HOLD: return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.holdSecond); break;
			case MOVE_ALARM_SETTING_MIN_LEVEL: return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.minLevel); break;
			case MOVE_ALARM_SETTING_MIN_WINDOW: return SETTINGS_u8GetByte(movementAlarm.morningZoneLevels.minWindow); break;
		}
	}
	if (TIME_tGetDayTimeZoneNow() == TIME_OF_DAY_MIDDAY) {
		switch(tMoveAlarmSetting) {
			case MOVE_ALARM_SETTING_MAX_LEVEL: return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.maxLevel); break;
			case MOVE_ALARM_SETTING_MAX_WINDOW: return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.maxWindow); break;
			case MOVE_ALARM_SETTING_FIRST_HOLD: return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.holdFirst); break;
			case MOVE_ALARM_SETTING_SECOND_HOLD: return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.holdSecond); break;
			case MOVE_ALARM_SETTING_MIN_LEVEL: return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.minLevel); break;
			case MOVE_ALARM_SETTING_MIN_WINDOW: return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.minWindow); break;
		}
	}
	if (TIME_tGetDayTimeZoneNow() == TIME_OF_DAY_AFTERNOON) {
		switch(tMoveAlarmSetting) {
			case MOVE_ALARM_SETTING_MAX_LEVEL: return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.maxLevel); break;
			case MOVE_ALARM_SETTING_MAX_WINDOW: return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.maxWindow); break;
			case MOVE_ALARM_SETTING_FIRST_HOLD: return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.holdFirst); break;
			case MOVE_ALARM_SETTING_SECOND_HOLD: return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.holdSecond); break;
			case MOVE_ALARM_SETTING_MIN_LEVEL: return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.minLevel); break;
			case MOVE_ALARM_SETTING_MIN_WINDOW: return SETTINGS_u8GetByte(movementAlarm.afternoonZoneLevels.minWindow); break;
		}
	}
	if (TIME_tGetDayTimeZoneNow() == TIME_OF_DAY_NIGHT) {
		switch(tMoveAlarmSetting) {
			case MOVE_ALARM_SETTING_MAX_LEVEL: return SETTINGS_u8GetByte(movementAlarm.nightZoneLevels.maxLevel); break;
			case MOVE_ALARM_SETTING_MAX_WINDOW: return SETTINGS_u8GetByte(movementAlarm.nightZoneLevels.maxWindow); break;
			case MOVE_ALARM_SETTING_FIRST_HOLD: return SETTINGS_u8GetByte(movementAlarm.nightZoneLevels.holdFirst); break;
			case MOVE_ALARM_SETTING_SECOND_HOLD: return SETTINGS_u8GetByte(movementAlarm.middayZoneLevels.holdSecond); break;
			case MOVE_ALARM_SETTING_MIN_LEVEL: return SETTINGS_u8GetByte(movementAlarm.nightZoneLevels.minLevel); break;
			case MOVE_ALARM_SETTING_MIN_WINDOW: return SETTINGS_u8GetByte(movementAlarm.nightZoneLevels.minWindow); break;
		}
	}
	// All the above should have returned a value
	return 0;

}

bool MOVE_bIsAlarmActiveNow(void)
{
	// Note:
	// ee_u32MoveAlarmTimeOfDayActive is mapped: [SUN_NIGHT,SUN_MORNING,SUN_MIDDAY,SUN_AFTERNOON,MON_NIGHT..SAT_AFTERNOON,0,0,0,0]
	// TIME_tGetDayTimeZoneNow() returns: TIME_OF_DAY_NIGHT=0..TIME_OF_DAY_AFTERNOON=3
	// TIME_u8GetDayOfWeek() returns: SUN=0..SAT=6

	return (bool)(SETTINGS_u32GetWord(movementAlarm.dayTimeActive) & ((uint32_t)0x00000001<<(TIME_u8GetDayOfWeek()*4+TIME_tGetDayTimeZoneNow())));
}
/*
static bool MOVE_bIsSlowAlarmActiveNow(void)
{
    return (bool)(EE_MAP_GET_DWORD(ee_u32SlowMoveAlarmTimeOfDayActive) & ((uint32_t)0x00000001<<(TIME_u8GetDayOfWeek()*4+TIME_tGetDayTimeZoneNow())));
}
*/

/**
 * @brief	Check current movement level
 *
 * Check current movement level. bMoveActiveFlag is cleared when movement is
 * not at a sufficient level for a sufficient time window, and set otherwise.
 */
bool MOVE_bIsActive(void)
{
	return u32MoveMinLevelWindowTmr!=0;
}

/**
 * Add no-movement alarm to fifo
 *
 * @param	tState	Type of no-movement alarm
 * @returns	bool	True if added OK, false on error
 */
bool MOVE_bNoActAlertPush(move_not_alarm_state_t tState)
{
	uint8_t u8;

	// Get next buf idx, check for full, add data
	u8 = ((MoveNotAlarmFifo.u8Head + 1) % MOVE_NOT_ALARM_FIFO_SIZE);
	if (u8 != MoveNotAlarmFifo.u8Tail) {
		// Store alert
		MoveNotAlarmFifo.Buf[MoveNotAlarmFifo.u8Head].tState = tState;
		// Advance head
		MoveNotAlarmFifo.u8Head = u8;

		DBG("+++ NO ACTIVITY ALARM (%s) +++", (char *)acMoveNoActAlarmState[tState]);
		return true;
	}
	else {


		DBG("*** no activity alarm buffer ERROR ***");
		return false;
	}
}

/**
 * Check (peek) at oldest element (tail) of no-movement alarm to fifo
 *
 * @returns	move_not_alarm_t *	Ptr to buf entry (NULL on empty buf)
 */
move_not_alarm_t * MOVE_pNoActAlertPeek(void)
{
	// Check for non-empty fifo, return ptr
	if (MoveNotAlarmFifo.u8Head != MoveNotAlarmFifo.u8Tail) {
		return &MoveNotAlarmFifo.Buf[MoveNotAlarmFifo.u8Tail];
	}
	else {
		return NULL;
	}
}

/**
 * Get (pull) oldest element (tail) of no-movement alarm to fifo
 *
 * @param	pMoveNotAlarm *	Ptr to target struct
 * @returns	bool	True if pulled OK, false on empty buf
 */
bool MOVE_bNoActAlertPull(move_not_alarm_t * pMoveNotAlarm)
{
	// Check for non-empty fifo, check for target ptr & copy data, remove tail
	if (MoveNotAlarmFifo.u8Head != MoveNotAlarmFifo.u8Tail) {
		if (pMoveNotAlarm) {
			memcpy(pMoveNotAlarm, &MoveNotAlarmFifo.Buf[MoveNotAlarmFifo.u8Tail], sizeof(move_not_alarm_t));
		}
		MoveNotAlarmFifo.u8Tail++;
		MoveNotAlarmFifo.u8Tail %= MOVE_NOT_ALARM_FIFO_SIZE;
		return true;
	}
	else {
		return false;
	}
}

/**
 * Add movement alarm to fifo
 *
 * Add movement alarm to fifo; fifo to be handled by calling functions (e.g. event module)
 *
 * @param	u32Id	Movement alarm ID (i.e. UTC timestamp)
 * @param	u8AlarmCnt	Sequential alarm number for event (used by event module to choose mc/sms)
 * @param	tMoveAlarmAlert	Movement alarm alert type (high act, tracking start, etc)
 * @param	u32WinExpTime	Movement alarm expiry time (i.e. UTC timestamp)
 * @returns	bool	True if added to buffer OK, false on error
 */
//bool MOVE_bHighActivityAlertPush(uint32_t u32Id, uint8_t u8AlarmCnt, move_alarm_alert_t tMoveAlarmAlert, uint8_t u8TrackingUpdateCnt, uint32_t u32WinExpTime)
//{
//	uint8_t u8;
//	int16_t i16;
//
//	// Get next buf idx, check for full, add data
//	u8 = ((xMoveAlarmFifo.u8Head + 1) % MOVE_ALARM_FIFO_SIZE);
//	if (u8 != xMoveAlarmFifo.u8Tail) {
//		// Store alert
//		xMoveAlarmFifo.Buf[xMoveAlarmFifo.u8Head].u32Id = u32Id;
//		xMoveAlarmFifo.Buf[xMoveAlarmFifo.u8Head].tAlert = tMoveAlarmAlert;
//		xMoveAlarmFifo.Buf[xMoveAlarmFifo.u8Head].u8AlarmCnt = u8AlarmCnt;
//		xMoveAlarmFifo.Buf[xMoveAlarmFifo.u8Head].u8TrackingUpdateCnt = u8TrackingUpdateCnt;
//		xMoveAlarmFifo.Buf[xMoveAlarmFifo.u8Head].u32WindowExpirationTime = u32WinExpTime;
//		// Advance head
//		xMoveAlarmFifo.u8Head = u8;
//		// Dbg
//		i16 = max(0, (int16_t)(u32WinExpTime - RTC_u64GetUTC()));		// Don't show negative number in case of final margin before tracking expired
//
//#warning Marius
////		DBG_LOG("+++ HIGH ACT ALARM (%s),cnt=%u,track=%02u:%02u +++", (char *)acMoveAlarmAlert[tMoveAlarmAlert], u8AlarmCnt,
////		((i16 / 60) % 60),
////		(i16 % 60)
////		);
//
//		return true;
//	}
//	else {
//		DBG_LOG("*** high act alarm buffer ERROR ***");
//		return false;
//	}
//}

/*
bool MOVE_bAlarmPush(move_alarm_event_t* event, move_alarm_alert_t tMoveAlarmAlert)
{
    uint8_t u8;
    int16_t i16;

    // Get next buf idx, check for full, add data
    u8 = ((MoveAlarmFifo.u8Head + 1) % MOVE_ALARM_FIFO_SIZE);
    if (u8 != MoveAlarmFifo.u8Tail) {
        // Store alert
        MoveAlarmFifo.Buf[MoveAlarmFifo.u8Head].u32Id = event->u32Id;
        MoveAlarmFifo.Buf[MoveAlarmFifo.u8Head].tAlert = tMoveAlarmAlert;
        MoveAlarmFifo.Buf[MoveAlarmFifo.u8Head].u8AlarmCnt = event->u8AlarmCnt;
        MoveAlarmFifo.Buf[MoveAlarmFifo.u8Head].u8TrackingUpdateCnt = event->u8TrackingUpdateCnt;
        MoveAlarmFifo.Buf[MoveAlarmFifo.u8Head].u32WindowExpirationTime = event->u32ExpiryTime;
        // Advance head
        MoveAlarmFifo.u8Head = u8;
        // Dbg
        i16 = max(0, (int16_t)(event->u32ExpiryTime - RTCTMR_u32GetRtc()));     // Don't show negative number in case of final margin before tracking expired

        DBG_LOG("+++ HIGH ACT ALARM (%s), cnt=%u, track=%02u:%02u +++",
        (char *)pgm_read_word(&acMoveAlarmAlert[tMoveAlarmAlert]),
        event->u8AlarmCnt,
        ((i16 / 60) % 60),
        (i16 % 60)
        );

        return true;
    } else {
        DBG_LOG("*** high act alarm buffer ERROR ***");
        return false;
    }
}
*/

/**
 * Check (peek) at oldest element (tail) of movement alarm to fifo
 *
 * @returns	move_alarm_t *	Ptr to buf entry (NULL on empty buf)
 */
//move_alarm_t * MOVE_pHighActivityAlertPeek(void)
//{
//	// Check for non-empty fifo, return ptr
//	if (xMoveAlarmFifo.u8Head != xMoveAlarmFifo.u8Tail) {
//		return &xMoveAlarmFifo.Buf[xMoveAlarmFifo.u8Tail];
//	}
//	else {
//		return NULL;
//	}
//}

/**
 * Get (pull) oldest element (tail) of no-movement alarm to fifo
 *
 * @param	pMoveNotAlarm *	Ptr to target struct
 * @returns	bool	True if pulled OK, false on empty buf
 */
//bool MOVE_bHighActivityAlertPull(move_alarm_t * pMoveAlarm)
//{
//	// Check for non-empty fifo, check for target ptr & copy data, remove tail
//	if (xMoveAlarmFifo.u8Head != xMoveAlarmFifo.u8Tail) {
//		if (pMoveAlarm) {
//			memcpy(pMoveAlarm, &xMoveAlarmFifo.Buf[xMoveAlarmFifo.u8Tail], sizeof(move_alarm_t));
//		}
//		xMoveAlarmFifo.u8Tail++;
//		xMoveAlarmFifo.u8Tail %= MOVE_ALARM_FIFO_SIZE;
////DBG_LOG("--- move alarm alert PULL OK ---");
//		return true;
//	}
//	else {
////DBG_LOG("--- move alarm alert PULL ERROR ---");
//		return false;
//	}
//}



/*
 * Get the number of events in the buffer
*/
uint8_t MOVE_u8HoldbackBufLen(void)
{
    return MoveHoldback.u8Idx;
}

/*
 * Get a pointer to the buffer. Do not modify the buffer
*/
move_holdback_event_t * MOVE_pGetHoldbackBuf (void)
{
    return MoveHoldback.Events;
}

/*
 * Clear out the holdback events buffer, after it has been uploaded to server
*/
void MOVE_vClearHoldbackBuf(void)
{
	memset(&MoveHoldback, 0, sizeof(move_holdback_t));
}

