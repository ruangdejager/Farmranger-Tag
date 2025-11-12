/*
 * Movement.h
 *
 *  Created on: Nov 11, 2025
 *      Author: Ruan de Jager
 */

#ifndef WORKER_MOVEMENT_MOVEMENT_H_
#define WORKER_MOVEMENT_MOVEMENT_H_

#include "Movement_Config.h"

//! Move alarm settings
typedef enum
{
	MOVE_ALARM_SETTING_MAX_LEVEL=0,
	MOVE_ALARM_SETTING_MAX_WINDOW,
	MOVE_ALARM_SETTING_FIRST_HOLD,
	MOVE_ALARM_SETTING_SECOND_HOLD,
	MOVE_ALARM_SETTING_MIN_LEVEL,
	MOVE_ALARM_SETTING_MIN_WINDOW
} move_alarm_setting_t;

//! Movement settings for a daytimezone
typedef struct move_settings_dtz_t{
	uint8_t u8MaxLevel;
	uint8_t u8MaxWindow;
	uint8_t u8HoldFirstCall;
	uint8_t u8HoldSecondCall;
	uint8_t u8MinLevel;
	uint8_t u8MinWindow;
} move_settings_dtz_t;

// Depreciated
typedef enum _move_alarm_alert_high_act_t {
	MOVE_ALARM_ALERT_HIGH_ACT_RES=0,
 	MOVE_ALARM_ALERT_HIGH_ACT_MC_SMS=1,
 	MOVE_ALARM_ALERT_HIGH_ACT_SMS=2
} move_alarm_alert_high_act_t;

// No movement alarm state
typedef enum _move_not_alarm_state_t {
	MOVE_NOT_ALARM_STATE_IDLE=0,
	MOVE_NOT_ALARM_STATE_TRIGGERED=1,
	MOVE_NOT_ALARM_STATE_CANCELLED=2
} move_not_alarm_state_t;

// No movement alarm
typedef struct _move_not_alarm_t {
	move_not_alarm_state_t tState;
} move_not_alarm_t;

// Holdback event type
typedef struct _move_holdback_event_t
{
    uint32_t u32Ts;
    uint8_t u8TrigCnt;		// Trigger count
    uint8_t u8HoldCnt;		// Total hold back alllowed
    uint8_t u8WinHwm;		// Window high water mark
    uint8_t u8WinLen;		// Window length
} move_holdback_event_t;

/*
typedef enum _move_sensor_error_alert_t {
	MOVE_SENSOR_ERROR_ALERT_NONE=0,
	MOVE_SENSOR_ERROR_ALERT_PRIM=1,
	MOVE_SENSOR_ERROR_ALERT_SEC=2
} move_sensor_error_alert_t;
*/

//! Macro to map alarm settings to array min / def / max
#define MOVE_ALARM_SETTING_IDX(TimeOfDay, Setting)		((6*TimeOfDay)+(Setting))

//! Function prototypes
void MOVE_vInit(void);
bool MOVE_bTick(void);
bool MOVE_bIsActive(void);
bool MOVE_bIsAlarmActiveNow(void);
uint8_t MOVE_u8GetAlarmSettingNow(move_alarm_setting_t tMoveAlarmSetting);
bool MOVE_bSetSensorAxis(uint8_t u8AxisMask);

bool MOVE_bNoActAlertPush(move_not_alarm_state_t tState);
move_not_alarm_t * MOVE_pNoActAlertPeek(void);
bool MOVE_bNoActAlertPull(move_not_alarm_t * pMoveNotAlarm);

bool MOVE_bSensorHealthAlertPeek(void);
bool MOVE_bSensorHealthAlertPull(void);
bool MOVE_bSensorHealthOk(void);

uint8_t MOVE_u8HighActivityAlarmCnt(void);
uint8_t MOVE_u8HighActAlarmSuccessCnt(void);
void MOVE_vHighActAlarmSuccessIncr(void);
uint8_t MOVE_u8NotActiveAlarmCnt(void);

uint8_t MOVE_u8HoldbackBufLen(void);
move_holdback_event_t * MOVE_pGetHoldbackBuf (void);
void MOVE_vClearHoldbackBuf(void);

//bool MOVE_bGetAlarmDtzActive(time_day_of_week_t tDayOfWeek, time_of_day_t tTimeOfDay);
//bool MOVE_bSetAlarmDtzActive(time_day_of_week_t tDayOfWeek, time_of_day_t tTimeOfDay, bool bValue);
//uint32_t MOVE_u32GetAlarmDtzActiveBm(void);
//bool MOVE_bCheckAlarmDtzActiveBm(uint32_t u32Value);
//bool MOVE_bSetAlarmDtzActiveBm(uint32_t u32Value);
//void MOVE_vResetAlarmDtzActiveBm(void);

//uint8_t MOVE_u8GetAlarmSetting(time_of_day_t tTimeOfDay, move_alarm_setting_t tSetting);
//bool MOVE_bCheckAlarmSetting(time_of_day_t tTimeOfDay, move_alarm_setting_t tSetting, uint8_t u8Value);
//bool MOVE_bSetAlarmSetting(time_of_day_t tTimeOfDay, move_alarm_setting_t tSetting, uint8_t u8Value);
//void MOVE_vResetAlarmSetting(time_of_day_t tTimeOfDay, move_alarm_setting_t tSetting);
//void MOVE_vGetAlarmSettingsDtz(time_of_day_t tTimeOfDay, move_settings_dtz_t * pSettingsDtz);
//void MOVE_vSetAlarmSettingsDtz(time_of_day_t tTimeOfDay, move_settings_dtz_t * pSettingsDtz);
//void MOVE_vResetAlarmSettings(void);







#ifdef SWITCH_MOVE_SENSOR_DATA
int8_t MOVE_i8SensorDataX(void);
int8_t MOVE_i8SensorDataY(void);
int8_t MOVE_i8SensorDataZ(void);
#endif


#endif /* WORKER_MOVEMENT_MOVEMENT_H_ */
