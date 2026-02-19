/*
 * MpptChg.c
 *
 *  Created on: Dec 8, 2025
 *      Author: Ruan de Jager
 */

#include "MpptChg.h"
#include "MpptChg_Driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "hard_timers.h"
#include "dbg_log.h"
#include <limits.h>
#include <stdbool.h>

// --- PRIVATE DEFINES ---
#define MPPT_PGSTATETASK_PRIORITY    	(configMAX_PRIORITIES - 4)
#define MPPT_MPPTTASK_PRIORITY   		(configMAX_PRIORITIES - 5)
#define MPPT_TASK_STACK_SIZE            (configMINIMAL_STACK_SIZE)

TaskHandle_t MPPTCHG_vPgStateTask_handle;
TaskHandle_t MPPTCHG_vMpptTask_handle;

//TimerHandle_t tMpptTmr;
//TimerHandle_t tCheckPGIntervalTmr;
//
//volatile bool bMpptTmrExpired = true;
//volatile bool bCheckPGIntervalTmrExpired = true;

volatile bool PgStateChange;

uint32_t u32mpptOffCntr;
uint32_t u32mppt5mACntr;
uint32_t u32mppt10mACntr;
uint32_t u32mppt15mACntr;
uint32_t u32mppt20mACntr;
uint32_t u32mppt25mACntr;
uint32_t u32mppt30mACntr;
uint32_t u32mppt35mACntr;
uint32_t u32mppt40mACntr;

chg_pg_state_t PgState;
//! PG line change flag; set in ISR, cleared in CHG_ccrTaskPg()
volatile bool bChgPgChangeFlag;
// The state of charge when in mppt mode
chg_mppt_state_t tMpptState;

TIMERS_timer_t tMpptTmr;
TIMERS_timer_t tCheckPGIntervalTmr;
TIMERS_timer_t tMpptOffCountdownTmr;

void MPPTCHG_vPgStateTask(void *pvParameters);
void MPPTCHG_vMpptTask(void *pvParameters);

void MPPTCHG_vUpdateMpptState(chg_mppt_bump_t tMpptBump);
void MPPTCHG_vSetMpptChgLevel(chg_mppt_state_t tMpptChgLevelSet);
chg_mppt_state_t MPPTCHG_tGetMpptChgLevel(void);

/*
static void MPPTCHG_vGenericTimerCallback(TimerHandle_t xTimer);
void MPPTCHG_vStartTimerSeconds(TimerHandle_t tmr, uint32_t seconds);
*/


/**
 * @brief	ISR for level change on PG.
 *
 */
void MPPTCHG_vPinChangeCallback(void)
{
	bChgPgChangeFlag = true;
}

/**
 * Module initialization
 *
 */
void MPPTCHG_vInit(void)
{
	/* Create timers with dummy period (1 second).
	   We set real period on start. Auto-reload = pdFALSE. */

	/*
	tMpptTmr = xTimerCreate("MPPTtmr",
							pdMS_TO_TICKS(1000),
							pdFALSE,
							(void *)&bMpptTmrExpired,
							MPPTCHG_vGenericTimerCallback);
	configASSERT(tMpptTmr != NULL);

	tCheckPGIntervalTmr = xTimerCreate("CheckPGtmr",
									   pdMS_TO_TICKS(1000),
									   pdFALSE,
									   (void *)&bCheckPGIntervalTmrExpired,
									   MPPTCHG_vGenericTimerCallback);
	configASSERT(tCheckPGIntervalTmr != NULL);
	*/

//    TIMERS_vTimerCreate(&tMpptTmr, TIMER_RTC_TICK);
//    TIMERS_vTimerStart(&tMpptTmr, 0);
//
//    TIMERS_vTimerCreate(&tCheckPGIntervalTmr, TIMER_RTC_TICK);
//    TIMERS_vTimerStart(&tCheckPGIntervalTmr, 0);
//
//	TIMERS_vTimerCreate(&tMpptOffCountdownTmr, TIMER_RTC_TICK);
//	TIMERS_vTimerStart(&tMpptOffCountdownTmr, 30);
//
//    BaseType_t status;
//    status = xTaskCreate(MPPTCHG_vPgStateTask,
//                "PgStateTask",
//				MPPT_TASK_STACK_SIZE,
//                NULL,
//				MPPT_PGSTATETASK_PRIORITY,
//				&MPPTCHG_vPgStateTask_handle);
//    configASSERT(status == pdPASS);
//
//    status = xTaskCreate(MPPTCHG_vMpptTask,
//                "MpptTask",
//				MPPT_TASK_STACK_SIZE,
//                NULL,
//				MPPT_MPPTTASK_PRIORITY,
//				&MPPTCHG_vMpptTask_handle);
//    configASSERT(status == pdPASS);
//
//
//	// Interrupt set up by HAL, just register callback
//	MPPTCHG_DRIVER_vRegisterCallback(MPPTCHG_vPinChangeCallback);
//
//	// PG
//	// Don't init PG line here
//	// Don't init PG interrupt here
//	PgState = CHG_PG_STATE_UNKNOWN;
//	MPPTCHG_DRIVER_vInitGpio();
//
//	tMpptState = CHG_SEL_MPPT_5mA;
//	MPPTCHG_DRIVER_vSetMppt5mA();
//
//	MPPTCHG_vClearMpptStateCounters();

		tMpptState = CHG_SEL_MPPT_20mA;
		MPPTCHG_DRIVER_vSetMppt20mA();

}

void MPPTCHG_vPgStateTask(void *pvParameters)
{
    (void)pvParameters;

	PLATFORM_bSubscribeToHeartbeat(xTaskGetCurrentTaskHandle(),
                        HB_ALLOW_IN_RECOVERY);

    bool bChgPgChange;
    bool bWasSleepActive = false;

    for (;;)
    {

    	// When the MPPT state goes to CHG_SEL_MPPT_OFF we de-init PG irq lines
    	// Here we make sure to check PG lines every 60s when not initialized
		while (1)
		{
			if ( bChgPgChangeFlag || (PgState == CHG_PG_STATE_UNSTABLE) ) break;
			// Incase we are stuck with power good
			#warning We need to change to hardware timers, software timers like this is pause when device goes to sleep
			if ( TIMERS_bTimerIsExpired(&tCheckPGIntervalTmr) )
			{
//				MPPTCHG_vStartTimerSeconds(tCheckPGIntervalTmr, 60);
				TIMERS_vTimerStart(&tCheckPGIntervalTmr, 30);
				break;
			}

	        // Wait 1s notification from PLATFORM module bliptask
			// Todo: Use events for 1s actions in the future
	        xTaskNotifyWait(
	            0x00,            // Don't clear bits on entry
	            ULONG_MAX,       // Clear all bits on exit
	            NULL,// Where the value is stored
	            portMAX_DELAY
	        );

		}

		bWasSleepActive = SYSTEM_bIsDeepSleepActive();
		if (bWasSleepActive) {
			SYSTEM_vDeactivateDeepSleep();
		}

		if (tMpptState == CHG_SEL_MPPT_OFF) MPPTCHG_DRIVER_vInitGpio();

		// Eval flag, update PG status
		// Wait out settling time
		vTaskDelay(pdMS_TO_TICKS(10));

		// Check PG on-change flag; no need to do it thread-safe.
		bChgPgChange = bChgPgChangeFlag;
		bChgPgChangeFlag = false;

		// Eval flag, update PG status
		if (bChgPgChange) {
			PgState = CHG_PG_STATE_UNSTABLE;
		} else {
			PgState = ( MPPTCHG_DRIVER_bGetPgPins() ? CHG_PG_STATE_HIGH : CHG_PG_STATE_LOW);
		}

		if (tMpptState == CHG_SEL_MPPT_OFF) MPPTCHG_DRIVER_vDeinitGpio();

		if (bWasSleepActive) {
			SYSTEM_vActivateDeepSleep();
		}
    }

    vTaskDelete(NULL);

}

/**
 * Main state machine for solar manual mppt charging
 *
 */
void MPPTCHG_vMpptTask(void *pvParameters)
{

    (void)pvParameters;

	PLATFORM_bSubscribeToHeartbeat(xTaskGetCurrentTaskHandle(),
                        HB_ALLOW_IN_RECOVERY);

    for (;;)
    {

		// Frequency of executing mppt switching
//		vTaskDelay(pdMS_TO_TICKS(2000));
        // Wait 1s notification from PLATFORM module bliptask
		// Todo: Use events for 1s actions in the future
        xTaskNotifyWait(
            0x00,            // Don't clear bits on entry
            ULONG_MAX,       // Clear all bits on exit
            NULL,// Where the value is stored
            portMAX_DELAY
        );

		// Increment mppt state counters
		MPPTCHG_vIncMpptStateCounters();

        if (TIMERS_bTimerIsExpired(&tMpptTmr))
        {

			if ( (PgState == CHG_PG_STATE_HIGH) || (PgState == CHG_PG_STATE_UNSTABLE) )
			{
				// Once we lowered the charge level we use a timer to set a waiting time before
				// we try a higher charge. This will ensure longer charging at an optimal level
				// This happens in the bump down function
				MPPTCHG_vUpdateMpptState(CHG_BUMP_MPPT_DOWN);
				// We reset the mppt off countdown timer
				// When mppt state has been at CHG_SEL_MPPT_20mA for 5 minutes and Power is NOT GOOD -> Enter OFF state
				if (tMpptState != CHG_SEL_MPPT_5mA)
				{
					TIMERS_vTimerStart(&tMpptOffCountdownTmr, 15);
				}

			} else if (PgState == CHG_PG_STATE_LOW) {

				MPPTCHG_vUpdateMpptState(CHG_BUMP_MPPT_UP);
			}

			TIMERS_vTimerStart(&tMpptTmr, 2);

        }

    }

}

void MPPTCHG_vUpdateMpptState(chg_mppt_bump_t tMpptBump)
{

	switch (tMpptState) {
		case CHG_SEL_MPPT_OFF:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN) return;
			if (tMpptBump == CHG_BUMP_MPPT_UP)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_5mA);
				// Init the PG line
				MPPTCHG_DRIVER_vInitGpio();
				// Forces a check on PG
				PgState = CHG_PG_STATE_UNSTABLE;
				TIMERS_vTimerStart(&tMpptOffCountdownTmr, 30);
			}
			break;
		case CHG_SEL_MPPT_5mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN && TIMERS_bTimerIsExpired(&tMpptOffCountdownTmr))
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_OFF);
				// De-Init the PG line
				MPPTCHG_DRIVER_vDeinitGpio();
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) {
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_10mA);
			}
			break;
		case CHG_SEL_MPPT_10mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_5mA);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) {
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_15mA);
			}
			break;
		case CHG_SEL_MPPT_15mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_10mA);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) {
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_20mA);
			}
			break;
		case CHG_SEL_MPPT_20mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_15mA);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) {
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_25mA);
			}
			break;
		case CHG_SEL_MPPT_25mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_20mA);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) {
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_30mA);
			}
			break;
		case CHG_SEL_MPPT_30mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_25mA);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) {
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_35mA);
			}
			break;
		case CHG_SEL_MPPT_35mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_30mA);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) {
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_40mA);
			}
			break;
		case CHG_SEL_MPPT_40mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_35mA);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) return;
			break;
		default:
			break;
	}
}

void MPPTCHG_vSetMpptChgLevel(chg_mppt_state_t tMpptChgLevelSet)
{
	// Ignore if the state is already on this level
	if (tMpptChgLevelSet == tMpptState) return;

	switch (tMpptChgLevelSet) {
		case CHG_SEL_MPPT_OFF:
			tMpptState = CHG_SEL_MPPT_OFF;
			DBG("\r\nMPPT SET: OFF\r\n");
			break;
		case CHG_SEL_MPPT_5mA:
			tMpptState = CHG_SEL_MPPT_5mA;
			MPPTCHG_DRIVER_vSetMppt5mA();
			DBG("\r\nMPPT SET: 5mA\r\n");
			break;
		case CHG_SEL_MPPT_10mA:
			tMpptState = CHG_SEL_MPPT_10mA;
			MPPTCHG_DRIVER_vSetMppt10mA();
			DBG("\r\nMPPT SET: 10mA\r\n");
			break;
		case CHG_SEL_MPPT_15mA:
			tMpptState = CHG_SEL_MPPT_15mA;
			MPPTCHG_DRIVER_vSetMppt15mA();
			DBG("\r\nMPPT SET: 15mA\r\n");
			break;
		case CHG_SEL_MPPT_20mA:
			tMpptState = CHG_SEL_MPPT_20mA;
			MPPTCHG_DRIVER_vSetMppt20mA();
			DBG("\r\nMPPT SET: 20mA\r\n");
			break;
		case CHG_SEL_MPPT_25mA:
			tMpptState = CHG_SEL_MPPT_25mA;
			MPPTCHG_DRIVER_vSetMppt25mA();
			DBG("\r\nMPPT SET: 25mA\r\n");
			break;
		case CHG_SEL_MPPT_30mA:
			tMpptState = CHG_SEL_MPPT_30mA;
			MPPTCHG_DRIVER_vSetMppt30mA();
			DBG("\r\nMPPT SET: 30mA\r\n");
			break;
		case CHG_SEL_MPPT_35mA:
			tMpptState = CHG_SEL_MPPT_35mA;
			MPPTCHG_DRIVER_vSetMppt35mA();
			DBG("\r\nMPPT SET: 35mA\r\n");
			break;
		case CHG_SEL_MPPT_40mA:
			tMpptState = CHG_SEL_MPPT_40mA;
			MPPTCHG_DRIVER_vSetMppt40mA();
			DBG("\r\nMPPT SET: 40mA\r\n");
			break;
		default:
			break;
	}
}

chg_mppt_state_t MPPTCHG_tGetMpptChgLevel(void)
{
	return tMpptState;
}

/*
void MPPTCHG_vStartTimerSeconds(TimerHandle_t tmr, uint32_t seconds)
{
    volatile bool *flag =
        (volatile bool *) pvTimerGetTimerID(tmr);

    *flag = false;   // reset before starting

    // Change timer period to "seconds"
    xTimerChangePeriod(tmr, pdMS_TO_TICKS(seconds * 1000), 0);
}

static void MPPTCHG_vGenericTimerCallback(TimerHandle_t xTimer)
{
    volatile bool *flag =
        (volatile bool *) pvTimerGetTimerID(xTimer);

    *flag = true;    // mark expired
}
*/

void MPPTCHG_vIncMpptStateCounters(void)
{
	switch (tMpptState) {
		case CHG_SEL_MPPT_OFF:
			u32mpptOffCntr++;
			break;
		case CHG_SEL_MPPT_5mA:
			u32mppt5mACntr++;
			break;
		case CHG_SEL_MPPT_10mA:
			u32mppt10mACntr++;
			break;
		case CHG_SEL_MPPT_15mA:
			u32mppt15mACntr++;
			break;
		case CHG_SEL_MPPT_20mA:
			u32mppt20mACntr++;
			break;
		case CHG_SEL_MPPT_25mA:
			u32mppt25mACntr++;
			break;
		case CHG_SEL_MPPT_30mA:
			u32mppt30mACntr++;
			break;
		case CHG_SEL_MPPT_35mA:
			u32mppt35mACntr++;
			break;
		case CHG_SEL_MPPT_40mA:
			u32mppt40mACntr++;
			break;
		default:
			break;
	}
}

void MPPTCHG_vClearMpptStateCounters(void)
{
	u32mpptOffCntr = 0;
	u32mppt5mACntr = 0;
	u32mppt10mACntr = 0;
	u32mppt15mACntr = 0;
	u32mppt20mACntr = 0;
	u32mppt25mACntr = 0;
	u32mppt30mACntr = 0;
	u32mppt35mACntr = 0;
	u32mppt40mACntr = 0;
}

uint32_t MPPTCHG_u32GetOffMpptCounter(void)
{
	return u32mpptOffCntr;
}

uint32_t MPPTCHG_u32Get5mAMpptCounter(void)
{
	return u32mppt5mACntr;
}

uint32_t MPPTCHG_u32Get10mAMpptCounter(void)
{
	return u32mppt10mACntr;
}

uint32_t MPPTCHG_u32Get15mAMpptCounter(void)
{
	return u32mppt15mACntr;
}

uint32_t MPPTCHG_u32Get20mAMpptCounter(void)
{
	return u32mppt20mACntr;
}

uint32_t MPPTCHG_u32Get25mAMpptCounter(void)
{
	return u32mppt25mACntr;
}

uint32_t MPPTCHG_u32Get30mAMpptCounter(void)
{
	return u32mppt30mACntr;
}

uint32_t MPPTCHG_u32Get35mAMpptCounter(void)
{
	return u32mppt35mACntr;
}

uint32_t MPPTCHG_u32Get40mAMpptCounter(void)
{
	return u32mppt40mACntr;
}

