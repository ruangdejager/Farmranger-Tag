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

#include "dbg_log.h"

#include <stdbool.h>

// --- PRIVATE DEFINES ---
#define MPPT_PGSTATETASK_PRIORITY    	(configMAX_PRIORITIES - 4)
#define MPPT_MPPTTASK_PRIORITY   		(configMAX_PRIORITIES - 5)
#define MPPT_TASK_STACK_SIZE            (configMINIMAL_STACK_SIZE)

TaskHandle_t MPPTCHG_vPgStateTask_handle;
TaskHandle_t MPPTCHG_vMpptTask_handle;

TimerHandle_t tMpptTmr;
TimerHandle_t tCheckPGIntervalTmr;
TimerHandle_t tMpptOffCountdownTmr;

volatile bool bMpptTmrExpired = true;
volatile bool bCheckPGIntervalTmrExpired = true;
volatile bool bMpptOffCountdownTmrExpired = true;

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

void MPPTCHG_vPgStateTask(void *pvParameters);
void MPPTCHG_ccrMpptTask(void *pvParameters);

void MPPTCHG_vUpdateMpptState(chg_mppt_bump_t tMpptBump);
void MPPTCHG_vSetMpptChgLevel(chg_mppt_state_t tMpptChgLevelSet);
chg_mppt_state_t MPPTCHG_tGetMpptChgLevel(void);

static void MPPTCHG_vGenericTimerCallback(TimerHandle_t xTimer);
void MPPTCHG_vStartTimerSeconds(TimerHandle_t tmr, uint32_t seconds);


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

	tMpptOffCountdownTmr = xTimerCreate("MPPTOfftmr",
										pdMS_TO_TICKS(1000),
										pdFALSE,
										(void *)&bMpptOffCountdownTmrExpired,
										MPPTCHG_vGenericTimerCallback);
	configASSERT(tMpptOffCountdownTmr != NULL);

	MPPTCHG_vStartTimerSeconds(tMpptOffCountdownTmr, 30);

    BaseType_t status;
    status = xTaskCreate(MPPTCHG_vPgStateTask,
                "PgStateTask",
				MPPT_TASK_STACK_SIZE,
                NULL,
				MPPT_PGSTATETASK_PRIORITY,
				&MPPTCHG_vPgStateTask_handle);
    configASSERT(status == pdPASS);

    status = xTaskCreate(MPPTCHG_ccrMpptTask,
                "MpptTask",
				MPPT_TASK_STACK_SIZE,
                NULL,
				MPPT_MPPTTASK_PRIORITY,
				&MPPTCHG_vMpptTask_handle);
    configASSERT(status == pdPASS);


	// Interrupt set up by HAL, just register callback
	MPPTCHG_DRIVER_vRegisterCallback(MPPTCHG_vPinChangeCallback);

	// PG
	// Don't init PG line here
	// Don't init PG interrupt here
	PgState = CHG_PG_STATE_UNKNOWN;
	MPPTCHG_DRIVER_vInitGpio();

	tMpptState = CHG_SEL_MPPT_5mA;
	MPPTCHG_DRIVER_vSetMppt5mA();

	MPPTCHG_vClearMpptStateCounters();

}

void MPPTCHG_vPgStateTask(void *pvParameters)
{
    (void)pvParameters;

    bool bChgPgChange;

    for (;;)
    {

    	// When the MPPT state goes to CHG_SEL_MPPT_OFF we de-init PG irq lines
    	// Here we make sure to check PG lines every 60s when not initialized
		while (1)
		{
			if ( bChgPgChangeFlag || (PgState == CHG_PG_STATE_UNSTABLE) ) break;
			// Incase we are stuck with power good
			if ( bCheckPGIntervalTmrExpired )
			{
				MPPTCHG_vStartTimerSeconds(tCheckPGIntervalTmr, 60);
				break;
			}
			vTaskDelay(pdMS_TO_TICKS(1000));

		}

		if (tMpptState == CHG_SEL_MPPT_OFF) MPPTCHG_DRIVER_vInitGpio();

		// Eval flag, update PG status
		// Wait out settling time
		vTaskDelay(pdMS_TO_TICKS(100));

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
    }

    vTaskDelete(NULL);

}

/**
 * Main state machine for solar manual mppt charging
 *
 */
void MPPTCHG_ccrMpptTask(void *pvParameters)
{

    (void)pvParameters;

    for (;;)
    {

		// Frequency of executing mppt switching
		vTaskDelay(pdMS_TO_TICKS(2000));

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
				MPPTCHG_vStartTimerSeconds(tMpptOffCountdownTmr, 300);
			}
		} else if (PgState == CHG_PG_STATE_LOW)
		{
			if (bMpptTmrExpired) MPPTCHG_vUpdateMpptState(CHG_BUMP_MPPT_UP);
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
				MPPTCHG_vStartTimerSeconds(tMpptOffCountdownTmr, 30);
			}
			break;
		case CHG_SEL_MPPT_5mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN && bMpptOffCountdownTmrExpired)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_OFF);
				// De-Init the PG line
				MPPTCHG_DRIVER_vDeinitGpio();
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_10mA);
			break;
		case CHG_SEL_MPPT_10mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_5mA);
				MPPTCHG_vStartTimerSeconds(tMpptTmr, 10);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_15mA);
			break;
		case CHG_SEL_MPPT_15mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_10mA);
				MPPTCHG_vStartTimerSeconds(tMpptTmr, 10);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_20mA);
			break;
		case CHG_SEL_MPPT_20mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_15mA);
				MPPTCHG_vStartTimerSeconds(tMpptTmr, 10);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_25mA);
			break;
		case CHG_SEL_MPPT_25mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_20mA);
				MPPTCHG_vStartTimerSeconds(tMpptTmr, 10);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_30mA);
			break;
		case CHG_SEL_MPPT_30mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_25mA);
				MPPTCHG_vStartTimerSeconds(tMpptTmr, 10);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_35mA);
			break;
		case CHG_SEL_MPPT_35mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_30mA);
				MPPTCHG_vStartTimerSeconds(tMpptTmr, 10);
			}
			if (tMpptBump == CHG_BUMP_MPPT_UP) MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_40mA);
			break;
		case CHG_SEL_MPPT_40mA:
			if (tMpptBump == CHG_BUMP_MPPT_DOWN)
			{
				MPPTCHG_vSetMpptChgLevel(CHG_SEL_MPPT_35mA);
				MPPTCHG_vStartTimerSeconds(tMpptTmr, 10);
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

void MPPTCHG_vStartTimerSeconds(TimerHandle_t tmr, uint32_t seconds)
{
    volatile bool *flag =
        (volatile bool *) pvTimerGetTimerID(tmr);

    *flag = false;   // reset before starting

    /* Change timer period to "seconds" */
    xTimerChangePeriod(tmr, pdMS_TO_TICKS(seconds * 1000), 0);
}

static void MPPTCHG_vGenericTimerCallback(TimerHandle_t xTimer)
{
    volatile bool *flag =
        (volatile bool *) pvTimerGetTimerID(xTimer);

    *flag = true;    // mark expired
}

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

