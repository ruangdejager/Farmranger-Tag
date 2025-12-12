/*
 * Battery.c
 *
 *  Created on: Dec 10, 2025
 *      Author: Ruan de Jager
 */

#include "Battery.h"
#include "hal_bsp.h"
#include "Battery_Driver.h"
#include "Battery_Config.h"

#include "platform.h"
#include "dbg_log.h"
#include "math_func.h"
#include "hard_timers.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <limits.h>

#define BAT_NOTIFY_DELTA_LIMIT   (1U << 0)
#define BAT_NOTIFY_PURGE_REQUEST (1U << 1)
#define BAT_NOTIFY_PURGE_DONE    (1U << 2)

// --- PRIVATE DEFINES ---
#define BAT_SAMPLETASK_PRIORITY   	(configMAX_PRIORITIES - 4)
#define BAT_SAMPLETASK_STACK_SIZE	(configMINIMAL_STACK_SIZE)
#define BAT_PURGETASK_PRIORITY   	(configMAX_PRIORITIES - 4)
#define BAT_PURGETASK_STACK_SIZE	(configMINIMAL_STACK_SIZE)
#define BAT_SCHEDULETASK_PRIORITY   (configMAX_PRIORITIES - 4)
#define BAT_SCHEDULETASK_STACK_SIZE	(configMINIMAL_STACK_SIZE)

TaskHandle_t BAT_vSampleTask_handle;
TaskHandle_t BAT_vBufferPurgeTask_handle;
TaskHandle_t BAT_vCheckSampleCheduleTask_handle;

//! Averaging buffer and idx
uint16_t au16BatAvgBuf[BAT_AVG_FACTOR];
uint8_t u8BatAvgIdx;
//! Variable to store previous sample for delta limiting
uint16_t u16BatPreviousSample;

static TIMERS_timer_t tBatSampleIntervalTmr;

//Function definition
void BAT_vSampleTask(void *pvParameters);
void BAT_vBufferPurgeTask(void *pvParameters);
void BAT_vCheckSampleCheduleTask(void *pvParameters);

void BAT_vInit(void)
{

    TIMERS_vTimerCreate(&tBatSampleIntervalTmr, TIMER_RTC_TICK);
    TIMERS_vTimerStart(&tBatSampleIntervalTmr, BAT_SAMPLE_INTERVAL);

    BaseType_t status;

    status = xTaskCreate(BAT_vSampleTask,
                "BatSampleTask",
				BAT_SAMPLETASK_STACK_SIZE,
                NULL,
				BAT_SAMPLETASK_PRIORITY,
				&BAT_vSampleTask_handle);
    configASSERT(status == pdPASS);

    status = xTaskCreate(BAT_vCheckSampleCheduleTask,
                "CheckSampleCheduleTask",
				BAT_SCHEDULETASK_STACK_SIZE,
                NULL,
				BAT_SCHEDULETASK_PRIORITY,
				&BAT_vCheckSampleCheduleTask_handle);
    configASSERT(status == pdPASS);

    // Run the purge task immediately at bat init
    BAT_vPurgeBuffer();
}

void BAT_vSampleTask(void *pvParameters)
{

    uint32_t ulNotifiedValue;
    bool bDeltaLimit;
    bool bPurge;

	uint8_t u8DelayMs;
	uint16_t u16AdcResult;

	bool bWasSleepActive = false;

    for(;;)
    {
        // Wait for a notification indefinitely
        xTaskNotifyWait(
            0x00,            // Don't clear bits on entry
            ULONG_MAX,       // Clear all bits on exit
            &ulNotifiedValue,// Where the value is stored
            portMAX_DELAY
        );

		bWasSleepActive = SYSTEM_bIsDeepSleepActive();
		if (bWasSleepActive) {
			BSP_LED_On(LED_RED);
			SYSTEM_vDeactivateDeepSleep();
		}

        bDeltaLimit = ulNotifiedValue & BAT_NOTIFY_DELTA_LIMIT;
        bPurge = ulNotifiedValue & BAT_NOTIFY_PURGE_REQUEST;

		// Bias the measurement circuit
		BAT_DRIVER_vEnableBiasCircuit();

		// Wait bias settling time
		vTaskDelay(pdMS_TO_TICKS(40));

		// Wait for ADC available
		u8DelayMs = 3;
		while (BAT_DRIVER_bIsEnabled() && u8DelayMs--) {
			vTaskDelay(pdMS_TO_TICKS(1));
		}
		if (u8DelayMs == 0) {
			DBG("bat: adc enable ERROR");
			BAT_DRIVER_vDisable();
		}
		// Enable ADC
		BAT_DRIVER_vEnable();
		// Clear interrupt
		BAT_DRIVER_vCleanInterrupt();
		// Start channel conversion
		BAT_DRIVER_vStartConversion();
		// Wait for ADC conversion on channel to complete
		u8DelayMs = 3;
		while ( !BAT_DRIVER_bGetInterruptFlag() && u8DelayMs--) {
			vTaskDelay(pdMS_TO_TICKS(1));
		}

		// Get ADC result
		u16AdcResult = BAT_DRIVER_u16GetResult();
		// Disable ADC module
		BAT_DRIVER_vDisable();
		// Unbias the measurement circuit
		BAT_DRIVER_vDisableBiasCircuit();

		// Use ADC result?
		if (u8DelayMs == 0) {
			//        DBG_LOG("bat: adc int flag ERROR");
		} else {
			// Update buffer with result
			au16BatAvgBuf[u8BatAvgIdx] = u16AdcResult;
			// Apply delta limiting
			if ( bDeltaLimit && ( au16BatAvgBuf[u8BatAvgIdx] < (u16BatPreviousSample - BAT_SAMPLE_VALUE_DELTA_MAX) ) ) {
				au16BatAvgBuf[u8BatAvgIdx] = u16BatPreviousSample - BAT_SAMPLE_VALUE_DELTA_MAX;
			}
			u16BatPreviousSample = au16BatAvgBuf[u8BatAvgIdx];
			// Move idx
			u8BatAvgIdx++;
			u8BatAvgIdx %= BAT_AVG_FACTOR;
		}

		if (bPurge)
		{
		    TaskHandle_t purge = NULL;

		    taskENTER_CRITICAL();
		    purge = (TaskHandle_t) BAT_vBufferPurgeTask_handle;
		    /* clear the global so we don't notify it twice */
		    BAT_vBufferPurgeTask_handle = NULL;
		    taskEXIT_CRITICAL();

		    if (purge != NULL)
		    {
		        xTaskNotify(purge, BAT_NOTIFY_PURGE_DONE, eSetBits);
		    }
		    else
		    {
		        /* Defensive: if no purge handle found, still continue (avoid deadlock) */
		        DBG("bat: purge notify - no purge handle");
		    }
		}

		if (bWasSleepActive) {
			SYSTEM_vActivateDeepSleep();
		}

    }

}

void BAT_vBufferPurgeTask(void *pvParameters)
{
    uint32_t notif;

    // 1. Tell the sample task to take a purge sample
    xTaskNotify(BAT_vSampleTask_handle,
                BAT_NOTIFY_PURGE_REQUEST,
                eSetBits);

    // 2. Wait for sample task to finish (bit2)
    for (;;)
    {
        xTaskNotifyWait(
            0,
            ULONG_MAX,
            &notif,
            portMAX_DELAY
        );

        if (notif & BAT_NOTIFY_PURGE_DONE)
            break;
    }

    // 3. Fill buffer
    for (uint8_t i = 0; i < BAT_AVG_FACTOR; i++)
        au16BatAvgBuf[i] = u16BatPreviousSample;

    u8BatAvgIdx = 0;

    taskENTER_CRITICAL();
    BAT_vBufferPurgeTask_handle = NULL;
    taskEXIT_CRITICAL();

    // 4. Self-delete — this is a one-shot task
    vTaskDelete(NULL);
}

void BAT_vCheckSampleCheduleTask(void *pvParameters)
{

	for (;;)
	{

		// Wait 1s notification from PLATFORM module bliptask
		// Todo: Use events for 1s actions in the future
		xTaskNotifyWait(
			0x00,            // Don't clear bits on entry
			ULONG_MAX,       // Clear all bits on exit
			NULL,// Where the value is stored
			portMAX_DELAY
		);

		if (TIMERS_bTimerIsExpired(&tBatSampleIntervalTmr))
		{
			// Notify sample task with delta limiting
			if(!SYSTEM_bIsDeepSleepActive()) {
				xTaskNotify(BAT_vSampleTask_handle, BAT_NOTIFY_DELTA_LIMIT, eSetBits);
			} else {
				BSP_LED_On(LED_RED);
			}

			TIMERS_vTimerStart(&tBatSampleIntervalTmr, BAT_SAMPLE_INTERVAL);
		}

	}

    vTaskDelete(NULL);
}

/**
 * @brief	Immediately fill the battery sample buffer
 *
 * In order to calculate measured battery voltage, many samples are averaged.
 * In the following cases the buffer is typically filled using a single fresh
 * sample:
 * - a) at initialization, to avoid gradual rise of measured battery voltage
 *      due to buffer filled from periodic sampling, and
 * - b) when a charger is connected or disconnected, to immediately observe
 *      the resulting step in measured battery voltage.
 *
 * @note The function is blocking.
 *
 * @note The coroutine context ccrBatUpdate is modified due to the fact that
 * an updated sample is "forced".
 */
void BAT_vPurgeBuffer(void)
{
    TaskHandle_t purgeHandle = NULL;
    BaseType_t status;

    /* Create a one-shot purge task each time a purge is needed and store its handle */
    status = xTaskCreate(BAT_vBufferPurgeTask,
                         "PurgeTask",
                         BAT_PURGETASK_STACK_SIZE,
                         NULL,
                         BAT_PURGETASK_PRIORITY,
                         &purgeHandle);

    configASSERT(status == pdPASS);

    /* Publish the handle so the sample task can notify it when sample is done.
       Do this after creation, under critical section to avoid race with sample task. */
    taskENTER_CRITICAL();
    BAT_vBufferPurgeTask_handle = purgeHandle;
    taskEXIT_CRITICAL();
}

/**
 * @brief   Return an average of the previous BAT_AVG_FACTOR number of samples
 *
 * @param   u16Voltage	Sampled voltage
 * @return  uint8_t		The percentage
 */
uint16_t BAT_u16GetVoltage(void)
{
    uint8_t u8Temp;
    uint32_t u32Temp;

    // Calculate average from buffered samples
    u32Temp = 0;
    for (u8Temp = 0; u8Temp < BAT_AVG_FACTOR; u8Temp++) {
        u32Temp += au16BatAvgBuf[u8Temp];
    }
    u32Temp = u32Temp / (uint32_t)BAT_AVG_FACTOR;

    // Convert avg to mV
    return MATH_FUNC_i16ConvLin((uint16_t) u32Temp, BAT_MEAS_ADC_M_NUM, BAT_MEAS_ADC_M_DEN, BAT_MEAS_ADC_C);
}


/**
 * @brief   Convert a sampled voltage to a percentage
 *
 * @param   u16Voltage	Sampled voltage
 * @return  uint8_t		The percentage
 */
uint8_t BAT_u8ConvertVoltageToPercentage(uint16_t u16Voltage)
{
    uint32_t u32Temp;

    // van Marius ontlaaikurwe: 98% ontlaai by 3.3V
    // Charger Vreg so laag as 4.168 - kies 4.100

    // Get avg value in mV and convert to percentage
    u32Temp = max(u16Voltage, 3350);
    u32Temp = u32Temp - 3350;
    u32Temp = 100 * u32Temp;
    u32Temp = min(u32Temp / (4100 - 3350), 100);

    return (uint8_t) u32Temp;
}

uint8_t BAT_u8GetPercentage(void)
{
    return BAT_u8ConvertVoltageToPercentage(BAT_u16GetVoltage());
}
