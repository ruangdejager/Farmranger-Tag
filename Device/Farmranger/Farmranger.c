/*
 * Farmranger.c
 *
 *  Created on: Nov 24, 2025
 *      Author: Ruan de Jager
 */

#include "Farmranger.h"
#include "FreeRTOS.h"
#include "task.h"

#include "dbg_log.h"

#include "str.h"

#include <string.h>

#define FR_RX_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
#define FR_RX_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE)

// --- PRIVATE FREE_RTOS TASK HANDLEs ---
TaskHandle_t Farmranger_vRxTask_handle;

// Buffer to build NMEA msgs before parsing
#define FR_RX_BUF_LEN	128
char acFrRxBuf[FR_RX_BUF_LEN];
uint8_t u8FrRxBufIdx;

bool bFRDeviceOn;

struct _farmranger_s
{
	hal_uart_t 	UartHandle;
	uint8_t		byte;
}farmranger;

void FARMRANGER_vInit(void)
{

	bFRDeviceOn = false;

	// Init UART
	FR_DRIVER_vInitFRDevice(&farmranger.UartHandle);
	// UART interface will be enabled/disabled at Farmranger device activation

    BaseType_t status;
    status = xTaskCreate(FARMRANGER_vRxTask,
            "FarmrangerRxTask",
            FR_RX_TASK_STACK_SIZE,
            NULL,
            FR_RX_TASK_PRIORITY,
			&Farmranger_vRxTask_handle);
    configASSERT(status == pdPASS);

}

void FARMRANGER_vRxTask(void *parameters)
{

	for (;;)
	{

//		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(1));
		while ( UART_bReadByte(&farmranger.UartHandle, &farmranger.byte) )
		{
			// Store to buffer
			acFrRxBuf[u8FrRxBufIdx] = farmranger.byte;
			u8FrRxBufIdx++;
			// NULL-terminate in order to eval
			acFrRxBuf[u8FrRxBufIdx] = 0;
		}

	}

}

void FARMRANGER_vDeviceOn(void)
{

	if (!bFRDeviceOn)
	{
		u8FrRxBufIdx = 0;
		// Enable the uart peripheral
		FR_DRIVER_vEnableUart(&farmranger.UartHandle);
		// Drive the interrupt gpio high to activate farmranger
		FR_DRIVER_vIntEnable();

		// Wait for "RDY" on the uart
		DBG("Wait for RDY...\r\n");
		do
		{
			// Start a timer to exit if Ready not found
			//
			//
			// Yield task while waiting
			vTaskDelay(pdMS_TO_TICKS(1));
		} while ( NULL == memmem(acFrRxBuf, u8FrRxBufIdx, "RDY\r\n", strlen("RDY\r\n") ) );

		bFRDeviceOn = true;
	}

	DBG("Farmranger Ready.\r\n");

}

void FARMRANGER_vDeviceOff(void)
{

	// Enable the uart peripheral
	FR_DRIVER_vDisableUart(&farmranger.UartHandle);
	// Drive the interrupt gpio high to activate farmranger
	FR_DRIVER_vIntDisable();

	bFRDeviceOn = false;
	DBG("Farmranger released.\r\n");

}

uint64_t FARMRANGER_u64RequestTimestamp(void)
{

	uint64_t timestamp = 7200;

	if (!bFRDeviceOn)
	{
		FARMRANGER_vDeviceOn();
	}

	// Send TS request cmd

	return timestamp;

}

