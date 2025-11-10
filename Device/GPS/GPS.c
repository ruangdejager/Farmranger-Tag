/*
 * GPS.c
 *
 *  Created on: Nov 5, 2025
 *      Author: Ruan de Jager
 */

#include "gps.h"
#include "debug_uart_output.h"

#define GPS_RX_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
#define GPS_RX_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE)

//#define GPS_MODULE_ON_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
//#define GPS_MODULE_ON_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 2)
//
//#define GPS_MODULE_OFF_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
//#define GPS_MODULE_OFF_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 2)

// --- PRIVATE FREE_RTOS TASK HANDLEs ---
TaskHandle_t GPS_vRxTask_handle;
//TaskHandle_t GPS_vModuleOnTask_handle;
//TaskHandle_t GPS_vModuleOffTask_handle;

// Buffer to build NMEA msgs before parsing
#define GPS_RX_BUF_LEN	128
char acGpsRxBuf[GPS_RX_BUF_LEN];
uint8_t u8GpsRxBufIdx;
uint32_t u32GnssRxLastTs;

struct _gps_s
{
	hal_uart_t 	UartHandle;
	uint8_t		byte;
	uint32_t	upTimeCounter;
} gps;


void GPS_vInit(void)
{

	// Init UART
	GPS_DRIVER_vInitGPS(&gps.UartHandle);
	// UART interface will be enabled/disabled at EG91 powerup/powerdown
	GPS_DRIVER_vEnableUart(&gps.UartHandle);
	// Drive the 1.8V VCC and 0.9V VCC_RF and VCC_CORE high
	GPS_DRIVER_vPowerEnHigh();
	// Here we make the reset pin high impedance, the reset pin is internally pulled high.
	HAL_GPIO_vInitInput(BSP_GPS_RESET_PORT, BSP_GPS_RESET_PIN, GPIO_NOPULL);

    BaseType_t status;
    status = xTaskCreate(GPS_vRxTask,
            "GPSRxTask",
            GPS_RX_TASK_STACK_SIZE,
            NULL,
            GPS_RX_TASK_PRIORITY,
			&GPS_vRxTask_handle);
    configASSERT(status == pdPASS);
//    status = xTaskCreate(GPS_vModuleOnTask,
//            "GPSModuleOnTask",
//            GPS_MODULE_ON_TASK_STACK_SIZE,
//            NULL,
//            GPS_MODULE_ON_TASK_PRIORITY,
//			&GPS_vModuleOnTask_handle);
//    configASSERT(status == pdPASS);
//    status = xTaskCreate(GPS_vModuleOffTask,
//            "GPSModuleOffTask",
//            GPS_MODULE_OFF_TASK_STACK_SIZE,
//            NULL,
//            GPS_MODULE_OFF_TASK_PRIORITY,
//			&GPS_vModuleOffTask_handle);
//    configASSERT(status == pdPASS);



}

void GPS_vRxTask(void *parameters)
{

	for (;;)
	{
		// Wait indefinitely for an RX ISR on the LORA radio
//		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(1));
		while ( UART_bReadByte(&gps.UartHandle, &gps.byte) )
		{
			DBG_UART_vPutByte(gps.byte);
//			u32GnssRxLastTs = HAL_TIMER_u32GetValue();
//			GPS_bOnRxByte(gps.byte);
		}

	}

}
