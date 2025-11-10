/*
 * GPS_driver.h
 *
 *  Created on: Nov 5, 2025
 *      Author: Ruan de Jager
 */

#ifndef DEVICE_GPS_GPS_DRIVER_H_
#define DEVICE_GPS_GPS_DRIVER_H_

#include "hal_bsp.h"
#include "hal_gpio.h"
#include "hal_uart.h"

#define GPS_DRIVER_vInitGPS(drv)                        HAL_UART_vSetup(drv, GPS_UART, FLOWCONTROL_NONE)

#define GPS_DRIVER_vEnableUart(drv)                     HAL_UART_vEnable(drv)
#define GPS_DRIVER_vDisableUart(drv)                    HAL_UART_vDisable(drv)
#define GPS_DRIVER_vClearBuffer(drv)                    HAL_UART_vClearBuffer(drv)

#define GPS_DRIVER_vPowerEnHigh()\
{\
	HAL_GPIO_WritePin(BSP_GPS_VCC_EN_PORT, BSP_GPS_VCC_EN_PIN, GPIO_PIN_SET);\
	HAL_GPIO_WritePin(BSP_GPS_0V9_EN_PORT, BSP_GPS_0V9_EN_PIN, GPIO_PIN_SET);\
}
#define GPS_DRIVER_bPowerEnGet()                        HAL_GPIO_ReadPin(BSP_GPS_VCC_EN_PORT, BSP_GPS_VCC_EN_PIN)
#define GPS_DRIVER_vPowerEnLow()\
{\
	HAL_GPIO_WritePin(BSP_GPS_VCC_EN_PORT, BSP_GPS_VCC_EN_PIN, GPIO_PIN_RESET);\
	HAL_GPIO_WritePin(BSP_GPS_0V9_EN_PORT, BSP_GPS_0V9_EN_PIN, GPIO_PIN_RESET);\
}

#endif /* DEVICE_GPS_GPS_DRIVER_H_ */
