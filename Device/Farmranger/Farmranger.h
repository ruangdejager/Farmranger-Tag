/*
 * Farmranger.h
 *
 *  Created on: Nov 24, 2025
 *      Author: Ruan de Jager
 */

#ifndef DEVICE_FARMRANGER_FARMRANGER_H_
#define DEVICE_FARMRANGER_FARMRANGER_H_

#include "stdbool.h"
#include "hal_uart.h"
#include "hal_gpio.h"
#include "hal_bsp.h"
#include "MeshNetwork.h"

#define FR_DRIVER_vInitFRDevice(drv)            HAL_UART_vSetup(drv, DEBUG_UART, FLOWCONTROL_NONE)
#define FR_DRIVER_vEnableUart(drv)              HAL_UART_vEnable(drv)
#define FR_DRIVER_vDisableUart(drv)           	HAL_UART_vDisable(drv)

#define FR_DRIVER_vUartPutBute(drv, byte)       HAL_UART_vTxPutByte(drv, byte)

#define FR_DRIVER_vIntEnable() 					HAL_GPIO_WritePin(BSP_FR_GPIO_INT_PORT, BSP_FR_GPIO_INT_PIN, GPIO_PIN_SET)
#define FR_DRIVER_vIntDisable() 				HAL_GPIO_WritePin(BSP_FR_GPIO_INT_PORT, BSP_FR_GPIO_INT_PIN, GPIO_PIN_RESET)


void FARMRANGER_vInit(void);
void FARMRANGER_vRxTask(void *parameters);
bool FARMRANGER_bDeviceOn(void);
void FARMRANGER_vDeviceOff(void);
uint64_t FARMRANGER_u64RequestTimestamp(void);
bool FARMRANGER_bLogData(MeshDiscoveredNeighbor_t * neighbors, uint16_t count);


#endif /* DEVICE_FARMRANGER_FARMRANGER_H_ */
