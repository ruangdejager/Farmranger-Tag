/*
 * Movement_Driver.h
 *
 *  Created on: Nov 11, 2025
 *      Author: Ruan de Jager
 */

#ifndef WORKER_MOVEMENT_MOVEMENT_DRIVER_H_
#define WORKER_MOVEMENT_MOVEMENT_DRIVER_H_

#include "hal_bsp.h"
#include <stdbool.h>
#include "Acc.h"


#define MOVE_DRIVER_acclSampleTypedef               acc_t

#define MOVE_DRIVER_u8GetAccelDeviceId()            ACC_u8GetDeviceId()
#define MOVE_DRIVER_u8NumSamplesInFifo()            ACC_u8NumSamplesInFifo()
#define MOVE_DRIVER_vGetAccSample(x)                ACC_vGetAccSample(x)
#define MOVE_DRIVER_vAccelInit()                    ACC_vInit()

#define MOVE_DRIVER_vUSBPutValues                   HAL_USB_vPutMoveMessage



#define MOVE_DRIVER_vLedOn()                        BSP_LED_On(LED_GREEN)
#define MOVE_DRIVER_vLedOff()                       BSP_LED_Off(LED_GREEN)

#endif /* WORKER_MOVEMENT_MOVEMENT_DRIVER_H_ */
