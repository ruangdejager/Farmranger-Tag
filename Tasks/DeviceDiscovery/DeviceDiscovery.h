/*
 * DeviceDiscovery.h
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */

#ifndef TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_
#define TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_

#include "FreeRTOS.h"
#include "event_groups.h"
#include "MeshNetwork.h" // Includes MeshDiscoveredNeighbor_t

// Discovery Timing
#define APP_WAKEUP_BUFFER_MS        (10 * 1000) // 10 seconds buffer after synchronized wake-up
#define APP_DISCOVERY_WINDOW_MS     (60 * 1000) // 60 seconds for the entire discovery process

// Event Group Bits for synchronized wake-up
#define DISCOVERY_WAKEUP_BIT    (1UL << 0UL) // Set by external RTC/timer for synchronized wake-up

#define DEVICE_DISCOVERY_DRIVER_bConnectLogger() 						FARMRANGER_bDeviceOn()
#define DEVICE_DISCOVERY_DRIVER_vDisconnectLogger() 					FARMRANGER_vDeviceOff()
#define DEVICE_DISCOVERY_DRIVER_u64RequestTS()							FARMRANGER_u64RequestTimestamp()
#define DEVICE_DISCOVERY_bSendDiscoveryData(items, size)				FARMRANGER_bLogData(items, size)

/**
 * @brief Initializes the Device Discovery application layer.
 * Creates FreeRTOS resources for this layer and configures the MeshNetwork layer.
 */
void DEVICE_DISCOVERY_vInit(void);

/**
 * @brief FreeRTOS Task for the Device Discovery application.
 * Manages the overall discovery process, including wake-up, initiation (for primary),
 * and result collection.
 */
void DEVICE_DISCOVERY_vAppTask(void *pvParameters);

/**
 * @brief External function to simulate a synchronized wake-up event.
 * In a real system, this would be called by an RTC or timer ISR.
 */
void DEVICE_DISCOVERY_vCheckWakeupSchedule(void);

DeviceRole_e DEVICE_DISCOVERY_eGetDeviceRole(void);

#endif /* TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_ */
