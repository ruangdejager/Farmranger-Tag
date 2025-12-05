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

// --- CONFIGURABLE APPLICATION PARAMETERS ---
#define APP_DEVICE_ID               (1) // This device's unique ID
#define APP_IS_PRIMARY_DEVICE       (APP_DEVICE_ID >= 1 && APP_DEVICE_ID <= 3) // Define if this device is a primary initiator

// Discovery Timing
#define APP_WAKEUP_BUFFER_MS        (5 * 1000) // 30 seconds buffer after synchronized wake-up
#define APP_DISCOVERY_WINDOW_MS     (15 * 1000) // 60 seconds for the entire discovery process

// Mesh Network Parameters (passed to MeshNetwork layer)
#define APP_MESH_MAX_TTL                 (4) // Max hops for DReq flooding
#define APP_MESH_MAX_NEIGHBORS_PER_PACKET (5) // Max neighbor entries in a single DISCOVERY_REPLY packet
#define APP_MESH_MAX_LOCAL_DISCOVERED_DEVICES (50) // Max devices an intermediate node can buffer for relaying
#define APP_MESH_BASE_HOP_DELAY_MS       (5 * 1000) // Base delay per hop for replies (e.g., 5 seconds)
#define APP_MESH_REPLY_JITTER_WINDOW_MS  (500) // Random jitter within a hop's reply window (0-500ms)
#define APP_MESH_DREQ_FLOOD_DELAY_MS     (50) // Small delay between DReq re-broadcasts

// Event Group Bits for synchronized wake-up
#define DISCOVERY_WAKEUP_BIT    (1UL << 0UL) // Set by external RTC/timer for synchronized wake-up

/**
 * @brief Enum to define set wake-up intervals.
 */
typedef enum {
	DEVICE_SECONDARY = 0,
	DEVICE_PRIMARY
} DiscoveryDeviceRole;

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
void DEVICE_DISCOVERY_vSendTS(void);
DiscoveryDeviceRole DEVICE_DISCOVERY_tGetDeviceRole(void);

#endif /* TASKS_DEVICEDISCOVERY_DEVICEDISCOVERY_H_ */
