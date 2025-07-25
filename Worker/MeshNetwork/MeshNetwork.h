/*
 * MeshNetwork.h
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */

#ifndef WORKER_MESHNETWORK_MESHNETWORK_H_
#define WORKER_MESHNETWORK_MESHNETWORK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"
#include <stdint.h>
#include <stdbool.h>

#include "meshNetwork.pb.h"

// --- CONFIGURABLE PARAMETERS (from DeviceDiscovery or build system) ---
#define MESH_MAX_TTL							4
#define MESH_MAX_NEIGHBORS_PER_PACKET			5
#define MESH_MAX_LOCAL_DISCOVERED_DEVICES		16
#define MESH_BASE_HOP_DELAY_MS					200
#define MESH_REPLY_JITTER_WINDOW_MS				1000
#define MESH_DREQ_FLOOD_DELAY_MS				10000

// --- PACKET STRUCTURES (Common to the mesh protocol) ---
// --- These are found in the protobuf header files ---
/**
 * @brief Enum to define set wake-up intervals.
 */
typedef enum {
    WAKEUP_INTERVAL_15_MIN  = 1, // Represents 15 minutes
    WAKEUP_INTERVAL_30_MIN  = 2, // Represents 30 minutes
    WAKEUP_INTERVAL_60_MIN  = 3, // Represents 60 minutes (1 hour)
    WAKEUP_INTERVAL_120_MIN = 4, // Represents 120 minutes (2 hours)
    WAKEUP_INTERVAL_MAX_COUNT = 4 // Not a real interval, just for array sizing/validation
} WakeupInterval;

/**
 * @brief Structure to hold discovered neighbors to be returned to the application layer.
 */
typedef struct {
    uint32_t    device_id;
    uint8_t     hop_count;
    int16_t     rssi;
    int8_t      snr;
    TickType_t  last_seen;
} MeshDiscoveredNeighbor_t;


// --- PUBLIC API FOR MESH NETWORK LAYER ---

/**
 * @brief Initializes the Mesh Network layer.
 * Creates FreeRTOS resources and the Mesh Network task.
 */
void MESHNETWORK_vInit(void);

/**
 * @brief Encodes the DReq message into a protobuf buffer..
 */
bool MESHNETWORK_bEncodeDReqMessage(MeshDReqPacket * pMeshDReqPacket, uint8_t * buffer, uint16_t buffer_length, uint8_t * message_length);

/**
 * @brief Encodes the DRep message into a protobuf buffer..
 */
bool MESHNETWORK_bEncodeDRepMessage(MeshDRepPacket * pMeshDRepPacket, uint8_t * buffer, uint16_t buffer_length, uint8_t * message_length);

/**
 * @brief Initiates a new discovery round.
 * Only called by the primary discovery device.
 * @param dreq_id The unique ID for this discovery round.
 * @param original_dreq_sender_id The ID of the device initiating the DReq.
 * @return true if the DReq initiation event was successfully sent, false otherwise.
 */
bool MESHNETWORK_bStartDiscoveryRound(uint32_t dreq_id, uint32_t original_dreq_sender_id);


/**
 * @brief Retrieves the currently discovered neighbors.
 * This function is typically called by the application layer after a discovery round.
 * @param buffer Pointer to an array of MeshDiscoveredNeighbor_t to fill.
 * @param max_entries Maximum number of entries the buffer can hold.
 * @param actual_entries Pointer to store the actual number of entries copied.
 * @return true if neighbors were retrieved, false if buffer is too small or no neighbors.
 */
bool MESHNETWORK_bGetDiscoveredNeighbors(MeshDiscoveredNeighbor_t *buffer, uint16_t max_entries, uint16_t *actual_entries);

/**
 * @brief Clears the internal discovered neighbors cache.
 * Should be called by the application layer before a new discovery round if desired.
 */
void MESHNETWORK_vClearDiscoveredNeighbors(void);

/**
 * @brief FreeRTOS Task for handling parsed DReq and DRep packets.
 */
void MESHNETWORK_vMeshRxEventTask(void *pvParameters); // Renamed from vMeshNetworkTask

/**
 * @brief FreeRTOS Task for managing DReq initiation and DRep relaying scheduling.
 */
void MESHNETWORK_vReplySchedulerTask(void *pvParameters);

/**
 * @brief FreeRTOS Task for parsing raw LoRa packets into mesh-specific types.
 */
void MESHNETWORK_vParserTask(void *pvParameters);

// --- FREE_RTOS TIMER PROTOTYPES ---
void MESHNETWORK_vReplyTimerCallback(TimerHandle_t xTimer);

bool MESHNETWORK_bEncodeTimesyncMessage(TimeSyncMessage * pTimeSyncMessage, uint8_t * buffer, uint16_t buffer_length, uint8_t * message_length);
void MESHNETWORK_vHandleTimeSyncMessage(const TimeSyncMessage *time_sync_msg);
void MESHNETWORK_vSendTimeSyncMessage(uint32_t timesync_id, uint32_t utc_timestamp, uint32_t wakeup_interval_enum_val); // Changed param name to clarify it's enum val

// --- Public Accessors for Wakeup Interval ---
uint8_t MESHNETWORK_u8GetWakeupInterval(void); // Returns current interval in minutes
void MESHNETWORK_vSetWakeupInterval(WakeupInterval new_interval);

#endif /* WORKER_MESHNETWORK_MESHNETWORK_H_ */
