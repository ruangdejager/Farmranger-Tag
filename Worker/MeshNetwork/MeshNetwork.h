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
#define MESH_BASE_HOP_DELAY_MS					100
#define MESH_REPLY_JITTER_WINDOW_MS				500
#define MESH_DREQ_FLOOD_DELAY_MS				10000

// --- PACKET STRUCTURES (Common to the mesh protocol) ---
// --- These are found in the protobuf header files ---


/**
 * @brief Structure to hold discovered neighbors to be returned to the application layer.
 */
typedef struct {
    uint16_t    device_id;
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
void MeshNetwork_init(void);

/**
 * @brief Initiates a new discovery round.
 * Only called by the primary discovery device.
 * @param dreq_id The unique ID for this discovery round.
 * @param original_dreq_sender_id The ID of the device initiating the DReq.
 * @return true if the DReq initiation event was successfully sent, false otherwise.
 */
bool MeshNetwork_start_discovery_round(uint32_t dreq_id, uint16_t original_dreq_sender_id);


/**
 * @brief Retrieves the currently discovered neighbors.
 * This function is typically called by the application layer after a discovery round.
 * @param buffer Pointer to an array of MeshDiscoveredNeighbor_t to fill.
 * @param max_entries Maximum number of entries the buffer can hold.
 * @param actual_entries Pointer to store the actual number of entries copied.
 * @return true if neighbors were retrieved, false if buffer is too small or no neighbors.
 */
bool MeshNetwork_get_discovered_neighbors(MeshDiscoveredNeighbor_t *buffer, uint16_t max_entries, uint16_t *actual_entries);

/**
 * @brief Clears the internal discovered neighbors cache.
 * Should be called by the application layer before a new discovery round if desired.
 */
void MeshNetwork_clear_discovered_neighbors(void);

/**
 * @brief FreeRTOS Task for the Mesh Network layer.
 * Handles DReq processing, DRep generation, and prioritized relaying.
 */
void vMeshNetworkTask(void *pvParameters);

// --- FREE_RTOS TIMER PROTOTYPES ---
void MESHNETWORK_vReplyTimerCallback(TimerHandle_t xTimer);

#endif /* WORKER_MESHNETWORK_MESHNETWORK_H_ */
