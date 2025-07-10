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
#include <stdint.h>
#include <stdbool.h>

// --- CONFIGURABLE PARAMETERS (from DeviceDiscovery or build system) ---
extern uint16_t MESH_DEVICE_ID;
extern uint8_t MESH_MAX_TTL;
extern uint16_t MESH_MAX_NEIGHBORS_PER_PACKET;
extern uint16_t MESH_MAX_LOCAL_DISCOVERED_DEVICES;
extern TickType_t MESH_BASE_HOP_DELAY_MS;
extern TickType_t MESH_REPLY_JITTER_WINDOW_MS;
extern TickType_t MESH_DREQ_FLOOD_DELAY_MS;

// --- PACKET STRUCTURES (Common to the mesh protocol) ---

// Common header for all mesh packets
typedef struct __attribute__((packed)) {
    uint8_t     packet_type; // MESH_PACKET_TYPE_DREQ or MESH_PACKET_TYPE_DREP
    uint16_t    sender_id;
    uint32_t    dreq_id;     // Unique ID for a discovery round
} MeshPacketHeader_t;

// Discovery Request Packet (DReq)
#define MESH_PACKET_TYPE_DREQ        (0x01)
typedef struct __attribute__((packed)) {
    MeshPacketHeader_t  header;
    uint8_t             ttl;         // Time-to-live for flooding
} MeshDiscoveryRequestPacket_t;

// Neighbor information entry (for replies)
typedef struct __attribute__((packed)) {
    uint16_t    device_id;
    uint8_t     hop_count; // Hops from Original_DReq_Sender_ID to this device
    int16_t     rssi;      // RSSI of the DReq packet received by this device
    int8_t      snr;       // SNR of the DReq packet received by this device
} MeshNeighborInfo_t;

// Discovery Reply Packet (DRep)
#define MESH_PACKET_TYPE_DREP        (0x02)
typedef struct __attribute__((packed)) {
    MeshPacketHeader_t  header;
    uint16_t            original_dreq_sender_id; // The ID of the device that initiated the DReq
    uint16_t            parent_id;               // The ID of the immediate upstream parent this reply is for
    uint8_t             num_neighbors;           // Number of neighbor entries in this packet
    MeshNeighborInfo_t  neighbors[1];            // Flexible array member (size determined at runtime)
                                                 // Actual size will be num_neighbors * sizeof(MeshNeighborInfo_t)
    uint8_t             crc;                     // Simple CRC for integrity
} MeshDiscoveryReplyPacket_t;

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
 * @return true if the DReq was successfully queued, false otherwise.
 */
bool MeshNetwork_start_discovery_round(uint32_t dreq_id, uint16_t original_dreq_sender_id);

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

#endif /* WORKER_MESHNETWORK_MESHNETWORK_H_ */
