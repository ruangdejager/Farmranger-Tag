/*
 * MeshNetwork.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */


#include "MeshNetwork.h"
#include "LoraRadio.h" // Interface to the LoRa radio driver
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h> // For memcpy, memset
#include <stdio.h>  // For printf (for debugging)
#include <stdlib.h> // For rand()

// --- CONFIGURABLE PARAMETERS (Extern declarations from MeshNetwork.h) ---
uint16_t MESH_DEVICE_ID; // Must be set by DeviceDiscovery or main
uint8_t MESH_MAX_TTL;
uint16_t MESH_MAX_NEIGHBORS_PER_PACKET;
uint16_t MESH_MAX_LOCAL_DISCOVERED_DEVICES;
TickType_t MESH_BASE_HOP_DELAY_MS;
TickType_t MESH_REPLY_JITTER_WINDOW_MS;
TickType_t MESH_DREQ_FLOOD_DELAY_MS;

// --- PRIVATE DEFINES ---
#define MESH_TASK_PRIORITY      (configMAX_PRIORITIES - 2)
#define MESH_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 6) // Increased stack for processing
#define MESH_EVENT_QUEUE_SIZE   (15) // Queue for events from LoraRadio

// --- INTERNAL DATA STRUCTURES ---

// Represents a discovered neighbor in the local cache of an intermediate node
typedef struct {
    uint16_t    device_id;
    uint8_t     hop_count; // Hops from Original_DReq_Sender_ID to this device
    int16_t     rssi;      // RSSI of the DReq packet received by this device
    int8_t      snr;       // SNR of the DReq packet received by this device
    TickType_t  last_updated; // For aging out entries
    bool        reported_upstream; // Flag to track if this device's info has been sent upstream
} LocalNeighborEntry_t;

// Cache for the current discovery round at an intermediate node
typedef struct {
    uint32_t            dreq_id;
    uint16_t            original_dreq_sender_id;
    uint16_t            preferred_parent_id; // The node this device will send its reply to
    uint8_t             my_hop_count_to_dreq_sender;
    TickType_t          dreq_received_time;
    LocalNeighborEntry_t local_discovered_devices[MESH_MAX_LOCAL_DISCOVERED_DEVICES];
    uint8_t             num_local_discovered_devices;
    bool                my_info_sent; // Flag to ensure this device's own info is sent once
    TickType_t          my_reply_scheduled_time; // When this node should send its DRep
} DiscoveryCache_t;

static DiscoveryCache_t current_discovery_cache;
static uint32_t last_processed_dreq_id = 0; // To prevent processing old DReqs

// Discovered neighbors for the application layer
#define MESH_MAX_GLOBAL_NEIGHBORS (250) // Max entries in the primary device's global table
static MeshDiscoveredNeighbor_t mesh_discovered_neighbors[MESH_MAX_GLOBAL_NEIGHBORS];
static uint16_t mesh_discovered_neighbors_count = 0;
static SemaphoreHandle_t xMeshNeighborTableMutex; // Protects mesh_discovered_neighbors

// --- INTERNAL FREE_RTOS QUEUES ---
typedef enum {
    MESH_EVENT_DREQ_RECEIVED,
    MESH_EVENT_DREP_RECEIVED,
    MESH_EVENT_INITIATE_DREQ // For primary device to initiate
} MeshEventType_t;

typedef struct {
    MeshEventType_t type;
    LoraRadio_Packet_t packet; // Raw LoRa packet, will be parsed
} MeshEvent_t;

static QueueHandle_t xMeshNetworkEventQueue; // Queue for events from LoraRadio

// --- PRIVATE FUNCTION PROTOTYPES ---
static uint8_t CRC8_calculate(const uint8_t *data, uint16_t len);
static void MeshNetwork_add_or_update_global_neighbor(uint16_t device_id, uint8_t hop_count, int16_t rssi, int8_t snr);
static void MeshNetwork_add_or_update_local_cache(uint16_t device_id, uint8_t hop_count, int16_t rssi, int8_t snr);
static TickType_t MeshNetwork_calculate_reply_delay(uint8_t my_hop_count_to_dreq_sender);
static void MeshNetwork_handle_dreq(const LoraRadio_Packet_t *rx_packet);
static void MeshNetwork_handle_drep(const LoraRadio_Packet_t *rx_packet);
static void MeshNetwork_send_dreq(uint32_t dreq_id, uint16_t sender_id, uint8_t ttl);
static void MeshNetwork_send_drep(void);

// --- PUBLIC FUNCTIONS ---

void MeshNetwork_init(void) {
    xMeshNetworkEventQueue = xQueueCreate(MESH_EVENT_QUEUE_SIZE, sizeof(MeshEvent_t));
    xMeshNeighborTableMutex = xSemaphoreCreateMutex();

    if (xMeshNetworkEventQueue == NULL || xMeshNeighborTableMutex == NULL) {
        printf("MeshNetwork: Failed to create FreeRTOS resources!\r\n");
        while(1);
    }

    memset(&current_discovery_cache, 0, sizeof(DiscoveryCache_t));
    mesh_discovered_neighbors_count = 0;

    xTaskCreate(vMeshNetworkTask,
                "MeshNetworkTask",
                MESH_TASK_STACK_SIZE,
                NULL,
                MESH_TASK_PRIORITY,
                NULL);

    printf("MeshNetwork: Initialized FreeRTOS resources and created MeshNetworkTask.\r\n");
}

bool MeshNetwork_start_discovery_round(uint32_t dreq_id, uint16_t original_dreq_sender_id) {
    MeshEvent_t event;
    event.type = MESH_EVENT_INITIATE_DREQ;
    // Packet data is not needed here, as the task will construct the DReq
    event.packet.len = 0;

    // Set up the cache for the primary device's own DReq
    memset(&current_discovery_cache, 0, sizeof(DiscoveryCache_t));
    current_discovery_cache.dreq_id = dreq_id;
    current_discovery_cache.original_dreq_sender_id = original_dreq_sender_id;
    current_discovery_cache.my_hop_count_to_dreq_sender = 0; // 0 hops for the initiator
    current_discovery_cache.dreq_received_time = xTaskGetTickCount(); // Use current time as reference
    current_discovery_cache.my_reply_scheduled_time = 0; // Not applicable for initiator's own DRep sending logic
    current_discovery_cache.preferred_parent_id = 0; // No parent for initiator

    // Add initiator's own info to its local cache (which is also its global table)
    MeshNetwork_add_or_update_local_cache(MESH_DEVICE_ID, 0, 0, 0); // RSSI/SNR 0 for self

    printf("MeshNetwork: Initiating DReq with ID %lu from %u\r\n", dreq_id, original_dreq_sender_id);
    return xQueueSend(xMeshNetworkEventQueue, &event, portMAX_DELAY) == pdPASS;
}

bool MeshNetwork_get_discovered_neighbors(MeshDiscoveredNeighbor_t *buffer, uint16_t max_entries, uint16_t *actual_entries) {
    if (xSemaphoreTake(xMeshNeighborTableMutex, portMAX_DELAY) == pdTRUE) {
        uint16_t count = (mesh_discovered_neighbors_count < max_entries) ? mesh_discovered_neighbors_count : max_entries;
        memcpy(buffer, mesh_discovered_neighbors, count * sizeof(MeshDiscoveredNeighbor_t));
        *actual_entries = count;
        xSemaphoreGive(xMeshNeighborTableMutex);
        return true;
    }
    *actual_entries = 0;
    return false;
}

void MeshNetwork_clear_discovered_neighbors(void) {
    if (xSemaphoreTake(xMeshNeighborTableMutex, portMAX_DELAY) == pdTRUE) {
        mesh_discovered_neighbors_count = 0;
        memset(mesh_discovered_neighbors, 0, sizeof(mesh_discovered_neighbors));
        xSemaphoreGive(xMeshNeighborTableMutex);
    }
}

// --- FREE_RTOS TASK IMPLEMENTATION ---

void vMeshNetworkTask(void *pvParameters) {
    (void)pvParameters;

    LoraRadio_Packet_t rx_packet;
    MeshEvent_t event;

    for (;;) {
        // 1. Check for incoming raw LoRa packets
        if (LoraRadio_receive_packet(&rx_packet, pdMS_TO_TICKS(10))) { // Short timeout to allow other logic
            // Parse packet type and push to internal event queue
            MeshPacketHeader_t *header = (MeshPacketHeader_t*)rx_packet.data;
            if (header->packet_type == MESH_PACKET_TYPE_DREQ) {
                event.type = MESH_EVENT_DREQ_RECEIVED;
            } else if (header->packet_type == MESH_PACKET_TYPE_DREP) {
                event.type = MESH_EVENT_DREP_RECEIVED;
            } else {
                printf("MeshNetwork: Unknown packet type %u received.\r\n", header->packet_type);
                continue; // Skip unknown packets
            }
            memcpy(&event.packet, &rx_packet, sizeof(LoraRadio_Packet_t));
            xQueueSend(xMeshNetworkEventQueue, &event, portMAX_DELAY);
        }

        // 2. Process internal mesh events
        if (xQueueReceive(xMeshNetworkEventQueue, &event, 0) == pdPASS) { // Non-blocking read
            switch (event.type) {
                case MESH_EVENT_INITIATE_DREQ:
                    MeshNetwork_send_dreq(current_discovery_cache.dreq_id, MESH_DEVICE_ID, MESH_MAX_TTL);
                    break;
                case MESH_EVENT_DREQ_RECEIVED:
                    MeshNetwork_handle_dreq(&event.packet);
                    break;
                case MESH_EVENT_DREP_RECEIVED:
                    MeshNetwork_handle_drep(&event.packet);
                    break;
                default:
                    break;
            }
        }

        // 3. Prioritized Relaying Logic (for all devices that received a DReq)
        // Check if we are in a valid discovery round and it's time to send our DRep/Relayed_DRep
        if (current_discovery_cache.dreq_id != 0 && // Valid discovery round active
            MESH_DEVICE_ID != current_discovery_cache.original_dreq_sender_id && // Not the original sender
            current_discovery_cache.preferred_parent_id != 0 && // Has a preferred parent
            xTaskGetTickCount() >= current_discovery_cache.my_reply_scheduled_time) { // It's our scheduled time

            MeshNetwork_send_drep(); // This function will handle constructing and sending the DRep
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield CPU
    }
}

// --- PRIVATE HELPER FUNCTIONS ---

static uint8_t CRC8_calculate(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

static void MeshNetwork_add_or_update_global_neighbor(uint16_t device_id, uint8_t hop_count, int16_t rssi, int8_t snr) {
    if (xSemaphoreTake(xMeshNeighborTableMutex, portMAX_DELAY) == pdTRUE) {
        bool found = false;
        for (uint16_t i = 0; i < mesh_discovered_neighbors_count; i++) {
            if (mesh_discovered_neighbors[i].device_id == device_id) {
                // Update if new info is better (lower hop count, or stronger RSSI if hop count is same)
                if (hop_count < mesh_discovered_neighbors[i].hop_count ||
                    (hop_count == mesh_discovered_neighbors[i].hop_count && rssi > mesh_discovered_neighbors[i].rssi)) {
                    mesh_discovered_neighbors[i].hop_count = hop_count;
                    mesh_discovered_neighbors[i].rssi = rssi;
                    mesh_discovered_neighbors[i].snr = snr;
                }
                mesh_discovered_neighbors[i].last_seen = xTaskGetTickCount();
                found = true;
                break;
            }
        }
        if (!found && mesh_discovered_neighbors_count < MESH_MAX_GLOBAL_NEIGHBORS) {
            mesh_discovered_neighbors[mesh_discovered_neighbors_count].device_id = device_id;
            mesh_discovered_neighbors[mesh_discovered_neighbors_count].hop_count = hop_count;
            mesh_discovered_neighbors[mesh_discovered_neighbors_count].rssi = rssi;
            mesh_discovered_neighbors[mesh_discovered_neighbors_count].snr = snr;
            mesh_discovered_neighbors[mesh_discovered_neighbors_count].last_seen = xTaskGetTickCount();
            mesh_discovered_neighbors_count++;
        }
        xSemaphoreGive(xMeshNeighborTableMutex);
    }
}

static void MeshNetwork_add_or_update_local_cache(uint16_t device_id, uint8_t hop_count, int16_t rssi, int8_t snr) {
    bool found = false;
    for (uint8_t i = 0; i < current_discovery_cache.num_local_discovered_devices; i++) {
        if (current_discovery_cache.local_discovered_devices[i].device_id == device_id) {
            if (hop_count < current_discovery_cache.local_discovered_devices[i].hop_count ||
                (hop_count == current_discovery_cache.local_discovered_devices[i].hop_count && rssi > current_discovery_cache.local_discovered_devices[i].rssi)) {
                current_discovery_cache.local_discovered_devices[i].hop_count = hop_count;
                current_discovery_cache.local_discovered_devices[i].rssi = rssi;
                current_discovery_cache.local_discovered_devices[i].snr = snr;
            }
            current_discovery_cache.local_discovered_devices[i].last_updated = xTaskGetTickCount();
            current_discovery_cache.local_discovered_devices[i].reported_upstream = false; // Mark as not yet reported
            found = true;
            break;
        }
    }
    if (!found && current_discovery_cache.num_local_discovered_devices < MESH_MAX_LOCAL_DISCOVERED_DEVICES) {
        current_discovery_cache.local_discovered_devices[current_discovery_cache.num_local_discovered_devices].device_id = device_id;
        current_discovery_cache.local_discovered_devices[current_discovery_cache.num_local_discovered_devices].hop_count = hop_count;
        current_discovery_cache.local_discovered_devices[current_discovery_cache.num_local_discovered_devices].rssi = rssi;
        current_discovery_cache.local_discovered_devices[current_discovery_cache.num_local_discovered_devices].snr = snr;
        current_discovery_cache.local_discovered_devices[current_discovery_cache.num_local_discovered_devices].last_updated = xTaskGetTickCount();
        current_discovery_cache.local_discovered_devices[current_discovery_cache.num_local_discovered_devices].reported_upstream = false;
        current_discovery_cache.num_local_discovered_devices++;
    }
}

static TickType_t MeshNetwork_calculate_reply_delay(uint8_t my_hop_count_to_dreq_sender) {
    TickType_t delay_ms = (TickType_t)my_hop_count_to_dreq_sender * MESH_BASE_HOP_DELAY_MS;
    delay_ms += (rand() % MESH_REPLY_JITTER_WINDOW_MS); // Add random jitter
    return delay_ms;
}

static void MeshNetwork_handle_dreq(const LoraRadio_Packet_t *rx_packet) {
    MeshDiscoveryRequestPacket_t *dreq = (MeshDiscoveryRequestPacket_t*)rx_packet->data;

    // Check if this is a new DReq round or a better path for current round
    // Only process if it's a newer DReq ID, or same ID but better (lower) hop count
    uint8_t received_hop_count = MESH_MAX_TTL - dreq->ttl + 1;

    if (dreq->header.dreq_id > last_processed_dreq_id ||
        (dreq->header.dreq_id == last_processed_dreq_id && received_hop_count < current_discovery_cache.my_hop_count_to_dreq_sender)) {

        // If new DReq round, reset cache and update last_processed_dreq_id
        if (dreq->header.dreq_id > last_processed_dreq_id) {
            memset(&current_discovery_cache, 0, sizeof(DiscoveryCache_t));
            current_discovery_cache.dreq_id = dreq->header.dreq_id;
            current_discovery_cache.original_dreq_sender_id = dreq->header.sender_id;
            current_discovery_cache.dreq_received_time = xTaskGetTickCount();
            last_processed_dreq_id = dreq->header.dreq_id;
            printf("MeshNetwork: New DReq round %lu started.\r\n", dreq->header.dreq_id);
        }

        // Update my hop count and preferred parent if this path is better
        if (current_discovery_cache.my_hop_count_to_dreq_sender == 0 || received_hop_count < current_discovery_cache.my_hop_count_to_dreq_sender) {
            current_discovery_cache.my_hop_count_to_dreq_sender = received_hop_count;
            current_discovery_cache.preferred_parent_id = dreq->header.sender_id; // The node that sent this DReq
            current_discovery_cache.my_reply_scheduled_time = current_discovery_cache.dreq_received_time +
                                                              MeshNetwork_calculate_reply_delay(current_discovery_cache.my_hop_count_to_dreq_sender);

            printf("MeshNetwork: DReq from %u (hop %u), new preferred parent. Scheduled reply at %lu\r\n",
                   dreq->header.sender_id, current_discovery_cache.my_hop_count_to_dreq_sender, current_discovery_cache.my_reply_scheduled_time);

            // If this device is the original sender, it doesn't re-broadcast
            if (MESH_DEVICE_ID != dreq->header.sender_id && dreq->ttl > 0) {
                // Re-broadcast DReq with decremented TTL
                MeshNetwork_send_dreq(dreq->header.dreq_id, MESH_DEVICE_ID, dreq->ttl - 1);
            }
        }

        // Add this device's own info to its local cache if not already there
        MeshNetwork_add_or_update_local_cache(MESH_DEVICE_ID,
                                              current_discovery_cache.my_hop_count_to_dreq_sender,
                                              rx_packet->rssi, rx_packet->snr);
    } else {
        printf("MeshNetwork: Ignored old/inferior DReq %lu from %u (TTL %u, current hop %u).\r\n",
               dreq->header.dreq_id, dreq->header.sender_id, dreq->ttl, current_discovery_cache.my_hop_count_to_dreq_sender);
    }
}

static void MeshNetwork_handle_drep(const LoraRadio_Packet_t *rx_packet) {
    // Dynamic size for DRep packet
    // Calculate expected size: header + num_neighbors * sizeof(MeshNeighborInfo_t) + crc
    // We need to be careful here because the packet might be truncated if num_neighbors is large.
    // For now, assume the packet is complete and num_neighbors is accurate.
    if (rx_packet->len < sizeof(MeshDiscoveryReplyPacket_t) - sizeof(MeshNeighborInfo_t)) {
        printf("MeshNetwork: DRep packet too short, ignoring.\r\n");
        return;
    }

    MeshDiscoveryReplyPacket_t *drep = (MeshDiscoveryReplyPacket_t*)rx_packet->data;

    // Verify CRC (excluding the CRC byte itself)
    uint8_t calculated_crc = CRC8_calculate(rx_packet->data, rx_packet->len - sizeof(uint8_t));
    if (calculated_crc != drep->crc) {
        printf("MeshNetwork: DRep from %u, CRC Mismatch! Calculated: 0x%02X, Received: 0x%02X\r\n",
               drep->header.sender_id, calculated_crc, drep->crc);
        return;
    }

    // Only process DRep for the current discovery round
    if (drep->header.dreq_id != current_discovery_cache.dreq_id ||
        drep->original_dreq_sender_id != current_discovery_cache.original_dreq_sender_id) {
        printf("MeshNetwork: Ignored DRep from %u for old/wrong DReq ID %lu.\r\n", drep->header.sender_id, drep->header.dreq_id);
        return;
    }

    // If this DRep is for the original DReq sender (i.e., we are the primary device)
    if (MESH_DEVICE_ID == current_discovery_cache.original_dreq_sender_id) {
        // Add all reported neighbors to the global table
        uint8_t num_entries = drep->num_neighbors;
        if (num_entries > MESH_MAX_NEIGHBORS_PER_PACKET) num_entries = MESH_MAX_NEIGHBORS_PER_PACKET; // Safety check

        for (uint8_t i = 0; i < num_entries; i++) {
            MeshNetwork_add_or_update_global_neighbor(drep->neighbors[i].device_id,
                                                      drep->neighbors[i].hop_count,
                                                      drep->neighbors[i].rssi,
                                                      drep->neighbors[i].snr);
        }
        printf("MeshNetwork: Primary Device %u: Received DRep from %u, added %u neighbors.\r\n",
               MESH_DEVICE_ID, drep->header.sender_id, drep->num_neighbors);
    }
    // If we are an intermediate relay node and this DRep is from a downstream child
    else if (drep->parent_id == MESH_DEVICE_ID) {
        printf("MeshNetwork: Received DRep from downstream %u, num_neighbors %u\r\n",
               drep->header.sender_id, drep->num_neighbors);
        // Add all reported neighbors to our local cache for potential relaying
        uint8_t num_entries = drep->num_neighbors;
        if (num_entries > MESH_MAX_NEIGHBORS_PER_PACKET) num_entries = MESH_MAX_NEIGHBORS_PER_PACKET; // Safety check

        for (uint8_t i = 0; i < num_entries; i++) {
            MeshNetwork_add_or_update_local_cache(drep->neighbors[i].device_id,
                                                  drep->neighbors[i].hop_count,
                                                  drep->neighbors[i].rssi,
                                                  drep->neighbors[i].snr);
        }
    } else {
        // DRep not for us or irrelevant for current discovery round
        printf("MeshNetwork: Ignored DRep from %u (parent %u, current parent %u).\r\n",
               drep->header.sender_id, drep->parent_id, current_discovery_cache.preferred_parent_id);
    }
}

static void MeshNetwork_send_dreq(uint32_t dreq_id, uint16_t sender_id, uint8_t ttl) {
    DiscoveryRequestPacket_t dreq_packet;
    dreq_packet.header.packet_type = MESH_PACKET_TYPE_DREQ;
    dreq_packet.header.sender_id = sender_id;
    dreq_packet.header.dreq_id = dreq_id;
    dreq_packet.ttl = ttl;

    LoraRadio_Packet_t tx_packet;
    memcpy(tx_packet.data, &dreq_packet, sizeof(DiscoveryRequestPacket_t));
    tx_packet.len = sizeof(DiscoveryRequestPacket_t);

    if (LoraRadio_send_packet(&tx_packet, pdMS_TO_TICKS(MESH_DREQ_FLOOD_DELAY_MS))) {
        printf("MeshNetwork: Sent DReq ID %lu from %u, TTL %u.\r\n", dreq_id, sender_id, ttl);
    } else {
        printf("MeshNetwork: Failed to send DReq ID %lu from %u.\r\n", dreq_id, sender_id);
    }
}

static void MeshNetwork_send_drep(void) {
    MeshDiscoveryReplyPacket_t *reply_packet = (MeshDiscoveryReplyPacket_t*)pvPortMalloc(sizeof(MeshDiscoveryReplyPacket_t) + (MESH_MAX_NEIGHBORS_PER_PACKET * sizeof(MeshNeighborInfo_t)));
    if (reply_packet == NULL) {
        printf("MeshNetwork: Failed to allocate memory for DRep packet.\r\n");
        return;
    }
    memset(reply_packet, 0, sizeof(MeshDiscoveryReplyPacket_t) + (MESH_MAX_NEIGHBORS_PER_PACKET * sizeof(MeshNeighborInfo_t)));

    reply_packet->header.packet_type = MESH_PACKET_TYPE_DREP;
    reply_packet->header.sender_id = MESH_DEVICE_ID;
    reply_packet->header.dreq_id = current_discovery_cache.dreq_id;
    reply_packet->original_dreq_sender_id = current_discovery_cache.original_dreq_sender_id;
    reply_packet->parent_id = current_discovery_cache.preferred_parent_id;

    uint8_t neighbors_added = 0;

    // 1. Add this device's own info first (if not already sent)
    if (!current_discovery_cache.my_info_sent) {
        for (uint8_t i = 0; i < current_discovery_cache.num_local_discovered_devices; i++) {
            if (current_discovery_cache.local_discovered_devices[i].device_id == MESH_DEVICE_ID) {
                memcpy(&reply_packet->neighbors[neighbors_added], &current_discovery_cache.local_discovered_devices[i], sizeof(MeshNeighborInfo_t));
                current_discovery_cache.local_discovered_devices[i].reported_upstream = true;
                neighbors_added++;
                current_discovery_cache.my_info_sent = true;
                break;
            }
        }
    }

    // 2. Add prioritized downstream neighbors
    // Simple prioritization: iterate and add up to MAX_NEIGHBORS_PER_PACKET
    // A more advanced prioritization would sort by hop_count, then RSSI, then filter out already reported ones.
    for (uint8_t i = 0; i < current_discovery_cache.num_local_discovered_devices && neighbors_added < MESH_MAX_NEIGHBORS_PER_PACKET; i++) {
        LocalNeighborEntry_t *entry = &current_discovery_cache.local_discovered_devices[i];
        if (entry->device_id != MESH_DEVICE_ID && !entry->reported_upstream) { // Don't re-add self, and only add if not already reported
            memcpy(&reply_packet->neighbors[neighbors_added], entry, sizeof(MeshNeighborInfo_t));
            entry->reported_upstream = true; // Mark as reported
            neighbors_added++;
        }
    }
    reply_packet->num_neighbors = neighbors_added;

    if (neighbors_added > 0) {
        // Calculate CRC
        uint16_t packet_base_len = sizeof(MeshDiscoveryReplyPacket_t) - sizeof(MeshNeighborInfo_t); // Header + parent_id + num_neighbors
        uint16_t total_payload_len = packet_base_len + (neighbors_added * sizeof(MeshNeighborInfo_t));
        reply_packet->crc = CRC8_calculate((uint8_t*)reply_packet, total_payload_len);

        LoraRadio_Packet_t tx_packet;
        memcpy(tx_packet.data, reply_packet, total_payload_len + sizeof(uint8_t)); // Include CRC byte
        tx_packet.len = total_payload_len + sizeof(uint8_t);

        if (LoraRadio_send_packet(&tx_packet, pdMS_TO_TICKS(LORA_TX_QUEUE_TIMEOUT_MS))) {
            printf("MeshNetwork: Sending DRep to parent %u with %u neighbors. Next scheduled reply at %lu\r\n",
                   reply_packet->parent_id, reply_packet->num_neighbors, current_discovery_cache.my_reply_scheduled_time + MESH_BASE_HOP_DELAY_MS);
        } else {
            printf("MeshNetwork: Failed to send DRep to parent %u.\r\n", reply_packet->parent_id);
        }
    }
    vPortFree(reply_packet); // Free the dynamically allocated memory

    // Reschedule next reply for this device's hop level
    current_discovery_cache.my_reply_scheduled_time += MESH_BASE_HOP_DELAY_MS;
}
