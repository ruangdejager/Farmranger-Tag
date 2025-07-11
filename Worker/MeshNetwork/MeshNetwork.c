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
#include <stdio.h>  // For DBG (for debugging)
#include <stdlib.h> // For rand()

#include "dbg_log.h"

uint32_t MESH_DEVICE_ID; // This is unique to the radio and can be retrieve from the device layer

// --- PRIVATE DEFINES ---
#define MESH_RX_PARSER_TASK_PRIORITY    (configMAX_PRIORITIES - 2)
#define MESH_RX_HANDLER_TASK_PRIORITY   (configMAX_PRIORITIES - 3)
#define MESH_REPLY_SCHED_TASK_PRIORITY  (configMAX_PRIORITIES - 4) // Lower priority for this task
#define MESH_TASK_STACK_SIZE            (configMINIMAL_STACK_SIZE * 6)

// Queue sizes
#define MESH_DREQ_QUEUE_SIZE            (5) // Max pending DReqs
#define MESH_DREP_QUEUE_SIZE            (10) // Max pending DReps
#define MESH_INIT_DREQ_QUEUE_SIZE       (1) // Queue for initiating DReq from app layer

// Event bits for Reply Scheduler
#define MESH_REPLY_SCHEDULE_BIT         (1UL << 0UL)

// --- INTERNAL DATA STRUCTURES ---

// Represents a discovered neighbor in the local cache of an intermediate node
typedef struct {
    uint32_t    u32DeviceID;
    uint8_t     u8HopCount; // Hops from Original_DReq_Sender_ID to this device
    int16_t     i16Rssi;      // RSSI of the DReq packet received by this device
    int8_t      u8Snr;       // SNR of the DReq packet received by this device
    TickType_t  tLastUpdated; // For aging out entries
    bool        bReportedUpstream; // Flag to track if this device's info has been sent upstream
} LocalNeighborEntry_t;

// Cache for the current discovery round at an intermediate node
typedef struct {
    uint32_t            u32DReqID;
    uint32_t            u32OGDreqSenderID;
    uint32_t            u32PreferredParentID; // The node this device will send its reply to
    uint8_t             u8MyHopCountToDReqSender;
    TickType_t          tDReqReceivedTime;
    LocalNeighborEntry_t tLocalDiscoveredDevices[MESH_MAX_LOCAL_DISCOVERED_DEVICES];
    uint8_t             u8LocalDiscoveredDevicesCount;
    bool                bInfoSent; // Flag to ensure this device's own info is sent once
    TickType_t          tScheduledReplyTime; // When this node should send its DRep
} DiscoveryCache_t;

static DiscoveryCache_t CurrentDiscoveryCache;
static uint32_t u32LastProcessedDReqID = 0; // To prevent processing old DReqs

// Discovered neighbors for the application layer
#define MESH_MAX_GLOBAL_NEIGHBORS (250) // Max entries in the primary device's global table
static MeshDiscoveredNeighbor_t tMeshDiscoveredNeighbors[MESH_MAX_GLOBAL_NEIGHBORS];
static uint8_t u8MeshDiscoveredNeighborsCount = 0;
static SemaphoreHandle_t xMeshNeighborTableMutex; // Protects mesh_discovered_neighbors

// --- INTERNAL FREE_RTOS QUEUES ---
typedef enum {
    MESH_EVENT_DREQ_RECEIVED = 1,
    MESH_EVENT_DREP_RECEIVED,
    MESH_EVENT_INITIATE_DREQ // For primary device to initiate
} MeshEventType_t;

// Queue for initiating DReq from application layer (send to reply scheduler task)
typedef struct {
    uint33_t DReqID;
    uint16_t OGDReqSenderID;
} MeshInitDReqEvent_t;

static QueueHandle_t xMeshNetworkInitDReqQueue;
static QueueHandle_t xMeshNetworkDReqQueue; // Queue for DReq from Mesh Device
static QueueHandle_t xMeshNetworkDRepQueue; // Queue for DReq from Mesh Device
QueueSetHandle_t xMeshNetworkEventQueueSet;

// Event group for the reply scheduler to signal it needs to check for replies
static EventGroupHandle_t xMeshReplySchedulerEventGroup;

// --- FREE_RTOS SOFTWARE TIMER ---
static TimerHandle_t xMeshReplySchedulerTimer;
#define MESH_REPLY_TIMER_ID (1) // Unique ID for the timer
#define MESH_REPLY_TIMER_BIT (1UL << 1UL) // New event bit for timer expiration

// --- PRIVATE FUNCTION PROTOTYPES ---
static uint8_t CRC8_calculate(const uint8_t *data, uint16_t len);
static void MeshNetwork_add_or_update_global_neighbor(uint16_t device_id, uint8_t hop_count, int16_t rssi, int8_t snr);
static void MeshNetwork_add_or_update_local_cache(uint16_t device_id, uint8_t hop_count, int16_t rssi, int8_t snr);
static TickType_t MeshNetwork_calculate_reply_delay(uint8_t my_hop_count_to_dreq_sender);
static void MeshNetwork_handle_dreq(const MeshDiscoveryRequestPacket_t *dreq_packet); // Changed param type
static void MeshNetwork_handle_drep(const MeshDiscoveryReplyPacket_t *drep_packet);   // Changed param type
static void MeshNetwork_send_dreq(uint32_t dreq_id, uint16_t sender_id, uint8_t ttl);
static void MeshNetwork_send_drep(void);

// --- PUBLIC FUNCTIONS ---

void MESHNETWORK_vInit(void) {

	xMeshNetworkDReqQueue = xQueueCreate(MESH_DREQ_QUEUE_SIZE, sizeof(MeshDReqPacket));
	xMeshNetworkDRepQueue = xQueueCreate(MESH_DREP_QUEUE_SIZE, sizeof(MeshDRepPacket));
	xMeshNetworkInitDReqQueue = xQueueCreate(MESH_INIT_DREQ_QUEUE_SIZE, sizeof(MeshInitDReqEvent_t));

    xMeshNetworkEventQueueSet = xQueueCreateSet(MESH_DREQ_QUEUE_SIZE + MESH_DREP_QUEUE_SIZE);
    configASSERT(xMeshNetworkEventQueueSet != NULL);
    xQueueAddToSet(xMeshNetworkDReqQueue, xQueueSet);
    xQueueAddToSet(xMeshNetworkDRepQueue, xQueueSet);

    xMeshNeighborTableMutex = xSemaphoreCreateMutex();
    xMeshReplySchedulerEventGroup = xEventGroupCreate();

    configASSERT(xMeshNetworkDReqQueue != NULL || xMeshNetworkDRepQueue != NULL ||
    		xMeshNeighborTableMutex != NULL || xMeshReplySchedulerEventGroup != NULL || xMeshNetworkInitDReqQueue != NULL);

    // Create the software timer for reply scheduling
	xMeshReplySchedulerTimer = xTimerCreate("MeshReplyTimer",
											pdMS_TO_TICKS(1), // Initial period, will be changed at runtime
											pdFALSE,          // One-shot timer
											(void *)MESH_REPLY_TIMER_ID,
											vMeshReplyTimerCallback);
	configASSERT(xMeshReplySchedulerTimer != NULL)

    memset(&CurrentDiscoveryCache, 0, sizeof(DiscoveryCache_t));
    u8MeshDiscoveredNeighborsCount = 0;

    BaseType_t status;
    status = xTaskCreate(MESHNETWORK_vMeshRxEventTask,
                "MeshRxEventTask",
                MESH_TASK_STACK_SIZE,
                NULL,
				MESH_RX_HANDLER_TASK_PRIORITY,
                NULL);
    configASSERT(status == pdPASS);

    status = xTaskCreate(MESHNETWORK_vParserTask,
                "MeshParserTask",
                MESH_TASK_STACK_SIZE,
                NULL,
				MESH_RX_PARSER_TASK_PRIORITY,
                NULL);
    configASSERT(status == pdPASS);

    status = xTaskCreate(MESHNETWORK_vReplySchedulerTask,
                "MeshReplyScheduler",
                MESH_TASK_STACK_SIZE,
                NULL,
				MESH_REPLY_SCHED_TASK_PRIORITY,
                NULL);
    configASSERT(status == pdPASS);

}

bool MESHNETWORK_bEncodeDReqMessage(MeshDReqPacket * pMeshDReqPacket, uint8_t * buffer, uint16_t buffer_length, uint16_t * message_length)
{

    pb_ostream_t PbOutputStream;

    // Setup output stream for PB encoding
    PbOutputStream = pb_ostream_from_buffer(buffer, buffer_length);

    buffer[0] = MeshPacketType_MESH_PACKET_TYPE_DREQ;

    // Setup output stream for PB encoding
    PbOutputStream = pb_ostream_from_buffer(&buffer[1], buffer_length-1);

	// Encode msg
	if (!pb_encode(&PbOutputStream, MeshDReqPacket_fields , pMeshDReqPacket))
	{
		return false;
	}

    *message_length = PbOutputStream.bytes_written + 1;

    return true;

}

bool MESHNETWORK_bEncodeDRepMessage(MeshDRepPacket * pMeshDRepPacket, uint8_t * buffer, uint16_t buffer_length, uint16_t * message_length)
{

    pb_ostream_t PbOutputStream;

    // Setup output stream for PB encoding
    PbOutputStream = pb_ostream_from_buffer(buffer, buffer_length);

    buffer[0] = MeshPacketType_MESH_PACKET_TYPE_DREP;

    // Setup output stream for PB encoding
    PbOutputStream = pb_ostream_from_buffer(&buffer[1], buffer_length-1);

	// Encode msg
	if (!pb_encode(&PbOutputStream, MeshDReqPacket_fields , pMeshDRepPacket))
	{
		return false;
	}

    *message_length = PbOutputStream.bytes_written + 1;

    return true;

}

void MESHNETWORK_vMeshRxEventTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {

    	QueueSetMemberHandle_t activeQueue = xQueueSelectFromSet(xMeshNetworkEventQueueSet, portMAX_DELAY);

        if (activeQueue == xMeshNetworkDReqQueue)
        {
        	MeshDReqPacket tMeshDReqPacket;
            xQueueReceive(xMeshNetworkDReqQueue, &tMeshDReqPacket, 0);  // Always 0 timeout after select
            MeshNetwork_handle_dreq(&tMeshDReqPacket);
            // Signal the reply scheduler to re-evaluate its state after processing a DReq
            xEventGroupSetBits(xMeshReplySchedulerEventGroup, MESH_REPLY_SCHEDULE_BIT);
        }
        else if (activeQueue == xMeshNetworkDRepQueue)
        {
        	MeshDRepPacket tMeshDRepPacket;
            xQueueReceive(myQueue2, &tMeshDRepPacket, 0);  // Always 0 timeout after select
            MeshNetwork_handle_drep(&tMeshDRepPacket);
        }

    }

    vTaskDelete(NULL);

}

void MESHNETWORK_vReplySchedulerTask(void *pvParameters) {
    (void)pvParameters;

    MeshInitDReqEvent_t init_event;
	EventBits_t uxBits;

	for (;;) {
		// Wait for specific event bits indefinitely (no timeout here for waiting)
		uxBits = xEventGroupWaitBits(xMeshReplySchedulerEventGroup,
									 MESH_REPLY_SCHEDULE_BIT | MESH_REPLY_TIMER_BIT, // Wait for either bit
									 pdTRUE, // Clear bits on exit
									 pdFALSE, // Don't wait for all bits, any is fine
									 portMAX_DELAY); // Block indefinitely until a bit is set

		// 1. Check for DReq initiation from app layer
		// This check should still be non-blocking as the EventGroup bit indicates new data.
		if ((uxBits & MESH_REPLY_SCHEDULE_BIT) != 0 || xQueueMessagesWaiting(xMeshNetworkInitDReqQueue) > 0) {
			if (xQueueReceive(xMeshNetworkInitDReqQueue, &init_event, 0) == pdPASS) { // Non-blocking read
				// This is the primary device initiating a DReq flood
				MeshNetwork_send_dreq(init_event.dreq_id, MESH_DEVICE_ID, MESH_MAX_TTL);
				DBG("MeshReplyScheduler: Initiated DReq flood from primary device.\r\n");
			}
		}

		// 2. Prioritized Relaying Logic (triggered by timer expiration)
		// Only run this logic if the timer has expired and indicated it's time to reply.
		// Also ensure current_discovery_cache is still valid for this specific DReq ID
		// and we haven't sent our info yet for this round.
		if ((uxBits & MESH_REPLY_TIMER_BIT) != 0) {
			if (current_discovery_cache.dreq_id != 0 && // Valid discovery round active
				LORARADIO_u32GetUniqueId() != current_discovery_cache.original_dreq_sender_id && // Not the original sender
				current_discovery_cache.preferred_parent_id != 0 && // Has a preferred parent
				!current_discovery_cache.my_info_sent) { // Only send our own reply once per round

				// Verify that the actual current time is greater than or equal to the scheduled time
				// (Small discrepancies can occur with timer granularity/task scheduling)
				if (xTaskGetTickCount() >= current_discovery_cache.my_reply_scheduled_time) {
					MeshNetwork_send_drep(); // This function will handle constructing and sending the DRep
				} else {
					DBG("MeshReplyScheduler: Timer fired early for some reason, re-scheduling. Current: %lu, Scheduled: %lu\r\n",
						   xTaskGetTickCount(), current_discovery_cache.my_reply_scheduled_time);
					// This could happen if a higher priority task pre-empts for a long time,
					// or if the tick count calculation for delay was slightly off due to context switch.
					// Reschedule the timer to fire at the correct time.
					TickType_t current_ticks = xTaskGetTickCount();
					TickType_t remaining_delay = 0;
					if (current_discovery_cache.my_reply_scheduled_time > current_ticks) {
						remaining_delay = current_discovery_cache.my_reply_scheduled_time - current_ticks;
					} else {
						remaining_delay = 1; // Trigger immediately if already passed
					}
					xTimerChangePeriod(xMeshReplySchedulerTimer, remaining_delay, 0);
					xTimerStart(xMeshReplySchedulerTimer, 0);
					// Do not clear MESH_REPLY_TIMER_BIT yet, let it be cleared when it actually fires at the right time.
					// Or, if pdTRUE is set in waitbits, it will be cleared, so we need to set it again.
					// For simplicity, `pdTRUE` is fine, just be aware of the "early" fire logic.
				}
			} else {
				 DBG("MeshReplyScheduler: Timer fired but current_discovery_cache not valid for reply.\r\n");
				 // This could happen if the discovery round was reset by a new DReq before the old timer fired.
				 // The timer was stopped by the new DReq, but if the signal somehow got through, this catches it.
			}
		}
		// Removed vTaskDelay(pdMS_TO_TICKS(10)); as the task blocks on EventGroup.
	}
}

void MESHNETWORK_vParserTask(void *pvParameters) {
    (void)pvParameters;

    LoraRadio_Packet_t rx_packet;

    MeshDReqPacket tMeshDReqPacket;
    MeshDRepPacket tMeshDRepPacket;

    for (;;)
    {

        // Monitor the mesh device rx queue for raw packets
    	if (LORARADIO_bRxPacket(&rx_packet))
		{

        	pb_istream_t PbInputStream;

            // Setup input stream for PB decoding
            PbInputStream = pb_istream_from_buffer(&rx_packet.buffer[1], rx_packet.length - 1);

			// Decode msg
			// First we see if it is a DREQ Message (Discovery request)
			if ( rx_packet.buffer[0] == MeshPacketType_MESH_PACKET_TYPE_DREQ )
			{
				// Clear struct for PB request msg
				memset(&tMeshDReqPacket, 0, sizeof(MeshDReqPacket));
				pb_decode(&PbInputStream, MeshDReqPacket_fields, &tMeshDReqPacket);
				DBG("Received DREQ\r\n");

				if (xQueueSend(xMeshNetworkDReqQueue, &tMeshDReqPacket, pdMS_TO_TICKS(100)) != pdPASS) {
					// Handle send failure (rare with portMAX_DELAY)
				}

			} else if ( rx_packet.buffer[0] == MeshPacketType_MESH_PACKET_TYPE_DREP ) {

				// Clear struct for PB request msg
				memset(&tMeshDRepPacket, 0, sizeof(MeshDRepPacket));
				// If not we check if it is a DREP Message (Discovery reply)
				pb_decode(&PbInputStream, MeshDRepPacket_fields, &tMeshDRepPacket);
				DBG("Received CDRep\r\n");

				if (xQueueSend(xMeshNetworkDRepQueue, &tMeshDRepPacket, pdMS_TO_TICKS(100)) != pdPASS) {
					// Handle send failure (rare with portMAX_DELAY)
				}

			} else if ( receivedParams.buffer[0] == MeshPacketType_MESH_PACKET_TYPE_TIMESYNC) {

			}
		}

    }

    vTaskDelete(NULL);
}

// NEW TASK
//void vMeshRxParserTask(void *pvParameters) {
//    (void)pvParameters;
//
//    LoraRadio_Packet_t rx_packet;
//
//    for (;;) {
//        // 1. Check for incoming raw LoRa packets from the LoraRadio layer
//        if (LoraRadio_receive_packet(&rx_packet, portMAX_DELAY)) { // Wait indefinitely for a raw packet
//            MeshPacketHeader_t *header = (MeshPacketHeader_t*)rx_packet.data;
//
//            if (rx_packet.len < sizeof(MeshPacketHeader_t)) {
//                DBG("MeshRxParser: Packet too short for header, ignoring.\r\n");
//                continue;
//            }
//
//            if (header->packet_type == MESH_PACKET_TYPE_DREQ) {
//                if (rx_packet.len < sizeof(MeshDiscoveryRequestPacket_t)) {
//                    DBG("MeshRxParser: DReq packet too short, ignoring.\r\n");
//                    continue;
//                }
//                MeshDiscoveryRequestPacket_t parsed_dreq;
//                memcpy(&parsed_dreq, rx_packet.data, sizeof(MeshDiscoveryRequestPacket_t));
//                parsed_dreq.rssi = rx_packet.rssi; // Pass RSSI/SNR from physical layer
//                parsed_dreq.snr = rx_packet.snr;
//
//                if (xQueueSend(xMeshNetworkDReqQueue, &parsed_dreq, 0) != pdPASS) { // Non-blocking send
//                    DBG("MeshRxParser: DReq queue full, DReq from %u dropped.\r\n", parsed_dreq.header.sender_id);
//                } else {
//                    DBG("MeshRxParser: Parsed DReq from %u, queued.\r\n", parsed_dreq.header.sender_id);
//                }
//            } else if (header->packet_type == MESH_PACKET_TYPE_DREP) {
//                // Determine the full size of the DRep packet based on num_neighbors
//                if (rx_packet.len < (sizeof(MeshDiscoveryReplyPacket_t) - sizeof(MeshNeighborInfo_t) + sizeof(uint8_t))) { // Min DRep size (header + parent_id + num_neighbors + CRC)
//                     DBG("MeshRxParser: DRep packet too short for header/min payload, ignoring.\r\n");
//                     continue;
//                }
//                uint8_t num_neighbors = ((MeshDiscoveryReplyPacket_t*)rx_packet.data)->num_neighbors;
//                uint16_t expected_total_len = (sizeof(MeshDiscoveryReplyPacket_t) - sizeof(MeshNeighborInfo_t)) +
//                                              (num_neighbors * sizeof(MeshNeighborInfo_t)) + sizeof(uint8_t); // +CRC
//
//                if (rx_packet.len < expected_total_len) {
//                    DBG("MeshRxParser: DRep packet truncated, ignoring. Expected %u, got %u.\r\n", expected_total_len, rx_packet.len);
//                    continue;
//                }
//
//                // Allocate memory dynamically for the variable-sized DRep
//                MeshDiscoveryReplyPacket_t *parsed_drep = (MeshDiscoveryReplyPacket_t*)pvPortMalloc(expected_total_len);
//                if (parsed_drep == NULL) {
//                    DBG("MeshRxParser: Failed to allocate memory for DRep, dropping.\r\n");
//                    continue;
//                }
//                memcpy(parsed_drep, rx_packet.data, expected_total_len); // Copy the full packet
//
//                if (xQueueSend(xMeshNetworkDRepQueue, &parsed_drep, 0) != pdPASS) { // Non-blocking send of pointer
//                    DBG("MeshRxParser: DRep queue full, DRep from %u dropped. Freeing memory.\r\n", parsed_drep->header.sender_id);
//                    vPortFree(parsed_drep); // Free if not successfully queued
//                } else {
//                    DBG("MeshRxParser: Parsed DRep from %u, queued pointer.\r\n", parsed_drep->header.sender_id);
//                }
//            } else {
//                DBG("MeshRxParser: Unknown packet type %u received from LoRaRadio.\r\n", header->packet_type);
//            }
//        }
//    }
//}

// NEW: Software timer callback function
void MESHNETWORK_vReplyTimerCallback(TimerHandle_t xTimer) {
    (void)xTimer; // Parameter not used, but required by FreeRTOS API
    // Signal the vMeshReplySchedulerTask that it's time to send a DRep
    xEventGroupSetBits(xMeshReplySchedulerEventGroup, MESH_REPLY_TIMER_BIT);
    DBG("MeshReplyTimer: Timer expired, signaling scheduler.\r\n");
}

// NEW FUNCTION
bool MeshNetwork_start_discovery_round(uint32_t dreq_id, uint16_t original_dreq_sender_id) {
    MeshInitDReqEvent_t event;
    event.dreq_id = dreq_id;
    event.original_dreq_sender_id = original_dreq_sender_id;

    // Set up the cache for the primary device's own DReq
    // This part is critical as it sets up the initial state for the reply scheduler.
    memset(&current_discovery_cache, 0, sizeof(DiscoveryCache_t));
    current_discovery_cache.dreq_id = dreq_id;
    current_discovery_cache.original_dreq_sender_id = original_dreq_sender_id;
    current_discovery_cache.my_hop_count_to_dreq_sender = 0; // 0 hops for the initiator
    current_discovery_cache.dreq_received_time = xTaskGetTickCount(); // Use current time as reference
    current_discovery_cache.my_reply_scheduled_time = 0; // Not applicable for initiator's own DRep sending logic immediately
    current_discovery_cache.preferred_parent_id = 0; // No parent for initiator

    // Add initiator's own info to its local cache (which is also its global table)
    MeshNetwork_add_or_update_local_cache(MESH_DEVICE_ID, 0, 0, 0); // RSSI/SNR 0 for self

    DBG("MeshNetwork: Initiating DReq with ID %lu from %u\r\n", dreq_id, original_dreq_sender_id);
    bool queued = xQueueSend(xMeshNetworkInitDReqQueue, &event, portMAX_DELAY) == pdPASS;
    if (queued) {
        // Signal the reply scheduler task to process the initiation
        xEventGroupSetBits(xMeshReplySchedulerEventGroup, MESH_REPLY_SCHEDULE_BIT);
        // Stop the timer if it's running, as the initiator doesn't schedule a timed DRep.
		// It might be running if a previous DReq was received.
		xTimerStop(xMeshReplySchedulerTimer, 0);
    }
    return queued;
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

static void MeshNetwork_handle_dreq(const MeshDiscoveryRequestPacket_t *dreq_packet) {
    // Check if this is a new DReq round or a better path for current round
    // Only process if it's a newer DReq ID, or same ID but better (lower) hop count
    // Calculate received hop count using TTL
    uint8_t received_hop_count = MESH_MAX_TTL - dreq_packet->ttl + 1; // +1 because TTL is decremented first

    if (dreq_packet->header.dreq_id > last_processed_dreq_id ||
        (dreq_packet->header.dreq_id == last_processed_dreq_id && received_hop_count < current_discovery_cache.my_hop_count_to_dreq_sender)) {

        // If new DReq round, reset cache and update last_processed_dreq_id
        if (dreq_packet->header.dreq_id > last_processed_dreq_id) {
            memset(&current_discovery_cache, 0, sizeof(DiscoveryCache_t));
            current_discovery_cache.dreq_id = dreq_packet->header.dreq_id;
            // MODIFIED: Use original_dreq_sender_id from DReq packet
            current_discovery_cache.original_dreq_sender_id = dreq_packet->original_dreq_sender_id;
            current_discovery_cache.dreq_received_time = xTaskGetTickCount();
            last_processed_dreq_id = dreq_packet->header.dreq_id;
            DBG("MeshNetwork: New DReq round %lu started.\r\n", dreq_packet->header.dreq_id);
            // Stop the timer immediately if a new DReq round begins, to cancel any pending reply
            xTimerStop(xMeshReplySchedulerTimer, 0);
        }

        // Update my hop count and preferred parent if this path is better
        if (current_discovery_cache.my_hop_count_to_dreq_sender == 0 || received_hop_count < current_discovery_cache.my_hop_count_to_dreq_sender) {
            current_discovery_cache.my_hop_count_to_dreq_sender = received_hop_count;
            current_discovery_cache.preferred_parent_id = dreq_packet->header.sender_id; // The node that sent this DReq
            current_discovery_cache.my_reply_scheduled_time = current_discovery_cache.dreq_received_time +
                                                              MeshNetwork_calculate_reply_delay(current_discovery_cache.my_hop_count_to_dreq_sender);

            DBG("MeshNetwork: DReq from %u (hop %u, RSSI %d, SNR %d), new preferred parent. Scheduled reply at %lu\r\n",
                   dreq_packet->header.sender_id, current_discovery_cache.my_hop_count_to_dreq_sender,
                   dreq_packet->rssi, dreq_packet->snr, current_discovery_cache.my_reply_scheduled_time);

            // Start/Reset the software timer to trigger at my_reply_scheduled_time
            // Calculate delay from current tick count to scheduled time.
            TickType_t current_ticks = xTaskGetTickCount();
            TickType_t delay_ticks = 0;
            if (current_discovery_cache.my_reply_scheduled_time > current_ticks) {
                delay_ticks = current_discovery_cache.my_reply_scheduled_time - current_ticks;
            } else {
                // Should not happen for newly scheduled replies, but for safety, if already past due, trigger immediately.
                delay_ticks = 1; // Trigger as soon as possible
            }
            xTimerStop(xMeshReplySchedulerTimer, 0); // Stop if already running
            xTimerChangePeriod(xMeshReplySchedulerTimer, delay_ticks, 0); // Set new period (delay)
            xTimerStart(xMeshReplySchedulerTimer, 0); // Start the timer

            // If this device is the original sender, it doesn't re-broadcast
            if (MESH_DEVICE_ID != dreq_packet->header.sender_id && dreq_packet->ttl > 0) {
                // Re-broadcast DReq with decremented TTL
                MeshNetwork_send_dreq(dreq_packet->header.dreq_id, MESH_DEVICE_ID, dreq_packet->ttl - 1);
            }
        }

        // Add this device's own info to its local cache if not already there
        MeshNetwork_add_or_update_local_cache(MESH_DEVICE_ID,
                                              current_discovery_cache.my_hop_count_to_dreq_sender,
                                              dreq_packet->rssi, dreq_packet->snr); // Use packet's RSSI/SNR
    } else {
        DBG("MeshNetwork: Ignored old/inferior DReq %lu from %u (TTL %u, current hop %u).\r\n",
               dreq_packet->header.dreq_id, dreq_packet->header.sender_id, dreq_packet->ttl, current_discovery_cache.my_hop_count_to_dreq_sender);
    }
}

// MODIFIED: Parameter type changed to const MeshDiscoveryReplyPacket_t *
static void MeshNetwork_handle_drep(const MeshDiscoveryReplyPacket_t *drep_packet) {
    // Verify CRC (excluding the CRC byte itself)
    uint16_t packet_base_len = sizeof(MeshDiscoveryReplyPacket_t) - sizeof(MeshNeighborInfo_t);
    uint16_t total_payload_len = packet_base_len + (drep_packet->num_neighbors * sizeof(MeshNeighborInfo_t));

    uint8_t calculated_crc = CRC8_calculate((uint8_t*)drep_packet, total_payload_len);
    if (calculated_crc != drep_packet->crc) {
        DBG("MeshNetwork: DRep from %u, CRC Mismatch! Calculated: 0x%02X, Received: 0x%02X\r\n",
               drep_packet->header.sender_id, calculated_crc, drep_packet->crc);
        return;
    }

    // Only process DRep for the current discovery round
    if (drep_packet->header.dreq_id != current_discovery_cache.dreq_id ||
        drep_packet->original_dreq_sender_id != current_discovery_cache.original_dreq_sender_id) { // Ensure original sender matches
        DBG("MeshNetwork: Ignored DRep from %u for old/wrong DReq ID %lu (Original DReq Sender: %u).\r\n",
               drep_packet->header.sender_id, drep_packet->header.dreq_id, drep_packet->original_dreq_sender_id);
        return;
    }

    // If this DRep is for the original DReq sender (i.e., we are the primary device)
    if (MESH_DEVICE_ID == current_discovery_cache.original_dreq_sender_id) {
        // Add all reported neighbors to the global table
        uint8_t num_entries = drep_packet->num_neighbors;
        if (num_entries > MESH_MAX_NEIGHBORS_PER_PACKET) num_entries = MESH_MAX_NEIGHBORS_PER_PACKET; // Safety check

        for (uint8_t i = 0; i < num_entries; i++) {
            MeshNetwork_add_or_update_global_neighbor(drep_packet->neighbors[i].device_id,
                                                      drep_packet->neighbors[i].hop_count,
                                                      drep_packet->neighbors[i].rssi,
                                                      drep_packet->neighbors[i].snr);
        }
        DBG("MeshNetwork: Primary Device %u: Received DRep from %u, added %u neighbors.\r\n",
               MESH_DEVICE_ID, drep_packet->header.sender_id, drep_packet->num_neighbors);
    }
    // If we are an intermediate relay node and this DRep is from a downstream child
    else if (drep_packet->parent_id == MESH_DEVICE_ID) {
        DBG("MeshNetwork: Received DRep from downstream %u, num_neighbors %u\r\n",
               drep_packet->header.sender_id, drep_packet->num_neighbors);
        // Add all reported neighbors to our local cache for potential relaying
        uint8_t num_entries = drep_packet->num_neighbors;
        if (num_entries > MESH_MAX_NEIGHBORS_PER_PACKET) num_entries = MESH_MAX_NEIGHBORS_PER_PACKET; // Safety check

        for (uint8_t i = 0; i < num_entries; i++) {
            MeshNetwork_add_or_update_local_cache(drep_packet->neighbors[i].device_id,
                                                  drep_packet->neighbors[i].hop_count,
                                                  drep_packet->neighbors[i].rssi,
                                                  drep_packet->neighbors[i].snr);
        }
        // NEW: Signal the reply scheduler to potentially send an updated DRep upstream
        xEventGroupSetBits(xMeshReplySchedulerEventGroup, MESH_REPLY_SCHEDULE_BIT);
    } else {
        // DRep not for us or irrelevant for current discovery round
        DBG("MeshNetwork: Ignored DRep from %u (parent %u, current parent %u).\r\n",
               drep_packet->header.sender_id, drep_packet->parent_id, current_discovery_cache.preferred_parent_id);
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
        DBG("MeshNetwork: Sent DReq ID %lu from %u, TTL %u.\r\n", dreq_id, sender_id, ttl);
    } else {
        DBG("MeshNetwork: Failed to send DReq ID %lu from %u.\r\n", dreq_id, sender_id);
    }
}

static void MeshNetwork_send_drep(void) {
    // Dynamically allocate memory for the flexible array member
    MeshDiscoveryReplyPacket_t *reply_packet = (MeshDiscoveryReplyPacket_t*)pvPortMalloc(sizeof(MeshDiscoveryReplyPacket_t) + (MESH_MAX_NEIGHBORS_PER_PACKET * sizeof(MeshNeighborInfo_t)));
    if (reply_packet == NULL) {
        DBG("MeshNetwork: Failed to allocate memory for DRep packet.\r\n");
        return;
    }
    // Clear the allocated memory
    memset(reply_packet, 0, sizeof(MeshDiscoveryReplyPacket_t) + (MESH_MAX_NEIGHBORS_PER_PACKET * sizeof(MeshNeighborInfo_t)));

    reply_packet->header.packet_type = MESH_PACKET_TYPE_DREP;
    reply_packet->header.sender_id = MESH_DEVICE_ID;
    reply_packet->header.dreq_id = current_discovery_cache.dreq_id;
    reply_packet->original_dreq_sender_id = current_discovery_cache.original_dreq_sender_id; // Added this field
    reply_packet->parent_id = current_discovery_cache.preferred_parent_id;

    uint8_t neighbors_added = 0;

    // 1. Add this device's own info first (if not already sent)
    // The `my_info_sent` flag in current_discovery_cache ensures this DRep is sent only once per round
    // at the scheduled time. This DRep will contain the device's own info.
    // Check `my_info_sent` flag
    if (!current_discovery_cache.my_info_sent) {
        for (uint8_t i = 0; i < current_discovery_cache.num_local_discovered_devices; i++) {
            if (current_discovery_cache.local_discovered_devices[i].device_id == MESH_DEVICE_ID) {
                memcpy(&reply_packet->neighbors[neighbors_added], &current_discovery_cache.local_discovered_devices[i], sizeof(MeshNeighborInfo_t));
                current_discovery_cache.local_discovered_devices[i].reported_upstream = true; // Mark as reported
                neighbors_added++;
                current_discovery_cache.my_info_sent = true; // Mark as sent for this round
                break;
            }
        }
    }

    // 2. Add prioritized downstream neighbors
    // Simple prioritization: iterate and add up to MAX_NEIGHBORS_PER_PACKET
    // A more advanced prioritization would sort by hop_count, then RSSI, then filter out already reported ones.
    // Don't re-add self, and only add if not already reported
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
        uint16_t total_packet_len = packet_base_len + (neighbors_added * sizeof(MeshNeighborInfo_t));
        reply_packet->crc = CRC8_calculate((uint8_t*)reply_packet, total_packet_len); // CRC covers up to num_neighbors * data

        LoraRadio_Packet_t tx_packet;
        memcpy(tx_packet.data, reply_packet, total_packet_len + sizeof(uint8_t)); // Include CRC byte
        tx_packet.len = total_packet_len + sizeof(uint8_t);

        if (LoraRadio_send_packet(&tx_packet, pdMS_TO_TICKS(LORA_TX_QUEUE_TIMEOUT_MS))) { // Assuming LORA_TX_QUEUE_TIMEOUT_MS is defined somewhere.
            DBG("MeshNetwork: Sending DRep to parent %u with %u neighbors. Current time: %lu. Next scheduled reply (if any): %lu\r\n",
                   reply_packet->parent_id, reply_packet->num_neighbors, xTaskGetTickCount(), current_discovery_cache.my_reply_scheduled_time);
            // Stop the timer after successfully sending the DRep
            xTimerStop(xMeshReplySchedulerTimer, 0);
        } else {
            DBG("MeshNetwork: Failed to queue DRep to parent %u for TX.\r\n", reply_packet->parent_id);
        }
    }
    // Free the dynamically allocated memory
    vPortFree(reply_packet);
}
