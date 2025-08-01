/*
 * MeshNetwork.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */


#include "MeshNetwork.h"
#include "MeshNetwork_Port.h"
#include "LoraRadio.h" // Interface to the LoRa radio driver
#include <string.h> // For memcpy, memset
#include <stdio.h>  // For DBG (for debugging)
#include <stdlib.h> // For rand()

#include "pb_encode.h"
#include "pb_decode.h"
#include "pb_common.h"
#include "pb.h"
#include "stdlib.h"

#include "dbg_log.h"

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
static uint32_t u32LastProcessedTimeSyncID = 0; // To prevent processing old TS messages

// Discovered neighbors for the application layer
#define MESH_MAX_GLOBAL_NEIGHBORS (250) // Max entries in the primary device's global table
static MeshDiscoveredNeighbor_t tMeshDiscoveredNeighbors[MESH_MAX_GLOBAL_NEIGHBORS];
static uint8_t u8MeshDiscoveredNeighborsCount = 0;
static SemaphoreHandle_t xMeshNeighborTableMutex; // Protects mesh_discovered_neighbors

// --- STATIC VARIABLES FOR WAKEUP INTERVAL ---
static WakeupInterval tCurrentWakeupInterval = WAKEUP_INTERVAL_15_MIN; // Default to 60 minutes
// Array to map enum to actual millisecond values
static const uint8_t u8CurrentWakeupIntervalMin[] = {
    [WAKEUP_INTERVAL_15_MIN]  = 15, // 15 minutes
    [WAKEUP_INTERVAL_30_MIN]  = 30, // 30 minutes
    [WAKEUP_INTERVAL_60_MIN]  = 60, // 60 minutes
    [WAKEUP_INTERVAL_120_MIN] = 120 // 120 minutes
};

uint8_t MESHNETWORK_u8GetWakeupInterval(void) {
    if (tCurrentWakeupInterval < WAKEUP_INTERVAL_MAX_COUNT) {
        return u8CurrentWakeupIntervalMin[tCurrentWakeupInterval];
    }
    return u8CurrentWakeupIntervalMin[WAKEUP_INTERVAL_60_MIN]; // Default or error value
}

WakeupInterval MESHNETWORK_tGetCurrentWakeupIntervalEnum(void) {
    return tCurrentWakeupInterval;
}

void MESHNETWORK_vSetWakeupInterval(WakeupInterval new_interval) {
    if (new_interval < WAKEUP_INTERVAL_MAX_COUNT) {
        tCurrentWakeupInterval = new_interval;
        DBG("MeshNetwork: Wakeup interval set to %u minutes (enum %u).\r\n",
               MESHNETWORK_u8GetWakeupInterval(), tCurrentWakeupInterval);
    } else {
        DBG("MeshNetwork: Attempted to set invalid wakeup interval enum: %u.\r\n", (unsigned int)new_interval);
    }
}

// --- INTERNAL FREE_RTOS QUEUES ---
typedef enum {
    MESH_EVENT_DREQ_RECEIVED = 1,
    MESH_EVENT_DREP_RECEIVED,
    MESH_EVENT_INITIATE_DREQ // For primary device to initiate
} MeshEventType_t;

// Queue for initiating DReq from application layer (send to reply scheduler task)
typedef struct {
    uint32_t DReqID;
    uint32_t OGDReqSenderID;
} MeshInitDReqEvent_t;

static QueueHandle_t xMeshNetworkInitDReqQueue;

// Event group for the reply scheduler to signal it needs to check for replies
static EventGroupHandle_t xMeshReplySchedulerEventGroup;

// --- FREE_RTOS SOFTWARE TIMER ---
static TimerHandle_t xMeshReplySchedulerTimer;
#define MESH_REPLY_TIMER_ID (1) // Unique ID for the timer
#define MESH_REPLY_TIMER_BIT (1UL << 1UL) // New event bit for timer expiration

// --- PRIVATE FUNCTION PROTOTYPES ---
static void MESHNETWORK_vAddOrUpdateGlobalNeighbor(uint32_t device_id, uint8_t hop_count, int16_t rssi, int8_t snr);
static void MESHNETWORK_vAddOrUpdateLocalCache(uint32_t device_id, uint8_t hop_count, int16_t rssi, int8_t snr);
static TickType_t MESHNETWORK_tCalculateReplyDelay(uint8_t my_hop_count_to_dreq_sender);
static void MESHNETWORK_vHandleDReq(const MeshDReqPacket *dreq_packet);
static void MESHNETWORK_vHandleDRep(const MeshDRepPacket *drep_packet);
static void MESHNETWORK_vSendDReq(uint32_t dreq_id, uint32_t sender_id, uint8_t ttl);
static void MESHNETWORK_vSendDRep(void);


// --- PUBLIC FUNCTIONS ---

void MESHNETWORK_vInit(void) {

	xMeshNetworkInitDReqQueue = xQueueCreate(MESH_INIT_DREQ_QUEUE_SIZE, sizeof(MeshInitDReqEvent_t));

    xMeshNeighborTableMutex = xSemaphoreCreateMutex();
    xMeshReplySchedulerEventGroup = xEventGroupCreate();

    // Create the software timer for reply scheduling
	xMeshReplySchedulerTimer = xTimerCreate("MeshReplyTimer",
											pdMS_TO_TICKS(1), // Initial period, will be changed at runtime
											pdFALSE,          // One-shot timer
											(void *)MESH_REPLY_TIMER_ID,
											MESHNETWORK_vReplyTimerCallback);
	configASSERT(xMeshReplySchedulerTimer != NULL)

    memset(&CurrentDiscoveryCache, 0, sizeof(DiscoveryCache_t));
    u8MeshDiscoveredNeighborsCount = 0;

    BaseType_t status;
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

bool MESHNETWORK_bEncodeDReqMessage(MeshDReqPacket * pMeshDReqPacket, uint8_t * buffer, uint16_t buffer_length, uint8_t * message_length)
{

    pb_ostream_t PbOutputStream;

    // Setup output stream for PB encoding
    PbOutputStream = pb_ostream_from_buffer(buffer, buffer_length);

    buffer[0] = MeshPacketType_MESH_PACKET_TYPE_DREQ;

    // Setup output stream for PB encoding
    PbOutputStream = pb_ostream_from_buffer(&buffer[1], buffer_length-1);

	// Encode msg
	if (!pb_encode(&PbOutputStream, MeshDReqPacket_fields, pMeshDReqPacket))
	{
		DBG("PB_ENCODE_ERROR_DREQ: %s\r\n", PB_GET_ERROR(&ostream));
		return false;
	}

    *message_length = PbOutputStream.bytes_written + 1;

    return true;

}

bool MESHNETWORK_bEncodeDRepMessage(MeshDRepPacket * pMeshDRepPacket, uint8_t * buffer, uint16_t buffer_length, uint8_t * message_length)
{

    pb_ostream_t PbOutputStream;

    // Setup output stream for PB encoding
    PbOutputStream = pb_ostream_from_buffer(buffer, buffer_length);

    buffer[0] = MeshPacketType_MESH_PACKET_TYPE_DREP;

    // Setup output stream for PB encoding
    PbOutputStream = pb_ostream_from_buffer(&buffer[1], buffer_length-1);

	// Encode msg
	if (!pb_encode(&PbOutputStream, MeshDRepPacket_fields , pMeshDRepPacket))
	{
		DBG("PB_ENCODE_ERROR_TIMESYNC: %s\r\n", PB_GET_ERROR(&PbOutputStream));
		return false;
	}

    *message_length = PbOutputStream.bytes_written + 1;

    return true;

}

bool MESHNETWORK_bEncodeTimesyncMessage(TimeSyncMessage * pTimeSyncMessage, uint8_t * buffer, uint16_t buffer_length, uint8_t * message_length)
{
    pb_ostream_t PbOutputStream;

    // Setup output stream for PB encoding
    PbOutputStream = pb_ostream_from_buffer(buffer, buffer_length);

    buffer[0] = MeshPacketType_MESH_PACKET_TYPE_TIMESYNC;

    // Setup output stream for PB encoding (offset by 1 byte for type)
    PbOutputStream = pb_ostream_from_buffer(&buffer[1], buffer_length - 1);

    // Encode msg
    if (!pb_encode(&PbOutputStream, TimeSyncMessage_fields, pTimeSyncMessage))
    {
        DBG("PB_ENCODE_ERROR_TIMESYNC: %s\r\n", PB_GET_ERROR(&PbOutputStream));
        return false;
    }

    *message_length = PbOutputStream.bytes_written + 1; // Add 1 for the type byte

    return true;
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
		if ((uxBits & MESH_REPLY_SCHEDULE_BIT) != 0 || uxQueueMessagesWaiting(xMeshNetworkInitDReqQueue) > 0) {
			if (xQueueReceive(xMeshNetworkInitDReqQueue, &init_event, 0) == pdPASS) { // Non-blocking read
				// This is the primary device initiating a DReq flood
				MESHNETWORK_vSendDReq(init_event.DReqID, MESHNETWORK_u32GetUniqueId(), MESH_MAX_TTL);
				DBG("MeshReplyScheduler: Initiated DReq flood from primary device.\r\n");
			}
		}

		// 2. Prioritized Relaying Logic (triggered by timer expiration)
		// Only run this logic if the timer has expired and indicated it's time to reply.
		// Also ensure CurrentDiscoveryCache is still valid for this specific DReq ID
		// and we haven't sent our info yet for this round.
		if ((uxBits & MESH_REPLY_TIMER_BIT) != 0) {
			if (CurrentDiscoveryCache.u32DReqID != 0 && // Valid discovery round active
				MESHNETWORK_u32GetUniqueId() != CurrentDiscoveryCache.u32OGDreqSenderID && // Not the original sender
				CurrentDiscoveryCache.u32PreferredParentID != 0 && // Has a preferred parent
				!CurrentDiscoveryCache.bInfoSent) { // Only send our own reply once per round

				// Verify that the actual current time is greater than or equal to the scheduled time
				// (Small discrepancies can occur with timer granularity/task scheduling)
				if (xTaskGetTickCount() >= CurrentDiscoveryCache.tScheduledReplyTime) {
					MESHNETWORK_vSendDRep(); // This function will handle constructing and sending the DRep
				} else {
					DBG("MeshReplyScheduler: Timer fired early for some reason, re-scheduling. Current: %lu, Scheduled: %lu\r\n",
						   xTaskGetTickCount(), CurrentDiscoveryCache.tScheduledReplyTime);
					// This could happen if a higher priority task pre-empts for a long time,
					// or if the tick count calculation for delay was slightly off due to context switch.
					// Reschedule the timer to fire at the correct time.
					TickType_t current_ticks = xTaskGetTickCount();
					TickType_t remaining_delay = 0;
					if (CurrentDiscoveryCache.tScheduledReplyTime > current_ticks) {
						remaining_delay = CurrentDiscoveryCache.tScheduledReplyTime - current_ticks;
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
				 DBG("MeshReplyScheduler: Timer fired but CurrentDiscoveryCache not valid for reply.\r\n");
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
    TimeSyncMessage tMeshTSPacket;

    for (;;)
    {

        // Monitor the mesh device rx queue for raw packets
    	if (MESHNETWORK_bReceiveMessage(&rx_packet))
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
				if(!pb_decode(&PbInputStream, MeshDReqPacket_fields, &tMeshDReqPacket))
				{
					DBG("--- PROTOBUF: PROBLEM DECODING DREQ ---\r\n");
				}
				tMeshDReqPacket.Rssi = rx_packet.rssi;
				tMeshDReqPacket.Snr = rx_packet.snr;

	            // Check if this DReq was originally sent by THIS device.
	            // If the Original DReq Sender ID in the packet matches this device's unique ID,
	            // then it's our own DReq being relayed back.
	            if (tMeshDReqPacket.OGDreqSenderID == MESHNETWORK_u32GetUniqueId()) {
	                DBG("MeshNetwork: Received own DReq (ID: %lu, Sender: %u), ignoring.\r\n",
	                        tMeshDReqPacket.Header.dReqID, tMeshDReqPacket.OGDreqSenderID);
	                // No further processing for this DReq if it's our own
	            } else {
	                // If it's NOT our own DReq, then handle it normally:
	                // 1. Evaluate if it's a new DReq round or offers a better path.
	                // 2. Schedule a DRep reply based on hop count.
	                // 3. Potentially re-broadcast the DReq.
	                MESHNETWORK_vHandleDReq(&tMeshDReqPacket);
	                // Signal the reply scheduler to re-evaluate its state after processing a DReq
	                xEventGroupSetBits(xMeshReplySchedulerEventGroup, MESH_REPLY_SCHEDULE_BIT);
	            }

			} else if ( rx_packet.buffer[0] == MeshPacketType_MESH_PACKET_TYPE_DREP ) {

				// Clear struct for PB request msg
				memset(&tMeshDRepPacket, 0, sizeof(MeshDRepPacket));
				// If not we check if it is a DREP Message (Discovery reply)
				if(!pb_decode(&PbInputStream, MeshDRepPacket_fields, &tMeshDRepPacket))
				{
					DBG("--- PROTOBUF: PROBLEM DECODING DREP ---\r\n");
				}

				MESHNETWORK_vHandleDRep(&tMeshDRepPacket);

			} else if ( rx_packet.buffer[0] == MeshPacketType_MESH_PACKET_TYPE_TIMESYNC) {

				// Clear struct for PB request msg
				memset(&tMeshTSPacket, 0, sizeof(TimeSyncMessage));
				// If not we check if it is a DREP Message (Discovery reply)
				if(!pb_decode(&PbInputStream, TimeSyncMessage_fields, &tMeshTSPacket))
				{
					DBG("--- PROTOBUF: PROBLEM DECODING TS ---\r\n");
				}

				MESHNETWORK_vHandleTimeSyncMessage(&tMeshTSPacket);

			}
		}
    }

    vTaskDelete(NULL);
}


void MESHNETWORK_vReplyTimerCallback(TimerHandle_t xTimer) {
    (void)xTimer; // Parameter not used, but required by FreeRTOS API
    // Signal the vMeshReplySchedulerTask that it's time to send a DRep
    xEventGroupSetBits(xMeshReplySchedulerEventGroup, MESH_REPLY_TIMER_BIT);
    DBG("MeshReplyTimer: Timer expired, signaling scheduler.\r\n");
}

bool MESHNETWORK_bStartDiscoveryRound(uint32_t dreq_id, uint32_t original_dreq_sender_id) {
    MeshInitDReqEvent_t event;
    event.DReqID = dreq_id;
    event.OGDReqSenderID = original_dreq_sender_id;

    DBG("\r\n--------\r\nCount: %d\r\n--------\r\n", CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount);
    // Set up the cache for the primary device's own DReq
    // This part is critical as it sets up the initial state for the reply scheduler.
    memset(&CurrentDiscoveryCache, 0, sizeof(DiscoveryCache_t));
    CurrentDiscoveryCache.u32DReqID = dreq_id;
    CurrentDiscoveryCache.u32OGDreqSenderID = original_dreq_sender_id;
    CurrentDiscoveryCache.u8MyHopCountToDReqSender = 0; // 0 hops for the initiator
    CurrentDiscoveryCache.tDReqReceivedTime = xTaskGetTickCount(); // Use current time as reference
    CurrentDiscoveryCache.tScheduledReplyTime = 0; // Not applicable for initiator's own DRep sending logic immediately
    CurrentDiscoveryCache.u32PreferredParentID = 0; // No parent for initiator

    // Add initiator's own info to its local cache (which is also its global table)
    MESHNETWORK_vAddOrUpdateLocalCache(MESHNETWORK_u32GetUniqueId(), 0, 0, 0); // RSSI/SNR 0 for self

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

bool MESHNETWORK_bGetDiscoveredNeighbors(MeshDiscoveredNeighbor_t *buffer, uint16_t max_entries, uint16_t *actual_entries) {
    if (xSemaphoreTake(xMeshNeighborTableMutex, portMAX_DELAY) == pdTRUE) {
        uint16_t count = (u8MeshDiscoveredNeighborsCount < max_entries) ? u8MeshDiscoveredNeighborsCount : max_entries;
        memcpy(buffer, tMeshDiscoveredNeighbors, count * sizeof(MeshDiscoveredNeighbor_t));
        *actual_entries = count;
        xSemaphoreGive(xMeshNeighborTableMutex);
        return true;
    }
    *actual_entries = 0;
    return false;
}

void MESHNETWORK_vClearDiscoveredNeighbors(void) {
    if (xSemaphoreTake(xMeshNeighborTableMutex, portMAX_DELAY) == pdTRUE) {
        u8MeshDiscoveredNeighborsCount = 0;
        memset(tMeshDiscoveredNeighbors, 0, sizeof(MeshDiscoveredNeighbor_t));
        xSemaphoreGive(xMeshNeighborTableMutex);
    }
}

// --- PRIVATE HELPER FUNCTIONS ---

static void MESHNETWORK_vAddOrUpdateGlobalNeighbor(uint32_t device_id, uint8_t hop_count, int16_t rssi, int8_t snr) {
    if (xSemaphoreTake(xMeshNeighborTableMutex, portMAX_DELAY) == pdTRUE) {
        bool found = false;
        for (uint16_t i = 0; i < u8MeshDiscoveredNeighborsCount; i++) {
            if (tMeshDiscoveredNeighbors[i].device_id == device_id) {
                // Update if new info is better (lower hop count, or stronger RSSI if hop count is same)
                if (hop_count < tMeshDiscoveredNeighbors[i].hop_count ||
                    (hop_count == tMeshDiscoveredNeighbors[i].hop_count && rssi > tMeshDiscoveredNeighbors[i].rssi)) {
                	tMeshDiscoveredNeighbors[i].hop_count = hop_count;
                	tMeshDiscoveredNeighbors[i].rssi = rssi;
                	tMeshDiscoveredNeighbors[i].snr = snr;
                }
                tMeshDiscoveredNeighbors[i].last_seen = xTaskGetTickCount();
                found = true;
                break;
            }
        }
        if (!found && u8MeshDiscoveredNeighborsCount < MESH_MAX_GLOBAL_NEIGHBORS) {
        	tMeshDiscoveredNeighbors[u8MeshDiscoveredNeighborsCount].device_id = device_id;
        	tMeshDiscoveredNeighbors[u8MeshDiscoveredNeighborsCount].hop_count = hop_count;
        	tMeshDiscoveredNeighbors[u8MeshDiscoveredNeighborsCount].rssi = rssi;
        	tMeshDiscoveredNeighbors[u8MeshDiscoveredNeighborsCount].snr = snr;
        	tMeshDiscoveredNeighbors[u8MeshDiscoveredNeighborsCount].last_seen = xTaskGetTickCount();
        	u8MeshDiscoveredNeighborsCount++;
        }
        xSemaphoreGive(xMeshNeighborTableMutex);
    }
}

static void MESHNETWORK_vAddOrUpdateLocalCache(uint32_t device_id, uint8_t hop_count, int16_t rssi, int8_t snr) {
    bool found = false;
    for (uint8_t i = 0; i < CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount; i++) {
        if (CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u32DeviceID == device_id) {
            if (hop_count < CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u8HopCount ||
                (hop_count == CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u8HopCount && rssi > CurrentDiscoveryCache.tLocalDiscoveredDevices[i].i16Rssi)) {
            	CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u8HopCount = hop_count;
            	CurrentDiscoveryCache.tLocalDiscoveredDevices[i].i16Rssi = rssi;
                CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u8Snr = snr;
            }
            CurrentDiscoveryCache.tLocalDiscoveredDevices[i].tLastUpdated = xTaskGetTickCount();
            CurrentDiscoveryCache.tLocalDiscoveredDevices[i].bReportedUpstream = false; // Mark as not yet reported
            found = true;
            break;
        }
    }
    if (!found && CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount < MESH_MAX_LOCAL_DISCOVERED_DEVICES) {
        CurrentDiscoveryCache.tLocalDiscoveredDevices[CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount].u32DeviceID = device_id;
        CurrentDiscoveryCache.tLocalDiscoveredDevices[CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount].u8HopCount = hop_count;
        CurrentDiscoveryCache.tLocalDiscoveredDevices[CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount].i16Rssi = rssi;
        CurrentDiscoveryCache.tLocalDiscoveredDevices[CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount].u8Snr = snr;
        CurrentDiscoveryCache.tLocalDiscoveredDevices[CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount].tLastUpdated = xTaskGetTickCount();
        CurrentDiscoveryCache.tLocalDiscoveredDevices[CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount].bReportedUpstream = false;
        CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount++;
    }
}

static TickType_t MESHNETWORK_tCalculateReplyDelay(uint8_t my_hop_count_to_dreq_sender) {
    TickType_t delay_ms = (TickType_t)my_hop_count_to_dreq_sender * MESH_BASE_HOP_DELAY_MS;
    delay_ms += MESHNETWORK_u32GetRandomNumber(MESH_REPLY_JITTER_WINDOW_MS); // Add random jitter
    return delay_ms;
}

static void MESHNETWORK_vHandleDReq(const MeshDReqPacket *dreq_packet) {
	uint8_t received_hop_count = MESH_MAX_TTL - dreq_packet->Ttl + 1;

	if (dreq_packet->Header.dReqID > u32LastProcessedDReqID ||
		(dreq_packet->Header.dReqID == u32LastProcessedDReqID && received_hop_count < CurrentDiscoveryCache.u8MyHopCountToDReqSender)) {

		if (dreq_packet->Header.dReqID > u32LastProcessedDReqID) {
			memset(&CurrentDiscoveryCache, 0, sizeof(DiscoveryCache_t));
			CurrentDiscoveryCache.u32DReqID = dreq_packet->Header.dReqID;
			CurrentDiscoveryCache.u32OGDreqSenderID = dreq_packet->OGDreqSenderID;
			CurrentDiscoveryCache.tDReqReceivedTime = xTaskGetTickCount();
			u32LastProcessedDReqID = dreq_packet->Header.dReqID;
			DBG("MeshNetwork: New DReq round %lu started.\r\n", dreq_packet->Header.dReqID);
			// Stop the timer immediately if a new DReq round begins, to cancel any pending reply
			xTimerStop(xMeshReplySchedulerTimer, 0);
		}

		if (CurrentDiscoveryCache.u8MyHopCountToDReqSender == 0 || received_hop_count < CurrentDiscoveryCache.u8MyHopCountToDReqSender) {
			CurrentDiscoveryCache.u8MyHopCountToDReqSender = received_hop_count;
			CurrentDiscoveryCache.u32PreferredParentID = dreq_packet->Header.senderID;
			CurrentDiscoveryCache.tScheduledReplyTime = CurrentDiscoveryCache.tDReqReceivedTime +
															  MESHNETWORK_tCalculateReplyDelay(CurrentDiscoveryCache.u8MyHopCountToDReqSender);

			DBG("MeshNetwork: DReq from %u (hop %u, RSSI %d, SNR %d), new preferred parent.\r\n",
				   dreq_packet->Header.senderID, CurrentDiscoveryCache.u8MyHopCountToDReqSender,
				   dreq_packet->Rssi, dreq_packet->Snr);
			DBG("Scheduled reply at %lu\r\n", CurrentDiscoveryCache.tScheduledReplyTime);

			// Start/Reset the software timer to trigger at my_reply_scheduled_time
			// Calculate delay from current tick count to scheduled time.
			TickType_t current_ticks = xTaskGetTickCount();
			TickType_t delay_ticks = 0;
			if (CurrentDiscoveryCache.tScheduledReplyTime > current_ticks) {
				delay_ticks = CurrentDiscoveryCache.tScheduledReplyTime - current_ticks;
			} else {
				// Should not happen for newly scheduled replies, but for safety, if already past due, trigger immediately.
				delay_ticks = 1; // Trigger as soon as possible
			}
			xTimerStop(xMeshReplySchedulerTimer, 0); // Stop if already running
			xTimerChangePeriod(xMeshReplySchedulerTimer, delay_ticks, 0); // Set new period (delay)
			xTimerStart(xMeshReplySchedulerTimer, 0); // Start the timer

			// ... (re-broadcast DReq logic) ...
			if (MESHNETWORK_u32GetUniqueId() != dreq_packet->Header.senderID && dreq_packet->Ttl > 0) {
				MESHNETWORK_vSendDReq(dreq_packet->Header.dReqID, MESHNETWORK_u32GetUniqueId(), dreq_packet->Ttl - 1);
			}
		}

		// ... (add_or_update_local_cache) ...
		MESHNETWORK_vAddOrUpdateLocalCache(MESHNETWORK_u32GetUniqueId(),
											  CurrentDiscoveryCache.u8MyHopCountToDReqSender,
											  dreq_packet->Rssi, dreq_packet->Snr);
	} else {
		DBG("MeshNetwork: Ignored old/inferior DReq %lu from %u (TTL %u, current hop %u).\r\n",
			   dreq_packet->Header.dReqID, dreq_packet->Header.senderID, dreq_packet->Ttl, CurrentDiscoveryCache.u8MyHopCountToDReqSender);
	}
}

static void MESHNETWORK_vHandleDRep(const MeshDRepPacket *drep_packet) {

    // Only process DRep for the current discovery round
    if (drep_packet->Header.dReqID != CurrentDiscoveryCache.u32DReqID ||
        drep_packet->OGDreqSenderID != CurrentDiscoveryCache.u32OGDreqSenderID) { // Ensure original sender matches
        DBG("MeshNetwork: Ignored DRep from %u for old/wrong DReq ID %lu (Original DReq Sender: %u).\r\n",
               drep_packet->Header.senderID, drep_packet->Header.dReqID, drep_packet->OGDreqSenderID);
        return;
    }

    // If this DRep is for the original DReq sender (i.e., we are the primary device)
    if (MESHNETWORK_u32GetUniqueId() == CurrentDiscoveryCache.u32OGDreqSenderID) {
        // Add all reported neighbors to the global table
        uint8_t num_entries = drep_packet->NeighborList_count;
        if (num_entries > MESH_MAX_NEIGHBORS_PER_PACKET) num_entries = MESH_MAX_NEIGHBORS_PER_PACKET; // Safety check

        for (uint8_t i = 0; i < num_entries; i++) {
            MESHNETWORK_vAddOrUpdateGlobalNeighbor(drep_packet->NeighborList[i].deviceID,
                                                      drep_packet->NeighborList[i].hopCount,
                                                      drep_packet->NeighborList[i].rssi,
                                                      drep_packet->NeighborList[i].snr);
        }
        DBG("MeshNetwork: Primary Device %u: Received DRep from %u, added %u neighbors.\r\n",
        		MESHNETWORK_u32GetUniqueId(), drep_packet->Header.senderID, drep_packet->NeighborList_count);
    }
    // If we are an intermediate relay node and this DRep is from a downstream child
    else if (drep_packet->ParentID == MESHNETWORK_u32GetUniqueId()) {
        DBG("MeshNetwork: Received DRep from downstream %u, num_neighbors %u\r\n",
               drep_packet->Header.senderID, drep_packet->NeighborList_count);
        // Add all reported neighbors to our local cache for potential relaying
        uint8_t num_entries = drep_packet->NeighborList_count;
        if (num_entries > MESH_MAX_NEIGHBORS_PER_PACKET) num_entries = MESH_MAX_NEIGHBORS_PER_PACKET; // Safety check

        for (uint8_t i = 0; i < num_entries; i++) {
            MESHNETWORK_vAddOrUpdateLocalCache(drep_packet->NeighborList[i].deviceID,
                                                  drep_packet->NeighborList[i].hopCount,
                                                  drep_packet->NeighborList[i].rssi,
                                                  drep_packet->NeighborList[i].snr);
        }
        // NEW: Signal the reply scheduler to potentially send an updated DRep upstream
        xEventGroupSetBits(xMeshReplySchedulerEventGroup, MESH_REPLY_SCHEDULE_BIT);
    } else {
        // DRep not for us or irrelevant for current discovery round
        DBG("MeshNetwork: Ignored DRep from %u (parent %u, current parent %u).\r\n",
               drep_packet->Header.senderID, drep_packet->ParentID, CurrentDiscoveryCache.u32PreferredParentID);
    }

}

void MESHNETWORK_vHandleTimeSyncMessage(const TimeSyncMessage *time_sync_msg)
{
    // Check if this TimeSyncID has been processed before
    if (time_sync_msg->TimesyncID > u32LastProcessedTimeSyncID) {
        DBG("MeshNetwork: New TimeSync (ID: %lu, TS: %lu, Interval: %lu) received.\r\n",
               time_sync_msg->TimesyncID, time_sync_msg->UtcTimestamp, time_sync_msg->WakeUpInterval);

        u32LastProcessedTimeSyncID = time_sync_msg->TimesyncID;
        MESHNETWORK_vSetWakeupInterval((WakeupInterval)time_sync_msg->WakeUpInterval);
        MESHNETWORK_vProcessUTCTimestamp(time_sync_msg->UtcTimestamp);

        // Re-broadcast (forward) the TimeSyncMessage to ensure it propagates through the mesh
        // Do NOT re-broadcast if this device was the original sender (though this function
        // should only be called for received packets, it's a good defensive check).
        // For simplicity, we'll re-broadcast it as is.
        MESHNETWORK_vSendTimesyncMessage(time_sync_msg->TimesyncID,
                                         time_sync_msg->UtcTimestamp,
                                         time_sync_msg->WakeUpInterval);

    } else {
        DBG("MeshNetwork: Ignored old/duplicate TimeSync (ID: %lu).\r\n", time_sync_msg->TimesyncID);
    }
}

static void MESHNETWORK_vSendDReq(uint32_t dreq_id, uint32_t sender_id, uint8_t ttl) {

	MeshDReqPacket meshDReqPacket;
	meshDReqPacket.Header.senderID = sender_id;
	meshDReqPacket.Header.dReqID = dreq_id;
	meshDReqPacket.OGDreqSenderID = (sender_id == MESHNETWORK_u32GetUniqueId() &&
			CurrentDiscoveryCache.u32DReqID == dreq_id) ? CurrentDiscoveryCache.u32OGDreqSenderID : MESHNETWORK_u32GetUniqueId();

	// A simpler approach for the initiator would be:
	// dreq_packet.original_dreq_sender_id = (sender_id == MESH_DEVICE_ID) ? MESH_DEVICE_ID : CurrentDiscoveryCache.original_dreq_sender_id;
	// The problem is that CurrentDiscoveryCache.original_dreq_sender_id is only valid if a DReq has been received.
	// When the *primary* device initiates (MeshNetwork_start_discovery_round), it sets this cache.
	// So, in MeshNetwork_send_dreq, if `sender_id == MESH_DEVICE_ID` and it's the *initial* send for this round,
	// then `original_dreq_sender_id` should be `MESH_DEVICE_ID`. Otherwise, it's whatever is in the cache.
    if (sender_id == MESHNETWORK_u32GetUniqueId() && dreq_id == CurrentDiscoveryCache.u32DReqID &&
    		CurrentDiscoveryCache.u32OGDreqSenderID != 0) {
        // We are relaying or initiating, and we know who the original sender is for this round.
    	meshDReqPacket.OGDreqSenderID = CurrentDiscoveryCache.u32OGDreqSenderID;
    } else {
        // This is typically for the very first DReq from the initiator,
        // or a fallback if cache is somehow not set (shouldn't happen for a valid round).
    	meshDReqPacket.OGDreqSenderID = MESHNETWORK_u32GetUniqueId();
    }

	meshDReqPacket.Ttl = ttl;
	meshDReqPacket.Rssi = 0;
	meshDReqPacket.Snr = 0;

    LoraRadio_Packet_t tx_packet;
	if(!MESHNETWORK_bEncodeDReqMessage(&meshDReqPacket, tx_packet.buffer, sizeof(tx_packet.buffer), &tx_packet.length))
	{
		DBG("\r\n--- PROTOBUF: PROBLEM ENCODING DREQ ---\r\n");
	}

    if (MESHNETWORK_bSendMessage(&tx_packet)) {
        DBG("MeshNetwork: Sent DReq ID %lu from %u, TTL %u.\r\n", dreq_id, sender_id, ttl);
    } else {
        DBG("MeshNetwork: Failed to send DReq ID %lu from %u.\r\n", dreq_id, sender_id);
    }
}

static void MESHNETWORK_vSendDRep(void) {
    // Dynamically allocate memory for the flexible array member
	MeshDRepPacket * pMeshDRepPacket = (MeshDRepPacket*)pvPortMalloc(sizeof(MeshDRepPacket) + (MESH_MAX_NEIGHBORS_PER_PACKET * sizeof(MeshNeighborInfo)));
    if (pMeshDRepPacket == NULL) {
        DBG("MeshNetwork: Failed to allocate memory for DRep packet.\r\n");
        return;
    }
    // Clear the allocated memory
    memset(pMeshDRepPacket, 0, sizeof(MeshDRepPacket) + (MESH_MAX_NEIGHBORS_PER_PACKET * sizeof(MeshNeighborInfo)));

    pMeshDRepPacket->Header.senderID = MESHNETWORK_u32GetUniqueId();
    pMeshDRepPacket->Header.dReqID = CurrentDiscoveryCache.u32DReqID;
    pMeshDRepPacket->OGDreqSenderID = CurrentDiscoveryCache.u32OGDreqSenderID; // Added this field
    pMeshDRepPacket->ParentID = CurrentDiscoveryCache.u32PreferredParentID;

    uint8_t neighbors_added = 0;

    // 1. Add this device's own info first (if not already sent)
    // The `my_info_sent` flag in CurrentDiscoveryCache ensures this DRep is sent only once per round
    // at the scheduled time. This DRep will contain the device's own info.
    // Check `my_info_sent` flag
    if (!CurrentDiscoveryCache.bInfoSent) {
        for (uint8_t i = 0; i < CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount; i++) {
            if (CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u32DeviceID == MESHNETWORK_u32GetUniqueId()) {
                memcpy(&pMeshDRepPacket->NeighborList[neighbors_added], &CurrentDiscoveryCache.tLocalDiscoveredDevices[i], sizeof(MeshNeighborInfo));
                CurrentDiscoveryCache.tLocalDiscoveredDevices[i].bReportedUpstream = true; // Mark as reported
                neighbors_added++;
                CurrentDiscoveryCache.bInfoSent = true; // Mark as sent for this round
                break;
            }
        }
    }

    // 2. Add prioritized downstream neighbors
    // Simple prioritization: iterate and add up to MAX_NEIGHBORS_PER_PACKET
    // A more advanced prioritization would sort by hop_count, then RSSI, then filter out already reported ones.
    // Don't re-add self, and only add if not already reported
    for (uint8_t i = 0; i < CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount && neighbors_added < MESH_MAX_NEIGHBORS_PER_PACKET; i++) {
        LocalNeighborEntry_t *entry = &CurrentDiscoveryCache.tLocalDiscoveredDevices[i];
        if (entry->u32DeviceID != MESHNETWORK_u32GetUniqueId() && !entry->bReportedUpstream) { // Don't re-add self, and only add if not already reported
            memcpy(&pMeshDRepPacket->NeighborList[neighbors_added], entry, sizeof(MeshNeighborInfo));
            entry->bReportedUpstream = true; // Mark as reported
            neighbors_added++;
        }
    }
    pMeshDRepPacket->NeighborList_count = neighbors_added;

    if (neighbors_added > 0) {

        LoraRadio_Packet_t tx_packet;


        if(!MESHNETWORK_bEncodeDRepMessage(pMeshDRepPacket, tx_packet.buffer, sizeof(tx_packet.buffer), &tx_packet.length))
        {
        	DBG("\r\n--- PROTOBUF: PROBLEM ENCODING DREP ---\r\n");
        }

        if (MESHNETWORK_bSendMessage(&tx_packet)) {
            DBG("MeshNetwork: Sending DRep to parent %u with %u neighbors. Current time: %lu.\r\n",
                   pMeshDRepPacket->ParentID, pMeshDRepPacket->NeighborList_count, xTaskGetTickCount());
            DBG("Next scheduled reply (if any): %lu\r\n", CurrentDiscoveryCache.tScheduledReplyTime);
            // Stop the timer after successfully sending the DRep
            xTimerStop(xMeshReplySchedulerTimer, 0);
        } else {
            DBG("MeshNetwork: Failed to queue DRep to parent %u for TX.\r\n", pMeshDRepPacket->ParentID);
        }
    }
    // Free the dynamically allocated memory
    vPortFree(pMeshDRepPacket);
}

void MESHNETWORK_vSendTimesyncMessage(uint32_t timesync_id, uint32_t utc_timestamp, WakeupInterval wakeup_interval) {

	TimeSyncMessage meshTimesyncPacket;
	meshTimesyncPacket.TimesyncID = timesync_id;
	meshTimesyncPacket.UtcTimestamp = utc_timestamp;
	meshTimesyncPacket.WakeUpInterval = wakeup_interval;

    LoraRadio_Packet_t tx_packet;
	if(!MESHNETWORK_bEncodeTimesyncMessage(&meshTimesyncPacket, tx_packet.buffer, sizeof(tx_packet.buffer), &tx_packet.length))
	{
		DBG("\r\n--- PROTOBUF: PROBLEM ENCODING TIMESYNC ---\r\n");
	}

    if (MESHNETWORK_bSendMessage(&tx_packet)) {
        DBG("MeshNetwork: Sent TS ID %u at %u UTC.\r\n", timesync_id, utc_timestamp);
    } else {
        DBG("MeshNetwork: Failed to send TS ID.\r\n", timesync_id);
    }
}
