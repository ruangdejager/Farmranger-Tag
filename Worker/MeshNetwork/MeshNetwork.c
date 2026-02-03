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
#include <limits.h>

#include "dbg_log.h"
#include "DeviceDiscovery.h"
#include "hal_rtc.h"
#include "Battery.h"

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

#define MAX_DREQ_ORIGINS 16

typedef struct {
    uint32_t origin_id;
    uint32_t last_dreq_id;
    bool     valid;
} DReqOriginState_t;

static DReqOriginState_t tDreqOrigins[MAX_DREQ_ORIGINS];

static DReqOriginState_t* MESHNETWORK_tFindOrCreateOrigin(uint32_t origin);

static DReqOriginState_t* MESHNETWORK_tFindOrCreateOrigin(uint32_t origin)
{
    for (int i = 0; i < MAX_DREQ_ORIGINS; i++) {
        if (tDreqOrigins[i].valid &&
        		tDreqOrigins[i].origin_id == origin) {
            return &tDreqOrigins[i];
        }
    }

    for (int i = 0; i < MAX_DREQ_ORIGINS; i++) {
        if (!tDreqOrigins[i].valid) {
        	tDreqOrigins[i].valid = true;
        	tDreqOrigins[i].origin_id = origin;
        	tDreqOrigins[i].last_dreq_id = 0;
            return &tDreqOrigins[i];
        }
    }

    return NULL; // table full
}


// --- INTERNAL DATA STRUCTURES ---

// Represents a discovered neighbor in the local cache of an intermediate node
typedef struct {
    uint32_t    u32DeviceID;
    uint8_t     u8HopCount; // Hops from Original_DReq_Sender_ID to this device
    int16_t     i16Rssi;      // RSSI of the DReq packet received by this device
    uint16_t	u16BatLevel;
    TickType_t  tLastUpdated; // For aging out entries
    bool        bReportedUpstream; // Flag to track if this device's info has been sent upstream
} LocalNeighborEntry_t;

// Cache for the current discovery round at an intermediate node
typedef struct {
    uint32_t            u32DReqID;
    uint32_t            u32OGDreqSenderID;

    uint32_t u32ParentCandidates[MESH_MAX_PARENT_CANDIDATES];
    uint8_t  u8ParentCandidateCount;

    uint8_t             u8MyHopCountToDReqSender;
    int16_t             i16BestParentRssi;
    TickType_t          tDReqReceivedTime;
    LocalNeighborEntry_t tLocalDiscoveredDevices[MESH_MAX_LOCAL_DISCOVERED_DEVICES];
    uint8_t             u8LocalDiscoveredDevicesCount;

    bool                bInfoSent;          // legacy flag (no longer used for gating)
    TickType_t          tScheduledReplyTime; // first time we may send

    uint8_t             u8DRepAttempts;     // how many DReps we’ve sent this round
    uint8_t             u8NextNeighborIndex;// round-robin index for fragmentation
} DiscoveryCache_t;

static uint16_t u16NodeId = 0;
static uint16_t u16MsgCounter = 0;

static DiscoveryCache_t CurrentDiscoveryCache;

static bool bFirstEverTimeSync = true; // We need to immediately go to sleep after first timesync to not burn unnecessary power

static uint32_t u32LastSeenTimeSyncID = 0;

// ---- Primary-heard tracking (used for recovery via App layer) ----
static uint64_t u64LastPrimaryHeardTick = 0;

// Discovered neighbors for the application layer
#define MESH_MAX_GLOBAL_NEIGHBORS (128) // Max entries in the primary device's global table
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
    if (tCurrentWakeupInterval <= WAKEUP_INTERVAL_MAX_COUNT) {
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
        DBG("MeshNetwork: Wakeup interval set to %d minutes (enum %d).\r\n",
               MESHNETWORK_u8GetWakeupInterval(), tCurrentWakeupInterval);
    } else {
        DBG("MeshNetwork: Attempted to set invalid wakeup interval enum: %X.\r\n", (unsigned int)new_interval);
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

TaskHandle_t MESHNETWORK_vParserTask_handle;
TaskHandle_t MESHNETWORK_vReplySchedulerTask_handle;

// --- FREE_RTOS SOFTWARE TIMER ---
static TimerHandle_t xMeshReplySchedulerTimer;
#define MESH_REPLY_TIMER_ID (1) // Unique ID for the timer
#define MESH_REPLY_TIMER_BIT (1UL << 1UL) // New event bit for timer expiration

// --- PRIVATE FUNCTION PROTOTYPES ---
static void MESHNETWORK_vAddOrUpdateGlobalNeighbor(uint32_t device_id, uint8_t hop_count, int16_t rssi, uint16_t batLevel);
static void MESHNETWORK_vAddOrUpdateLocalCache(uint32_t device_id, uint8_t hop_count, int16_t rssi, uint16_t batLevel);
static void MESHNETWORK_vAddParentCandidate(uint32_t senderID);
static bool MESHNETWORK_bIsMyParent(uint32_t parent_id);
static TickType_t MESHNETWORK_tCalculateReplyDelay(uint8_t my_hop_count_to_dreq_sender);
static void MESHNETWORK_vHandleDReq(const MeshDReqPacket *dreq_packet);
static void MESHNETWORK_vHandleDRep(const MeshDRepPacket *drep_packet);
static void MESHNETWORK_vSendDReq(uint32_t dreq_id, uint32_t sender_id, uint8_t ttl);
static void MESHNETWORK_vSendDRep(uint8_t parent_index);

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

    u16NodeId = MESHNETWORK_u32GetUniqueId();
    u16MsgCounter = 0;

    BaseType_t status;
    status = xTaskCreate(MESHNETWORK_vParserTask,
                "MeshParserTask",
                MESH_TASK_STACK_SIZE,
                NULL,
				MESH_RX_PARSER_TASK_PRIORITY,
				&MESHNETWORK_vParserTask_handle);
    configASSERT(status == pdPASS);

    status = xTaskCreate(MESHNETWORK_vReplySchedulerTask,
                "MeshReplyScheduler",
                MESH_TASK_STACK_SIZE,
                NULL,
				MESH_REPLY_SCHED_TASK_PRIORITY,
				&MESHNETWORK_vReplySchedulerTask_handle);
    configASSERT(status == pdPASS);

}

uint32_t MESHNETWORK_u32GenerateGlobalMsgID(void)
{
    u16MsgCounter++;
    return ((uint32_t)u16NodeId << 16) | (uint32_t)u16MsgCounter;
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
            // Only relay if:
            //  - we are in a valid discovery round
            //  - we are not the original DReq sender
            //  - we have a parent
            //  - we have something in local cache
            //  - we haven't exhausted our retry budget
            if (CurrentDiscoveryCache.u32DReqID != 0 && // Valid discovery round active
                MESHNETWORK_u32GetUniqueId() != CurrentDiscoveryCache.u32OGDreqSenderID &&
				CurrentDiscoveryCache.u8ParentCandidateCount > 0 &&
                CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount > 0 &&
                CurrentDiscoveryCache.u8DRepAttempts < MESH_DREP_RETRY_COUNT) {

                TickType_t now = xTaskGetTickCount();

                // Verify that the current time is >= scheduled time
                if (now >= CurrentDiscoveryCache.tScheduledReplyTime) {

                    // Send one DRep attempt
                	uint8_t parent_index =
                	    CurrentDiscoveryCache.u8DRepAttempts %
                	    CurrentDiscoveryCache.u8ParentCandidateCount;

                	MESHNETWORK_vSendDRep(parent_index);
                	CurrentDiscoveryCache.u8DRepAttempts++;


                    // Schedule next retry if we still have attempts left
                    if (CurrentDiscoveryCache.u8DRepAttempts < MESH_DREP_RETRY_COUNT) {
                        TickType_t delay_ticks = pdMS_TO_TICKS(MESH_DREP_RETRY_DELAY_MS);
                        xTimerChangePeriod(xMeshReplySchedulerTimer, delay_ticks, 0);
                        xTimerStart(xMeshReplySchedulerTimer, 0);
                        // Next send will be governed by the same conditions above
                    }

                } else {
                    DBG("MeshReplyScheduler: Timer fired early for some reason, re-scheduling. Current: %lu, Scheduled: %lu\r\n",
                           now, CurrentDiscoveryCache.tScheduledReplyTime);

                    TickType_t remaining_delay = 1;
                    if (CurrentDiscoveryCache.tScheduledReplyTime > now) {
                        remaining_delay = CurrentDiscoveryCache.tScheduledReplyTime - now;
                    }

                    xTimerChangePeriod(xMeshReplySchedulerTimer, remaining_delay, 0);
                    xTimerStart(xMeshReplySchedulerTimer, 0);
                }
            } else {
                DBG("MeshReplyScheduler: Timer fired but CurrentDiscoveryCache not valid or retry budget exhausted.\r\n");
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
#ifdef TEST_LORA_LED
    		if ((rx_packet.buffer[0] == 0xAA) && (rx_packet.buffer[2] == 0xCC))
    		{
            	BSP_LED_On(LED_RED);
            	vTaskDelay(pdMS_TO_TICKS(40));
            	BSP_LED_Off(LED_RED);
    		}
#endif
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

	            // Check if this DReq was originally sent by THIS device.
	            // If the Original DReq Sender ID in the packet matches this device's unique ID,
	            // then it's our own DReq being relayed back.
	            if (tMeshDReqPacket.OGDreqSenderID == MESHNETWORK_u32GetUniqueId()) {
	                DBG("MeshNetwork: Received own DReq (ID: %lu, Sender: %X), ignoring.\r\n",
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

    // Set up the cache for the primary device's own DReq
    // This part is critical as it sets up the initial state for the reply scheduler.
    memset(&CurrentDiscoveryCache, 0, sizeof(DiscoveryCache_t));
    CurrentDiscoveryCache.u32DReqID = dreq_id;
    CurrentDiscoveryCache.u32OGDreqSenderID = original_dreq_sender_id;
    CurrentDiscoveryCache.u8MyHopCountToDReqSender = 0; // 0 hops for the initiator
    CurrentDiscoveryCache.tDReqReceivedTime = xTaskGetTickCount(); // Use current time as reference
    CurrentDiscoveryCache.tScheduledReplyTime = 0; // Not applicable for initiator's own DRep sending logic immediately

    // Add initiator's own info to its local cache (which is also its global table)
    MESHNETWORK_vAddOrUpdateLocalCache(MESHNETWORK_u32GetUniqueId(), 0, 0, BAT_u16GetVoltage()); // RSSI for self

    DBG("MeshNetwork: Initiating DReq with ID %X from %X\r\n", dreq_id, original_dreq_sender_id);
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

static void MESHNETWORK_vAddOrUpdateGlobalNeighbor(uint32_t device_id,
                                                  uint8_t hop_count,
                                                  int16_t rssi,
                                                  uint16_t batLevel)
{
    if (xSemaphoreTake(xMeshNeighborTableMutex, portMAX_DELAY) == pdTRUE) {

        for (uint16_t i = 0; i < u8MeshDiscoveredNeighborsCount; i++) {
            if (tMeshDiscoveredNeighbors[i].device_id == device_id) {

                if (hop_count < tMeshDiscoveredNeighbors[i].hop_count ||
                   (hop_count == tMeshDiscoveredNeighbors[i].hop_count &&
                    rssi > tMeshDiscoveredNeighbors[i].rssi)) {

                    tMeshDiscoveredNeighbors[i].hop_count = hop_count;
                    tMeshDiscoveredNeighbors[i].rssi = rssi;
                    tMeshDiscoveredNeighbors[i].batLevel = batLevel;
                }

                tMeshDiscoveredNeighbors[i].last_seen = xTaskGetTickCount();
                xSemaphoreGive(xMeshNeighborTableMutex);
                return;
            }
        }

        // ---- New device: always add ----
        if (u8MeshDiscoveredNeighborsCount < MESH_MAX_GLOBAL_NEIGHBORS) {
            tMeshDiscoveredNeighbors[u8MeshDiscoveredNeighborsCount++] =
                (MeshDiscoveredNeighbor_t){
                    .device_id = device_id,
                    .hop_count = hop_count,
                    .rssi = rssi,
                    .batLevel = batLevel,
                    .last_seen = xTaskGetTickCount()
                };
        }

        xSemaphoreGive(xMeshNeighborTableMutex);
    }
}


static void MESHNETWORK_vAddOrUpdateLocalCache(uint32_t device_id, uint8_t hop_count, int16_t rssi, uint16_t batLevel) {
    bool found = false;
    for (uint8_t i = 0; i < CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount; i++) {
        if (CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u32DeviceID == device_id) {
            if (hop_count < CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u8HopCount ||
                (hop_count == CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u8HopCount && rssi > CurrentDiscoveryCache.tLocalDiscoveredDevices[i].i16Rssi)) {
            	CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u8HopCount = hop_count;
            	CurrentDiscoveryCache.tLocalDiscoveredDevices[i].i16Rssi = rssi;
            	CurrentDiscoveryCache.tLocalDiscoveredDevices[i].u16BatLevel = batLevel;
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
        CurrentDiscoveryCache.tLocalDiscoveredDevices[CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount].u16BatLevel = batLevel;
        CurrentDiscoveryCache.tLocalDiscoveredDevices[CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount].tLastUpdated = xTaskGetTickCount();
        CurrentDiscoveryCache.tLocalDiscoveredDevices[CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount].bReportedUpstream = false;
        CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount++;
    }
}

static void MESHNETWORK_vAddParentCandidate(uint32_t senderID)
{
    // Avoid duplicates
    for (uint8_t i = 0; i < CurrentDiscoveryCache.u8ParentCandidateCount; i++) {
        if (CurrentDiscoveryCache.u32ParentCandidates[i] == senderID) {
            return;
        }
    }

    if (CurrentDiscoveryCache.u8ParentCandidateCount < MESH_MAX_PARENT_CANDIDATES) {
        CurrentDiscoveryCache.u32ParentCandidates[
            CurrentDiscoveryCache.u8ParentCandidateCount++
        ] = senderID;
    }
}

static bool MESHNETWORK_bIsMyParent(uint32_t parent_id)
{
    for (uint8_t i = 0; i < CurrentDiscoveryCache.u8ParentCandidateCount; i++) {
        if (CurrentDiscoveryCache.u32ParentCandidates[i] == parent_id) {
            return true;
        }
    }
    return false;
}

static TickType_t MESHNETWORK_tCalculateReplyDelay(uint8_t my_hop_count_to_dreq_sender)
{
    uint8_t clamped_hop = my_hop_count_to_dreq_sender ? my_hop_count_to_dreq_sender : 1;
    if (clamped_hop > MESH_MAX_TTL) clamped_hop = MESH_MAX_TTL;

    uint8_t hops_from_edge = (MESH_MAX_TTL - clamped_hop);

    uint32_t delay_ms = (uint32_t)hops_from_edge * MESH_BASE_HOP_DELAY_MS;
    delay_ms += MESHNETWORK_u32GetRandomNumber(MESH_REPLY_JITTER_WINDOW_MS);

    return pdMS_TO_TICKS(delay_ms); // return ticks now
}


static void MESHNETWORK_vHandleDReq(const MeshDReqPacket *dreq_packet) {

	// ---- Update last primary-heard tick ----
	// Only update if this node is NOT the original sender.
	if (DEVICE_DISCOVERY_tGetDeviceRole() == DEVICE_SECONDARY)
	{
#warning only do ths at time sync request?
//		MESHNETWORK_vUpdatePrimaryLastSeen();
	}


	uint8_t received_hop_count = MESH_MAX_TTL - dreq_packet->Ttl + 1;

	DReqOriginState_t *origin = MESHNETWORK_tFindOrCreateOrigin(dreq_packet->OGDreqSenderID);
	if (!origin) {
	    DBG("MeshNetwork: DReq origin table full, dropping.\r\n");
	    return;
	}

	bool isNewRound = (origin->last_dreq_id != dreq_packet->Header.dReqID);

	uint8_t oldHop = CurrentDiscoveryCache.u8MyHopCountToDReqSender;
	uint8_t newHop = received_hop_count;

	int16_t oldRssi = CurrentDiscoveryCache.i16BestParentRssi;
	int16_t newRssi = dreq_packet->Rssi;

	bool accept = false;

	if (isNewRound) {
	    accept = true;
	}
	else if (newHop < oldHop) {
	    accept = true;
	}
	else if (newHop == oldHop && newRssi > oldRssi) {
	    accept = true;
	}
	else if ((newHop == oldHop + 1) &&
	         (newRssi >= oldRssi + MESH_RSSI_HOP_BONUS_DB)) {
	    accept = true;
	}

	if (accept)
	{
	    if (isNewRound)
	    {
	        memset(&CurrentDiscoveryCache, 0, sizeof(DiscoveryCache_t));
	        CurrentDiscoveryCache.u32DReqID = dreq_packet->Header.dReqID;
	        CurrentDiscoveryCache.u32OGDreqSenderID = dreq_packet->OGDreqSenderID;
	        CurrentDiscoveryCache.tDReqReceivedTime = xTaskGetTickCount();
	        CurrentDiscoveryCache.i16BestParentRssi = INT16_MIN;

	        origin->last_dreq_id = dreq_packet->Header.dReqID;

	        xTimerStop(xMeshReplySchedulerTimer, 0);

	        DBG("MeshNetwork: New DReq round %X from origin %X\r\n",
	            dreq_packet->Header.dReqID,
	            dreq_packet->OGDreqSenderID);
	    }

	    // --- Switch parent ---
	    CurrentDiscoveryCache.u8MyHopCountToDReqSender = received_hop_count;
	    CurrentDiscoveryCache.i16BestParentRssi = dreq_packet->Rssi;

	    CurrentDiscoveryCache.u8ParentCandidateCount = 0;
	    MESHNETWORK_vAddParentCandidate(dreq_packet->Header.senderID);

	    DBG("MeshNetwork: Parent switch to %X (hop=%u, rssi=%d)\r\n",
	        dreq_packet->Header.senderID,
	        received_hop_count,
	        dreq_packet->Rssi);

	    // --- Schedule reply ---
	    CurrentDiscoveryCache.tScheduledReplyTime =
	        xTaskGetTickCount() +
	        MESHNETWORK_tCalculateReplyDelay(CurrentDiscoveryCache.u8MyHopCountToDReqSender);

	    TickType_t now = xTaskGetTickCount();
	    TickType_t delay_ticks = 1;
	    if (CurrentDiscoveryCache.tScheduledReplyTime > now) {
	        delay_ticks = CurrentDiscoveryCache.tScheduledReplyTime - now;
	    }

	    xTimerStop(xMeshReplySchedulerTimer, 0);
	    xTimerChangePeriod(xMeshReplySchedulerTimer, delay_ticks, 0);
	    xTimerStart(xMeshReplySchedulerTimer, 0);

	    MESHNETWORK_vAddOrUpdateLocalCache(
	        MESHNETWORK_u32GetUniqueId(),
	        CurrentDiscoveryCache.u8MyHopCountToDReqSender,
	        dreq_packet->Rssi,
	        BAT_u16GetVoltage());

	    // --------------------------------------------------
	    // DREQ FLOODING (controlled)
	    // --------------------------------------------------
	    if (dreq_packet->Ttl > 1) {

	        MeshDReqPacket fwd = *dreq_packet;

	        fwd.Header.senderID = MESHNETWORK_u32GetUniqueId();
	        fwd.Ttl = dreq_packet->Ttl - 1;
	        fwd.Rssi = 0; // filled by receiver

	        LoraRadio_Packet_t tx;
	        if (MESHNETWORK_bEncodeDReqMessage(&fwd,
	                                           tx.buffer,
	                                           sizeof(tx.buffer),
	                                           &tx.length))
	        {
	            if (MESHNETWORK_bSendMessage(&tx)) {
	                DBG("MeshNetwork: Forwarded DReq %X ttl=%u\r\n",
	                    fwd.Header.dReqID, fwd.Ttl);
	            }
	        }
	    }

	}
	else
	{
	    DBG("MeshNetwork: Ignored inferior DReq (hop=%u rssi=%d)\r\n",
	        received_hop_count,
	        dreq_packet->Rssi);
	}

}

static void MESHNETWORK_vHandleDRep(const MeshDRepPacket *drep_packet) {

    // Only process DRep for the current discovery round
    if (drep_packet->Header.dReqID != CurrentDiscoveryCache.u32DReqID ||
        drep_packet->OGDreqSenderID != CurrentDiscoveryCache.u32OGDreqSenderID) { // Ensure original sender matches
        DBG("MeshNetwork: Ignored DRep from %X for old/wrong DReq ID %lu (Original DReq Sender: %X).\r\n",
               drep_packet->Header.senderID, drep_packet->Header.dReqID, drep_packet->OGDreqSenderID);
        return;
    }

    // -------------------------------------------------------------
    // Primary device: accept DReps
    // -------------------------------------------------------------
    if (MESHNETWORK_u32GetUniqueId() == CurrentDiscoveryCache.u32OGDreqSenderID) {

        bool sender_is_direct_child =
            (drep_packet->ParentID == MESHNETWORK_u32GetUniqueId());

        uint8_t num_entries = drep_packet->NeighborList_count;
        if (num_entries > MESH_MAX_NEIGHBORS_PER_PACKET)
            num_entries = MESH_MAX_NEIGHBORS_PER_PACKET;

        for (uint8_t i = 0; i < num_entries; i++) {

            MeshNeighborInfo *n = &drep_packet->NeighborList[i];

            // ❌ Drop self-entry from non-direct children
            if (!sender_is_direct_child &&
                n->deviceID == drep_packet->Header.senderID) {
                continue;
            }

            MESHNETWORK_vAddOrUpdateGlobalNeighbor(
                n->deviceID,
                n->hopCount,
                n->rssi,
                n->batVoltage);
        }

        DBG("MeshNetwork: Primary accepted DRep from %X (direct=%d)\r\n",
            drep_packet->Header.senderID,
            sender_is_direct_child);
    }

    // If we are an intermediate relay node and this DRep is from a downstream child
    /* ------------------ intermediate handling of child DRep ------------------ */
    else if (drep_packet->ParentID == MESHNETWORK_u32GetUniqueId())
    {

        uint8_t num_entries = drep_packet->NeighborList_count;
        if (num_entries > MESH_MAX_NEIGHBORS_PER_PACKET) num_entries = MESH_MAX_NEIGHBORS_PER_PACKET;

        TickType_t now = xTaskGetTickCount();
        bool added_any = false;

        for (uint8_t i = 0; i < num_entries; i++) {

            uint8_t adjusted_hop = drep_packet->NeighborList[i].hopCount + 1;

            /* Add/update local cache with child's neighbor entries (increment hop). */
            MESHNETWORK_vAddOrUpdateLocalCache(
                drep_packet->NeighborList[i].deviceID,
                adjusted_hop,
                drep_packet->NeighborList[i].rssi,
                drep_packet->NeighborList[i].batVoltage);

            added_any = true;
            DBG("MeshNetwork: Intermediate %X added child-entry ID=%X hop=%u rssi=%d\r\n",
                MESHNETWORK_u32GetUniqueId(),
                drep_packet->NeighborList[i].deviceID,
                adjusted_hop,
                drep_packet->NeighborList[i].rssi);
        }

        /*
         * Important: when a downstream child sends a DRep, give the node a short
         * propagation window so it can collect further DReps from other children
         * before it composes its own upstream DRep.
         *
         * Strategy:
         *  - compute a small wait equal to one base-hop delay (plus jitter)
         *  - ensure tScheduledReplyTime is at least now + wait
         *  - restart reply timer accordingly
         */

        if (added_any) {
            TickType_t child_wait = pdMS_TO_TICKS(MESH_BASE_HOP_DELAY_MS) +
                                    pdMS_TO_TICKS(MESHNETWORK_u32GetRandomNumber(MESH_REPLY_JITTER_WINDOW_MS));

            TickType_t new_sched = now + child_wait;

            if (CurrentDiscoveryCache.tScheduledReplyTime < new_sched) {
                CurrentDiscoveryCache.tScheduledReplyTime = new_sched;
                CurrentDiscoveryCache.u8DRepAttempts = 0; // reset attempts so we try fresh after new data

                /* start/adjust the reply timer to fire at tScheduledReplyTime */
                TickType_t delay_ticks = (CurrentDiscoveryCache.tScheduledReplyTime > now) ?
                                          (CurrentDiscoveryCache.tScheduledReplyTime - now) : 1;

                xTimerStop(xMeshReplySchedulerTimer, 0);
                xTimerChangePeriod(xMeshReplySchedulerTimer, delay_ticks, 0);
                xTimerStart(xMeshReplySchedulerTimer, 0);

                DBG("MeshNetwork: Intermediate %X extended scheduled reply by %lu ms (now %lu -> sched %lu)\r\n",
                    MESHNETWORK_u32GetUniqueId(),
                    (unsigned long)child_wait,
                    (unsigned long)now,
                    (unsigned long)CurrentDiscoveryCache.tScheduledReplyTime);
            } else {
                DBG("MeshNetwork: Intermediate %X received child DRep but existing schedule (%lu) is later than new (%lu)\r\n",
                    MESHNETWORK_u32GetUniqueId(),
                    (unsigned long)CurrentDiscoveryCache.tScheduledReplyTime,
                    (unsigned long)new_sched);
            }
        }

        /* Trigger upstream relay evaluation (reply scheduler task will check scheduled time) */
        xEventGroupSetBits(xMeshReplySchedulerEventGroup,
                           MESH_REPLY_SCHEDULE_BIT);

    } else {
        // DRep not for us or irrelevant for current discovery round
    	DBG("MeshNetwork: Ignored DRep from %X (parent %X not in my parent set).\r\n",
    	       drep_packet->Header.senderID,
    	       drep_packet->ParentID);
    }

}

void MESHNETWORK_vHandleTimeSyncMessage(const TimeSyncMessage *time_sync_msg)
{

	if (time_sync_msg->TimesyncID == u32LastSeenTimeSyncID) {
	    DBG("MeshNetwork: Duplicate TimeSync %X ignored.\r\n",
	        time_sync_msg->TimesyncID);
	    return;
	}

	u32LastSeenTimeSyncID = time_sync_msg->TimesyncID;


    // Check if this TimeSyncID has been processed before

	// ---- Update primary-heard tick ----
	if (DEVICE_DISCOVERY_tGetDeviceRole() == DEVICE_SECONDARY)
	{
		MESHNETWORK_vUpdatePrimaryLastSeen();
	}

	DBG("MeshNetwork: New TimeSync (ID: %X, TS: %lu, Interval: %lu) received.\r\n",
		   time_sync_msg->TimesyncID, time_sync_msg->UtcTimestamp, time_sync_msg->WakeUpInterval);

	MESHNETWORK_vSetWakeupInterval((WakeupInterval)time_sync_msg->WakeUpInterval);
	MESHNETWORK_vProcessUTCTimestamp(time_sync_msg->UtcTimestamp);

	// Re-broadcast (forward) the TimeSyncMessage to ensure it propagates through the mesh
	// Do NOT re-broadcast if this device was the original sender (though this function
	// should only be called for received packets, it's a good defensive check).
	// For simplicity, we'll re-broadcast it as is.
	MESHNETWORK_vSendTimesyncMessage(time_sync_msg->TimesyncID,
									 time_sync_msg->UtcTimestamp,
									 time_sync_msg->WakeUpInterval);

//        if (bFirstEverTimeSync) {
//		    LORARADIO_vEnterDeepSleep();
//		    SYSTEM_vActivateDeepSleep();
//		    bFirstEverTimeSync = false;
//        }

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

    LoraRadio_Packet_t tx_packet;
	if(!MESHNETWORK_bEncodeDReqMessage(&meshDReqPacket, tx_packet.buffer, sizeof(tx_packet.buffer), &tx_packet.length))
	{
		DBG("\r\n--- PROTOBUF: PROBLEM ENCODING DREQ ---\r\n");
	}

    if (MESHNETWORK_bSendMessage(&tx_packet)) {
        DBG("MeshNetwork: Sent DReq ID %X from %X, TTL %X.\r\n", dreq_id, sender_id, ttl);
    } else {
        DBG("MeshNetwork: Failed to send DReq ID %lu from %X.\r\n", dreq_id, sender_id);
    }
}

static void MESHNETWORK_vSendDRep(uint8_t parent_index) {
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

    if (parent_index >= CurrentDiscoveryCache.u8ParentCandidateCount) {
        vPortFree(pMeshDRepPacket);
        return;
    }

    pMeshDRepPacket->ParentID =
        CurrentDiscoveryCache.u32ParentCandidates[parent_index];

    uint8_t neighbors_added = 0;
    uint32_t my_id = MESHNETWORK_u32GetUniqueId();

    // 1. Always add this device's own info first (if present in local cache)
    for (uint8_t i = 0; i < CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount &&
                        neighbors_added < MESH_MAX_NEIGHBORS_PER_PACKET; i++) {
        LocalNeighborEntry_t *entry = &CurrentDiscoveryCache.tLocalDiscoveredDevices[i];
        if (entry->u32DeviceID == my_id) {

        	MeshNeighborInfo *out = &pMeshDRepPacket->NeighborList[neighbors_added];

        	out->deviceID   = entry->u32DeviceID;
        	out->hopCount   = entry->u8HopCount;
        	out->rssi       = entry->i16Rssi;
        	out->batVoltage = entry->u16BatLevel;

        	neighbors_added++;

            break;
        }
    }

    // 2. Add other neighbors using a simple round-robin starting from u8NextNeighborIndex
    if (CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount > 0 &&
        neighbors_added < MESH_MAX_NEIGHBORS_PER_PACKET) {

        uint8_t count = CurrentDiscoveryCache.u8LocalDiscoveredDevicesCount;
        uint8_t start = CurrentDiscoveryCache.u8NextNeighborIndex;

        for (uint8_t n = 0; n < count && neighbors_added < MESH_MAX_NEIGHBORS_PER_PACKET; n++) {
            uint8_t idx = (start + n) % count;
            LocalNeighborEntry_t *entry = &CurrentDiscoveryCache.tLocalDiscoveredDevices[idx];

            // Skip self here (we already added it above if present)
            if (entry->u32DeviceID == my_id) {
                continue;
            }

            MeshNeighborInfo *out = &pMeshDRepPacket->NeighborList[neighbors_added];

            out->deviceID   = entry->u32DeviceID;
            out->hopCount   = entry->u8HopCount;
            out->rssi       = entry->i16Rssi;
            out->batVoltage = entry->u16BatLevel;

            neighbors_added++;

        }

        // Advance the round-robin index for the next send attempt
        if (neighbors_added > 0) {
            CurrentDiscoveryCache.u8NextNeighborIndex =
                (start + neighbors_added) % count;
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
        	DBG("MeshNetwork: Sending DRep to parent[%X]=%X ...",
        	    parent_index,
        	    pMeshDRepPacket->ParentID);
            DBG("Next scheduled reply (if any): %lu\r\n", CurrentDiscoveryCache.tScheduledReplyTime);
        } else {
            DBG("MeshNetwork: Failed to queue DRep to parent %X for TX.\r\n", pMeshDRepPacket->ParentID);
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
        DBG("MeshNetwork: Sent TS ID %X at %ul UTC.\r\n", timesync_id, utc_timestamp);
    } else {
        DBG("MeshNetwork: Failed to send TS ID.\r\n", timesync_id);
    }
}

void MESHNETWORK_vUpdatePrimaryLastSeen(void)
{
	u64LastPrimaryHeardTick = HAL_RTC_u64GetValue();
}

uint64_t MESHNETWORK_u64GetLastPrimaryHeardTick(void)
{
    return u64LastPrimaryHeardTick;
}

