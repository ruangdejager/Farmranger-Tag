/*
 * MeshNetwork.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */


#include "MeshNetwork.h"
#include "MeshNetwork_Port.h"
#include "LoraRadio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "stdlib.h"
#include <limits.h>

#include "dbg_log.h"
#include "DeviceDiscovery.h"
#include "hal_rtc.h"
#include "Battery.h"


/* Local config aliases */
#define MESH_BEACON_INTERVAL_MS_CFG 			MESH_BEACON_INTERVAL_MS
#define MESH_PRIMARY_ACK_INTERVAL_MS_CFG 		MESH_PRIMARY_ACK_INTERVAL_MS
#define MESH_DISCOVERY_IDLE_MS_CFG 				MESH_DISCOVERY_IDLE_MS

#define MESH_TX_QUEUE_LEN        24
#define MESH_TX_MAX_PACKET_SIZE  128

typedef struct
{
    uint8_t  u8Buf[MESH_TX_MAX_PACKET_SIZE];
    uint16_t u16Len;
    TickType_t tReadyTick;
} MeshTxItem_t;

static QueueHandle_t xMeshTxQueue = NULL;
static TaskHandle_t  xMeshTxTaskHandle = NULL;

/* FreeRTOS objects */
static TaskHandle_t xParserTaskHandle = NULL;
static TimerHandle_t xBeaconTimer = NULL;
static TimerHandle_t xPrimaryAckTimer = NULL;
static SemaphoreHandle_t xForwardRingMutex = NULL;
static SemaphoreHandle_t xNeighborTableMutex = NULL;

/* State */
static ForwardRing_t tForwardRing;
static NeighborEntry_t tNeighborTable[MESH_MAX_NEIGHBORS];
static uint16_t u16NeighborCount = 0;
static TickType_t tLastBeaconHeardTick = 0;

static bool bNodeBeaconing = false;
static uint32_t u32NodeBeaconDreqId = 0;
static uint8_t u8NodeHopCount = 0;
static NodeRole_e eNodeRole = NODE_ROLE_UNKNOWN;

/* Wakeup interval */
static WakeupInterval tCurrentWakeupInterval = WAKEUP_INTERVAL_15_MIN;
static const uint8_t u8CurrentWakeupIntervalMin[] = {
[WAKEUP_INTERVAL_15_MIN] = 15,
[WAKEUP_INTERVAL_30_MIN] = 30,
[WAKEUP_INTERVAL_60_MIN] = 60,
[WAKEUP_INTERVAL_120_MIN] = 120
};

static const char * const MeshPktTypeStr[] =
{
    [MeshPktType_Reserved] = "Reserved",
    [MeshPktType_DReq]     = "DReq",
    [MeshPktType_DBeacon]  = "DBeacon",
    [MeshPktType_DAck]     = "DAck",
    [MeshPktType_TimeSync] = "TimeSync"
};

typedef struct
{
    uint8_t  u8Buf[128];
    size_t   u32Len;
    bool     bPending;
} MeshJitterTx_t;

/* local msg counter */
static uint16_t u16MsgCounter = 0;
static uint64_t u64LastPrimaryHeardTick = 0;
static int16_t i16LastDreqRssi = 0;

static void MESHNETWORK_vTxTask(void *pvParameters);
static bool MESHNETWORK_bSendPacket(const uint8_t *pBuf, size_t u32Len);

/* Local node controls */
static void MESHNETWORK_vStartBeaconing(uint32_t u32DreqId, uint8_t u8HopCount);
static void MESHNETWORK_vStopBeaconing(uint32_t u32DreqId);


/* ------------------------------------------------------------------ */
/* Endian helpers: on-wire is big-endian */
static void write_u32_be(uint8_t *pBuf, uint32_t u32Val) {
    pBuf[0] = (u32Val >> 24) & 0xFF;
    pBuf[1] = (u32Val >> 16) & 0xFF;
    pBuf[2] = (u32Val >> 8) & 0xFF;
    pBuf[3] = (u32Val) & 0xFF;
}
static uint32_t read_u32_be(const uint8_t *pBuf) {
    return ((uint32_t)pBuf[0] << 24) | ((uint32_t)pBuf[1] << 16) | ((uint32_t)pBuf[2] << 8) | (uint32_t)pBuf[3];
}
static void write_u16_be(uint8_t *pBuf, uint16_t u16Val) {
    pBuf[0] = (u16Val >> 8) & 0xFF;
    pBuf[1] = (u16Val) & 0xFF;
}
static uint16_t read_u16_be(const uint8_t *pBuf) {
    return (uint16_t)(((uint16_t)pBuf[0] << 8) | (uint16_t)pBuf[1]);
}
static inline void write_s16_be(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)(v & 0xFF);
}
static inline int16_t read_s16_be(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* ------------------------------------------------------------------ */
/* Forward ring operations */
static bool FORWARD_bHasSeen(uint32_t u32MsgId) {
    bool bFound = false;
    if (xSemaphoreTake(xForwardRingMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (uint8_t i=0;i < tForwardRing.u8Count;i++) {
            uint8_t u8Idx = (tForwardRing.u8Head + FORWARD_RING_SIZE - tForwardRing.u8Count + i) % FORWARD_RING_SIZE;
            if (tForwardRing.u32Ring[u8Idx] == u32MsgId) { bFound = true; break; }
        }
        xSemaphoreGive(xForwardRingMutex);
    }
    return bFound;
}
static void FORWARD_vAdd(uint32_t u32MsgId) {
    if (xSemaphoreTake(xForwardRingMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        tForwardRing.u32Ring[tForwardRing.u8Head] = u32MsgId;
        tForwardRing.u8Head = (tForwardRing.u8Head + 1) % FORWARD_RING_SIZE;
        if (tForwardRing.u8Count < FORWARD_RING_SIZE) tForwardRing.u8Count++;
        xSemaphoreGive(xForwardRingMutex);
    }
}

/* ------------------------------------------------------------------ */
/* Neighbor table operations */
static void NEIGHBOR_vAddOrUpdate(uint32_t u32DeviceId, uint8_t u8HopCount, int16_t i16Rssi, uint16_t u16BatMv) {
    if (xSemaphoreTake(xNeighborTableMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (uint16_t i=0;i<u16NeighborCount;i++) {
            if (tNeighborTable[i].u32DeviceId == u32DeviceId) {
                tNeighborTable[i].u8HopCount = u8HopCount;
                tNeighborTable[i].u16BatMv = u16BatMv;
                tNeighborTable[i].i16Rssi = i16Rssi;
                if (tNeighborTable[i].bAcked) tNeighborTable[i].bAcked = false; /* requeue */
                xSemaphoreGive(xNeighborTableMutex);
                tLastBeaconHeardTick = xTaskGetTickCount();
                return;
            }
        }
        if (u16NeighborCount < MESH_MAX_NEIGHBORS) {
            tNeighborTable[u16NeighborCount].u32DeviceId = u32DeviceId;
            tNeighborTable[u16NeighborCount].u8HopCount = u8HopCount;
            tNeighborTable[u16NeighborCount].u16BatMv = u16BatMv;
            tNeighborTable[u16NeighborCount].i16Rssi = i16Rssi;
            tNeighborTable[u16NeighborCount].bAcked = false;
            u16NeighborCount++;
            tLastBeaconHeardTick = xTaskGetTickCount();
        }
        xSemaphoreGive(xNeighborTableMutex);
    }
}
static void NEIGHBOR_vClearAll(void) {
    if (xSemaphoreTake(xNeighborTableMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memset(tNeighborTable,0,sizeof(tNeighborTable));
        u16NeighborCount = 0;
        xSemaphoreGive(xNeighborTableMutex);
    }
}

/* ------------------------------------------------------------------ */
/* Encoding helpers */
static bool ENCODE_bDReq(uint32_t u32DreqId, uint32_t u32OriginId, uint32_t u32SenderId, uint8_t u8SenderHopCount, uint8_t *pBuf, size_t u32BufLen, size_t *pu32Written) {
    if (u32BufLen < 1 + 4 + 4 + 4 + 1) return false;
    pBuf[0] = (uint8_t)MeshPktType_DReq;
    write_u32_be(&pBuf[1], u32DreqId);
    write_u32_be(&pBuf[5], u32OriginId);
    write_u32_be(&pBuf[9], u32SenderId);
    pBuf[13] = u8SenderHopCount;
    *pu32Written = 14;
    return true;
}
static bool ENCODE_bDBeacon(const MeshPktDBeacon_t *ptBeacon, uint8_t *pBuf, size_t u32BufLen, size_t *pu32Written) {
    if (u32BufLen < 18) return false;
    pBuf[0] = (uint8_t)MeshPktType_DBeacon;
    write_u32_be(&pBuf[1],  ptBeacon->u32DreqId);
    write_u32_be(&pBuf[5],  ptBeacon->u32DeviceId);
    write_u16_be(&pBuf[9],  ptBeacon->u16BatMv);
    pBuf[11] = ptBeacon->u8HopCount;
    write_s16_be(&pBuf[12], ptBeacon->i16Rssi);
    write_u32_be(&pBuf[14], ptBeacon->u32BeaconMsgId);
    *pu32Written = 18;
    return true;
}

static bool ENCODE_bDAck(const MeshPktDAck_t *ptAck, uint8_t *pBuf, size_t u32BufLen, size_t *pu32Written) {
    size_t u32Needed = 1 + 4 + 4 + 4 + 1 + (4 * ptAck->u8AckCount);
    if (u32BufLen < u32Needed) return false;
    pBuf[0] = (uint8_t)MeshPktType_DAck;
    write_u32_be(&pBuf[1], ptAck->u32AckMsgId);
    write_u32_be(&pBuf[5], ptAck->u32DreqId);
    write_u32_be(&pBuf[9], ptAck->u32SenderId);
    pBuf[13] = ptAck->u8AckCount;
    for (uint8_t i=0;i<ptAck->u8AckCount;i++) write_u32_be(&pBuf[14 + 4*i], ptAck->u32AckedIds[i]);
    *pu32Written = u32Needed;
    return true;
}
static bool ENCODE_bTimeSync(const MeshPktTimeSync_t *ptTS, uint8_t *pBuf, size_t u32BufLen, size_t *pu32Written) {
    if (u32BufLen < 1 + 4 + 1) return false;
    pBuf[0] = (uint8_t)MeshPktType_TimeSync;
    write_u32_be(&pBuf[1], ptTS->u32UtcTimestamp);
    pBuf[5] = (uint8_t)ptTS->tWakeupInterval;
    *pu32Written = 6;
    return true;
}

/* ------------------------------------------------------------------ */
/* TX helper */
static bool TX_bSendRaw(const uint8_t *pBuf, size_t u32Len) {
    LoraRadio_Packet_t tTx;
    memset(&tTx,0,sizeof(tTx));
    if (u32Len > sizeof(tTx.buffer)) return false;
    memcpy(tTx.buffer, pBuf, u32Len);
    tTx.length = (uint16_t)u32Len;
    DBG("MeshNetwork: Transmitting %s len=%u\r\n", MeshPktTypeStr[pBuf[0]], (unsigned)u32Len);
    return LORARADIO_bTxPacket(&tTx);
}

static uint32_t MESHNETWORK_u32GetTxJitterMs(void)
{
    uint32_t u32Range = MESH_TX_JITTER_MAX_MS - MESH_TX_JITTER_MIN_MS;
    return MESH_TX_JITTER_MIN_MS + (rand() % (u32Range + 1));
}
/* ------------------------------------------------------------------ */
/* Timers callbacks */
static void MESHNETWORK_vBeaconTimerCallback(TimerHandle_t xTimer) {
    (void)xTimer;
    if (!bNodeBeaconing) return;
    MeshPktDBeacon_t tBeacon;
    tBeacon.u32DreqId = u32NodeBeaconDreqId;
    tBeacon.u32DeviceId = LORARADIO_u32GetUniqueId();
    tBeacon.u16BatMv = BAT_u16GetVoltage();
    tBeacon.u8HopCount = u8NodeHopCount;
    tBeacon.i16Rssi = i16LastDreqRssi;
    tBeacon.u32BeaconMsgId = MESHNETWORK_u32GenerateGlobalMsgID();

    DBG("MeshNetwork: Sending Beacon %08X\r\n", tBeacon.u32BeaconMsgId);

    uint8_t u8Buf[64]; size_t u32Len=0;
    if (!ENCODE_bDBeacon(&tBeacon, u8Buf, sizeof(u8Buf), &u32Len)) return;
    FORWARD_vAdd(tBeacon.u32BeaconMsgId); /* avoid forwarding own beacon */
    MESHNETWORK_bSendPacket(u8Buf, u32Len);
}

static void MESHNETWORK_vPrimaryAckTimerCallback(TimerHandle_t xTimer) {
    (void)xTimer;
    MeshPktDAck_t tAck; memset(&tAck,0,sizeof(tAck));
    tAck.u32AckMsgId = MESHNETWORK_u32GenerateGlobalMsgID();
    tAck.u32DreqId = u32NodeBeaconDreqId; /* primary stores current round here */
    tAck.u32SenderId = LORARADIO_u32GetUniqueId();

    if (xSemaphoreTake(xNeighborTableMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t u8Added=0;
        for (uint16_t i=0;i<u16NeighborCount && u8Added < MESH_MAX_ACK_IDS_PER_PACKET;i++) {
            if (!tNeighborTable[i].bAcked) {
                tAck.u32AckedIds[u8Added++] = tNeighborTable[i].u32DeviceId;
                tNeighborTable[i].bAcked = true; /* mark acked */
            }
        }
        tAck.u8AckCount = u8Added;
        xSemaphoreGive(xNeighborTableMutex);

        if (tAck.u8AckCount > 0) {
            uint8_t u8Buf[128]; size_t u32Len=0;
            if (ENCODE_bDAck(&tAck, u8Buf, sizeof(u8Buf), &u32Len)) {
                FORWARD_vAdd(tAck.u32AckMsgId);
                MESHNETWORK_bSendPacket(u8Buf, u32Len);
            }
        }
    }
}

static bool MESHNETWORK_bSendPacket(const uint8_t *pBuf,
                                         size_t u32Len)
{
    if (pBuf == NULL ||
        u32Len == 0 ||
        u32Len > MESH_TX_MAX_PACKET_SIZE)
    {
        return false;
    }

    MeshTxItem_t tItem;

    memcpy(tItem.u8Buf, pBuf, u32Len);
    tItem.u16Len = (uint16_t)u32Len;

    TickType_t now = xTaskGetTickCount();
    uint32_t jitterMs = MESHNETWORK_u32GetTxJitterMs();

    tItem.tReadyTick = now + pdMS_TO_TICKS(jitterMs);

    if (xQueueSend(xMeshTxQueue, &tItem, pdMS_TO_TICKS(50)) != pdPASS)
    {
        DBG("MeshNetwork: TX queue full, dropping packet\r\n");
        return false;
    }

    DBG("MeshNetwork: Queued TX (len=%u, jitter=%lu ms)\r\n",
        (unsigned)u32Len,
		jitterMs);

    return true;
}


/* ------------------------------------------------------------------ */
/* Incoming packet handlers */
static void MESHNETWORK_vHandleDReq(const uint8_t *pBuf, size_t u32Len, int16_t s16Rssi) {
    if (u32Len < 14) return;
    uint32_t u32DreqId = read_u32_be(&pBuf[1]);
    uint32_t u32OriginId = read_u32_be(&pBuf[5]);
    uint32_t u32SenderId = read_u32_be(&pBuf[9]);
    uint8_t u8SenderHopCount = pBuf[13];

    DBG("MeshNetwork: DReq received: dreq=%08X origin=%04X sender=%04X hop=%u rssi=%d\r\n",
    		u32DreqId, u32OriginId, u32SenderId, u8SenderHopCount, s16Rssi);

    /* Capture RSSI of the received DReq */
    i16LastDreqRssi = s16Rssi;

    /* ignore our own rounds */
    if (u32OriginId == LORARADIO_u32GetUniqueId()) return;

    /* forwarder behavior */
    if (eNodeRole == NODE_ROLE_FORWARDER) {
        if (!FORWARD_bHasSeen(u32DreqId)) {
            uint8_t u8Out[32]; size_t u32OutLen=0;
            if (ENCODE_bDReq(u32DreqId, u32OriginId, LORARADIO_u32GetUniqueId(), (uint8_t)(u8SenderHopCount + 1), u8Out, sizeof(u8Out), &u32OutLen)) {
                FORWARD_vAdd(u32DreqId);
                MESHNETWORK_bSendPacket(u8Out, u32OutLen);
                DBG("MeshNetwork: DReq forwarded\r\n");
            }
        } else {
        	DBG("MeshNetwork: DReq seen before\r\n");
        }
    } else {
        /* non-forwarder: start beaconing */
    	MESHNETWORK_vStartBeaconing(u32DreqId, (uint8_t)(u8SenderHopCount + 1));
    	/* Force callback to immediately fire, then wait for the beacon timer */
//    	MESHNETWORK_vBeaconTimerCallback(xBeaconTimer);
    }
}

static void MESHNETWORK_vHandleDBeacon(const uint8_t *pBuf,
                                      size_t u32Len,
                                      int16_t s16Rssi)
{
    if (u32Len < 18) return;

    MeshPktDBeacon_t tBeacon;
    tBeacon.u32DreqId      = read_u32_be(&pBuf[1]);
    tBeacon.u32DeviceId    = read_u32_be(&pBuf[5]);
    tBeacon.u16BatMv       = read_u16_be(&pBuf[9]);
    tBeacon.u8HopCount     = pBuf[11];
    tBeacon.i16Rssi        = read_s16_be(&pBuf[12]);
    tBeacon.u32BeaconMsgId = read_u32_be(&pBuf[14]);

    DBG("MeshNetwork: Received Beacon: dev=%04X dreq=%08X hop=%u bat=%u rssi=%d id=%08X\r\n",
        tBeacon.u32DeviceId,
        tBeacon.u32DreqId,
        tBeacon.u8HopCount,
        tBeacon.u16BatMv,
        tBeacon.i16Rssi,
        tBeacon.u32BeaconMsgId);

    /* ---------------------------------------------------------
     * 1. DEDUPLICATE FIRST (all roles)
     * --------------------------------------------------------- */
    if (FORWARD_bHasSeen(tBeacon.u32BeaconMsgId))
    {
        DBG("MeshNetwork: Beacon seen before\r\n");
        tLastBeaconHeardTick = xTaskGetTickCount(); /* liveness only */
        return;
    }

    /* ---------------------------------------------------------
     * 2. MARK AS SEEN IMMEDIATELY (all roles)
     * --------------------------------------------------------- */
    FORWARD_vAdd(tBeacon.u32BeaconMsgId);

    /* ---------------------------------------------------------
     * 3. PRIMARY behavior
     * --------------------------------------------------------- */
    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY)
    {
        NEIGHBOR_vAddOrUpdate(tBeacon.u32DeviceId,
                              tBeacon.u8HopCount,
                              tBeacon.i16Rssi,
                              tBeacon.u16BatMv);

        tLastBeaconHeardTick = xTaskGetTickCount();
        return;
    }

    /* ---------------------------------------------------------
     * 4. SECONDARY behavior (forward exactly once)
     * --------------------------------------------------------- */
    tBeacon.u8HopCount++;

    uint8_t u8Buf[64];
    size_t u32TempLen = 0;

    if (!ENCODE_bDBeacon(&tBeacon, u8Buf, sizeof(u8Buf), &u32TempLen))
    {
        return;
    }

    MESHNETWORK_bSendPacket(u8Buf, u32TempLen);
    DBG("MeshNetwork: Forwarding Beacon\r\n");
}



static void MESHNETWORK_vHandleDAck(const uint8_t *pBuf, size_t u32Len, int16_t s16Rssi) {
    if (u32Len < 14) return;
    uint32_t u32AckMsgId = read_u32_be(&pBuf[1]);
    uint32_t u32DreqId = read_u32_be(&pBuf[5]);
    uint32_t u32SenderId = read_u32_be(&pBuf[9]);
    uint8_t u8AckCount = pBuf[13];
    if (u32Len < (size_t)(14 + 4*u8AckCount)) return;
    uint32_t u32Ids[MESH_MAX_ACK_IDS_PER_PACKET];
    for (uint8_t i=0;i<u8AckCount;i++) u32Ids[i] = read_u32_be(&pBuf[14 + 4*i]);

    DBG("MeshNetwork: DAck Received: ackId=%08X dreq=%08X sender=%04X count=%u\r\n", u32AckMsgId, u32DreqId, u32SenderId, u8AckCount);

    if (FORWARD_bHasSeen(u32AckMsgId))
    {
    	DBG("MeshNetwork: Ack seen before\r\n");
    	return;
    }
    FORWARD_vAdd(u32AckMsgId);

    /* if beaconing node and my id is included -> stop beaconing and become forwarder */
    uint32_t u32MyId = LORARADIO_u32GetUniqueId();
    for (uint8_t i=0;i<u8AckCount;i++) {
        if (u32Ids[i] == u32MyId) {
        	MESHNETWORK_vStopBeaconing(u32DreqId);
            break;
        }
    }

    MESHNETWORK_bSendPacket(pBuf, u32Len);
    DBG("MeshNetwork: Ack forwarded\r\n");

}

static void MESHNETWORK_vHandleTimeSync(const uint8_t *pBuf, size_t u32Len, int16_t s16Rssi) {
    if (u32Len < 6) return;
    uint32_t u32Utc = read_u32_be(&pBuf[1]);
    WakeupInterval tInterval = (WakeupInterval)pBuf[5];

    DBG("MeshNetwork: TimeSync received: utc=%u interval=%u\r\n", u32Utc, tInterval);

    if (FORWARD_bHasSeen(u32Utc))
	{
    	DBG("MeshNetwork: TimeSync seen before\r\n");
    	return;
	}
    FORWARD_vAdd(u32Utc);

	RTC_vSetUTC(u32Utc);
	MESHNETWORK_vSetWakeupInterval(tInterval);
	MESHNETWORK_vUpdatePrimaryLastSeen();
	DBG("MeshNetwork: TimeSync applied: %u interval=%u\r\n", u32Utc, tInterval);

    MESHNETWORK_bSendPacket(pBuf, u32Len);
    DBG("MeshNetwork: TimeSync forwarded\r\n");

    /* Notify DeviceDiscovery that discovery is complete */
    TaskHandle_t xAppTask = DEVICE_DISCOVERY_xGetTaskHandle();
    if (xAppTask != NULL)
    {
        xTaskNotify(
            xAppTask,
            DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
            eSetBits
        );
    }


}

/* ------------------------------------------------------------------ */
/* Parser task */
void MESHNETWORK_vParserTask(void *pvParameters) {
    (void)pvParameters;
    LoraRadio_Packet_t tRx;
    for (;;) {
        if (!LORARADIO_bRxPacket(&tRx)) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
        if (tRx.length < 1) continue;
        MeshPktType_e eType = (MeshPktType_e)tRx.buffer[0];
        switch (eType) {
            case MeshPktType_DReq: MESHNETWORK_vHandleDReq(tRx.buffer, tRx.length, tRx.rssi); break;
            case MeshPktType_DBeacon: MESHNETWORK_vHandleDBeacon(tRx.buffer, tRx.length, tRx.rssi); break;
            case MeshPktType_DAck: MESHNETWORK_vHandleDAck(tRx.buffer, tRx.length, tRx.rssi); break;
            case MeshPktType_TimeSync: MESHNETWORK_vHandleTimeSync(tRx.buffer, tRx.length, tRx.rssi); break;
            default: DBG("MeshNetwork: Unknown mesh pkt type %u\r\n", tRx.buffer[0]); break;
        }
    }
}

static void MESHNETWORK_vTxTask(void *pvParameters)
{
    MeshTxItem_t tItem;

    for (;;)
    {
        if (xQueueReceive(xMeshTxQueue, &tItem, portMAX_DELAY))
        {
            TickType_t now = xTaskGetTickCount();

            if (tItem.tReadyTick > now)
            {
                // Not ready yet → requeue and try later
                xQueueSendToBack(xMeshTxQueue, &tItem, 0);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            DBG("MeshNetwork: TX (len=%u)\r\n",
                tItem.u16Len);
            TX_bSendRaw(tItem.u8Buf, tItem.u16Len);
        }
    }
}


/* ------------------------------------------------------------------ */
/* Public API implementations */
void MESHNETWORK_vInit(void) {

    xForwardRingMutex = xSemaphoreCreateMutex(); configASSERT(xForwardRingMutex != NULL);
    xNeighborTableMutex = xSemaphoreCreateMutex(); configASSERT(xNeighborTableMutex != NULL);
    memset(&tForwardRing,0,sizeof(tForwardRing));
    memset(tNeighborTable,0,sizeof(tNeighborTable));

    u16NeighborCount = 0;
    tLastBeaconHeardTick = 0;
    bNodeBeaconing = false;

    eNodeRole = NODE_ROLE_UNKNOWN;

    /* Create TX queue */
    xMeshTxQueue = xQueueCreate(MESH_TX_QUEUE_LEN, sizeof(MeshTxItem_t));
    configASSERT(xMeshTxQueue != NULL);

    xBeaconTimer = xTimerCreate(
    		"BeaconT",
    		pdMS_TO_TICKS(MESH_BEACON_INTERVAL_MS_CFG),
			pdTRUE,
			NULL,
			MESHNETWORK_vBeaconTimerCallback);
    configASSERT(xBeaconTimer != NULL);

    xPrimaryAckTimer = xTimerCreate(
    		"AckT",
			pdMS_TO_TICKS(MESH_PRIMARY_ACK_INTERVAL_MS_CFG),
			pdTRUE,
			NULL,
			MESHNETWORK_vPrimaryAckTimerCallback);
    configASSERT(xPrimaryAckTimer != NULL);

    BaseType_t xRes = xTaskCreate(
    		MESHNETWORK_vParserTask,
			"MeshParser",
			configMINIMAL_STACK_SIZE * 5,
			NULL,
			configMAX_PRIORITIES - 3,
			&xParserTaskHandle);
    configASSERT(xRes == pdPASS);

    /* Create TX worker task */
    BaseType_t xTxRes = xTaskCreate(
            MESHNETWORK_vTxTask,
            "MeshTx",
            configMINIMAL_STACK_SIZE * 4,
            NULL,
            configMAX_PRIORITIES - 2,   /* Higher than parser */
            &xMeshTxTaskHandle);

    configASSERT(xTxRes == pdPASS);

    DBG("MeshNetwork initialized\r\n");
}

uint32_t MESHNETWORK_u32GenerateGlobalMsgID(void) {
    u16MsgCounter++;
    uint32_t u32Hi = (LORARADIO_u32GetUniqueId() & 0xFFFF);
    return (u32Hi << 16) | (uint32_t)u16MsgCounter;
}

bool MESHNETWORK_bStartDiscoveryRound(uint32_t u32DreqId)
{
    /* Start a new DReq wave (do NOT clear neighbors here) */
    tLastBeaconHeardTick = xTaskGetTickCount();
    u32NodeBeaconDreqId = u32DreqId;   /* track active round */

    uint8_t u8Out[32];
    size_t u32Len = 0;

    if (!ENCODE_bDReq(u32DreqId,
                      LORARADIO_u32GetUniqueId(),  /* origin */
                      LORARADIO_u32GetUniqueId(),  /* sender */
                      0,                           /* hop count */
                      u8Out,
                      sizeof(u8Out),
                      &u32Len)) {
        return false;
    }

    if (!MESHNETWORK_bSendPacket(u8Out, u32Len)) {
        return false;
    }

    /* Schedule immediate ACK attempt */
    MESHNETWORK_vStartPrimaryAck();

    DBG("MeshNetwork: DReq %08X sent from primary\r\n", u32DreqId);

    return true;
}

void MESHNETWORK_vSendTimeSync(uint32_t u32UtcTimestamp,
                               WakeupInterval tWakeupInterval)
{
    MeshPktTimeSync_t tTs = {
        .u32UtcTimestamp = u32UtcTimestamp,
        .tWakeupInterval = tWakeupInterval
    };

    uint8_t u8Buf[16];
    size_t u32Len = 0;

    if (!ENCODE_bTimeSync(&tTs, u8Buf, sizeof(u8Buf), &u32Len)) {
        return;
    }

    /* prevent re-forwarding our own TS */
    FORWARD_vAdd(u32UtcTimestamp);
    /* We do not jitter TimeSync sent from the primary */
    MESHNETWORK_bSendPacket(u8Buf, u32Len);

    DBG("MeshNetwork: TimeSync sent %u interval=%u\r\n",
             u32UtcTimestamp, tWakeupInterval);
}



static void MESHNETWORK_vStartBeaconing(uint32_t u32DreqId, uint8_t u8HopCount) {
    if (bNodeBeaconing && u32NodeBeaconDreqId == u32DreqId) return;
    bNodeBeaconing = true;
    u32NodeBeaconDreqId = u32DreqId;
    u8NodeHopCount = u8HopCount;
    eNodeRole = NODE_ROLE_BEACONING;
    xTimerStart(xBeaconTimer, 0);
    DBG("MeshNetwork: Start beaconing for dreq %08X\r\n", u32DreqId);
}

static void MESHNETWORK_vStopBeaconing(uint32_t u32DreqId) {
    if (!bNodeBeaconing) return;
    if (u32NodeBeaconDreqId != u32DreqId) return;
    bNodeBeaconing = false;
    u32NodeBeaconDreqId = 0;
    xTimerStop(xBeaconTimer, 0);
    eNodeRole = NODE_ROLE_FORWARDER;
    DBG("MeshNetwork: Stop beaconing, become forwarder\r\n");
}

bool MESHNETWORK_bGetDiscoveredNeighbors(MeshDiscoveredNeighbor_t *pBuffer, uint16_t u16MaxEntries, uint16_t *pu16ActualEntries) {
    if (xSemaphoreTake(xNeighborTableMutex, pdMS_TO_TICKS(200)) != pdTRUE) { *pu16ActualEntries = 0; return false; }
    uint16_t u16Count = (u16NeighborCount < u16MaxEntries) ? u16NeighborCount : u16MaxEntries;
    for (uint16_t i=0;i<u16Count;i++) {
        pBuffer[i].u32DeviceId = tNeighborTable[i].u32DeviceId;
        pBuffer[i].u8HopCount  = tNeighborTable[i].u8HopCount;
        pBuffer[i].i16Rssi     = tNeighborTable[i].i16Rssi;
        pBuffer[i].u16BatMv    = tNeighborTable[i].u16BatMv;
    }
    *pu16ActualEntries = u16Count;
    xSemaphoreGive(xNeighborTableMutex);
    return true;
}

void MESHNETWORK_vClearDiscoveredNeighbors(void)
{
	NEIGHBOR_vClearAll();
}

void MESHNETWORK_vSetWakeupInterval(WakeupInterval tNewInterval)
{
	if (tNewInterval <= WAKEUP_INTERVAL_MAX_COUNT)
	{
		tCurrentWakeupInterval = tNewInterval;
	}
}

WakeupInterval MESHNETWORK_tGetWakeupInterval(void)
{
	return tCurrentWakeupInterval;
}

uint8_t MESHNETWORK_u8GetWakeupInterval(void)
{
	return u8CurrentWakeupIntervalMin[tCurrentWakeupInterval];
}

TickType_t MESHNETWORK_tGetLastBeaconHeardTick(void)
{
	return tLastBeaconHeardTick;
}

void MESHNETWORK_vUpdatePrimaryLastSeen(void)
{
	u64LastPrimaryHeardTick = HAL_RTC_u64GetValue();
}

uint64_t MESHNETWORK_u64GetLastPrimaryHeardTick(void)
{
    return u64LastPrimaryHeardTick;
}

void MESHNETWORK_vStartPrimaryAck(void)
{
    if (xPrimaryAckTimer != NULL) {
        xTimerStart(xPrimaryAckTimer, 0);
    }
}

void MESHNETWORK_vStopPrimaryAck(void)
{
    if (xPrimaryAckTimer != NULL) {
        xTimerStop(xPrimaryAckTimer, 0);
    }
}

void MESHNETWORK_vResetNodeRole(void)
{
    bNodeBeaconing = false;
    u32NodeBeaconDreqId = 0;
    xTimerStop(xBeaconTimer, 0);
	eNodeRole = NODE_ROLE_UNKNOWN;
}


