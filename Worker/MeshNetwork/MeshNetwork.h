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
#include "timers.h"
#include <stdint.h>
#include <stdbool.h>


#define MESH_BEACON_INTERVAL_MS            4000U
#define MESH_PRIMARY_ACK_INTERVAL_MS       6000U
#define MESH_DISCOVERY_IDLE_MS             15000U
#define FORWARD_RING_SIZE                  64
#define MESH_MAX_NEIGHBORS                 128

#define MESH_TX_JITTER_MIN_MS   		   50U
#define MESH_TX_JITTER_MAX_MS   		   2000U


/* ------------------------------------------------------------------ */
/* Packet types (wire, first byte) */
typedef enum {
    MeshPktType_Reserved = 0,
    MeshPktType_DReq     = 1,
    MeshPktType_DBeacon  = 2,
    MeshPktType_DAck     = 3,
    MeshPktType_TimeSync = 4
} MeshPktType_e;

/**
 * @brief Enum to define set wake-up intervals.
 */
typedef enum {
    WAKEUP_INTERVAL_15_MIN  = 1,
    WAKEUP_INTERVAL_30_MIN  = 2,
    WAKEUP_INTERVAL_60_MIN  = 3,
    WAKEUP_INTERVAL_120_MIN = 4,
    WAKEUP_INTERVAL_MAX_COUNT = 4
} WakeupInterval;

/* ------------------ On-wire structs (in-memory) ------------------ */
/* DReq: origin-driven discovery request */
typedef struct {
    uint32_t u32DreqId;          /* discovery round id */
    uint32_t u32OriginId;        /* primary id */
    uint32_t u32SenderId;        /* immediate sender id */
    uint8_t  u8SenderHopCount;   /* hop count from origin to sender */
} MeshPktDReq_t;

/* DBeacon: node beacon */
typedef struct {
    uint32_t u32DreqId;         /* round id */
    uint32_t u32DeviceId;       /* device origin of beacon */
    uint16_t u16BatMv;          /* battery in mV */
    uint8_t  u8HopCount;        /* hops from origin */
    int16_t  i16Rssi;           /* RSSI of received DReq */
    uint32_t u32BeaconMsgId;    /* globally unique beacon id */
} MeshPktDBeacon_t;

/* DAck: primary ack list */
#define MESH_MAX_ACK_IDS_PER_PACKET 8
typedef struct {
    uint32_t u32AckMsgId;       /* unique ack id */
    uint32_t u32DreqId;         /* round id */
    uint32_t u32SenderId;       /* primary id */
    uint8_t  u8AckCount;        /* number of ids */
    uint32_t u32AckedIds[MESH_MAX_ACK_IDS_PER_PACKET];
} MeshPktDAck_t;

/* TimeSync */
typedef struct {
    uint32_t u32UtcTimestamp;   /* unique id (timestamp) */
    WakeupInterval tWakeupInterval;
} MeshPktTimeSync_t;

/* Forward ring dedupe */
typedef struct {
    uint32_t u32Ring[FORWARD_RING_SIZE];
    uint8_t  u8Head;   /* next write index */
    uint8_t  u8Count;  /* number of valid entries */
} ForwardRing_t;

/* Primary neighbor entry */
typedef struct {
    uint32_t u32DeviceId;
    uint8_t  u8HopCount;
    int16_t i16Rssi;
    uint16_t u16BatMv;
    bool     bAcked;   /* true if primary has acked this device */
} NeighborEntry_t;

/* Node role */
typedef enum {
	NODE_ROLE_UNKNOWN = 0,
	NODE_ROLE_BEACONING = 1,
	NODE_ROLE_FORWARDER = 2
} NodeRole_e;

/* Device role used for role-detection (compile-time GPIO mapping) */
typedef enum {
	DEVICE_ROLE_UNKNOWN = 0,
	DEVICE_ROLE_PRIMARY = 1,
	DEVICE_ROLE_SECONDARY = 2
} DeviceRole_e;

/* Discovered neighbor structure returned to application */
typedef struct {
    uint32_t u32DeviceId;
    uint8_t  u8HopCount;
    int16_t i16Rssi;
    uint16_t u16BatMv;
} MeshDiscoveredNeighbor_t;


/* Mesh network core */
void MESHNETWORK_vInit(void);
void MESHNETWORK_vParserTask(void *pvParameters);

/* Start a discovery round (primary) */
bool MESHNETWORK_bStartDiscoveryRound(uint32_t u32DreqId);
void MESHNETWORK_vSendTimeSync(uint32_t u32UtcTimestamp,
                               WakeupInterval tWakeupInterval);

/* Access neighbor table (primary) */
bool MESHNETWORK_bGetDiscoveredNeighbors(MeshDiscoveredNeighbor_t *pBuffer, uint16_t u16MaxEntries, uint16_t *pu16ActualEntries);
void MESHNETWORK_vClearDiscoveredNeighbors(void);

/* Generate a global unique msg id */
uint32_t MESHNETWORK_u32GenerateGlobalMsgID(void);

/* Set/get wakeup interval */
void MESHNETWORK_vSetWakeupInterval(WakeupInterval tNewInterval);
WakeupInterval MESHNETWORK_tGetWakeupInterval(void);

uint8_t MESHNETWORK_u8GetWakeupInterval(void);

TickType_t MESHNETWORK_tGetLastBeaconHeardTick(void);

uint64_t MESHNETWORK_u64GetLastPrimaryHeardTick(void);
void MESHNETWORK_vUpdatePrimaryLastSeen(void);

void MESHNETWORK_vStartPrimaryAck(void);
void MESHNETWORK_vStopPrimaryAck(void);

void MESHNETWORK_vResetNodeRole(void);


#endif /* WORKER_MESHNETWORK_MESHNETWORK_H_ */
