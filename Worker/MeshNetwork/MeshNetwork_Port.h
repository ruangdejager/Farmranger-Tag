/*
 * MeshNetwork_Port.h
 *
 *  Created on: Jul 25, 2025
 *      Author: Ruan de Jager
 */

#include "platform_rtc.h"
#include "LoraRadio.h"

#ifndef WORKER_MESHNETWORK_MESHNETWORK_PORT_H_
#define WORKER_MESHNETWORK_MESHNETWORK_PORT_H_

#define MESHNETWORK_vProcessUTCTimestamp(x) 	RTC_vSetUTC(x)
#define MESHNETWORK_u32GetUniqueId				LORARADIO_u32GetUniqueId
#define MESHNETWORK_bSendMessage(x)				LORARADIO_bTxPacket(x)
#define MESHNETWORK_bReceiveMessage(x)			LORARADIO_bRxPacket(x)
#define MESHNETWORK_u32GetRandomNumber(x)		LORARADIO_u32GetRandomNumber(x)

#endif /* WORKER_MESHNETWORK_MESHNETWORK_PORT_H_ */
