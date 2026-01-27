/*
 * DeviceDiscovery.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */
#include "LoraRadio.h"
#include "DeviceDiscovery.h"
#include "MeshNetwork.h" // Interface to the Mesh Network layer
#include "Farmranger.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include <stdio.h> // For DBG (debugging)
#include <stdlib.h> // For srand

#include "dbg_log.h"
#include "platform_rtc.h"
#include "hal_rtc.h"

// --- PRIVATE DEFINES ---
#define APP_TASK_PRIORITY       	(configMAX_PRIORITIES - 3) // Lower priority for app logic
#define APP_TASK_STACK_SIZE     	(configMINIMAL_STACK_SIZE * 10)

#define LOST_PRIMARY_TIMEOUT_MIN    40      // ~24 hours

// --- PRIVATE FREE_RTOS RESOURCES ---
static EventGroupHandle_t xDiscoveryEventGroup;
DiscoveryDeviceRole tDeviceRole;
BaseType_t status;

static void DEVICE_DISCOVERY_vRecoveryMode(void);

// --- PUBLIC FUNCTIONS ---

void DEVICE_DISCOVERY_vInit(void) {

    xDiscoveryEventGroup = xEventGroupCreate();
    if (xDiscoveryEventGroup == NULL) {
        DBG("DeviceDiscovery: Failed to create FreeRTOS Event Group!\r\n");
        while(1);
    }

    status = xTaskCreate(DEVICE_DISCOVERY_vAppTask,
                "DeviceDiscoveryApp",
                APP_TASK_STACK_SIZE,
                NULL,
                APP_TASK_PRIORITY,
                NULL);
    configASSERT(status == pdPASS);

    tDeviceRole = (DiscoveryDeviceRole)HAL_GPIO_ReadPin(BSP_VERSION_BIT0_PORT, BSP_VERSION_BIT0_PIN);
    HAL_GPIO_DeInit(BSP_VERSION_BIT0_PORT, BSP_VERSION_BIT0_PIN);

    DBG("DeviceDiscovery: Initialized FreeRTOS resources and created DeviceDiscoveryAppTask.\r\n");
}

void DEVICE_DISCOVERY_vAppTask(void *pvParameters)
{
	(void)pvParameters;

	// Configuration — how many discovery rounds per wakeup?
	const uint8_t DISCOVERY_ROUNDS = 3;   // <---- Tune this based on reliability needs

#ifdef TEST_LORA_LED
LoraRadio_Packet_t myPacket;
#endif


	for (;;) {

#ifdef TEST_LORA_LED
while (1)
{
	vTaskDelay(pdMS_TO_TICKS(1000));
	myPacket.buffer[0] = 0xAA;
	myPacket.buffer[1] = 0xBB;
	myPacket.buffer[2] = 0xCC;
	myPacket.length = 3;
	if (tDeviceRole == DEVICE_PRIMARY){
		LORARADIO_bTxPacket(&myPacket);
	}
}
#endif

		// ---------------------------------------------------------------------
		// 1. Wake-up Synchronization
		// ---------------------------------------------------------------------
		DBG("DeviceDiscovery: Waiting for synchronized wake-up...\r\n");
		xEventGroupWaitBits(
			xDiscoveryEventGroup,
			DISCOVERY_WAKEUP_BIT,
			pdTRUE,     // clear on exit
			pdFALSE,
			portMAX_DELAY);

		DBG("DeviceDiscovery %X: Woke up for discovery.\r\n",
			LORARADIO_u32GetUniqueId());

		vTaskDelay(pdMS_TO_TICKS(APP_WAKEUP_BUFFER_MS));

		// ---------------------------------------------------------------------
		// 2. Clear global neighbor table BEFORE multi-round campaign
		// ---------------------------------------------------------------------
		if (tDeviceRole == DEVICE_PRIMARY)
		{
			DBG("DeviceDiscovery %X: Clearing neighbor table for new campaign.\r\n",
				LORARADIO_u32GetUniqueId());
			MESHNETWORK_vClearDiscoveredNeighbors();
		}

		// ---------------------------------------------------------------------
		// 3. Run MULTIPLE DISCOVERY ROUNDS (Primary only initiates)
		// ---------------------------------------------------------------------
		for (uint8_t round = 0; round < DISCOVERY_ROUNDS; round++)
		{
			if (tDeviceRole == DEVICE_PRIMARY)
			{

				// Generate a base DReqID so each round gets unique IDs
				uint32_t dreq_id = MESHNETWORK_u32GenerateGlobalMsgID();

				DBG("DeviceDiscovery %X: Starting Discovery Round %X (DReqID=%X)\r\n",
					LORARADIO_u32GetUniqueId(),
					round,
					dreq_id);

				MESHNETWORK_bStartDiscoveryRound(
					dreq_id,
					LORARADIO_u32GetUniqueId());
			}
			else
			{
				DBG("DeviceDiscovery %X: Secondary waiting for DReq flood in Round %X\r\n",
					LORARADIO_u32GetUniqueId(), round);
			}

			// Wait for this round's window to complete
			vTaskDelay(pdMS_TO_TICKS(APP_DISCOVERY_WINDOW_MS));
		}

		// ---------------------------------------------------------------------
		// 4. ALL ROUNDS DONE — Primary Processes the UNION of all neighbors
		// ---------------------------------------------------------------------
		DBG("DeviceDiscovery %X: Multi-round discovery campaign complete.\r\n",
			LORARADIO_u32GetUniqueId());

		if (tDeviceRole == DEVICE_PRIMARY)
		{
			MeshDiscoveredNeighbor_t discovered_neighbors_buffer[250];
			uint16_t actual_count = 0;

			if (MESHNETWORK_bGetDiscoveredNeighbors(discovered_neighbors_buffer,
													250,
													&actual_count))
			{
				DBG("DeviceDiscovery %X: Final UNION Result: %u neighbors discovered.\r\n",
					LORARADIO_u32GetUniqueId(), actual_count);

				for (uint16_t i = 0; i < actual_count; i++)
				{
					DBG("  ID:%X  Hops:%X  RSSI:%d  Bat:%d  LastSeen:%lu\r\n",
						discovered_neighbors_buffer[i].device_id,
						discovered_neighbors_buffer[i].hop_count,
						discovered_neighbors_buffer[i].rssi,
						discovered_neighbors_buffer[i].batLevel,
						discovered_neighbors_buffer[i].last_seen);
				}
			}
			else
			{
				DBG("DeviceDiscovery %X: Error retrieving final neighbor table.\r\n",
					LORARADIO_u32GetUniqueId());
			}

#ifndef ENABLE_DBG_UART
			// -----------------------------------------------------------------
			// 5. Existing PROCESSING — Logger Connection + Upload + Time Sync
			// -----------------------------------------------------------------
			if (DEVICE_DISCOVERY_DRIVER_bConnectLogger())
			{
				BSP_LED_On(LED_GREEN);
			}

			DBG("DeviceDiscovery %X: Logger connected.\r\n",
				LORARADIO_u32GetUniqueId());

			if (DEVICE_DISCOVERY_bSendDiscoveryData(discovered_neighbors_buffer,
													actual_count))
			{
				DBG("DeviceDiscovery %X: Log SUCCESS.\r\n",
					LORARADIO_u32GetUniqueId());
			}
			else
			{
				DBG("DeviceDiscovery %X: Log FAILED.\r\n",
					LORARADIO_u32GetUniqueId());
				vTaskDelay(pdMS_TO_TICKS(5000));
			}

//			vTaskDelay(pdMS_TO_TICKS(1000));

			// Timestamp sync
			uint64_t now = DEVICE_DISCOVERY_DRIVER_u64RequestTS();
			if (now > 0)
			{
				RTC_vSetUTC(now);
			}
			else
			{
				DBG("Failed to get timestamp\n");
			}

			DEVICE_DISCOVERY_DRIVER_vDisconnectLogger();
			BSP_LED_Off(LED_GREEN);
#endif

			DEVICE_DISCOVERY_vSendTS();
			vTaskDelay(pdMS_TO_TICKS(5000));

		}
		else
		{
			// Secondary nodes just chill after rounds
			vTaskDelay(pdMS_TO_TICKS(10000));
		}

		// ---------------------------------------------------------------------
		// 6. Decide sleep strategy: normal sleep, recovery mode, or stay-awake
		// ---------------------------------------------------------------------
		uint64_t now = HAL_RTC_u64GetValue();
		uint64_t last_heard = MESHNETWORK_u64GetLastPrimaryHeardTick();

		// ---- A: Enter Recovery Mode ----
		if (tDeviceRole == DEVICE_SECONDARY &&
		    (now - last_heard) > LOST_PRIMARY_TIMEOUT_MIN*60)
		{
		    DBG("Node %X: ENTERING RECOVERY MODE.\r\n", LORARADIO_u32GetUniqueId());
		    DEVICE_DISCOVERY_vRecoveryMode();
		}
		// ---- B: Deep sleep normally ----
		else
		{
		    LORARADIO_vEnterDeepSleep();
		    SYSTEM_vActivateDeepSleep();
		}


	}
}


void DEVICE_DISCOVERY_vCheckWakeupSchedule(void)
{

	// Check for wakeup condition
	if(RTC_u64GetUTC() % (((uint64_t)MESHNETWORK_u8GetWakeupInterval())*60) == 0 )
	{

		SYSTEM_vDeactivateDeepSleep();

		if (tDeviceRole == DEVICE_PRIMARY)
		{
			FARMRANGER_vUartOnWake();
		}

#ifdef ENABLE_DBG_UART
		HAL_UART_vInit();
		DBG_UART_vInit();
#endif

		LORARADIO_vWakeUp();

	    DBG("\r\n--- WAKEUP ---\r\n");
	    xEventGroupSetBits(xDiscoveryEventGroup, DISCOVERY_WAKEUP_BIT);
	}

}


void DEVICE_DISCOVERY_vSendTS(void)
{
    DBG("\r\n--- START TIMESYNC ---\r\n");
    uint32_t current_ts_id = MESHNETWORK_u32GenerateGlobalMsgID();
    MESHNETWORK_vSendTimesyncMessage(current_ts_id, RTC_u64GetUTC(), MESHNETWORK_tGetCurrentWakeupIntervalEnum());
}

DiscoveryDeviceRole DEVICE_DISCOVERY_tGetDeviceRole(void)
{
    return tDeviceRole;
}

// ======================================================================
//                       RECOVERY MODE IMPLEMENTATION
// ======================================================================
static void DEVICE_DISCOVERY_vRecoveryMode(void)
{
    DBG("RecoveryMode: Node %X LISTENING for primary.\r\n",
        LORARADIO_u32GetUniqueId());

//    LORARADIO_vSetRxContinuous();

    for (uint8_t i = 0; i < 120*60; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 minute

        uint64_t last_heard = MESHNETWORK_u64GetLastPrimaryHeardTick();

        if (last_heard != 0 &&
            (HAL_RTC_u64GetValue() - last_heard) < LOST_PRIMARY_TIMEOUT_MIN*60)
        {
            DBG("RecoveryMode: Primary detected. Exiting recovery.\r\n");
            return;
        }
    }

    DBG("RecoveryMode: No primary found.\r\n");
#warning Should we reset last seen here?
    MESHNETWORK_vUpdatePrimaryLastSeen();

    // ---- B / common: Deep sleep after this cycle ----
//    LORARADIO_vEnterDeepSleep();
//    SYSTEM_vActivateDeepSleep();

}


