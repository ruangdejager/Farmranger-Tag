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
DeviceRole_e eDeviceRole;
BaseType_t status;

static TaskHandle_t xDeviceDiscoveryTaskHandle = NULL;

static void DEVICE_DISCOVERY_vRecoveryMode(void);
static void DEVICE_DISCOVERY_vSendTS(void);

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
                &xDeviceDiscoveryTaskHandle);
    configASSERT(status == pdPASS);

    eDeviceRole = (DeviceRole_e)HAL_GPIO_ReadPin(BSP_VERSION_BIT0_PORT, BSP_VERSION_BIT0_PIN);
    HAL_GPIO_DeInit(BSP_VERSION_BIT0_PORT, BSP_VERSION_BIT0_PIN);

    DBG("DeviceDiscovery: Initialized FreeRTOS resources and created DeviceDiscoveryAppTask.\r\n");
    if (eDeviceRole)
    {
    	DBG("DeviceDiscovery: Device Role = PRIMARY\r\n");
    } else {
    	DBG("DeviceDiscovery: Device Role = SECONDARY\r\n");
    }


}

void DEVICE_DISCOVERY_vAppTask(void *pvParameters)
{
	(void)pvParameters;

	for (;;) {

		// ---------------------------------------------------------------------
		// Waiting for synchronized wake-up...
		// ---------------------------------------------------------------------
		xEventGroupWaitBits(
			xDiscoveryEventGroup,
			DISCOVERY_WAKEUP_BIT,
			pdTRUE,     // clear on exit
			pdFALSE,
			portMAX_DELAY);

		/* Clear table for next campaign */
		MESHNETWORK_vClearDiscoveredNeighbors();

		DBG("DeviceDiscovery %X: Woke up for discovery.\r\n",
			LORARADIO_u32GetUniqueId());

		vTaskDelay(pdMS_TO_TICKS(APP_WAKEUP_BUFFER_MS));

		if (eDeviceRole == DEVICE_ROLE_PRIMARY)
		{

			/* Start discovery waves until silence */
			bool bDiscoveryFinished = false;
			DBG("DeviceDiscovery: Primary starting discovery campaign\r\n");

			while (!bDiscoveryFinished)
			{
				uint32_t u32DreqId = MESHNETWORK_u32GenerateGlobalMsgID();
				bool bBeaconSeenThisWave = false;

				MESHNETWORK_bStartDiscoveryRound(u32DreqId);

				TickType_t tWaveStartTick = xTaskGetTickCount();
				TickType_t tLastBeaconTick = MESHNETWORK_tGetLastBeaconHeardTick();

				/* ---- Monitor beacon activity for this wave ---- */
				for (;;)
				{
					vTaskDelay(pdMS_TO_TICKS(500));

					TickType_t tNow = xTaskGetTickCount();
					TickType_t tMeshLastBeacon = MESHNETWORK_tGetLastBeaconHeardTick();

					if (tMeshLastBeacon != tLastBeaconTick)
					{
						bBeaconSeenThisWave = true;
						tLastBeaconTick = tMeshLastBeacon;
					}

					/* idle window elapsed */
					if ((tNow - tLastBeaconTick) > pdMS_TO_TICKS(MESH_DISCOVERY_IDLE_MS))
					{
						break;
					}

				}


				if (!bBeaconSeenThisWave)
				{
					/* No devices responded at this distance -> discovery complete */
					bDiscoveryFinished = true;
					MESHNETWORK_vStopPrimaryAck();
				} else
				{
					DBG("DeviceDiscovery: Primary extending discovery with new DReq wave\r\n");
				}
			}

		} else
		{

			DBG("DeviceDiscovery %X: Secondary waiting for timesync.\r\n",
				LORARADIO_u32GetUniqueId());

			// Wait for this round's window to complete
		    uint32_t ulNotifyValue = 0;

		    /* Block until TimeSync arrives */
		    BaseType_t notified = xTaskNotifyWait(
		        0,                              /* don't clear on entry */
		        DEVICE_DISCOVERY_NOTIFY_TIMESYNC,
		        &ulNotifyValue,
				pdMS_TO_TICKS(APP_DISCOVERY_WINDOW_TIMEOUT_MS)
		    );

		    if (notified == pdTRUE)
		    {
		        /* Notification received */
		        if (ulNotifyValue & DEVICE_DISCOVERY_NOTIFY_TIMESYNC)
		        {
		            /* TimeSync arrived */
				    DBG("DeviceDiscovery: Secondary %04X: TimeSync received, discovery complete\r\n",
				        LORARADIO_u32GetUniqueId());
		        }
		    }
		    else
		    {
		        /* Timeout occurred */
			    DBG("DeviceDiscovery: Secondary %04X: TimeSync timed out, discovery complete\r\n",
			        LORARADIO_u32GetUniqueId());
		    }

		}

		// ---------------------------------------------------------------------
		// Primary Processes the UNION of all neighbors
		// ---------------------------------------------------------------------
		DBG("DeviceDiscovery %X: Discovery complete.\r\n",
			LORARADIO_u32GetUniqueId());

		if (eDeviceRole == DEVICE_ROLE_PRIMARY)
		{

			/* Pull union of discovered neighbors */
			MeshDiscoveredNeighbor_t tNeighbors[MESH_MAX_NEIGHBORS];
			uint16_t u16NeighborCount = 0;

			if (MESHNETWORK_bGetDiscoveredNeighbors(tNeighbors, MESH_MAX_NEIGHBORS, &u16NeighborCount))
			{
				DBG("DeviceDiscovery %X: Final UNION Result: %u neighbors discovered.\r\n",
					LORARADIO_u32GetUniqueId(), u16NeighborCount);
				for (uint16_t i = 0; i < u16NeighborCount; i++)
				{
					DBG("  ID:%X  Hops:%X  RSSI:%d  Bat:%d\r\n",
							tNeighbors[i].u32DeviceId,
							tNeighbors[i].u8HopCount,
							tNeighbors[i].i16Rssi,
							tNeighbors[i].u16BatMv);
				}
			}
			else
			{
				DBG("DeviceDiscovery %X: Error retrieving final neighbor table.\r\n",
					LORARADIO_u32GetUniqueId());
			}

#ifndef ENABLE_DBG_UART
			// -----------------------------------------------------------------
			// Logger Connection + Upload + Time Sync
			// -----------------------------------------------------------------
			if (DEVICE_DISCOVERY_DRIVER_bConnectLogger())
			{
				BSP_LED_On(LED_GREEN);
			}

			DBG("DeviceDiscovery %X: Logger connected.\r\n",
				LORARADIO_u32GetUniqueId());

			if (DEVICE_DISCOVERY_bSendDiscoveryData(tNeighbors,
					u16NeighborCount))
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

			// Timestamp sync
			uint64_t now = DEVICE_DISCOVERY_DRIVER_u64RequestTS();
			if (now > 0)
			{
				RTC_vSetUTC(now);
			}
			else
			{
				DBG("DeviceDiscovery: Failed to get timestamp\n");
			}

			DEVICE_DISCOVERY_DRIVER_vDisconnectLogger();
			BSP_LED_Off(LED_GREEN);
#endif

			/* Discovery finished: send TimeSync */
			DEVICE_DISCOVERY_vSendTS();

			vTaskDelay(pdMS_TO_TICKS(5000));

		}
		else
		{
			/* Secondary nodes just chill after rounds */
			vTaskDelay(pdMS_TO_TICKS(5000));
			/* Reset the node role for the next discovery round */
			MESHNETWORK_vResetNodeRole();
		}

		// ---------------------------------------------------------------------
		// 6. Decide sleep strategy: normal sleep, recovery mode, or stay-awake
		// ---------------------------------------------------------------------
		uint64_t now = HAL_RTC_u64GetValue();
		uint64_t last_heard = MESHNETWORK_u64GetLastPrimaryHeardTick();

		// ---- A: Enter Recovery Mode ----
		if (eDeviceRole == DEVICE_ROLE_SECONDARY &&
		    (now - last_heard) > LOST_PRIMARY_TIMEOUT_MIN*60)
		{
		    DBG("DeviceDiscovery: Node %X: ENTERING RECOVERY MODE.\r\n", LORARADIO_u32GetUniqueId());
		    DEVICE_DISCOVERY_vRecoveryMode();
		}
		// ---- B: Deep sleep normally ----
		else
		{
			DBG("DeviceDiscovery: Waiting for synchronized wake-up...\r\n");
			vTaskDelay(pdMS_TO_TICKS(100));
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

		if (eDeviceRole == DEVICE_ROLE_PRIMARY)
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


static void DEVICE_DISCOVERY_vSendTS(void)
{
    DBG("\r\n--- START TIMESYNC ---\r\n");
	MESHNETWORK_vSendTimeSync(RTC_u64GetUTC(), MESHNETWORK_tGetWakeupInterval());
}

DeviceRole_e DEVICE_DISCOVERY_eGetDeviceRole(void)
{
    return eDeviceRole;
}

// ======================================================================
//                       RECOVERY MODE IMPLEMENTATION
// ======================================================================
static void DEVICE_DISCOVERY_vRecoveryMode(void)
{
    DBG("DeviceDiscovery: RecoveryMode: Node %X LISTENING for primary.\r\n",
        LORARADIO_u32GetUniqueId());

    for (uint8_t i = 0; i < 120*60; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second

        uint64_t last_heard = MESHNETWORK_u64GetLastPrimaryHeardTick();

        if (last_heard != 0 &&
            (HAL_RTC_u64GetValue() - last_heard) < LOST_PRIMARY_TIMEOUT_MIN*60)
        {
            DBG("DeviceDiscovery: RecoveryMode: Primary detected. Exiting recovery.\r\n");
            return;
        }
    }

    DBG("DeviceDiscovery: RecoveryMode: No primary found.\r\n");
#warning Should we reset last seen here?
    MESHNETWORK_vUpdatePrimaryLastSeen();

    // ---- B / common: Deep sleep after this cycle ----
//    LORARADIO_vEnterDeepSleep();
//    SYSTEM_vActivateDeepSleep();

}

TaskHandle_t DEVICE_DISCOVERY_xGetTaskHandle(void)
{
    return xDeviceDiscoveryTaskHandle;
}



