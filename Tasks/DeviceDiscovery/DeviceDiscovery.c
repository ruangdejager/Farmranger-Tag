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

#include <stdio.h> // For DBG (debugging)
#include <stdlib.h> // For srand

#include "dbg_log.h"
#include "platform_rtc.h"
#include "hal_rtc.h"
#include "platform.h"
#include "Power.h"
#include "MpptChg.h"

// --- PRIVATE DEFINES ---
#define APP_TASK_PRIORITY       	(configMAX_PRIORITIES - 3) // Lower priority for app logic
#define APP_TASK_STACK_SIZE     	(configMINIMAL_STACK_SIZE * 10)

#define LOST_PRIMARY_TIMEOUT_MIN    120      // ~8 hours

// --- PRIVATE FREE_RTOS RESOURCES ---
static EventGroupHandle_t xDiscoveryEventGroup;
DeviceRole_e eDeviceRole;
BaseType_t status;

static TaskHandle_t DeviceDiscoveryAppTask_handle;
static TaskHandle_t DeviceDiscoveryWakeupTask_handle;

static void DEVICE_DISCOVERY_vRecoveryMode(void);
static void DEVICE_DISCOVERY_vSendTS(void);

void DEVICE_DISCOVERY_vCheckWakeupScheduleTask(void *pvParameters);

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
                &DeviceDiscoveryAppTask_handle);
    configASSERT(status == pdPASS);

    status = xTaskCreate(DEVICE_DISCOVERY_vCheckWakeupScheduleTask,
                "CheckWakeupScheduleTask",
				configMINIMAL_STACK_SIZE,
                NULL,
				(configMAX_PRIORITIES - 4),
				&DeviceDiscoveryWakeupTask_handle);
    configASSERT(status == pdPASS);

    DBG("DeviceDiscovery: Initialized FreeRTOS resources and created DeviceDiscoveryAppTask.\r\n");
    if (eDeviceRole == DEVICE_ROLE_PRIMARY)
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

		    xTaskNotifyStateClear(NULL);   // clear pending state
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
		}

		// ---------------------------------------------------------------------
		// 6. Decide sleep strategy: normal sleep, recovery mode, or stay-awake
		// ---------------------------------------------------------------------
		uint64_t now = HAL_RTC_u64GetValue();
		uint64_t last_heard = MESHNETWORK_u64GetLastPrimaryHeardTick();

		/* ---- Enter Recovery Mode ---- */
		if ( (eDeviceRole == DEVICE_ROLE_SECONDARY) &&
		    ((now - last_heard) > LOST_PRIMARY_TIMEOUT_MIN*60) )
		{
		    DBG("DeviceDiscovery: ENTERING RECOVERY MODE.\r\n", LORARADIO_u32GetUniqueId());
		    DEVICE_DISCOVERY_vRecoveryMode();
		}

		/* ---- Deep sleep normally ---- */
		/* Reset the node role for the next discovery round */
		MESHNETWORK_vResetNodeRole();

		if ( (eDeviceRole == DEVICE_ROLE_SECONDARY))
		{
	        DBG("\r\n\r\n*---MPPT---\r\n");
	        DBG("*Mppt chg level times (These are reset at ToD change) :\r\n");
	        DBG("*Mppt OFF = %us\r\n", MPPTCHG_u32GetOffMpptCounter());
	        DBG("*Mppt at 5mA = %us\r\n", MPPTCHG_u32Get5mAMpptCounter());
	        DBG("*Mppt at 10mA = %us\r\n", MPPTCHG_u32Get10mAMpptCounter());
	        DBG("*Mppt at 15mA = %us\r\n", MPPTCHG_u32Get15mAMpptCounter());
	        DBG("*Mppt at 20mA = %us\r\n", MPPTCHG_u32Get20mAMpptCounter());
	    	DBG("*Mppt at 25mA = %us\r\n", MPPTCHG_u32Get25mAMpptCounter());
	    	DBG("*Mppt at 30mA = %us\r\n", MPPTCHG_u32Get30mAMpptCounter());
	    	DBG("*Mppt at 35mA = %us\r\n", MPPTCHG_u32Get35mAMpptCounter());
	    	DBG("*Mppt at 40mA = %us\r\n\r\n", MPPTCHG_u32Get40mAMpptCounter());
		}

		DBG("DeviceDiscovery: Waiting for synchronized wake-up...\r\n");
		vTaskDelay(pdMS_TO_TICKS(100));
		/* Turn off radio and deep sleep... zzz... */
		LORARADIO_vEnterDeepSleep();
		SYSTEM_vActivateDeepSleep();

	}
}

void DEVICE_DISCOVERY_vCheckWakeupScheduleTask(void *pvParameters)
{

	PLATFORM_bSubscribeToHeartbeat(xTaskGetCurrentTaskHandle(),
                        HB_ALLOW_IN_RECOVERY);

	for (;;)
	{

		// Wait 1s heartbeat
		xTaskNotifyWait(
			0x00,            // Don't clear bits on entry
			0xFFFFFFFF,       // Clear all bits on exit
			NULL,
			portMAX_DELAY
		);

		// Check for wakeup condition
		if(RTC_u64GetUTC() % (((uint64_t)MESHNETWORK_u8GetWakeupInterval())*60) == 0 )
		{

			if ( (POWER_tGetState() & POWER_CLASS_NORMAL) )
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

	}

    vTaskDelete(NULL);
}

static void DEVICE_DISCOVERY_vSendTS(void)
{
    DBG("\r\n--- START TIMESYNC ---\r\n");
	MESHNETWORK_vSendTimeSync(RTC_u64GetUTC(), MESHNETWORK_tGetWakeupInterval());
}

void DEVICE_DISCOVERY_vConfigDeviceRole(void)
{
    eDeviceRole = HAL_GPIO_ReadPin(BSP_VERSION_BIT0_PORT, BSP_VERSION_BIT0_PIN) ? DEVICE_ROLE_PRIMARY : DEVICE_ROLE_SECONDARY ;
    HAL_GPIO_DeInit(BSP_VERSION_BIT0_PORT, BSP_VERSION_BIT0_PIN);
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

    DBG("DeviceDiscovery: Node %X recovery; LISTENING FOR PRIMARY.\r\n", LORARADIO_u32GetUniqueId());

    for (uint8_t i = 0; i < 120*60; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second

        uint64_t last_heard = MESHNETWORK_u64GetLastPrimaryHeardTick();

        if (last_heard != 0 &&
            (HAL_RTC_u64GetValue() - last_heard) < LOST_PRIMARY_TIMEOUT_MIN*60)
        {
        	DBG("DeviceDiscovery: Node %X recovered; PRIMARY FOUND.\r\n", LORARADIO_u32GetUniqueId());
            return;
        }
    }

    DBG("DeviceDiscovery: Node %X not recovered; NO PRIMARY FOUND.\r\n", LORARADIO_u32GetUniqueId());
    /* We reset last seen even though we didn't see to let the device sleep before trying another recovery */
    MESHNETWORK_vUpdatePrimaryLastSeen();

}

TaskHandle_t DEVICE_DISCOVERY_xGetTaskHandle(void)
{
    return DeviceDiscoveryAppTask_handle;
}



