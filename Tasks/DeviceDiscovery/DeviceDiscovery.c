/*
 * DeviceDiscovery.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */
#include "LoraRadio.h"
#include "DeviceDiscovery.h"
#include "MeshNetwork.h" // Interface to the Mesh Network layer
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include <stdio.h> // For DBG (debugging)
#include <stdlib.h> // For srand

#include "dbg_log.h"
#include "platform_rtc.h"

// --- PRIVATE DEFINES ---
#define APP_TASK_PRIORITY       (configMAX_PRIORITIES - 3) // Lower priority for app logic
#define APP_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE * 10)

// --- PRIVATE FREE_RTOS RESOURCES ---
static EventGroupHandle_t xDiscoveryEventGroup;
DiscoveryDeviceRole tDeviceRole = DEVICE_SECONDARY;
BaseType_t status;

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

    DBG("DeviceDiscovery: Initialized FreeRTOS resources and created DeviceDiscoveryAppTask.\r\n");
}

void DEVICE_DISCOVERY_vAppTask(void *pvParameters) {
    (void)pvParameters;

    uint32_t current_dreq_id = 0; // Unique ID for each discovery round

    for (;;) {

		// 1. Wait for synchronized wake-up signal
		DBG("DeviceDiscovery: Waiting for synchronized wake-up...\r\n");
		xEventGroupWaitBits(xDiscoveryEventGroup,
							DISCOVERY_WAKEUP_BIT,
							pdTRUE, // Clear bit on exit
							pdFALSE, // Don't wait for all bits
							portMAX_DELAY);


		DBG("DeviceDiscovery %u: Woke up for discovery.\r\n", LORARADIO_u32GetUniqueId());
		vTaskDelay(pdMS_TO_TICKS(APP_WAKEUP_BUFFER_MS)); // Wait for buffer time

		if (tDeviceRole == DEVICE_PRIMARY) {
			DBG("DeviceDiscovery %u: Starting discovery window.\r\n", LORARADIO_u32GetUniqueId());
		}

		// 2. Clear previous discovery results in MeshNetwork layer
		MESHNETWORK_vClearDiscoveredNeighbors();

		// 3. Initiate Discovery (only if this is a primary device)
		if (tDeviceRole == DEVICE_PRIMARY) {
			current_dreq_id = xTaskGetTickCount(); // Simple unique ID for this round
			DBG("DeviceDiscovery %u: Initiating DReq with ID %lu\r\n", LORARADIO_u32GetUniqueId(), current_dreq_id);
			MESHNETWORK_bStartDiscoveryRound(current_dreq_id, LORARADIO_u32GetUniqueId());
		}

		// 4. Wait for the discovery window to complete
		vTaskDelay(pdMS_TO_TICKS(APP_DISCOVERY_WINDOW_MS));

		// 5. Discovery Window Ended, Process Results
		DBG("DeviceDiscovery %u: Discovery window ended.\r\n", LORARADIO_u32GetUniqueId());

		if (tDeviceRole == DEVICE_PRIMARY)
		{
			MeshDiscoveredNeighbor_t discovered_neighbors_buffer[250];
			uint16_t actual_count = 0;

			if (MESHNETWORK_bGetDiscoveredNeighbors(discovered_neighbors_buffer, 250, &actual_count)) {
				DBG("DeviceDiscovery %u: Final Discovered Neighbors (%u total):\r\n", LORARADIO_u32GetUniqueId(), actual_count);
				for (uint16_t i = 0; i < actual_count; i++) {
					DBG("  ID:%u, Hops:%u, RSSI:%d, SNR:%d, LastSeen:%lu\r\n",
						   discovered_neighbors_buffer[i].device_id,
						   discovered_neighbors_buffer[i].hop_count,
						   discovered_neighbors_buffer[i].rssi,
						   discovered_neighbors_buffer[i].snr,
						   discovered_neighbors_buffer[i].last_seen);
				}
			} else {
				DBG("DeviceDiscovery %u: No neighbors discovered or error retrieving.\r\n", LORARADIO_u32GetUniqueId());
			}
		}

		// 6. Wait a moment after discovery window has complete. Execute time sync.
		vTaskDelay(pdMS_TO_TICKS(1000));

		if (tDeviceRole == DEVICE_PRIMARY)
		{
			DEVICE_DISCOVERY_vSendTS();
		}

		// 7. Wait for the time sync to complete
		vTaskDelay(pdMS_TO_TICKS(5000));

		LORARADIO_vEnterDeepSleep();
		SYSTEM_vActivateDeepSleep();

    }
}

void DEVICE_DISCOVERY_vCheckWakeupSchedule(void)
{

	// Check for wakeup condition
	if(RTC_u64GetUTC() % (MESHNETWORK_u8GetWakeupInterval()*60) == 0 )
	{

		SYSTEM_vDeactivateDeepSleep();

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
    uint32_t current_ts_id = xTaskGetTickCount();
    MESHNETWORK_vSendTimesyncMessage(current_ts_id, RTC_u64GetUTC(), MESHNETWORK_tGetCurrentWakeupIntervalEnum());
}
