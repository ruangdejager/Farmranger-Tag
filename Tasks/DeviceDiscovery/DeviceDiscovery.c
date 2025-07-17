/*
 * DeviceDiscovery.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */

#include "DeviceDiscovery.h"
#include "MeshNetwork.h" // Interface to the Mesh Network layer
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include <stdio.h> // For DBG (debugging)
#include <stdlib.h> // For srand

#include "dbg_log.h"
#include "LoraRadio.h"

// --- PRIVATE DEFINES ---
#define APP_TASK_PRIORITY       (configMAX_PRIORITIES - 3) // Lower priority for app logic
#define APP_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE * 10)

// --- PRIVATE FREE_RTOS RESOURCES ---
static EventGroupHandle_t xDiscoveryEventGroup;

// --- PUBLIC FUNCTIONS ---

void DeviceDiscovery_init(void) {
    xDiscoveryEventGroup = xEventGroupCreate();
    if (xDiscoveryEventGroup == NULL) {
        DBG("DeviceDiscovery: Failed to create FreeRTOS Event Group!\r\n");
        while(1);
    }

    xTaskCreate(vDeviceDiscoveryAppTask,
                "DeviceDiscoveryApp",
                APP_TASK_STACK_SIZE,
                NULL,
                APP_TASK_PRIORITY,
                NULL);

    DBG("DeviceDiscovery: Initialized FreeRTOS resources and created DeviceDiscoveryAppTask.\r\n");
}

void vDeviceDiscoveryAppTask(void *pvParameters) {
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

        DBG("DeviceDiscovery %u: Starting discovery window.\r\n", LORARADIO_u32GetUniqueId());
        TickType_t discovery_start_time = xTaskGetTickCount();

        // 2. Clear previous discovery results in MeshNetwork layer
        MESHNETWORK_vClearDiscoveredNeighbors();

        // 3. Initiate Discovery (only if this is a primary device)
        if (1/*Maybe check if this is the primary device*/) {
            current_dreq_id = xTaskGetTickCount(); // Simple unique ID for this round
            DBG("DeviceDiscovery %u: Initiating DReq with ID %lu\r\n", LORARADIO_u32GetUniqueId(), current_dreq_id);
            MESHNETWORK_bStartDiscoveryRound(current_dreq_id, LORARADIO_u32GetUniqueId());
        }

        // 4. Wait for the discovery window to complete
        vTaskDelay(pdMS_TO_TICKS(APP_DISCOVERY_WINDOW_MS));

        // 5. Discovery Window Ended, Process Results
        DBG("DeviceDiscovery %u: Discovery window ended.\r\n", LORARADIO_u32GetUniqueId());

        MeshDiscoveredNeighbor_t discovered_neighbors_buffer[250];
        uint16_t actual_count = 0;

        if (MESHNETWORK_bGetDiscoveredNeighbors(discovered_neighbors_buffer, 250, &actual_count)) {
            DBG("DeviceDiscovery %u: Final Discovered Neighbors (%u total):\r\n", LORARADIO_u32GetUniqueId(), actual_count);
            for (uint16_t i = 0; i < actual_count; i++) {
                DBG("  ID: %u, Hops: %u, RSSI: %d, SNR: %d, Last Seen: %lu ticks\r\n",
                       discovered_neighbors_buffer[i].device_id,
                       discovered_neighbors_buffer[i].hop_count,
                       discovered_neighbors_buffer[i].rssi,
                       discovered_neighbors_buffer[i].snr,
                       discovered_neighbors_buffer[i].last_seen);
            }
        } else {
            DBG("DeviceDiscovery %u: No neighbors discovered or error retrieving.\r\n", LORARADIO_u32GetUniqueId());
        }

        // Go back to sleep until next synchronized wake-up
    }
}

void DeviceDiscovery_SimulateSynchronizedWakeup(void) {
    DBG("\r\n--- SIMULATING SYNCHRONIZED WAKEUP ---\r\n");
    xEventGroupSetBits(xDiscoveryEventGroup, DISCOVERY_WAKEUP_BIT);
}
