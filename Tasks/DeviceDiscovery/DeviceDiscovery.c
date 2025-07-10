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
#include <stdio.h> // For printf (debugging)
#include <stdlib.h> // For srand

// --- GLOBAL VARIABLES (for MeshNetwork layer configuration) ---
// These are defined here and linked to MeshNetwork.c via extern in MeshNetwork.h
uint16_t MESH_DEVICE_ID = APP_DEVICE_ID;
uint8_t MESH_MAX_TTL = APP_MESH_MAX_TTL;
uint16_t MESH_MAX_NEIGHBORS_PER_PACKET = APP_MESH_MAX_NEIGHBORS_PER_PACKET;
uint16_t MESH_MAX_LOCAL_DISCOVERED_DEVICES = APP_MESH_MAX_LOCAL_DISCOVERED_DEVICES;
TickType_t MESH_BASE_HOP_DELAY_MS = APP_MESH_BASE_HOP_DELAY_MS;
TickType_t MESH_REPLY_JITTER_WINDOW_MS = APP_MESH_REPLY_JITTER_WINDOW_MS;
TickType_t MESH_DREQ_FLOOD_DELAY_MS = APP_MESH_DREQ_FLOOD_DELAY_MS;

// --- PRIVATE DEFINES ---
#define APP_TASK_PRIORITY       (configMAX_PRIORITIES - 3) // Lower priority for app logic
#define APP_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE * 4)

// --- PRIVATE FREE_RTOS RESOURCES ---
static EventGroupHandle_t xDiscoveryEventGroup;

// --- PUBLIC FUNCTIONS ---

void DeviceDiscovery_init(void) {
    xDiscoveryEventGroup = xEventGroupCreate();
    if (xDiscoveryEventGroup == NULL) {
        printf("DeviceDiscovery: Failed to create FreeRTOS Event Group!\r\n");
        while(1);
    }

    // Initialize random seed (e.g., using a hardware RNG or unique ID)
    srand(APP_DEVICE_ID);

    xTaskCreate(vDeviceDiscoveryAppTask,
                "DeviceDiscoveryApp",
                APP_TASK_STACK_SIZE,
                NULL,
                APP_TASK_PRIORITY,
                NULL);

    printf("DeviceDiscovery: Initialized FreeRTOS resources and created DeviceDiscoveryAppTask.\r\n");
}

void vDeviceDiscoveryAppTask(void *pvParameters) {
    (void)pvParameters;

    uint32_t current_dreq_id = 0; // Unique ID for each discovery round

    for (;;) {
        // 1. Wait for synchronized wake-up signal
        printf("DeviceDiscovery: Waiting for synchronized wake-up...\r\n");
        xEventGroupWaitBits(xDiscoveryEventGroup,
                            DISCOVERY_WAKEUP_BIT,
                            pdTRUE, // Clear bit on exit
                            pdFALSE, // Don't wait for all bits
                            portMAX_DELAY);

        printf("DeviceDiscovery %u: Woke up for discovery.\r\n", APP_DEVICE_ID);
        vTaskDelay(pdMS_TO_TICKS(APP_WAKEUP_BUFFER_MS)); // Wait for buffer time

        printf("DeviceDiscovery %u: Starting discovery window.\r\n", APP_DEVICE_ID);
        TickType_t discovery_start_time = xTaskGetTickCount();

        // 2. Clear previous discovery results in MeshNetwork layer
        MeshNetwork_clear_discovered_neighbors();

        // 3. Initiate Discovery (only if this is a primary device)
        if (APP_IS_PRIMARY_DEVICE) {
            current_dreq_id = xTaskGetTickCount(); // Simple unique ID for this round
            printf("DeviceDiscovery %u: Initiating DReq with ID %lu\r\n", APP_DEVICE_ID, current_dreq_id);
            MeshNetwork_start_discovery_round(current_dreq_id, APP_DEVICE_ID);
        }

        // 4. Wait for the discovery window to complete
        vTaskDelay(pdMS_TO_TICKS(APP_DISCOVERY_WINDOW_MS));

        // 5. Discovery Window Ended, Process Results
        printf("DeviceDiscovery %u: Discovery window ended.\r\n", APP_DEVICE_ID);

        MeshDiscoveredNeighbor_t discovered_neighbors_buffer[MESH_MAX_GLOBAL_NEIGHBORS];
        uint16_t actual_count = 0;

        if (MeshNetwork_get_discovered_neighbors(discovered_neighbors_buffer, MESH_MAX_GLOBAL_NEIGHBORS, &actual_count)) {
            printf("DeviceDiscovery %u: Final Discovered Neighbors (%u total):\r\n", APP_DEVICE_ID, actual_count);
            for (uint16_t i = 0; i < actual_count; i++) {
                printf("  ID: %u, Hops: %u, RSSI: %d, SNR: %d, Last Seen: %lu ticks\r\n",
                       discovered_neighbors_buffer[i].device_id,
                       discovered_neighbors_buffer[i].hop_count,
                       discovered_neighbors_buffer[i].rssi,
                       discovered_neighbors_buffer[i].snr,
                       discovered_neighbors_buffer[i].last_seen);
            }
        } else {
            printf("DeviceDiscovery %u: No neighbors discovered or error retrieving.\r\n", APP_DEVICE_ID);
        }

        // Go back to sleep until next synchronized wake-up
    }
}

void DeviceDiscovery_SimulateSynchronizedWakeup(void) {
    printf("\r\n--- SIMULATING SYNCHRONIZED WAKEUP ---\r\n");
    xEventGroupSetBits(xDiscoveryEventGroup, DISCOVERY_WAKEUP_BIT);
}
