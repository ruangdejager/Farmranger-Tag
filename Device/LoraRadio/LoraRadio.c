/*
 * LoraRadio.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */


#include "LoraRadio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h> // For memcpy, memset
#include <stdio.h>  // For printf (for debugging)

// --- PRIVATE DEFINES ---
#define LORA_TX_QUEUE_SIZE      (10)
#define LORA_RX_QUEUE_SIZE      (10)
#define LORA_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
#define LORA_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define LORA_POLLING_DELAY_MS   (10) // Delay for polling RX in transceiver task

// --- PRIVATE FREE_RTOS QUEUES ---
static QueueHandle_t xLoRaTxQueue; // Queue for packets to be sent by LoraRadio_send_packet
static QueueHandle_t xLoRaRxQueue; // Queue for raw received packets from LoraRadio_HwReceive

// --- PUBLIC FUNCTIONS ---

void LoraRadio_init(void) {
    xLoRaTxQueue = xQueueCreate(LORA_TX_QUEUE_SIZE, sizeof(LoraRadio_Packet_t));
    xLoRaRxQueue = xQueueCreate(LORA_RX_QUEUE_SIZE, sizeof(LoraRadio_Packet_t));

    if (xLoRaTxQueue == NULL || xLoRaRxQueue == NULL) {
        printf("LoraRadio: Failed to create FreeRTOS queues!\r\n");
        // Handle error appropriately, e.g., infinite loop or system reset
        while(1);
    }

    xTaskCreate(vLoRaTransceiverTask,
                "LoRaTask",
                LORA_TASK_STACK_SIZE,
                NULL,
                LORA_TASK_PRIORITY,
                NULL);

    printf("LoraRadio: Initialized FreeRTOS resources and created LoRaTask.\r\n");
}

bool LoraRadio_send_packet(const LoraRadio_Packet_t *packet, TickType_t timeout_ms) {
    if (packet->len > LORA_MAX_PACKET_SIZE) {
        printf("LoraRadio: Packet too large to send (%u bytes).\r\n", packet->len);
        return false;
    }
    if (xQueueSend(xLoRaTxQueue, packet, pdMS_TO_TICKS(timeout_ms)) == pdPASS) {
        return true;
    }
    printf("LoraRadio: Failed to queue packet for TX (queue full or timeout).\r\n");
    return false;
}

bool LoraRadio_receive_packet(LoraRadio_Packet_t *packet, TickType_t timeout_ms) {
    return xQueueReceive(xLoRaRxQueue, packet, pdMS_TO_TICKS(timeout_ms)) == pdPASS;
}

// --- FREE_RTOS TASK IMPLEMENTATION ---

void vLoRaTransceiverTask(void *pvParameters) {
    (void)pvParameters;

    LoraRadio_HwInit(); // Initialize LoRa hardware
    LoraRadio_HwSetReceiveMode(); // Start listening

    LoraRadio_Packet_t tx_packet;
    LoraRadio_Packet_t rx_packet;

    for (;;) {
        // 1. Process outgoing packets from the TX queue
        if (xQueueReceive(xLoRaTxQueue, &tx_packet, 0) == pdPASS) {
            printf("LoRaTask: Sending packet (len: %u).\r\n", tx_packet.len);
            if (LoraRadio_HwSend(tx_packet.data, tx_packet.len)) {
                printf("LoRaTask: Packet sent successfully.\r\n");
            } else {
                printf("LoRaTask: Failed to send packet.\r\n");
            }
            LoraRadio_HwSetReceiveMode(); // Return to RX mode after TX
        }

        // 2. Check for received packets from hardware
        uint16_t actual_len;
        int16_t rssi;
        int8_t snr;
        if (LoraRadio_HwReceive(rx_packet.data, LORA_MAX_PACKET_SIZE, &actual_len, &rssi, &snr)) {
            rx_packet.len = actual_len;
            rx_packet.rssi = rssi;
            rx_packet.snr = snr;
            if (xQueueSend(xLoRaRxQueue, &rx_packet, 0) != pdPASS) {
                printf("LoRaTask: RX queue full, packet dropped.\r\n");
            } else {
                printf("LoRaTask: Packet received (len: %u, RSSI: %d, SNR: %d).\r\n", rx_packet.len, rx_packet.rssi, rx_packet.snr);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LORA_POLLING_DELAY_MS)); // Small delay to yield CPU
    }
}

// --- LORA DRIVER PLACEHOLDERS (YOU MUST IMPLEMENT THESE) ---
// Replace these with actual calls to your LoRa chip driver functions.

void LoraRadio_HwInit(void) {
    printf("LoRaRadio_Hw: Initializing LoRa module hardware...\r\n");
    // Example: SX1276_init(); Configure frequency, SF, BW, CR, TX power etc.
    vTaskDelay(pdMS_TO_TICKS(100)); // Simulate init time
    printf("LoRaRadio_Hw: Module hardware initialized.\r\n");
}

bool LoraRadio_HwSend(const uint8_t *data, uint16_t len) {
    printf("LoRaRadio_Hw: Sending %u bytes over air...\r\n", len);
    // Example: SX1276_transmit(data, len);
    vTaskDelay(pdMS_TO_TICKS(500)); // Simulate airtime
    printf("LoRaRadio_Hw: Transmission complete.\r\n");
    return true; // Assume success for now
}

void LORARADIO_vEnterRxMode(uint32_t u32RxTimeout)
{

	SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
						  IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
						  IRQ_RADIO_NONE,
						  IRQ_RADIO_NONE );
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
	//  packetParams.Params.LoRa.PayloadLength = 0xFF;
	//  SUBGRF_SetPacketParams(&packetParams);
	SUBGRF_SetRx(u32RxTimeout << 6);

}

bool LoraRadio_HwReceive(uint8_t *data_buffer, uint16_t max_len, uint16_t *actual_len, int16_t *rssi, int8_t *snr) {
    // This is a polling example. In a real system, you'd likely use an interrupt
    // to signal a received packet and then read it.
    // For this example, we simulate receiving a packet occasionally.

    static uint32_t rx_counter = 0;
    if ((xTaskGetTickCount() % 5000) == 0 && rx_counter < 2) { // Simulate a packet every 5 seconds for testing
        const char *test_msg = "Hello from simulated LoRa!";
        uint16_t test_len = strlen(test_msg);
        if (test_len <= max_len) {
            memcpy(data_buffer, test_msg, test_len);
            *actual_len = test_len;
            *rssi = -60; // Example RSSI
            *snr = 8;    // Example SNR
            rx_counter++;
            return true;
        }
    }
    return false; // No packet received
}
