/*
 * LoraRadio.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */

#include "LoraRadio.h"
#include <string.h>
#include <stdio.h>

#include "LoraRadio_Driver.h"
#include "dbg_log.h"

// --- PRIVATE DEFINES ---
#define LORA_TX_QUEUE_SIZE      (10)
#define LORA_RX_QUEUE_SIZE      (10)
#define LORA_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
#define LORA_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define LORA_POLLING_DELAY_MS   (10) // Delay for polling RX in transceiver task

// --- PRIVATE FREE_RTOS QUEUES ---
static QueueHandle_t xLoRaTxQueue; // Queue for packets to be sent by LoraRadio_send_packet
static QueueHandle_t xLoRaRxQueue; // Queue for raw received packets from LoraRadio_HwReceive

// --- PRIVATE FREE_RTOS TASK HANDLEs ---
TaskHandle_t LORARADIO_vRxTask_handle;
TaskHandle_t LORARADIO_vTxTask_handle;

// --- LOCAL VARIABLE DEFINES ---
static uint8_t u8DevEUI[8];

// --- PUBLIC FUNCTIONS ---

void LORARADIO_vInit(void) {

    xLoRaTxQueue = xQueueCreate(LORA_TX_QUEUE_SIZE, sizeof(LoraRadio_Packet_t));
    xLoRaRxQueue = xQueueCreate(LORA_RX_QUEUE_SIZE, sizeof(LoraRadio_Packet_t));

    configASSERT(xLoRaTxQueue != NULL || xLoRaRxQueue != NULL); // catch creation failure

    BaseType_t status;
    status = xTaskCreate(LORARADIO_vRxTask,
            "LoRaRadioRxTask",
            LORA_TASK_STACK_SIZE,
            NULL,
            LORA_TASK_PRIORITY,
			&LORARADIO_vRxTask_handle);
    configASSERT(status == pdPASS);
    status = xTaskCreate(LORARADIO_vTxTask,
            "LoRaRadioTxTask",
            LORA_TASK_STACK_SIZE,
            NULL,
            LORA_TASK_PRIORITY,
			&LORARADIO_vTxTask_handle);
    configASSERT(status == pdPASS);

    LORARADIO_DRIVER_vInit(u8DevEUI); // Initialize LoRa hardware
    LORARADIO_DRIVER_vEnterRxMode(0x00); // Start listening

}

// Function that exposes a read from the rx queue
bool LORARADIO_bRxPacket(LoraRadio_Packet_t * packet) {
    return xQueueReceive(xLoRaRxQueue, packet, pdMS_TO_TICKS(100)) == pdPASS;
}
// Function that exposes a push to the tx queue
bool LORARADIO_bTxPacket(LoraRadio_Packet_t * packet) {
    if (packet->length > LORA_MAX_PACKET_SIZE) {
        return false;
    }
    if (xQueueSend(xLoRaTxQueue, packet, pdMS_TO_TICKS(100)) == pdPASS) {
        return true;
    }
    return false;
}

void LORARADIO_vRxTask(void *parameters)
{

    LoraRadio_Packet_t rx_packet;

	for (;;)
	{
		// Wait indefinitely for an RX ISR on the LORA radio
		uint32_t ulCount = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		DBG("RX IRQ\r\n");

		// If the ulCount is greater than one, we received an RX ISR while we weren't ready for it and
		// probably lost it
		if (ulCount > 1)
		{
			// RX ISR before done with SUBGHZ buf read
		}

		memset(&rx_packet, 0, sizeof(LoraRadio_Packet_t));
		LORARADIO_DRIVER_bReceivePayload(&rx_packet);

    	if (xQueueSend(xLoRaRxQueue, &rx_packet, pdMS_TO_TICKS(100)) != pdPASS) {
    	    // handle send failure
    		// Maybe keep track of a bitmasked error code
    	} else {

    	}

    	// Now we re-enter RX listening mode
    	DBG("Entering RX\r\n");
    	LORARADIO_DRIVER_vEnterRxMode(0x00);

	}

	vTaskDelete(NULL);
}

void LORARADIO_vTxTask(void *parameters)
{

	LoraRadio_Packet_t tx_packet;

	for (;;)
	{

		if (xQueueReceive(xLoRaTxQueue, &tx_packet, portMAX_DELAY) == pdPASS)
		{

			vTaskDelay(pdMS_TO_TICKS(100));

			if (LORARADIO_DRIVER_bTransmitPayload(tx_packet.buffer, tx_packet.length))
			{
				// We got the TX_DONE interrupt and the TX was successful
				DBG("TX IRQ\r\n");
			} else {
				// No TX_DONE interrupt, the freertos notification timeout triggered
			}

			DBG("Entering RX\r\n");
			LORARADIO_DRIVER_vEnterRxMode(0x00);

		}

	}

	vTaskDelete(NULL);
}

uint32_t LORARADIO_u32GetUniqueId (void)
{
	return ((uint32_t)u8DevEUI[0] << 16) |
	           ((uint32_t)u8DevEUI[1] << 8)  |
	           ((uint32_t)u8DevEUI[2]);
}

/**
  * @brief  Process the RX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void LORARADIO_vEventRxDone(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(LORARADIO_vRxTask_handle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
  * @brief  Process the TX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void LORARADIO_vEventTxDone(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(LORARADIO_vTxTask_handle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
