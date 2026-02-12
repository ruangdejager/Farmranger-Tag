/*
 * LoraRadio.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */

#include "LoraRadio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "LoraRadio_Driver.h"
#include "dbg_log.h"

// --- PRIVATE DEFINES ---
#define LORA_TX_QUEUE_SIZE      (10)
#define LORA_RX_QUEUE_SIZE      (10)
#define LORA_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
#define LORA_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 10)
#define LORA_POLLING_DELAY_MS   (10) // Delay for polling RX in transceiver task

#define CAD_CLEAR_BIT   		(1 << 0)
#define CAD_BUSY_BIT    		(1 << 1)


// --- PRIVATE FREE_RTOS QUEUES ---
static QueueHandle_t xLoRaTxQueue; // Queue for packets to be sent by LoraRadio_send_packet
static QueueHandle_t xLoRaRxQueue; // Queue for raw received packets from LoraRadio_HwReceive

// --- PRIVATE FREE_RTOS TASK HANDLEs ---
TaskHandle_t LORARADIO_vRxTask_handle;
TaskHandle_t LORARADIO_vTxTask_handle;

TaskHandle_t LORARADIO_vCarrierSenseTaskHandle = NULL; // Assigned when function called


// --- LOCAL VARIABLE DEFINES ---
static uint8_t u8DevEUI[8];

// --- PRIVATE FUNCTION PROTOTYPES ---

static uint8_t LORARADIO_u8CRC8_Calculate(const uint8_t *data, uint16_t len);

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

    /* Here is code to test tx output power - comment out mesh and discovery inits */
//    -------------------------------------------------------------------------------------
//	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
//	// Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
//	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
//    SUBGRF_SetTxContinuousWave();
//    -------------------------------------------------------------------------------------

    DBG("\r\nDevice ID: %X\r\n", LORARADIO_u32GetUniqueId());

}

// Function that exposes a read from the rx queue
bool LORARADIO_bRxPacket(LoraRadio_Packet_t * packet) {
    return xQueueReceive(xLoRaRxQueue, packet, portMAX_DELAY) == pdPASS;
}
// Function that exposes a push to the tx queue
bool LORARADIO_bTxPacket(LoraRadio_Packet_t * packet) {
    if (packet->length > LORA_MAX_PACKET_SIZE) {
        return false;
    }
    if (xQueueSend(xLoRaTxQueue, packet, portMAX_DELAY) == pdPASS) {
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

	    uint8_t received_crc = rx_packet.buffer[rx_packet.length-1];
	    uint8_t calculated_crc = LORARADIO_u8CRC8_Calculate((uint8_t*)rx_packet.buffer, rx_packet.length-1);

	    if (calculated_crc != received_crc) {
	        DBG("LoraRadio: CRC Mismatch! Calculated: 0x%02X, Received: 0x%02X\r\n",
	               calculated_crc, received_crc);
	        // Re-enter RX before continuing
	    	LORARADIO_DRIVER_vEnterRxMode(0x00);
	        continue;
	    }
	    // Decrement packet length to remove crc index
	    rx_packet.length--;
    	if (xQueueSend(xLoRaRxQueue, &rx_packet, pdMS_TO_TICKS(100)) != pdPASS) {
    	    // handle send failure
    		// Maybe keep track of a bitmasked error code
    	} else {

    	}

    	// Now we re-enter RX listening mode
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

			uint8_t calculated_crc = LORARADIO_u8CRC8_Calculate((uint8_t*)tx_packet.buffer, tx_packet.length);
			tx_packet.buffer[tx_packet.length] = calculated_crc;
			tx_packet.length++;

			if (!LORARADIO_bCarrierSenseAndWait(5000)) // 5 seconds max wait
			{
			    DBG("Abort TX: channel busy too long\r\n");
			    LORARADIO_DRIVER_vEnterRxMode(0x00);
			    continue; // Skip this packet
			}

			if (LORARADIO_DRIVER_bTransmitPayload(tx_packet.buffer, tx_packet.length))
			{
				// We got the TX_DONE interrupt and the TX was successful
				DBG("TX IRQ: len=%d\r\n", tx_packet.length);
			} else {
				DBG("LoraRadio: Failed to transmit payload\r\n");
				// No TX_DONE interrupt, the freertos notification timeout triggered
			}

			LORARADIO_DRIVER_vEnterRxMode(0x00);

		}

	}

	vTaskDelete(NULL);
}

uint32_t LORARADIO_u32GetUniqueId (void)
{
	return (((uint32_t)u8DevEUI[1] << 24) |
	           ((uint32_t)u8DevEUI[3] << 16)) >> 16 ;
}

/**
  * @brief  Process the RX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void LORARADIO_vEventRxDone(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (LORARADIO_vRxTask_handle != NULL) {
    	vTaskNotifyGiveFromISR(LORARADIO_vRxTask_handle, &xHigherPriorityTaskWoken);
    }
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
    if (LORARADIO_vTxTask_handle != NULL) {
    	vTaskNotifyGiveFromISR(LORARADIO_vTxTask_handle, &xHigherPriorityTaskWoken);
    }
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LORARADIO_vEventCADDetected(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (LORARADIO_vCarrierSenseTaskHandle != NULL)
    {
        xTaskNotifyFromISR(LORARADIO_vCarrierSenseTaskHandle,
                           CAD_BUSY_BIT,
                           eSetBits,
                           &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void LORARADIO_vEventCADClear(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (LORARADIO_vCarrierSenseTaskHandle != NULL)
    {
        xTaskNotifyFromISR(LORARADIO_vCarrierSenseTaskHandle,
                           CAD_CLEAR_BIT,
                           eSetBits,
                           &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static uint8_t LORARADIO_u8CRC8_Calculate(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

uint32_t LORARADIO_u32GetRandomNumber(uint32_t max_value)
{
    return LORARADIO_DRIVER_u32GetRandomNumber(max_value);
}

/**
  * @brief  Use CAD to monitor channel activity
  * @param  void
  * @retval True if channel is clean, false if the channel is busy
  */
bool LORARADIO_bCarrierSense(void)
{
    LORARADIO_vCarrierSenseTaskHandle = xTaskGetCurrentTaskHandle();

    // Start CAD detection
    LORARADIO_DRIVER_vEnterCAD();

    uint32_t notifyValue = 0;

    // Wait for IRQ to notify us (CAD_DONE or CAD_DETECTED)
    if (xTaskNotifyWait(0, ULONG_MAX, &notifyValue, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        if (notifyValue & CAD_BUSY_BIT)
        {
            DBG("CAD: Channel busy\r\n");
            return false;
        }
        else if (notifyValue & CAD_CLEAR_BIT)
        {
            DBG("CAD: Channel clear\r\n");
            return true;
        }
    }

    DBG("CAD: Timeout, assuming channel busy\r\n");
    return false; // Fail safe if no response
}


bool LORARADIO_bCarrierSenseAndWait(uint32_t maxWaitMs)
{
    uint32_t startTick = xTaskGetTickCount();

    while ((xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(maxWaitMs))
    {
        if (LORARADIO_bCarrierSense())
        {
        	DBG("Channel clear\r\n");
            return true; // Channel clear, proceed
        }

        // Channel busy, apply random backoff
        uint16_t backoffMs = 100 + (rand() % 300); // random 200–600 ms
        DBG("CAD busy, retrying after %d ms\r\n", backoffMs);
        vTaskDelay(pdMS_TO_TICKS(backoffMs));
    }

    DBG("Carrier sense timed out after %lu ms\r\n", maxWaitMs);
    return false; // Timeout reached
}

void LORARADIO_vEnterDeepSleep(void)
{
	LORARADIO_DRIVER_vEnterDeepSleep();
}

void LORARADIO_vWakeUp(void)
{
	LORARADIO_DRIVER_vWakeUp();
}

