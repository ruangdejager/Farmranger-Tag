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
#define LORA_TX_QUEUE_SIZE      (24)
#define LORA_RX_QUEUE_SIZE      (8)
#define LORA_TASK_PRIORITY      (1) // Highest priority
#define LORA_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 10)
#define LORA_POLLING_DELAY_MS   (10) // Delay for polling RX in transceiver task

#define CAD_BASE_BACKOFF_MS     100
#define CAD_MAX_BACKOFF_MS      2000
#define CAD_MAX_EXPONENT        4


// --- PRIVATE FREE_RTOS QUEUES ---
static QueueHandle_t xLoRaTxQueue; // Queue for packets to be sent by LoraRadio_send_packet
static QueueHandle_t xLoRaRxQueue; // Queue for raw received packets from LoraRadio_HwReceive

// --- PRIVATE FREE_RTOS TASK HANDLEs ---
TaskHandle_t LORARADIO_vRadioTask_handle;

// --- LOCAL VARIABLE DEFINES ---
static uint8_t u8DevEUI[8];

// --- PRIVATE FUNCTION PROTOTYPES ---

static uint8_t LORARADIO_u8CRC8_Calculate(const uint8_t *data, uint16_t len);
static void LORARADIO_vNotifyFromISR(uint32_t evt);

// --- PUBLIC FUNCTIONS ---

void LORARADIO_vInit(void) {

    xLoRaTxQueue = xQueueCreate(LORA_TX_QUEUE_SIZE, sizeof(LoraRadio_Packet_t));
    xLoRaRxQueue = xQueueCreate(LORA_RX_QUEUE_SIZE, sizeof(LoraRadio_Packet_t));

    configASSERT(xLoRaTxQueue != NULL || xLoRaRxQueue != NULL); // catch creation failure

    BaseType_t status;
    status = xTaskCreate(LORARADIO_vRadioTask,
            "LoaaRadioTask",
            LORA_TASK_STACK_SIZE,
            NULL,
            LORA_TASK_PRIORITY,
			&LORARADIO_vRadioTask_handle);
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

    if (packet->length > LORA_MAX_PACKET_SIZE)
        return false;

    if (xQueueSend(xLoRaTxQueue, packet, 0) != pdPASS)
    {
        DBG("TX queue full\r\n");
        return false;
    }

    xTaskNotify(LORARADIO_vRadioTask_handle, RADIO_EVT_TX_PENDING, eSetBits);
    return true;

}

void LORARADIO_vRadioTask(void *arg)
{
    uint32_t events;
    LoraRadio_Packet_t pkt;

    LORARADIO_DRIVER_vEnterRxMode(0x00);

    for (;;)
    {
        /* Wait for IRQ or TX request */
        xTaskNotifyWait(0, ULONG_MAX, &events, portMAX_DELAY);

        /* ---------- RX DONE ---------- */
        if (events & RADIO_EVT_RX_DONE)
        {
            memset(&pkt, 0, sizeof(pkt));
            LORARADIO_DRIVER_bReceivePayload(&pkt);

            uint8_t crc_rx = pkt.buffer[pkt.length-1];
            uint8_t crc = LORARADIO_u8CRC8_Calculate(pkt.buffer, pkt.length-1);

            if (crc == crc_rx)
            {
                pkt.length--;
                xQueueSend(xLoRaRxQueue, &pkt, 0);
            }
            else
            {
                DBG("CRC mismatch\r\n");
            }

            SUBGRF_ClearIrqStatus(IRQ_RX_DONE);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- ERRORS ---------- */
        if (events & RADIO_EVT_CRC_ERROR)
        {
            DBG("CRC IRQ\r\n");
            SUBGRF_ClearIrqStatus(IRQ_CRC_ERROR);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        if (events & RADIO_EVT_HEADER_ERROR)
        {
            DBG("HEADER IRQ\r\n");
            SUBGRF_ClearIrqStatus(IRQ_HEADER_ERROR);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        if (events & RADIO_EVT_TIMEOUT)
        {
            DBG("TIMEOUT IRQ\r\n");
            SUBGRF_ClearIrqStatus(IRQ_RX_TX_TIMEOUT);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- TX DONE ---------- */
        if (events & RADIO_EVT_TX_DONE)
        {
            DBG("TX DONE\r\n");
            SUBGRF_ClearIrqStatus(IRQ_TX_DONE);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- TX REQUEST ---------- */
        if (events & RADIO_EVT_TX_PENDING ||
            uxQueueMessagesWaiting(xLoRaTxQueue))
        {
            if (xQueueReceive(xLoRaTxQueue, &pkt, 0) == pdPASS)
            {
                uint8_t crc = LORARADIO_u8CRC8_Calculate(pkt.buffer, pkt.length);
                pkt.buffer[pkt.length++] = crc;

                if (!LORARADIO_bCarrierSenseAndWait(5000))
                {
                    DBG("Abort TX busy\r\n");
                    LORARADIO_DRIVER_vEnterRxMode(0);
                    continue;
                }

                LORARADIO_DRIVER_bTransmitPayload(pkt.buffer, pkt.length);

                LORARADIO_DRIVER_vEnterRxMode(0x00);
            }
        }
    }
}

//void LORARADIO_vRxTask(void *parameters)
//{
//
//    LoraRadio_Packet_t rx_packet;
//
//	for (;;)
//	{
//		// Wait indefinitely for an RX ISR on the LORA radio
////		uint32_t ulCount = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//
//		uint32_t flags;
//		xTaskNotifyWait(0, ULONG_MAX, &flags, portMAX_DELAY);
//
//		if (flags & LORA_EVT_RX_DONE)
//		{
//		    DBG("[RADIO] RX_DONE\r\n");
//
//			// If the ulCount is greater than one, we received an RX ISR while we weren't ready for it and
//			// probably lost it
////			if (ulCount > 1)
////			{
////				// RX ISR before done with SUBGHZ buf read
////			}
//
//			memset(&rx_packet, 0, sizeof(LoraRadio_Packet_t));
//			LORARADIO_DRIVER_bReceivePayload(&rx_packet);
//
//		    uint8_t received_crc = rx_packet.buffer[rx_packet.length-1];
//		    uint8_t calculated_crc = LORARADIO_u8CRC8_Calculate((uint8_t*)rx_packet.buffer, rx_packet.length-1);
//
//		    if (calculated_crc != received_crc) {
//		        DBG("LoraRadio: CRC Mismatch! Calculated: 0x%02X, Received: 0x%02X\r\n",
//		               calculated_crc, received_crc);
//		        // Re-enter RX before continuing
//		    	LORARADIO_DRIVER_vEnterRxMode(0x00);
//		        continue;
//		    }
//		    // Decrement packet length to remove crc index
//		    rx_packet.length--;
//	    	if (xQueueSend(xLoRaRxQueue, &rx_packet, pdMS_TO_TICKS(100)) != pdPASS) {
//	    	    // handle send failure
//	    		// Maybe keep track of a bitmasked error code
//	    		DBG("LoraRadio: RX queue full, dropping packet\r\n");
//	    	} else {
//
//	    	}
//
//	    	// Now we re-enter RX listening mode
//	    	LORARADIO_DRIVER_vEnterRxMode(0x00);
//
//		}
//
//
//	}
//
//	vTaskDelete(NULL);
//}
//
//void LORARADIO_vTxTask(void *parameters)
//{
//
//	LoraRadio_Packet_t tx_packet;
//
//	for (;;)
//	{
//
//		if (xQueueReceive(xLoRaTxQueue, &tx_packet, portMAX_DELAY) == pdPASS)
//		{
//
//			uint8_t calculated_crc = LORARADIO_u8CRC8_Calculate((uint8_t*)tx_packet.buffer, tx_packet.length);
//			tx_packet.buffer[tx_packet.length] = calculated_crc;
//			tx_packet.length++;
//
//			if (!LORARADIO_bCarrierSenseAndWait(5000)) // 5 seconds max wait
//			{
//			    DBG("Abort TX: channel busy too long\r\n");
//			    LORARADIO_DRIVER_vEnterRxMode(0x00);
//			    continue; // Skip this packet
//			}
//
//			if (LORARADIO_DRIVER_bTransmitPayload(tx_packet.buffer, tx_packet.length))
//			{
//				// We got the TX_DONE interrupt and the TX was successful
//				DBG("TX IRQ: len=%d\r\n", tx_packet.length);
//			} else {
//				DBG("LoraRadio: Failed to transmit payload\r\n");
//				// No TX_DONE interrupt, the freertos notification timeout triggered
//			}
//
//			LORARADIO_DRIVER_vEnterRxMode(0x00);
//
//		}
//
//	}
//
//	vTaskDelete(NULL);
//}

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
	LORARADIO_vNotifyFromISR(RADIO_EVT_RX_DONE);
}

/**
  * @brief  Process the TX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void LORARADIO_vEventTxDone(void)
{
	LORARADIO_vNotifyFromISR(RADIO_EVT_TX_DONE);
}

void LORARADIO_vEventCrcError(void)
{
	LORARADIO_vNotifyFromISR(RADIO_EVT_CRC_ERROR);
}

void LORARADIO_vEventHeaderError(void)
{
	LORARADIO_vNotifyFromISR(RADIO_EVT_HEADER_ERROR);
}

void LORARADIO_vEventTimeout(void)
{
	LORARADIO_vNotifyFromISR(RADIO_EVT_TIMEOUT);
}

void LORARADIO_vEventCADDetected(void)
{
	LORARADIO_vNotifyFromISR(RADIO_EVT_CAD_BUSY);
}

void LORARADIO_vEventCADClear(void)
{
	LORARADIO_vNotifyFromISR(RADIO_EVT_CAD_CLEAR);
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

    // Start CAD detection
    LORARADIO_DRIVER_vEnterCAD();

    uint32_t notifyValue = 0;

    // Wait for IRQ to notify us (CAD_DONE or CAD_DETECTED)
    if (xTaskNotifyWait(0, ULONG_MAX, &notifyValue, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        if (notifyValue & RADIO_EVT_CAD_BUSY)
        {
            DBG("CAD: Channel busy\r\n");
            return false;
        }
        else if (notifyValue & RADIO_EVT_CAD_CLEAR)
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
    uint32_t failCount = 0;

    while ((xTaskGetTickCount() - startTick) < pdMS_TO_TICKS(maxWaitMs))
    {
        if (LORARADIO_bCarrierSense())
        {
            return true;    // SUCCESS → reset behavior implicitly
        }

        /* ---- Adaptive backoff ---- */

        uint32_t exponent = (failCount > CAD_MAX_EXPONENT)
                            ? CAD_MAX_EXPONENT
                            : failCount;

        uint32_t window = CAD_BASE_BACKOFF_MS << exponent;

        if (window > CAD_MAX_BACKOFF_MS)
            window = CAD_MAX_BACKOFF_MS;

        uint32_t backoffMs =
            CAD_BASE_BACKOFF_MS +
            (LORARADIO_u32GetRandomNumber(window - CAD_BASE_BACKOFF_MS + 1));

        DBG("CAD: Adaptive backoff %lu ms (fail=%lu)\r\n",
            backoffMs,
            failCount);

        vTaskDelay(pdMS_TO_TICKS(backoffMs));

        failCount++;
    }

    DBG("CAD: Carrier sense timed out after %lu ms\r\n", maxWaitMs);
    return false;
}


void LORARADIO_vEnterDeepSleep(void)
{
	LORARADIO_DRIVER_vEnterDeepSleep();
}

void LORARADIO_vWakeUp(void)
{
	LORARADIO_DRIVER_vWakeUp();
}

static void LORARADIO_vNotifyFromISR(uint32_t evt)
{
    BaseType_t hpw = pdFALSE;

    if (LORARADIO_vRadioTask_handle)
        xTaskNotifyFromISR(LORARADIO_vRadioTask_handle, evt, eSetBits, &hpw);

    portYIELD_FROM_ISR(hpw);
}
