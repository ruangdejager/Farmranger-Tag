/*
 * LoraRadio.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */

#include "dbg_log.h"


#include "LoraRadio.h"
#include <string.h>
#include <stdio.h>

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

    LORARADIO_vRadioHWInit(); // Initialize LoRa hardware
    LORARADIO_vEnterHWRxMode(0x00); // Start listening

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

		// If the ulCount is greater than one, we received an RX ISR while we weren't ready for it and
		// probably lost it
		if (ulCount > 1)
		{
			// RX ISR before done with SUBGHZ buf read
		}

		memset(&rx_packet, 0, sizeof(LoraRadio_Packet_t));
		LORARADIO_bRadioHWRx(&rx_packet);

    	if (xQueueSend(xLoRaRxQueue, &rx_packet, pdMS_TO_TICKS(100)) != pdPASS) {
    	    // handle send failure
    		DBG("RX-Queue Full\r\n");
    		// Maybe keep track of a bitmasked error code
    	} else {
    		DBG("Sending to RX-Queue\r\n");
    	}

    	// Now we re-enter RX listening mode
    	LORARADIO_vEnterHWRxMode(0x00);

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

			if (LORARADIO_bRadioHWTx(tx_packet.buffer, tx_packet.length))
			{
				// We got the TX_DONE interrupt and the TX was successful
			} else {
				// No TX_DONE interrupt, the freertos notification timeout triggered
			}

			SUBGRF_SetStandby( STDBY_RC );
			LORARADIO_vEnterHWRxMode(0x00);

		}

	}

	vTaskDelete(NULL);
}

void LORARADIO_vRadioHWInit(void) {

	PacketParams_t packetParams;

    // Initialize the hardware (SPI bus, TCXO control, RF switch)
    SUBGRF_Init(LORARADIO_vRadioOnDioIrq);

    // Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
    // "By default, the SMPS clock detection is disabled and must be enabled before enabling the SMPS." (6.1 in RM0453)
    SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
    SUBGRF_SetRegulatorMode();

    // Use the whole 256-byte buffer for both TX and RX
    SUBGRF_SetBufferBaseAddress(0x00, 0x00);

    SUBGRF_SetRfFrequency(RF_FREQUENCY);
    SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
    SUBGRF_SetStopRxTimerOnPreambleDetect(false);

    SUBGRF_SetPacketType(PACKET_TYPE_LORA);

    SUBGRF_WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
    SUBGRF_WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );

    ModulationParams_t modulationParams;
    modulationParams.PacketType = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.Bandwidth = LORA_BANDWIDTH;
    modulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)LORA_CODINGRATE;
    modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
    modulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
    SUBGRF_SetModulationParams(&modulationParams);

    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    packetParams.Params.LoRa.PayloadLength = 0xFF;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    SUBGRF_SetPacketParams(&packetParams);

    //SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

    // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
    // RegIqPolaritySetup @address 0x0736
    SUBGRF_WriteRegister( 0x0736, SUBGRF_ReadRegister( 0x0736 ) | ( 1 << 2 ) );

    SUBGRF_SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);

    SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RX_DONE,
                          IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RX_DONE,
                          IRQ_RADIO_NONE );

    HAL_SUBGHZ_vSetUniqueId(u8DevEUI);
}

bool LORARADIO_bRadioHWTx(uint8_t *payload, uint8_t payload_length)
{

	// To-Do: First we should check if the radio is in the correct state capable of going into transmission
	//
	//
	//

	SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_NONE,
						  IRQ_RADIO_NONE );
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);

	// Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));

	PacketParams_t packetParams;
	packetParams.Params.LoRa.PayloadLength = payload_length;
	SUBGRF_SetPacketParams(&packetParams);
	SUBGRF_SendPayload(payload, payload_length, 0x00);

    // Wait for notification with timeout - Received from TX_DONE interrupt
    uint32_t result = ulTaskNotifyTake(pdTRUE, 1000);

    return (result > 0);  // true if we got the notification

}

bool LORARADIO_bRadioHWRx(LoraRadio_Packet_t * rxParams) {
	PacketStatus_t packetStatus;
	// Workaround 15.3 in DS.SX1261-2.W.APP (because following RX w/ timeout sequence)
	SUBGRF_WriteRegister(0x0920, 0x00);
	SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

	SUBGRF_GetPayload((uint8_t *)rxParams->buffer, &rxParams->length, 0xFF);
	SUBGRF_GetPacketStatus(&packetStatus);

	rxParams->rssi = packetStatus.Params.LoRa.RssiPkt;
	rxParams->snr = packetStatus.Params.LoRa.SnrPkt;

	// We can later think how we decide if the reception was successfull, maybe with a CRC?
	return true;
}

void LORARADIO_vEnterHWRxMode(uint32_t u32RxTimeout)
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

/**
  * @brief  Receive data trough SUBGHZSPI peripheral
  * @param  radioIrq  interrupt pending status information
  * @retval None
  */
void LORARADIO_vRadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
  switch (radioIrq)
  {
    case IRQ_TX_DONE:
    	LORARADIO_vEventTxDone();
      break;
    case IRQ_RX_DONE:
    	LORARADIO_vEventRxDone();
      break;
    case IRQ_RX_TX_TIMEOUT:

      break;
    case IRQ_CRC_ERROR:

      break;
    default:
      break;
  }
}
