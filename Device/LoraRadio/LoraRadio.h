/*
 * LoraRadio.h
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */

#ifndef DEVICE_LORARADIO_LORARADIO_H_
#define DEVICE_LORARADIO_LORARADIO_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>
#include <stdbool.h>

#include "LoraRadio_Config.h"

// Structure for a raw LoRa packet (used for both TX and RX)
typedef struct {
    uint8_t buffer[128];
    uint8_t length;
    int8_t rssi;                                //!< The RSSI of the last packet
    int8_t snr;                                 //!< The SNR of the last packet
} LoraRadio_Packet_t;

// Function Prototypes

/**
 * @brief Initializes the LoRa radio hardware and FreeRTOS resources for this layer.
 * Creates the LoRa transceiver task.
 */
void LORARADIO_vInit(void);

/**
 * @brief Attempts to receive a LoRa packet from the RX queue.
 * @param packet Pointer to a LoraRadio_Packet_t structure to fill with received data.
 * @param timeout_ms Timeout in milliseconds to wait for a packet.
 * @return true if a packet was received, false otherwise.
 */
bool LORARADIO_bRxPacket(LoraRadio_Packet_t * packet);

/**
 * @brief Sends a LoRa packet.
 * @param packet Pointer to the LoraRadio_Packet_t to send.
 * @param timeout_ms Timeout in milliseconds to wait if the TX queue is full.
 * @return true if the packet was successfully queued for transmission, false otherwise.
 */
bool LORARADIO_bTxPacket(LoraRadio_Packet_t * packet);

/**
 * @brief LoRa Radio RX FreeRTOS Task.
 * This task manages the low-level LoRa hardware reception communication,
 * pushing received packets to the RX queue.
 */
void LORARADIO_vRxTask(void *parameters);

/**
 * @brief LoRa Radio RX FreeRTOS Task.
 * This task manages the low-level LoRa hardware reception communication,
 * sending packets from the TX queue.
 */
void LORARADIO_vTxTask(void *parameters);


// These functions would interact with your specific LoRa transceiver hardware.

/**
 * @brief Low-level LoRa hardware initialization.
 */
void LORARADIO_vRadioHWInit(void);

/**
 * @brief Low-level function to send a packet via LoRa.
 * @param data Pointer to the data buffer.
 * @param len Length of the data.
 * @return true on successful transmission, false otherwise.
 */
bool LORARADIO_bRadioHWTx(uint8_t *payload, uint8_t payload_length);

/**
 * @brief Low-level function to check for received packet and get its info.
 * This would typically be called by a vLoRaTransceiverTask in a polling loop,
 * or triggered by an interrupt.
 * @param LoraRadio_Packet_t Pointer to a struct that contains the
 * Lora packet parameters.
 */
bool LORARADIO_bRadioHWRx(LoraRadio_Packet_t * rxParams);

/**
 * @brief Low-level function to set LoRa module to receive mode.
 */
void LORARADIO_vEnterHWRxMode(uint32_t u32RxTimeout);

/**
 * @brief The function return the Lora device's unique ID
 */
uint32_t LORARADIO_u32GetUniqueId (void);

/**
  * @brief  Process the RX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void LORARADIO_vEventRxDone(void);

/**
  * @brief  Process the TX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void LORARADIO_vEventTxDone(void);

/**
  * @brief  Receive data trough SUBGHZSPI peripheral
  * @param  radioIrq  interrupt pending status information
  * @retval None
  */
void LORARADIO_vRadioOnDioIrq(RadioIrqMasks_t radioIrq);

#endif /* DEVICE_LORARADIO_LORARADIO_H_ */
