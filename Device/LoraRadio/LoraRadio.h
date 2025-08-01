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
    uint8_t buffer[LORA_MAX_PACKET_SIZE];
    uint8_t length;
    int16_t rssi;                                //!< The RSSI of the last packet
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

void LORARADIO_vEventCADDetected(void);

void LORARADIO_vEventCADClear(void);

uint32_t LORARADIO_u32GetRandomNumber(uint32_t max_value);

bool LORARADIO_bCarrierSense(void);

bool LORARADIO_bCarrierSenseAndWait(uint32_t maxWaitMs);

void LORARADIO_vEnterDeepSleep(void);

void LORARADIO_vWakeUp(void);

#endif /* DEVICE_LORARADIO_LORARADIO_H_ */
