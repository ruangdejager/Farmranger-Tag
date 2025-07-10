/*
 * LoraRadio.h
 *
 *  Created on: Jul 10, 2025
 *      Author: Ruan de Jager
 */

#ifndef DEVICE_LORARADIO_LORARADIO_H_
#define DEVICE_LORARADIO_LORARADIO_H_

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>
#include <stdbool.h>

// Max LoRa packet size (adjust based on your LoRa module's capabilities)
#define LORA_MAX_PACKET_SIZE    (256)

// Structure for a raw LoRa packet (used for both TX and RX)
typedef struct {
    uint8_t  data[LORA_MAX_PACKET_SIZE];
    uint16_t len;
    int16_t  rssi; // Received Signal Strength Indicator (only valid for RX)
    int8_t   snr;  // Signal-to-Noise Ratio (only valid for RX)
} LoraRadio_Packet_t;

// Function Prototypes

/**
 * @brief Initializes the LoRa radio hardware and FreeRTOS resources for this layer.
 * Creates the LoRa transceiver task.
 */
void LoraRadio_init(void);

/**
 * @brief Sends a LoRa packet.
 * @param packet Pointer to the LoraRadio_Packet_t to send.
 * @param timeout_ms Timeout in milliseconds to wait if the TX queue is full.
 * @return true if the packet was successfully queued for transmission, false otherwise.
 */
bool LoraRadio_send_packet(const LoraRadio_Packet_t *packet, TickType_t timeout_ms);

/**
 * @brief Attempts to receive a LoRa packet from the RX queue.
 * @param packet Pointer to a LoraRadio_Packet_t structure to fill with received data.
 * @param timeout_ms Timeout in milliseconds to wait for a packet.
 * @return true if a packet was received, false otherwise.
 */
bool LoraRadio_receive_packet(LoraRadio_Packet_t *packet, TickType_t timeout_ms);

/**
 * @brief LoRa Radio Transceiver FreeRTOS Task.
 * This task manages the low-level LoRa hardware communication,
 * sending packets from the TX queue and pushing received packets to the RX queue.
 */
void vLoRaTransceiverTask(void *pvParameters);

// --- LoRa Driver Placeholders (YOU MUST IMPLEMENT THESE) ---
// These functions would interact with your specific LoRa transceiver hardware.
// They are called by vLoRaTransceiverTask.

/**
 * @brief Low-level LoRa hardware initialization.
 */
void LoraRadio_HwInit(void);

/**
 * @brief Low-level function to send a packet via LoRa.
 * @param data Pointer to the data buffer.
 * @param len Length of the data.
 * @return true on successful transmission, false otherwise.
 */
bool LoraRadio_HwSend(const uint8_t *data, uint16_t len);

/**
 * @brief Low-level function to set LoRa module to receive mode.
 */
void LORARADIO_vEnterRxMode(uint32_t u32RxTimeout);

/**
 * @brief Low-level function to check for received packet and get its info.
 * This would typically be called by the vLoRaTransceiverTask in a polling loop,
 * or triggered by an interrupt.
 * @param data_buffer Pointer to a buffer to copy received data into.
 * @param max_len Maximum length of the data_buffer.
 * @param actual_len Pointer to store the actual length of the received data.
 * @param rssi Pointer to store the RSSI of the received packet.
 * @param snr Pointer to store the SNR of the received packet.
 * @return true if a packet was received, false otherwise.
 */
bool LoraRadio_HwReceive(uint8_t *data_buffer, uint16_t max_len, uint16_t *actual_len, int16_t *rssi, int8_t *snr);

#endif /* DEVICE_LORARADIO_LORARADIO_H_ */
