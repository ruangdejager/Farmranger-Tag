/*
 * LoraRadio_Config.h
 *
 *  Created on: Jul 11, 2025
 *      Author: Ruan de Jager
 */

#ifndef DEVICE_LORARADIO_LORARADIO_CONFIG_H_
#define DEVICE_LORARADIO_LORARADIO_CONFIG_H_

#include "radio_driver.h"

#define RF_FREQUENCY                                868000000 /* Hz */
#define TX_OUTPUT_POWER                             22        /* dBm */
#define LORA_BANDWIDTH                           	LORA_BW_250;   /* kHz - This define comes from radio_driver.h*/
#define LORA_SPREADING_FACTOR                       8
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */

// Max LoRa packet size (adjust based on your LoRa module's capabilities)
#define LORA_MAX_PACKET_SIZE    (256)

#endif /* DEVICE_LORARADIO_LORARADIO_CONFIG_H_ */
