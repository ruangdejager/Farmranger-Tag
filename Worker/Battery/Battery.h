/*
 * Battery.h
 *
 *  Created on: Dec 10, 2025
 *      Author: Ruan de Jager
 */

#ifndef WORKER_BATTERY_BATTERY_H_
#define WORKER_BATTERY_BATTERY_H_

#include <stdbool.h>
#include <stdint.h>

#include "coroutine.h"

void BAT_vInit(void);
void BAT_vPurgeBuffer(void);
uint16_t BAT_u16GetVoltage(void);
uint8_t BAT_u8GetPercentage(void);
uint8_t BAT_u8ConvertVoltageToPercentage(uint16_t u16Voltage);

#endif /* WORKER_BATTERY_BATTERY_H_ */
