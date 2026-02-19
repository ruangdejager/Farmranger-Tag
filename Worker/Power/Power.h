/*
 * Power.h
 *
 *  Created on: Feb 19, 2026
 *      Author: Ruan de Jager
 */

#ifndef WORKER_POWER_POWER_H_
#define WORKER_POWER_POWER_H_

#include "platform.h"

#define POWER_CLASS_NORMAL     (1 << 0)
#define POWER_CLASS_RECOVERY   (1 << 1)
#define POWER_CLASS_ALWAYS     (1 << 2)

void POWER_vInit(void);
void POWER_vSetModeNormal(void);
void POWER_vSetModeRecovery(void);
void POWER_vWaitForClass(EventBits_t classMask);
EventBits_t POWER_tGetState(void);

#endif /* WORKER_POWER_POWER_H_ */
