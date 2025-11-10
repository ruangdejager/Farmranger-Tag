/*
 * GPS.h
 *
 *  Created on: Nov 5, 2025
 *      Author: Ruan de Jager
 */

#ifndef DEVICE_GPS_GPS_H_
#define DEVICE_GPS_GPS_H_

#include "platform.h"
#include "GPS_driver.h"

void GPS_vInit(void);
void GPS_vRxTask(void *parameters);

#endif /* DEVICE_GPS_GPS_H_ */
