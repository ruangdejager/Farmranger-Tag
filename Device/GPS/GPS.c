/*
 * GPS.c
 *
 *  Created on: Nov 5, 2025
 *      Author: Ruan de Jager
 */

#include "gps.h"
#include "debug_uart_output.h"

#define GPS_RX_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
#define GPS_RX_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE)

//#define GPS_MODULE_ON_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
//#define GPS_MODULE_ON_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 2)
//
//#define GPS_MODULE_OFF_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
//#define GPS_MODULE_OFF_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 2)

// --- PRIVATE FREE_RTOS TASK HANDLEs ---
TaskHandle_t GPS_vRxTask_handle;
//TaskHandle_t GPS_vModuleOnTask_handle;
//TaskHandle_t GPS_vModuleOffTask_handle;

gnss_session_t GnssSession;

gnss_sol_t GnssSolDraft;		// Struct used to build up solution as NMEA strings is parsed
gnss_sol_t GnssSol;		// Struct where solution is stored upon completed parsing
gnss_sol_t GnssSolTs;		// Thread-safe struct used to expose most recent solution to calling functions.

// Assist data expiry timestamp
uint32_t u32GnssAssistDataExpiryTs;

// Scratch buffer for conversions
char acGnssStrBuf[16];

// Buffer to build NMEA msgs before parsing
#define GPS_RX_BUF_LEN	128
char acGpsRxBuf[GPS_RX_BUF_LEN];
uint8_t u8GpsRxBufIdx;
uint32_t u32GnssRxLastTs;

//! Helper vars for SV metric calculation over multi-line GPGSV msgs
//! ToDo: Consider adding these to the gnss_sol_t struct?
uint16_t u16GpsSvSnrAvg;		// Accumulate SV SNR over multi-line GPGSV msgs
uint8_t u8GpsSvTrackingCnt;		// Count tracked SVs over multi-line GPGSV msgs

// Flag to log GPS debug info to syslog (default enabled)
// Expose through EG91_vSetRxLog()
bool GnssSyslogFlag = true;

struct _gps_s
{
	hal_uart_t 	UartHandle;
	uint8_t		byte;
	uint32_t	upTimeCounter;
} gps;

uint8_t u8HexCharToInt(char cHexChar);
//bool _bFindNext_P(char ** p1, const char * p2, bool bAdvance);
bool __bFindNext_P(char ** p1, const char * p2, bool bAdvance);

uint8_t u8CalcNmeaChecksum(char * pacStart, char * pacStop);
bool GPS_bOnRxByte(char pcRxByte);
bool GPS_bCheckNmeaMsg(char * pacNmeaMsg);

bool GPS_bParseNmeaMsgGsv(char * pNmeaMsg, gnss_sol_t * pSol);
bool GPS_bParseNmeaMsgRmc(char * pNmeaMsg, gnss_sol_t * pSol);
bool GPS_bParseNmeaMsgGga(char * pGgaMsg, gnss_sol_t * pSol);

bool GPS_bParseNmeaMsgPQTMEPE(char * pNmeaMsg, gnss_sol_t * pSol);
bool GPS_bParseNmeaMsgPubx00(char * pNmeaMsg, gnss_sol_t * pSol);

void GPS_vCoordConvertDdmToDeg(gnss_coord_deg_t * pDeg, const gnss_coord_ddm_t * pDdm);
void GPS_vCoordConvertDdmToDms(gnss_coord_dms_t * pDms, const gnss_coord_ddm_t * pDdm);

uint8_t u8SnrDbHzToPercent(uint8_t u8DbHz);

void GNSS_vOnSolution(void);

bool bAddUbxChecksum(uint8_t * pUbxFrame, uint16_t u16Len);

/**
 * Convert signal strength from dBHz to percentage
 * <p>
 * General principle: 44dB/Hz and up is 100%
 *
 * @param	u8DbHz	Signal strength [dBHz]
 * @return	Signal strength [percentage]
 */
uint8_t u8SnrDbHzToPercent(uint8_t u8DbHz)
{
	uint16_t u16Temp;

	u16Temp = ((u8DbHz > 44) ? 44 : u8DbHz);
	u16Temp = (100*u16Temp)/44;

	return (uint8_t) u16Temp;
}

bool __bFindNext_P(char ** p1, const char * p2, bool bAdvance)
{
	char * s = strstr(*p1, p2);
	if (s == NULL) return false;
	else {
		if (bAdvance) {
			s += strlen(p2);
		}
		*p1 = s;
	}

	return true;
}

void GPS_vInit(void)
{

	// Init UART
//	GPS_DRIVER_vInitGPS(&gps.UartHandle);
//	// UART interface will be enabled/disabled at EG91 powerup/powerdown
//	GPS_DRIVER_vEnableUart(&gps.UartHandle);
//	// Drive the 1.8V VCC and 0.9V VCC_RF and VCC_CORE high
//	GPS_DRIVER_vPowerEnHigh();
//	// Here we make the reset pin high impedance, the reset pin is internally pulled high.
//	HAL_GPIO_vInitInput(BSP_GPS_RESET_PORT, BSP_GPS_RESET_PIN, GPIO_NOPULL);

//	HAL_UART_vClearBuffer(&gps.UartHandle);
//	//___ Enable GPS Accuracy ___
//	uint8_t buffer[] = {"$PQTMCFGMSGRATE,W,PQTMEPE,1,2*1D\r\n"};
//	GPS_DRIVER_u8TxPutBuffer(&gps.UartHandle, buffer, sizeof(buffer));

    BaseType_t status;
    status = xTaskCreate(GPS_vRxTask,
            "GPSRxTask",
            GPS_RX_TASK_STACK_SIZE,
            NULL,
            GPS_RX_TASK_PRIORITY,
			&GPS_vRxTask_handle);
    configASSERT(status == pdPASS);
//    status = xTaskCreate(GPS_vModuleOnTask,
//            "GPSModuleOnTask",
//            GPS_MODULE_ON_TASK_STACK_SIZE,
//            NULL,
//            GPS_MODULE_ON_TASK_PRIORITY,
//			&GPS_vModuleOnTask_handle);
//    configASSERT(status == pdPASS);
//    status = xTaskCreate(GPS_vModuleOffTask,
//            "GPSModuleOffTask",
//            GPS_MODULE_OFF_TASK_STACK_SIZE,
//            NULL,
//            GPS_MODULE_OFF_TASK_PRIORITY,
//			&GPS_vModuleOffTask_handle);
//    configASSERT(status == pdPASS);



}

void GPS_vRxTask(void *parameters)
{

	for (;;)
	{
		// Wait indefinitely for an RX ISR on the LORA radio
//		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(1));
		while ( UART_bReadByte(&gps.UartHandle, &gps.byte) )
		{
//			DBG_UART_vPutByte(gps.byte);
//			u32GnssRxLastTs = HAL_TIMER_u32GetValue();
			GPS_bOnRxByte(gps.byte);
		}

	}

}

/**
 * @brief	NMEA output on-byte callback.
 *
 * Callback for the NMEA output byte stream. Wait for start and stop sequence of
 * a NMEA line, and then call NMEA parser. Once all the required NMEA lines are
 * received & parsed (& stored in a draft solution variable), GNSS_vOnSolution()
 * is called (consider moving to module tick?).
 *
 * Note that NMEA line processing does not care about line sequence, i.e. it
 * may use the RMC line from solution ts along with the GSV solution line from
 * solution (t+1)s (this has the added advantage of getting around the apparent
 * quirk of the EG91 to not generate solutions at exactly 1s intervals)
 *
 * @param	pcRxByte	Byte from NMEA output stream
 * @return	bool	True if handled OK, false otherwise
 */
bool GPS_bOnRxByte(char pcRxByte)
{
	// If empty buffer, wait for start char
	if (u8GpsRxBufIdx==0 && pcRxByte!='$') return true;
	// Pack byte in buffer
	uint8_t u8NextIdx = u8GpsRxBufIdx + 1;
	if (u8NextIdx < GPS_RX_BUF_LEN) {
		acGpsRxBuf[u8GpsRxBufIdx] = pcRxByte;
		acGpsRxBuf[u8NextIdx] = 0;
		// Send todebug terminal; tidy this up?
		DBG_UART_vPutByte((uint8_t)pcRxByte);
	}
	else {
		// RX processing buffer is full; clear & reset
		u8NextIdx = 0;
		memset(acGpsRxBuf, 0, sizeof(acGpsRxBuf));
	}
	u8GpsRxBufIdx = u8NextIdx;

	// Check for complete msg ("\r\n" indicates candidate for a complete msg)
	if ( strstr(acGpsRxBuf, "\r\n") ) {
		// Check msg integrity
		if ( GPS_bCheckNmeaMsg(acGpsRxBuf) ) {
			// Check parse msg
			if ( GPS_bParseNmeaMsgGsv(acGpsRxBuf, &GnssSolDraft) ) {
				GnssSolDraft.tNmeaMsgFlags |= GNSS_NMEA_MSG_GSV_bm;
			}
			else if ( GPS_bParseNmeaMsgRmc(acGpsRxBuf, &GnssSolDraft) ) {
				GnssSolDraft.tNmeaMsgFlags |= GNSS_NMEA_MSG_RMC_bm;
			}
#ifdef GNSS_NMEA_GGA
			else if ( GPS_bParseNmeaMsgGga(acGpsRxBuf, &GnssSolDraft) ) {
				GnssSolDraft.tNmeaMsgFlags |= GNSS_NMEA_MSG_GGA_bm;
			}
#endif

			else if (GPS_bParseNmeaMsgPQTMEPE(acGpsRxBuf, &GnssSolDraft))
			{
				GnssSolDraft.tNmeaMsgFlags |= GNSS_NMEA_MSG_PQTMEPE_bm;
			}
			else if (GPS_bParseNmeaMsgPubx00(acGpsRxBuf, &GnssSolDraft))
			{
				GnssSolDraft.tNmeaMsgFlags |= GNSS_NMEA_MSG_PUBX00_bm;
			}
			// else parse error or unknown msg (we'll clear the buffer at the end)

			// Check complete solution; note that we use only the RMC message as
			// qualifier for complete solution.
			//
			// Previously we had:
			// "if ( GnssSolDraft.tNmeaMsgFlags == (GNSS_NMEA_MSG_GSV_bm | GNSS_NMEA_MSG_RMC_bm) ) {"
			// but GSV message does not get streamed if no SVs are tracked (e.g.
			// due to bad signal), so vOnSolution() would then not fire.
			//
			// Note that this means for the nth RMC message we'll use the (n-1)th
			// GSV message; this is OK.

			//			if ( GnssSolDraft.tNmeaMsgFlags & GNSS_NMEA_MSG_RMC_bm ) {
//			if ( GnssSolDraft.tNmeaMsgFlags == (GNSS_NMEA_MSG_GSV_bm | GNSS_NMEA_MSG_RMC_bm | GNSS_NMEA_MSG_PQTMEPE_bm
//			if ( ((gps.bHasAccuracy == 1) && (GnssSolDraft.tNmeaMsgFlags == (GNSS_NMEA_MSG_GSV_bm | GNSS_NMEA_MSG_RMC_bm | GNSS_NMEA_MSG_PQTMEPE_bm
#ifdef GNSS_NMEA_GGA
//				| GNSS_NMEA_MSG_GGA_bm
#endif
//			)))
//			||
//			((gps.bHasAccuracy == 0) && (GnssSolDraft.tNmeaMsgFlags == (GNSS_NMEA_MSG_GSV_bm | GNSS_NMEA_MSG_RMC_bm
#ifdef GNSS_NMEA_GGA
//				| GNSS_NMEA_MSG_GGA_bm
#endif
//			)))
//			)

			if ( GnssSolDraft.tNmeaMsgFlags == GnssSession.tNmeaMsgFlagsBm )
			{
				// We have a complete new solution
				GNSS_vOnSolution();
			}
		}
//		else
//		{
//			volatile uint8_t count  = 10;
//			while(count--);
//		}
		// else integrity failed

		// Remove data from buffer / clear
		u8GpsRxBufIdx = 0;
		memset(acGpsRxBuf, 0, sizeof(acGpsRxBuf));
	}
	// else still receiving msg

	return true;
}

/**
 * @brief	Called periodically when a complete solution is received
 *
 * Note that it not not guaranteed to fire on every 1s mark, due to quirk with
 * EG91.
 */
void GNSS_vOnSolution(void)
{
	uint16_t u16SessionTmr;
	int32_t i32LatDelta=0, i32LongDelta=0;
	bool bGnssEstPosErrorOk;

	// Store new solution (don't care if valid or not)
	memcpy(&GnssSol, &GnssSolDraft, sizeof(gnss_sol_t));
	// Clear temp solution
	memset(&GnssSolDraft, 0, sizeof(gnss_sol_t));

	// Manage session timer (for use further down)
	u16SessionTmr = (uint16_t)((uint32_t)RTC_u64GetTicks() - GnssSession.u32StartTime);

	// Manage TTFF timer
	if (GnssSession.u16TimeToFirstFix==0)
	{
		// We choose to update timer, and then cap it; choose to do it this
		// way (rather than first check for < and then update) because this
		// will fire even if we miss a second mark (which is possible, because
		// we use the GPS NMEA stream as timebase, and not the local RTC).
		GnssSession.u16TtffTmr = u16SessionTmr;
		if (GnssSession.u16TtffTmr > GnssSession.u16TtffTimeout) GnssSession.u16TtffTmr = GnssSession.u16TtffTimeout;
	}

	// Check if the new solution is valid
	if (GnssSol.bStatusValid)
	{
		// Increment the sequential valid solution count
		if (GnssSession.bHasStableFixNow==false)
		{
			GnssSession.u8ValidFixCount++;
		}

		// Calculate deltas
		i32LatDelta = GnssSession.LastValidLat.Deg.i32MicroDeg - GnssSol.Lat.Deg.i32MicroDeg;
		i32LongDelta = GnssSession.LastValidLong.Deg.i32MicroDeg - GnssSol.Long.Deg.i32MicroDeg;
		i32LatDelta = abs((int16_t)i32LatDelta);
		i32LongDelta = abs((int16_t)i32LongDelta);

		// Check if estimated position error is small enough
		if ( GnssSol.PositionError.bHasValue )
		{
			// Position error estimate is available (e.g. LC76); criteria
			// depends on how long we've waited so far.
			if (GnssSession.u8ValidFixCount<=GNSS_SESSION_VALID_FIX_CNT_NOM)
			{
				bGnssEstPosErrorOk = (GnssSol.PositionError.u16ValueInt < GNSS_SESSION_VALID_EST_POS_ERROR_NOM);
			}
			else
			{
				bGnssEstPosErrorOk = (GnssSol.PositionError.u16ValueInt < GNSS_SESSION_VALID_EST_POS_ERROR_MAX);
			}
		}
		else
		{
			// Position error estimate is not available (e.g. EG91); use lat & long deltas
			// Note: For now we choose to eval deltas for every refix; this will result in longer
			// report times when the device is actually moving, but it will add immunity against GSM noise.
			bGnssEstPosErrorOk = ( ((int16_t)i32LatDelta <= GNSS_SESSION_VALID_LAT_LONG_DELTA_MAX) && ((int16_t)i32LongDelta <= GNSS_SESSION_VALID_LAT_LONG_DELTA_MAX) );
		}

		// Save lat & long for use in next solution
		memcpy(&GnssSession.LastValidLat, &GnssSol.Lat, sizeof(gnss_coordinate_t));
		memcpy(&GnssSession.LastValidLong, &GnssSol.Long, sizeof(gnss_coordinate_t));

		// Check for stable solution; criteria:
		// - #1 (best case) Small pos error after minimum time:
		//   GPS on at t=0, wait e.g. 5s for valid fix (t=5s), wait at least 15s
		//   for stable fix (t=20s), wait <=GNSS_SESSION_VALID_FIX_CNT_NOM (30s) for
		//   GNSS_SESSION_VALID_EST_POS_ERROR_NOM (5m) -> 20s<t<=35s
		// - #2 (2nd best case) Medium pos error before timeout:
		//   GPS on at t=0, wait e.g. 5s for valid fix (t=5s), wait at least 15s
		//   for stable fix (t=20s), wait until timeout (60s/180s) for
		//   GNSS_SESSION_VALID_EST_POS_ERROR_MAX (10m) -> 35s<t<=60s
		// - #3 (3nd best case) Large pos error on timeout:
		//   GPS on at t=0, wait e.g. 5s for valid fix (t=5s), wait at least 15s
		//   for stable fix (t=20s), wait until timeout (60s/180s) for
		//   GNSS_SESSION_VALID_EST_POS_ERROR_MAX (10m) -> t<=45s
		if (
			(GnssSession.u8ValidFixCount>=GNSS_SESSION_VALID_FIX_CNT_MIN)
			&& (
				bGnssEstPosErrorOk		// Criteria #1 & #2
				|| (GnssSession.u16TtffTmr>=GnssSession.u16TtffTimeout)		// Criteria #3
			)
		) {
			// We have a stable session solution; first one?
			if (GnssSession.u16TimeToFirstFix==0)
			{
				// Yes; store the ts
				GnssSession.u16TimeToFirstFix = u16SessionTmr;
			}
			GnssSession.bHasFirstStableFix = true;
			GnssSession.bHasStableFixNow = true;
		}
		// else minimum not yet reached, or minimum reached but not accurate enough; keep waiting
	}
	else
	{
		// Invalid solution
		if (GnssSession.u8ValidFixCount!=0) {
			DBG("gnss session stable solution LOST ERROR");
		}
		// This is not a valid solution; reset
		GnssSession.u8ValidFixCount = 0;
		GnssSession.bHasStableFixNow = false;
	}


	// Update session fix (only used by USB prog!)
//	if ( GPS_pCurrentSession()->bHasStableFixNow ) {
	if ( GnssSol.bStatusValid ) {
		GnssSession.Fix = GNSS_SESSION_FIX_OK;
		//DBG("GPS Fix OK");
	}
	else {
		if ( GnssSession.u16TtffTmr < GnssSession.u16TtffTimeout ) {
			GnssSession.Fix = GNSS_SESSION_FIX_BUSY;
			//DBG("GPS Fix Busy");
		}
		else {
			GnssSession.Fix = GNSS_SESSION_FIX_ERROR;
			//DBG("GPS Fix Error");
		}
	}

#ifndef SWITCH_GNSS_LOGGER
	// Dbg/log header
	if (
		GnssSyslogFlag		// Global enable/disable by GNSS debug flag
		&& !GnssSession.bDbgStartFlag		// Only add once at start of session
	) {
		// Format string according to type of position error estimation
		if (GnssSol.PositionError.bHasValue)
		{
			// Position error estimate available from PQTMEPE (LC76) or PUBX00 (MAX-M10S)
			DBG("\t\t\t\t\t\tgps ttff, sv, snr, fix_cnt, pos_err, [d_lat, d_long]\r\n");
		}
		else
		{
			// Position error estimate not available (EG91); use delta lat & long
			DBG("\t\t\t\t\t\tgps ttff, sv, snr, fix_cnt, [d_lat, d_long]\r\n");
		}
	}
	GnssSession.bDbgStartFlag = true;		// (Re)set session dbg start flag

	// Dbg/log periodic updates
	if (
		GnssSyslogFlag		// Global enable/disable by GNSS debug flag
		&& !GnssSession.bDbgStopFlag		// Only add until session stop condition is met (see below)
		&& (
			(u16SessionTmr % 10 == 0)		// Log every 10s while waiting for first valid fix,
			|| (GnssSession.u8ValidFixCount==1)		// OR log the first valid fix,
			|| ((u16SessionTmr % 5 == 0) && GnssSession.u8ValidFixCount)		// OR log every 5s while fixes are settling,
			|| (bool)GnssSession.u16TimeToFirstFix		// OR log the final valid fix once settled.
			)
	) {
		// Format dbg string according to type of position error estimation
		if (GnssSol.PositionError.bHasValue)
		{
			// Position error estimate available from PQTMEPE (LC76) or PUBX00 (MAX-M10S)
			DBG("gps %02u/%02u, %02u/%02u, %u, %u, %u, [%i,%i]",
				GnssSession.u16TtffTmr,
				GnssSession.u16TtffTimeout,
				GnssSol.Sv.u8SvTracking,
				GnssSol.Sv.u8SvInView,
				GnssSol.Sv.u8SnrAvgDbHz,
				GnssSession.u8ValidFixCount,
				GnssSol.PositionError.u16ValueInt,
				(int16_t)i32LatDelta,
				(int16_t)i32LongDelta
			);
		}
		else
		{
			// Position error estimate not available (EG91); use delta lat & long
			DBG("gps %02u/%02u, %02u/%02u, %u, %u, [%i,%i]",
				GnssSession.u16TtffTmr,
				GnssSession.u16TtffTimeout,
				GnssSol.Sv.u8SvTracking,
				GnssSol.Sv.u8SvInView,
				GnssSol.Sv.u8SnrAvgDbHz,
				GnssSession.u8ValidFixCount,
				(int16_t)i32LatDelta,
				(int16_t)i32LongDelta
			);
		}
	}

	// Update session log stop condition
	GnssSession.bDbgStopFlag = (bool)GnssSession.u16TimeToFirstFix	|| GnssSession.u16TtffTmr==GnssSession.u16TtffTimeout;	// On successful fix OR timeout
#else
	if (
		TIME_bIsRtcValid()		// Only if RTC valid (otherwise post-process issues)
		&& !GnssSession.bDbgStopFlag
	) {
		dbg_log_syslog_prefix_t SyslogPrefix = DBG_LOG_tGetSysLogPrefix();
		DBG_LOG_vSetSysLogPrefix(DBG_LOG_SYSLOG_PREFIX_FULL);
		DBG("gnss_log,%u,%u,%u,%u,%u,%i.%06lu,%i.%06lu,%u.%u",
			(uint8_t)((uint16_t)u16SessionTmr),
			GnssSol.bStatusValid,
			GnssSol.Sv.u8SvTracking,
			GnssSol.Sv.u8SvInView,
			GnssSol.Sv.u8SnrAvgDbHz,
			GPS_pCurrentSolution()->Long.Deg.i16Deg,
			GPS_pCurrentSolution()->Long.Deg.u32DeciMicroDeg,
			GPS_pCurrentSolution()->Lat.Deg.i16Deg,
			GPS_pCurrentSolution()->Lat.Deg.u32DeciMicroDeg,
#ifdef GNSS_NMEA_GGA
			GPS_pCurrentSolution()->Altitude.i16Value,
			GPS_pCurrentSolution()->Altitude.u8DecimalValue
#else
			0,
			0
#endif
			);
		DBG_LOG_vSetSysLogPrefix(SyslogPrefix);
	}
	// Update session log stop condition
	GnssSession.bDbgStopFlag = (u16SessionTmr >= 60);	// 60s
#endif

/*
#ifdef SWITCH_GNSS_LOGGER
	if (
		GPS_pCurrentSolution()->bStatusValid
		&& ((GPS_pCurrentSolution()->u32TimeUnix % 10) == 0)		// Log every 10s
		&& TIME_bIsRtcValid()		// Only if RTC valid (otherwise post-process issues)
	) {
		dbg_log_syslog_prefix_t SyslogPrefix = DBG_LOG_tGetSysLogPrefix();
		DBG_LOG_vSetSysLogPrefix(DBG_LOG_SYSLOG_PREFIX_FULL);
		DBG_LOG_("pos,%i.%06lu,%i.%06lu", GPS_pCurrentSolution()->Long.Deg.i16Deg,  GPS_pCurrentSolution()->Long.Deg.u32DeciMicroDeg, GPS_pCurrentSolution()->Lat.Deg.i16Deg,  GPS_pCurrentSolution()->Lat.Deg.u32DeciMicroDeg);
#ifdef GNSS_NMEA_GGA
		_DBG_LOG_(",%u.%u", GPS_pCurrentSolution()->Altitude.i16Value, GPS_pCurrentSolution()->Altitude.u8DecimalValue);
#endif
		_DBG_LOG_("\r\n");
		DBG_LOG_vSetSysLogPrefix(SyslogPrefix);
	}
#endif
*/
}

bool GPS_bParseNmeaMsgGsv(char * pNmeaMsg, gnss_sol_t * pSol)
{
	// $GPGSV,3,3,12,17,30,064,25,19,56,082,29,25,08,217,26,20,45,319,,1*6A
	// 		1    = Total number of messages of this type in this cycle
	// 		2    = Message number
	// 		3    = Total number of SVs in view
	// 		4    = SV PRN number
	// 		5    = Elevation in degrees, 90 maximum
	// 		6    = Azimuth, degrees from true north, 000 to 359
	// 		7    = SNR, 00-99 dB (null when not tracking)
	// 		8-11 = Information about second SV, same as field 4-7
	// 		12-15= Information about third SV, same as field 4-7
	// 		16-19= Information about fourth SV, same as field 4-7
	//		20 = SignalId (NMEA 4.10 and later)

// $GPGSV,4,1,14,32,36,045,34,01,,,,03,26,250,,04,12,220,,1*50
// $GPGSV,4,2,14,08,,,,10,,,,16,48,309,,18,,,,1*56
// $GPGSV,4,3,14,21,,,,25,00,135,,26,78,222,,27,10,357,,1*59
// $GPGSV,4,4,14,29,17,126,,31,50,146,,1*6D
//
// $GPGSV,4,1,14,31,46,126,31,32,24,036,35,03,26,265,,04,23,220,,1*64
// $GPGSV,4,2,14,08,,,,09,,,,10,,,,16,58,292,,1*54
// $GPGSV,4,3,14,18,,,,21,,,,25,,,,26,69,178,,1*5E
// $GPGSV,4,4,14,27,22,000,,29,11,136,,1*6B
//
// $GPGSV,4,1,14,31,46,126,31,32,24,036,35,03,26,265,,04,23,220,,1*64
// $GPGSV,4,2,14,08,,,,09,,,,10,,,,16,58,292,,1*54
// $GPGSV,4,3,14,18,,,,21,,,,25,,,,26,69,178,,1*5E
// $GPGSV,4,4,14, 27,22,000,, 29,11,136,, 1*6B

// On uBlox MAX-M10S
//	$GPGSV,1,1,01,31,41,290,17,1*5F
//	$GPGSV,3,1,11,01,10,231,,02,14,261,,10,57,011,,12,24,131,,0*60
//	$GPGSV,3,2,11,12,24,131,,23,17,026,,25,49,104,,26,17,344,,0*6F
//	$GPGSV,3,3,11,26,17,344,,28,67,260,,32,65,156,,0*5B
//	$GAGSV,3,1,11,03,30,241,,05,64,173,,06,11,088,,09,32,099,,0*7D
//	$GAGSV,3,2,11,09,32,099,,15,20,240,,16,44,032,,24,50,072,,0*7D
//	$GAGSV,3,3,11,24,50,072,,25,29,008,,31,23,132,,0*45
//	$GBGSV,1,1,00,0*77
//	$GQGSV,1,1,00,0*64

	char *p;
	uint8_t u8totalNumberOfMsgs, u8msgNumber, i, u8SvFieldCnt, u8SvInView;

	// Check result ptr
	if (pSol==NULL)	return false;

	p = pNmeaMsg;
	// Find ',' before totalNumberOfMsgs
	if ( !__bFindNext_P(&p, "GPGSV,", true) ) return false;		// Expand later to GxGSV?
	if ( !isdigit((unsigned char)*p) ) return false;
	u8totalNumberOfMsgs = *p - 48;
	// Find ',' before msgNumber
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	if ( !isdigit((unsigned char)*p) ) return false;
	u8msgNumber = *p - 48;
	// Find ',' before svInView
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Next two chars will be svInView; check & convert
	if ( !isdigit((unsigned char)*p) ) return false;
	if ( !isdigit((unsigned char)*(p+1)) ) return false;
	memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
	strncpy(acGnssStrBuf, p, 2);
	u8SvInView = atoi(acGnssStrBuf);

	// First SV SNR for solution?
	if (u8msgNumber == 1) {
		u16GpsSvSnrAvg = 0;
		u8GpsSvTrackingCnt = 0;
	}

	if (u8totalNumberOfMsgs == u8msgNumber) {
		u8SvFieldCnt = u8SvInView - ((u8msgNumber - 1) * 4);
	}
	else {
		u8SvFieldCnt = 4;
	}

	// Repeated block
	for (i=0; i<u8SvFieldCnt; i++) {
		// Find ',' before satelliteId
		if ( !__bFindNext_P(&p, ",", true) ) break;
		// Find ',' before elevation
		if ( !__bFindNext_P(&p, ",", true) ) break;
		// Find ',' before azimuth
		if ( !__bFindNext_P(&p, ",", true) ) break;
		// Find ',' before snr
		if ( !__bFindNext_P(&p, ",", true) ) break;
		if ( *p != ',' ) {		// ',' if not tracking
			if ( !isdigit((unsigned char)*p) ) return false;
			if ( !isdigit((unsigned char)*(p+1)) ) return false;
			memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
			strncpy(acGnssStrBuf, p, 2);
			u16GpsSvSnrAvg +=  atoi(acGnssStrBuf);
			u8GpsSvTrackingCnt++;
		}
	}

	// Find ',' before SignalId
	if ( !__bFindNext_P(&p, ",", true) ) return false;

	// Check for L1 signal
	if (*p != '1') return false;

	// End of repeated block; last message?
	if (u8totalNumberOfMsgs == u8msgNumber) {
		// SVs in view
		pSol->Sv.u8SvInView = u8SvInView;

		if (u8GpsSvTrackingCnt) {
			pSol->Sv.u8SnrAvgDbHz = (uint8_t)(u16GpsSvSnrAvg / (uint16_t)u8GpsSvTrackingCnt);
		}
		else {
			pSol->Sv.u8SnrAvgDbHz = 0;
			// Don't return false; doesn't register solutions for 0/x SVs!
		}
		// Percentage
		pSol->Sv.u8SnrAvgPercent = u8SnrDbHzToPercent(pSol->Sv.u8SnrAvgDbHz);
		// Tracking
		pSol->Sv.u8SvTracking = u8GpsSvTrackingCnt;

#ifdef SWITCH_GNSS_SPOOF
	// Spoof for testing
	pSol->Sv.u8SvInView = gnssSpoof[u16GnssSpoofIdx].u8SvInView;
	pSol->Sv.u8SnrAvgDbHz = gnssSpoof[u16GnssSpoofIdx].u8SnrAvgDbHz;
	pSol->Sv.u8SnrAvgPercent = u8SnrDbHzToPercent(pSol->Sv.u8SnrAvgDbHz);
	pSol->Sv.u8SvTracking = gnssSpoof[u16GnssSpoofIdx].u8SvTracking;
#endif

		return true;
	}

	return false;
}


/**
 * @brief	Get UTC time from current GPS solution
 * <p>
 * Note that GPS solution must be valid before UTC time is considered valid;
 * it was observed that valid UTC time may be returned before the entire
 * solution is valid.
 *
 * @return	calendar_date	Pointer to local struct with UTC time
 */
// calendar_date * _GPS_pUtcTime(void)
// {
// 	return &GnssSol.TimeDateUtc;
// }


bool GPS_bParseNmeaMsgRmc(char * pNmeaMsg, gnss_sol_t * pSol)
{
	// $GPRMC,123836.00,A,3357.836846,S,01850.297442,E,0.0,173.6,261121,25.0,W,A,V*79
	// $GPRMC,,V,,,,,,,,,,N*53

	char *p;
	uint8_t u8LetLongDecimalCnt, i;

	// Check result ptr
	if (pSol==NULL)	return false;

	p = pNmeaMsg;
	// Find ',' before utcTime
	if ( !__bFindNext_P(&p, "RMC,", true) ) return false;
	// --- UTC Time ---
	// Next 0/6 chars will be hhmmss
	if (*p != ',') {
		for (i=0; i<6; i++) {
			if ( !isdigit((unsigned char)*(p+i)) ) return false;
		}
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, (p+0), 2);
		pSol->TimeDateUtc.hour = atoi(acGnssStrBuf);
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, (p+2), 2);
		pSol->TimeDateUtc.minute = atoi(acGnssStrBuf);
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, (p+4), 2);
		pSol->TimeDateUtc.second = atoi(acGnssStrBuf);
	}
	// Find ',' before Status
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// --- Status ---
	// Next char will be status (always present)
	if ( *p == ',' );
	else if ( *p == 'A' ) pSol->bStatusValid = true;
	else if ( *p == 'V' ) pSol->bStatusValid = false;
	else return false;
	// Find ',' before latitude
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// --- Latitude --- (DDM)
	// Next 0/? chars will be ddmm.mmm...
	if (*p != ',') {
		for (i=0; i<4; i++) {
			if ( !isdigit((unsigned char)*(p+i)) ) return false;
		}
		if ( *(p+4) != '.' ) return false;
		u8LetLongDecimalCnt = 0;
		for (i=0; i<6; i++) {
			if ( isdigit((unsigned char)*(p+5+i)) ) u8LetLongDecimalCnt++;
		}
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, (p+0), 2);
		pSol->Lat.Ddm.i16Degrees = atoi(acGnssStrBuf);
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, (p+2), 2);
		pSol->Lat.Ddm.u8Minutes = atoi(acGnssStrBuf);
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		memset(acGnssStrBuf, '0', 6);
		strncpy(acGnssStrBuf, (p+5), u8LetLongDecimalCnt);
		pSol->Lat.Ddm.u32MicroMinutes = atol(acGnssStrBuf);
	}
	// Find ',' before NSIndicator
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	if ( *p == ',' );
	else if ( *p == 'N' );
	else if ( *p == 'S' ) pSol->Lat.Ddm.i16Degrees = -pSol->Lat.Ddm.i16Degrees;
	else return false;
#ifdef SWITCH_GNSS_SPOOF
	// ...,3356.205400,S,...
	pSol->bStatusValid = true;
	pSol->Lat.Ddm.i16Degrees = gnssSpoof[u16GnssSpoofIdx].i16LatDegrees;
	pSol->Lat.Ddm.u8Minutes = gnssSpoof[u16GnssSpoofIdx].u8LatMinutes;
	pSol->Lat.Ddm.u32MicroMinutes = gnssSpoof[u16GnssSpoofIdx].u32LatMicroMinutes;
#endif
	// Convert to other formats (after confirming the sign)
	GPS_vCoordConvertDdmToDms(&pSol->Lat.Dms, &pSol->Lat.Ddm);
	GPS_vCoordConvertDdmToDeg(&pSol->Lat.Deg, &pSol->Lat.Ddm);
	// Find ',' before longitude
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// --- Longitude --- (DDM)
	// Next 0/? chars will be dddmm.mmm...
	if (*p != ',') {
		for (i=0; i<5; i++) {
			if ( !isdigit((unsigned char)*(p+i)) ) return false;
		}
		if ( *(p+5) != '.' ) return false;
		u8LetLongDecimalCnt = 0;
		for (i=0; i<6; i++) {
			if ( isdigit((unsigned char)*(p+6+i)) ) u8LetLongDecimalCnt++;
		}
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, (p+0), 3);
		pSol->Long.Ddm.i16Degrees = atoi(acGnssStrBuf);
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, (p+3), 2);
		pSol->Long.Ddm.u8Minutes = atoi(acGnssStrBuf);
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		memset(acGnssStrBuf, '0', 6);
		strncpy(acGnssStrBuf, (p+6), u8LetLongDecimalCnt);
		pSol->Long.Ddm.u32MicroMinutes = atol(acGnssStrBuf);
	}
	// Find ',' before EWIndicator
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	if ( *p == ',' );
	else if ( *p == 'E' );
	else if ( *p == 'W' ) pSol->Long.Ddm.i16Degrees = -pSol->Long.Ddm.i16Degrees;
	else return false;
#ifdef SWITCH_GNSS_SPOOF
	// ...,01851.528100,E,...
	pSol->Long.Ddm.i16Degrees = gnssSpoof[u16GnssSpoofIdx].i16LongDegrees;
	pSol->Long.Ddm.u8Minutes = gnssSpoof[u16GnssSpoofIdx].u8LongMinutes;
	pSol->Long.Ddm.u32MicroMinutes = gnssSpoof[u16GnssSpoofIdx].u32LongMicroMinutes;
#endif
	// Convert to other formats (after confirming the sign)
	GPS_vCoordConvertDdmToDms(&pSol->Long.Dms, &pSol->Long.Ddm);
	GPS_vCoordConvertDdmToDeg(&pSol->Long.Deg, &pSol->Long.Ddm);
	// Find ',' before speedOverGround
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Find ',' before courseOverGround
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Find ',' before date
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// --- DATE --- blank / ddmmyy
	// Next 0/6 chars will be ddmmyy
	if (*p != ',') {
		for (i=0; i<6; i++) {
			if ( !isdigit((unsigned char)*(p+i)) ) return false;
		}
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, (p+0), 2);
		pSol->TimeDateUtc.date = atoi(acGnssStrBuf);
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, (p+2), 2);
		pSol->TimeDateUtc.month = atoi(acGnssStrBuf);
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, (p+4), 2);
		pSol->TimeDateUtc.year = atoi(acGnssStrBuf);

		if (pSol->TimeDateUtc.date) pSol->TimeDateUtc.date--;		// calendar_date.date is [0..30], not[1..31]
		if (pSol->TimeDateUtc.month) pSol->TimeDateUtc.month--;		// calendar_date.month is [0..11], not[1..12]
		pSol->TimeDateUtc.year += 2000;		// Change 21 to 2021

		// Calc UTC timestamp
		pSol->u32TimeUnix = DATETIME_u32DateTimetoTimestamp(&pSol->TimeDateUtc);
	}

	// Calculate other data fields in solution

	return true;
}

bool GPS_bParseNmeaMsgGga(char * pNmeaMsg, gnss_sol_t * pSol)
{
	// $GPGGA,040028.00,3356.843520,S,01850.951799,E,1,06,0.7,76.3,M,38.0,M,,*4A
	// $GPGGA,,,,,,0,,,,,,,,*66

	char *p;
	uint8_t u8LetLongDecimalCnt, i;

	// Check result ptr
	if (pSol==NULL)	return false;

	p = pNmeaMsg;
	// Find ',' before utcTime
	if ( !__bFindNext_P(&p, "GGA,", true) ) return false;
	// Find ',' before latitude
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Find ',' before N/S indicator
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Find ',' before longitude
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Find ',' before E/W indicator
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Find ',' before position fix indicator
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Find ',' before satellites used
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Find ',' before HDOP
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Find ',' before altitude
	if ( !__bFindNext_P(&p, ",", true) ) return false;
	// Next ? chars will be altitude (if present)
	if (*p != ',')
	{
		// Look for '.' in "76.3," or "99999.9"
		u8LetLongDecimalCnt = 0;
		for (i=0; i<5; i++)		// Unsure about max altitude...
		{
			if ( isdigit((unsigned char)*(p+i)) ) u8LetLongDecimalCnt++;
			else break;
		}
		// Check '.' in "76.3"
		if (*(p+u8LetLongDecimalCnt) != '.') return false;
		// Check '3' in "76.3"
		if ( !isdigit((unsigned char)*(p+u8LetLongDecimalCnt+1)) ) return false;
		// Convert
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, p, u8LetLongDecimalCnt);
		pSol->Altitude.i16Value = atoi(acGnssStrBuf);
		pSol->Altitude.u8DecimalValue = *(p) - 48;
	}
	// Don't care about rest of fields

	return true;
}

bool GPS_bParseNmeaMsgPQTMEPE(char * pNmeaMsg, gnss_sol_t * pSol)
{
	// $PQTMEPE,<MsgVer>,<EPE_North>,<EPE_East>,<EPE_Down>,<EPE_2D>,<EPE_3D>*<Checksum><CR><LF>
	// $PQTMEPE,2,1.000,1.000,1.000,1.414,1.732*52
	// $PQTMEPE,2,27.8,43.492,87.809*63
	// - 1 = Message Version
	// - 2 = Estimated North Error
	// - 3 = Estimated East Error
	// - 4 = Estimated Down Error
	// - 5 = Estimated 2D Position Error
	// - 6 = Estimated 3D Position Error

	char *s1, *s2;

	// Find ',' before MsgVer
	if ( !__bFindNext_P(&pNmeaMsg, "PQTMEPE,", true) ) return false;
	// Find ',' before EPE_North
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	// Find ',' before EPE_East
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	// Find ',' before EPE_Down
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	// Find ',' before EPE_2D
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	// Next ? chars will be EPE_2D; assume x.xxx to xxxxx.xxx
	// Isolate integer part of EPE_2D
	s1 = pNmeaMsg;
	if ( !__bFindNext_P(&pNmeaMsg, ".", true) ) return false;
	s2 = pNmeaMsg - 1;		// Point to '.'
	if (s2-s1<=3)
	{
		memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
		strncpy(acGnssStrBuf, s1, (s2-s1));
		pSol->PositionError.u16ValueInt = atoi(acGnssStrBuf);
		// Isolate decimal part of EPE_2D to one digit
		s1 = pNmeaMsg;
		if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
		if ( !isdigit((unsigned char)*s1) ) return false;
		pSol->PositionError.u8ValueDecimal = *s1 - 48;

	}
	else
	{
		// Limit
		pSol->PositionError.u16ValueInt = 999;
		pSol->PositionError.u8ValueDecimal = 9;
	}
	// Set has-flag
	pSol->PositionError.bHasValue = true;

	return true;
}

bool GPS_bParseNmeaMsgPubx00(char * pNmeaMsg, gnss_sol_t * pSol)
{
	// $PUBX,00,141753.00,3356.877,E,125.580,G3,1.8,3.1,0.068,387,1.24,0.93,18,0,0*45
	// Also without decimals!
	// $PUBX,00,195800.00,3356.85000,S,01850.95763,E,130.950,G3,13,16,1.279,100.81,etc

	char *s1, *s2;
//	uint8_t u8LetLongDecimalCnt, i;

	// Check result ptr
	if (pSol==NULL)	return false;

	// Find ',' before time
	if ( !__bFindNext_P(&pNmeaMsg, "PUBX,00,", true) ) return false;
	// Find ',' before lat
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	// Find ',' before NS
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	// Find ',' before long
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	// Find ',' before EW
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	// Find ',' before altRef
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	// Find ',' before navStat
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	// Find ',' before hAcc
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	s1 = pNmeaMsg;		// s1 points to first char of hAcc
	// Find ',' before vAcc
	if ( !__bFindNext_P(&pNmeaMsg, ",", true) ) return false;
	s2 = pNmeaMsg - 1;		// s1 points to last char of hAcc
	// Sanity check, covering both intermediate buffer size and atoi later; covers "DDDD.D" or "DDDDD"
	if (s2-s1>5) return false;
	memset(acGnssStrBuf, 0, sizeof(acGnssStrBuf));
	pSol->PositionError.u8ValueDecimal = 0;		// Default decimal if no digit
	uint8_t i = 0;
	while (s1<=s2)
	{
		if (*s1=='.')
		{
			pSol->PositionError.u8ValueDecimal = *(s1+1) - 48;
			break;
		}
		else
		{
			acGnssStrBuf[i++] = *s1++;
		}
	}
	// Calculate int part
	pSol->PositionError.u16ValueInt = atoi(acGnssStrBuf);
	// Set has-flag
	pSol->PositionError.bHasValue = true;

	return true;
}

/**
 * Convert coordinate from DDM to DMS format
 * <p>
 * Convert coordinate from degrees-decimal-minutes format to degrees-minutes-seconds format
 *
 * @param	gps_coord_dms_t	Pointer to struct with DMS coordinate
 * @param	gps_coord_ddm_t	Pointer to struct with DDM coordinate
 */
void GPS_vCoordConvertDdmToDeg(gnss_coord_deg_t * pDeg, const gnss_coord_ddm_t * pDdm)
{
	int32_t i32MicroDegrees;

	// -33 56.842939 to -33.947382

	// -33 56.842939 will have pDdm->u32MilliMinutes == 842939
	// -33 56.042939 will have pDdm->u32MilliMinutes == 42939
	// -33 56.000100 will have pDdm->u32MilliMinutes == 100
	// -33 56.100000 will have pDdm->u32MilliMinutes == 100000

	pDeg->i16Deg = pDdm->i16Degrees; // -33
	pDeg->u32DeciMicroDeg = (uint32_t)pDdm->u8Minutes*10000;		// 56 to 560000
	pDeg->u32DeciMicroDeg += (pDdm->u32MicroMinutes/100);		// Reduce resolution from -6 (milli) to -4
	pDeg->u32DeciMicroDeg *= 100;
	pDeg->u32DeciMicroDeg /= 60;

	i32MicroDegrees = ( pDeg->i16Deg > 0 ? pDeg->u32DeciMicroDeg : -pDeg->u32DeciMicroDeg);

	// Float format
	pDeg->fDegrees = ((float) pDeg->i16Deg) + ((float) i32MicroDegrees) / 1000000;

	//
	pDeg->i32MicroDeg = 1000000*((int32_t)pDeg->i16Deg) + i32MicroDegrees;
};


void GPS_vCoordConvertDdmToDms(gnss_coord_dms_t * pDms, const gnss_coord_ddm_t * pDdm)
{
	uint16_t u16;

	// e.g. convert 47 Degrees, 17.112671 Minutes to 47 Degrees, 17 Minutes, 6.76026 Seconds

	// Degrees
	pDms->i16Degrees = pDdm->i16Degrees;
	// Minutes
	pDms->u8Minutes = pDdm->u8Minutes;
	// Seconds
	u16 = (60*pDdm->u32MicroMinutes)/1000000;
	pDms->u8Seconds = (uint8_t) u16;
	// Fractional seconds
	u16 = (60*pDdm->u32MicroMinutes)%1000000;
	pDms->u16MilliSeconds = u16;
};


bool GPS_bCheckNmeaMsg(char * pacNmeaMsg)
{
	// ...$GNGNS,122310.0,3722.425671,N,12258.856215,W,AA,15,0.9,1005.543,6.5,,*77<CR><LF>

	char *s1, *s2;
	uint8_t u8CkCalc, u8CkMsg;

	// Check for start char, and start of checksum range (char after '$')
	s1 = pacNmeaMsg;
	if ( !__bFindNext_P(&s1, "$", true) ) return false;
	// Check for end sequence
	s2 = s1;
	if ( !__bFindNext_P(&s2, "\r\n", false) ) return false;
	// Find end of checksum range (char before the '*')
	s2 = s1;
	if ( !__bFindNext_P(&s2, "*", false) ) return false;
	s2--;

	// Calc checksum
	u8CkCalc = u8CalcNmeaChecksum(s1, s2);
//GPS_DBG("u8CkCalc=%u", u8CkCalc);
	// Isolate checksum in msg
	if ( !__bFindNext_P(&s1, "*", true) ) return false;
	s2 = s1;
	if ( !__bFindNext_P(&s2, "\r\n", false) ) return false;
	s2--;
	if (s2 - s1 != 1) return false;
	u8CkMsg = (u8HexCharToInt(*s1)<<4) | (u8HexCharToInt(*s2));
//GPS_DBG("u8CkMsg=%u", u8CkMsg);
	// Check checksum
	if (u8CkCalc != u8CkMsg)
	{
		return false;
	}

	return true;

}

uint8_t u8HexCharToInt(char cHexChar)
{
	// '0'..'9' is ASCII 48..57, and 'A'..'F' is ASCII 65..70
	if ((cHexChar >= 48) && (cHexChar <= 57)) return cHexChar - 48;
	else if ((cHexChar >= 65) && (cHexChar <= 70)) return cHexChar - 65 + 10;
	else return 0;
}

uint8_t u8CalcNmeaChecksum(char * pacStart, char * pacStop)
{
	uint8_t u8Ck;
	char * p = pacStart;
//GPS_DBG("%u %u", p, pacStop);
	// XOR checksum (run through msg from after '$' until NULL)
	u8Ck = 0;
	while (p <= pacStop) {
		u8Ck ^= *p;
		p++;
	}
	return u8Ck;
}

bool bAddUbxChecksum(uint8_t * pUbxFrame, uint16_t u16Len)
{
	// Check preamble
	if (*pUbxFrame++ != 0xB5) return false;
	if (*pUbxFrame++ != 0x62) return false;
	uint8_t ck_a = 0, ck_b = 0;
	u16Len -= 4;
	while (u16Len--)
	{
		ck_a = ck_a + *pUbxFrame++;
		ck_b = ck_b + ck_a;
	}
	// Add checksum
	*pUbxFrame++ = ck_a;
	*pUbxFrame = ck_b;
	return true;
}

