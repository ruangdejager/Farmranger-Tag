/*
 * Gps.h
 *
 *  Created on: Nov 14, 2025
 *      Author: Ruan de Jager
 */

#ifndef DEVICE_GPS_GPS_H_
#define DEVICE_GPS_GPS_H_

#include "Gps_Config.h"
#include "Gps_Driver.h"
#include "platform.h"
#include "dbg_log.h"

#include "string.h"
#include "ctype.h"

typedef struct _gnss_sol_sv_t
{
	uint8_t u8SvInView;
	uint8_t u8SvTracking;
	uint8_t u8SnrAvgDbHz;
	uint8_t u8SnrAvgPercent;
} gnss_sol_sv_t;

typedef struct _gnss_time_utc_t
{
	uint8_t u8Hour;
	uint8_t u8Minute;
	uint8_t u8Second;
} gnss_time_utc_t;

typedef struct _gnss_date_t
{
	uint8_t u8Day;
	uint8_t u8Month;
	uint8_t u8Year;
} gnss_date_t;

//! Type definition for GPS coordinate in DEGREES format (i.e. -47.085211)
typedef struct _gnss_coord_deg_t
{
	int16_t		i16Deg;		// The integer part of the coordinate in DEG-format (e.g. -47); use with u32DeciMicroDeg
	uint32_t	u32DeciMicroDeg;		// The decimal part of the coordinate in DEG-format, in microdegrees (e.g. 85211); use with i16Deg (pad with 6 zeros)
	float		fDegrees;		// Coordinate in DEG-format, represented as a float (repoirted to server)
	int32_t		i32MicroDeg;		// The coordinate in microdegrees (e.g. -47085211) (used for numeric manipulations)
} gnss_coord_deg_t;

//! Type definition for GPS coordinate in degrees-minutes-seconds (e.g. )
typedef struct _gnss_coord_dms_t
{
	int16_t i16Degrees;
	uint8_t u8Minutes;
	uint8_t u8Seconds;
	uint16_t u16MilliSeconds;
} gnss_coord_dms_t;

//! Type definition for GPS coordinate in degrees-decimal-minutes (DDM) (i.e. 47deg 17.112671min)
typedef struct _gnss_coord_ddm_t
{
	int16_t		i16Degrees;
	uint8_t		u8Minutes;
	uint32_t	u32MicroMinutes;
} gnss_coord_ddm_t;


typedef struct _gnss_position_error_t
{
	bool 		bHasValue;
	uint16_t	u16ValueInt;		// The integer part of the position_error, i.e. 259 for "259.3m"
	uint8_t		u8ValueDecimal;	// The decimal part of the position_error (to 1 digit), i.e. 3 for "259.3m"
} gnss_position_error_t;

typedef struct _gnss_nmea_rmc_t
{
	bool bStatusValid;
	gnss_time_utc_t TimeUtc;
	gnss_date_t TimeDate;
	gnss_coord_ddm_t Latitude;
	gnss_coord_ddm_t Longitude;

	uint8_t u8SnrAvgPercent;
} gnss_nmea_rmc_t;

// Type definition for altitude metric
typedef struct _gnss_altitude_t
{
	int16_t		i16Value;		// The integer part of the altitude, i.e. 259 for "259.3m"
	uint8_t		u8DecimalValue;	// The decimal part of the altitude (to 1 digit), i.e. 3 for "259.3m"
} gnss_altitude_t;
// Type definition for NMEA GGA struct
typedef struct _gnss_nmea_gga_t
{
	gnss_altitude_t Altitude;
} gnss_nmea_gga_t;

//! Type definition for GPS coordinates (DDM, deg and DMS formats)
typedef struct _gnss_coordinate_t
{
	gnss_coord_ddm_t		Ddm;
	gnss_coord_deg_t		Deg;
	gnss_coord_dms_t		Dms;
} gnss_coordinate_t;

typedef enum _gnss_nmea_msg_bm_t
{
	GNSS_NMEA_MSG_RMC_bm 		= 	(1<<0),		//0x01,
	GNSS_NMEA_MSG_GSV_bm 		= 	(1<<1),		//0x02,
	GNSS_NMEA_MSG_GGA_bm 		= 	(1<<2),		//0x04,
	GNSS_NMEA_MSG_PQTMEPE_bm	=	(1<<3),		//0x08
	GNSS_NMEA_MSG_PUBX00_bm	=	(1<<4),		//0x016
} gnss_nmea_msg_bm_t;

// For compatibility with instelprogram
typedef enum _gnss_session_fix_t {
	GNSS_SESSION_FIX_NONE=0,		// off
	GNSS_SESSION_FIX_BUSY,
	GNSS_SESSION_FIX_OK,
	GNSS_SESSION_FIX_ERROR		// Timeout
} gnss_session_fix_t;

typedef struct _gnss_sol_t
{
	gnss_nmea_msg_bm_t 	tNmeaMsgFlags;
	bool 				bStatusValid;
	gnss_sol_sv_t 		Sv;
	gnss_coordinate_t 	Lat;
	gnss_coordinate_t 	Long;
	gnss_altitude_t 	Altitude;
	datetime_t 			TimeDateUtc;
	uint32_t 			u32TimeUnix;
//	int32_t 			i32Accuracy;
//	bool				bHasAccuracy;
	gnss_position_error_t PositionError;
} gnss_sol_t;

//! GPS session
typedef struct _gnss_session_t {
	bool bDbgStartFlag;		//! One-shot flag used to trigger dbg/log info at start of session
	bool bDbgStopFlag;
	uint16_t u16TimeToFirstFix;		//! Time [s] to first fix (0 when no fix yet); locks in TTFF
	uint32_t u32StartTime;		//! Relative RTCTMR timestamp for session start; used for all session timing
	gnss_coordinate_t LastValidLat;		//! Most recent valid & stable lat & long
	gnss_coordinate_t LastValidLong;
//	bool bHasLastStable;
	uint8_t u8ValidFixCount;		//! Count sequential valid fixes towards a stable position solution
	uint16_t u16TtffTimeout;		//! TTFF timeout; setup when session starts & stays static
	uint16_t u16TtffTmr;		//! TTFF timer; counts up to u16TtffTimeout while session is not stable
//	int32_t i32Accuracy;
//	bool	bHasAccuracy;
	bool bHasFirstStableFix;
	bool bHasStableFixNow;
	gnss_nmea_msg_bm_t 	tNmeaMsgFlagsBm;		// Bitmask for solution message  flags
//	bool bCommsOk;		// One-shot flag to check comms for every session



//	gnss_session_sol_t Sol;


//	bool bGotFirstFix;
//	uint16_t u16Tmr;		// Time session fix
//	uint16_t u16Ttff;

	gnss_session_fix_t Fix;		// Used by termprog


//	uint16_t u16LatDelta;
//	uint16_t u16LongDelta;
//	bool bFirstFix;
} gnss_session_t;

void GPS_vInit(void);
void GPS_vRxTask(void *parameters);

#endif /* DEVICE_GPS_GPS_H_ */
