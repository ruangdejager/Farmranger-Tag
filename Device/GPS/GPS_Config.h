/*
 * GPS_Config.h
 *
 *  Created on: Nov 10, 2025
 *      Author: Ruan de Jager
 */

#ifndef DEVICE_GPS_GPS_CONFIG_H_
#define DEVICE_GPS_GPS_CONFIG_H_

/// Enable software module debugging
#define GPS_DBG_ENABLE		true

// Default TTFF timeout [s]
#define GNSS_TTFF_TIMEOUT_1_AID_ASSIST	UINT16_C(60)

// Extended GNSS timeout [s]
#define GNSS_TTFF_TIMEOUT_2_COLD_START		UINT16_C(180)		// was 300 during testing

// Minimum sequential fixes for a stable session solution
// How is this number chosen? From testing (800006, 230217) we observed that
// in non-optimal signal conditions, the solution deltas *seem* to converge
// for up to 7-8 solutions; it then ramps up a bit, after which it seems to
// settle for good. We choose the number bigger than 7-8.
#define GNSS_SESSION_VALID_FIX_CNT_MIN	15 // was 10

// Maximum sequential fixes for a stable session solution
// This time allows additional time (on top of the minimum) for the solution
// deltas to settle, but also caps the time we wait for sequential fixes to
// settle.
#define GNSS_SESSION_VALID_FIX_CNT_NOM		30	// was 15
//#define GNSS_SESSION_VALID_FIX_CNT_MAX		40

// Maximum lat & long variation between subsequent GPS locations for valid session
#define GNSS_SESSION_VALID_LAT_LONG_DELTA_MAX	3

// Primary maximum allowed estimated position error [m] (if available)
#define GNSS_SESSION_VALID_EST_POS_ERROR_NOM		5
// Secondary maximum allowed estimated position error [m] (if available)
#define GNSS_SESSION_VALID_EST_POS_ERROR_MAX		10

// GPS assist info
#define GNSS_ASSIST_LOCAL_FILENAME		"UFS:xtra2.bin"		// For now use one local filename
#define GNSS_ASSIST_SERVER_RESOURCE_1		"http://xtrapath1.izatcloud.net/xtra2.bin"
#define GNSS_ASSIST_SERVER_RESOURCE_2		"http://xtrapath2.izatcloud.net/xtra2.bin"

// Compile-time option to enable NMEA GGA string; typically used for altitude
// measurements. We define this functionality as optional (as opposed to
// nominal) because it is currently only used in experimental applications (e.g.
// GPS logger)
//#if defined SWITCH_GNSS_LOGGER || defined SWITCH_POSITION_LOGGER
#if defined SWITCH_GNSS_LOGGER
#define GNSS_NMEA_GGA	true
#endif

// Timeout for external GNSS module detect
// Time to wait from powering possibly present GNSS receiver until a sentence
// containing the model details has been parsed.
// For initial testing using samples parts
// (MODULE_LC76GPANR12A02S,2023/07/03,09:27:58):
// - fail<320ms, pass>330ms, choose 600ms
// (also tested MAX-M10S: fail<550ms, pass>560ms)
// However, this value turned to be marginal for production parts
// (MODULE,LC76GPANR12A03S,2024/04/14,15:42:19):
// - min=435ms, max=639ms (5 test samples, shorter startup time SEEMS to occur
//   consistently when the module was placed in power down mode using
//   "$PAIR650,0*25\r\n")
// Choose 2s timeout for >50% safety margin in worst case (don't want this value
// too large as units without external GNSS module will wait out this time)
#define GNSS_EXT_MODULE_DETECT_TIMEOUT	2000

#endif /* DEVICE_GPS_GPS_CONFIG_H_ */
