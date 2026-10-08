/*
 * GPS_Config.h
 *
 * GNSS session parameters and timing constants.
 */

#ifndef DEVICE_GPS_GPS_CONFIG_H_
#define DEVICE_GPS_GPS_CONFIG_H_

#include "build_config.h"   /* ENABLE_GPS_PERIODIC_LOG, ENABLE_GNSS_* */

/* Default TTFF timeout [s] — aided start */
#define GNSS_TTFF_TIMEOUT_1_AID_ASSIST      UINT16_C(60)

/* Extended TTFF timeout [s] — cold start */
#define GNSS_TTFF_TIMEOUT_2_COLD_START      UINT16_C(180)

/*
 * Sentinel TTFF timeout meaning "no timeout" — used when GPS_vRequestFix()
 * is called with bAutoShutdown == false, so the session is allowed to run
 * indefinitely until a stable fix is declared or GPS_vShutdown() is called.
 */
#define GNSS_TTFF_NO_TIMEOUT                UINT16_C(0xFFFF)

/*
 * Minimum sequential valid fixes before declaring a stable solution.
 * Testing shows solution deltas can take up to 7-8 fixes to converge;
 * choose a margin above that.
 */
#define GNSS_SESSION_VALID_FIX_CNT_MIN      5

/*
 * Nominal sequential fix count: extra time beyond the minimum for the
 * position error to settle before applying GNSS_SESSION_VALID_EST_POS_ERROR_MAX.
 */
#define GNSS_SESSION_VALID_FIX_CNT_NOM      30

/* Maximum lat/long variation between consecutive fixes for a stable session */
#define GNSS_SESSION_VALID_LAT_LONG_DELTA_MAX   3

/* Primary position error threshold [m] (EPE_2D, tight) */
#define GNSS_SESSION_VALID_EST_POS_ERROR_NOM    5

/* Secondary position error threshold [m] (EPE_2D, relaxed) */
#define GNSS_SESSION_VALID_EST_POS_ERROR_MAX    10

/* GNSS assist data filenames / URLs */
#define GNSS_ASSIST_LOCAL_FILENAME          "UFS:xtra2.bin"
#define GNSS_ASSIST_SERVER_RESOURCE_1       "http://xtrapath1.izatcloud.net/xtra2.bin"
#define GNSS_ASSIST_SERVER_RESOURCE_2       "http://xtrapath2.izatcloud.net/xtra2.bin"

/* A GNSS-logger build always parses GGA (altitude); a normal build can opt in
 * on its own with ENABLE_GNSS_NMEA_GGA in build_config.h. */
#if defined(ENABLE_GNSS_LOGGER_BUILD) && !defined(ENABLE_GNSS_NMEA_GGA)
#  define ENABLE_GNSS_NMEA_GGA
#endif

/*
 * Timeout for external GNSS module detection [ms].
 * Chosen for >50% safety margin over worst-case production startup time.
 */
#define GNSS_EXT_MODULE_DETECT_TIMEOUT      2000

/*
 * GNSS rail settle time [ms]: GNSS_ON (PA0) high -> USART1 pins attached.
 *
 * PA0 drives the enable of the LDO that makes GNSS_VCC. The GNSS I/O sits on
 * that rail (1.8 V, abs max 1.98 V) while the MCU runs at 2.0 V, so the power
 * order is rail first, pins second (GPS_vHwPowerOn in GPS.c). With PB6
 * open-drain and PB7 unpulled the MCU can no longer back-feed the module even
 * if this wait were too short; the wait keeps the pins off a module whose I/O
 * rail is still ramping (no junk RX bytes at attach) and keeps the order right
 * if the pin configuration ever changes again.
 *
 * The LDO part number is not known to the firmware, so the value is sized from
 * the part class: small LDOs reach regulation within roughly 0.05-2 ms of EN,
 * soft-start into a few uF included. osDelay(10) guarantees at least 9 ms on
 * the 1 ms tick, more than 4x the slow end, and is negligible next to the
 * 300 ms the cold start already waits for the receiver to boot. Re-measure
 * (scope GNSS_VCC against PA0) once the LDO is identified.
 *
 * Used with osDelay(), so task context only. Both callers of GPS_vHwPowerOn()
 * are tasks: the GPS dispatcher under xGpsLock, and the init-task self-test.
 */
#define GPS_LDO_SETTLE_MS                   10U

/*
 * Minimum RTC error [s] required before a GPS-derived time is applied.
 * Differences smaller than this threshold are treated as within acceptable
 * oscillator drift and the RTC is left unchanged.
 */
#define GPS_RTC_SYNC_MIN_ERROR_S            5U

#endif /* DEVICE_GPS_GPS_CONFIG_H_ */
