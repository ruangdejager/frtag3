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
 * PA0 drives EN of the TPS7A0218 (U6), the 1.8 V LDO that makes GNSS_VCC for
 * the module's VCC and V_IO. The MCU's rail is always on, so the power order
 * is rail first, pins second (GPS_vHwPowerOn in GPS.c). On the final
 * secondary PB6 is push-pull (R21, the GNSS_RX pull-up, is not fitted), so a
 * wait that were too short WOULD drive the MCU's TX high into a module whose
 * I/O rail is still ramping; the wait also keeps junk RX bytes out at attach.
 *
 * osDelay(10) guarantees at least 9 ms on the 1 ms tick. The TPS7A02 family
 * reaches regulation in well under a millisecond after EN into the few uF on
 * GNSS_VCC (C44 10 uF + C45 100 nF), so this is a wide margin, and it is
 * negligible next to the 300 ms the cold start already waits for the receiver
 * to boot. Re-measure (scope GNSS_VCC against PA0) if the LDO or its output
 * capacitance changes.
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
#define GPS_RTC_SYNC_MIN_ERROR_S            10U

#endif /* DEVICE_GPS_GPS_CONFIG_H_ */
