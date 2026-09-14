/*
 * build_config.h
 *
 * Single place to select this build's compile-time feature set. Every
 * option below used to be a project preprocessor symbol (STM32CubeIDE:
 * Project > Properties > C/C++ Build > Settings > Preprocessor), toggled by
 * hand per flash. That made "what did I actually just flash" hard to
 * recover once the device was on the bench, and easy to lose track of
 * between primary / secondary / bridge builds. Now: comment the options you
 * want in below and rebuild — INIT_vInitialization() also logs the active
 * set over DBG_LOG at boot (see init.c), so it's recoverable from the
 * terminal or the persisted flash log after the fact too.
 *
 * DEBUG, and the toolchain-required CORE_CM4 / USE_HAL_DRIVER / STM32WLE5xx /
 * USE_FULL_LL_DRIVER, remain project preprocessor symbols — they're IDE/
 * toolchain plumbing (ST-generated), not application features, and some are
 * needed by CMSIS/HAL source files compiled before any of our own headers
 * would be visible.
 */

#ifndef CONFIG_BUILD_CONFIG_H_
#define CONFIG_BUILD_CONFIG_H_

/* --------------------------------------------------------------------------
 * FrKernel command-interface transport — either or both of UART/LORA, OR
 * LORA_BRIDGE alone (mutually exclusive with the other two — it's a
 * dedicated bench-rig mode, see FrKernel_Config.h). Defining both UART and
 * LORA lets the same build answer "tag <cmd>" over the debug UART and
 * "tag <ID> <cmd>" over the mesh at the same time.
 * See fr_app/Worker/FrKernel/FrKernel_Config.h.
 * -------------------------------------------------------------------------- */
// #define FRKERNEL_INTERFACE_UART        /* debug UART (bench use) */
 #define FRKERNEL_INTERFACE_LORA        /* LoRa radio (deployed secondary) */
//#define FRKERNEL_INTERFACE_LORA_BRIDGE    /* UART<->LoRa relay (bench test rig) */

/* --------------------------------------------------------------------------
 * Debug transport — select at most one. Neither defined compiles Debug.c to
 * stubs (no output produced). See fr_app/Services/Debug/Debug.h.
 * -------------------------------------------------------------------------- */
#define DEBUG_OUTPUT_UART
// #define DEBUG_OUTPUT_USB               /* not yet implemented -- stub bodies only */

/* --------------------------------------------------------------------------
 * Feature toggles
 *
 * Movement (shake-to-wake / movement sequencing) and GPS are always built
 * in -- not configurable here, no ENABLE_MOVE / ENABLE_GPS guards remain in
 * the code. Neither is the radio link/range test: it is entered at run time
 * with "tag <ID> radiotest" (fr_app/Tasks/RadioTest/RadioTestMode).
 *
 * Keep notes above a define, not trailing it: a trailing block comment
 * spanning several lines stops being a comment the moment the define is
 * commented out with //, and the rest of it lands in the translation unit.
 * -------------------------------------------------------------------------- */
/* Drive the status LEDs. Comment out to TERMINATE them: both pins are left in
 * their analog reset state (Hi-Z, undriven, zero current) and every BSP_LED_*
 * call becomes a no-op. Use the terminated build for production / low-power
 * current measurement. See Hal/hal_bsp.h. */
#define ENABLE_LEDS

// #define ENABLE_LOW_POWER_RECOVERY      /* see fr_app/Tasks/DeviceDiscovery/DeviceDiscovery.h */

/* Persistent flash event logger: 8-byte records (timestamp + event + value)
 * written to internal flash 0x08020000-0x0803FFE0 by a low-priority background
 * task. Off compiles the whole logger away — no task, no flash region, every
 * EVTLOG() / FLASHLOG_v* call becomes nothing. See fr_app/inc/flashLog.h. */
// #define ENABLE_FLASH_LOG

/* Expose MOVE_i8SensorDataX/Y/Z() — accessors returning the last raw
 * accelerometer sample, for bench telemetry. See fr_app/Worker/Movement. */
// #define ENABLE_MOVEMENT_SENSOR_DATA_API

/* --------------------------------------------------------------------------
 * Mesh
 * -------------------------------------------------------------------------- */
/*
 * Extra per-campaign diagnostic counters for a field-test build. These are not
 * verbose (one aggregate line per campaign, nothing per-packet); they cost RAM,
 * and .bss is byte-exact full — so enabling them halves MESH_MAX_NEIGHBORS from
 * 120 to 64 to pay for the ~8 B. That is a capacity change, not just a logging
 * one: a test herd above 64 nodes needs this off.
 *
 * What they exist to answer, none of which the production stats line can:
 *   - fwdBeacon vs fwdDreq/fwdAck/fwdTs: "forwarded=" is one lumped number, so
 *     beacon relay amplification - the term that should stop scaling with herd
 *     size now the dedupe ring holds 48 - cannot be separated from the rest.
 *   - dedupeHit: beacons correctly suppressed as already-seen. The direct read
 *     on whether the 16-bit fold bought real history.
 *   - bpSkipBeacon: beacon relays refused because the TX queue was already
 *     deep. The cleanest single measure of node saturation, and deliberately
 *     NOT in the production tally (see u16StatTxDropped, where counting
 *     back-pressure would make the number climb when things are working).
 */
#define ENABLE_MESH_DIAG_COUNTERS

/*
 * Per-packet mesh logging: TX queued, TX sent, and "... seen before" dedup
 * hits — these multiply O(N) across the fleet and are the dominant flash-log /
 * ring-buffer noise. Off by default: the event narrative (received/forwarded/
 * sent, beacon start/stop, timesync applied) and a single "Transmitting <type>
 * len=" per TX are still logged. Define for bench debugging of the TX queue or
 * dedup behaviour.
 */
// #define ENABLE_MESH_VERBOSE_LOG

/* --------------------------------------------------------------------------
 * Periodic telemetry logging — both off in production, where the per-session
 * outcome lines already capture what matters and these only add noise.
 * -------------------------------------------------------------------------- */
/* A "solar: Vsolar=... Vdda=..." line on every 10 s ADC sample instead of one
 * reading per production wake. On a primary these land mid-campaign. */
// #define ENABLE_SOLAR_PERIODIC_LOG

/* One GNSS telemetry CSV line per second of every fix session (ttff, sv, snr,
 * fix count, position error, lat/long delta), plus a header line. Suppressed
 * anyway in an ENABLE_GNSS_LOGGER_BUILD. */
// #define ENABLE_GPS_PERIODIC_LOG

/* --------------------------------------------------------------------------
 * GNSS build variant
 * -------------------------------------------------------------------------- */
/* Dedicated GNSS-logger build: turns on GGA/altitude NMEA parsing and
 * suppresses the periodic GPS CSV above. */
// #define ENABLE_GNSS_LOGGER_BUILD

/* Parse GGA (altitude) sentences in the NMEA dispatcher. Implied by
 * ENABLE_GNSS_LOGGER_BUILD; define on its own to add altitude to a normal
 * build. */
// #define ENABLE_GNSS_NMEA_GGA

/* --------------------------------------------------------------------------
 * Default timezone
 *
 * Sets TIME_TZ_GMT_ADJ_DEF, which drives the tag's whole day/night schedule.
 * Undefined = South Africa (UTC+02); defined = Brisbane, Australia (UTC+10).
 * See fr_app/inc/config/time_config.h.
 * -------------------------------------------------------------------------- */
// #define TIME_TZ_DEFAULT_BRISBANE

/* --------------------------------------------------------------------------
 * Compile-time guards
 * -------------------------------------------------------------------------- */
#if defined(FRKERNEL_INTERFACE_LORA_BRIDGE) && \
    (defined(FRKERNEL_INTERFACE_UART) || defined(FRKERNEL_INTERFACE_LORA))
#  error "build_config.h: FRKERNEL_INTERFACE_LORA_BRIDGE is exclusive of FRKERNEL_INTERFACE_UART/_LORA"
#endif
#if !defined(FRKERNEL_INTERFACE_LORA_BRIDGE) && \
    !defined(FRKERNEL_INTERFACE_UART) && !defined(FRKERNEL_INTERFACE_LORA)
#  error "build_config.h: define FRKERNEL_INTERFACE_LORA_BRIDGE, or at least one of FRKERNEL_INTERFACE_UART/_LORA"
#endif

#endif /* CONFIG_BUILD_CONFIG_H_ */
