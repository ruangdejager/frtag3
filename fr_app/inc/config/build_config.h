/*
 * build_config.h
 *
 * Single place to select this build's compile-time feature set. Every
 * option below used to be a project preprocessor symbol (STM32CubeIDE:
 * Project > Properties > C/C++ Build > Settings > Preprocessor), toggled by
 * hand per flash. That made "what did I actually just flash" hard to
 * recover once the device was on the bench, and easy to lose track of
 * between primary / secondary / console builds. Now: comment the options you
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
 * FrKernel command interface (kernelV2) — either or both of UART/LORA for a
 * tag, OR LORA_CONSOLE alone for the operator console rig.
 *
 *   FRKERNEL_INTERFACE_UART     tag: single-letter commands on the debug UART
 *                               (bench). Any command opens a session.
 *   FRKERNEL_INTERFACE_LORA     tag: v2 frames over LoRa from a console. A
 *                               session opens only inside a connect window
 *                               (2 min after shake, first 5 s of a discovery
 *                               wake). Both may be defined; they share one
 *                               session.
 *   FRKERNEL_INTERFACE_LORA_CONSOLE
 *                               the console: this image's only job is "c <id>"
 *                               / "c bulk" and sending the letters typed on its
 *                               debug UART. Skips DeviceDiscovery, Farmranger,
 *                               GPS and Movement (init.c). Replaces the v1
 *                               FRKERNEL_INTERFACE_LORA_BRIDGE raw relay.
 *
 * See fr_app/Worker/FrKernel/FrKernel.h for the session model and
 * FrKernel_Commands.c for the command letters.
 *
 * Release images carry UART as well as LORA: the production station
 * (production/provision_tag.ps1) tests and serialises every unit through the
 * debug UART on the ST-LINK's virtual COM port. Without UART the station cannot
 * talk to the tag, and DeviceDiscovery does not hold the MCU awake in the boot
 * window (APP_BOOT_WINDOW_MS) because nothing could use it.
 * -------------------------------------------------------------------------- */
#define FRKERNEL_INTERFACE_UART           /* tag: debug UART (bench + production station) */
 #define FRKERNEL_INTERFACE_LORA           /* tag: LoRa (deployed tag)    */
// #define FRKERNEL_INTERFACE_LORA_CONSOLE   /* operator console rig        */

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
 * with kernel command 'R' (fr_app/Tasks/RadioTest/RadioTestMode).
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

/* Persistent flash event logger: 8-byte records (timestamp + event + value)
 * written to internal flash 0x08020000-0x0803FFE0 by a low-priority background
 * task. Off compiles the whole logger away — no task, no flash region, every
 * EVTLOG() / FLASHLOG_v* call becomes nothing. See fr_app/inc/flashLog.h. */
// #define ENABLE_FLASH_LOG

/* Expose MOVE_i8SensorDataX/Y/Z() — accessors returning the last raw
 * accelerometer sample, for bench telemetry. See fr_app/Worker/Movement. */
// #define ENABLE_MOVEMENT_SENSOR_DATA_API

/* --------------------------------------------------------------------------
 * External flash bring-up and diagnostics
 *
 * See fr_app/Device/Flash/Flash_SelfTest.c for all three.
 * -------------------------------------------------------------------------- */
/* Destructive half of the boot flash self-test: erase, program and byte-compare
 * a 256-byte pattern on the reserved sector at FLASH_SELFTEST_ADDR (0x7F000 -
 * the log's top sector, which is not shared with anything). The read-only half
 * - JEDEC ID and status register - always runs, with or without this.
 *
 * Off in production. It costs one 4 KB erase per boot against a 100,000-cycle
 * endurance rating (so the wear is negligible) plus ~50 ms of boot time, but a
 * production image has nothing to do with the answer: the write path is
 * exercised continuously by the log anyway. This is a bring-up and returns
 * tool. */
// #define ENABLE_FLASH_SELFTEST_DESTRUCTIVE

/* Verify every buffer FLASH_vRead returns by reading it a second time and
 * comparing, retrying up to FLASH_READ_CRC_MAX_RETRIES before giving up, and
 * counting both outcomes. Turns silent read corruption from something only
 * visible as a downstream checksum failure into a counted, loggable event with
 * an address attached - readable at any time with kernel command 'k'.
 *
 * Costs one extra read per FLASH_vRead. That is real on the OTA verify path
 * (~1900 reads per whole-image pass) and invisible everywhere else. Left ON
 * because frtag2 had an open defect whose whole difficulty is that corrupted
 * reads report success: flash reads issued soon after a radio TX came back
 * wrong (see OTA_LORA_CHUNK_GAP_MS in Fota_Config.h). The mechanism was never
 * established - the supply-rail-sag explanation is untested, and noise on the
 * long MISO/MOSI lines over a non-continuous ground plane is the likelier
 * cause - and the V3 RevA layout has not yet been shown to be free of it. */
#define ENABLE_FLASH_READ_CRC

/* Console flash diagnostics: kernel command 'p' and
 * 'z [tx|park] <N>'. The stress loop is the harness for the
 * post-TX read corruption - it takes a callback so the disturbance under
 * investigation is injectable rather than hard-wired, which is what lets a TX,
 * a power-down cycle and a quiet baseline be compared against each other. */
#define ENABLE_FLASH_STRESS_TEST

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

/*
 * Per-packet radio trace on the debug UART: "TX done", "CAD busy/clear" and
 * every CAD back-off, i.e. one or two lines for every transmission. Off by
 * default - during a kernel log stream these alone overran the DbgLog ring and
 * displaced real events, and on the kernel console they scrolled a line per
 * CONNECT beacon. Define for bench debugging of the radio task. See
 * fr_app/Device/LoraRadio/LoraRadio.c (RADIO_VDBG).
 */
// #define ENABLE_RADIO_VERBOSE_LOG

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
#if defined(FRKERNEL_INTERFACE_LORA_BRIDGE)
#  error "build_config.h: FRKERNEL_INTERFACE_LORA_BRIDGE was replaced by FRKERNEL_INTERFACE_LORA_CONSOLE (kernelV2)"
#endif
#if defined(FRKERNEL_INTERFACE_LORA_CONSOLE) && \
    (defined(FRKERNEL_INTERFACE_UART) || defined(FRKERNEL_INTERFACE_LORA))
#  error "build_config.h: FRKERNEL_INTERFACE_LORA_CONSOLE is exclusive of FRKERNEL_INTERFACE_UART/_LORA"
#endif
#if !defined(FRKERNEL_INTERFACE_LORA_CONSOLE) && \
    !defined(FRKERNEL_INTERFACE_UART) && !defined(FRKERNEL_INTERFACE_LORA)
#  error "build_config.h: define FRKERNEL_INTERFACE_LORA_CONSOLE, or at least one of FRKERNEL_INTERFACE_UART/_LORA"
#endif
/* The console prints everything through DbgLog's UART path. */
#if defined(FRKERNEL_INTERFACE_LORA_CONSOLE) && !defined(DEBUG_OUTPUT_UART)
#  error "build_config.h: FRKERNEL_INTERFACE_LORA_CONSOLE needs DEBUG_OUTPUT_UART"
#endif
/* Tag UART commands arrive on the debug UART. */
#if defined(FRKERNEL_INTERFACE_UART) && !defined(DEBUG_OUTPUT_UART)
#  error "build_config.h: FRKERNEL_INTERFACE_UART needs DEBUG_OUTPUT_UART"
#endif

#endif /* CONFIG_BUILD_CONFIG_H_ */
