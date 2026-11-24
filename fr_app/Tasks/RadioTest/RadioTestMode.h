/*
 * RadioTestMode.h
 *
 * Runtime RSSI test - R&D and field tool, built into the PRODUCTION firmware.
 * There is no separate radio-test build: the mode is entered at run time by a
 * FrKernel command and nothing else changes.
 *
 * The tag transmits; an EXTERNAL device - the hand-held signal-strength meter -
 * receives and measures. (This replaces the old tag-to-tag range test, kernel
 * 'R', in which a primary listened, measured each "RT" beacon and pushed the
 * results to the fr9 over AT+RTLOG. That role, its task, its RX queue and its
 * fr9 upload are gone: every test is now done with the meter.)
 *
 * Nothing here runs until the command explicitly enters the mode, and
 * everything it touches is behind RADIOTESTMODE_bActive(). A unit that is never
 * given the command behaves exactly as it would without this file: nothing is
 * allocated, no sleep lock is held, and every gate in MeshNetwork and
 * DeviceDiscovery is a single already-false read.
 *
 * Entering (either role): kernel command B, single-connection only
 *
 *   B      basic: no GPS.
 *   B gps  secondary only: the GPS is powered for the whole test (held on, no
 *          timeout) and every frame sent while it has a LIVE fix carries that
 *          epoch's position and satellite count (flag bit 3, 33-byte frame).
 *          Before the first fix, or after losing it, frames are the basic
 *          24-byte ones. The GPS is powered down when the test ends.
 *
 *   The tag sends the binary "RS" frame below every RSSI_TEST_BEACON_MS (10 s)
 *   and flashes LED_AUX (blue on a secondary, yellow on a primary) on each TX.
 *   It goes deliberately deaf: every inbound radio packet is dropped, kernel
 *   traffic included, so nothing on the air can disturb the measurement. It
 *   rides the mesh TX worker (MESHNETWORK_vTxTask), which sits idle during a
 *   test - a secondary has no heap left for a task of its own.
 *
 *   It leaves the mode on a shake (secondary - a primary has no
 *   accelerometer), on kernel 'Q' / 'x' over the UART, or by itself after
 *   RSSI_TEST_TIMEOUT_MS (1 h), after which the tag resumes its schedule.
 *
 * CW test (either role): kernel command C, single-connection only
 *
 *   An unmodulated carrier on RF_FREQUENCY at TX_OUTPUT_POWER for
 *   CW_TEST_DURATION_MS (1 min), for a spectrum analyser or power meter. It
 *   shares everything above - deaf, mesh stood down, LED_AUX lit for the
 *   whole carrier - and ends after the minute, on a shake (secondary), or on
 *   'Q' / 'x' over the UART. The carrier starts once the command's ack and the
 *   session's goodbye have left the radio, so neither is cut off by it.
 *   A full minute of carrier is far beyond the 1 % duty cycle of the 868 MHz
 *   band: a bench / shielded-room test, not one for the field.
 *
 * The mode is RAM-only and deliberately not persisted: a reboot returns the
 * unit to normal operation. A mode that survived a power cycle would be an
 * easy way to strand a unit in the field with no mesh participation and, on a
 * secondary, no over-the-air way back.
 *
 * ---- RSSI test frame (format version 1), little-endian, unencrypted ----
 * 24 bytes, or 33 with a live GPS fix. The CRC is always the last two bytes.
 *
 *   [0]      0x00  header: MeshPktType_Reserved, so production units drop it
 *   [1..2]   0x52 0x53  "RS"
 *   [3]      0x01  format version; later versions only add fields before the
 *                  CRC, so a meter must accept longer frames and ignore what
 *                  it does not know
 *   [4..7]   u32   tag id, LORARADIO_u32GetUniqueId()
 *   [8..9]   u16   sequence, +1 per TX, wraps; restarts at 0 only on a reset
 *                  (it carries on across B entries within one boot)
 *   [10]     i8    configured TX power, dBm (TX_OUTPUT_POWER)
 *   [11]     u8    antenna configuration id (lookup list in RadioTestMode.c)
 *   [12]     u8    radio profile id (lookup list in RadioTestMode.c)
 *   [13..14] u16   TX interval, units of 10 ms
 *   [15..16] u16   battery, mV (0 = not measured: primaries, or no sample yet)
 *   [17]     i8    temperature, deg C; 0x80 = not available (always, for now)
 *   [18..20] 3x i8 acceleration X/Y/Z, 1 LSB = 1/64 g; 0x80 = not available
 *   [21]     u8    flags: bit0 accelerometer valid, bit1 low battery,
 *                  bit2 recently reset (first 10 frames since boot),
 *                  bit3 GPS position included (bytes 22..30)
 *   only with flag bit3 (a live fix; a cached or old position is never sent):
 *   [22..25] i32   latitude,  microdegrees (as i32LatUDeg)
 *   [26..29] i32   longitude, microdegrees (as i32LonUDeg)
 *   [30]     u8    satellites (tracked with signal, GPS_bGetLiveFix)
 *   always last:
 *   [22..23] or [31..32]  u16  CRC-16/CCITT-FALSE over all preceding bytes,
 *                  low byte first
 */

#ifndef FR_APP_TASKS_RADIOTEST_RADIOTESTMODE_H_
#define FR_APP_TASKS_RADIOTEST_RADIOTESTMODE_H_

#include <stdint.h>
#include <stdbool.h>

/* True while the RSSI or CW test is running. Every hook this feature adds to the core
 * (MeshNetwork parser + TX, DeviceDiscovery scheduler) is guarded by this and
 * does nothing when it is false. Safe to call from any task. */
bool RADIOTESTMODE_bActive(void);

/* Why the most recent RADIOTESTMODE_bEnter() call failed, or "" if it
 * succeeded (or none has been made). A command that arrives over LoRa only
 * ever gets its ack back - the DBG_LOG lines land on the failing unit's own
 * UART - so FrKernel puts this string in the "could not enter" ack. */
const char *RADIOTESTMODE_pcLastEnterError(void);

/* Enter the test. bGps: hold the GPS on and send live positions ("B gps");
 * refused on a primary and outside POWER_CLASS_NORMAL. Returns false, with
 * nothing left half-started, if already active, if there is no MeshNetwork TX
 * worker to beacon from, or if the context cannot be allocated. Call from a
 * task, not an ISR. On return the radio is the test's: the caller should
 * already have sent (and let drain) anything it wanted on the air. */
bool RADIOTESTMODE_bEnter(bool bGps);

/* Enter the CW test (kernel C): same contract and the same failures as
 * RADIOTESTMODE_bEnter, minus the GPS ones. The carrier itself starts a moment
 * later, on the mesh TX worker, once the radio has sent what was queued. */
bool RADIOTESTMODE_bEnterCw(void);

/* Leave the test and restore normal operation: stop the beacon, power the GPS
 * down (B gps), release the sleep lock and re-open the mesh gates. Safe when
 * not active (no-op) and from any task - the shake path (Movement), FrKernel
 * ('Q', 'x') and the mesh TX worker (the 1 h limit) all call it. pcReason is
 * logged. */
void RADIOTESTMODE_vExit(const char *pcReason);

/* Send the next frame if one is due, and return how long the caller may block
 * before calling again. Returns osWaitForever whenever the test is not running,
 * which is the normal case and costs one volatile read.
 *
 * Called from MESHNETWORK_vTxTask at the top of its wait loop, and nowhere
 * else. MESHNETWORK_bSendPacket refuses for the whole test, so that task has
 * nothing else to do while one runs; borrowing it is what lets the test
 * allocate nothing but its small context.
 *
 * Self-timing: it keeps its own deadline, so a caller that wakes early does not
 * bring the next frame forward. A due frame is held back while the radio still
 * has the previous one (queued or in carrier sense), so frames never pile up
 * behind each other on a busy channel. */
uint32_t RADIOTESTMODE_u32ServiceBeacon(void);

/* Decode one RSSI test frame and log it, for the FRKERNEL_INTERFACE_LORA_CONSOLE
 * bench rig, which listens to the whole mesh and is a handy second view of a
 * test. pBuf/u8Len are the payload with the type byte already stripped.
 * Returns false if the frame is not one of ours. Stateless; safe from the
 * parser task. */
bool RADIOTESTMODE_bLogBeacon(const uint8_t *pBuf, uint8_t u8Len,
                              int16_t i16Rssi, int8_t i8Snr, int16_t i16NoiseFloor);

#endif /* FR_APP_TASKS_RADIOTEST_RADIOTESTMODE_H_ */
