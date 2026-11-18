/*
 * Identity_Config.h
 *
 * Where the unit's production serial number is stored, and the record format.
 * See Identity.c for the read and write rules.
 *
 * ---- Storage ----
 *
 * Primary copy: the STM32WL OTP area, write-once. It survives a mass erase, a
 * reflash over SWD, a bootloader fault and a replaced or chip-erased NOR, and
 * it sits on the same die as the 96-bit UID the radio id is built from, so
 * serial number and radio id cannot drift apart. Being write-once is the point:
 * a production serial number is not something firmware should be able to
 * change by accident.
 *
 * Corrections are possible but deliberate: a new record goes into the next
 * free slot and the highest valid slot wins. The identity uses the first half
 * of OTP - 32 slots of 16 bytes - so a unit can be re-serialised 31 times. The
 * second half (0x1FFF7200-0x1FFF73FF) is left unallocated so a later per-unit
 * record (factory calibration, say) has write-once space of its own.
 *
 * Mirror: the same 16-byte record at the start of the NOR identity sector
 * (SETTINGS_STORE_IDENTITY_ADDR, see SettingsStore_Config.h). Tooling that
 * reads the NOR finds it there, and it is the fallback for a unit whose OTP
 * holds nothing. It is rewritten from OTP at boot whenever it differs, so a
 * NOR chip erase (kernel 'W') or a replaced NOR heals itself on the next boot.
 *
 * ---- Record (16 bytes, little-endian) ----
 *
 *   [0..1]   magic      IDENTITY_MAGIC ("SN" as stored bytes)
 *   [2]      format     IDENTITY_FORMAT
 *   [3]      reserved   0
 *   [4..7]   serial     the production serial number, 1..0xFFFFFFFE
 *   [8..11]  radio id   LORARADIO_u32GetUniqueId() of the unit it was written
 *                       on - the explicit serial <-> radio id link
 *   [12..15] crc        CRC-32 (crc32.h) over bytes 0..11 followed by the
 *                       12-byte UID, UID word 0 first, each word little-endian
 *
 * Folding the UID into the CRC means a record only validates on the die it was
 * written for. A NOR mirror moved to another board, or a record image copied
 * between units by a programming script, reads as "no serial" rather than as
 * someone else's.
 *
 * The record is exactly two OTP double-words. Off-board tooling reproduces the
 * CRC as zlib.crc32(record[0:12] + uid_w0 + uid_w1 + uid_w2), each uid word as
 * 4 little-endian bytes.
 *
 * ---- Serial number format ----
 *
 * Always written and shown as 8 hex digits, 0xVRNNNNNN:
 *
 *   V       hardware version, bits 31..28. Units with the same hardware
 *           version run the same code base and can be flashed with the same
 *           image; frtag3 is hardware version 3 (IDENTITY_SERIAL_HW_VERSION).
 *   R       hardware revision within that version, bits 27..24, 1..F. The
 *           Tagskaap V3 RevA boards are revision 1.
 *   NNNNNN  unit number within that revision, bits 23..0, 000001..FFFFFF.
 *
 * The first unit ever made is 0x31000001. Revision 0 and unit number 0 are
 * never issued, so 0 can keep meaning "no serial" everywhere it is reported.
 *
 * The format is enforced only when a serial is written (IDENTITY_eSetSerial,
 * so kernel 'N' refuses anything else). Reading accepts any CRC-valid record
 * in IDENTITY_SERIAL_MIN..MAX: a record that validates is authentic, and
 * refusing it would make a serialised unit look unserialised - the one thing
 * write-once storage is there to prevent. Firmware does not check the
 * revision against the board it runs on: within one hardware version every
 * revision runs this same image, so the firmware cannot tell revisions apart
 * and has no business refusing one.
 */

#ifndef SETTINGS_IDENTITY_CONFIG_H_
#define SETTINGS_IDENTITY_CONFIG_H_

#include "hal_otp.h"
#include "SettingsStore_Config.h"   /* SETTINGS_STORE_IDENTITY_ADDR */

#define IDENTITY_MAGIC              0x4E53U     /* bytes 'S','N' */
#define IDENTITY_FORMAT             1U
#define IDENTITY_RECORD_LEN         16U

#define IDENTITY_OTP_BASE_ADDR      HAL_OTP_BASE_ADDR
#define IDENTITY_OTP_SLOTS          32U
#define IDENTITY_OTP_END_ADDR       (IDENTITY_OTP_BASE_ADDR + IDENTITY_OTP_SLOTS * IDENTITY_RECORD_LEN)

#define IDENTITY_NOR_ADDR           SETTINGS_STORE_IDENTITY_ADDR

/* 0 and all-ones are refused as serial numbers: 0 reads as "none" everywhere
 * the serial is reported, and all-ones is what an erased record holds. */
#define IDENTITY_SERIAL_MIN         1UL
#define IDENTITY_SERIAL_MAX         0xFFFFFFFEUL

/* Serial number fields (format above). Only serials whose hardware version is
 * this code base's own may be written; production/provision_tag.ps1 carries
 * the same constants. */
#define IDENTITY_SERIAL_HW_VERSION  3UL                 /* frtag3 */
#define IDENTITY_SERIAL_VER_SHIFT   28U
#define IDENTITY_SERIAL_REV_SHIFT   24U
#define IDENTITY_SERIAL_NIBBLE_MASK 0xFUL
#define IDENTITY_SERIAL_UNIT_MASK   0x00FFFFFFUL
#define IDENTITY_SERIAL_FIRST       0x31000001UL        /* version 3, revision 1, unit 1 */

#endif /* SETTINGS_IDENTITY_CONFIG_H_ */
