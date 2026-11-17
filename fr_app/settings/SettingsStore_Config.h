/*
 * SettingsStore_Config.h
 *
 * Where the unit's persisted data lives on the external NOR, and the record
 * format the settings store writes there. See SettingsStore.c for the write
 * and recovery rules.
 *
 * ---- External NOR map (AT25EU0041A, 512 KB, 4 KB sectors) ----
 *
 *   0x00000 - 0x3AFFF   OTA image scratchpad          Fota_Config.h
 *   0x3B000             OTA metadata sector           Fota_Config.h
 *   0x3C000             unit identity (serial number) reserved, see below
 *   0x3D000             settings bank A               this file
 *   0x3E000             settings bank B               this file
 *   0x3F000             reserved                      this file
 *   0x40000 - 0x7EFFF   text log                      Log.h
 *   0x7F000             flash self-test scratch       Flash_Config.h
 *
 * The 16 KB at 0x3C000 came off the bottom of the text log (268 -> 252 KB).
 * Both flashes were already fully allocated: internal flash is bootloader +
 * the maximum app image, and every NOR sector belonged to OTA, the log or the
 * self-test. Of those, the log is the only one that can shrink without changing
 * a contract - the OTA scratchpad and metadata are shared with the bootloader
 * (Fota_Config.h is the app/bootloader contract) and the bootloader only ever
 * reads those, so nothing it does moves.
 *
 * Why the NOR and not internal flash, which is where the fr9 keeps its
 * settings: internal flash has no free page. Carving one would shrink
 * OTA_APP_MAX_SIZE, a value the bootloader and the scratchpad size both depend
 * on. A NOR sector costs the log 1.5 %.
 *
 * The identity sector is reserved for a mirror of the production serial number
 * and is not written by anything yet. The reserved sector at 0x3F000 is
 * unclaimed headroom (factory calibration is the obvious candidate) so that the
 * next per-unit record does not have to take another bite out of the log and
 * move its start again.
 *
 * Kernel command 'W' (LOG_vEraseAll) is a NOR chip erase and takes this whole
 * partition with it. The unit then runs on compiled-in defaults until the next
 * SETTINGS_bCommit().
 */

#ifndef SETTINGS_SETTINGSSTORE_CONFIG_H_
#define SETTINGS_SETTINGSSTORE_CONFIG_H_

#include "Flash_Config.h"   /* FLASH_SECTOR_SIZE_BYTES, FLASH_PAGE_SIZE_BYTES */

/* ---- Partition ---- */
#define SETTINGS_STORE_REGION_START     0x03C000UL
#define SETTINGS_STORE_IDENTITY_ADDR    0x03C000UL   /* reserved: serial number */
#define SETTINGS_STORE_BANK_A_ADDR      0x03D000UL
#define SETTINGS_STORE_BANK_B_ADDR      0x03E000UL
#define SETTINGS_STORE_SPARE_ADDR       0x03F000UL   /* reserved: unclaimed     */
#define SETTINGS_STORE_REGION_END       0x040000UL   /* first byte of the log   */

#define SETTINGS_STORE_BANK_SIZE        FLASH_SECTOR_SIZE_BYTES

/* ---- Record slots ----
 * A bank is cut into fixed slots and each commit takes the next one. Fixed
 * slots rather than packed records so that a slot can be judged on its own
 * ("all 0xFF" = free, anything else = used) without trusting a length field
 * from a record that may have been torn by a power cut.
 *
 * 512 B holds today's Settings_EE (a few hundred bytes) with room to grow; the
 * _Static_assert in SettingsStore.c fails the build once it no longer fits.
 * Going to 1 KB halves the commits per erase and needs nothing else changed.
 * The slot is a multiple of the 256 B program page so a record never shares a
 * page with its neighbour.
 *
 * Endurance: 8 slots per bank, two banks taking turns, so each sector is erased
 * once per 16 commits. At this part's 10,000 cycles that is 160,000 commits.
 * Commits that change nothing are skipped (see SETTINGS_bStorageSave), so the
 * budget is spent only on real changes. */
#define SETTINGS_STORE_SLOT_SIZE        512U
#define SETTINGS_STORE_SLOTS_PER_BANK   (SETTINGS_STORE_BANK_SIZE / SETTINGS_STORE_SLOT_SIZE)

/* ---- Record header (16 bytes, little-endian, at the start of a slot) ----
 *   [0..3]   magic     SETTINGS_STORE_MAGIC
 *   [4..5]   layout    SETTINGS_LAYOUT_VERSION of the firmware that wrote it
 *   [6..7]   length    payload bytes that follow the header
 *   [8..11]  sequence  +1 per commit; the highest valid one is current
 *   [12..15] crc       CRC-32 (IEEE, reflected, 0xEDB88320) over bytes 0..11
 *                      followed by the payload
 * The full 32-bit CRC is kept on purpose: the fr9's settings record truncates
 * its CRC-32 to 16 bits, which is a 1-in-65536 chance of accepting a corrupt
 * record as good. */
#define SETTINGS_STORE_HDR_LEN          16U
#define SETTINGS_STORE_MAGIC            0x54535246UL  /* "FRST" as stored bytes */
#define SETTINGS_STORE_PAYLOAD_MAX      (SETTINGS_STORE_SLOT_SIZE - SETTINGS_STORE_HDR_LEN)

/* Slots a single commit may try before giving up. A failed write-verify burns
 * its slot (it is no longer all 0xFF) and the retry takes the next one, so
 * this also bounds how much one bad commit can wear the bank. */
#define SETTINGS_STORE_WRITE_TRIES      2U

/* Read granularity for scanning and CRC passes; sized for the stack of the
 * init thread and of any task that commits. */
#define SETTINGS_STORE_CHUNK            32U

#endif /* SETTINGS_SETTINGSSTORE_CONFIG_H_ */
