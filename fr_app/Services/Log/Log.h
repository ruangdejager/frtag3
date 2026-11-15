/*
 * Log.h
 *
 * Circular FIFO text log on external NOR flash (AT25EU0041A, 512 KB).
 *
 * FIFO semantics: u32WriteAddr is the head, u32StartAddr is the tail.
 * When the log wraps and a sector is erased, the tail advances past the
 * erased sector so the oldest data is transparently replaced.
 */

#ifndef SERVICES_LOG_LOG_H_
#define SERVICES_LOG_LOG_H_

#include <stdint.h>
#include <stdbool.h>

/* Log partition: 0x3C000-0x7CFFF, 260 KB of the 512 KB device.
 *
 * The region below (0x00000-0x3BFFF) belongs to the OTA image scratchpad +
 * metadata — see fr_app/Worker/Fota/Fota_Config.h for the full partition map.
 *
 * The top three 4 KB sectors were taken off the log:
 *
 *   0x7F000-0x7FFFF  flash self-test scratch (FLASH_SELFTEST_ADDR in
 *                    Device/Flash/Flash_Config.h). The self-test needs
 *                    somewhere it can erase and program destructively at
 *                    boot; erasing part of a staged firmware image on every
 *                    power cycle is not a trade anyone would take.
 *   0x7D000-0x7EFFF  LoRaWAN DevNonce journal, two sectors
 *                    (fr_app/Worker/LoRaWan/LoRaWan.c, LORAWAN_NVM_SECTOR0/1).
 *                    Two, not one (review LW01, 2026-11-13): with a single
 *                    sector the journal had to erase the only copy of the
 *                    nonce high-water mark before writing its replacement,
 *                    and a reset in that gap restarted DevNonce at 0, after
 *                    which the network server refuses every join. With two,
 *                    the sector holding the high-water mark is never the one
 *                    being erased. The second sector came off the top of the
 *                    log; on a unit that ran the single-sector build it holds
 *                    old log text, which the journal reads as "no records" and
 *                    erases on first rotation. LOG_vErase and LOG_vEraseAll
 *                    both leave these sectors alone (see LOG_vEraseAll).
 *
 * The bootloader is unaffected: it only ever reads the scratchpad and the
 * metadata sector, both far below this boundary. */
#define LOG_FLASH_START_ADDR       0x03C000UL
#define LORAWAN_NVM_SECTOR0_ADDR   0x07D000UL
#define LORAWAN_NVM_SECTOR1_ADDR   0x07E000UL
#define LOG_FLASH_END_ADDR         LORAWAN_NVM_SECTOR0_ADDR

/* Everything at or above this address is persistent state that no log or
 * maintenance erase may touch: the DevNonce journal and, above it, the
 * self-test sector (which the self-test manages itself). */
#define LOG_PROTECTED_START_ADDR   LORAWAN_NVM_SECTOR0_ADDR

/* The DevNonce sectors and the self-test sector must never overlap: the
 * self-test erases its sector at boot, which would wipe the journal and get
 * every join after the next reboot refused by the network server. */
#if (LORAWAN_NVM_SECTOR1_ADDR != LORAWAN_NVM_SECTOR0_ADDR + 0x1000UL) || \
    (LORAWAN_NVM_SECTOR1_ADDR + 0x1000UL) > 0x07F000UL
#  error "LoRaWAN DevNonce sectors must be two adjacent sectors below the flash self-test sector (0x7F000)"
#endif
#define LOG_FLASH_SIZE_BYTES    (LOG_FLASH_END_ADDR - LOG_FLASH_START_ADDR)

void     LOG_vInit(void);
void     LOG_vWrite(const char *buf, uint16_t len);

uint32_t LOG_u32GetUsedBytes(void);
uint8_t  LOG_u8GetUsedPercent(void);

/* Erase the log PARTITION only (per-sector, LOG_FLASH_START_ADDR..END_ADDR).
 * Leaves the OTA image scratchpad + metadata (below LOG_FLASH_START_ADDR)
 * untouched. */
void     LOG_vErase(void);

/* Erase the NOR device below LOG_PROTECTED_START_ADDR — log partition AND
 * the OTA image scratchpad + metadata below it. Destroys any
 * staged-but-not-yet-installed OTA image along with the log; only intended
 * for a deliberate field-unit reset ("tag flash eraseall"), not routine log
 * maintenance. The LoRaWAN DevNonce journal and the self-test sector are
 * kept (review LW04): it used to be a chip erase, which reset DevNonce. */
void     LOG_vEraseAll(void);

/* Release the NOR flash into deep-power-down for the idle/STOP2 period that
 * follows. Called by the DbgLog consumer as its last act before it blocks. */
void     LOG_vPark(void);

/* Stream the entire log FIFO (oldest → newest) out the debug transport via
 * DEBUG_vPutBuffer. Intended as a test/readback hook; call when no task is
 * concurrently writing the log (e.g. at boot right after LOG_vInit). */
void     LOG_vStreamToDebug(void);

/* Same content and chunking as LOG_vStreamToDebug, but each chunk is handed
 * to the given sink instead of hardcoding DEBUG_vPutBuffer -- e.g. FrKernel's
 * LoRa interface redirects "tag flash stream" over the radio instead of the
 * debug UART, since the requester has no UART session to read it from. */
void     LOG_vStreamViaSink(void (*sink)(const uint8_t *data, uint16_t len));

#endif /* SERVICES_LOG_LOG_H_ */
