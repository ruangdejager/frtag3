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

/* Log partition: 0x3C000-0x7DFFF, 264 KB of the 512 KB device.
 *
 * The region below (0x00000-0x3BFFF) belongs to the OTA image scratchpad +
 * metadata — see fr_app/Worker/Fota/Fota_Config.h for the full partition map.
 *
 * The top two 4 KB sectors were taken off the log:
 *
 *   0x7F000-0x7FFFF  flash self-test scratch (FLASH_SELFTEST_ADDR in
 *                    Device/Flash/Flash_Config.h). The self-test needs
 *                    somewhere it can erase and program destructively at
 *                    boot; erasing part of a staged firmware image on every
 *                    power cycle is not a trade anyone would take.
 *   0x7E000-0x7EFFF  LoRaWAN DevNonce record (fr_app/Worker/LoRaWan/LoRaWan.c,
 *                    LORAWAN_NVM_FLASH_ADDR). frtag2 put this at 0x7F000; on
 *                    frtag3 that sector was already the self-test's, so the
 *                    DevNonce record sits one sector lower. LOG_vErase leaves
 *                    it alone; LOG_vEraseAll (chip erase) does not - after a
 *                    "tag flash eraseall" the tag restarts its DevNonce at 0,
 *                    and TTN will refuse its joins until the device's used
 *                    DevNonces are reset in the TTN console.
 *
 * The bootloader is unaffected: it only ever reads the scratchpad and the
 * metadata sector, both far below this boundary. */
#define LOG_FLASH_START_ADDR    0x03C000UL
#define LORAWAN_NVM_FLASH_ADDR  0x07E000UL
#define LOG_FLASH_END_ADDR      LORAWAN_NVM_FLASH_ADDR

/* The DevNonce sector and the self-test sector must never overlap: the
 * self-test erases its sector at boot, which would reset the DevNonce and
 * get every join after the next reboot refused by the network server. */
#if (LORAWAN_NVM_FLASH_ADDR + 0x1000UL) > 0x07F000UL
#  error "LORAWAN_NVM_FLASH_ADDR overlaps the flash self-test sector (0x7F000)"
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

/* Erase the WHOLE NOR device — log partition AND the OTA image scratchpad +
 * metadata below it (a single chip-erase command, faster than per-sector
 * for the full 512 KB). Destroys any staged-but-not-yet-installed OTA
 * image along with the log; only intended for a deliberate full field-unit
 * reset ("tag flash eraseall"), not routine log maintenance. */
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
