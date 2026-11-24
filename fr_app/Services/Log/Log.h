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

/* Log partition: 0x40000-0x7CFFF, 244 KB of the 512 KB device.
 *
 * The region below belongs to the OTA image scratchpad + metadata
 * (0x00000-0x3BFFF, fr_app/Worker/Fota/Fota_Config.h) and the unit's settings
 * and identity (0x3C000-0x3FFFF). The full NOR map is in
 * fr_app/settings/SettingsStore_Config.h.
 *
 * The log used to start at 0x3C000; its bottom 16 KB went to the settings
 * partition, because both flashes were otherwise fully allocated and the log
 * is the one region whose size is not a contract with the bootloader. On a
 * unit that ran older firmware the old log text left in 0x3C000-0x3FFFF is
 * not mistaken for settings - it carries no valid record header - and is
 * erased the first time the settings store rotates into each bank.
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
#define LOG_FLASH_START_ADDR       0x040000UL
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

/* Erase the NOR device below LOG_PROTECTED_START_ADDR — log partition, the
 * settings + identity mirror sectors, AND
 * the OTA image scratchpad + metadata below it. Destroys any
 * staged-but-not-yet-installed OTA image along with the log; only intended
 * for a deliberate field-unit reset (kernel command 'W'), not routine log
 * maintenance. The LoRaWAN DevNonce journal and the self-test sector are
 * kept (review LW04): it used to be a chip erase, which reset DevNonce. */
void     LOG_vEraseAll(void);

/* Release the NOR flash into deep-power-down for the idle/STOP2 period that
 * follows. Called by the DbgLog consumer as its last act before it blocks. */
void     LOG_vPark(void);

/* A stream sink takes one chunk and returns false to stop the stream early:
 * the LoRa sink returns false when the radio could not take the packet in a
 * bounded time, so a stream whose session has gone away (radio asleep, kernel
 * exited) ends instead of blocking the DbgLog consumer forever. */
typedef bool (*LogSink_t)(const uint8_t *data, uint16_t len);

/* What a stream did, for the caller's end-of-stream report. */
typedef struct {
    uint32_t u32Bytes;       /* log bytes handed to the sink                 */
    uint32_t u32Total;       /* bytes the stream set out to send             */
    uint16_t u16ReadFails;   /* chunks that could not be read and were
                              * replaced by a "[read fail ...]" marker       */
    bool     bAborted;       /* LOG_vAbortStream() or the sink stopped it    */
} LogStreamResult_t;

/* Stream the entire log FIFO (oldest → newest) out the debug transport via
 * DEBUG_vPutBuffer. Intended as a test/readback hook; call when no task is
 * concurrently writing the log (e.g. at boot right after LOG_vInit). */
void     LOG_vStreamToDebug(uint8_t u8Percent);

/* Stream the newest u8Percent % of the log (100 = all of it, oldest to
 * newest) through the given sink. A partial stream starts at the first line
 * boundary inside its window, so it never opens mid-line. Same content and
 * chunking as LOG_vStreamToDebug - e.g. FrKernel's LoRa interface sends it
 * over the radio, since the requester has no UART session to read it from.
 *
 * Must run on the DbgLog consumer (see DBGLOG_bRequestDumpVia): that task is
 * the only writer of the log, so nothing can erase the sector being read.
 * pResult may be NULL. */
void     LOG_vStreamViaSink(LogSink_t sink, uint8_t u8Percent,
                            LogStreamResult_t *pResult);

/* Ask a running stream to stop at its next chunk. Safe from any task; a no-op
 * when nothing is streaming. Cleared when the next stream starts. */
void     LOG_vAbortStream(void);

#endif /* SERVICES_LOG_LOG_H_ */
