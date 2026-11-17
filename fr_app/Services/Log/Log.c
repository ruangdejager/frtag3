/*
 * Log.c
 *
 * Circular FIFO text log on external NOR flash.
 *
 * Synchronous flash primitive: LOG_vWrite() programs bytes directly. It is
 * called only from the single DbgLog consumer task (see DbgLog.c), which owns
 * all buffering — so callers of DBG/LOG/DBG_LOG never touch the flash and never
 * see its latency.
 *
 * Flash FIFO: u32WriteAddr is the head, u32StartAddr is the tail. On every
 * sector-boundary crossing the sector is erased first; once wrapped, the tail
 * advances past each freshly-erased sector so FIFO order is maintained.
 *
 * Init: binary search at sector granularity to find the first erased sector,
 * then a byte-scan within it for the exact write head.
 */

#include "build_config.h"


#include "Log.h"
#include "Flash.h"
#include "Flash_Config.h"
#include "Debug.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* Number of 4 KB sectors in the log PARTITION (not the whole device — the
 * lower sectors belong to the OTA scratchpad, see OtaStore_Config.h). */
#define LOG_NUM_SECTORS  (LOG_FLASH_SIZE_BYTES / FLASH_SECTOR_SIZE_BYTES)

static uint32_t u32WriteAddr;
static uint32_t u32StartAddr;
static bool     bWrapped;

/* -------------------------------------------------------------------------- */

/*
 * Returns the byte at offset (FLASH_SECTOR_SIZE_BYTES - 1) of the given
 * sector — i.e. its last byte.
 */
static uint8_t LOG_u8ReadSectorTailByte(uint32_t sector)
{
    uint8_t  byte;
    uint32_t addr = LOG_FLASH_START_ADDR + sector * FLASH_SECTOR_SIZE_BYTES
                  + (FLASH_SECTOR_SIZE_BYTES - 1U);
    FLASH_vRead(addr, &byte, 1);
    return byte;
}

/*
 * Find the FIFO write-head sector: the one (and only) sector that still has
 * unused 0xFF padding at its tail. All other sectors — whether holding the
 * current generation's data or an older, not-yet-overwritten generation —
 * are filled right to their last byte. Returns LOG_NUM_SECTORS if no such
 * sector is found (the degenerate case where every sector is filled to
 * exactly its last byte).
 */
static uint32_t LOG_u32FindHeadByTailPadding(void)
{
    for (uint32_t sector = 0U; sector < LOG_NUM_SECTORS; sector++)
    {
        if (LOG_u8ReadSectorTailByte(sector) == 0xFFU)
            return sector;
    }
    return LOG_NUM_SECTORS;
}

/* -------------------------------------------------------------------------- */

void LOG_vInit(void)
{
    /* Pass 1: first byte of every sector. A sector whose first byte is
     * 0xFF is virgin — never written since the last chip erase. */
    uint32_t u32FirstVirgin  = LOG_NUM_SECTORS;
    uint32_t u32VirginCount  = 0U;

    for (uint32_t sector = 0U; sector < LOG_NUM_SECTORS; sector++)
    {
        uint8_t  byte;
        FLASH_vRead(LOG_FLASH_START_ADDR + sector * FLASH_SECTOR_SIZE_BYTES, &byte, 1);
        if (byte == 0xFFU)
        {
            if (u32FirstVirgin == LOG_NUM_SECTORS)
                u32FirstVirgin = sector;
            u32VirginCount++;
        }
    }

    uint32_t u32HeadSector;
    uint32_t u32TailSector;

    if ((u32FirstVirgin != LOG_NUM_SECTORS) &&
        (u32VirginCount == (LOG_NUM_SECTORS - u32FirstVirgin)))
    {
        /* The virgin sectors form a clean suffix up to the partition end — the
         * FIFO has never wrapped. The write head is the last *written*
         * sector (or sector 0 if the log is completely empty); the tail
         * (oldest data) is fixed at sector 0. */
        u32HeadSector = (u32FirstVirgin == 0U) ? 0U : (u32FirstVirgin - 1U);
        u32TailSector = 0U;
        bWrapped      = false;
    }
    else
    {
        /* The FIFO has wrapped at least once: every sector holds data from
         * some generation, so the write head can no longer be identified by
         * its first byte. Find it instead by its trailing 0xFF padding. The
         * sector right after it holds the oldest surviving data. */
        u32HeadSector = LOG_u32FindHeadByTailPadding();
        if (u32HeadSector == LOG_NUM_SECTORS)
        {
            /* Degenerate: every sector filled to its last byte. Fall back to
             * sector 0 — the next write will erase it and re-establish a
             * normal FIFO layout. */
            u32HeadSector = 0U;
        }
        u32TailSector = (u32HeadSector + 1U) % LOG_NUM_SECTORS;
        bWrapped      = true;
    }

    /* Fine-scan the head sector for the exact write-pointer byte offset. */
    uint32_t sectorStart = LOG_FLASH_START_ADDR + u32HeadSector * FLASH_SECTOR_SIZE_BYTES;
    uint32_t sectorEnd   = sectorStart + FLASH_SECTOR_SIZE_BYTES;
    uint32_t addr        = sectorStart;
    uint8_t  byte;

    while (addr < sectorEnd)
    {
        FLASH_vRead(addr, &byte, 1);
        if (byte == 0xFFU) break;
        addr++;
    }

    u32WriteAddr = addr;
    u32StartAddr = LOG_FLASH_START_ADDR + u32TailSector * FLASH_SECTOR_SIZE_BYTES;
}

/* -------------------------------------------------------------------------- */

/*
 * Writes are batched into page-aligned chunks (one FLASH_vPageWrite per
 * chunk) instead of one SPI page-program transaction per byte. A 50-byte
 * log line used to mean 50 separate select/cmd/wait-ready SPI transactions
 * — each one a window in which a campaign burst could preempt the consumer
 * task past the SPI timeout and leave a truncated command on the bus. One
 * (or, across a page boundary, two) transactions per line closes almost all
 * of that window. If a chunk write fails (device busy/unresponsive beyond
 * the bounded wait), the remainder of this call is dropped rather than
 * risking a write at a stale address.
 */
void LOG_vWrite(const char *buf, uint16_t len)
{
    /* There is deliberately no "suspend flash logging during OTA" path here.
     * A LOG_vSuspend()/bLogSuspended flag used to exist and was never read by
     * this function, so its three OTA call sites implied a protection that did
     * not exist. It has been removed rather than implemented: suspending was
     * tried, did not fix the transfer corruption (the read-priming fix in
     * FOTA_bSendChunk did), and it blinded the field-collected primary log for
     * the whole OTA window. It also cannot corrupt an image — every write here
     * is confined to the log partition by the range check below, which is far
     * above the OTA scratch area. */
    uint16_t off = 0U;

    while (off < len)
    {
        /* Never touch storage outside the log partition.
         *
         * u32WriteAddr is a zero-initialised static, and 0 is
         * OTA_SCRATCH_START_ADDR - the base of the staged firmware image. So
         * a LOG_vWrite() that ran before LOG_vInit() had set the head would
         * see 0 % FLASH_SECTOR_SIZE_BYTES == 0, sector-erase the image's
         * first 4 KB, and then write log text over it. That is reachable:
         * the DbgLog consumer runs at osPriorityAboveNormal and preempts the
         * osPriorityNormal init task the instant a DBG_LOG is queued, and
         * init.c had DBG_LOG calls ahead of LOG_vInit(). It only ever
         * destroyed a staged image on a RESET (boot-only code), which is
         * exactly the "image verifies fine until the tag is power-cycled"
         * signature this was found from.
         *
         * init.c now runs LOG_vInit() before the first DBG_LOG, so the
         * ordering hole is closed at the call site too - but this check is
         * what makes it impossible rather than merely unlikely, and it costs
         * one compare per chunk against a failure mode that silently
         * corrupts firmware. Checked per iteration, not once on entry, so it
         * holds for every erase/write no matter how this loop is edited. */
        if ((u32WriteAddr < LOG_FLASH_START_ADDR) ||
            (u32WriteAddr >= LOG_FLASH_END_ADDR))
            return;

        /* Erase the sector when crossing into a new one */
        if ((u32WriteAddr % FLASH_SECTOR_SIZE_BYTES) == 0U)
        {
            if (!FLASH_vSectorErase(u32WriteAddr))
                return;

            if (bWrapped)
            {
                /* Advance tail past the freshly-erased sector */
                u32StartAddr = u32WriteAddr + FLASH_SECTOR_SIZE_BYTES;
                if (u32StartAddr >= LOG_FLASH_END_ADDR)
                    u32StartAddr = LOG_FLASH_START_ADDR;
            }
        }

        uint32_t u32ToPageEnd   = FLASH_PAGE_SIZE_BYTES   - (u32WriteAddr % FLASH_PAGE_SIZE_BYTES);
        uint32_t u32ToSectorEnd = FLASH_SECTOR_SIZE_BYTES - (u32WriteAddr % FLASH_SECTOR_SIZE_BYTES);
        uint32_t u32Max         = (u32ToPageEnd < u32ToSectorEnd) ? u32ToPageEnd : u32ToSectorEnd;

        uint16_t u16Chunk = (uint16_t)(len - off);
        if (u16Chunk > u32Max)
            u16Chunk = (uint16_t)u32Max;

        if (!FLASH_vPageWrite(u32WriteAddr, (const uint8_t *)&buf[off], u16Chunk))
            return;

        u32WriteAddr += u16Chunk;
        off          += u16Chunk;

        if (u32WriteAddr >= LOG_FLASH_END_ADDR)
        {
            u32WriteAddr = LOG_FLASH_START_ADDR;
            bWrapped     = true;
        }
    }
}

/* -------------------------------------------------------------------------- */

uint32_t LOG_u32GetUsedBytes(void)
{
    if (!bWrapped)
        return u32WriteAddr - LOG_FLASH_START_ADDR;

    /* Wrapped with the head sitting exactly on the tail: the partition is
     * FULL, not empty. It happens whenever a write ends on the last byte of
     * the sector just before the tail - LOG_vWrite has already erased that
     * sector and moved the tail past it, so the head lands on the tail's first
     * byte. The general branch below read that as 0 bytes used, so a stream
     * requested in that state sent nothing at all until the next log line
     * moved the head, which on a sleeping tag can be a whole wake interval. */
    if (u32WriteAddr == u32StartAddr)
        return LOG_FLASH_SIZE_BYTES;

    if (u32WriteAddr >= u32StartAddr)
        return u32WriteAddr - u32StartAddr;

    return (LOG_FLASH_END_ADDR - u32StartAddr) + (u32WriteAddr - LOG_FLASH_START_ADDR);
}

uint8_t LOG_u8GetUsedPercent(void)
{
    return (uint8_t)(LOG_u32GetUsedBytes() * 100UL / LOG_FLASH_SIZE_BYTES);
}

/* -------------------------------------------------------------------------- */

void LOG_vErase(void)
{
    /* Per-sector erase of the log PARTITION only. A chip erase would also
     * wipe the OTA image scratchpad + metadata in the lower sectors. */
    for (uint32_t u32Addr = LOG_FLASH_START_ADDR;
         u32Addr < LOG_FLASH_END_ADDR;
         u32Addr += FLASH_SECTOR_SIZE_BYTES)
    {
        if (!FLASH_vSectorErase(u32Addr))
            break;
    }
    u32WriteAddr = LOG_FLASH_START_ADDR;
    u32StartAddr = LOG_FLASH_START_ADDR;
    bWrapped     = false;
}

void LOG_vEraseAll(void)
{
    /* Whole-device erase: also wipes the OTA scratchpad + metadata sectors
     * and the stored settings below LOG_FLASH_START_ADDR — deliberate, see
     * Log.h; the settings store copes with its banks vanishing under it and
     * rewrites on the next commit. FLASH_vChipErase
     * blocks until the device reports done (a full 512 KB erase takes
     * several seconds); Flash.c's own mutex keeps this safe against the
     * DbgLog consumer's other flash traffic, but this function is still
     * only ever called FROM that consumer task (DBGLOG_vRequestEraseAll),
     * same as LOG_vErase, so there's nothing else to race here anyway. */
    (void)FLASH_vChipErase();
    u32WriteAddr = LOG_FLASH_START_ADDR;
    u32StartAddr = LOG_FLASH_START_ADDR;
    bWrapped     = false;
}

/* -------------------------------------------------------------------------- */

static volatile bool s_bStreamAbort = false;

void LOG_vAbortStream(void)
{
    s_bStreamAbort = true;
}

/* DEBUG_vPutBuffer adapted to the LogSink_t shape: the UART ring never
 * refuses a chunk, so this sink never stops a stream on its own. */
static bool LOG_bDebugSink(const uint8_t *data, uint16_t len)
{
    DEBUG_vPutBuffer(data, len);
    return true;
}

void LOG_vStreamToDebug(uint8_t u8Percent)
{
    LOG_vStreamViaSink(LOG_bDebugSink, u8Percent, NULL);
}

void LOG_vStreamViaSink(LogSink_t sink, uint8_t u8Percent,
                        LogStreamResult_t *pResult)
{
    LogStreamResult_t tRes = {0};
    s_bStreamAbort = false;

    if ((u8Percent == 0U) || (u8Percent > 100U))
        u8Percent = 100U;

    /* Snapshot once. Nothing can move the head or tail while this runs - the
     * caller is the DbgLog consumer, the log's only writer - but reading the
     * pair once keeps the window self-consistent by construction rather than
     * by that convention alone. */
    const uint32_t u32Used = LOG_u32GetUsedBytes();

    /* The newest u8Percent % is the window that ENDS at the head, so it
     * starts (100 - u8Percent) % of the way from the tail. Computed in 64 bits
     * so a full 268 KB partition times 100 cannot overflow. */
    const uint32_t u32Skip = (uint32_t)(((uint64_t)u32Used * (100U - u8Percent)) / 100U);
    uint32_t u32Remaining  = u32Used - u32Skip;
    uint32_t u32Addr       = u32StartAddr + u32Skip;   /* FIFO tail = oldest byte */
    if (u32Addr >= LOG_FLASH_END_ADDR)
        u32Addr -= LOG_FLASH_SIZE_BYTES;               /* wrap */

    tRes.u32Total = u32Remaining;

    char hdr[80];
    int  n = snprintf(hdr, sizeof(hdr),
                      "\r\n==== EXT-FLASH LOG DUMP (latest %u%%: %lu of %lu bytes) ====\r\n",
                      (unsigned)u8Percent, (unsigned long)u32Remaining,
                      (unsigned long)u32Used);
    if ((n > 0) && !sink((const uint8_t *)hdr, (uint16_t)n))
        tRes.bAborted = true;

    /* 200 B: comfortably under the FrKernel v2 per-frame payload
     * (FRK_MAX_PAYLOAD, 246 B), so a chunk maps to exactly one radio packet,
     * while still being far fewer, larger flash reads / sink calls than the
     * old 64 B chunking (which throttled throughput over both the debug UART
     * and, worse, LoRa). A sink that needs a smaller unit may split further. */
    uint8_t  au8Chunk[200];
    /* A partial stream starts inside some line; drop the bytes up to the
     * first newline so the operator's first line is a whole one. Only the
     * first chunk is searched - a 200-byte line does not exist in this log,
     * and if it did the cost is one partial line, not a lost stream. */
    bool     bTrimToLine = (u32Skip > 0U);

    while ((u32Remaining > 0U) && !tRes.bAborted)
    {
        if (s_bStreamAbort)
        {
            tRes.bAborted = true;
            break;
        }

        uint16_t u16Chunk = (u32Remaining > sizeof(au8Chunk))
                          ? (uint16_t)sizeof(au8Chunk)
                          : (uint16_t)u32Remaining;

        if ((u32Addr + u16Chunk) > LOG_FLASH_END_ADDR)
            u16Chunk = (uint16_t)(LOG_FLASH_END_ADDR - u32Addr);

        /* FLASH_vRead fails when the device has gated off, when it never
         * comes ready, on an SPI error, or (ENABLE_FLASH_READ_CRC) when two
         * reads of the same range disagree. The result used to be ignored and
         * the buffer sent anyway - on a gated-off part that meant the whole
         * log's worth of whatever the stack buffer last held, presented as
         * the log. Retry once (a read-verify mismatch is usually transient),
         * then send an explicit marker in place of the bytes, so a hole in
         * the log is visible as a hole. */
        bool bRead = FLASH_vRead(u32Addr, au8Chunk, u16Chunk);
        if (!bRead)
            bRead = FLASH_vRead(u32Addr, au8Chunk, u16Chunk);

        bool bSinkOk;
        if (bRead)
        {
            const uint8_t *pu8Out = au8Chunk;
            uint16_t       u16Out = u16Chunk;
            if (bTrimToLine)
            {
                bTrimToLine = false;
                for (uint16_t i = 0U; i < u16Chunk; i++)
                {
                    if (au8Chunk[i] == '\n')
                    {
                        pu8Out = &au8Chunk[i + 1U];
                        u16Out = (uint16_t)(u16Chunk - i - 1U);
                        break;
                    }
                }
            }
            bSinkOk = (u16Out == 0U) || sink(pu8Out, u16Out);
        }
        else
        {
            if (tRes.u16ReadFails < UINT16_MAX)
                tRes.u16ReadFails++;
            n = snprintf(hdr, sizeof(hdr), "\r\n[read fail @0x%06lX, %u bytes skipped]\r\n",
                         (unsigned long)u32Addr, (unsigned)u16Chunk);
            bSinkOk = (n <= 0) || sink((const uint8_t *)hdr, (uint16_t)n);
        }

        if (!bSinkOk)
        {
            tRes.bAborted = true;
            break;
        }

        tRes.u32Bytes += u16Chunk;
        u32Addr += u16Chunk;
        if (u32Addr >= LOG_FLASH_END_ADDR)
            u32Addr = LOG_FLASH_START_ADDR;      /* wrap */

        u32Remaining -= u16Chunk;
    }

    /* Footer even on an abort (unless the sink itself is what failed, in
     * which case it would only fail again), so a reader of a UART capture can
     * tell a stream that finished from one that was cut. */
    if (!tRes.bAborted || s_bStreamAbort)
    {
        n = snprintf(hdr, sizeof(hdr), "\r\n==== EXT-FLASH LOG DUMP %s ====\r\n",
                     tRes.bAborted ? "ABORTED" : "END");
        if (n > 0)
            (void)sink((const uint8_t *)hdr, (uint16_t)n);
    }

    s_bStreamAbort = false;
    if (pResult != NULL)
        *pResult = tRes;
}

/* -------------------------------------------------------------------------- */

void LOG_vPark(void)
{
    /* Park the NOR flash in deep-power-down for the idle/STOP2 period. No-op
     * if it was never woken; the next flash access transparently resumes it. */
    FLASH_vDeepPowerDown();
}

