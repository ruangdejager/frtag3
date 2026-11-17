/*
 * SettingsStore.c
 *
 * Persistent store for Settings_EE on the external NOR. Implements the
 * SETTINGS_bStorageLoad / SETTINGS_bStorageSave hooks of the Platform settings
 * engine; partition and record format are in SettingsStore_Config.h.
 *
 * Modelled on the fr9's settings storage (hal_storage.c in fr9_application:
 * header + payload records appended into slots, newest wins), with the four
 * things that went wrong there designed out:
 *
 * 1. The fr9 erases BOTH of its pages before writing a fresh slot 0, so a
 *    power cut in that window loses every setting, the serial number
 *    included. Here two banks take turns: when the active bank is full the
 *    OTHER bank is erased and written, and the bank holding the current record
 *    is never erased while it is the newest copy. At every instant at least
 *    one valid record exists once one has been written.
 *
 * 2. The fr9 only looks at its newest slot; if that one is corrupt it falls
 *    back to defaults although older good records sit right beside it. Here
 *    the current record is the valid one with the highest sequence number, so
 *    a torn newest record simply leaves its predecessor current.
 *
 * 3. The fr9's CRC-32 is truncated to 16 bits. This one is kept whole.
 *
 * 4. The fr9 computes its program length as len/8 + 1 doublewords, which can
 *    run past the end of its 1 KB slot. Here the payload size is checked
 *    against the slot at compile time and again at run time.
 *
 * Write order inside a slot is payload first, header last. The magic lives in
 * the header, so a power cut anywhere before the header is complete leaves a
 * slot with no valid header - it is skipped as "used, invalid" (it is not all
 * 0xFF, so it is never mistaken for free and programmed over either). A torn
 * header fails its CRC the same way.
 *
 * Layout compatibility: Settings_EE may only grow at its end. A record shorter
 * than the running struct loads as a prefix and the new tail keeps its
 * compiled-in defaults with has_<field> false; a longer one (written by newer
 * firmware, then downgraded) loads the prefix this firmware understands. A
 * change that is not append-only bumps SETTINGS_LAYOUT_VERSION, and a record
 * with a different layout version is not loaded at all.
 *
 * Concurrency: the scan runs from SETTINGS_vInit() in the init thread, before
 * any other task reads settings. Saves are serialised by a mutex. The flash
 * driver serialises SPI access with the log and OTA on its own.
 */

#include "SettingsStore.h"
#include "SettingsStore_Config.h"
#include "settings.h"           /* the hooks implemented here, Settings_EE */
#include "Flash.h"
#include "Log.h"                /* LOG_FLASH_START_ADDR, for the overlap check */
#include "Fota_Config.h"        /* OTA_META_SECTOR_ADDR, for the overlap check */
#include "DbgLog.h"
#include "crc32.h"
#include "cmsis_os2.h"

#include <stddef.h>
#include <string.h>

/* ---- Compile-time guards ---- */
_Static_assert(sizeof(Settings_EE) <= SETTINGS_STORE_PAYLOAD_MAX,
               "Settings_EE no longer fits a settings slot - raise SETTINGS_STORE_SLOT_SIZE");
_Static_assert((SETTINGS_STORE_SLOT_SIZE % FLASH_PAGE_SIZE_BYTES) == 0U,
               "a settings slot must be a whole number of program pages");
_Static_assert((SETTINGS_STORE_BANK_SIZE % SETTINGS_STORE_SLOT_SIZE) == 0U,
               "a settings bank must be a whole number of slots");
_Static_assert(SETTINGS_STORE_SLOTS_PER_BANK <= 127U,
               "slot indices are tracked as int8_t");
_Static_assert(SETTINGS_STORE_REGION_END <= LOG_FLASH_START_ADDR,
               "settings partition overlaps the text log");
_Static_assert(SETTINGS_STORE_REGION_START >= (OTA_META_SECTOR_ADDR + FLASH_SECTOR_SIZE_BYTES),
               "settings partition overlaps the OTA metadata sector");

typedef struct {
    uint32_t u32Magic;
    uint16_t u16Layout;
    uint16_t u16Len;
    uint32_t u32Seq;
    uint32_t u32Crc;
} SettingsStore_Hdr_t;

_Static_assert(sizeof(SettingsStore_Hdr_t) == SETTINGS_STORE_HDR_LEN,
               "settings record header must be 16 bytes");

/* Bytes of the header covered by the CRC: everything before the CRC itself. */
#define SETTINGS_STORE_HDR_CRC_LEN   (offsetof(SettingsStore_Hdr_t, u32Crc))

typedef enum {
    SLOT_FREE = 0,     /* every byte 0xFF                                    */
    SLOT_VALID,        /* header and CRC check out                           */
    SLOT_INVALID,      /* used, but not a valid record (torn, foreign, bad)  */
} SlotState_e;

static const uint32_t au32BankAddr[2] = {
    SETTINGS_STORE_BANK_A_ADDR, SETTINGS_STORE_BANK_B_ADDR
};

static osMutexId_t s_xMutex       = NULL;
static bool        s_bScanned     = false;
static uint8_t     s_u8ActiveBank = 0U;  /* bank the next commit goes into  */
static uint8_t     s_u8NextSlot   = 0U;  /* SLOTS_PER_BANK = bank is full   */
static SettingsStore_Status_t s_tNewest; /* the current record              */

/* -------------------------------------------------------------------------- */

static uint32_t SETTINGS_STORE_u32SlotAddr(uint8_t u8Bank, uint8_t u8Slot)
{
    return au32BankAddr[u8Bank] + (uint32_t)u8Slot * SETTINGS_STORE_SLOT_SIZE;
}

/* Continue a CRC over a range of flash. False if the read failed. */
static bool SETTINGS_STORE_bCrcFlash(uint32_t u32Addr, uint32_t u32Len, uint32_t *pu32Crc)
{
    uint8_t au8Buf[SETTINGS_STORE_CHUNK];
    while (u32Len > 0U)
    {
        uint16_t n = (u32Len > sizeof(au8Buf)) ? (uint16_t)sizeof(au8Buf) : (uint16_t)u32Len;
        if (!FLASH_vRead(u32Addr, au8Buf, n))
            return false;
        *pu32Crc = CRC32_u32Update(*pu32Crc, au8Buf, n);
        u32Addr += n;
        u32Len  -= n;
    }
    return true;
}

/* True if every byte of the range reads 0xFF. A failed read counts as "not
 * blank" - the caller must never program over bytes it could not check. */
static bool SETTINGS_STORE_bIsBlank(uint32_t u32Addr, uint32_t u32Len)
{
    uint8_t au8Buf[SETTINGS_STORE_CHUNK];
    while (u32Len > 0U)
    {
        uint16_t n = (u32Len > sizeof(au8Buf)) ? (uint16_t)sizeof(au8Buf) : (uint16_t)u32Len;
        if (!FLASH_vRead(u32Addr, au8Buf, n))
            return false;
        for (uint16_t i = 0U; i < n; i++)
            if (au8Buf[i] != FLASH_ERASED_BYTE)
                return false;
        u32Addr += n;
        u32Len  -= n;
    }
    return true;
}

/* True if the flash range holds exactly these bytes. */
static bool SETTINGS_STORE_bFlashEquals(uint32_t u32Addr, const uint8_t *pu8, uint32_t u32Len)
{
    uint8_t au8Buf[SETTINGS_STORE_CHUNK];
    while (u32Len > 0U)
    {
        uint16_t n = (u32Len > sizeof(au8Buf)) ? (uint16_t)sizeof(au8Buf) : (uint16_t)u32Len;
        if (!FLASH_vRead(u32Addr, au8Buf, n) || (memcmp(au8Buf, pu8, n) != 0))
            return false;
        u32Addr += n;
        pu8     += n;
        u32Len  -= n;
    }
    return true;
}

/* Program a range that may span program pages; FLASH_vPageWrite must not
 * cross a page boundary. */
static bool SETTINGS_STORE_bProgram(uint32_t u32Addr, const uint8_t *pu8, uint32_t u32Len)
{
    while (u32Len > 0U)
    {
        uint32_t u32Room = FLASH_PAGE_SIZE_BYTES - (u32Addr % FLASH_PAGE_SIZE_BYTES);
        uint16_t n       = (uint16_t)((u32Len < u32Room) ? u32Len : u32Room);
        if (!FLASH_vPageWrite(u32Addr, pu8, n))
            return false;
        u32Addr += n;
        pu8     += n;
        u32Len  -= n;
    }
    return true;
}

static SlotState_e SETTINGS_STORE_eClassifySlot(uint32_t u32Addr, SettingsStore_Hdr_t *ptHdr)
{
    if (!FLASH_vRead(u32Addr, (uint8_t *)ptHdr, sizeof(*ptHdr)))
        return SLOT_INVALID;

    if (ptHdr->u32Magic == 0xFFFFFFFFUL && ptHdr->u32Crc == 0xFFFFFFFFUL &&
        ptHdr->u32Seq == 0xFFFFFFFFUL && ptHdr->u16Len == 0xFFFFU &&
        ptHdr->u16Layout == 0xFFFFU)
    {
        /* Header blank. Free only if the payload area is blank too: a power
         * cut between payload and header leaves exactly this, and those bytes
         * cannot be programmed over. */
        return SETTINGS_STORE_bIsBlank(u32Addr + SETTINGS_STORE_HDR_LEN,
                                       SETTINGS_STORE_PAYLOAD_MAX)
               ? SLOT_FREE : SLOT_INVALID;
    }

    if ((ptHdr->u32Magic != SETTINGS_STORE_MAGIC) ||
        (ptHdr->u16Len > SETTINGS_STORE_PAYLOAD_MAX))
        return SLOT_INVALID;

    uint32_t u32Crc = CRC32_u32Update(CRC32_INIT, (const uint8_t *)ptHdr,
                                            SETTINGS_STORE_HDR_CRC_LEN);
    if (!SETTINGS_STORE_bCrcFlash(u32Addr + SETTINGS_STORE_HDR_LEN, ptHdr->u16Len, &u32Crc))
        return SLOT_INVALID;

    return ((~u32Crc) == ptHdr->u32Crc) ? SLOT_VALID : SLOT_INVALID;
}

/* Find the current record and where the next commit goes. */
static void SETTINGS_STORE_vScan(void)
{
    int8_t ai8LastUsed[2] = { -1, -1 };
    SettingsStore_Hdr_t tHdr;

    s_tNewest.bHaveRecord = false;

    for (uint8_t b = 0U; b < 2U; b++)
    {
        for (uint8_t s = 0U; s < SETTINGS_STORE_SLOTS_PER_BANK; s++)
        {
            SlotState_e e = SETTINGS_STORE_eClassifySlot(SETTINGS_STORE_u32SlotAddr(b, s), &tHdr);
            if (e == SLOT_FREE)
                continue;

            ai8LastUsed[b] = (int8_t)s;
            if ((e == SLOT_VALID) &&
                (!s_tNewest.bHaveRecord || (tHdr.u32Seq > s_tNewest.u32Seq)))
            {
                s_tNewest.bHaveRecord = true;
                s_tNewest.u8Bank      = b;
                s_tNewest.u8Slot      = s;
                s_tNewest.u16Layout   = tHdr.u16Layout;
                s_tNewest.u16Len      = tHdr.u16Len;
                s_tNewest.u32Seq      = tHdr.u32Seq;
            }
        }
    }

    /* Append after the last used slot of the bank holding the current record,
     * never into a gap before it: a gap is either a slot that was free when a
     * later one was written (impossible in normal operation) or bytes nobody
     * can vouch for. With no record at all, start in bank A. */
    s_u8ActiveBank = s_tNewest.bHaveRecord ? s_tNewest.u8Bank : 0U;
    s_u8NextSlot   = (uint8_t)(ai8LastUsed[s_u8ActiveBank] + 1);
    s_bScanned     = true;
}

/* -------------------------------------------------------------------------- */

bool SETTINGS_bStorageLoad(void *pvDst, uint16_t u16Size, uint16_t u16Layout)
{
    if (s_xMutex == NULL)
        s_xMutex = osMutexNew(NULL);

    SETTINGS_STORE_vScan();

    if (!s_tNewest.bHaveRecord)
    {
        DBG_LOG("Settings: no stored record - running on defaults\r\n");
        return false;
    }
    if (s_tNewest.u16Layout != u16Layout)
    {
        DBG_LOG("Settings: stored record has layout %u, firmware expects %u - running on defaults\r\n",
                (unsigned)s_tNewest.u16Layout, (unsigned)u16Layout);
        return false;
    }

    const uint32_t u32Addr = SETTINGS_STORE_u32SlotAddr(s_tNewest.u8Bank, s_tNewest.u8Slot);
    const uint16_t u16Copy = (s_tNewest.u16Len < u16Size) ? s_tNewest.u16Len : u16Size;

    /* The scan verified the record, but on separate reads from these; check
     * again over the bytes actually handed to the caller, so a transient bad
     * read cannot load corrupt settings that a later commit would then make
     * permanent. One retry, then give up and let the caller use defaults. */
    for (uint8_t u8Try = 0U; u8Try < 2U; u8Try++)
    {
        SettingsStore_Hdr_t tHdr;
        if (!FLASH_vRead(u32Addr, (uint8_t *)&tHdr, sizeof(tHdr)) ||
            !FLASH_vRead(u32Addr + SETTINGS_STORE_HDR_LEN, (uint8_t *)pvDst, u16Copy))
            continue;

        uint32_t u32Crc = CRC32_u32Update(CRC32_INIT, (const uint8_t *)&tHdr,
                                                SETTINGS_STORE_HDR_CRC_LEN);
        u32Crc = CRC32_u32Update(u32Crc, (const uint8_t *)pvDst, u16Copy);
        /* A record longer than this firmware's struct: the tail is not loaded
         * but is still part of what the CRC covers. */
        if ((s_tNewest.u16Len > u16Copy) &&
            !SETTINGS_STORE_bCrcFlash(u32Addr + SETTINGS_STORE_HDR_LEN + u16Copy,
                                      (uint32_t)(s_tNewest.u16Len - u16Copy), &u32Crc))
            continue;

        if ((~u32Crc) == tHdr.u32Crc)
        {
            DBG_LOG("Settings: loaded record #%lu from bank %c slot %u (%u of %u bytes)\r\n",
                    (unsigned long)s_tNewest.u32Seq, (s_tNewest.u8Bank == 0U) ? 'A' : 'B',
                    (unsigned)s_tNewest.u8Slot, (unsigned)u16Copy, (unsigned)u16Size);
            return true;
        }
    }

    DBG_LOG("Settings: record #%lu failed its re-read check - running on defaults\r\n",
            (unsigned long)s_tNewest.u32Seq);
    return false;
}

bool SETTINGS_bStorageSave(const void *pvSrc, uint16_t u16Size, uint16_t u16Layout)
{
    const uint8_t *pu8Src = (const uint8_t *)pvSrc;
    bool bOk = false;

    if (u16Size > SETTINGS_STORE_PAYLOAD_MAX)
        return false;
    if ((s_xMutex != NULL) && (osMutexAcquire(s_xMutex, osWaitForever) != osOK))
        return false;

    if (!s_bScanned)
        SETTINGS_STORE_vScan();

    /* Nothing changed: spend no slot. This is what lets callers commit after
     * every settings fetch without wearing the bank down. */
    if (s_tNewest.bHaveRecord && (s_tNewest.u16Layout == u16Layout) &&
        (s_tNewest.u16Len == u16Size) &&
        SETTINGS_STORE_bFlashEquals(SETTINGS_STORE_u32SlotAddr(s_tNewest.u8Bank, s_tNewest.u8Slot) +
                                    SETTINGS_STORE_HDR_LEN, pu8Src, u16Size))
    {
        bOk = true;
        goto out;
    }

    for (uint8_t u8Try = 0U; u8Try < SETTINGS_STORE_WRITE_TRIES; u8Try++)
    {
        if (s_u8NextSlot >= SETTINGS_STORE_SLOTS_PER_BANK)
        {
            /* Active bank full: switch to the other one. The current record
             * lives in the active bank, so erasing the other never touches
             * it - and nothing ever erases the bank it is in. */
            const uint8_t u8Other = (uint8_t)(s_u8ActiveBank ^ 1U);
            /* Normally impossible: it takes every write in the active bank
             * failing since the last rotation. Then the current record is
             * still in the other bank, and erasing it would leave no valid
             * record at all. Refuse; the next boot's scan makes that bank
             * active again and rotation erases the failed one instead. */
            if (s_tNewest.bHaveRecord && (s_tNewest.u8Bank == u8Other))
            {
                DBG_LOG("Settings: bank %c full of failed writes - not saved\r\n",
                        (s_u8ActiveBank == 0U) ? 'A' : 'B');
                goto out;
            }
            if (!FLASH_vSectorErase(au32BankAddr[u8Other]))
            {
                DBG_LOG("Settings: erase of bank %c failed - not saved\r\n",
                        (u8Other == 0U) ? 'A' : 'B');
                s_tNewest.u16WriteFails++;
                goto out;
            }
            s_u8ActiveBank = u8Other;
            s_u8NextSlot   = 0U;
        }

        const uint8_t  u8Slot  = s_u8NextSlot++;
        const uint32_t u32Addr = SETTINGS_STORE_u32SlotAddr(s_u8ActiveBank, u8Slot);

        SettingsStore_Hdr_t tHdr = {
            .u32Magic  = SETTINGS_STORE_MAGIC,
            .u16Layout = u16Layout,
            .u16Len    = u16Size,
            .u32Seq    = s_tNewest.bHaveRecord ? (s_tNewest.u32Seq + 1U) : 1U,
            .u32Crc    = 0U,
        };
        uint32_t u32Crc = CRC32_u32Update(CRC32_INIT, (const uint8_t *)&tHdr,
                                                SETTINGS_STORE_HDR_CRC_LEN);
        tHdr.u32Crc = ~CRC32_u32Update(u32Crc, pu8Src, u16Size);

        /* Payload first, header last - see the file comment. */
        SettingsStore_Hdr_t tBack;
        if (SETTINGS_STORE_bProgram(u32Addr + SETTINGS_STORE_HDR_LEN, pu8Src, u16Size) &&
            SETTINGS_STORE_bProgram(u32Addr, (const uint8_t *)&tHdr, sizeof(tHdr)) &&
            (SETTINGS_STORE_eClassifySlot(u32Addr, &tBack) == SLOT_VALID) &&
            (tBack.u32Seq == tHdr.u32Seq) && (tBack.u32Crc == tHdr.u32Crc))
        {
            s_tNewest.bHaveRecord = true;
            s_tNewest.u8Bank      = s_u8ActiveBank;
            s_tNewest.u8Slot      = u8Slot;
            s_tNewest.u16Layout   = u16Layout;
            s_tNewest.u16Len      = u16Size;
            s_tNewest.u32Seq      = tHdr.u32Seq;
            DBG_LOG("Settings: saved record #%lu to bank %c slot %u\r\n",
                    (unsigned long)tHdr.u32Seq, (s_u8ActiveBank == 0U) ? 'A' : 'B',
                    (unsigned)u8Slot);
            bOk = true;
            goto out;
        }

        /* The slot is burnt either way; the next try takes a fresh one. */
        s_tNewest.u16WriteFails++;
        DBG_LOG("Settings: write-verify failed at bank %c slot %u\r\n",
                (s_u8ActiveBank == 0U) ? 'A' : 'B', (unsigned)u8Slot);
    }

out:
    if (s_xMutex != NULL)
        osMutexRelease(s_xMutex);
    return bOk;
}

void SETTINGS_STORE_vGetStatus(SettingsStore_Status_t *ptStatus)
{
    if (ptStatus != NULL)
        *ptStatus = s_tNewest;
}
