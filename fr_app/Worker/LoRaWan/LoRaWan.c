/*
 * LoRaWan.c
 *
 * LoRaWAN uplink session for DISCOVERY_MODE_LORAWAN. See LoRaWan.h for the
 * contract; the notes here are about how the two radio drivers are made to
 * take turns.
 *
 * Radio hand-over
 * ---------------
 * The chip has one DIO-IRQ callback slot (radio_driver.c's SUBGRF_Init()).
 * The mesh driver takes it on every mesh wake (LORARADIO_vWakeUp ->
 * LORARADIO_DRIVER_vInit), LoRaMac's driver (radio.c) takes it on
 * Radio.Init(). So a session starts with Radio.Init(NULL) - "re-attach,
 * keep LoRaMac's events" (the NULL case is a small frtag2 change in radio.c)
 * - and ends by giving the mesh driver its callback back and deep-sleeping
 * the chip, which is the state the mesh left it in. A cold deep sleep keeps
 * no registers, so neither driver ever inherits the other's configuration;
 * each wake/attach configures the chip from scratch.
 *
 * Radio.Init() resets radio.c's idea of the sync word to "private", so the
 * public LoRaWAN sync word is re-applied explicitly after every attach.
 *
 * Where things run
 * ----------------
 *   radio IRQ    SUBGHZ ISR -> radio.c RadioOnDioIrq -> RADIO_IRQ_PROCESS()
 *                = LORAWAN_vRadioIrqFromIsr (radio_conf.h): latch + wake
 *                the session loop, which runs RadioIrqProcess() in task
 *                context. LoRaMac arms its RX-window timers from there, and
 *                those are FreeRTOS timers that must not be touched from an
 *                ISR.
 *   MAC timers   FreeRTOS timer task (Core/Src/timer_if.c).
 *   MAC process  the session loop below, in the AppTask, whenever
 *                LORAWAN_THREAD_FLAG is raised (MacProcessNotify) or on a
 *                short poll.
 *
 * The confirm/indication callbacks are only acted on while a request of
 * ours is outstanding: a session cut short by its cap can leave a radio.c
 * timeout timer armed, whose late event LoRaMac would otherwise report as
 * the completion of the NEXT session's request. The first thing each
 * session does is let LoRaMac swallow any such leftover with no request
 * open.
 */

#include "LoRaWan.h"

#include "LoRaMac.h"
#include "LoRaMacTest.h"
#include "secure-element.h"
#include "radio.h"
#include "timer.h"

#include "cmsis_os2.h"
#include "stm32wlxx_hal.h"
#include "stm32wlxx_ll_system.h"

#include "build_config.h"
#include "dbg_log.h"
#include "LoraRadio.h"
#include "Battery.h"
#include "Movement.h"
#include "GPS.h"
#include "version_config.h"

#include "Flash.h"
#include "Flash_Config.h"
#include "Log.h"          /* LORAWAN_NVM_SECTOR0/1_ADDR */

#include <string.h>

/* ---- MAC configuration ---- */

/* First join at DR0 (SF12): the farms are rural and the gateways are other
 * people's, so the join uses the longest-reach rate. ADR then walks the data
 * rate up (and the airtime down) once TTN has heard enough uplinks. */
#define LORAWAN_JOIN_DATARATE          DR_0

/* How far off its nominal start an RX window may open. Wider than ST's 20 ms
 * default because the window is opened by the FreeRTOS timer task (priority
 * 2), which a busy higher-priority task can hold off for a few ms, and the
 * TX-done that starts the RX delay is itself processed from a task, not the
 * ISR. A wider window costs a few ms of RX per uplink. */
#define LORAWAN_MAX_RX_ERROR_MS        30U

/* Poll period of the session loop when nothing has signalled it. */
#define LORAWAN_POLL_MS                20U

typedef enum
{
    LW_RESULT_PENDING = 0,
    LW_RESULT_OK,
    LW_RESULT_FAILED,
} LoRaWanResult_e;

/* ---- State (kept small: every byte here is .bss on a full part) ---- */
static LoRaMacPrimitives_t tMacPrimitives;
static LoRaMacCallback_t   tMacCallbacks;

static void           (*pfnRadioIrqProcess)(void) = NULL;
static volatile bool    bRadioIrqPending = false;
static osThreadId_t     volatile xSessionTask = NULL;

static bool             bMacReady      = false;
static bool             bRequestOpen   = false;   /* one of ours is outstanding */
static volatile uint8_t eResult        = LW_RESULT_PENDING;
/* DevNonce journal state (see "DevNonce persistence" below). */
static bool             bNvmKnown      = false;   /* high-water mark read OK  */
static uint16_t         u16NvmHigh     = 0U;      /* largest nonce on flash   */
static uint8_t          u8NvmActive    = 0U;      /* sector holding it        */
static uint16_t         u16NvmNextRec  = 0U;      /* next free slot in it     */

/* --------------------------------------------------------------------------
 * Radio IRQ deferral (radio_conf.h hooks)
 * -------------------------------------------------------------------------- */
void LORAWAN_vRegisterRadioIrqProcess(void (*pfnProcess)(void))
{
    pfnRadioIrqProcess = pfnProcess;
}

void LORAWAN_vRadioIrqFromIsr(void)
{
    bRadioIrqPending = true;
    osThreadId_t xTask = xSessionTask;
    if (xTask != NULL)
        osThreadFlagsSet(xTask, LORAWAN_THREAD_FLAG);
}

static void LORAWAN_vOnMacProcessNotify(void)
{
    osThreadId_t xTask = xSessionTask;
    if (xTask != NULL)
        osThreadFlagsSet(xTask, LORAWAN_THREAD_FLAG);
}

/* --------------------------------------------------------------------------
 * Identity
 * -------------------------------------------------------------------------- */

/* STM32WL UID64 as the DevEUI - same derivation as ST's reference apps:
 * the 64-bit UID is an IEEE EUI-64 in ST's OUI, unique per die. */
static void LORAWAN_vGetUniqueId(uint8_t *id)
{
    uint32_t val = LL_FLASH_GetUDN();
    if (val == 0xFFFFFFFFUL)   /* not expected on a production part */
    {
        uint32_t ID_1_3_val = HAL_GetUIDw0() + HAL_GetUIDw2();
        uint32_t ID_2_val   = HAL_GetUIDw1();

        id[7] = (uint8_t)(ID_1_3_val >> 24);
        id[6] = (uint8_t)(ID_1_3_val >> 16);
        id[5] = (uint8_t)(ID_1_3_val >> 8);
        id[4] = (uint8_t)(ID_1_3_val);
        id[3] = (uint8_t)(ID_2_val >> 24);
        id[2] = (uint8_t)(ID_2_val >> 16);
        id[1] = (uint8_t)(ID_2_val >> 8);
        id[0] = (uint8_t)(ID_2_val);
    }
    else
    {
        id[7] = (uint8_t)(val & 0xFFU);
        id[6] = (uint8_t)((val >> 8) & 0xFFU);
        id[5] = (uint8_t)((val >> 16) & 0xFFU);
        id[4] = (uint8_t)((val >> 24) & 0xFFU);
        val = LL_FLASH_GetDeviceID();
        id[3] = (uint8_t)(val & 0xFFU);
        val = LL_FLASH_GetSTCompanyID();
        id[2] = (uint8_t)(val & 0xFFU);
        id[1] = (uint8_t)((val >> 8) & 0xFFU);
        id[0] = (uint8_t)((val >> 16) & 0xFFU);
    }
}

/* Only consulted for ABP; OTAA gets its DevAddr from the JoinAccept. */
static void LORAWAN_vGetDevAddr(uint32_t *devAddr)
{
    *devAddr = LL_FLASH_GetUDN();
    if (*devAddr == 0xFFFFFFFFUL)
        *devAddr = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
}

void LORAWAN_vGetDevEui(uint8_t au8DevEui[8])
{
    LORAWAN_vGetUniqueId(au8DevEui);
}

/* 255 = "not able to measure", per the DevStatusAns definition. The battery
 * voltage itself is in every report payload. */
static uint8_t LORAWAN_u8GetBatteryLevel(void)
{
    return 255U;
}

/* --------------------------------------------------------------------------
 * DevNonce persistence
 *
 * LoRaWAN 1.0.4 DevNonce is a counter the network server remembers per
 * device: a JoinRequest whose DevNonce is not above every one it has seen
 * before is refused, and keeps being refused until someone resets the
 * device's nonce history in the TTN console. So the one hard rule here is
 * that a DevNonce that has gone on air is never used again, across any number
 * of resets, brown-outs and failed writes.
 *
 * History (review LW01, 2026-11-13, plus a worse finding while fixing it):
 *   - This code was ported from frtag2 inside #ifdef STORAGE_BACKEND_FLASH.
 *     frtag3 dropped that option (7122280, "Drop MicroSD"), so on this branch
 *     the whole journal compiled out: restore returned 0 and store returned
 *     false, i.e. every boot restarted DevNonce at 0 and only logged a
 *     warning. The guard is gone - flash is the only backend frtag3 has.
 *   - The journal was one sector. Rolling it over erased the only copy of the
 *     high-water mark before writing the replacement; a reset in that gap
 *     read back a blank sector and restarted at 0.
 *   - A read error or a torn record made restore return whatever it had seen
 *     so far, and a failed store still let the join go out.
 *
 * Journal: two adjacent 4 KB sectors (LORAWAN_NVM_SECTOR0/1_ADDR, Log.h).
 * Each holds append-only 8-byte records - magic "LW", nonce, ~nonce, 0x0000
 * - programmed only onto blank bytes; 8-byte alignment keeps every record
 * inside one 256-byte program page. The high-water mark is the LARGEST valid
 * nonce found in either sector (the nonce is monotonic, so "largest" and
 * "newest" agree, and it does not matter which record was written last).
 * Records go into the sector holding the high-water mark. When that sector
 * is full - or holds a torn or foreign record, so appending to it can no
 * longer be trusted - the OTHER sector is erased and the record is written
 * there. The sector holding the high-water mark is therefore never erased
 * while it is the only copy: a reset at any point leaves at least one
 * readable record at or above every nonce that has been sent.
 *
 * Reservation: LORAWAN_bStartJoin reserves the nonce the MAC is about to use
 * (current + 1; LoRaMacCryptoPrepareJoinRequest increments and then sends)
 * and sends the JoinRequest only if that record was written AND read back
 * intact. Anything less - flash absent, read error, erase/program failure,
 * readback mismatch, 16-bit space exhausted - refuses the join; the next
 * session tries again. A tag that cannot persist its nonce must not join,
 * because a join it cannot remember is exactly the one that locks it out
 * after the next reset.
 *
 * Both sectors blank is the only state that restarts at 0. That is a fresh
 * device; LOG_vEraseAll no longer erases these sectors (review LW04), so a
 * maintenance erase cannot produce it. Wear: 512 records per sector, one
 * sector erase per 512 joins, against the part's 10k-cycle rating.
 * -------------------------------------------------------------------------- */
#define LORAWAN_NVM_MAGIC              0x4C57U     /* "LW" */
#define LORAWAN_NVM_REC_BYTES          8U
#define LORAWAN_NVM_RECS_PER_SECTOR    (FLASH_SECTOR_SIZE_BYTES / LORAWAN_NVM_REC_BYTES)
#define LORAWAN_NVM_SECTORS            2U

static const uint32_t au32NvmSector[LORAWAN_NVM_SECTORS] = {
    LORAWAN_NVM_SECTOR0_ADDR, LORAWAN_NVM_SECTOR1_ADDR,
};

static bool LORAWAN_bNvmRecValid(const uint8_t *r, uint16_t *pu16Nonce)
{
    uint16_t u16Magic = (uint16_t)((r[0] << 8) | r[1]);
    uint16_t u16Nonce = (uint16_t)((r[2] << 8) | r[3]);
    uint16_t u16Inv   = (uint16_t)((r[4] << 8) | r[5]);
    if (u16Magic != LORAWAN_NVM_MAGIC || (uint16_t)~u16Nonce != u16Inv ||
        r[6] != 0x00U || r[7] != 0x00U)
        return false;
    *pu16Nonce = u16Nonce;
    return true;
}

static void LORAWAN_vNvmBuildRec(uint8_t r[LORAWAN_NVM_REC_BYTES], uint16_t u16Nonce)
{
    r[0] = (uint8_t)(LORAWAN_NVM_MAGIC >> 8);
    r[1] = (uint8_t)LORAWAN_NVM_MAGIC;
    r[2] = (uint8_t)(u16Nonce >> 8);
    r[3] = (uint8_t)u16Nonce;
    r[4] = (uint8_t)((uint16_t)~u16Nonce >> 8);
    r[5] = (uint8_t)~u16Nonce;
    r[6] = 0x00U;
    r[7] = 0x00U;
}

/* Scan one sector. Returns false only if it could not be read. On success:
 *   *pbAny     a valid record was found
 *   *pu16Max   the largest valid nonce (0 if none)
 *   *pu16Next  the slot to append to, or LORAWAN_NVM_RECS_PER_SECTOR when
 *              the sector must not be appended to: full, or anything that is
 *              neither a valid record nor blank (a torn record, old log text
 *              on a unit upgraded from the one-sector layout), or a written
 *              slot after a blank one. Valid records are still counted in
 *              all of those cases; only appending stops. */
static bool LORAWAN_bNvmScanSector(uint32_t u32Base, bool *pbAny,
                                   uint16_t *pu16Max, uint16_t *pu16Next)
{
    uint8_t  au8Buf[64];
    uint16_t u16FirstBlank = LORAWAN_NVM_RECS_PER_SECTOR;
    bool     bDirty        = false;

    *pbAny   = false;
    *pu16Max = 0U;

    for (uint16_t u16Rec = 0U; u16Rec < LORAWAN_NVM_RECS_PER_SECTOR; )
    {
        uint32_t u32Addr = u32Base + (uint32_t)u16Rec * LORAWAN_NVM_REC_BYTES;
        if (!FLASH_vRead(u32Addr, au8Buf, sizeof(au8Buf)))
            return false;

        for (uint16_t i = 0U; i < sizeof(au8Buf); i += LORAWAN_NVM_REC_BYTES, u16Rec++)
        {
            const uint8_t *r = &au8Buf[i];
            bool bBlank = true;
            for (uint8_t k = 0U; k < LORAWAN_NVM_REC_BYTES; k++)
                if (r[k] != 0xFFU) { bBlank = false; break; }

            uint16_t u16Nonce = 0U;
            if (bBlank)
            {
                if (u16FirstBlank == LORAWAN_NVM_RECS_PER_SECTOR)
                    u16FirstBlank = u16Rec;
            }
            else
            {
                if (u16FirstBlank != LORAWAN_NVM_RECS_PER_SECTOR)
                    bDirty = true;               /* written slot after a blank */
                if (LORAWAN_bNvmRecValid(r, &u16Nonce))
                {
                    if (!*pbAny || u16Nonce > *pu16Max)
                        *pu16Max = u16Nonce;
                    *pbAny = true;
                }
                else
                {
                    bDirty = true;               /* torn or foreign */
                }
            }
        }
    }

    *pu16Next = bDirty ? LORAWAN_NVM_RECS_PER_SECTOR : u16FirstBlank;
    return true;
}

/* Read both sectors and adopt the high-water mark. Returns false - leaving
 * bNvmKnown false, so no join can be sent - if either sector is unreadable:
 * a partial view could miss the larger of the two. */
static bool LORAWAN_bNvmRestore(void)
{
    bool     abAny[LORAWAN_NVM_SECTORS];
    uint16_t au16Max[LORAWAN_NVM_SECTORS];
    uint16_t au16Next[LORAWAN_NVM_SECTORS];

    bNvmKnown = false;
    for (uint8_t s = 0U; s < LORAWAN_NVM_SECTORS; s++)
    {
        if (!LORAWAN_bNvmScanSector(au32NvmSector[s], &abAny[s], &au16Max[s], &au16Next[s]))
        {
            DBG_LOG("lorawan: devNonce journal sector %u unreadable\r\n", (unsigned)s);
            return false;
        }
    }

    /* Active sector = the one holding the high-water mark. With no records
     * anywhere (fresh device) start in sector 0. */
    uint8_t u8Active = 0U;
    if (abAny[1] && (!abAny[0] || au16Max[1] > au16Max[0]))
        u8Active = 1U;

    u8NvmActive   = u8Active;
    u16NvmHigh    = abAny[u8Active] ? au16Max[u8Active] : 0U;
    u16NvmNextRec = au16Next[u8Active];
    bNvmKnown     = true;
    return true;
}

/* Durably reserve u16Nonce. True only once a record >= u16Nonce is on flash
 * and has been read back intact. */
static bool LORAWAN_bNvmReserve(uint16_t u16Nonce)
{
    if (!bNvmKnown && !LORAWAN_bNvmRestore())
        return false;

    if (u16Nonce <= u16NvmHigh)
        return true;                 /* already covered (e.g. a join the MAC
                                        refused before preparing it) */

    uint8_t  u8Sector = u8NvmActive;
    uint16_t u16Slot  = u16NvmNextRec;
    if (u16Slot >= LORAWAN_NVM_RECS_PER_SECTOR)
    {
        /* Rotate. The other sector holds nothing above u16NvmHigh, so
         * erasing it loses nothing; the active one is left intact. */
        u8Sector = (uint8_t)(u8NvmActive ^ 1U);
        if (!FLASH_vSectorErase(au32NvmSector[u8Sector]))
            return false;
        u16Slot = 0U;
    }

    uint8_t  au8Rec[LORAWAN_NVM_REC_BYTES];
    uint8_t  au8Back[LORAWAN_NVM_REC_BYTES];
    uint16_t u16Back = 0U;
    uint32_t u32Addr = au32NvmSector[u8Sector] + (uint32_t)u16Slot * LORAWAN_NVM_REC_BYTES;
    LORAWAN_vNvmBuildRec(au8Rec, u16Nonce);

    bool bOk = FLASH_vPageWrite(u32Addr, au8Rec, sizeof(au8Rec)) &&
               FLASH_vRead(u32Addr, au8Back, sizeof(au8Back)) &&
               LORAWAN_bNvmRecValid(au8Back, &u16Back) && (u16Back == u16Nonce);
    if (!bOk)
    {
        /* That slot may now hold a partial record: stop appending to this
         * sector. If it was the active one, the next attempt rotates; if it
         * was the freshly erased one, the next attempt erases it again. The
         * high-water mark is untouched either way. */
        if (u8Sector == u8NvmActive)
            u16NvmNextRec = LORAWAN_NVM_RECS_PER_SECTOR;
        return false;
    }

    u8NvmActive   = u8Sector;
    u16NvmNextRec = (uint16_t)(u16Slot + 1U);
    u16NvmHigh    = u16Nonce;
    return true;
}

static LoRaMacNvmData_t *LORAWAN_ptNvm(void)
{
    MibRequestConfirm_t mib;
    mib.Type = MIB_NVM_CTXS;
    LoRaMacMibGetRequestConfirm(&mib);
    return mib.Param.Contexts;
}

/* --------------------------------------------------------------------------
 * MAC primitives
 * -------------------------------------------------------------------------- */
static void LORAWAN_vMcpsConfirm(McpsConfirm_t *ptConfirm)
{
    if (!bRequestOpen)
        return;
    bRequestOpen = false;

    DBG_LOG("lorawan: uplink fcnt=%lu status=%u dr=%u txpwr=%u toa=%lums\r\n",
            (unsigned long)ptConfirm->UpLinkCounter, (unsigned)ptConfirm->Status,
            (unsigned)ptConfirm->Datarate, (unsigned)ptConfirm->TxPower,
            (unsigned long)ptConfirm->TxTimeOnAir);

    /* Unconfirmed uplink: OK means it went out and both RX windows closed. */
    eResult = (ptConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
            ? LW_RESULT_OK : LW_RESULT_FAILED;
}

static void LORAWAN_vMcpsIndication(McpsIndication_t *ptInd, LoRaMacRxStatus_t *ptRx)
{
    if (ptInd->Status != LORAMAC_EVENT_INFO_STATUS_OK)
        return;

    if (ptInd->RxData)
    {
        DBG_LOG("lorawan: downlink port=%u len=%u rssi=%d snr=%d fcntdown=%lu\r\n",
                (unsigned)ptInd->Port, (unsigned)ptInd->BufferSize,
                (int)ptRx->Rssi, (int)ptRx->Snr,
                (unsigned long)ptInd->DownLinkCounter);
        if (ptInd->Port != 0U)
            LORAWAN_vOnDownlink(ptInd->Port, ptInd->Buffer, ptInd->BufferSize);
    }
}

static void LORAWAN_vMlmeConfirm(MlmeConfirm_t *ptConfirm)
{
    if (ptConfirm->MlmeRequest != MLME_JOIN || !bRequestOpen)
        return;
    bRequestOpen = false;

    if (ptConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
    {
        MibRequestConfirm_t mib;
        mib.Type = MIB_DEV_ADDR;
        LoRaMacMibGetRequestConfirm(&mib);
        DBG_LOG("lorawan: JOINED devaddr=%08lX\r\n", (unsigned long)mib.Param.DevAddr);
        eResult = LW_RESULT_OK;
    }
    else
    {
        DBG_LOG("lorawan: join failed status=%u\r\n", (unsigned)ptConfirm->Status);
        eResult = LW_RESULT_FAILED;
    }
}

static void LORAWAN_vMlmeIndication(MlmeIndication_t *ptInd, LoRaMacRxStatus_t *ptRx)
{
    (void)ptRx;
    /* MLME_SCHEDULE_UPLINK (the network has MAC answers waiting) needs no
     * special handling: the next report, 15 min later, carries them. */
    (void)ptInd;
}

__attribute__((weak)) void LORAWAN_vOnDownlink(uint8_t u8Port, const uint8_t *pu8Data, uint8_t u8Len)
{
    (void)u8Port; (void)pu8Data; (void)u8Len;
}

/* --------------------------------------------------------------------------
 * One-time MAC bring-up (first session after boot, radio already awake)
 * -------------------------------------------------------------------------- */
static bool LORAWAN_bMacInit(void)
{
    MibRequestConfirm_t mib;

    if (UTIL_TIMER_Init() != UTIL_TIMER_OK)
    {
        DBG_LOG("lorawan: timer init FAILED\r\n");
        return false;
    }

    tMacPrimitives.MacMcpsConfirm    = LORAWAN_vMcpsConfirm;
    tMacPrimitives.MacMcpsIndication = LORAWAN_vMcpsIndication;
    tMacPrimitives.MacMlmeConfirm    = LORAWAN_vMlmeConfirm;
    tMacPrimitives.MacMlmeIndication = LORAWAN_vMlmeIndication;

    tMacCallbacks.GetBatteryLevel     = LORAWAN_u8GetBatteryLevel;
    tMacCallbacks.GetTemperatureLevel = NULL;
    tMacCallbacks.GetUniqueId         = LORAWAN_vGetUniqueId;
    tMacCallbacks.GetDevAddress       = LORAWAN_vGetDevAddr;
    tMacCallbacks.NvmDataChange       = NULL;
    tMacCallbacks.MacProcessNotify    = LORAWAN_vOnMacProcessNotify;

    /* Calls Radio.Init(&its own events), which attaches radio.c. */
    LoRaMacStatus_t eStatus = LoRaMacInitialization(&tMacPrimitives, &tMacCallbacks,
                                                    LORAMAC_REGION_EU868);
    if (eStatus != LORAMAC_STATUS_OK)
    {
        DBG_LOG("lorawan: MAC init FAILED status=%u\r\n", (unsigned)eStatus);
        return false;
    }

    if (SecureElementInitMcuID(LORAWAN_vGetUniqueId, LORAWAN_vGetDevAddr) != SECURE_ELEMENT_SUCCESS)
    {
        DBG_LOG("lorawan: SE id init FAILED\r\n");
        return false;
    }

    mib.Type = MIB_PUBLIC_NETWORK;
    mib.Param.EnablePublicNetwork = true;
    LoRaMacMibSetRequestConfirm(&mib);

    mib.Type = MIB_ADR;
    mib.Param.AdrEnable = true;
    LoRaMacMibSetRequestConfirm(&mib);

    mib.Type = MIB_SYSTEM_MAX_RX_ERROR;
    mib.Param.SystemMaxRxError = LORAWAN_MAX_RX_ERROR_MS;
    LoRaMacMibSetRequestConfirm(&mib);

    /* EU868 duty cycle is on by default (and must stay on); LoRaMac also
     * applies the join back-off from it, which is what keeps a tag with no
     * coverage from joining every 15 min forever. */

    /* Continue the DevNonce count from before this boot. If the journal
     * cannot be read now, DevNonce stays 0 here and bNvmKnown stays false:
     * LORAWAN_bStartJoin retries the read and refuses to join until it
     * succeeds, so the placeholder is never sent. */
    bool bNvmOk = LORAWAN_bNvmRestore();
    LORAWAN_ptNvm()->Crypto.DevNonce = bNvmOk ? u16NvmHigh : 0U;

    uint8_t au8Eui[8];
    LORAWAN_vGetUniqueId(au8Eui);
    DBG_LOG("lorawan: DevEUI=%02X%02X%02X%02X%02X%02X%02X%02X devNonce=%u%s\r\n",
            au8Eui[0], au8Eui[1], au8Eui[2], au8Eui[3],
            au8Eui[4], au8Eui[5], au8Eui[6], au8Eui[7], (unsigned)u16NvmHigh,
            bNvmOk ? "" : " (journal UNREADABLE - joins held)");

    bMacReady = true;
    return true;
}

bool LORAWAN_bIsJoined(void)
{
    if (!bMacReady)
        return false;
    MibRequestConfirm_t mib;
    mib.Type = MIB_NETWORK_ACTIVATION;
    if (LoRaMacMibGetRequestConfirm(&mib) != LORAMAC_STATUS_OK)
        return false;
    return mib.Param.NetworkActivation != ACTIVATION_TYPE_NONE;
}

/* --------------------------------------------------------------------------
 * Requests
 * -------------------------------------------------------------------------- */
static bool LORAWAN_bStartJoin(void)
{
    /* Persist the nonce this request will use BEFORE sending it: LoRaMac
     * increments DevNonce and then transmits, so reserving (current + 1)
     * first means a reset at any point after this - mid-join included -
     * restarts above every nonce that has gone on air. If the reservation
     * cannot be made durable, the join is NOT sent (review LW01): this used
     * to log a warning and join anyway, which is precisely the join a reset
     * would then repeat. See "DevNonce persistence" above. */
    if (!bNvmKnown)
    {
        if (!LORAWAN_bNvmRestore())
        {
            DBG_LOG("lorawan: join NOT sent - devNonce journal unreadable\r\n");
            return false;
        }
    }

    LoRaMacNvmData_t *ptNvm = LORAWAN_ptNvm();
    /* Never step below the journal: covers a restore that only succeeded
     * after init had left the 0 placeholder in the MAC. */
    if (ptNvm->Crypto.DevNonce < u16NvmHigh)
        ptNvm->Crypto.DevNonce = u16NvmHigh;

    /* 16-bit space exhausted: the next increment would wrap to 0, a value
     * the server has already seen. Only reprovisioning (new keys and a reset
     * nonce history on the server) can bring the tag back. */
    if (ptNvm->Crypto.DevNonce == 0xFFFFU)
    {
        DBG_LOG("lorawan: join NOT sent - devNonce space exhausted, reprovision required\r\n");
        return false;
    }

    uint16_t u16Next = (uint16_t)(ptNvm->Crypto.DevNonce + 1U);
    if (!LORAWAN_bNvmReserve(u16Next))
    {
        DBG_LOG("lorawan: join NOT sent - devNonce %u could not be persisted\r\n",
                (unsigned)u16Next);
        return false;
    }

    MlmeReq_t tReq;
    memset(&tReq, 0, sizeof(tReq));
    tReq.Type = MLME_JOIN;
    tReq.Req.Join.NetworkActivation = ACTIVATION_TYPE_OTAA;
    tReq.Req.Join.Datarate          = LORAWAN_JOIN_DATARATE;

    bRequestOpen = true;
    LoRaMacStatus_t eStatus = LoRaMacMlmeRequest(&tReq);
    if (eStatus != LORAMAC_STATUS_OK)
    {
        bRequestOpen = false;
        DBG_LOG("lorawan: join not sent status=%u wait=%lums\r\n",
                (unsigned)eStatus, (unsigned long)tReq.ReqReturn.DutyCycleWaitTime);
        return false;
    }
    DBG_LOG("lorawan: JoinRequest devNonce=%u dr=%u\r\n",
            (unsigned)u16Next, (unsigned)LORAWAN_JOIN_DATARATE);
    return true;
}

static uint8_t LORAWAN_u8BuildReport(uint8_t *p)
{
    uint32_t u32Id  = LORARADIO_u32GetUniqueId();
    uint16_t u16Bat = BAT_u16GetVoltage();
    uint8_t  u8Flags = (MOVE_eGetState() == MOVE_STATE_STILL) ? LORAWAN_REPORT_FLAG_STILL : 0U;

    /* Last known fix with its age, however old - same policy as the
     * basic-mode beacon this replaces: the server judges freshness. */
    gnss_coord_deg_t tLat, tLon;
    uint32_t u32AgeS = UINT32_MAX;
    bool bGps = GPS_bGetLastKnownFix(&tLat, &tLon, &u32AgeS);
    if (bGps)
        u8Flags |= LORAWAN_REPORT_FLAG_GPS_VALID;

    p[0] = (uint8_t)(u32Id >> 24);  p[1] = (uint8_t)(u32Id >> 16);
    p[2] = (uint8_t)(u32Id >> 8);   p[3] = (uint8_t)u32Id;
    p[4] = (uint8_t)(u16Bat >> 8);  p[5] = (uint8_t)u16Bat;
    p[6] = u8Flags;
    p[7] = (uint8_t)VERSION_SW_PATCH;

    if (!bGps)
        return LORAWAN_REPORT_LEN_BASE;

    uint32_t u32Lat = (uint32_t)tLat.i32MicroDeg;
    uint32_t u32Lon = (uint32_t)tLon.i32MicroDeg;
    p[8]  = (uint8_t)(u32Lat >> 24);  p[9]  = (uint8_t)(u32Lat >> 16);
    p[10] = (uint8_t)(u32Lat >> 8);   p[11] = (uint8_t)u32Lat;
    p[12] = (uint8_t)(u32Lon >> 24);  p[13] = (uint8_t)(u32Lon >> 16);
    p[14] = (uint8_t)(u32Lon >> 8);   p[15] = (uint8_t)u32Lon;
    p[16] = (uint8_t)(u32AgeS >> 24); p[17] = (uint8_t)(u32AgeS >> 16);
    p[18] = (uint8_t)(u32AgeS >> 8);  p[19] = (uint8_t)u32AgeS;
    return LORAWAN_REPORT_LEN_GPS;
}

static bool LORAWAN_bStartReport(void)
{
    /* Static: LoRaMac keeps a pointer to the buffer until the frame is
     * serialised, which is after this function returns. */
    static uint8_t au8Payload[LORAWAN_REPORT_LEN_GPS];
    uint8_t u8Len = LORAWAN_u8BuildReport(au8Payload);

    McpsReq_t tReq;
    memset(&tReq, 0, sizeof(tReq));

    LoRaMacTxInfo_t tTxInfo;
    if (LoRaMacQueryTxPossible(u8Len, &tTxInfo) != LORAMAC_STATUS_OK)
    {
        /* Pending MAC answers leave no room at this data rate: send them on
         * an empty frame now, the report goes next time. */
        tReq.Type = MCPS_UNCONFIRMED;
        tReq.Req.Unconfirmed.fBuffer     = NULL;
        tReq.Req.Unconfirmed.fBufferSize = 0U;
        DBG_LOG("lorawan: report does not fit (max %u) - flushing MAC commands\r\n",
                (unsigned)tTxInfo.MaxPossibleApplicationDataSize);
    }
    else
    {
        tReq.Type = MCPS_UNCONFIRMED;
        tReq.Req.Unconfirmed.fPort       = LORAWAN_REPORT_FPORT;
        tReq.Req.Unconfirmed.fBuffer     = au8Payload;
        tReq.Req.Unconfirmed.fBufferSize = u8Len;
    }
    tReq.Req.Unconfirmed.Datarate = LORAWAN_JOIN_DATARATE;   /* ignored while ADR is on */

    bRequestOpen = true;
    LoRaMacStatus_t eStatus = LoRaMacMcpsRequest(&tReq, false);
    if (eStatus != LORAMAC_STATUS_OK)
    {
        bRequestOpen = false;
        DBG_LOG("lorawan: report not sent status=%u wait=%lums\r\n",
                (unsigned)eStatus, (unsigned long)tReq.ReqReturn.DutyCycleWaitTime);
        return false;
    }
    DBG_LOG("lorawan: report queued len=%u gps=%u\r\n", (unsigned)u8Len,
            (unsigned)((au8Payload[6] & LORAWAN_REPORT_FLAG_GPS_VALID) != 0U));
    return true;
}

/* --------------------------------------------------------------------------
 * Session loop
 * -------------------------------------------------------------------------- */
static void LORAWAN_vProcess(void)
{
    if (bRadioIrqPending)
    {
        bRadioIrqPending = false;
        if (pfnRadioIrqProcess != NULL)
            pfnRadioIrqProcess();
    }
    LoRaMacProcess();
}

/* Run the MAC until the open request completes or the budget runs out. */
static bool LORAWAN_bRunUntilDone(uint32_t u32StartTick, uint32_t u32BudgetMs)
{
    for (;;)
    {
        LORAWAN_vProcess();

        if (eResult != LW_RESULT_PENDING)
            return eResult == LW_RESULT_OK;

        if ((osKernelGetTickCount() - u32StartTick) >= u32BudgetMs)
        {
            DBG_LOG("lorawan: session cap (%lu ms) hit - abandoning request\r\n",
                    (unsigned long)u32BudgetMs);
            bRequestOpen = false;
            return false;
        }

        (void)osThreadFlagsWait(LORAWAN_THREAD_FLAG, osFlagsWaitAny, LORAWAN_POLL_MS);
    }
}

bool LORAWAN_bSendReport(uint32_t u32BudgetMs)
{
    uint32_t u32Start = osKernelGetTickCount();
    bool     bOk      = false;

    if (u32BudgetMs > LORAWAN_SESSION_MAX_MS)
        u32BudgetMs = LORAWAN_SESSION_MAX_MS;

    /* Any IRQ latched since the last session belongs to that session. */
    bRadioIrqPending = false;
    (void)osThreadFlagsClear(LORAWAN_THREAD_FLAG);
    xSessionTask = osThreadGetId();

    if (!bMacReady)
    {
        if (!LORAWAN_bMacInit())
            goto done;
    }
    else
    {
        Radio.Init(NULL);   /* re-attach radio.c, keep LoRaMac's events */
    }
    Radio.SetPublicNetwork(true);

    /* Drain anything a previous, capped session left behind (see the file
     * header) before a request of ours is open to be mistaken for it. */
    bRequestOpen = false;
    LoRaMacProcess();

    LoRaMacStart();
    eResult = LW_RESULT_PENDING;

    if (!LORAWAN_bIsJoined())
    {
        /* A join session carries no report: EU868 duty cycle would hold the
         * data frame off for longer than the session is allowed to run. The
         * first report goes one period later. */
        if (LORAWAN_bStartJoin())
            bOk = LORAWAN_bRunUntilDone(u32Start, u32BudgetMs);
    }
    else
    {
        if (LORAWAN_bStartReport())
            bOk = LORAWAN_bRunUntilDone(u32Start, u32BudgetMs);
    }

done:
    if (bMacReady)
        (void)LoRaMacHalt();   /* stops the MAC timers, radio to sleep */

    xSessionTask = NULL;

    /* Back to the mesh driver: its DIO callback, its configuration, then a
     * cold deep sleep - the state a mesh wake expects to find. */
    LORARADIO_vWakeUp();
    LORARADIO_vEnterDeepSleep();

    DBG_LOG("lorawan: session end ok=%u joined=%u %lu ms\r\n",
            (unsigned)bOk, (unsigned)LORAWAN_bIsJoined(),
            (unsigned long)(osKernelGetTickCount() - u32Start));
    return bOk;
}
