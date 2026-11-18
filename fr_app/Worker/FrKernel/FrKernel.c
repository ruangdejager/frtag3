/*
 * FrKernel.c
 *
 * Device command interface Worker - the TAG side of kernelV2. The operator's
 * end lives in FrKernelConsole.c; the frame format and the single-letter
 * command table both ends share live in FrKernel_Protocol.h and
 * FrKernel_Commands.c.
 *
 * What changed from v1, and why
 * -----------------------------
 * v1 opened a "session" on ANY addressed command, at any time the radio
 * happened to be listening - including in the middle of a discovery campaign,
 * on every tag that heard a "tag -devicereq" scan, and on every tag that heard
 * a bulk "tag * ..." command. Each of those sessions held its tag awake for
 * five minutes, and nothing stood the rest of the firmware down while it ran:
 * campaigns, GPS, scheduled wakes and the kernel all fought over the radio,
 * and the campaign-end TX flush then threw the kernel's replies away. Bulk
 * mode was supposed to be silent but two commands still replied, every bulk
 * session ended in a herd-wide "session timed out" transmission, and the
 * reply to a log-stream request was queued behind the stream it started.
 *
 * kernelV2 makes the session explicit and exclusive:
 *
 *   IDLE --(CONNECT inside a connect window, or any UART command)--> ACTIVE
 *   ACTIVE --("x", 5 min without a command, sleep/radio-test command)--> IDLE
 *
 * Connect windows are opened by DeviceDiscovery (see FrKernel.h): 2 min after
 * the shake sequence, and the first 5 s of every discovery wake. While ACTIVE
 * the kernel owns the device - DeviceDiscovery cancels and skips everything
 * else, and MeshNetwork drops all non-kernel, non-OTA traffic - and the AppTask
 * is notified (DEVICE_DISCOVERY_NOTIFY_KERNEL) on every transition, so it
 * reacts at once instead of on a 500 ms poll.
 *
 * Reply rule: a tag replies to a frame if and only if the frame carried its
 * own id. Bulk frames (FRK_ID_BULK) are executed silently and their result is
 * DBG_LOGged locally. One function, FRKERNEL_vReply, applies that rule, so no
 * command handler can bypass it the way v1's "flash recover" did.
 *
 * Transports
 * ----------
 *   UART  UART2_vNotifyOnRX() (overrides the weak HAL callback) -> thread flag
 *         -> drain ring -> accumulate a line -> parse on CR/LF. The line is a
 *         command letter and an optional argument, exactly what the console
 *         would send. Replies go straight back out the UART.
 *   LoRa  MESHNETWORK_vOnFrKernelPacket() (overrides the weak MeshNetwork
 *         callback, parser task) filters the frame - version, direction,
 *         address - and only then queues it, so other tags' replies and log
 *         streams never occupy the 4-slot queue. Replies are REPLY frames.
 * Each command carries a FrkCtx_t saying which transport it came in on and
 * whether it may be answered, so the answer goes back the same way.
 */

#include "FrKernel.h"
#include "FrKernel_Config.h"
#include "FrKernel_Protocol.h"

#include "build_config.h"

/* The console build has its own implementation of the task and the callbacks
 * (FrKernelConsole.c). This file only supplies the tag-side session queries
 * there, as constant "no session" answers, because DeviceDiscovery and
 * MeshNetwork are still compiled into that image and link against them. */
#if defined(FRKERNEL_INTERFACE_LORA_CONSOLE)

void FRKERNEL_vOpenWindow(FrKernelWindow_e eWindow, uint32_t u32Ms) { (void)eWindow; (void)u32Ms; }
void FRKERNEL_vCloseWindow(void) { }
bool FRKERNEL_bWindowOpen(void)  { return false; }
bool FRKERNEL_bIsActive(void)    { return false; }
bool FRKERNEL_bHoldsDevice(void) { return false; }
void FRKERNEL_vEndSession(const char *pcReason) { (void)pcReason; }

#else /* tag: FRKERNEL_INTERFACE_UART and/or FRKERNEL_INTERFACE_LORA */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Battery.h"
#include "SolarPower.h"
#include "MeshNetwork.h"
#include "LoraRadio.h"
#include "DeviceDiscovery.h"

#include "version_config.h"
#include "DbgLog.h"
#include "Log.h"
#include "SelfTest.h"
#include "RadioTestMode.h"     /* R&D radio link/range test                 */
#include "stm32wlxx_hal.h"     /* TAMP->BKP2R, HAL_GetUIDw*                  */
#include "Fota.h"
#include "Flash.h"             /* flash health query / on-demand recovery   */
#include "Flash_SelfTest.h"    /* flash self-test + stress harness          */
#include "Identity.h"          /* serial number: 'n', 'N', status line      */
#include "SelfTest.h"          /* production test: 'u'                      */

#ifdef FRKERNEL_INTERFACE_UART
#  include "Debug.h"
#endif

/* -------------------------------------------------------------------------- */

#define FRKERNEL_FLAG_UART  0x01U
#define FRKERNEL_FLAG_LORA  0x02U

typedef enum {
    FRKERNEL_XPORT_UART,
    FRKERNEL_XPORT_LORA,
} FrKernelXport_e;

/* Where a command came from and whether it may be answered. */
typedef struct {
    FrKernelXport_e eXport;
    bool            bReply;   /* false for a bulk-addressed frame */
    uint8_t         u8Seq;    /* echoed in a LoRa reply           */
} FrkCtx_t;

static osThreadId_t      s_taskHandle      = NULL;

/* Session. Written by the FrKernel task (open/close) and read from every
 * task, so each is a single aligned word; FRKERNEL_vEndSession from another
 * task only ever requests, via s_bEndRequested, and the FrKernel task does
 * the actual teardown. */
static volatile bool     s_bActive         = false;
static volatile bool     s_bSingle         = false;   /* a CONNECT named us  */
static volatile uint8_t  s_eSessionXport   = FRKERNEL_XPORT_LORA;
static volatile uint32_t s_u32LastActivity = 0U;
static volatile bool     s_bEndRequested   = false;
static const char *volatile s_pcEndReason  = NULL;

/* Connect window. */
static volatile uint8_t  s_eWindow         = FRKERNEL_WINDOW_NONE;
static volatile uint32_t s_u32WindowEnd    = 0U;

#ifdef FRKERNEL_INTERFACE_LORA
/* Duplicate suppression: the console resends an unanswered command with the
 * same seq, so the same seq twice in a row is a retry, not a new command. */
static uint8_t           s_u8LastSeq       = 0U;
static bool              s_bSeqValid       = false;

typedef struct {
    uint8_t data[FRKERNEL_RX_ITEM_LEN];
    uint8_t len;
} FrKernelPkt_t;

static osMessageQueueId_t s_rxQueue        = NULL;
static volatile uint16_t  s_u16StreamFrames = 0U;
static volatile uint16_t  s_u16RxDrops      = 0U;   /* queue full */
#endif

#ifdef FRKERNEL_INTERFACE_UART
static char    s_lineBuf[FRKERNEL_LINE_BUF_LEN];
static uint8_t s_lineIdx = 0U;
#endif

static void FRKERNEL_vTask(void *arg);
static void FRKERNEL_vCloseSession(const char *pcReason, bool bNotify);

/* -------------------------------------------------------------------------- */
/* Session / window queries                                                   */
/* -------------------------------------------------------------------------- */

bool FRKERNEL_bIsActive(void)
{
    return s_bActive;
}

bool FRKERNEL_bWindowOpen(void)
{
    if (s_eWindow == FRKERNEL_WINDOW_NONE)
        return false;
    return (int32_t)(s_u32WindowEnd - osKernelGetTickCount()) > 0;
}

bool FRKERNEL_bHoldsDevice(void)
{
    return s_bActive ||
           ((s_eWindow == FRKERNEL_WINDOW_SHAKE) && FRKERNEL_bWindowOpen());
}

void FRKERNEL_vOpenWindow(FrKernelWindow_e eWindow, uint32_t u32Ms)
{
    if (s_bActive || (eWindow == FRKERNEL_WINDOW_NONE))
        return;

    uint32_t u32End = osKernelGetTickCount() + u32Ms;

    taskENTER_CRITICAL();
    bool bOpen = FRKERNEL_bWindowOpen();
    if (!bOpen || ((int32_t)(u32End - s_u32WindowEnd) > 0))
        s_u32WindowEnd = u32End;
    if (!bOpen || (eWindow > (FrKernelWindow_e)s_eWindow))
        s_eWindow = (uint8_t)eWindow;
    taskEXIT_CRITICAL();
}

void FRKERNEL_vCloseWindow(void)
{
    s_eWindow = FRKERNEL_WINDOW_NONE;
}

void FRKERNEL_vEndSession(const char *pcReason)
{
    if (!s_bActive)
        return;
    s_pcEndReason   = pcReason;
    s_bEndRequested = true;
    if (s_taskHandle != NULL)
        osThreadFlagsSet(s_taskHandle, FRKERNEL_FLAG_UART | FRKERNEL_FLAG_LORA);
}

/* -------------------------------------------------------------------------- */
/* Init                                                                       */
/* -------------------------------------------------------------------------- */

void FRKERNEL_vInit(void)
{
#ifdef FRKERNEL_INTERFACE_LORA
    s_rxQueue = osMessageQueueNew(FRKERNEL_LORA_QUEUE_LEN, sizeof(FrKernelPkt_t), NULL);
    configASSERT(s_rxQueue != NULL);
#endif

    static const osThreadAttr_t attr = {
        .name       = "FrKernel",
        /* CMSIS-RTOS v2 stack_size is BYTES; configMINIMAL_STACK_SIZE is a
         * WORD count, so every other task in this codebase multiplies by
         * sizeof(StackType_t). The v1 task once didn't, and ran on a quarter
         * of the intended ~3 KB until the production-sleep call chain
         * overran it. */
        .stack_size = configMINIMAL_STACK_SIZE * 6U * sizeof(StackType_t),
        .priority   = osPriorityLow,
    };
    s_taskHandle = osThreadNew(FRKERNEL_vTask, NULL, &attr);
    configASSERT(s_taskHandle != NULL);
}

/* -------------------------------------------------------------------------- */
/* Output                                                                     */
/* -------------------------------------------------------------------------- */

#ifdef FRKERNEL_INTERFACE_LORA
/* Build one v2 frame from this tag. */
static void FRKERNEL_vBuildFrame(LoraRadio_Packet_t *pPkt, FrkKind_e eKind,
                                 uint8_t u8Seq, const uint8_t *pu8Data,
                                 uint16_t u16Len)
{
    memset(pPkt, 0, sizeof(*pPkt));
    pPkt->buffer[0] = (uint8_t)MeshPktType_FrKernel;
    uint8_t *pF = &pPkt->buffer[1];
    pF[FRK_OFS_VERSION] = FRK_PROTO_VERSION;
    pF[FRK_OFS_KIND]    = (uint8_t)eKind;
    FRK_vPutU32(&pF[FRK_OFS_ID], LORARADIO_u32GetUniqueId());
    pF[FRK_OFS_SEQ]     = u8Seq;
    if (u16Len > FRK_MAX_PAYLOAD)
        u16Len = FRK_MAX_PAYLOAD;
    if (u16Len > 0U)
        memcpy(&pF[FRK_OFS_PAYLOAD], pu8Data, u16Len);
    pPkt->length = (uint8_t)(1U + FRK_HDR_LEN + u16Len);
}

/* Send text as REPLY frames, split across as many frames as it needs - v1
 * clamped every reply to one 253 B packet, which cut its own help text to the
 * first four of ~25 commands. Blocking enqueue with a bound, so a reply is not
 * dropped just because a stream frame is momentarily occupying the queue.
 * Never transmits during a FOTA session: MeshNetwork drops kernel traffic
 * then anyway, and a TX in the middle of a chunk stream is exactly the
 * radio-near-flash disturbance the OTA code is built to avoid. */
static void FRKERNEL_vLoraReply(uint8_t u8Seq, const char *pcMsg)
{
    if (FOTA_bSessionActive())
    {
        DBG_LOG("FrKernel: reply suppressed during FOTA: %s", pcMsg);
        return;
    }

    uint16_t u16Len = (uint16_t)strlen(pcMsg);
    do
    {
        uint16_t n = (u16Len > FRK_MAX_PAYLOAD) ? FRK_MAX_PAYLOAD : u16Len;
        LoraRadio_Packet_t tPkt;
        FRKERNEL_vBuildFrame(&tPkt, FRK_KIND_REPLY, u8Seq, (const uint8_t *)pcMsg, n);
        if (!LORARADIO_bTxPacketWait(&tPkt, 2000U))
        {
            DBG_LOG("FrKernel: reply dropped, TX queue busy\r\n");
            return;
        }
        pcMsg  += n;
        u16Len  = (uint16_t)(u16Len - n);
    } while (u16Len > 0U);
}

/* Wait (bounded) until the radio has nothing of ours left to send, so a final
 * reply actually leaves before the device is handed back for deep sleep. */
static void FRKERNEL_vDrainTx(void)
{
    uint32_t u32Start = osKernelGetTickCount();
    while (!LORARADIO_bTxQuiet(0U) &&
           ((osKernelGetTickCount() - u32Start) < FRKERNEL_TX_DRAIN_MAX_MS))
        osDelay(20);
}
#endif /* FRKERNEL_INTERFACE_LORA */

#ifdef FRKERNEL_INTERFACE_UART
/* UART output goes through DbgLog's plain path, so the DbgLog consumer stays
 * the only writer of the debug UART. v1 wrote it directly from this task,
 * racing the consumer on HAL_UART_vTxPutByte's unlocked ring head. Messages
 * are capped at 255 B by DbgLog; every reply and help line is shorter. */
static void FRKERNEL_vUartPutLine(const char *pcLine)
{
    _DBG_("%s", pcLine);
}
#endif

/* The one reply path. bReply false (bulk) never transmits. */
static void FRKERNEL_vReply(const FrkCtx_t *pCtx, const char *pcMsg)
{
    if (!pCtx->bReply)
    {
        size_t len = strlen(pcMsg);
        while ((len > 0U) && ((pcMsg[len - 1U] == '\r') || (pcMsg[len - 1U] == '\n')))
            len--;
        DBG_LOG("FrKernel bulk: %.*s\r\n", (int)len, pcMsg);
        return;
    }

#ifdef FRKERNEL_INTERFACE_UART
    if (pCtx->eXport == FRKERNEL_XPORT_UART)
    {
        FRKERNEL_vUartPutLine(pcMsg);
        return;
    }
#endif
#ifdef FRKERNEL_INTERFACE_LORA
    if (pCtx->eXport == FRKERNEL_XPORT_LORA)
        FRKERNEL_vLoraReply(pCtx->u8Seq, pcMsg);
#endif
}

/* -------------------------------------------------------------------------- */
/* Session open / close                                                       */
/* -------------------------------------------------------------------------- */

static void FRKERNEL_vNotifyApp(bool bOpened)
{
    DEVICE_DISCOVERY_vOnKernelSession(bOpened);
}

static void FRKERNEL_vOpenSession(FrKernelXport_e eXport, bool bSingle)
{
    s_eWindow         = FRKERNEL_WINDOW_NONE;
    s_eSessionXport   = (uint8_t)eXport;
    s_bSingle         = bSingle;
#ifdef FRKERNEL_INTERFACE_LORA
    s_bSeqValid       = false;
#endif
    s_bEndRequested   = false;
    s_u32LastActivity = osKernelGetTickCount();
    s_bActive         = true;

    DBG_LOG("FrKernel: session OPEN (%s, %s)\r\n",
            (eXport == FRKERNEL_XPORT_UART) ? "uart" : "lora",
            bSingle ? "single" : "bulk");
    FRKERNEL_vNotifyApp(true);
}

/* Tear down a session. bNotify: tell a single LoRa console why (bulk never
 * hears anything; the UART operator gets the line anyway). Everything the
 * session may have left running is stopped here, in one place, for every way
 * a session can end. */
static void FRKERNEL_vCloseSession(const char *pcReason, bool bNotify)
{
    if (!s_bActive)
        return;

    DBGLOG_vCancelDump();

    /* An explicit "w" arm belongs to this session. v1 left it set after
     * release/timeout, so a later unrelated session (a log download) could
     * start an OTA receive on a stray OtaPrep - the exact case the hold loop
     * comment said must not happen. */
    if (FOTA_bAcceptanceArmedViaKernel())
        FOTA_vDisarmAcceptance();

    char acMsg[64];
    (void)snprintf(acMsg, sizeof(acMsg), "kernel closed: %s\r\n", pcReason);

#ifdef FRKERNEL_INTERFACE_UART
    if (s_eSessionXport == FRKERNEL_XPORT_UART)
        FRKERNEL_vUartPutLine(acMsg);
#endif
#ifdef FRKERNEL_INTERFACE_LORA
    if ((s_eSessionXport == FRKERNEL_XPORT_LORA) && bNotify && s_bSingle &&
        !FOTA_bSessionActive())
    {
        LoraRadio_Packet_t tPkt;
        FRKERNEL_vBuildFrame(&tPkt, FRK_KIND_BYE, 0U, (const uint8_t *)acMsg,
                             (uint16_t)strlen(acMsg));
        (void)LORARADIO_bTxPacketWait(&tPkt, 2000U);
    }
    FRKERNEL_vDrainTx();
#endif

    s_bActive = false;
    s_bSingle = false;
    DBG_LOG("FrKernel: session CLOSED (%s)\r\n", pcReason);
    FRKERNEL_vNotifyApp(false);
}

/* -------------------------------------------------------------------------- */
/* Log streaming                                                              */
/* -------------------------------------------------------------------------- */

#ifdef FRKERNEL_INTERFACE_LORA
/* DbgLog's dump sink for a LoRa stream. Runs on the DbgLog consumer task.
 * Each chunk becomes STREAM frames with a running seq, so the console can
 * count lost frames instead of showing a silently holed log. Returns false -
 * stopping the stream - when the session has ended or the radio cannot take a
 * frame within FRKERNEL_STREAM_TX_WAIT_MS. */
static bool FRKERNEL_bLoraStreamSink(const uint8_t *pu8Data, uint16_t u16Len)
{
    while (u16Len > 0U)
    {
        if (!s_bActive)
            return false;

        uint16_t n = (u16Len > FRK_MAX_PAYLOAD) ? FRK_MAX_PAYLOAD : u16Len;
        LoraRadio_Packet_t tPkt;
        FRKERNEL_vBuildFrame(&tPkt, FRK_KIND_STREAM,
                             (uint8_t)s_u16StreamFrames, pu8Data, n);
        if (!LORARADIO_bTxPacketWait(&tPkt, FRKERNEL_STREAM_TX_WAIT_MS))
            return false;
        s_u16StreamFrames++;

        /* Keep the next flash read off the PA - see
         * FRKERNEL_STREAM_READ_SETTLE_MS. Bounded by the same wait. */
        uint32_t u32Start = osKernelGetTickCount();
        while (!LORARADIO_bTxQuiet(FRKERNEL_STREAM_READ_SETTLE_MS))
        {
            if ((osKernelGetTickCount() - u32Start) >= FRKERNEL_STREAM_TX_WAIT_MS)
                return false;
            osDelay(10);
        }

        /* An outbound stream is activity the operator is waiting on. */
        s_u32LastActivity = osKernelGetTickCount();

        pu8Data += n;
        u16Len   = (uint16_t)(u16Len - n);
    }
    return true;
}

/* End-of-stream report, on the DbgLog consumer task. seq carries the frame
 * count (mod 256) so the console can check it against what it received. */
static void FRKERNEL_vLoraStreamDone(const LogStreamResult_t *pRes)
{
    char acMsg[128];
    int  n = snprintf(acMsg, sizeof(acMsg),
                      "stream %s: %lu of %lu bytes, %u frames, %u read fail(s)\r\n",
                      pRes->bAborted ? "ABORTED" : "complete",
                      (unsigned long)pRes->u32Bytes, (unsigned long)pRes->u32Total,
                      (unsigned)s_u16StreamFrames, (unsigned)pRes->u16ReadFails);
    DBG_LOG("FrKernel: %s", acMsg);

    if (!s_bActive || FOTA_bSessionActive() || (n <= 0))
        return;

    LoraRadio_Packet_t tPkt;
    FRKERNEL_vBuildFrame(&tPkt, FRK_KIND_STREAM_END, (uint8_t)s_u16StreamFrames,
                         (const uint8_t *)acMsg, (uint16_t)n);
    (void)LORARADIO_bTxPacketWait(&tPkt, FRKERNEL_STREAM_TX_WAIT_MS);
}
#endif

static void FRKERNEL_vStartStream(const FrkCtx_t *pCtx, uint8_t u8Percent, char *resp, size_t szResp)
{
    if (DBGLOG_bDumpBusy())
    {
        FRKERNEL_vReply(pCtx, "stream already running - 's' stops it\r\n");
        return;
    }

    uint32_t u32Used  = LOG_u32GetUsedBytes();
    uint32_t u32Bytes = (uint32_t)(((uint64_t)u32Used * u8Percent) / 100U);

    /* The acknowledgement goes out BEFORE the stream is requested. v1 asked
     * for the dump first; the consumer (osPriorityAboveNormal) preempted this
     * task at once and filled the radio queue, so the ack was almost always
     * dropped, and when it was not it landed inside the log body. */
    (void)snprintf(resp, szResp, "streaming latest %u%% of log: ~%lu of %lu bytes\r\n",
                   (unsigned)u8Percent, (unsigned long)u32Bytes, (unsigned long)u32Used);
    FRKERNEL_vReply(pCtx, resp);

#ifdef FRKERNEL_INTERFACE_LORA
    if (pCtx->eXport == FRKERNEL_XPORT_LORA)
    {
        s_u16StreamFrames = 0U;
        if (!DBGLOG_bRequestDumpVia(FRKERNEL_bLoraStreamSink, u8Percent,
                                    FRKERNEL_vLoraStreamDone))
            FRKERNEL_vReply(pCtx, "stream could not start (busy)\r\n");
        return;
    }
#endif
    if (!DBGLOG_bRequestDump(u8Percent))
        FRKERNEL_vReply(pCtx, "stream could not start (busy)\r\n");
}

/* -------------------------------------------------------------------------- */
/* Flash stress test (bench diagnostics, build-gated)                         */
/* -------------------------------------------------------------------------- */

#ifdef ENABLE_FLASH_STRESS_TEST
/* ---- Stress-test disturbance callbacks ------------------------------------
 *
 * These exist so the thing being blamed for the post-TX read corruption is
 * INJECTED rather than assumed. The stress loop itself names neither the radio
 * nor the power-down path; it just calls whichever of these the operator asked
 * for, which is what makes the three runs comparable:
 *
 *   z <N>        quiet baseline. Anything that shows up here is a property of
 *                the bus at 12 MHz, not of the radio.
 *   z tx <N>     a real PA pulse between every read.
 *   z park <N>   a deep-power-down park (B9h) and wake (ABh) between every
 *                read, with no radio activity at all.
 *
 * The second and third are the discriminating pair. The existing field
 * evidence, gathered on frtag2 (V2) hardware (see OTA_LORA_CHUNK_GAP_MS in
 * Fota_Config.h), is consistent with either a TX-coupled disturbance or an
 * unhonoured flash recovery time, because in production the two always happen
 * close together - a DBG_LOG in a radio gap parks the chip, and the next read
 * wakes it. Separating them is the point. "TX-coupled" deliberately names no
 * path: the PA-current rail-sag explanation was never tested and TX was not
 * shown to sag the rail, and noise on the very long MISO/MOSI runs over a
 * non-continuous ground plane is the likelier one. A "tx" run that corrupts
 * reads while "park" stays clean still says the radio is involved, not how.
 *
 * Single-connection only (FRK_CMDF_SINGLE): v1 accepted it under bulk, where
 * "tx" keyed the PA of every tag in earshot thousands of times at once.
 * -------------------------------------------------------------------------- */

/* Key the PA and wait until it has actually fired, not merely been queued.
 * LORARADIO_bTxPacket only enqueues; the radio task's carrier sense can defer
 * the real transmission by up to LORA_TX_CARRIER_WAIT_MS. LORARADIO_bTxQuiet(0)
 * going true means nothing is queued, nothing is in flight, and the pulse is
 * behind us. The frame is a v2 REPLY carrying "S", which a console prints as
 * a stray line and every tag ignores. */
static void FRKERNEL_vStressTxCallback(void)
{
#ifdef FRKERNEL_INTERFACE_LORA
    LoraRadio_Packet_t tPkt;
    FRKERNEL_vBuildFrame(&tPkt, FRK_KIND_REPLY, 0U, (const uint8_t *)"S", 1U);
    if (!LORARADIO_bTxPacketWait(&tPkt, 1000U))
        return;

    uint32_t u32Start = osKernelGetTickCount();
    while (!LORARADIO_bTxQuiet(0U) &&
           ((osKernelGetTickCount() - u32Start) < 2000U))
        osDelay(5);
#endif
}

/* Park the flash and let the next read wake it, with the radio untouched.
 * FLASH_vDeepPowerDown is a no-op while an OTA session holds the inhibit, so
 * this measures nothing during a firmware transfer - which is correct, and
 * worth knowing if a run reports suspiciously clean numbers. */
static void FRKERNEL_vStressParkCallback(void)
{
    FLASH_vDeepPowerDown();
}

static void FRKERNEL_vRunStress(const FrkCtx_t *pCtx, char *resp, size_t szResp,
                                const char *pcArg)
{
    void      (*pfnCb)(void) = NULL;
    const char *pcLabel      = "quiet";

    if (strncmp(pcArg, "tx", 2) == 0)
    {
        pfnCb   = FRKERNEL_vStressTxCallback;
        pcLabel = "tx";
        pcArg  += 2;
    }
    else if (strncmp(pcArg, "park", 4) == 0)
    {
        pfnCb   = FRKERNEL_vStressParkCallback;
        pcLabel = "park";
        pcArg  += 4;
    }

    uint32_t u32Cycles = (uint32_t)strtoul(pcArg, NULL, 10);
    if (u32Cycles == 0UL)     u32Cycles = 100UL;
    if (u32Cycles > 10000UL)  u32Cycles = 10000UL;   /* ~10 s at 1 ms/cycle */

    Flash_StressStats_t tStats;
    if (!FLASH_bStressTest(u32Cycles, pfnCb, &tStats))
    {
        FRKERNEL_vReply(pCtx, "flash stress: reference page not programmed - "
                              "run 'p' (flash self-test) first\r\n");
        return;
    }

    /* One line, every classification. A bare mismatch count would not
     * distinguish the hypotheses this run exists to separate.
     *
     * READ drvRetry FIRST when ENABLE_FLASH_READ_CRC is on. The driver's own
     * read-verify retries corruption away before this loop ever sees it, so
     * "bad" is the RESIDUAL - what survived three attempts - and drvRetry is
     * the sensitive measure of how often the bus lied at all. A run with
     * bad=0 and drvRetry=40 is not a clean run; it is forty disturbances that
     * were caught. With the option off, "bad" carries the whole signal. */
    (void)snprintf(resp, szResp,
                   "flash stress %s: %lu cycles, %lu bad (FF=%lu 00=%lu shift=%lu "
                   "noise=%lu) readFail=%lu first=%lu drvRetry=%u drvFail=%u\r\n",
                   pcLabel,
                   (unsigned long)tStats.u32Cycles,
                   (unsigned long)tStats.u32Mismatch,
                   (unsigned long)tStats.u32AllFF,
                   (unsigned long)tStats.u32All00,
                   (unsigned long)tStats.u32Shifted,
                   (unsigned long)tStats.u32Noise,
                   (unsigned long)tStats.u32ReadFail,
                   (unsigned long)tStats.u32FirstBadCycle,
                   (unsigned)tStats.u16DriverRetries,
                   (unsigned)tStats.u16DriverFails);
    FRKERNEL_vReply(pCtx, resp);
}
#endif /* ENABLE_FLASH_STRESS_TEST */

/* -------------------------------------------------------------------------- */
/* Commands                                                                   */
/* -------------------------------------------------------------------------- */

static const char *FRKERNEL_pcProdState(void)
{
    switch (DEVICE_DISCOVERY_eGetProductionState())
    {
        case PRODUCTION_SLEEP:  return "sleep";
        case PRODUCTION_ACTIVE: return "active";
        default:                return "ready";
    }
}

/* One line that answers most first questions about a tag. Also the reply to a
 * single CONNECT, so the operator learns what they connected to without
 * spending a command on it. The 96-bit UID is included so that what the radio
 * id was derived from (see LORARADIO_u32GetUniqueId) is on record, and so two
 * tags answering one 16-bit id show up as two UIDs. */
static void FRKERNEL_vFormatStatus(char *resp, size_t szResp)
{
    (void)snprintf(resp, szResp,
                   "%08" PRIX32 " sn=%08lX %s fw=v%u.%u.%u bl=v%lu bat=%umV sol=%umV "
                   "state=%s int=%umin log=%u%% uid=%08lX%08lX%08lX\r\n",
                   LORARADIO_u32GetUniqueId(),
                   (unsigned long)IDENTITY_u32GetSerial(),
                   (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY) ? "PRIMARY" : "secondary",
                   (unsigned)VERSION_SW_MAJOR, (unsigned)VERSION_SW_MINOR,
                   (unsigned)VERSION_SW_PATCH,
                   (unsigned long)TAMP->BKP2R,
                   (unsigned)BAT_u16GetVoltage(), (unsigned)SOLAR_u16GetVSolarMV(),
                   FRKERNEL_pcProdState(),
                   (unsigned)MESHNETWORK_u8GetWakeupInterval(),
                   (unsigned)LOG_u8GetUsedPercent(),
                   (unsigned long)HAL_GetUIDw2(), (unsigned long)HAL_GetUIDw1(),
                   (unsigned long)HAL_GetUIDw0());
}

/* SelfTestReply_f adapter: the production test streams several lines, each
 * going back on the transport the command came in on. */
static void FRKERNEL_vSelfTestReply(void *pvCtx, const char *pcLine)
{
    FRKERNEL_vReply((const FrkCtx_t *)pvCtx, pcLine);
}

/* A command that hands the device to something else ends the session after
 * its reply has gone out. */
static void FRKERNEL_vTerminal(const char *pcReason)
{
    /* bNotify: the console gets a BYE, so it drops back to idle instead of
     * retrying commands into a tag that has just gone to sleep. */
    FRKERNEL_vCloseSession(pcReason, true);
}

static void FRKERNEL_vDispatch(const FrkCtx_t *pCtx, char cLetter, const char *pcArg)
{
    char resp[FRKERNEL_RESP_BUF_LEN];
    const FrkCmd_t *pCmd = FRK_ptFindCmd(cLetter);

    if (pCmd == NULL)
    {
        (void)snprintf(resp, sizeof(resp), "unknown command '%c' - '?' lists them\r\n",
                       (cLetter >= ' ' && cLetter <= '~') ? cLetter : '.');
        FRKERNEL_vReply(pCtx, resp);
        return;
    }

    /* The console refuses these under bulk already; this is the tag's own
     * guard, so a bulk frame from anything else still cannot reach them. */
    if (!pCtx->bReply && (pCmd->u8Flags & FRK_CMDF_SINGLE))
    {
        DBG_LOG("FrKernel: '%c' is single-connection only - ignored under bulk\r\n", cLetter);
        return;
    }

    switch (cLetter)
    {
    case '?':
#ifdef FRKERNEL_INTERFACE_UART
        if (pCtx->eXport == FRKERNEL_XPORT_UART)
        {
            FRK_vPrintHelp(FRKERNEL_vUartPutLine, false);
            break;
        }
#endif
        /* Over LoRa the console prints the table itself; sending ~1.5 KB of
         * help over the air on every '?' would be pure waste. */
        FRKERNEL_vReply(pCtx, "help is printed by the console - type ? there\r\n");
        break;

    case 'i':
        FRKERNEL_vFormatStatus(resp, sizeof(resp));
        FRKERNEL_vReply(pCtx, resp);
        break;

    case 'n':
    {
        Identity_Status_t tId;
        IDENTITY_vGetStatus(&tId);
        if (tId.eSource == IDENTITY_SRC_NONE)
            (void)snprintf(resp, sizeof(resp), "serial: none  id=%08lX  otp free=%u\r\n",
                           (unsigned long)LORARADIO_u32GetUniqueId(), (unsigned)tId.u8OtpFree);
        else
            (void)snprintf(resp, sizeof(resp),
                           "serial: %08lX  id=%08lX  src=%s  otp slot=%u free=%u  mirror=%s%s\r\n",
                           (unsigned long)tId.u32Serial,
                           (unsigned long)LORARADIO_u32GetUniqueId(),
                           IDENTITY_pcSourceText(tId.eSource),
                           (unsigned)tId.u8OtpSlot, (unsigned)tId.u8OtpFree,
                           tId.bMirrorOk ? "ok" : "BAD",
                           tId.bRadioIdMatch ? "" : "  RADIO-ID-MISMATCH");
        FRKERNEL_vReply(pCtx, resp);
        break;
    }

    case 'u':
    {
        /* u [hold s] - production test. Blocks this task for the checks and
         * the hold; the session stays open throughout, so nothing else can
         * start on the device meanwhile. */
        char    *pcEnd;
        uint32_t u32Hold = SELFTEST_PROD_HOLD_DEFAULT_S;
        if (*pcArg != '\0')
        {
            u32Hold = (uint32_t)strtoul(pcArg, &pcEnd, 10);
            if ((pcEnd == pcArg) || (*pcEnd != '\0'))
            {
                FRKERNEL_vReply(pCtx, "usage: u [hold seconds, 0..120]\r\n");
                break;
            }
        }
        (void)SELFTEST_bRunProductionTest(u32Hold, FRKERNEL_vSelfTestReply, (void *)pCtx);
        break;
    }

    case 'N':
    {
        /* N <serial, 8 hex digits> <radio id, hex> [fix]
         *
         * The serial must be exactly 8 hex digits (format in
         * Identity_Config.h). Exactly 8, so a decimal number typed by habit -
         * the old format was decimal - is refused rather than read as hex.
         *
         * The radio id is required, not optional, and must be the full 8-digit
         * one: it is what stops a production station from serialising the
         * wrong unit when more than one can hear it. The reply echoes all of it
         * so the station can log exactly what this unit accepted. */
        char       *pcEnd;
        const char *pc        = pcArg;
        uint32_t    u32Serial = (uint32_t)strtoul(pc, &pcEnd, 16);
        bool        bParsed   = ((pcEnd - pc) == 8) && (*pcEnd == ' ');

        pc = pcEnd;
        while (*pc == ' ') pc++;
        uint32_t u32Id = (uint32_t)strtoul(pc, &pcEnd, 16);
        bParsed = bParsed && (pcEnd != pc);

        pc = pcEnd;
        while (*pc == ' ') pc++;
        bool bFix = false;
        if (strcmp(pc, "fix") == 0) bFix = true;
        else if (*pc != '\0')       bParsed = false;

        if (!bParsed)
        {
            FRKERNEL_vReply(pCtx, "usage: N <serial, 8 hex> <radio id> [fix]  e.g. N 31000001 0A3C1A2B\r\n");
            break;
        }

        Identity_SetResult_e eRes = IDENTITY_eSetSerial(u32Serial, u32Id, bFix);
        Identity_Status_t tId;
        IDENTITY_vGetStatus(&tId);
        (void)snprintf(resp, sizeof(resp), "N %08lX %08lX: %s (serial now %08lX, otp free=%u)\r\n",
                       (unsigned long)u32Serial, (unsigned long)u32Id,
                       IDENTITY_pcSetResultText(eRes),
                       (unsigned long)tId.u32Serial, (unsigned)tId.u8OtpFree);
        FRKERNEL_vReply(pCtx, resp);
        break;
    }

    case 'e':
        /* GPS and ACC answer from the memoized boot-time run; flash is live
         * from the driver gate (it can close and reopen after boot), plus the
         * boot reason code - "FAIL" alone sends an operator to guess,
         * "FAIL:wrong-part" sends them to the BOM. */
        (void)snprintf(resp, sizeof(resp), "selftest: gps=%s acc=%s flash=%s (boot: %s)\r\n",
                       SELFTEST_bGpsApplicable()   ? (SELFTEST_bGpsOk()   ? "OK" : "FAIL") : "n/a",
                       SELFTEST_bAccApplicable()   ? (SELFTEST_bAccOk()   ? "OK" : "FAIL") : "n/a",
                       SELFTEST_bFlashApplicable() ? (SELFTEST_bFlashOk() ? "OK" : "FAIL") : "n/a",
                       SELFTEST_pcFlashDetail());
        FRKERNEL_vReply(pCtx, resp);
        break;

    case 'f':
    {
        /* Full flash health over the air. A tag whose flash has gated off has
         * lost its log - the one place this would otherwise be recorded - and
         * cannot be reached with a UART in the field. */
        Flash_Health_t tHealth;
        FLASH_vGetHealth(&tHealth);
        (void)snprintf(resp, sizeof(resp),
                       "flash: %s id=%02X%02X%02X tries=%u absent=%u recov=%u "
                       "probeFail=%u wprot=%u unprotFail=%u eraseVfy=%u "
                       "crcRetry=%u crcFail=%u\r\n",
                       tHealth.bPresent ? "USABLE" : "GATED-OFF",
                       tHealth.au8LastId[0], tHealth.au8LastId[1], tHealth.au8LastId[2],
                       (unsigned)tHealth.u8LastAttempts,
                       (unsigned)tHealth.bEverAbsent,
                       (unsigned)tHealth.u16Recoveries,
                       (unsigned)tHealth.u16ProbeFailures,
                       (unsigned)tHealth.bWriteProtected,
                       (unsigned)tHealth.bUnprotectFailed,
                       (unsigned)tHealth.u16EraseVerifyFails,
                       (unsigned)tHealth.u16ReadCrcRetries,
                       (unsigned)tHealth.u16ReadCrcFails);
        FRKERNEL_vReply(pCtx, resp);
        break;
    }

    case 'k':
    {
        /* Reading clears, so two reads bracket whatever happened in between -
         * which is how to attribute corruption to an OTA session, a log stream
         * or a TX burst rather than to "sometime since boot". */
        uint16_t u16Retries = 0U;
        uint16_t u16Fails   = 0U;
        (void)FLASH_u32GetAndClearReadCrcStats(&u16Retries, &u16Fails);
        (void)snprintf(resp, sizeof(resp), "flash crc: retries=%u fails=%u (cleared)\r\n",
                       (unsigned)u16Retries, (unsigned)u16Fails);
        FRKERNEL_vReply(pCtx, resp);
        break;
    }

    case 'a': FRKERNEL_vStartStream(pCtx, FRK_STREAM_ALL_PCT,     resp, sizeof(resp)); break;
    case 'h': FRKERNEL_vStartStream(pCtx, FRK_STREAM_HALF_PCT,    resp, sizeof(resp)); break;
    case 'q': FRKERNEL_vStartStream(pCtx, FRK_STREAM_QUARTER_PCT, resp, sizeof(resp)); break;
    case 't': FRKERNEL_vStartStream(pCtx, FRK_STREAM_TENTH_PCT,   resp, sizeof(resp)); break;

    case 's':
        if (DBGLOG_bDumpBusy())
        {
            DBGLOG_vCancelDump();
            FRKERNEL_vReply(pCtx, "stream stop requested\r\n");
        }
        else
        {
            FRKERNEL_vReply(pCtx, "no stream running\r\n");
        }
        break;

    case 'r':
    {
        /* Force a probe + bring-up now, ignoring the re-probe cooldown. */
        uint8_t au8Id[3]   = {0};
        uint8_t u8Attempts = 0U;
        bool    bOk        = FLASH_bRecoverDevice(au8Id, &u8Attempts);
        (void)snprintf(resp, sizeof(resp),
                       "flash recover: %s id=%02X%02X%02X after %u attempt(s)\r\n",
                       bOk ? "USABLE" : "STILL GATED-OFF",
                       au8Id[0], au8Id[1], au8Id[2], (unsigned)u8Attempts);
        FRKERNEL_vReply(pCtx, resp);
        break;
    }

    case 'p':
    {
        /* Re-runs the boot self-test on demand, which also leaves the known
         * pattern at FLASH_SELFTEST_ADDR for the stress variants. Destructive
         * to the reserved sector only, and only where the build asked for the
         * destructive half (ENABLE_FLASH_SELFTEST_DESTRUCTIVE). */
        Flash_SelfTestResult_t eRes = FLASH_eSelfTest();
        (void)snprintf(resp, sizeof(resp), "flash test: %s\r\n", FLASH_pcSelfTestResult(eRes));
        FRKERNEL_vReply(pCtx, resp);
        break;
    }

#ifdef ENABLE_FLASH_STRESS_TEST
    case 'z':
        FRKERNEL_vRunStress(pCtx, resp, sizeof(resp), pcArg);
        break;
#endif

    case 'd':
    {
        if (*pcArg == '\0')
        {
            (void)snprintf(resp, sizeof(resp), "discovery interval: %u min\r\n",
                           (unsigned)MESHNETWORK_u8GetWakeupInterval());
            FRKERNEL_vReply(pCtx, resp);
            break;
        }

        /* Bulk-capable ("d 60" on a bulk connection): every connected tag
         * applies it. A single tag's value is overwritten by the primary's next
         * TimeSync, which broadcasts whatever interval the PRIMARY holds - for
         * a lasting change, set it on the primary (or the fr9's minWindow).
         *
         * Validity is a separate flag, not a sentinel enum value (review F25,
         * 2026-11-13): WAKEUP_INTERVAL_MAX_COUNT aliases WAKEUP_INTERVAL_240_MIN. */
        char    *pcEnd;
        uint32_t u32Mins = (uint32_t)strtoul(pcArg, &pcEnd, 10);
        WakeupInterval eNew = WAKEUP_INTERVAL_15_MIN;
        bool bValid = (pcEnd != pcArg) && (*pcEnd == '\0');
        if      (u32Mins == 15U)  eNew = WAKEUP_INTERVAL_15_MIN;
        else if (u32Mins == 30U)  eNew = WAKEUP_INTERVAL_30_MIN;
        else if (u32Mins == 60U)  eNew = WAKEUP_INTERVAL_60_MIN;
        else if (u32Mins == 120U) eNew = WAKEUP_INTERVAL_120_MIN;
        else if (u32Mins == 240U) eNew = WAKEUP_INTERVAL_240_MIN;
        else                      bValid = false;

        if (bValid)
        {
            MESHNETWORK_vSetWakeupInterval(eNew);
            (void)snprintf(resp, sizeof(resp), "discovery interval set to %u min\r\n",
                           (unsigned)u32Mins);
        }
        else
        {
            (void)snprintf(resp, sizeof(resp),
                           "invalid interval '%s' - allowed: 15,30,60,120,240\r\n", pcArg);
        }
        FRKERNEL_vReply(pCtx, resp);
        break;
    }

    case 'w':
        if (strcmp(pcArg, "off") == 0)
        {
            FOTA_vDisarmAcceptance();
            FRKERNEL_vReply(pCtx, "firmware acceptance disarmed\r\n");
        }
        else if (*pcArg != '\0')
        {
            FRKERNEL_vReply(pCtx, "usage: w  or  w off\r\n");
        }
        else if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_SECONDARY)
        {
            FRKERNEL_vReply(pCtx, "w: secondary only\r\n");
        }
        else
        {
            /* The session stays open; the AppTask's session loop runs the
             * receive when the primary's OtaPrep arrives. */
            FOTA_vArmAcceptanceKernel();
            FRKERNEL_vReply(pCtx, "firmware acceptance ARMED - awaiting OtaPrep\r\n");
        }
        break;

    case 'o':
        if (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY)
            FRKERNEL_vReply(pCtx, "o: primary only\r\n");
        else if (FOTA_bRequestDistribute())
            FRKERNEL_vReply(pCtx, "firmware distribute requested\r\n");
        else
            FRKERNEL_vReply(pCtx, "o: no valid image staged\r\n");
        break;

    case 'x':
        if (RADIOTESTMODE_bActive())
            RADIOTESTMODE_vExit("kernel exit");
        FRKERNEL_vReply(pCtx, "bye - resuming schedule\r\n");
        FRKERNEL_vTerminal("exit");
        break;

    case 'E':
        /* Routed through the DbgLog consumer so it can't race log writes. */
        DBGLOG_vRequestErase();
        FRKERNEL_vReply(pCtx, "ext-flash log erase requested\r\n");
        break;

    case 'W':
        /* Whole-chip erase: log AND the OTA scratchpad/metadata. Destroys a
         * staged-but-not-installed image - a deliberate full reset. */
        DBGLOG_vRequestEraseAll();
        FRKERNEL_vReply(pCtx, "full ext-flash erase requested (log + staged OTA image)\r\n");
        break;

    case 'P':
    case 'S':
    {
        const bool bProd = (cLetter == 'P');
        bool bOk = bProd ? DEVICE_DISCOVERY_bEnterProductionSleep()
                         : DEVICE_DISCOVERY_bEnterSolarSleep();
        if (!bOk)
        {
            /* v1 acked "ProductionSleep entered" on a primary that had just
             * refused it. */
            FRKERNEL_vReply(pCtx, bProd ? "P: secondary only\r\n" : "S: secondary only\r\n");
            break;
        }
        FRKERNEL_vReply(pCtx, bProd ? "production sleep - wakes on solar >= 3000 mV or shake\r\n"
                                    : "solar sleep - wakes on shake only\r\n");
        FRKERNEL_vTerminal(bProd ? "production sleep" : "solar sleep");
        break;
    }

    case 'R':
        /* Entered BEFORE the ack so the reply reflects what happened: the
         * entry flushes the radio TX queue, which would otherwise discard an
         * ack queued ahead of it. TX still works once the mode is up - it is
         * only the receive path a beaconing secondary closes.
         *
         * Single-connection only (FRK_CMDF_SINGLE): over bulk it would make
         * every secondary in earshot deaf to the radio until each was found
         * and shaken by hand. */
        if (RADIOTESTMODE_bEnter())
        {
            if (RADIOTESTMODE_bIsListener())
            {
                /* A listening primary stays reachable: the session stays open
                 * (the test counts as activity) so "Q" can end it. */
                FRKERNEL_vReply(pCtx, "radio test: listening - 'Q' ends it\r\n");
            }
            else
            {
                FRKERNEL_vReply(pCtx, "radio test: beaconing every 5 s - shake to end\r\n");
                FRKERNEL_vTerminal("radio test");
            }
        }
        else
        {
            /* The reason matters: this reply is very likely the ONLY thing a
             * remote operator ever sees of the failure. */
            (void)snprintf(resp, sizeof(resp), "radio test: could not enter (%s)\r\n",
                           RADIOTESTMODE_pcLastEnterError());
            FRKERNEL_vReply(pCtx, resp);
        }
        break;

    case 'Q':
        if (RADIOTESTMODE_bActive())
        {
            RADIOTESTMODE_vExit("Q");
            FRKERNEL_vReply(pCtx, "radio test stopped\r\n");
        }
        else
        {
            FRKERNEL_vReply(pCtx, "radio test not running\r\n");
        }
        break;

    default:
        /* In the table but not handled - a build that compiled the table
         * entry without its handler. */
        (void)snprintf(resp, sizeof(resp), "'%c' not available in this build\r\n", cLetter);
        FRKERNEL_vReply(pCtx, resp);
        break;
    }
}

/* Split "<letter>[ <arg>]" and dispatch. Leading/trailing blanks ignored. */
static void FRKERNEL_vRunLine(const FrkCtx_t *pCtx, char *pcLine)
{
    while (*pcLine == ' ')
        pcLine++;
    size_t len = strlen(pcLine);
    while ((len > 0U) && (pcLine[len - 1U] == ' '))
        pcLine[--len] = '\0';
    if (len == 0U)
        return;

    char cLetter = pcLine[0];
    const char *pcArg = "";
    if (len > 1U)
    {
        if (pcLine[1] != ' ')
        {
            char resp[48];
            (void)snprintf(resp, sizeof(resp), "commands are one letter - '?' lists them\r\n");
            FRKERNEL_vReply(pCtx, resp);
            return;
        }
        pcArg = &pcLine[2];
        while (*pcArg == ' ')
            pcArg++;
    }
    FRKERNEL_vDispatch(pCtx, cLetter, pcArg);
}

/* -------------------------------------------------------------------------- */
/* LoRa frame handling                                                        */
/* -------------------------------------------------------------------------- */

#ifdef FRKERNEL_INTERFACE_LORA
/* Status line with a prefix, as a reply. Its own (non-inlined) frame so the
 * 256 B buffer is not on the stack underneath a command's dispatch - the
 * FrKernel task's deepest chain (frame -> dispatch -> session close -> reply
 * -> DBG_LOG formatting) has to fit its 3 KB. */
static void __attribute__((noinline)) FRKERNEL_vReplyStatus(const FrkCtx_t *pCtx,
                                                             const char *pcPrefix)
{
    char resp[FRKERNEL_RESP_BUF_LEN];
    int  n = snprintf(resp, sizeof(resp), "%s", pcPrefix);
    if ((n < 0) || ((size_t)n >= sizeof(resp)))
        n = 0;
    FRKERNEL_vFormatStatus(&resp[n], sizeof(resp) - (size_t)n);
    FRKERNEL_vReply(pCtx, resp);
}

static void FRKERNEL_vHandleFrame(const uint8_t *pu8, uint8_t u8Len)
{
    const FrkKind_e eKind  = (FrkKind_e)pu8[FRK_OFS_KIND];
    const uint32_t  u32Id  = FRK_u32GetU32(&pu8[FRK_OFS_ID]);
    const uint8_t   u8Seq  = pu8[FRK_OFS_SEQ];
    /* LORARADIO_bIsOwnId, not equality: a console can address a tag by the
     * 16-bit id an operator reads off an older label or log. */
    const bool      bForMe = LORARADIO_bIsOwnId(u32Id);
    char            resp[48];

    switch (eKind)
    {
    case FRK_KIND_CONNECT:
        if (!s_bActive)
        {
            if (!FRKERNEL_bWindowOpen())
                return;              /* not listening for a connect right now */
            FRKERNEL_vOpenSession(FRKERNEL_XPORT_LORA, bForMe);
        }
        else if (bForMe)
        {
            /* Already in a session (e.g. joined by a bulk connect): a single
             * connect to us upgrades it so we reply from now on. A repeated
             * bulk CONNECT is not activity - see FRKERNEL_INACTIVITY_TIMEOUT_MS. */
            s_bSingle         = true;
            s_eSessionXport   = FRKERNEL_XPORT_LORA;
            s_u32LastActivity = osKernelGetTickCount();
        }

        if (bForMe)
        {
            /* A (re)connecting console may have restarted its seq counter
             * (rebooted), so its first command could otherwise collide with
             * the last seq seen and be dropped as a repeat. Only on a single
             * connect: a bulk console repeats CONNECT every second, which
             * would defeat the dedupe of its own repeated commands. */
            s_bSeqValid = false;
            FrkCtx_t tCtx = { FRKERNEL_XPORT_LORA, true, u8Seq };
            FRKERNEL_vReplyStatus(&tCtx, "connected: ");
        }
        break;

    case FRK_KIND_SCAN:
        /* Answer only where the operator is plausibly standing next to the
         * tag - just shaken, or already connected. A tag in its 5 s wake
         * window stays quiet: that window is open on the whole herd at once. */
        if (s_bActive ||
            ((s_eWindow == FRKERNEL_WINDOW_SHAKE) && FRKERNEL_bWindowOpen()))
        {
            osDelay(MESHNETWORK_u32Random() % (FRKERNEL_SCAN_JITTER_MAX_MS + 1U));
            FrkCtx_t tCtx = { FRKERNEL_XPORT_LORA, true, u8Seq };
            FRKERNEL_vReplyStatus(&tCtx, s_bActive ? "scan (connected): " : "scan (shaken): ");
        }
        break;

    case FRK_KIND_CMD:
    {
        if (!s_bActive || (u8Len <= FRK_HDR_LEN))
            return;

        s_u32LastActivity = osKernelGetTickCount();
        if (bForMe)
            s_eSessionXport = FRKERNEL_XPORT_LORA;

        FrkCtx_t tCtx = { FRKERNEL_XPORT_LORA, bForMe, u8Seq };

        if (s_bSeqValid && (u8Seq == s_u8LastSeq))
        {
            /* A retry (single) or a repeat (bulk). Re-running an action would
             * do it twice; re-running a pure read is harmless and gives the
             * console the answer it missed. */
            const char cL = (char)pu8[FRK_OFS_PAYLOAD];
            if (bForMe && ((cL == 'i') || (cL == 'e') || (cL == 'f') || (cL == '?')))
                break;   /* fall out and re-run below */
            if (bForMe)
            {
                (void)snprintf(resp, sizeof(resp), "'%c' already done (repeat ignored)\r\n",
                               (cL >= ' ' && cL <= '~') ? cL : '.');
                FRKERNEL_vReply(&tCtx, resp);
            }
            return;
        }
        break;
    }

    default:
        return;
    }

    if (eKind == FRK_KIND_CMD)
    {
        s_u8LastSeq = u8Seq;
        s_bSeqValid = true;

        char acLine[FRKERNEL_RX_ITEM_LEN];
        uint8_t n = (uint8_t)(u8Len - FRK_HDR_LEN);
        if (n >= sizeof(acLine))
            n = (uint8_t)(sizeof(acLine) - 1U);
        memcpy(acLine, &pu8[FRK_OFS_PAYLOAD], n);
        acLine[n] = '\0';

        FrkCtx_t tCtx = { FRKERNEL_XPORT_LORA, bForMe, u8Seq };
        FRKERNEL_vRunLine(&tCtx, acLine);
    }
}

/* Override the weak MeshNetwork callback - called from the parser task.
 * Everything that is not a console->tag frame addressed to us (or to all) is
 * dropped HERE, before it takes a queue slot: v1 queued every FrKernel packet
 * it heard, including other tags' replies and log streams, so a 4-deep queue
 * could be full of chatter when the operator's command arrived. */
void MESHNETWORK_vOnFrKernelPacket(const uint8_t *buf, uint8_t len)
{
    if ((s_rxQueue == NULL) || (len < FRK_HDR_LEN) ||
        (buf[FRK_OFS_VERSION] != FRK_PROTO_VERSION))
        return;

    const uint8_t u8Kind = buf[FRK_OFS_KIND];
    if ((u8Kind != (uint8_t)FRK_KIND_CONNECT) && (u8Kind != (uint8_t)FRK_KIND_SCAN) &&
        (u8Kind != (uint8_t)FRK_KIND_CMD))
        return;

    const uint32_t u32Id = FRK_u32GetU32(&buf[FRK_OFS_ID]);
    if (!LORARADIO_bIsOwnId(u32Id) && (u32Id != FRK_ID_BULK))
        return;

    /* Cheap pre-filter so an idle tag does not wake its task for every
     * CONNECT beacon of a console in range: a CONNECT/CMD can only matter
     * while a window is open or a session is active. */
    if ((u8Kind != (uint8_t)FRK_KIND_SCAN) && !s_bActive && !FRKERNEL_bWindowOpen())
        return;

    FrKernelPkt_t tPkt;
    tPkt.len = (len < (uint8_t)sizeof(tPkt.data)) ? len : (uint8_t)sizeof(tPkt.data);
    memcpy(tPkt.data, buf, tPkt.len);
    if (osMessageQueuePut(s_rxQueue, &tPkt, 0U, 0U) != osOK)
    {
        if (s_u16RxDrops < UINT16_MAX)
            s_u16RxDrops++;
        return;
    }
    if (s_taskHandle != NULL)
        osThreadFlagsSet(s_taskHandle, FRKERNEL_FLAG_LORA);
}
#endif /* FRKERNEL_INTERFACE_LORA */

/* -------------------------------------------------------------------------- */
/* UART                                                                       */
/* -------------------------------------------------------------------------- */

#ifdef FRKERNEL_INTERFACE_UART
/* Override the weak HAL callback - called from the USART2 ISR. */
void UART2_vNotifyOnRX(void)
{
    if (s_taskHandle != NULL)
        osThreadFlagsSet(s_taskHandle, FRKERNEL_FLAG_UART);
}

static void FRKERNEL_vServiceUart(void)
{
    uint8_t byte;
    while (DEBUG_bReadByte(&byte))
    {
        if ((byte == '\r') || (byte == '\n'))
        {
            if (s_lineIdx > 0U)
            {
                s_lineBuf[s_lineIdx] = '\0';
                s_lineIdx = 0U;

                /* The bench UART has no connect window: the operator is
                 * holding the cable, which is consent enough. */
                if (!s_bActive)
                    FRKERNEL_vOpenSession(FRKERNEL_XPORT_UART, true);
                s_eSessionXport   = FRKERNEL_XPORT_UART;
                s_u32LastActivity = osKernelGetTickCount();

                FrkCtx_t tCtx = { FRKERNEL_XPORT_UART, true, 0U };
                FRKERNEL_vRunLine(&tCtx, s_lineBuf);
            }
        }
        else if (s_lineIdx < (FRKERNEL_LINE_BUF_LEN - 1U))
        {
            s_lineBuf[s_lineIdx++] = (char)byte;
        }
    }
}
#endif

/* -------------------------------------------------------------------------- */
/* Main task                                                                  */
/* -------------------------------------------------------------------------- */

/* Work in progress that the operator is waiting on keeps the session alive
 * even though no command arrives: a log stream, a FOTA receive/distribute, a
 * listening radio test. */
static bool FRKERNEL_bBusy(void)
{
    return DBGLOG_bDumpBusy() || FOTA_bSessionActive() ||
           (RADIOTESTMODE_bActive() && RADIOTESTMODE_bIsListener());
}

static void FRKERNEL_vTask(void *arg)
{
    (void)arg;

    for (;;)
    {
        (void)osThreadFlagsWait(FRKERNEL_FLAG_UART | FRKERNEL_FLAG_LORA,
                                osFlagsWaitAny,
                                s_bActive ? FRKERNEL_POLL_INTERVAL_MS : osWaitForever);

#ifdef FRKERNEL_INTERFACE_UART
        FRKERNEL_vServiceUart();
#endif

#ifdef FRKERNEL_INTERFACE_LORA
        FrKernelPkt_t tPkt;
        while (osMessageQueueGet(s_rxQueue, &tPkt, NULL, 0U) == osOK)
            FRKERNEL_vHandleFrame(tPkt.data, tPkt.len);
#endif

        if (!s_bActive)
            continue;

        if (s_bEndRequested)
        {
            s_bEndRequested = false;
            FRKERNEL_vCloseSession((s_pcEndReason != NULL) ? s_pcEndReason : "ended", true);
            continue;
        }

        if (FRKERNEL_bBusy())
        {
            s_u32LastActivity = osKernelGetTickCount();
        }
        else if ((osKernelGetTickCount() - s_u32LastActivity) >= FRKERNEL_INACTIVITY_TIMEOUT_MS)
        {
            FRKERNEL_vCloseSession("5 min inactivity", true);
        }
    }
}

#endif /* tag build */
