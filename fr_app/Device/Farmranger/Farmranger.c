/*
 * Farmranger.c
 *
 * Farmranger device layer — UART AT-command interface to the companion
 * logger board.
 *
 * All task / queue / semaphore management uses CMSIS-RTOS v2.
 * taskENTER_CRITICAL / taskEXIT_CRITICAL are kept (valid under the
 * CMSIS overlay) for short critical sections that protect shared buffers.
 *
 * AT-command notification protocol
 * ---------------------------------
 * ATSend() waits on two dedicated thread-flag bits:
 *   FR_AT_NOTIFY_SUCCESS  — ATHandlerTask found a matching response
 *   FR_AT_NOTIFY_TIMEOUT  — ATHandlerTask timed out
 *
 * The bits are set on the CALLER thread (osThreadGetId() at enqueue time)
 * so that ATSend() is safely re-entrant from different calling tasks.
 */

#include "Farmranger.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"   /* configMINIMAL_STACK_SIZE, taskENTER/EXIT_CRITICAL */
#include "task.h"

#include "dbg_log.h"
#include "flashLog.h"
#include "str.h"
#include "version_config.h"   /* VERSION_u32Get() — primary's own running version */
#include "LoraRadio.h"        /* LORARADIO_u32GetUniqueId() — AT+BLOG header */
#include "crc16.h"            /* AT+BLOG payload integrity */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* ---- Private defines ---- */
#define FR_RX_TASK_STACK_SIZE         (configMINIMAL_STACK_SIZE)
#define FR_AT_HANDLER_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)

/* Thread flag used by UART ISR to wake the RX task */
#define FR_RX_FLAG              (1UL << 0)

/* Thread flags set on the CALLER thread by ATHandlerTask */
#define FR_AT_NOTIFY_SUCCESS    (1UL << 8)
#define FR_AT_NOTIFY_TIMEOUT    (1UL << 9)

/* ---- RX buffer ----
 * The fr9 logger's AT responses are all short fixed tokens ("RDY\r\n",
 * "Logger ready\r\n", "OK\r\n", a 10-digit timestamp, a wake interval) -
 * the longest is ~14 bytes. 48 keeps a >3x margin. */
#define FR_RX_BUF_LEN 48
static char     acFrRxBuf[FR_RX_BUF_LEN];
static uint8_t  u8FrRxBufIdx = 0;

/* Delivered-line queue. Replaces the earlier single-slot
 * (acFrLineBuf + binary xLineReadySem) design, which lost lines whenever
 * two AT responses arrived back-to-back: for AT+LOG=0 the fr9's own state
 * machine sends "Logger ready\r\n" and "OK\r\n" on adjacent ticks (see
 * fr9's FRTAG_vLogCmdHandler), so the two land at this RX task inside a
 * single ring-drain pass. With one slot the RX task's second "\n" would
 * overwrite the still-unread "Logger ready" with "OK" and the second
 * osSemaphoreRelease would silently no-op (binary sem capped at 1). The AT
 * handler then acquired the sem and parsed "OK" against the "Logger ready"
 * parser -> mismatch, Step 1 timeout, whole log retried. A small queue
 * absorbs those bursts atomically instead. */
typedef struct { char data[FR_RX_BUF_LEN]; } FrRxLine_t;
#define FR_LINE_QUEUE_DEPTH  8

/* ---- Raw block capture (OTA firmware pull) ----
 * While armed (u16CaptureLen > 0 and not yet filled), incoming bytes bypass
 * the line path straight into the caller's buffer. The RX task only copies —
 * no parsing, no flash — so capture can't stall reception. Armed/filled/
 * cancelled from the AppTask; consumed on the RX task (volatile indices). */
static uint8_t *pu8CaptureBuf;
static volatile uint16_t u16CaptureLen;   /* capture target (0 = disarmed) */
static volatile uint16_t u16CaptureIdx;   /* bytes captured so far          */

/* ---- CMSIS-RTOS v2 objects ---- */
static osThreadId_t  Farmranger_vRxTask_handle;
static osThreadId_t  Farmranger_vATHandlerTask_handle;

static osMessageQueueId_t xLineQueue;
static osMessageQueueId_t xATQueue;

/* Minimum time PA0 (the fr9 wake line) is held low between two sessions, in
 * ms (= ticks at the 1 kHz tick). The fr9 debounces its wake input for 20 ms
 * and opens a session only on a debounced RISING edge (fr9_application
 * frtag.c: the edge interrupt sets a flag, the level is re-checked 20 ms
 * later from its main loop). A low shorter than that is invisible to it.
 * That used to happen on the basic-mode boundary flush in DeviceDiscovery,
 * which closes a session and reopens one straight away: PA0 dropped for well
 * under 1 ms, the fr9 never saw a new edge, and the tag waited out the full
 * 3 s RDY timeout ("RDY not received") while its time sync and settings
 * fetch depended on the fr9 still treating the old session as open. 50 ms is
 * the hold RadioTestMode already used by hand: 2.5x the debounce, so a late
 * poll on the fr9 side still catches the low. Enforced inside
 * FARMRANGER_bDeviceOn() so no caller has to remember it. */
#define FR_WAKE_MIN_LOW_MS      50U

/* PA0 state as this driver last drove it, and the tick at which it last went
 * low. bFrWakeLowTickValid stays false until the first session ends: until
 * then PA0 has been low since HAL_GPIO_vInit() in main(), far longer than
 * FR_WAKE_MIN_LOW_MS, so the first session after boot does not wait. */
static bool     bFrWakeHigh          = false;
static bool     bFrWakeLowTickValid  = false;
static uint32_t u32FrWakeLowTick;

/* ---- Device state ---- */
bool bFRDeviceOn;

/* Set from the fr9's ready line: true when it read "RDY,B", i.e. this logger
 * accepts the binary discovery upload. Written on the AT handler task inside
 * bParseRDY and read on the AppTask after bDeviceOn returns, which is a plain
 * happens-before through the thread-flag handshake in bATSend - no barrier
 * needed, but it is cleared before every wait so a failed handshake can never
 * leave the previous session's answer standing. */
static bool bFRLoggerBinary;

/* ---- AT parser type ---- */
typedef bool (*ATParserFn)(const char *line, void *context);

typedef struct {
    const char    *cmd;
    ATParserFn     parser;
    void          *context;
    char          *out;
    size_t         outLen;
    osThreadId_t   caller;
    uint32_t       timeout;     /* ms — 1 tick == 1 ms */
} ATReq_t;

/* ---- UART handle ---- */
struct _farmranger_s {
    hal_uart_t UartHandle;
    uint8_t    byte;
} farmranger;

/* ---- Forward declarations ---- */
static bool FARMRANGER_bATSend(const char *cmd,
                               ATParserFn parser,
                               char *out,
                               size_t outLen,
                               void *context,
                               uint32_t timeout);
static void FARMRANGER_vATHandlerTask(void *args);
static bool FARMRANGER_bParseTimestamp(const char *line, void *ctx);
static bool FARMRANGER_bParseSettings(const char *line, void *ctx);
static bool FARMRANGER_bParseLoggerReady(const char *line, void *ctx);
static bool FARMRANGER_bParseLogVerdict(const char *line, void *ctx);
static bool FARMRANGER_bParseRDY(const char *line, void *ctx);

/* --------------------------------------------------------------------------
 * FARMRANGER_vInit
 * -------------------------------------------------------------------------- */
void FARMRANGER_vInit(void)
{
    bFRDeviceOn = false;

    FR_DRIVER_vInitFRDevice(&farmranger.UartHandle);

    xLineQueue       = osMessageQueueNew(FR_LINE_QUEUE_DEPTH, sizeof(FrRxLine_t), NULL);
    xATQueue         = osMessageQueueNew(4, sizeof(ATReq_t), NULL);

    configASSERT(xLineQueue     != NULL);
    configASSERT(xATQueue       != NULL);

    static const osThreadAttr_t rx_attr = {
        .name       = "FRRxTask",
        .stack_size = FR_RX_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityRealtime,
    };
    static const osThreadAttr_t at_attr = {
        .name       = "FRAtHandlerTask",
        .stack_size = FR_AT_HANDLER_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityHigh,
    };

    Farmranger_vRxTask_handle        = osThreadNew(FARMRANGER_vRxTask,        NULL, &rx_attr);
    Farmranger_vATHandlerTask_handle = osThreadNew(FARMRANGER_vATHandlerTask, NULL, &at_attr);

    configASSERT(Farmranger_vRxTask_handle        != NULL);
    configASSERT(Farmranger_vATHandlerTask_handle  != NULL);
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vUartOnWake
 * -------------------------------------------------------------------------- */
void FARMRANGER_vUartOnWake(void)
{
    HAL_UART_vInit();
    FR_DRIVER_vInitFRDevice(&farmranger.UartHandle);
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vRxTask — drains UART ring buffer, signals line-ready
 * -------------------------------------------------------------------------- */
void FARMRANGER_vRxTask(void *parameters)
{
    uint8_t byte;

    for (;;)
    {
        /* Block until UART ISR notifies us that a byte arrived */
        osThreadFlagsWait(FR_RX_FLAG, osFlagsWaitAny, osWaitForever);

        /* Drain all available bytes from the ring buffer */
        while (UART_bReadByte(&farmranger.UartHandle, &byte))
        {
            /* Raw block capture (OTA pull): bytes bypass the line path. */
            if (u16CaptureIdx < u16CaptureLen)
            {
                pu8CaptureBuf[u16CaptureIdx] = byte;
                u16CaptureIdx++;
                continue;
            }

            if (u8FrRxBufIdx < FR_RX_BUF_LEN - 1)
                acFrRxBuf[u8FrRxBufIdx++] = byte;

            if (byte == '\n')
            {
                /* Publish the completed line into the queue. Non-blocking:
                 * if the queue is full (shouldn't happen with the 8-slot
                 * depth for our protocol) the newest line is dropped rather
                 * than the RX task stalling and letting the UART ring
                 * overflow. */
                FrRxLine_t line;
                uint8_t n = (u8FrRxBufIdx < (uint8_t)sizeof(line.data))
                          ? u8FrRxBufIdx
                          : (uint8_t)(sizeof(line.data) - 1U);
                memcpy(line.data, acFrRxBuf, n);
                line.data[n] = '\0';
                (void)osMessageQueuePut(xLineQueue, &line, 0U, 0U);

                u8FrRxBufIdx = 0;
                memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vNotifyOnRX — called from UART RX ISR via uart_callbacks.c
 * -------------------------------------------------------------------------- */
void FARMRANGER_vNotifyOnRX(void)
{
    if (Farmranger_vRxTask_handle != NULL)
        osThreadFlagsSet(Farmranger_vRxTask_handle, FR_RX_FLAG);
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vWakeLineLow — drive PA0 low and note when, for the minimum
 * low-time check in FARMRANGER_bDeviceOn().
 * -------------------------------------------------------------------------- */
static void FARMRANGER_vWakeLineLow(void)
{
    FR_DRIVER_vIntDisable();
    bFrWakeHigh         = false;
    u32FrWakeLowTick    = osKernelGetTickCount();
    bFrWakeLowTickValid = true;
}

/* --------------------------------------------------------------------------
 * FARMRANGER_bDeviceOn — power up device and wait for RDY
 * -------------------------------------------------------------------------- */
bool FARMRANGER_bDeviceOn(void)
{
    osThreadResume(Farmranger_vRxTask_handle);

    if (bFRDeviceOn)
        return true;

    /* Clear stale state */
    taskENTER_CRITICAL();
    memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
    u8FrRxBufIdx = 0;
    taskEXIT_CRITICAL();
    osMessageQueueReset(xLineQueue);        /* drop any queued stale lines */

    /* UART first, then the PA0 rising edge. The fr9 opens a session on the
     * rising edge of PA0 (after a ~20 ms debounce) and answers RDY some time
     * after it, so USART1 is attached before the edge: the tag's receiver is
     * ready whenever the fr9 talks, without relying on the fr9's current
     * ~1 s response delay. And with PB6 already push-pull at its idle-high
     * level, the fr9's RX sees a clean idle line when it wakes instead of
     * whatever a parked analog pin floats to (PB6 has no MCU pull).
     *
     * This is the reverse of the GNSS power order on a secondary (rail, then
     * pins: GPS_vHwPowerOn in GPS.c), and deliberately so: on a V3 RevA
     * primary PA0 is only the fr9 wake line, and no GNSS module is fitted, so
     * there is no unpowered 1.8 V I/O on these pins to protect. */
    FR_DRIVER_vEnableUart(&farmranger.UartHandle);

    /* A rising edge needs PA0 to have been low for at least
     * FR_WAKE_MIN_LOW_MS (see its definition). If an earlier attempt failed
     * on RDY without FARMRANGER_vDeviceOff() being called, PA0 is still high
     * and raising it again would be no edge at all, so drop it first. Then
     * wait out whatever part of the minimum low time has not yet passed since
     * PA0 last went low. */
    if (bFrWakeHigh)
        FARMRANGER_vWakeLineLow();
    if (bFrWakeLowTickValid)
    {
        uint32_t u32LowMs = osKernelGetTickCount() - u32FrWakeLowTick;
        if (u32LowMs < FR_WAKE_MIN_LOW_MS)
            osDelay(FR_WAKE_MIN_LOW_MS - u32LowMs);
    }
    FR_DRIVER_vIntEnable();
    bFrWakeHigh = true;

    /* Assume no binary support until the ready line says otherwise, so a
     * handshake that fails or an fr9 that is swapped out between sessions can
     * never leave a stale "yes" behind. */
    bFRLoggerBinary = false;

    char respBuf[32] = {0};
    DBG_LOG("Wait for RDY...\r\n");
    if (!FARMRANGER_bATSend(NULL,
                            FARMRANGER_bParseRDY,
                            respBuf,
                            sizeof(respBuf),
                            respBuf,
                            3000))
    {
        DBG_LOG("RDY not received\r\n");
        return false;
    }

    bFRDeviceOn = true;
    DBG_LOG("Farmranger Ready (binary upload %s).\r\n",
            bFRLoggerBinary ? "supported" : "not supported");
    return true;
}

bool FARMRANGER_bLoggerSupportsBinary(void)
{
    return bFRDeviceOn && bFRLoggerBinary;
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vDeviceOff
 * -------------------------------------------------------------------------- */
void FARMRANGER_vDeviceOff(void)
{
    osThreadSuspend(Farmranger_vRxTask_handle);
    /* UART off (PB6/PB7 back to analog), then PA0 low. The fr9 only acts on
     * the next RISING edge after a debounced low; FARMRANGER_bDeviceOn()
     * enforces the minimum low time (FR_WAKE_MIN_LOW_MS) from the tick
     * recorded here, so a caller may reopen a session immediately. */
    FR_DRIVER_vDisableUart(&farmranger.UartHandle);
    FARMRANGER_vWakeLineLow();
    bFRDeviceOn = false;
    DBG_LOG("Farmranger released.\r\n");
}

/* --------------------------------------------------------------------------
 * FARMRANGER_bATSend — enqueue an AT command and block for the response
 *
 * Returns true on success, false on timeout or queue-full.
 * The ATHandlerTask notifies this thread via thread flags:
 *   FR_AT_NOTIFY_SUCCESS  → response matched
 *   FR_AT_NOTIFY_TIMEOUT  → timed out
 * -------------------------------------------------------------------------- */
static bool FARMRANGER_bATSend(const char *cmd,
                               ATParserFn parser,
                               char *out,
                               size_t outLen,
                               void *context,
                               uint32_t timeout)
{
    ATReq_t req = {
        .cmd     = cmd,
        .parser  = parser,
        .context = context,
        .out     = out,
        .outLen  = outLen,
        .caller  = osThreadGetId(),
        .timeout = timeout,
    };

    /* Clear any stale AT notification flags on this thread. Do NOT drain
     * xLineQueue here -- the AT handler resets it after dequeueing this
     * request, and doing it here as well would race with anything already
     * in flight from the caller task's own perspective. */
    osThreadFlagsClear(FR_AT_NOTIFY_SUCCESS | FR_AT_NOTIFY_TIMEOUT);

    configASSERT(xATQueue != NULL);
    if (osMessageQueuePut(xATQueue, &req, 0, 100) != osOK)
        return false;

    uint32_t r = osThreadFlagsWait(FR_AT_NOTIFY_SUCCESS | FR_AT_NOTIFY_TIMEOUT,
                                   osFlagsWaitAny,
                                   timeout);

    if (r & osFlagsError)
        return false;

    return (r & FR_AT_NOTIFY_SUCCESS) ? true : false;
}

/* --------------------------------------------------------------------------
 * FARMRANGER_vATHandlerTask — dequeues ATReq_t and polls for a response
 * -------------------------------------------------------------------------- */
static void FARMRANGER_vATHandlerTask(void *args)
{
    ATReq_t req;

    for (;;)
    {
        if (osMessageQueueGet(xATQueue, &req, NULL, osWaitForever) != osOK)
            continue;

        /* Reset RX state and drop any stale queued lines from before this
         * command was issued. Bytes accumulating in acFrRxBuf that don't
         * yet form a complete line stay put -- the RX task will finish
         * them on the next '\n' -- but partial garbage from a previous
         * command shouldn't hang around either. */
        taskENTER_CRITICAL();
        u8FrRxBufIdx = 0;
        memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
        taskEXIT_CRITICAL();
        osMessageQueueReset(xLineQueue);

        if (req.cmd && strlen(req.cmd) > 0)
        {
            HAL_UART_vTxPutBuffer(&farmranger.UartHandle,
                                  (uint8_t *)req.cmd,
                                  strlen(req.cmd));
        }

        uint32_t start     = osKernelGetTickCount();
        bool     notified  = false;
        FrRxLine_t line;

        while ((osKernelGetTickCount() - start) < req.timeout)
        {
            if (osMessageQueueGet(xLineQueue, &line, NULL, 50U) == osOK)
            {
                if (req.parser(line.data, req.context))
                {
                    osThreadFlagsSet(req.caller, FR_AT_NOTIFY_SUCCESS);
                    notified = true;
                    break;
                }
                /* Non-matching line -- discard and keep waiting. Queue
                 * absorbs the next line (e.g. an early "OK" that arrived
                 * back-to-back with the "Logger ready" we were looking
                 * for) so nothing is lost. */
            }
        }

        if (!notified)
            osThreadFlagsSet(req.caller, FR_AT_NOTIFY_TIMEOUT);
    }
}

/* --------------------------------------------------------------------------
 * FARMRANGER_u64RequestTimestamp
 * -------------------------------------------------------------------------- */
uint64_t FARMRANGER_u64RequestTimestamp(void)
{
    char     tsStr[32]  = {0};
    uint64_t tsValue    = 0;

    if (FARMRANGER_bATSend("AT+TSREQ\r\n",
                           FARMRANGER_bParseTimestamp,
                           tsStr,
                           sizeof(tsStr),
                           tsStr,
                           2000))
    {
        tsValue = strtoull(tsStr, NULL, 10);
    }

    return tsValue;
}

/* --------------------------------------------------------------------------
 * FARMRANGER_bRequestSettings
 *
 * Fetch the three per-system settings the primary needs at the start of
 * every wake, in one AT round trip. Response wire format on the fr9 side
 * (see FRTAG_vSetReqHandler in fr9_application/fr_app/src/frtag.c):
 *
 *     "<interval_min>,<lorawan_mode>,<gps_enabled>\r\n"
 *
 * where lorawan_mode and gps_enabled are 0 or 1 (the mode field used to mean
 * basic mode; fr9 still answers 1 for its codes 62/63, which v2.5.0 reads as
 * LoRaWAN mode - see DiscoveryMode_e). Any bogus/short response
 * leaves the outputs untouched and returns false; caller keeps its
 * previous defaults (safe cold-boot behavior).
 * -------------------------------------------------------------------------- */
bool FARMRANGER_bRequestSettings(uint8_t *pu8Interval,
                                 bool    *pbLoRaWanMode,
                                 bool    *pbGpsEnabled)
{
    char respBuf[32] = {0};

    if (!FARMRANGER_bATSend("AT+SETREQ\r\n",
                            FARMRANGER_bParseSettings,
                            respBuf,
                            sizeof(respBuf),
                            respBuf,
                            10000))
    {
        return false;
    }

    /* Parse "<int>,<mode>,<gps>" (line stripped of trailing CRLF by the
     * parser). All three fields must be present and numeric or we reject
     * — safer to fall back to caller defaults than partial apply. */
    char *p = respBuf;
    char *endptr;

    unsigned long ulInterval = strtoul(p, &endptr, 10);
    if (endptr == p || *endptr != ',') return false;
    p = endptr + 1;

    unsigned long ulMode = strtoul(p, &endptr, 10);
    if (endptr == p || *endptr != ',') return false;
    p = endptr + 1;

    unsigned long ulGps = strtoul(p, &endptr, 10);
    if (endptr == p) return false;

    if (pu8Interval)  *pu8Interval  = (uint8_t)ulInterval;
    if (pbLoRaWanMode) *pbLoRaWanMode = (ulMode != 0UL);
    if (pbGpsEnabled) *pbGpsEnabled = (ulGps  != 0UL);
    return true;
}

/* --------------------------------------------------------------------------
 * AT+LOG (CSV) upload removed — primary now speaks AT+BLOG only. See
 * FARMRANGER_bBLogData below.
 * -------------------------------------------------------------------------- */

/* Inter-row pacing for the binary upload (ms). The fr9 receives into a
 * 128-byte UART RX ring drained by a co-operative loop that echoes every
 * byte to USB; if the whole payload is streamed back-to-back at line rate
 * that ring overflows and the fr9 never counts the full length (→ "LOG
 * TIMEOUT"). We therefore wait for each chunk to leave the wire and pause
 * briefly, bounding the in-flight data and giving the fr9 time to drain. */
#define FR_LOG_ROW_GAP_MS 10U

/* Upper bound (ms) on waiting for one upload chunk to leave the wire. A chunk
 * is at most FR_BLOG_RECS_PER_CHUNK * FR_BLOG_RECORD_BYTES = 48 bytes, ~4 ms
 * at 115200, so this only trips when USART1 is wedged. Without a bound such a
 * wait hangs the AppTask with the campaign sleep lock held, so the device
 * never reaches STOP2 again. */
#define FR_LOG_TX_DRAIN_TIMEOUT_MS 3500U

/* Upload retry policy. The fr9 signals a failed transfer (bytes lost on the
 * wire, e.g. its RX ring overflowed during a GNSS acquire or a flash-log page
 * erase) with "ERR" 5 s after the last byte it received, so the verdict wait
 * must exceed that; an fr9 running older firmware answers "OK" either way,
 * which degrades this to today's single-attempt behaviour. */
#define FR_LOG_ATTEMPTS        3U
#define FR_LOG_VERDICT_MS      6500U
#define FR_LOG_RETRY_DELAY_MS  250U

/* ==========================================================================
 * Binary discovery upload (AT+BLOG)
 *
 * See Farmranger.h for the wire format and why it exists. What follows is the
 * CSV path's structure with the formatting replaced; the parts that are the
 * same are the same on purpose - the pacing, the attempt count and the 6.5 s
 * verdict window are all values the CSV path arrived at against real fr9
 * behaviour, and none of them changes because the bytes are binary.
 *
 * Two differences worth knowing:
 *
 * - No count==0 special case. The CSV path needs one because AT+LOG=0 has no
 *   payload at all, so the fr9 answers "Logger ready" and "OK" on adjacent
 *   ticks and a queue reset between them loses the OK. An empty binary
 *   campaign still has its 8-byte header to send, so the normal three-step
 *   flow applies and that race cannot arise here.
 * - The CRC has to be known before the header line goes out, so pass 1 folds
 *   it while it counts. That is the only reason pass 1 exists at all; the
 *   length is just FR_BLOG_HDR_BYTES + count*FR_BLOG_RECORD_BYTES.
 * ========================================================================== */

/* Records per UART write.
 *
 * The bound is the in-flight byte count, not the record count: it must stay
 * at or under the ~67-byte worst case of a single CSV row - the in-flight size
 * that FR_LOG_ROW_GAP_MS was tuned against and that the fr9's 128-byte
 * co-operatively-drained RX ring is known to survive. With v2's 22-byte
 * records that was 3 per chunk (66 B). v3's 24-byte records would make 3 a
 * 72-byte chunk, past the proven size, so it is 2 (48 B). Larger chunks would
 * be faster and are probably fine, but "fine" here means "does not overflow a
 * ring on the other board"; the extra chunks cost ~0.3 s on a full 120-node
 * campaign. */
#define FR_BLOG_RECS_PER_CHUNK  2U
_Static_assert(FR_BLOG_RECS_PER_CHUNK * FR_BLOG_RECORD_BYTES <= 67U,
               "AT+BLOG chunk exceeds the in-flight size the fr9 RX ring is proven against");

/* ---- little-endian writers ---- */

static void FARMRANGER_vPut16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void FARMRANGER_vPut32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

/* Payload header: version, record type, count, this primary's device id,
 * discovery duration (0 when the caller has none to report — see the
 * FR_BLOG_HDR_BYTES note in Farmranger.h). */
static void FARMRANGER_vPackHeader(uint8_t *hdr, uint8_t u8Type, uint16_t count,
                                   uint32_t u32DurationS)
{
    hdr[0] = (uint8_t)FR_BLOG_FORMAT_VER;
    hdr[1] = u8Type;
    FARMRANGER_vPut16(&hdr[2], count);
    FARMRANGER_vPut32(&hdr[4], LORARADIO_u32GetUniqueId());
    FARMRANGER_vPut32(&hdr[8], u32DurationS);
}

/* Advanced record, FR_BLOG_RECORD_BYTES bytes. Field order is the wire
 * contract in Farmranger.h, NOT the struct's own layout - see the note there
 * about why this is written out a field at a time. */
static void FARMRANGER_vPackRow(uint8_t *rec, const MeshDiscoveredNeighbor_t *n)
{
    FARMRANGER_vPut32(&rec[0],  n->u32DeviceId);
    FARMRANGER_vPut32(&rec[4],  (uint32_t)n->i32LatUDeg);
    FARMRANGER_vPut32(&rec[8],  (uint32_t)n->i32LonUDeg);
    FARMRANGER_vPut16(&rec[12], (uint16_t)n->i16Rssi);
    FARMRANGER_vPut16(&rec[14], n->u16BatMv);
    FARMRANGER_vPut16(&rec[16], n->u16BestRssiSrcId);
    rec[18] = n->u8HopCount;
    rec[19] = n->u8Wave;
    rec[20] = n->u8FwPatch;
    /* bit0 carries u8MoveState as-is (0 = moving, 1 = still). Not inverted
     * into a "moving" boolean anywhere along the way - the fr9 renders this
     * straight back into the Move column it has always printed. */
    rec[21] = (uint8_t)((n->u8MoveState ? 0x01U : 0x00U)
                      | (n->bGpsValid  ? 0x02U : 0x00U));
    FARMRANGER_vPut16(&rec[22], n->u16MoveCnt);     /* v3 */
}

/* Queue a chunk and wait for it to leave the wire, then pause. Same shape as
 * the CSV row loop: bound the in-flight bytes to one chunk so the fr9's RX
 * ring cannot overflow. Returns false on a TX stall. */
static bool FARMRANGER_bBLogSendChunk(const uint8_t *p, uint16_t n)
{
    HAL_UART_vTxPutBuffer(&farmranger.UartHandle, (uint8_t *)p, n);

    uint32_t u32TxStart = osKernelGetTickCount();
    while (!HAL_UART_bTxIdle(&farmranger.UartHandle))
    {
        if ((osKernelGetTickCount() - u32TxStart) >= FR_LOG_TX_DRAIN_TIMEOUT_MS)
        {
            EVTLOG(LOG_FRLOG_ERROR, 2);
            DBG_LOG("UART TX timeout (BLOG)\r\n");
            return false;
        }
        osDelay(1);
    }

    osDelay(FR_LOG_ROW_GAP_MS);
    return true;
}

/* Wait for the fr9's verdict. Read INLINE off xLineQueue without resetting it,
 * for the same reason the CSV path does - see the long note in
 * FARMRANGER_bLogAttempt's Step 3. Returns true only on "OK". */
static bool FARMRANGER_bBLogVerdict(void)
{
    char       cVerdict    = '\0';
    uint32_t   u32Start    = osKernelGetTickCount();
    bool       bGotVerdict = false;
    FrRxLine_t line;

    while ((osKernelGetTickCount() - u32Start) < FR_LOG_VERDICT_MS)
    {
        if (osMessageQueueGet(xLineQueue, &line, NULL, 50U) == osOK)
        {
            if (FARMRANGER_bParseLogVerdict(line.data, &cVerdict))
            {
                bGotVerdict = true;
                break;
            }
        }
    }

    if (!bGotVerdict)
    {
        DBG_LOG("BLogData: no verdict received (Step 3 timeout).\r\n");
        EVTLOG(LOG_FRLOG_ERROR, 3);
        return false;
    }

    if (cVerdict != 'O')
    {
        DBG_LOG("BLogData: fr9 reported ERR (crc or bytes lost).\r\n");
        EVTLOG(LOG_FRLOG_ERROR, 4);
        return false;
    }

    return true;
}

/* One AT+BLOG attempt. `u16Len` and `u16Crc` come from the single pass the
 * caller already made over the table, so an attempt never re-folds the CRC. */
static bool FARMRANGER_bBLogAttempt(const MeshDiscoveredNeighbor_t *neighbors,
                                    uint16_t count, uint16_t u16Len, uint16_t u16Crc,
                                    uint32_t u32DurationS)
{
    uint8_t  au8Chunk[FR_BLOG_RECS_PER_CHUNK * FR_BLOG_RECORD_BYTES];
    uint8_t  au8Hdr[FR_BLOG_HDR_BYTES];
    char     cmd[48];
    char     respBuf[32] = {0};

    snprintf(cmd, sizeof(cmd), "AT+BLOG=%u,%lu,%04X\r\n",
             (unsigned)u16Len, (unsigned long)VERSION_u32Get(), (unsigned)u16Crc);

    DBG_LOG("BLogData: AT+BLOG=%u,%lu,%04X (%u records)\r\n",
            (unsigned)u16Len, (unsigned long)VERSION_u32Get(),
            (unsigned)u16Crc, (unsigned)count);

    /* Step 1: without "Logger ready" the fr9 is not in its payload state, so
     * the stream would go into the void. Fail rather than transmit blind. */
    if (!FARMRANGER_bATSend(cmd,
                            FARMRANGER_bParseLoggerReady,
                            respBuf,
                            sizeof(respBuf),
                            respBuf,
                            1000))
    {
        EVTLOG(LOG_FRLOG_ERROR, 1);
        DBG_LOG("BLogData: No 'Logger ready' received (Step 1 fail).\r\n");
        return false;
    }

    /* Step 2: header, then the records in paced chunks. */
    FARMRANGER_vPackHeader(au8Hdr, (uint8_t)FR_BLOG_TYPE_ADVANCED, count, u32DurationS);
    if (!FARMRANGER_bBLogSendChunk(au8Hdr, (uint16_t)FR_BLOG_HDR_BYTES))
        return false;

    for (uint16_t i = 0; i < count; )
    {
        uint16_t u16InChunk = 0;

        while ((u16InChunk < FR_BLOG_RECS_PER_CHUNK) && (i < count))
        {
            FARMRANGER_vPackRow(&au8Chunk[u16InChunk * FR_BLOG_RECORD_BYTES],
                                &neighbors[i]);
            u16InChunk++;
            i++;
        }

        if (!FARMRANGER_bBLogSendChunk(au8Chunk,
                (uint16_t)(u16InChunk * FR_BLOG_RECORD_BYTES)))
            return false;
    }

    /* Step 3 */
    return FARMRANGER_bBLogVerdict();
}

bool FARMRANGER_bBLogData(MeshDiscoveredNeighbor_t *neighbors, uint16_t count,
                          uint32_t u32DurationS)
{
    uint8_t  au8Hdr[FR_BLOG_HDR_BYTES];
    uint8_t  au8Rec[FR_BLOG_RECORD_BYTES];
    uint16_t u16Len;
    uint16_t u16Crc;

    /* Pass 1: fold the CRC over exactly the bytes pass 2 will send. One record
     * buffer, so RAM use is constant in the neighbour count - the same reason
     * the CSV path makes two passes over one row buffer instead of building
     * the whole payload. */
    u16Len = (uint16_t)(FR_BLOG_HDR_BYTES + (count * FR_BLOG_RECORD_BYTES));

    FARMRANGER_vPackHeader(au8Hdr, (uint8_t)FR_BLOG_TYPE_ADVANCED, count, u32DurationS);
    u16Crc = CRC16_u16CcittUpdate(CRC16_CCITT_INIT, au8Hdr, (uint16_t)FR_BLOG_HDR_BYTES);

    for (uint16_t i = 0; i < count; i++)
    {
        FARMRANGER_vPackRow(au8Rec, &neighbors[i]);
        u16Crc = CRC16_u16CcittUpdate(u16Crc, au8Rec, (uint16_t)FR_BLOG_RECORD_BYTES);
    }

    /* Pass 2: upload, retrying a failed transfer. The fr9 returns to idle
     * before the next AT+BLOG goes out because the verdict wait outlasts its
     * silence timeout, so each attempt is self-contained. */
    for (uint8_t u8Attempt = 1U; u8Attempt <= FR_LOG_ATTEMPTS; u8Attempt++)
    {
        if (FARMRANGER_bBLogAttempt(neighbors, count, u16Len, u16Crc, u32DurationS))
        {
            if (u8Attempt > 1U)
                DBG_LOG("BLogData: upload OK on attempt %u\r\n", u8Attempt);
            return true;
        }

        if (u8Attempt < FR_LOG_ATTEMPTS)
        {
            DBG_LOG("BLogData: attempt %u failed, retrying...\r\n", u8Attempt);
            osDelay(FR_LOG_RETRY_DELAY_MS);
        }
    }

    DBG_LOG("BLogData: all %u attempts failed\r\n", FR_LOG_ATTEMPTS);
    return false;
}

/* --------------------------------------------------------------------------
 * Firmware-file pull (OTA acquire) — see Farmranger.h for the protocol
 * -------------------------------------------------------------------------- */

/* Response timeouts. A 1 KB block at 115200 with the fr9's in-block pacing is
 * ~150 ms on the wire plus the fr9's modem-filesystem QFREAD turnaround. */
#define FR_FW_QUERY_TIMEOUT_MS  1500U
#define FR_FW_BLOCK_TIMEOUT_MS  2000U

typedef struct {
    FarmrangerFw_e eResult;
    uint32_t       u32Version;
    uint32_t       u32FileBytes;
    uint8_t        u8ExpectedXor;
} FwQueryCtx_t;

typedef struct {
    uint32_t u32Offset;
    uint8_t  u8Xor;
} FwTrailerCtx_t;

/* "FW,NONE" | "FW,WAIT" | "FW,<verMMmmpp>,<fileBytes>,<xor8hex>" */
static bool FARMRANGER_bParseFwInfo(const char *line, void *ctx)
{
    FwQueryCtx_t *pt = (FwQueryCtx_t *)ctx;

    const char *p = strstr(line, "FW,");
    if (p == NULL)
        return false;
    p += 3;

    if (strncmp(p, "NONE", 4) == 0) { pt->eResult = FARMRANGER_FW_NONE; return true; }
    if (strncmp(p, "WAIT", 4) == 0) { pt->eResult = FARMRANGER_FW_WAIT; return true; }

    char *pcEnd;
    uint32_t u32Ver = strtoul(p, &pcEnd, 10);
    if (pcEnd == p || *pcEnd != ',')
        return false;
    uint32_t u32Bytes = strtoul(pcEnd + 1, &pcEnd, 10);
    if (u32Bytes == 0UL || *pcEnd != ',')
        return false;
    uint8_t u8Xor = (uint8_t)strtoul(pcEnd + 1, &pcEnd, 16);

    pt->eResult       = FARMRANGER_FW_AVAILABLE;
    pt->u32Version    = u32Ver;
    pt->u32FileBytes  = u32Bytes;
    pt->u8ExpectedXor = u8Xor;
    return true;
}

/* "FB,<offset>,<xor8hex>" — sent by the logger after the raw block bytes. */
static bool FARMRANGER_bParseFwTrailer(const char *line, void *ctx)
{
    FwTrailerCtx_t *pt = (FwTrailerCtx_t *)ctx;

    const char *p = strstr(line, "FB,");
    if (p == NULL)
        return false;
    p += 3;

    char *pcEnd;
    pt->u32Offset = strtoul(p, &pcEnd, 10);
    if (pcEnd == p || *pcEnd != ',')
        return false;
    pt->u8Xor = (uint8_t)strtoul(pcEnd + 1, NULL, 16);
    return true;
}

/* Plain "OK" acknowledgement (AT+FWDONE). ctx is unused. */
static bool FARMRANGER_bParseOK(const char *line, void *ctx)
{
    (void)ctx;
    return (line != NULL && strstr(line, "OK") != NULL);
}

/* "+FWCHECK: OK" | "+FWCHECK: BUSY" */
static bool FARMRANGER_bParseFwCheckAck(const char *line, void *ctx)
{
    bool *pbOk = (bool *)ctx;

    if (line == NULL)
        return false;
    if (strstr(line, "+FWCHECK: OK") != NULL)   { *pbOk = true;  return true; }
    if (strstr(line, "+FWCHECK: BUSY") != NULL) { *pbOk = false; return true; }
    return false;
}

bool FARMRANGER_bFwCheckRequest(uint32_t u32CurrentVer, bool bHasStaged)
{
    char cmd[32];
    char respBuf[32] = {0};
    bool bAcked = false;

    snprintf(cmd, sizeof(cmd), "AT+FWCHECK=%lu,%u\r\n",
             (unsigned long)u32CurrentVer, bHasStaged ? 1U : 0U);

    if (!FARMRANGER_bATSend(cmd,
                            FARMRANGER_bParseFwCheckAck,
                            respBuf,
                            sizeof(respBuf),
                            &bAcked,
                            FR_FW_QUERY_TIMEOUT_MS))
    {
        return false;   /* no response from the logger */
    }

    return bAcked;   /* false on "+FWCHECK: BUSY" — caller may still poll FWREQ */
}

FarmrangerFw_e FARMRANGER_eFwQuery(uint32_t *pu32Version, uint32_t *pu32FileBytes,
                                   uint8_t *pu8ExpectedXor)
{
    FwQueryCtx_t tCtx = { .eResult = FARMRANGER_FW_NONE };
    char respBuf[32] = {0};

    if (!FARMRANGER_bATSend("AT+FWREQ\r\n",
                            FARMRANGER_bParseFwInfo,
                            respBuf,
                            sizeof(respBuf),
                            &tCtx,
                            FR_FW_QUERY_TIMEOUT_MS))
    {
        return FARMRANGER_FW_NONE;   /* no answer — treat as nothing to fetch */
    }

    *pu32Version    = tCtx.u32Version;
    *pu32FileBytes  = tCtx.u32FileBytes;
    *pu8ExpectedXor = tCtx.u8ExpectedXor;
    return tCtx.eResult;
}

bool FARMRANGER_bFwGetBlock(uint32_t u32Offset, uint16_t u16Len, uint8_t *pu8Buf)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+FWGET=%lu,%u\r\n",
             (unsigned long)u32Offset, (unsigned)u16Len);

    /* Arm the raw capture BEFORE the command goes out so the first response
     * byte can never race past the line path. Length is written last — it is
     * the enable. */
    pu8CaptureBuf = pu8Buf;
    u16CaptureIdx = 0U;
    u16CaptureLen = u16Len;

    FwTrailerCtx_t tTrailer = {0};
    char respBuf[32] = {0};
    bool bOk = FARMRANGER_bATSend(cmd,
                                  FARMRANGER_bParseFwTrailer,
                                  respBuf,
                                  sizeof(respBuf),
                                  &tTrailer,
                                  FR_FW_BLOCK_TIMEOUT_MS);

    /* Disarm capture whatever happened (lost bytes leave it partly filled —
     * the trailer line then got eaten by the capture and bOk is false). */
    uint16_t u16Got = u16CaptureIdx;
    u16CaptureLen = 0U;
    u16CaptureIdx = 0U;
    pu8CaptureBuf = NULL;

    if (!bOk || u16Got != u16Len || tTrailer.u32Offset != u32Offset)
        return false;

    /* Verify the block against the logger's XOR-8. */
    uint8_t u8Xor = 0U;
    for (uint16_t i = 0U; i < u16Len; i++)
        u8Xor ^= pu8Buf[i];

    return (u8Xor == tTrailer.u8Xor);
}

bool FARMRANGER_bFwReportDone(bool bOk)
{
    char respBuf[32] = {0};
    return FARMRANGER_bATSend(bOk ? "AT+FWDONE=OK\r\n" : "AT+FWDONE=ERR\r\n",
                              FARMRANGER_bParseOK,
                              respBuf,
                              sizeof(respBuf),
                              respBuf,
                              FR_FW_QUERY_TIMEOUT_MS);
}

/* --------------------------------------------------------------------------
 * AT response parsers
 * -------------------------------------------------------------------------- */

static bool FARMRANGER_bParseTimestamp(const char *line, void *ctx)
{
    char *out = (char *)ctx;
    size_t len = strlen(line);

    /* Expect: 10 digits + "\r\n" */
    if (len == 12 && line[10] == '\r' && line[11] == '\n')
    {
        for (int i = 0; i < 10; i++)
            if (line[i] < '0' || line[i] > '9') return false;

        memcpy(out, line, 10);
        out[10] = '\0';
        return true;
    }
    return false;
}

/* Parse the AT+SETREQ response: "<int>,<mode>,<gps>\r\n" (up to ~12 B).
 * Only lightly validated here — the tighter numeric parse happens in
 * FARMRANGER_bRequestSettings after CRLF strip. We just accept any line
 * that ends in "\r\n" and copy the pre-CRLF part into ctx. */
static bool FARMRANGER_bParseSettings(const char *line, void *ctx)
{
    char  *out = (char *)ctx;
    size_t len = strlen(line);

    /* Need at least "0,0,0\r\n" = 7 bytes, cap at whatever the ATSend
     * caller sized its buffer to (sizeof(respBuf) in FARMRANGER_bRequestSettings). */
    if (len < 7 || line[len - 2] != '\r' || line[len - 1] != '\n')
        return false;

    if (len - 2 >= 32) return false;   /* would overflow the 32-B respBuf */

    memcpy(out, line, len - 2);
    out[len - 2] = '\0';
    return true;
}

static bool FARMRANGER_bParseLoggerReady(const char *line, void *ctx)
{
    (void)ctx;
    return (line != NULL && strstr(line, "Logger ready") != NULL);
}

/* Transfer verdict: matches "OK" (success) or "ERR" (fr9 timed out — bytes
 * were lost on the wire). Writes 'O'/'E' into ctx so the caller can tell the
 * two apart; matching ERR here (instead of only OK) lets a failed upload
 * fail fast rather than burn the whole verdict window. Note "ERR" must be
 * checked first: strstr("ERR...", "OK") can't false-match, but keeping the
 * explicit order makes the precedence obvious. */
static bool FARMRANGER_bParseLogVerdict(const char *line, void *ctx)
{
    char *pcVerdict = (char *)ctx;

    if (line == NULL)
        return false;

    if (strstr(line, "ERR") != NULL)
    {
        *pcVerdict = 'E';
        return true;
    }
    if (strstr(line, "OK") != NULL)
    {
        *pcVerdict = 'O';
        return true;
    }
    return false;
}

/* The fr9 sends "RDY\r\n" on firmware before 9.12.13 and "RDY,B\r\n" from
 * 9.12.13, where ",B" means "I accept AT+BLOG". Matching on "RDY" alone, as
 * this always has, is what makes the suffix safe to add; the suffix is then
 * read separately so the caller can pick the binary path without having to
 * probe for it and eat three timeouts against an older board. */
static bool FARMRANGER_bParseRDY(const char *line, void *ctx)
{
    (void)ctx;

    if (line == NULL || strstr(line, "RDY") == NULL)
        return false;

    bFRLoggerBinary = (strstr(line, ",B") != NULL);

    return true;
}

