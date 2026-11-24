/*
 * LoraRadio.c
 *
 * LoRa radio device layer — CMSIS-RTOS v2 task, queues and
 * carrier-sense back-off.
 *
 * IMPORTANT — osThreadFlagsWait mask must be 0x7FFFFFFFU
 * --------------------------------------------------------
 * osThreadFlagsWait() wraps xTaskNotifyWait() but only clears the bits
 * that are IN the supplied mask.  If any other bits are pending in the
 * notification word when the call is made, xTaskNotifyWait returns
 * immediately, the non-masked bits remain, and the next call does the
 * same — producing an infinite spin loop that burns the entire timeout
 * without ever seeing a CAD result.
 *
 * Using 0x7FFFFFFFU as the mask guarantees all pending bits are
 * consumed on every wait; non-CAD events that arrive during carrier-
 * sense are stashed and replayed in the main loop.
 */

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "LoraRadio.h"
#include "LoraRadio_Driver.h"
#include "dbg_log.h"
#include "radio_driver.h"
#include "build_config.h"

#include <string.h>
#include <stdlib.h>

/* Per-packet radio trace (TX done, CAD busy/clear, CAD back-off): one or two
 * lines for EVERY transmission. Compiled out unless ENABLE_RADIO_VERBOSE_LOG.
 * During a kernel log stream they alone filled the 4 KB DbgLog ring within
 * ~30 frames and so pushed out every real event, and on the kernel console
 * they scrolled a line per CONNECT beacon across the operator's terminal. */
#ifdef ENABLE_RADIO_VERBOSE_LOG
#  define RADIO_VDBG(...)   DBG(__VA_ARGS__)
#else
#  define RADIO_VDBG(...)   do { } while (0)
#endif

/* ---- Private defines ---- */
/* TX queue: 8 (was 24). Each slot is a full ~264 B LoraRadio_Packet_t from the
 * 48 KB heap, and the mesh layer already paces its TX through its own 24-deep
 * jitter queue upstream — this queue only buffers what the radio task is about
 * to drain. 8 still absorbs a multi-line FrKernel LoRa response burst;
 * overflow is handled gracefully (logged drop). Frees ~4.2 KB of heap. */
#define LORA_TX_QUEUE_SIZE      (8)
#define LORA_RX_QUEUE_SIZE      (8)
#define LORA_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 10)

#define CAD_BASE_BACKOFF_MS     100
#define CAD_MAX_BACKOFF_MS      2000
#define CAD_MAX_EXPONENT        4

/* Per-packet carrier-sense budget in the TX drain. Was 5000 ms inline; named
 * here because it multiplies with the drain batch size to bound how long the
 * radio can be away from RX. */
#define LORA_TX_CARRIER_WAIT_MS 5000U

/* Max packets sent per pass through the radio task's event loop.
 *
 * This used to drain the ENTIRE queue in one `while`, never returning to the
 * event loop until empty. With an 8-deep queue and a 5 s carrier-sense budget
 * each, a congested node could sit up to ~40 s inside that loop — and carrier
 * sense runs the chip in CAD, not RX, so the device is deaf for the whole
 * time. A heavy-forwarding unit in the field missed the primary's TimeSync in
 * 39 of 47 campaigns that way, which meant FOTA acceptance was never armed and
 * it silently refused every firmware update. Draining a couple of packets per
 * pass keeps the radio cycling back through RX. */
#define LORA_TX_DRAIN_PER_PASS  2U

/* Packets older than this are dropped instead of transmitted. A mesh forward
 * that has been stuck behind a congested queue for this long is stale — the
 * campaign that cared about it has moved on — and sending it only adds to the
 * congestion that delayed it. */
#define LORA_TX_MAX_AGE_MS      10000U

/* Mask covering all 31 usable CMSIS thread-flag bits */
#define ALL_FLAGS               (0x7FFFFFFFU)

/* ---- CMSIS-RTOS v2 objects ---- */
static osMessageQueueId_t xLoRaTxQueue;
static osMessageQueueId_t xLoRaRxQueue;
static osThreadId_t       LORARADIO_vRadioTask_handle;

/* CAD-timeout tally. A CAD that never reports a result within the 300 ms
 * window is normal-ish under congestion but used to emit a DBG_LOG line every
 * single time — hundreds per campaign, drowning the flash log. Counted here
 * and reported once per campaign via the mesh stats line. */
static volatile uint16_t u16CadTimeoutCount = 0;

/* Packets discarded unsent for exceeding LORA_TX_MAX_AGE_MS, plus those
 * dropped by LORARADIO_vFlushTxQueue(). Reported alongside the CAD tally. */
static volatile uint16_t u16TxStaleDropCount = 0;

/* Pending radio events captured during carrier-sense / back-off */
static volatile uint32_t gRadioPendingEvents = 0;

/* Most recent channel noise-floor sample, dBm. Written only by the radio task
 * on its idle pass and stamped into each received packet — see the sampling
 * site in LORARADIO_vRadioTask for why that timing is the meaningful one.
 * Only sampled while bNoiseFloorSampling is set, which the radio-test paths
 * turn on and normal operation leaves off. */
static int16_t          i16NoiseFloor      = LORA_NOISE_FLOOR_INVALID;
static volatile bool    bNoiseFloorSampling = false;

/* True from the moment a packet is pulled off xLoRaTxQueue until this task is
 * done with it (sent, or dropped after carrier-sense failed) — the window in
 * which the queue itself reads empty even though a TX is still in flight,
 * because osMessageQueueGet() dequeues before carrier-sense runs. Queue depth
 * alone cannot tell a caller "wait, something is still being tried"; this can.
 * See LORARADIO_u16GetTxQueueDepth()'s header comment for the caller that
 * needs it. */
static volatile bool    bTxInProgress      = false;

/* While set, LORARADIO_vEnterDeepSleep() is a no-op — see the comment there.
 * Held by the runtime radio range test for its duration, because the chip has
 * other owners whose normal teardown would otherwise sleep it mid-test. */
static volatile bool    bKeepAwake         = false;

/* Tick at which the PA last actually fired: the TX that OTA_LORA_CHUNK_GAP_MS
 * and OTA_VERIFY_TX_SETTLE_MS wait out, because flash reads issued too close
 * to a TX have been seen to come back corrupted (the mechanism is untested
 * and is not a proven rail sag; see OTA_LORA_CHUNK_GAP_MS in Fota_Config.h).
 *
 * This is the one timestamp nothing in the codebase recorded. Every existing
 * log line marks when a packet was *queued*, and queue time says nothing
 * about PA time: MESHNETWORK_bSendPacket only stamps a ready-tick (jitter
 * 20-1500 ms), and this task's carrier sense can then defer the real
 * transmission by up to LORA_TX_CARRIER_WAIT_MS on top. So "was the radio
 * transmitting while that flash read happened?" has never been answerable
 * from a field log. Written here on the radio task, read by the FOTA verify
 * path. */
static volatile uint32_t u32LastTxDoneTick = 0U;

/* True from the moment a packet is committed to transmission until its PA
 * pulse has actually fired (or the attempt was abandoned).
 *
 * The TX queue going empty is NOT sufficient to conclude the radio is about to
 * stay quiet: carrier sense can hold an already-dequeued packet for up to
 * LORA_TX_CARRIER_WAIT_MS before the PA fires, during which the queue reads as
 * idle. Anything timing a quiet window off the queue alone would sail straight
 * into that TX. */
static volatile bool bTxInFlight = false;

/* Chip power state as this layer last set it (see LORARADIO_bIsAsleep). True
 * from LORARADIO_vInit's sleep until the radio task's first wake. */
static volatile bool    bAsleep            = true;

/* Beacon + listen window (LORARADIO_bStartBeaconWindow). bWindowBusy is the
 * claim: set under a critical section by the requester only while the chip is
 * asleep, cleared by the radio task when the window is over. vWakeUp and
 * vEnterDeepSleep wait for it, which is the whole of the coordination with the
 * chip's other owners. The frame is a few bytes of static, not a packet: the
 * full 264 B LoraRadio_Packet_t would not fit in this build's RAM. */
static volatile bool    bWindowBusy        = false;
static uint8_t          au8WindowFrame[LORA_BEACON_FRAME_MAX + 1U];  /* + CRC */
static uint8_t          u8WindowFrameLen   = 0U;
static uint32_t         u32WindowListenMs  = 0U;

/* Carrier on (LORARADIO_bStartCw). Written by the radio task only. */
static volatile bool    bCw                = false;

/* -------------------------------------------------------------------------- */

static uint8_t LORARADIO_u8CRC8_Calculate(const uint8_t *data, uint16_t len);
static void    LORARADIO_vNotifyFromISR(uint32_t evt);
static void    LORARADIO_vRunBeaconWindow(void);
static void    LORARADIO_vWaitBeaconWindow(void);

static inline void LORARADIO_vStashPendingEvents(uint32_t evt)
{
    taskENTER_CRITICAL();
    gRadioPendingEvents |= evt;
    taskEXIT_CRITICAL();
}

static inline uint32_t LORARADIO_u32ConsumePendingEvents(void)
{
    uint32_t v;
    taskENTER_CRITICAL();
    v = gRadioPendingEvents;
    gRadioPendingEvents = 0;
    taskEXIT_CRITICAL();
    return v;
}

/* --------------------------------------------------------------------------
 * LORARADIO_vInit
 * -------------------------------------------------------------------------- */
void LORARADIO_vInit(void)
{
    xLoRaTxQueue = osMessageQueueNew(LORA_TX_QUEUE_SIZE, sizeof(LoraRadio_Packet_t), NULL);
    xLoRaRxQueue = osMessageQueueNew(LORA_RX_QUEUE_SIZE, sizeof(LoraRadio_Packet_t), NULL);
    configASSERT(xLoRaTxQueue != NULL && xLoRaRxQueue != NULL);  /* && not ||: both must succeed */

    /*
     * Initialise hardware and enter deep sleep BEFORE creating the task.
     * The task calls vWakeUp() as its first action; creating it first risks
     * the scheduler running it before we reach vEnterDeepSleep() here,
     * leaving the radio awake while the task has already passed vWakeUp().
     */
    LORARADIO_DRIVER_vInit();
    LORARADIO_vEnterDeepSleep();

    DBG("\r\nLoraradio: Device ID %08lX\r\n", (unsigned long)LORARADIO_u32GetUniqueId());

    static const osThreadAttr_t radioTask_attr = {
        .name       = "LoRaRadioTask",
        .stack_size = LORA_TASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };
    LORARADIO_vRadioTask_handle = osThreadNew(LORARADIO_vRadioTask, NULL, &radioTask_attr);
    configASSERT(LORARADIO_vRadioTask_handle != NULL);
}

/* --------------------------------------------------------------------------
 * LORARADIO_bRxPacket — blocks until a verified packet is available
 * -------------------------------------------------------------------------- */
bool LORARADIO_bRxPacket(LoraRadio_Packet_t *packet)
{
    return (osMessageQueueGet(xLoRaRxQueue, packet, NULL, osWaitForever) == osOK);
}

/* --------------------------------------------------------------------------
 * LORARADIO_bTxPacket — enqueue a packet for transmission
 * -------------------------------------------------------------------------- */
bool LORARADIO_bTxPacket(LoraRadio_Packet_t *packet)
{
    /* Reserve room for the appended CRC byte: the length field is uint8_t, so
     * type+payload+CRC must fit in 255. A 255-byte payload would wrap
     * pkt.length to 0 on the CRC append in the radio task (corrupt frame). */
    if (packet->length > (LORA_MAX_PACKET_SIZE - 2))
        return false;

    packet->u32QueuedTick = osKernelGetTickCount();   /* for the stale-drop check */

    if (osMessageQueuePut(xLoRaTxQueue, packet, 0, 0) != osOK)
    {
        DBG_LOG("Loraradio: TX PKT queue full\r\n");
        return false;
    }

    osThreadFlagsSet(LORARADIO_vRadioTask_handle, RADIO_EVT_TX_PENDING);
    return true;
}

/* --------------------------------------------------------------------------
 * LORARADIO_bTxPacketWait — enqueue, blocking while the queue is full
 * -------------------------------------------------------------------------- */
bool LORARADIO_bTxPacketWait(LoraRadio_Packet_t *packet, uint32_t timeoutMs)
{
    if (packet->length > (LORA_MAX_PACKET_SIZE - 2))
        return false;

    /* Blocking put: FreeRTOS wakes this caller the moment the radio task's
     * osMessageQueueGet() pulls the head item, i.e. as soon as it has
     * actually finished with the previous packet -- no polling, no guessed
     * delay. */
    packet->u32QueuedTick = osKernelGetTickCount();   /* for the stale-drop check */

    if (osMessageQueuePut(xLoRaTxQueue, packet, 0, timeoutMs) != osOK)
        return false;

    osThreadFlagsSet(LORARADIO_vRadioTask_handle, RADIO_EVT_TX_PENDING);
    return true;
}

/* --------------------------------------------------------------------------
 * LORARADIO_vRadioTask — main radio state machine
 * -------------------------------------------------------------------------- */
void LORARADIO_vRadioTask(void *arg)
{
    (void)arg;
    LoraRadio_Packet_t pkt;

    /*
     * LORARADIO_vInit() puts the radio into deep sleep (WarmStart=0) so the
     * chip draws minimal current during boot.  A plain vEnterRxMode() after
     * WarmStart=0 sleep would leave modulation/frequency/TX-power registers
     * in an undefined state.  vWakeUp() performs a full re-initialisation
     * (LORARADIO_DRIVER_vInit + vEnterRxMode) before the task loop begins.
     */
    LORARADIO_vWakeUp();

    for (;;)
    {
        /* Wait up to 50 ms for any radio event — ALL_FLAGS clears every bit */
        uint32_t events = osThreadFlagsWait(ALL_FLAGS, osFlagsWaitAny, 50);
        if (events & osFlagsError) events = 0;

        /* Merge any stashed events from a carrier-sense interruption */
        events |= LORARADIO_u32ConsumePendingEvents();

        /* ---------- BEACON + LISTEN WINDOW ----------
         * Only ever requested while the chip sleeps, so nothing else below
         * can be in progress; the window owns the chip until it returns. */
        if (events & RADIO_EVT_BEACON_WINDOW)
        {
            LORARADIO_vRunBeaconWindow();
            events &= ~RADIO_EVT_BEACON_WINDOW;
            events |= LORARADIO_u32ConsumePendingEvents();
        }

        /* ---------- CONTINUOUS WAVE ----------
         * Stop is handled first, so a start and a stop that land in one wait
         * (an exit right behind an entry) end with the carrier off. */
        if (events & RADIO_EVT_CW_STOP)
        {
            if (bCw)
            {
                SUBGRF_SetStandby(STDBY_RC);
                bCw = false;
                LORARADIO_DRIVER_vEnterRxMode(0);
            }
            events &= ~(RADIO_EVT_CW_STOP | RADIO_EVT_CW_START);
        }
        if ((events & RADIO_EVT_CW_START) && !bAsleep)
        {
            SUBGRF_SetStandby(STDBY_RC);
            SUBGRF_SetRfFrequency(RF_FREQUENCY);
            SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
            SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
            SUBGRF_SetTxContinuousWave();
            bCw = true;
        }
        if (bCw)
        {
            /* The carrier owns the chip: nothing is received (it is in TX)
             * and nothing queued may go out, because sending it would mean
             * standby + CAD - the end of the carrier. */
            LORARADIO_vFlushTxQueue();
            continue;
        }

        /* ---------- NOISE FLOOR ----------
         * A bare 50 ms timeout with nothing queued: no IRQ fired, no TX is
         * waiting, and every path out of this loop leaves the chip in
         * continuous RX — so the receiver is live and the channel is quiet,
         * which is exactly the condition an instantaneous RSSI reading needs
         * to mean "noise floor" rather than "some other signal".
         *
         * Sampled here rather than from the RadioTest task because this task
         * is the sole owner of the chip; a second task issuing SUBGRF
         * commands would race this one on the SPI.
         *
         * Sampling *before* the packet rather than after it is the point: the
         * reading taken while the channel was last idle is the right
         * reference for the beacon that lands next, and it costs no blocking
         * delay waiting for the receiver to settle after an RX.
         *
         * Off unless a radio test switched it on, so normal builds issue no
         * extra SPI traffic here. */
        if (bNoiseFloorSampling &&
            events == 0U && osMessageQueueGetCount(xLoRaTxQueue) == 0U)
            i16NoiseFloor = (int16_t)SUBGRF_GetRssiInst();

        /* ---------- RX DONE ---------- */
        if (events & RADIO_EVT_RX_DONE)
        {
            memset(&pkt, 0, sizeof(pkt));
            LORARADIO_DRIVER_bReceivePayload(&pkt);
            pkt.i16NoiseFloor = i16NoiseFloor;

            /* Shortest valid frame is type byte + CRC byte. A 0/1-byte frame
             * (corrupt header that passed the radio CRC) would otherwise
             * underflow pkt.length - 1 below: buffer[-1] read and a CRC pass
             * over (uint16_t)-1 = 65535 bytes, far past the 256-byte buffer. */
            if (pkt.length < 2)
            {
                DBG_LOG("Loraradio: runt frame dropped (len=%u)\r\n", pkt.length);
            }
            else
            {
                uint8_t crc_rx = pkt.buffer[pkt.length - 1];
                uint8_t crc    = LORARADIO_u8CRC8_Calculate(pkt.buffer, pkt.length - 1);
                if (crc == crc_rx)
                {
                    pkt.length--;

                    /* Fast reply BEFORE anything else - the sender is
                     * listening for only a few tens of ms. No carrier sense
                     * (the sender has just finished, the channel is ours) and
                     * no queue: transmitted from here, then the packet that
                     * triggered it is queued as usual. */
                    uint8_t au8Reply[LORA_FAST_REPLY_MAX_LEN + 1U];
                    uint8_t u8ReplyLen = 0U;
                    if (LORARADIO_bFastReply(&pkt, au8Reply, &u8ReplyLen) &&
                        (u8ReplyLen > 0U) && (u8ReplyLen <= LORA_FAST_REPLY_MAX_LEN))
                    {
                        au8Reply[u8ReplyLen] = LORARADIO_u8CRC8_Calculate(au8Reply, u8ReplyLen);
                        bTxInFlight = true;
                        (void)LORARADIO_DRIVER_bTransmitPayloadEx(au8Reply, (uint8_t)(u8ReplyLen + 1U),
                                                                  LORA_FAST_REPLY_PREAMBLE_LENGTH,
                                                                  false);
                        u32LastTxDoneTick = osKernelGetTickCount();
                        bTxInFlight       = false;
                    }

                    osMessageQueuePut(xLoRaRxQueue, &pkt, 0, 0);
                }
                else
                {
                    DBG_LOG("CRC mismatch\r\n");
                }
            }
            SUBGRF_ClearIrqStatus(IRQ_RX_DONE);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- ERRORS ----------
         * Log every hardware-detected packet fault. These paths used to be
         * completely silent, which made it impossible to distinguish "no
         * packet was on the air" from "packet arrived corrupted and got
         * dropped by the SX126x's own LoRa CRC check". During OTA
         * distribution debugging especially, these lines are what tells
         * you whether an intermittent 1-chunk-missing pattern is a real
         * RF corruption event vs. some upstream logic bug. */
        if (events & RADIO_EVT_CRC_ERROR)
        {
            DBG_LOG("LoraRadio: RX CRC error (packet dropped by hardware)\r\n");
            SUBGRF_ClearIrqStatus(IRQ_CRC_ERROR);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }
        if (events & RADIO_EVT_HEADER_ERROR)
        {
            DBG_LOG("LoraRadio: RX header error (packet dropped by hardware)\r\n");
            SUBGRF_ClearIrqStatus(IRQ_HEADER_ERROR);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }
        if (events & RADIO_EVT_TIMEOUT)
        {
            SUBGRF_ClearIrqStatus(IRQ_RX_TX_TIMEOUT);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- TX DONE ---------- */
        if (events & RADIO_EVT_TX_DONE)
        {
            RADIO_VDBG("LoraRadio: TX done IRQ received\r\n");
            SUBGRF_ClearIrqStatus(IRQ_TX_DONE);
            LORARADIO_DRIVER_vEnterRxMode(0);
        }

        /* ---------- TX REQUEST ----------
         * Bounded drain: at most LORA_TX_DRAIN_PER_PASS packets before falling
         * back to the event loop. Carrier sense puts the chip in CAD, not RX,
         * so time spent here is time spent deaf; emptying the whole queue in
         * one go could hold that for tens of seconds and make the node miss
         * the very TimeSync/OtaPrep it needs (see LORA_TX_DRAIN_PER_PASS).
         * Anything still queued is picked up next pass — the queue-count test
         * above re-enters this block without needing a fresh TX_PENDING. */
        if ((events & RADIO_EVT_TX_PENDING) ||
            (osMessageQueueGetCount(xLoRaTxQueue) > 0))
        {
            uint8_t u8Sent = 0U;

            while (u8Sent < LORA_TX_DRAIN_PER_PASS &&
                   osMessageQueueGet(xLoRaTxQueue, &pkt, NULL, 0) == osOK)
            {
                /* Drop rather than send a packet that has been stuck behind a
                 * congested queue: it is a mesh forward whose moment has
                 * passed, and transmitting it now only feeds the congestion
                 * that delayed it. Doesn't count against the send budget. */
                if ((osKernelGetTickCount() - pkt.u32QueuedTick) > LORA_TX_MAX_AGE_MS)
                {
                    if (u16TxStaleDropCount < UINT16_MAX) u16TxStaleDropCount++;
                    continue;
                }

                u8Sent++;
                bTxInProgress = true;

                /* Committed to sending this one: from here until the PA has
                 * fired (or we give up below) the radio is not quiet, even
                 * though the queue may now read as empty. */
                bTxInFlight = true;

                uint8_t crc = LORARADIO_u8CRC8_Calculate(pkt.buffer, pkt.length);
                pkt.buffer[pkt.length++] = crc;

                if (!LORARADIO_bCarrierSenseAndWait(LORA_TX_CARRIER_WAIT_MS))
                {
                    bTxInFlight = false;   /* abandoned - no PA pulse happened */
                    uint32_t stashed = LORARADIO_u32ConsumePendingEvents();
                    if (stashed)
                    {
                        events |= stashed;
                        DBG_LOG("Loraradio: TX REQ interrupted by events 0x%08lX\r\n", stashed);
                        LORARADIO_DRIVER_vEnterRxMode(0);
                        bTxInProgress = false;
                        continue;
                    }
                    DBG_LOG("Loraradio: TX REQ abort — carrier busy\r\n");
                    LORARADIO_DRIVER_vEnterRxMode(0);
                    bTxInProgress = false;
                    continue;
                }

                LORARADIO_DRIVER_bTransmitPayload(pkt.buffer, pkt.length);
                u32LastTxDoneTick = osKernelGetTickCount();
                bTxInFlight       = false;
                LORARADIO_DRIVER_vEnterRxMode(0x00);
                bTxInProgress = false;
            }
        }
    }
}

/* -------------------------------------------------------------------------- */

void LORARADIO_vSetNoiseFloorSampling(bool bEnable)
{
    /* Drop the stale reading when switching off, so a later run cannot report
     * a noise floor measured minutes ago in a different place. */
    if (!bEnable)
        i16NoiseFloor = LORA_NOISE_FLOOR_INVALID;

    bNoiseFloorSampling = bEnable;
}

int16_t LORARADIO_i16GetNoiseFloor(void)
{
    return i16NoiseFloor;
}

uint16_t LORARADIO_u16GetTxQueueDepth(void)
{
    return (uint16_t)osMessageQueueGetCount(xLoRaTxQueue);
}

bool LORARADIO_bTxInProgress(void)
{
    return bTxInProgress;
}

/* --------------------------------------------------------------------------
 * Radio device id
 *
 * Built from the MCU's 96-bit unique ID. RM0461 lays that ID out as:
 *   w0[31:16] X and w0[15:0] Y position of the die on its wafer, BCD
 *   w1[7:0]   wafer number
 *   w1[31:8]  + w2[31:0]   lot number, 7 ASCII characters
 *
 * The id used to be 16 bits: the low byte of X and the low byte of Y.
 * That is a die POSITION, and every wafer has the same set of positions, so
 * two tags cut from the same spot on two different wafers have always shared
 * an id. Being BCD it also has at most 100 x 100 values. This was never going
 * to be "running out of numbers" - it was collisions from the first batch on.
 *
 * The id is now 32 bits:
 *   [31:24] wafer number
 *   [23:16] CRC-8 of the lot number (see LORA_ID_CRC8_POLY for the exact
 *           algorithm and byte order)
 *   [15:0]  the old 16-bit id, unchanged
 *
 * Keeping the old id as the low half is deliberate. Every tag keeps its old
 * number as (uint16_t)id, the fields that stayed 16 bits on air (the top half
 * of every mesh message id, the DReq sender id) still carry it, and a peer on
 * older firmware that only knows the 16 bits can still be matched against us
 * - see LORARADIO_bSameDevice.
 *
 * Within one lot the id is unique by construction: X, Y and wafer number
 * together name exactly one die. Across lots two tags collide only if they sit
 * at the same position on same-numbered wafers AND their lots hash to the same
 * byte. That is rare, not impossible; the production station is what turns it
 * into a guarantee, by refusing a unit whose id is already in its database.
 *
 * The UID64 "UDN" at 0x1FFF7580 would have been unique outright, but it is a
 * separate number and would have thrown away every tag's existing id.
 *
 * Computed on first use and cached: the inputs are fixed silicon, so a race
 * between two first callers just computes the same value twice. Zero means
 * "not computed yet"; a real id is never zero because its high half is not.
 * -------------------------------------------------------------------------- */
static uint32_t s_u32UniqueId = 0U;

static uint8_t LORARADIO_u8IdCrc8(const uint8_t *pu8, uint8_t u8Len)
{
    uint8_t u8Crc = 0x00U;
    for (uint8_t i = 0U; i < u8Len; i++)
    {
        u8Crc ^= pu8[i];
        for (uint8_t b = 0U; b < 8U; b++)
            u8Crc = (u8Crc & 0x80U) ? (uint8_t)((u8Crc << 1) ^ LORA_ID_CRC8_POLY)
                                    : (uint8_t)(u8Crc << 1);
    }
    return u8Crc;
}

uint32_t LORARADIO_u32GetUniqueId(void)
{
    if (s_u32UniqueId != 0U)
        return s_u32UniqueId;

    uint32_t au32Uid[3];
    LORARADIO_DRIVER_vGetUid96(au32Uid);

    const uint16_t u16Lo = (uint16_t)((((au32Uid[0] >> 16) & 0xFFU) << 8) |
                                      (au32Uid[0] & 0xFFU));

    const uint8_t au8Lot[7] = {
        (uint8_t)(au32Uid[1] >>  8), (uint8_t)(au32Uid[1] >> 16),
        (uint8_t)(au32Uid[1] >> 24),
        (uint8_t)(au32Uid[2]      ), (uint8_t)(au32Uid[2] >>  8),
        (uint8_t)(au32Uid[2] >> 16), (uint8_t)(au32Uid[2] >> 24),
    };
    uint16_t u16Hi = (uint16_t)(((au32Uid[1] & 0xFFU) << 8) |
                                LORARADIO_u8IdCrc8(au8Lot, sizeof(au8Lot)));
    if (u16Hi == 0U)
        u16Hi = LORA_ID_HI_ZERO_SUBST;

    s_u32UniqueId = ((uint32_t)u16Hi << 16) | u16Lo;
    return s_u32UniqueId;
}

uint16_t LORARADIO_u16GetShortId(void)
{
    return (uint16_t)LORARADIO_u32GetUniqueId();
}

bool LORARADIO_bSameDevice(uint32_t u32A, uint32_t u32B)
{
    if (u32A == u32B)
        return true;
    if ((uint16_t)u32A != (uint16_t)u32B)
        return false;
    /* Same legacy id, different high halves: the same tag only if one side
     * does not know its high half. */
    return ((u32A >> 16) == 0U) || ((u32B >> 16) == 0U);
}

bool LORARADIO_bIsOwnId(uint32_t u32Id)
{
    return LORARADIO_bSameDevice(u32Id, LORARADIO_u32GetUniqueId());
}

void LORARADIO_vEventRxDone(void)      { LORARADIO_vNotifyFromISR(RADIO_EVT_RX_DONE);      }
void LORARADIO_vEventTxDone(void)      { LORARADIO_vNotifyFromISR(RADIO_EVT_TX_DONE);      }
void LORARADIO_vEventCrcError(void)    { LORARADIO_vNotifyFromISR(RADIO_EVT_CRC_ERROR);    }
void LORARADIO_vEventHeaderError(void) { LORARADIO_vNotifyFromISR(RADIO_EVT_HEADER_ERROR); }
void LORARADIO_vEventTimeout(void)     { LORARADIO_vNotifyFromISR(RADIO_EVT_TIMEOUT);      }
void LORARADIO_vEventCADDetected(void) { LORARADIO_vNotifyFromISR(RADIO_EVT_CAD_BUSY);     }
void LORARADIO_vEventCADClear(void)    { LORARADIO_vNotifyFromISR(RADIO_EVT_CAD_CLEAR);    }

/* Byte-wise XOR fold — a weak checksum, but kept as such deliberately for
 * over-the-air compatibility with older firmware in the field. A real CRC
 * upgrade is a hard protocol break (mixed old/new mesh drops every packet
 * between mismatched pairs, including the OTA packets that would deliver
 * the upgrade) so we rely on the SX126x's own hardware LoRa CRC (enabled
 * via LORA_CRC_ON in the packet params, and its failure reported through
 * IRQ_CRC_ERROR / RADIO_EVT_CRC_ERROR) as the primary line of defence
 * against RF corruption; this app-layer check is a secondary sanity check
 * only. Do NOT change this to a real CRC without a coordinated flash of
 * every device on the mesh via SWD. */
static uint8_t LORARADIO_u8CRC8_Calculate(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

uint32_t LORARADIO_u32GetRandomNumber(uint32_t max_value)
{
    return LORARADIO_DRIVER_u32GetRandomNumber(max_value);
}

/* --------------------------------------------------------------------------
 * LORARADIO_bCarrierSense — single CAD measurement
 *
 * Uses ALL_FLAGS (0x7FFFFFFF) so that any event that arrives during the
 * 300 ms window — including RX_DONE, CRC_ERROR etc. — is consumed and
 * the wait does not spin.  Non-CAD events are stashed for the main loop.
 * -------------------------------------------------------------------------- */
bool LORARADIO_bCarrierSense(void)
{
    LORARADIO_DRIVER_vEnterCAD();

    uint32_t r = osThreadFlagsWait(ALL_FLAGS, osFlagsWaitAny, 300);

    if (r & osFlagsError)
    {
        /* Counted, not logged per occurrence: under congestion this fires
         * hundreds of times per campaign and used to bury everything else in
         * the flash log. The tally is reported once per campaign instead —
         * see LORARADIO_u16GetAndClearCadTimeouts(). */
        if (u16CadTimeoutCount < UINT16_MAX) u16CadTimeoutCount++;
        return false;
    }

    /* TX_PENDING is self-noise here: we're already inside TX processing for
     * a packet already dequeued off xLoRaTxQueue (that's why CAD is running
     * at all). LORARADIO_bTxPacket() sets this flag unconditionally, and it
     * can still be pending the first time we wait on it if the flag was set
     * after the radio task's top-of-loop flags-wait already snapshotted (a
     * fast responder, e.g. FrKernel answering a just-received request, wins
     * that race essentially every time). Treating our own trigger as a
     * foreign interrupting event caused every first-attempt TX to abort and
     * drop the packet with no retry. */
    r &= ~RADIO_EVT_TX_PENDING;

    if (r & RADIO_EVT_CAD_BUSY)
    {
        RADIO_VDBG("Loraradio: CAD busy\r\n");
        return false;
    }
    if (r & RADIO_EVT_CAD_CLEAR)
    {
        RADIO_VDBG("Loraradio: CAD clear\r\n");
        return true;
    }

    if (r)
    {
        /* Another, genuinely foreign radio event arrived during CAD — stash
         * it, report busy. */
        LORARADIO_vStashPendingEvents(r);
    }
    return false;
}

uint16_t LORARADIO_u16GetAndClearCadTimeouts(void)
{
    uint16_t u16Count = u16CadTimeoutCount;
    u16CadTimeoutCount = 0U;
    return u16Count;
}

uint16_t LORARADIO_u16GetAndClearTxStaleDrops(void)
{
    uint16_t u16Count = u16TxStaleDropCount;
    u16TxStaleDropCount = 0U;
    return u16Count;
}

uint32_t LORARADIO_u32GetLastTxTick(void)
{
    return u32LastTxDoneTick;
}

bool LORARADIO_bTxQuiet(uint32_t u32SettleMs)
{
    /* Ordered cheapest-first, and all three conditions are needed:
     *  - in-flight covers a packet already dequeued and sitting in carrier
     *    sense, which the queue count cannot see;
     *  - queue count covers packets not yet picked up;
     *  - the settle age covers a PA pulse that has just happened, which is the
     *    whole point - flash reads too soon after a TX have been seen to come
     *    back corrupted (see OTA_LORA_CHUNK_GAP_MS; the cause is untested and
     *    more likely noise on the long SPI traces than rail sag). */
    if (bTxInFlight) return false;

    if ((xLoRaTxQueue != NULL) && (osMessageQueueGetCount(xLoRaTxQueue) != 0U))
        return false;

    if (u32LastTxDoneTick == 0U) return true;   /* nothing transmitted yet */

    /* Unsigned difference, so tick wrap is handled. */
    return ((osKernelGetTickCount() - u32LastTxDoneTick) >= u32SettleMs);
}

void LORARADIO_vFlushTxQueue(void)
{
    LoraRadio_Packet_t tDiscard;
    uint16_t           u16Dropped = 0U;

    if (xLoRaTxQueue == NULL) return;

    while (osMessageQueueGet(xLoRaTxQueue, &tDiscard, NULL, 0) == osOK)
        u16Dropped++;

    if (u16Dropped > 0U)
    {
        if ((uint32_t)u16TxStaleDropCount + u16Dropped > UINT16_MAX)
            u16TxStaleDropCount = UINT16_MAX;
        else
            u16TxStaleDropCount = (uint16_t)(u16TxStaleDropCount + u16Dropped);
    }
}

/* --------------------------------------------------------------------------
 * LORARADIO_bCarrierSenseAndWait — adaptive back-off loop
 * -------------------------------------------------------------------------- */
bool LORARADIO_bCarrierSenseAndWait(uint32_t maxWaitMs)
{
    uint32_t startMs  = osKernelGetTickCount();
    uint32_t failCount = 0;

    while ((osKernelGetTickCount() - startMs) < maxWaitMs)
    {
        if (LORARADIO_bCarrierSense()) return true;

        if (gRadioPendingEvents) return false;

        uint32_t exponent = (failCount > CAD_MAX_EXPONENT) ? CAD_MAX_EXPONENT : failCount;
        uint32_t window   = CAD_BASE_BACKOFF_MS << exponent;
        if (window > CAD_MAX_BACKOFF_MS) window = CAD_MAX_BACKOFF_MS;

        uint32_t backoffMs = CAD_BASE_BACKOFF_MS
            + LORARADIO_u32GetRandomNumber(window - CAD_BASE_BACKOFF_MS + 1);

        RADIO_VDBG("Loraradio: CAD back-off %lu ms (fail=%lu)\r\n", backoffMs, failCount);

        /* Back-off sleep — also uses ALL_FLAGS so any event wakes us cleanly */
        uint32_t r = osThreadFlagsWait(ALL_FLAGS, osFlagsWaitAny, backoffMs);
        if (!(r & osFlagsError))
        {
            /* Same self-noise as LORARADIO_bCarrierSense() — see comment
             * there. Strip it before deciding whether something genuinely
             * foreign woke us. */
            r &= ~RADIO_EVT_TX_PENDING;

            if (r & RADIO_EVT_CAD_CLEAR) return true;
            if (r & RADIO_EVT_CAD_BUSY)  return false;
            if (r)
            {
                LORARADIO_vStashPendingEvents(r);
                return false;
            }
            /* Only our own TX_PENDING fired — spurious wake, retry CAD. */
        }

        failCount++;
    }

    DBG_LOG("Loraradio: carrier-sense timed out after %lu ms\r\n", maxWaitMs);
    return false;
}

void LORARADIO_vEnterDeepSleep(void)
{
    /* Refuse while someone holds the radio awake. The chip is a shared
     * resource with several owners on different tasks — DeviceDiscovery's
     * campaign tail and its sleep-mode entry, and the LoRaWAN session, all call
     * this — and a caller finishing ITS work is not evidence that the radio is
     * idle. The radio range test found this the hard way: entering it from a
     * FrKernel session released the AppTask's "waiting for release" hold, and
     * the campaign tail that followed deep-slept the chip out from under a
     * test that had just started. Every CAD after that timed out at 300 ms
     * with no IRQ, which reads on the terminal as an endlessly busy channel
     * rather than as a sleeping radio.
     *
     * Gated here rather than at each call site: this is the layer that owns
     * the chip's power state, it is one place instead of four, and a call site
     * added later cannot silently reintroduce the same failure. */
    if (bKeepAwake)
        return;

    LORARADIO_vWaitBeaconWindow();
    LORARADIO_DRIVER_vEnterDeepSleep();
    bAsleep = true;
}

/* Wait (bounded) for a beacon window to finish, then mark the chip as taken so
 * no new window can start on it. The claim and the test are one critical
 * section: a window request checks bAsleep under the same lock, so once this
 * returns no window can be starting behind the caller's back. */
static void LORARADIO_vClaimFromBeaconWindow(void)
{
    const uint32_t u32Start = osKernelGetTickCount();
    for (;;)
    {
        taskENTER_CRITICAL();
        if (!bWindowBusy)
        {
            bAsleep = false;
            taskEXIT_CRITICAL();
            return;
        }
        taskEXIT_CRITICAL();

        if ((osKernelGetTickCount() - u32Start) >= LORA_BEACON_WAIT_MAX_MS)
        {
            DBG_LOG("Loraradio: beacon window overran %u ms - taking the radio\r\n",
                    (unsigned)LORA_BEACON_WAIT_MAX_MS);
            bAsleep = false;
            return;
        }
        osDelay(2);
    }
}

static void LORARADIO_vWaitBeaconWindow(void)
{
    const uint32_t u32Start = osKernelGetTickCount();
    while (bWindowBusy &&
           ((osKernelGetTickCount() - u32Start) < LORA_BEACON_WAIT_MAX_MS))
        osDelay(2);
}

void LORARADIO_vWakeUp(void)
{
    LORARADIO_vClaimFromBeaconWindow();
    LORARADIO_DRIVER_vWakeUp();
}

void LORARADIO_vSetKeepAwake(bool bEnable)
{
    bKeepAwake = bEnable;
}

bool LORARADIO_bIsAsleep(void)
{
    return bAsleep;
}

/* --------------------------------------------------------------------------
 * Continuous wave - the work is done on the radio task, see the main loop
 * -------------------------------------------------------------------------- */
bool LORARADIO_bStartCw(void)
{
    if (bAsleep || (LORARADIO_vRadioTask_handle == NULL))
        return false;
    osThreadFlagsSet(LORARADIO_vRadioTask_handle, RADIO_EVT_CW_START);
    return true;
}

void LORARADIO_vStopCw(void)
{
    if (LORARADIO_vRadioTask_handle == NULL)
        return;
    osThreadFlagsSet(LORARADIO_vRadioTask_handle, RADIO_EVT_CW_STOP);

    /* The radio task normally answers within one pass; bounded so a caller on
     * a shake or a timeout can never hang here. */
    const uint32_t u32Start = osKernelGetTickCount();
    while (bCw && ((osKernelGetTickCount() - u32Start) < 200U))
        osDelay(5);
}

bool LORARADIO_bCwActive(void)
{
    return bCw;
}

/* --------------------------------------------------------------------------
 * Beacon + listen window
 * -------------------------------------------------------------------------- */
bool LORARADIO_bStartBeaconWindow(const uint8_t *pu8Frame, uint8_t u8Len,
                                  uint32_t u32ListenMs)
{
    if ((pu8Frame == NULL) || (u8Len == 0U) || (u8Len > LORA_BEACON_FRAME_MAX) ||
        (LORARADIO_vRadioTask_handle == NULL))
        return false;

    taskENTER_CRITICAL();
    if (!bAsleep || bKeepAwake || bWindowBusy)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    bWindowBusy = true;
    taskEXIT_CRITICAL();

    memcpy(au8WindowFrame, pu8Frame, u8Len);
    u8WindowFrameLen  = u8Len;
    u32WindowListenMs = u32ListenMs;
    osThreadFlagsSet(LORARADIO_vRadioTask_handle, RADIO_EVT_BEACON_WINDOW);
    return true;
}

/* Radio task. The chip is asleep (cold) and nobody else will touch it until
 * bWindowBusy clears: re-init, beacon, listen, then sleep again unless the
 * keep-awake hook claimed what arrived. Every event that is not the window's
 * own is stashed for the main loop, exactly as carrier sense does. */
static void LORARADIO_vRunBeaconWindow(void)
{
    bool bKeep = false;

    LORARADIO_DRIVER_vInit();

    au8WindowFrame[u8WindowFrameLen] =
        LORARADIO_u8CRC8_Calculate(au8WindowFrame, u8WindowFrameLen);

    bTxInFlight = true;
    const bool bSent = LORARADIO_DRIVER_bTransmitPayloadEx(au8WindowFrame,
                                                           (uint8_t)(u8WindowFrameLen + 1U),
                                                           LORA_PREAMBLE_LENGTH, false);
    bTxInFlight = false;

    if (bSent)
    {
        u32LastTxDoneTick = osKernelGetTickCount();
        LORARADIO_DRIVER_vEnterRxWindow(u32WindowListenMs);

        const uint32_t u32Start = osKernelGetTickCount();
        const uint32_t u32Limit = u32WindowListenMs + LORA_BEACON_WINDOW_TAIL_MS;
        for (;;)
        {
            const uint32_t u32Elapsed = osKernelGetTickCount() - u32Start;
            if (u32Elapsed >= u32Limit)
                break;

            uint32_t r = osThreadFlagsWait(ALL_FLAGS, osFlagsWaitAny, u32Limit - u32Elapsed);
            if (r & osFlagsError)
                break;

            const uint32_t u32Foreign = r & ~(RADIO_EVT_RX_DONE | RADIO_EVT_TIMEOUT |
                                              RADIO_EVT_CRC_ERROR | RADIO_EVT_HEADER_ERROR);
            if (u32Foreign != 0U)
                LORARADIO_vStashPendingEvents(u32Foreign);

            if (r & RADIO_EVT_RX_DONE)
            {
                LoraRadio_Packet_t tRx;
                memset(&tRx, 0, sizeof(tRx));
                LORARADIO_DRIVER_bReceivePayload(&tRx);
                tRx.i16NoiseFloor = LORA_NOISE_FLOOR_INVALID;
                if ((tRx.length >= 2U) &&
                    (LORARADIO_u8CRC8_Calculate(tRx.buffer, (uint16_t)(tRx.length - 1U)) ==
                     tRx.buffer[tRx.length - 1U]))
                {
                    tRx.length--;
                    bKeep = LORARADIO_bKeepAwakeAfterWindow(&tRx);
                    osMessageQueuePut(xLoRaRxQueue, &tRx, 0, 0);
                }
                break;
            }
            if (r & (RADIO_EVT_TIMEOUT | RADIO_EVT_CRC_ERROR | RADIO_EVT_HEADER_ERROR))
                break;
        }
    }

    SUBGRF_ClearIrqStatus(IRQ_RADIO_ALL);

    if (bKeep)
    {
        /* A session is about to own the radio: hand it over awake and in the
         * normal continuous receive the rest of the firmware expects. */
        LORARADIO_DRIVER_vEnterRxMode(0);
        bAsleep = false;
    }
    else
    {
        LORARADIO_DRIVER_vEnterDeepSleep();
    }

    bWindowBusy = false;
}

/* Weak defaults: a build with no beacon listener and no beacon-window user. */
__attribute__((weak)) bool LORARADIO_bKeepAwakeAfterWindow(const LoraRadio_Packet_t *pRx)
{
    (void)pRx;
    return false;
}

__attribute__((weak)) bool LORARADIO_bFastReply(const LoraRadio_Packet_t *pRx,
                                                uint8_t *pu8Reply, uint8_t *pu8ReplyLen)
{
    (void)pRx;
    (void)pu8Reply;
    (void)pu8ReplyLen;
    return false;
}

/* --------------------------------------------------------------------------
 * LORARADIO_vNotifyFromISR — set thread flags from ISR context
 * osThreadFlagsSet handles IS_IRQ() internally and calls portYIELD_FROM_ISR.
 * -------------------------------------------------------------------------- */
static void LORARADIO_vNotifyFromISR(uint32_t evt)
{
    if (LORARADIO_vRadioTask_handle != NULL)
        osThreadFlagsSet(LORARADIO_vRadioTask_handle, evt);
}
