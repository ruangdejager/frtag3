/*
 * RadioTestMode.c
 *
 * Runtime RSSI test: the tag beacons, an external meter measures. See
 * RadioTestMode.h for what the mode does, the frame it sends and how it is
 * entered and left.
 *
 * Design notes worth knowing before editing:
 *
 *  - Almost nothing is static. This firmware links with very little RAM to
 *    spare, so everything the mode needs while it runs lives in one small
 *    heap-allocated context, allocated on entry and freed on exit. What stays
 *    static is the gate flag, the context pointer, the last entry error and
 *    the frame's boot-long sequence counter.
 *
 *  - No task and no timer. The beacon runs on MESHNETWORK_vTxTask via
 *    RADIOTESTMODE_u32ServiceBeacon. That is not tidiness, it is the only
 *    thing that fits: a secondary reaches this code with roughly 2.6 KB of
 *    FreeRTOS heap left, split across free blocks, and heap_4 must satisfy a
 *    request from ONE of them. Three attempts at sizing a beacon task ended in
 *    "could not enter (out of heap (task): 2640 B free, wanted 2048)" - a sum
 *    that looks sufficient right up until you remember it is a sum.
 *
 *  - This file used to hold a second, tag-to-tag range test as well (kernel
 *    'R'): a beaconing secondary sent ASCII "RT <id> <seq>" frames and a
 *    listening primary measured them and logged each to the fr9 over
 *    AT+RTLOG, on a task of its own. All tests are now made with the meter,
 *    so that role was removed with its task, queue and fr9 upload.
 */

#include "RadioTestMode.h"

#include "LoraRadio.h"
#include "MeshNetwork.h"        /* MeshPktType_Reserved, TX worker hooks */
#include "DeviceDiscovery.h"    /* DEVICE_DISCOVERY_eGetDeviceRole */
#include "hal_system.h"         /* SYSTEM_vSleepLockAcquire / Release */
#include "hal_bsp.h"            /* BSP_LED_On / Off, LED_AUX */
#include "Battery.h"            /* frame: battery mV */
#include "Power.h"              /* frame: low-battery flag */
#include "Movement.h"           /* frame: acceleration */
#include "SelfTest.h"           /* frame: accelerometer present + healthy */
#include "GPS.h"                /* "B gps": held-on receiver, live fix */
#include "crc16.h"
#include "build_config.h"
#include "dbg_log.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* --------------------------------------------------------------------------
 * Tunables
 * -------------------------------------------------------------------------- */

/* ---- CW test (kernel C) ---- */
/* Carrier duration: RADIOTESTMODE_bEnterCw(u8Minutes), default and ceiling in
 * RadioTestMode.h (RADIOTESTMODE_CW_DEFAULT_MIN / _MAX_MIN). */
#define CW_TEST_MS_PER_MIN      (60U * 1000U)

/* The carrier starts on the TX worker no sooner than this after entry, and
 * only once the radio has nothing of ours left to send: FrKernel acks the
 * command and closes the session (a BYE) right after entering, and a carrier
 * started under those would cut them off - the operator would see a tag that
 * simply stopped answering. */
#define CW_TEST_START_DELAY_MS  500U
#define CW_TEST_QUIET_POLL_MS   100U
#define CW_TEST_QUIET_MAX_MS    5000U           /* start anyway after this  */

#define RSSI_TEST_BEACON_MS     10000U          /* one frame every 10 s     */
#define RSSI_TEST_TIMEOUT_MS    (60U * 60U * 1000U) /* then back to normal  */
#define RSSI_TEST_LED_MS        50U             /* LED_AUX flash per TX     */

/* How long to wait, when a frame comes due but the PREVIOUS one is still in
 * the LoRa TX queue or in carrier sense, before checking again. */
#define RSSI_TEST_RETRY_MS      500U

#define RSSI_TEST_FRAME_LEN     24U             /* without a GPS position   */
#define RSSI_TEST_GPS_BYTES     9U              /* lat, lon, satellites     */
#define RSSI_TEST_FORMAT_VER    1U
#define RSSI_TEST_ID_0          0x52U           /* 'R' */
#define RSSI_TEST_ID_1          0x53U           /* 'S' */
#define RSSI_TEST_NA            0x80U           /* "not available" for i8 fields */
#define RSSI_TEST_RECENT_FRAMES 10U             /* flags bit2 for this many */

#define RSSI_TEST_FLAG_ACC_VALID    0x01U
#define RSSI_TEST_FLAG_LOW_BATTERY  0x02U
#define RSSI_TEST_FLAG_RECENT_RESET 0x04U
#define RSSI_TEST_FLAG_GPS          0x08U   /* bytes 22..30 hold a live fix */

/* Antenna configuration ids (frame byte 11). This list is the lookup the
 * meter's labels come from: assign the next free number to a new antenna
 * variant here, never reuse or renumber one. 0 = unknown / not yet assigned.
 * The firmware cannot sense which antenna is fitted, so the id is per board
 * role - the two V3 RevA boards are the only thing it can tell apart.
 *
 *   0  unknown / unassigned
 */
#define RSSI_TEST_ANTENNA_ID_SECONDARY  0U      /* Tagskaap3RevA collar tag */
#define RSSI_TEST_ANTENNA_ID_PRIMARY    0U      /* Tagskaap3RevA_DB         */

/* Radio profile ids (frame byte 12): the radio settings the frame is sent
 * with, so the meter can confirm it listens with the same ones. Same rules
 * as the antenna list. The test beacons on the mesh's own settings
 * (LoraRadio_Config.h), so a change there needs a new id here.
 *
 *   1  mesh: 868.0 MHz, SF7, BW 250 kHz, CR 4/5, preamble 8, sync 0x1424
 */
#define RSSI_TEST_RADIO_PROFILE_ID      1U

/* --------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------- */

/* Everything the test needs while it runs, in one heap block. */
typedef struct {
    uint32_t u32TxCount;          /* frames claimed, for the exit log line  */
    uint32_t u32NextBeaconTick;   /* RSSI: tick the next frame is due;
                                   * CW: tick the carrier may start, then
                                   * the tick it ends                       */
    uint32_t u32EnterTick;        /* start of the 1 h limit / quiet wait    */
    bool     bGps;                /* GPS held on ("B gps")                  */
    bool     bCw;                 /* this is the CW test (kernel C)         */
    bool     bCwOn;               /* CW: carrier started                    */
    uint32_t u32CwMs;             /* CW: how long the carrier stays on      */
} RtmCtx_t;

/* The gates in MeshNetwork's parser and TX paths read bActive on every packet,
 * so it stays a plain static rather than something behind a pointer chase.
 * volatile because those reads happen on other tasks. */
static volatile bool  bActive = false;
static RtmCtx_t      *pCtx    = NULL;

/* Why the last RADIOTESTMODE_bEnter() failed. Always a string literal in
 * flash: it has to outlive the call, and a formatted static buffer would cost
 * RAM this build does not have. The numbers go to DBG_LOG on the unit. */
static const char *pcLastEnterError = "";

/* Frame sequence and frames-since-boot. Static, not in the context, because
 * the format says the sequence restarts only on a reset: a meter that sees the
 * same tag entered twice in one boot must not read the second run's restart
 * as 65000-odd lost frames. The frame count saturates at
 * RSSI_TEST_RECENT_FRAMES, which is all it is for. */
static uint16_t u16RssiSeq            = 0U;
static uint8_t  u8RssiFramesSinceBoot = 0U;

static uint8_t  RTM_u8BuildFrame(uint8_t *pu8Buf, uint16_t u16Seq, bool bGps);
static bool     RTM_bEnter(bool bCw, bool bGps, uint32_t u32CwMs);
static uint32_t RTM_u32ServiceCw(uint32_t u32Now);

/* --------------------------------------------------------------------------
 * Public state
 * -------------------------------------------------------------------------- */
bool RADIOTESTMODE_bActive(void) { return bActive; }

const char *RADIOTESTMODE_pcLastEnterError(void) { return pcLastEnterError; }

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_bEnter
 * -------------------------------------------------------------------------- */
bool RADIOTESTMODE_bEnter(bool bGps) { return RTM_bEnter(false, bGps, 0U); }
bool RADIOTESTMODE_bEnterCw(uint8_t u8Minutes)
{
    /* 0 means "the default"; anything over the ceiling is held to it, so a
     * mistyped "C 50" is a 5-minute carrier, not an hour of one. */
    if (u8Minutes == 0U)
        u8Minutes = RADIOTESTMODE_CW_DEFAULT_MIN;
    if (u8Minutes > RADIOTESTMODE_CW_MAX_MIN)
        u8Minutes = RADIOTESTMODE_CW_MAX_MIN;
    return RTM_bEnter(true, false, (uint32_t)u8Minutes * CW_TEST_MS_PER_MIN);
}

/* Both tests share the whole set-up: the sleep lock, the radio held awake, the
 * mesh stood down and every RADIOTESTMODE_bActive() gate. They differ only in
 * what the TX worker then does with the radio (RADIOTESTMODE_u32ServiceBeacon). */
static bool RTM_bEnter(bool bCw, bool bGps, uint32_t u32CwMs)
{
    pcLastEnterError = "";

    if (bActive || pCtx != NULL)
    {
        pcLastEnterError = "already active";
        return false;
    }

    /* The beacon has no task of its own - it rides MeshNetwork's TX worker.
     * Refuse up front if that worker is not there rather than enter a mode
     * that would then never transmit; from the far end, a silent no-beacon
     * "success" is indistinguishable from a link that does not reach. */
    if (!MESHNETWORK_bTxWorkerReady())
    {
        pcLastEnterError = "no mesh TX worker";
        DBG_LOG("RadioTest: enter failed - %s\r\n", pcLastEnterError);
        return false;
    }

    /* "B gps" needs a receiver to hold on: secondaries only, and GPS refuses
     * to power up outside POWER_CLASS_NORMAL. Refused here, with the reason in
     * the ack, rather than entering a test whose frames would then never carry
     * a position for a reason the operator cannot see. */
    if (bGps && (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY))
    {
        pcLastEnterError = "no GPS on a primary";
        return false;
    }
    if (bGps && ((POWER_tGetState() & POWER_CLASS_NORMAL) == 0U))
    {
        pcLastEnterError = "battery too low for GPS";
        return false;
    }

    RtmCtx_t *pNew = pvPortMalloc(sizeof(RtmCtx_t));
    if (pNew == NULL)
    {
        /* The free TOTAL is only indicative: heap_4 satisfies a request from
         * ONE block, so a failure can read as plenty free and still be real. */
        pcLastEnterError = "out of heap (ctx)";
        DBG_LOG("RadioTest: enter failed - ctx alloc of %u B, %u B free total\r\n",
                (unsigned)sizeof(RtmCtx_t), (unsigned)xPortGetFreeHeapSize());
        return false;
    }
    memset(pNew, 0, sizeof(RtmCtx_t));
    pNew->bGps              = bGps;
    pNew->bCw               = bCw;
    pNew->u32CwMs           = u32CwMs;
    pNew->u32EnterTick      = osKernelGetTickCount();
    /* RSSI: due immediately, so the first frame goes out on the TX worker's
     * very next pass rather than 10 s after the operator sees the ack.
     * CW: the earliest the carrier may start (CW_TEST_START_DELAY_MS). */
    pNew->u32NextBeaconTick = pNew->u32EnterTick + (bCw ? CW_TEST_START_DELAY_MS : 0U);
    pCtx = pNew;

    /* Hold the system out of STOP2 for the duration: STOP2 parks the debug
     * UART and suspends the core, and the beacon must keep its cadence.
     * Released in RADIOTESTMODE_vExit, and nowhere else. */
    SYSTEM_vSleepLockAcquire();

    /* ...and hold the RADIO out of deep sleep, which is a separate thing from
     * the MCU sleep lock above. Taken BEFORE the wake-up below, because the
     * thing it defends against is already in flight by the time we get here:
     * entering from a FrKernel session ends that session, and the AppTask's
     * session tail then calls LORARADIO_vEnterDeepSleep(). Without this the
     * test wakes the radio, sends one or two frames, and is then slept out
     * from under - after which every carrier sense times out with no IRQ. */
    LORARADIO_vSetKeepAwake(true);

    /* Make sure the radio is actually awake: the command may have arrived in a
     * window that would otherwise have ended in a deep sleep. */
    LORARADIO_vWakeUp();

    /* Publish the state last: every gate keys off bActive, so nothing starts
     * dropping mesh traffic until the mode is fully built. */
    bActive = true;

    /* Stand the mesh down. The parser and TX gates (which now see bActive)
     * stop anything new; these clear what was already in flight. false: the
     * test owns the air, so a held-back TimeSync relay could not go anyway. */
    MESHNETWORK_vResetNodeRole();
    MESHNETWORK_vStopPrimaryAck();
    MESHNETWORK_vFlushTxQueue(false);
    LORARADIO_vFlushTxQueue();

    /* The TX worker is parked on osWaitForever with nothing left to wake it.
     * Poke it so it goes round once more and picks up the beacon deadline as
     * its timeout. After bActive, or it would go straight back to sleep. */
    MESHNETWORK_vWakeTxWorker();

    /* Last, once nothing can fail any more: hold the GPS on for the whole test
     * (no auto-shutdown, no TTFF timeout) so every frame can carry the latest
     * live position. bForce: the operator asked for GPS explicitly, so the
     * fr9's herd-wide GPS-disable does not apply. GPS owns its own sleep lock;
     * RADIOTESTMODE_vExit powers it down. */
    if (bGps)
        GPS_vRequestFix(false, 0U, true);

    /* Free heap logged on a successful entry too, so the next failure is a
     * comparison rather than a first data point. */
    if (bCw)
        DBG_LOG("RadioTest: CW test ENTERED (%u s carrier), %u B heap free\r\n",
                (unsigned)(u32CwMs / 1000U), (unsigned)xPortGetFreeHeapSize());
    else
        DBG_LOG("RadioTest: RSSI test ENTERED%s (10 s, 1 h limit), %u B heap free\r\n",
                bGps ? " with GPS" : "", (unsigned)xPortGetFreeHeapSize());
    return true;
}

/* --------------------------------------------------------------------------
 * Frame - layout in RadioTestMode.h
 * -------------------------------------------------------------------------- */
static void RTM_vPut16Le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

static void RTM_vPut32Le(uint8_t *p, uint32_t v)
{
    RTM_vPut16Le(&p[0], (uint16_t)(v & 0xFFFFU));
    RTM_vPut16Le(&p[2], (uint16_t)(v >> 16));
}

#ifdef ENABLE_MOVEMENT_SENSOR_DATA_API
/* Movement keeps its last sample divided by MOVE_ACC_RAW_SCALE_VALUE (192)
 * from the LIS2DH's left-justified ±2 g output, where 1 g = 16384, and
 * clamped to int8. 1/64 g is 256 raw, so the frame value is v * 192 / 256 =
 * v * 3 / 4, rounded to nearest. Movement's clamp limits it to about ±1.5 g
 * (-96..95 here), which is more than an orientation reading needs and can
 * never produce the 0x80 "not available" code. */
static uint8_t RTM_u8AccTo64thG(int8_t i8Move)
{
    int16_t i16 = (int16_t)i8Move * 3;
    i16 = (int16_t)((i16 + ((i16 >= 0) ? 2 : -2)) / 4);
    return (uint8_t)(int8_t)i16;
}
#endif

static uint8_t RTM_u8BuildFrame(uint8_t *pu8Buf, uint16_t u16Seq, bool bGps)
{
    const bool bSecondary =
        (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY);
    uint8_t u8Flags = 0U;
    uint8_t u8Len   = 22U;   /* bytes 0..21; the position (if any) and CRC follow */

    pu8Buf[0] = (uint8_t)MeshPktType_Reserved;
    pu8Buf[1] = RSSI_TEST_ID_0;
    pu8Buf[2] = RSSI_TEST_ID_1;
    pu8Buf[3] = RSSI_TEST_FORMAT_VER;
    RTM_vPut32Le(&pu8Buf[4], LORARADIO_u32GetUniqueId());
    RTM_vPut16Le(&pu8Buf[8], u16Seq);
    pu8Buf[10] = (uint8_t)(int8_t)TX_OUTPUT_POWER;
    pu8Buf[11] = bSecondary ? RSSI_TEST_ANTENNA_ID_SECONDARY
                            : RSSI_TEST_ANTENNA_ID_PRIMARY;
    pu8Buf[12] = RSSI_TEST_RADIO_PROFILE_ID;
    RTM_vPut16Le(&pu8Buf[13], (uint16_t)(RSSI_TEST_BEACON_MS / 10U));

    /* Battery and power class exist on a secondary only (no battery
     * measurement is fitted on the primary daughter board): 0 mV there means
     * "not measured", and the low-battery flag stays clear. */
    RTM_vPut16Le(&pu8Buf[15],
                 (bSecondary && BAT_bHasValidSample()) ? BAT_u16GetVoltage() : 0U);
    if (bSecondary && ((POWER_tGetState() & POWER_CLASS_NORMAL) == 0U))
        u8Flags |= RSSI_TEST_FLAG_LOW_BATTERY;

    /* No temperature source is read anywhere in this firmware yet. */
    pu8Buf[17] = RSSI_TEST_NA;

    pu8Buf[18] = RSSI_TEST_NA;
    pu8Buf[19] = RSSI_TEST_NA;
    pu8Buf[20] = RSSI_TEST_NA;
#ifdef ENABLE_MOVEMENT_SENSOR_DATA_API
    /* SELFTEST_bAccOk() is false on a primary (no accelerometer fitted) and on
     * a secondary whose LIS2DH failed its boot check. */
    if (bSecondary && SELFTEST_bAccOk())
    {
        pu8Buf[18] = RTM_u8AccTo64thG(MOVE_i8SensorDataX());
        pu8Buf[19] = RTM_u8AccTo64thG(MOVE_i8SensorDataY());
        pu8Buf[20] = RTM_u8AccTo64thG(MOVE_i8SensorDataZ());
        u8Flags |= RSSI_TEST_FLAG_ACC_VALID;
    }
#endif

    if (u8RssiFramesSinceBoot < RSSI_TEST_RECENT_FRAMES)
    {
        u8RssiFramesSinceBoot++;
        u8Flags |= RSSI_TEST_FLAG_RECENT_RESET;
    }

    /* GPS setup: only a LIVE fix goes out (GPS_bGetLiveFix - this epoch's
     * solution, never a cached one); until the first fix, and whenever it is
     * lost, the frame is the plain 24-byte one with flag bit 3 clear. */
    int32_t i32Lat;
    int32_t i32Lon;
    uint8_t u8Sats;
    if (bGps && GPS_bGetLiveFix(&i32Lat, &i32Lon, &u8Sats))
    {
        RTM_vPut32Le(&pu8Buf[22], (uint32_t)i32Lat);
        RTM_vPut32Le(&pu8Buf[26], (uint32_t)i32Lon);
        pu8Buf[30] = u8Sats;
        u8Len      = (uint8_t)(u8Len + RSSI_TEST_GPS_BYTES);
        u8Flags   |= RSSI_TEST_FLAG_GPS;
    }
    pu8Buf[21] = u8Flags;

    /* CRC over everything before it, always the last two bytes. */
    RTM_vPut16Le(&pu8Buf[u8Len], CRC16_u16Ccitt(pu8Buf, u8Len));
    return (uint8_t)(u8Len + 2U);
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_u32ServiceBeacon - runs on MESHNETWORK_vTxTask
 * -------------------------------------------------------------------------- */
uint32_t RADIOTESTMODE_u32ServiceBeacon(void)
{
    LoraRadio_Packet_t pkt;
    uint32_t           u32Now;
    uint16_t           u16Seq;
    bool               bGps;
    bool               bTimedOut;

    /* Cheap out first: this is on every trip round the mesh TX worker's loop
     * in normal operation, and must cost one volatile read. */
    if (!bActive)
        return osWaitForever;

    u32Now = osKernelGetTickCount();

    /* Read the context under a critical section and work from locals only. We
     * are on a task this file does not own, and RADIOTESTMODE_vExit can free
     * pCtx from Movement (shake) or FrKernel at any moment; nothing below the
     * critical sections touches pCtx, so the worst a teardown landing
     * mid-frame can do is put one last frame on the air. */
    taskENTER_CRITICAL();
    if (!bActive || pCtx == NULL)
    {
        taskEXIT_CRITICAL();
        return osWaitForever;
    }
    /* Signed difference, so this survives the tick counter wrapping. */
    if ((int32_t)(pCtx->u32NextBeaconTick - u32Now) > 0)
    {
        uint32_t u32Left = pCtx->u32NextBeaconTick - u32Now;
        taskEXIT_CRITICAL();
        return u32Left;             /* woken early - nothing due yet */
    }
    const bool bCwTest = pCtx->bCw;
    taskEXIT_CRITICAL();

    if (bCwTest)
        return RTM_u32ServiceCw(u32Now);

    /* One of our frames outstanding at a time, never two. The radio task
     * DEQUEUES a packet before it carrier-senses it, so the queue alone reads
     * empty for the whole multi-second window a packet can still be waiting
     * for a clear channel; both checks are needed. Checked before the
     * sequence number is claimed, so a delayed frame keeps its number. */
    if (LORARADIO_u16GetTxQueueDepth() > 0U || LORARADIO_bTxInProgress())
        return RSSI_TEST_RETRY_MS;

    taskENTER_CRITICAL();
    if (!bActive || pCtx == NULL)
    {
        taskEXIT_CRITICAL();
        return osWaitForever;
    }
    bGps      = pCtx->bGps;
    bTimedOut = ((u32Now - pCtx->u32EnterTick) >= RSSI_TEST_TIMEOUT_MS);
    pCtx->u32NextBeaconTick = u32Now + RSSI_TEST_BEACON_MS;
    /* Claimed even though the send below may drop it: to the meter a dropped
     * frame is identical to one that never made it across, and the sequence
     * gap should say so. */
    u16Seq = u16RssiSeq;
    if (!bTimedOut)
    {
        u16RssiSeq++;
        pCtx->u32TxCount++;
    }
    taskEXIT_CRITICAL();

    if (bTimedOut)
    {
        /* The hour is up: back to normal use. Checked after the "nothing of
         * ours still in flight" test above, so the radio is idle when it is
         * put back to sleep. Exiting from this task is safe: there is no task
         * of ours to wait for, so RADIOTESTMODE_vExit only clears the gates,
         * powers the GPS down, frees the context and drops the sleep lock.
         * The radio is deep-slept because nothing else will do it until the
         * next scheduled wake, which wakes it again itself; a shake exit
         * instead goes through the kernel window. */
        RADIOTESTMODE_vExit("1 h timeout");
        LORARADIO_vEnterDeepSleep();
        return osWaitForever;
    }

    memset(&pkt, 0, sizeof(pkt));
    pkt.length = RTM_u8BuildFrame(pkt.buffer, u16Seq, bGps);

    /* Flash on the enqueue: the radio task sends it straight away unless the
     * channel is busy, so this is when the meter should see it. The 50 ms
     * blocks only this borrowed TX worker, which has nothing else to do while
     * a test runs. */
    BSP_LED_On(LED_AUX);
    const bool bQueued = LORARADIO_bTxPacket(&pkt);
    osDelay(RSSI_TEST_LED_MS);
    BSP_LED_Off(LED_AUX);

    if (bQueued)
        DBG_LOG("RadioTest: RSSI TX seq=%u len=%u%s\r\n", (unsigned)u16Seq,
                (unsigned)pkt.length,
                (pkt.buffer[21] & RSSI_TEST_FLAG_GPS) ? " gps" : "");
    else
        DBG_LOG("RadioTest: RSSI TX drop seq=%u (queue full)\r\n", (unsigned)u16Seq);

    return RSSI_TEST_BEACON_MS;
}

/* --------------------------------------------------------------------------
 * CW test - TX worker. Called only once u32NextBeaconTick is due: first to
 * start the carrier, then once more when its minute is up.
 * -------------------------------------------------------------------------- */
static uint32_t RTM_u32ServiceCw(uint32_t u32Now)
{
    bool     bOn;
    uint32_t u32Enter;
    uint32_t u32CwMs;
    taskENTER_CRITICAL();
    if (!bActive || pCtx == NULL)
    {
        taskEXIT_CRITICAL();
        return osWaitForever;
    }
    bOn      = pCtx->bCwOn;
    u32Enter = pCtx->u32EnterTick;
    u32CwMs  = pCtx->u32CwMs;
    taskEXIT_CRITICAL();

    if (bOn)
    {
        /* The minute is up. Same exit as the RSSI test's hour, for the same
         * reasons: from this task, then the radio to sleep, which nothing else
         * would do before the next scheduled wake. */
        RADIOTESTMODE_vExit("1 min done");
        LORARADIO_vEnterDeepSleep();
        return osWaitForever;
    }

    /* Not started yet: wait for the ack and the session's BYE to leave the
     * radio (bounded - a stuck queue must not hold the test off for ever). */
    const bool bQuiet = (LORARADIO_u16GetTxQueueDepth() == 0U) && !LORARADIO_bTxInProgress();
    if (!bQuiet && ((u32Now - u32Enter) < CW_TEST_QUIET_MAX_MS))
        return CW_TEST_QUIET_POLL_MS;

    /* LED_AUX lit for the whole carrier: the one sign on the board that it
     * is on, since the tag answers nothing while it is. Lit before the
     * request, so an exit racing this always finds it on and turns it off. */
    BSP_LED_On(LED_AUX);
    if (!LORARADIO_bStartCw())
    {
        /* Only refused with the chip asleep, which the keep-awake taken on
         * entry rules out; leave rather than sit in a test with no carrier. */
        RADIOTESTMODE_vExit("carrier refused");
        return osWaitForever;
    }

    taskENTER_CRITICAL();
    if (!bActive || pCtx == NULL)
    {
        taskEXIT_CRITICAL();
        LORARADIO_vStopCw();         /* exited under us: take it back off */
        BSP_LED_Off(LED_AUX);
        return osWaitForever;
    }
    pCtx->bCwOn             = true;
    pCtx->u32NextBeaconTick = u32Now + u32CwMs;
    taskEXIT_CRITICAL();

    DBG_LOG("RadioTest: CW carrier ON %lu Hz %d dBm for %u s\r\n",
            (unsigned long)RF_FREQUENCY, (int)TX_OUTPUT_POWER,
            (unsigned)(u32CwMs / 1000U));
    return u32CwMs;
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_vExit
 * -------------------------------------------------------------------------- */
void RADIOTESTMODE_vExit(const char *pcReason)
{
    /* Clear the gate first so the mesh is live again the moment we stop. It
     * also stops the next RADIOTESTMODE_u32ServiceBeacon dead and puts the TX
     * worker back on osWaitForever. Test-and-clear in one step: the shake
     * path, a kernel 'Q' and the TX worker's 1 h limit can all land at once,
     * and only one of them may release the sleep lock. */
    taskENTER_CRITICAL();
    if (!bActive)
    {
        taskEXIT_CRITICAL();
        return;
    }
    bActive = false;
    taskEXIT_CRITICAL();

    /* Unpublish before freeing, in one atomic step: ServiceBeacon reads pCtx
     * under the same critical section, so it either gets a context that is
     * still whole or sees NULL. */
    RtmCtx_t *pDoomed;
    taskENTER_CRITICAL();
    pDoomed = pCtx;
    pCtx    = NULL;
    taskEXIT_CRITICAL();

    uint32_t u32Tx    = 0U;
    bool     bGpsHeld = false;
    bool     bWasCw   = false;
    if (pDoomed != NULL)
    {
        u32Tx    = pDoomed->u32TxCount;
        bGpsHeld = pDoomed->bGps;
        bWasCw   = pDoomed->bCw;
        vPortFree(pDoomed);
    }

    /* CW test: carrier off, BEFORE the radio is released. Asked whenever this
     * was a CW test, not only when the carrier already reads as on: a start
     * the TX worker has requested but the radio task has not run yet would
     * otherwise come on after we left. The radio task handles a stop ahead of
     * a start in the same pass, so this always ends with the carrier off.
     * vStopCw waits until it really is. */
    if (bWasCw)
    {
        LORARADIO_vStopCw();
        BSP_LED_Off(LED_AUX);
    }

    /* Hand the radio's power state back to its normal owners. */
    LORARADIO_vSetKeepAwake(false);

    /* "B gps" held the receiver on with no timeout; nothing else will turn it
     * off. (A shake exit then asks for its own fix, which simply starts a new
     * auto-shutdown session.) */
    if (bGpsHeld)
        GPS_vShutdown();

    SYSTEM_vSleepLockRelease();

    /* The tag is deaf for the duration, so the frame count is the only result
     * it can report, and this line is the only place it is reported. */
    if (bWasCw)
        DBG_LOG("RadioTest: CW test EXITED (%s)\r\n", (pcReason != NULL) ? pcReason : "?");
    else
        DBG_LOG("RadioTest: RSSI test EXITED (%s) - tx=%lu\r\n",
                (pcReason != NULL) ? pcReason : "?", (unsigned long)u32Tx);
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_bLogBeacon - console rig's passive decode, no state
 * -------------------------------------------------------------------------- */
bool RADIOTESTMODE_bLogBeacon(const uint8_t *pBuf, uint8_t u8Len,
                              int16_t i16Rssi, int8_t i8Snr, int16_t i16NoiseFloor)
{
    /* pBuf starts after the type byte, so the frame's byte N is pBuf[N - 1]
     * here, and the CRC has to fold the 0x00 type byte back in. The CRC is
     * always the last two bytes, whatever the length: a GPS frame, or a later
     * format version that added fields. */
    if ((pBuf == NULL) || (u8Len < (uint8_t)(RSSI_TEST_FRAME_LEN - 1U)) ||
        (pBuf[0] != RSSI_TEST_ID_0) || (pBuf[1] != RSSI_TEST_ID_1))
        return false;

    const uint8_t u8Type = (uint8_t)MeshPktType_Reserved;
    uint16_t u16Crc = CRC16_u16CcittUpdate(CRC16_CCITT_INIT, &u8Type, 1U);
    u16Crc = CRC16_u16CcittUpdate(u16Crc, pBuf, (uint16_t)(u8Len - 2U));
    const uint16_t u16Got = (uint16_t)(pBuf[u8Len - 2U] | ((uint16_t)pBuf[u8Len - 1U] << 8));
    const bool bGps = ((pBuf[20] & RSSI_TEST_FLAG_GPS) != 0U) &&
                      (u8Len >= (uint8_t)(RSSI_TEST_FRAME_LEN - 1U + RSSI_TEST_GPS_BYTES));

    /* Noise floor is sampled by the console build only (init.c); margin =
     * rssi - noise floor is what says how far the link is from failing. */
    const bool bNf = (i16NoiseFloor != LORA_NOISE_FLOOR_INVALID);
    DBG_LOG("RadioTest: RSSI RX id=%08lX seq=%u v%u rssi=%d dBm snr=%d dB "
            "nf=%d margin=%d bat=%u acc=%d,%d,%d flags=%02X%s\r\n",
            (unsigned long)((uint32_t)pBuf[3] | ((uint32_t)pBuf[4] << 8) |
                            ((uint32_t)pBuf[5] << 16) | ((uint32_t)pBuf[6] << 24)),
            (unsigned)(pBuf[7] | ((uint16_t)pBuf[8] << 8)), (unsigned)pBuf[2],
            i16Rssi, i8Snr,
            bNf ? (int)i16NoiseFloor : 0, bNf ? (int)(i16Rssi - i16NoiseFloor) : 0,
            (unsigned)(pBuf[14] | ((uint16_t)pBuf[15] << 8)),
            (int)(int8_t)pBuf[17], (int)(int8_t)pBuf[18], (int)(int8_t)pBuf[19],
            (unsigned)pBuf[20], (u16Crc == u16Got) ? "" : " CRC-BAD");
    if (bGps)
        DBG_LOG("RadioTest: RSSI RX   lat=%ld lon=%ld (udeg) sats=%u\r\n",
                (long)(int32_t)((uint32_t)pBuf[21] | ((uint32_t)pBuf[22] << 8) |
                                ((uint32_t)pBuf[23] << 16) | ((uint32_t)pBuf[24] << 24)),
                (long)(int32_t)((uint32_t)pBuf[25] | ((uint32_t)pBuf[26] << 8) |
                                ((uint32_t)pBuf[27] << 16) | ((uint32_t)pBuf[28] << 24)),
                (unsigned)pBuf[29]);
    return true;
}
