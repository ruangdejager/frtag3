/*
 * RadioTestMode.c
 *
 * Runtime radio link/range test. See RadioTestMode.h for what the mode does
 * and how it is entered and left.
 *
 * Design notes worth knowing before editing:
 *
 *  - Almost nothing is static. This firmware links with well under 100 bytes
 *    of RAM to spare (see the "reclaim RAM so FRKERNEL_INTERFACE_UART links"
 *    fix), so every counter, buffer and handle this mode needs lives in one
 *    heap-allocated context allocated on entry and freed on exit. What remains
 *    static is three flags and two pointers — the minimum needed for the gates
 *    in MeshNetwork and DeviceDiscovery to be a cheap read on the hot path.
 *
 *  - The beacon role has no task and no timer. It allocates its context and
 *    nothing else, and runs on MESHNETWORK_vTxTask via
 *    RADIOTESTMODE_u32ServiceBeacon. That is not tidiness, it is the only
 *    thing that fits: a secondary reaches this code with roughly 2.6 KB of
 *    FreeRTOS heap left, split across free blocks, and heap_4 must satisfy a
 *    request from ONE of them. Three attempts at sizing a beacon task ended
 *    in "could not enter (out of heap (task): 2640 B free, wanted 2048)" —
 *    a sum that looks sufficient right up until you remember it is a sum.
 *    Only the listener, which must block for seconds inside a Farmranger AT
 *    exchange and therefore cannot borrow anyone's task, still creates one,
 *    and only a primary — which has GPS, Movement and SolarPower absent from
 *    its heap and ~6.5 KB more of it free — ever takes that path.
 *
 *  - The listener does NOT drain the LoRa RX queue. That queue belongs to
 *    MESHNETWORK_vParserTask. Beacons reach us through
 *    RADIOTESTMODE_vOnBeacon, called from the parser.
 *
 *  - The parser hands us beacons on its own task. We only parse and enqueue
 *    there; the logging and the Farmranger round-trip happen on our task,
 *    because an fr9 handshake can block for seconds and stalling the parser
 *    would back up the LoRa RX queue and drop packets.
 *
 *  - Every beacon goes to the fr9 immediately, nothing is buffered, so the
 *    flash log tracks the walk live and a unit that loses power mid-test has
 *    already banked what it heard. One beacon, one fr9 session - see
 *    RTM_vLogToFr9.
 */

#include "RadioTestMode.h"

#include "LoraRadio.h"
#include "MeshNetwork.h"        /* MeshPktType_Reserved, DeviceRole_e */
#include "DeviceDiscovery.h"    /* DEVICE_DISCOVERY_eGetDeviceRole */
#include "Farmranger.h"
#include "hal_system.h"         /* SYSTEM_vSleepLockAcquire / Release */
#include "hal_bsp.h"            /* BSP_LED_On / Off, LED_AUX */
#include "Battery.h"            /* RSSI frame: battery mV */
#include "Power.h"              /* RSSI frame: low-battery flag */
#include "Movement.h"           /* RSSI frame: acceleration */
#include "SelfTest.h"           /* RSSI frame: accelerometer present + healthy */
#include "crc16.h"
#include "build_config.h"
#include "dbg_log.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Tunables
 * -------------------------------------------------------------------------- */

/* Listener only — the beacon role creates no task at all; see the note at the
 * top of the file and RADIOTESTMODE_u32ServiceBeacon below.
 *
 * This task has to fit on the
 * heap ALONGSIDE everything the mesh build already created — LoRa radio task,
 * MeshNetwork's two tasks, DeviceDiscovery's two tasks, FrKernel, Farmranger's
 * two, Battery and Power. Those already commit on the order of 30 KB of the
 * 46 KB FreeRTOS heap between their stacks and queues, so headroom is tight
 * even here, where it is least tight.
 *
 * It stays at *6 regardless. -fstack-usage measures the deep path —
 * RTM_vTask 32 + RTM_vHandleBeacon 72 + FARMRANGER_bLogRadioTestData 288
 * (its own row/cmd/response/line buffers) + FARMRANGER_bATSend 40 — at ~430 B,
 * and under all of it sits DBG_LOG's localtime/strftime/vsnprintf, which is
 * library code -fstack-usage cannot see. It is that unmeasured tail, not the
 * 430 B, that the margin is for. Same figure the compile-time RadioTest.c
 * task uses, for the same reason FrKernel does (see FrKernel.c's own *6
 * comment) — and unlike the beacon role, this allocation is not the one that
 * was failing: only a primary ever reaches it. */
#define RTM_STACK_SIZE_LISTEN   (configMINIMAL_STACK_SIZE * 6)

#define RTM_BEACON_MS       5000U    /* 0.2 Hz, per the test spec */

/* Deliberately shorter than RTM_BEACON_MS: how long to wait, when a beacon
 * comes due but a PREVIOUS one is still sitting in the LoRa TX queue, before
 * checking again. See the "one at a time" note on RADIOTESTMODE_u32ServiceBeacon
 * for why that check exists at all. */
#define RTM_BEACON_RETRY_MS 500U

/* Beacon frame: [0] = type, [1..] = ASCII "RT <id:hex> <seq:dec>".
 *
 * Same wire format as the compile-time test in RadioTest.c, deliberately: a
 * compile-time-built beacon and a runtime-built one are interchangeable on the
 * bench. MeshPktType_Reserved keeps it inert to production units, whose parser
 * switch drops reserved types in its default arm. */
#define RTM_TAG             "RT "
#define RTM_TAG_LEN         3U
#define RTM_LINE_MAX        48U

/* ---- RSSI test beacon (kernel B) ----
 * Frame layout is documented in RadioTestMode.h. */
#define RSSI_TEST_BEACON_MS     10000U          /* one frame every 10 s     */
#define RSSI_TEST_TIMEOUT_MS    (60U * 60U * 1000U) /* then back to normal  */
#define RSSI_TEST_LED_MS        50U             /* LED_AUX flash per TX     */
#define RSSI_TEST_FRAME_LEN     24U
#define RSSI_TEST_FORMAT_VER    1U
#define RSSI_TEST_ID_0          0x52U           /* 'R' */
#define RSSI_TEST_ID_1          0x53U           /* 'S' */
#define RSSI_TEST_NA            0x80U           /* "not available" for i8 fields */
#define RSSI_TEST_RECENT_FRAMES 10U             /* flags bit2 for this many */

#define RSSI_TEST_FLAG_ACC_VALID    0x01U
#define RSSI_TEST_FLAG_LOW_BATTERY  0x02U
#define RSSI_TEST_FLAG_RECENT_RESET 0x04U

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

/* How long RADIOTESTMODE_vExit waits for the task to wind itself down. Sized
 * for the worst case on the way out: the task may be mid-upload for a beacon
 * it just measured — an fr9 session open (~1 s to RDY) plus an AT+RTLOG whose
 * verdict wait is ~6.5 s. */
#define RTM_STOP_WAIT_MS    20000U

/* Thread flags. Listener only — there is no task on the beacon side to flag. */
#define RTM_FLAG_RX         (1UL << 0)   /* a beacon landed in the queue */
#define RTM_FLAG_STOP       (1UL << 1)   /* exit requested               */

/* --------------------------------------------------------------------------
 * State
 * -------------------------------------------------------------------------- */

/* One received beacon, as passed from the parser task to ours. */
typedef struct {
    uint32_t u32Id;
    uint32_t u32Seq;
    int16_t  i16Rssi;
    int16_t  i16NoiseFloor;
    int8_t   i8Snr;
} RtmBeacon_t;

/* Everything the mode needs while it is running, in one heap block — the only
 * allocation a beaconing secondary makes, at 44 bytes plus heap_4's 8-byte
 * block header.
 *
 * No timer object for the beacon cadence, and no task either: the beacon runs
 * on MESHNETWORK_vTxTask, which keeps its own deadline here and calls
 * RADIOTESTMODE_u32ServiceBeacon when it comes round. MeshNetwork's own beacon
 * timer exists to hand work off from the tiny FreeRTOS timer-service stack to
 * that same TX task; there is no reason to take the long way round to a task
 * we are already running on. */
typedef struct {
    osMessageQueueId_t xRxQueue;      /* listener only */

    uint32_t u32TxSeq;                /* beacon: next sequence number      */
    uint32_t u32NextBeaconTick;       /* beacon: tick the next one is due  */
    uint32_t u32EnterTick;            /* RSSI test: start of the 1 h limit */
    bool     bRssi;                   /* beacon is the RSSI test (kernel B) */

    uint32_t u32RxCount;              /* listener bookkeeping              */
    uint32_t u32Missed;
    uint32_t u32LastId;
    uint32_t u32LastSeq;
    bool     bHaveLast;

    uint32_t u32Fr9Logged;            /* beacons the fr9 acknowledged        */
    uint32_t u32Fr9Failed;            /* beacons it did not                  */
} RtmCtx_t;

/* The gates in MeshNetwork's parser and TX paths read bActive on every packet,
 * so it stays a plain static rather than something behind a pointer chase.
 * volatile because those reads happen on other tasks. */
static volatile bool  bActive      = false;
static volatile bool  bIsListener  = false;

/* Set before the task starts, cleared by the task as its very last act.
 * RADIOTESTMODE_vExit waits on this before freeing anything the task uses. */
static volatile bool  bTaskRunning = false;

static osThreadId_t   xRtmTask     = NULL;
static RtmCtx_t      *pCtx         = NULL;

/* Why the last RADIOTESTMODE_bEnter() call failed, if it did. A LoRa-addressed
 * command's ack is the ONLY thing the caller ever sees — the DBG_LOG lines
 * below land on the failing unit's own debug UART, which is exactly what a
 * remote operator cannot read. Surfaced via RADIOTESTMODE_pcLastEnterError()
 * so FrKernel can put the real reason in the ack instead of a bare
 * "could not enter" that gives a field failure nothing to go on. */
static const char *pcLastEnterError = "";

/* RSSI test frame sequence and frames-since-boot. Static, not in the context,
 * because the format says the sequence restarts only on a reset: a meter that
 * sees the same tag entered twice in one boot must not read the second run's
 * restart as 65000-odd lost frames. Three bytes of permanent RAM; the frame
 * count saturates at RSSI_TEST_RECENT_FRAMES, which is all it is for. */
static uint16_t u16RssiSeq        = 0U;
static uint8_t  u8RssiFramesSinceBoot = 0U;

/* Every reason above is a string literal in flash, deliberately. An earlier
 * version formatted heap byte counts into a 47-byte static buffer, which was
 * scaffolding for a heap failure the design has since removed — the beacon
 * role allocates nothing but its 44-byte context now. 47 bytes of permanently
 * resident RAM to describe a failure that no longer happens is a bad trade in
 * a build that links with well under 100 bytes to spare. The numbers still go
 * to DBG_LOG on the failing unit, which formats into the log ring and costs no
 * static at all; only the ack, which has to outlive this function, is fixed
 * text. */

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static void     RTM_vTask(void *arg);
static void     RTM_vHandleBeacon(const RtmBeacon_t *pB);
static void     RTM_vLogToFr9(const FarmrangerRtBeacon_t *pRow);
static void     RTM_vTeardown(void);
static bool     RTM_bEnter(bool bRssi);
static uint8_t  RTM_u8BuildRssiFrame(uint8_t *pu8Buf, uint16_t u16Seq);

/* --------------------------------------------------------------------------
 * Public state
 * -------------------------------------------------------------------------- */
bool RADIOTESTMODE_bActive(void)     { return bActive; }
bool RADIOTESTMODE_bIsListener(void) { return bIsListener; }

const char *RADIOTESTMODE_pcLastEnterError(void) { return pcLastEnterError; }

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_bEnter
 * -------------------------------------------------------------------------- */
bool RADIOTESTMODE_bEnter(void)     { return RTM_bEnter(false); }
bool RADIOTESTMODE_bEnterRssi(void) { return RTM_bEnter(true); }

/* Both entries share everything but the role choice and the beacon content:
 * the RSSI test is a beacon on either role, so it never takes the listener
 * path, and it carries its own frame, cadence and 1 h limit (see
 * RADIOTESTMODE_u32ServiceBeacon). */
static bool RTM_bEnter(bool bRssi)
{
    pcLastEnterError = "";

    if (bActive || pCtx != NULL)
    {
        pcLastEnterError = "already active";
        return false;
    }

    bIsListener = !bRssi && (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_PRIMARY);

    /* A beaconing secondary has no task of its own — it rides MeshNetwork's TX
     * worker. Refuse up front if that worker is not there rather than entering
     * a mode that would then never transmit (a console build skips
     * MESHNETWORK_vInit's campaign side). A silent no-beacon "success" would be
     * indistinguishable, from the far end of a range walk, from a link that
     * simply does not reach. */
    if (!bIsListener && !MESHNETWORK_bTxWorkerReady())
    {
        pcLastEnterError = "no mesh TX worker";
        DBG_LOG("RadioTest: enter failed - %s\r\n", pcLastEnterError);
        return false;
    }

    pCtx = pvPortMalloc(sizeof(RtmCtx_t));
    if (pCtx == NULL)
    {
        /* Fixed string for the ack (it has to outlive this function), numbers
         * to the log. Note the free TOTAL is only indicative: heap_4 satisfies
         * a request from ONE block, so a failure here can read as plenty free
         * and still be genuine. */
        pcLastEnterError = "out of heap (ctx)";
        DBG_LOG("RadioTest: enter failed - ctx alloc of %u B, %u B free total\r\n",
                (unsigned)sizeof(RtmCtx_t), (unsigned)xPortGetFreeHeapSize());
        return false;
    }
    memset(pCtx, 0, sizeof(RtmCtx_t));
    pCtx->bRssi        = bRssi;
    pCtx->u32EnterTick = osKernelGetTickCount();

    if (bIsListener)
    {
        pCtx->xRxQueue = osMessageQueueNew(4U, sizeof(RtmBeacon_t), NULL);
        if (pCtx->xRxQueue == NULL)
        {
            pcLastEnterError = "out of heap (queue)";
            DBG_LOG("RadioTest: enter failed - RX queue, %u B free total\r\n",
                    (unsigned)xPortGetFreeHeapSize());
            RTM_vTeardown();
            return false;
        }

        const osThreadAttr_t attr = {
            .name       = "RadioTestMode",
            .stack_size = RTM_STACK_SIZE_LISTEN * sizeof(StackType_t),
            .priority   = osPriorityNormal,
        };

        /* Set before the thread is created: the task clears it on the way out,
         * and a task that gets scheduled before osThreadNew returns must not
         * observe a stale false here. */
        bTaskRunning = true;

        xRtmTask = osThreadNew(RTM_vTask, NULL, &attr);
        if (xRtmTask == NULL)
        {
            /* Log the size wanted alongside the total free. They can disagree
             * wildly and both be right — heap_4 allocates from a single block,
             * and "2640 B free, wanted 2048" failing is what sent three rounds
             * of stack-size guessing down the wrong path. Only the listener
             * (a primary) reaches this at all; the beacon role stopped needing
             * a task of its own. */
            pcLastEnterError = "out of heap (task)";
            DBG_LOG("RadioTest: enter failed - task stack %u B, %u B free total\r\n",
                    (unsigned)attr.stack_size,
                    (unsigned)xPortGetFreeHeapSize());
            bTaskRunning = false;
            RTM_vTeardown();
            return false;
        }
    }
    else
    {
        /* Due immediately, so the first beacon goes out on the TX worker's
         * very next pass rather than 5 s after the operator sees the ack. */
        pCtx->u32NextBeaconTick = osKernelGetTickCount();
    }

    /* Hold the system out of STOP2 for the duration. STOP2 parks the debug
     * UART pins to analog and suspends the core, so the radio task could only
     * service RX once per 1 Hz RTC heartbeat - fine for a sleeping node,
     * useless for a beacon on a 5 s cadence or for a listener. Released in
     * RADIOTESTMODE_vExit, and nowhere else, so the pairing stays obvious. */
    SYSTEM_vSleepLockAcquire();

    /* ...and hold the RADIO out of deep sleep, which is a separate thing from
     * the MCU sleep lock above. Both are needed and neither substitutes for
     * the other: the sleep lock keeps the core running and the debug UART
     * alive, this keeps the transceiver able to answer a CAD.
     *
     * Taken BEFORE the wake-up below, because the thing it defends against is
     * already in flight by the time we get here: entering the test from a
     * FrKernel session clears s_bConnected, which releases DeviceDiscovery's
     * AppTask from its "waiting for release" hold, and the campaign tail that
     * follows calls LORARADIO_vEnterDeepSleep(). Without this the test wakes
     * the radio, sends one or two beacons, and is then slept out from under —
     * after which every carrier sense times out with no IRQ and the terminal
     * shows an endlessly busy channel instead of a sleeping radio. */
    LORARADIO_vSetKeepAwake(true);

    /* Make sure the radio is actually awake: the command may have arrived in a
     * kernel window that would otherwise have ended in a deep sleep. */
    LORARADIO_vWakeUp();

    /* Noise floor is only meaningful to the listener, and sampling costs the
     * radio task an extra SPI read per idle pass - leave it off elsewhere. */
    if (bIsListener)
        LORARADIO_vSetNoiseFloorSampling(true);

    /* Publish the state last: every gate this feature adds keys off bActive,
     * so nothing starts dropping mesh traffic until the mode is fully built
     * and can actually do something with what it keeps. */
    bActive = true;

    /* Stand the mesh down. The parser and TX gates (which now see bActive)
     * stop anything new; these calls clear what was already in flight so the
     * node does not spend the test chewing through a stale backlog. */
    MESHNETWORK_vResetNodeRole();
    MESHNETWORK_vStopPrimaryAck();
    /* false: the test owns the air from here, so there is nothing to keep -
     * a held-back TimeSync relay could not be transmitted anyway. */
    MESHNETWORK_vFlushTxQueue(false);
    LORARADIO_vFlushTxQueue();

    /* The TX worker we are about to beacon from is parked on osWaitForever
     * with nothing left to wake it - the gates above just made sure of that.
     * Poke it so it goes round once more and picks up the beacon deadline as
     * its timeout. After bActive, or it would go straight back to sleep. */
    if (!bIsListener)
        MESHNETWORK_vWakeTxWorker();

    /* Free heap on the way in, not just when it runs out: this is the number
     * that decides whether the mode fits, and having it logged for a
     * successful entry is what makes the next failure a comparison rather
     * than another first data point. */
    DBG_LOG("RadioTest: ENTERED as %s, %u B heap free\r\n",
            bRssi       ? "RSSI test beacon (10 s, 1 h limit)" :
            bIsListener ? "PRIMARY (listen)" : "SECONDARY (beacon)",
            (unsigned)xPortGetFreeHeapSize());
    return true;
}

/* --------------------------------------------------------------------------
 * RSSI test frame - layout in RadioTestMode.h
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

static uint8_t RTM_u8BuildRssiFrame(uint8_t *pu8Buf, uint16_t u16Seq)
{
    const bool bSecondary =
        (DEVICE_DISCOVERY_eGetDeviceRole() != DEVICE_ROLE_PRIMARY);
    uint8_t u8Flags = 0U;

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
    pu8Buf[21] = u8Flags;

    RTM_vPut16Le(&pu8Buf[22], CRC16_u16Ccitt(pu8Buf, RSSI_TEST_FRAME_LEN - 2U));
    return RSSI_TEST_FRAME_LEN;
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_u32ServiceBeacon - runs on MESHNETWORK_vTxTask
 * -------------------------------------------------------------------------- */
uint32_t RADIOTESTMODE_u32ServiceBeacon(void)
{
    LoraRadio_Packet_t pkt;
    uint32_t           u32Now;
    uint32_t           u32Seq;
    uint32_t           u32Id;
    uint32_t           u32Period;
    bool               bRssi;
    bool               bTimedOut;
    int                n;

    /* Cheap out first: this is on every trip round the mesh TX worker's loop
     * in normal operation, and must cost one volatile read. */
    if (!bActive || bIsListener)
        return osWaitForever;

    u32Now = osKernelGetTickCount();

    /* Read the context and claim the beacon in one atomic step, then work from
     * locals only. We are on a task this file does not own, and
     * RADIOTESTMODE_vExit can free pCtx from Movement (shake) or from FrKernel
     * at any moment. Nothing below the critical section touches pCtx, so the
     * worst a teardown landing mid-beacon can do is put one last frame on the
     * air - it can never dereference freed memory, whatever the priorities of
     * the two tasks happen to be. */
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

    taskEXIT_CRITICAL();

    /* One of our own beacons outstanding at a time, never two.
     *
     * RTM_BEACON_MS (5 s) and LORA_TX_CARRIER_WAIT_MS (5 s, the radio task's
     * own carrier-sense budget per packet) are numerically equal, which is not
     * a coincidence to route around so much as a fact to respect: under a busy
     * channel, a packet can still be being carrier-sensed when the NEXT
     * beacon becomes due by our own clock. Enqueuing anyway would let our
     * beacons pile up behind each other — worse, faster than they can
     * possibly be sent — for no gain.
     *
     * Both checks are needed: the radio task DEQUEUES a packet before it
     * carrier-senses it, so the queue alone reads empty for the entire
     * multi-second window a packet can be in flight — that window is exactly
     * what caused the pile-up, so it is the one that must not be missed.
     *
     * Checking here, before claiming the sequence number, means a beacon
     * delayed this way is not yet "sent" and is retried at its own number
     * rather than skipped — the count of genuinely un-transmittable beacons is
     * what RTM_BEACON_RETRY_MS is for. */
    if (LORARADIO_u16GetTxQueueDepth() > 0U || LORARADIO_bTxInProgress())
        return RTM_BEACON_RETRY_MS;

    taskENTER_CRITICAL();

    if (!bActive || pCtx == NULL)
    {
        taskEXIT_CRITICAL();
        return osWaitForever;
    }

    bRssi     = pCtx->bRssi;
    u32Period = bRssi ? RSSI_TEST_BEACON_MS : RTM_BEACON_MS;
    bTimedOut = bRssi &&
                ((u32Now - pCtx->u32EnterTick) >= RSSI_TEST_TIMEOUT_MS);

    /* Claimed even though the send below may drop it: to the listener a
     * dropped beacon is identical to one that never made it across, and the
     * sequence gap should say so. */
    u32Seq                  = pCtx->u32TxSeq++;
    pCtx->u32NextBeaconTick = u32Now + u32Period;
    if (bRssi && !bTimedOut)
        u32Seq = u16RssiSeq++;    /* the frame's own boot-long counter */

    taskEXIT_CRITICAL();

    if (bTimedOut)
    {
        /* The RSSI test's hour is up: back to normal use. Checked here, after
         * the "nothing of ours still in flight" test above, so the radio is
         * idle when it is put back to sleep. Exiting from this task is safe:
         * the beacon role has no task of its own to wait for, so
         * RADIOTESTMODE_vExit only clears the gates, frees the context and
         * drops the sleep lock. The radio is deep-slept because nothing else
         * will do it until the next scheduled wake, which wakes it again
         * itself; a shake exit instead goes through the kernel window. */
        RADIOTESTMODE_vExit("1 h timeout");
        LORARADIO_vEnterDeepSleep();
        return osWaitForever;
    }

    if (bRssi)
    {
        memset(&pkt, 0, sizeof(pkt));
        pkt.length = RTM_u8BuildRssiFrame(pkt.buffer, (uint16_t)u32Seq);

        /* Flash on the enqueue: the radio task sends it straight away unless
         * the channel is busy, so this is when the meter should see it. The
         * 50 ms blocks only this borrowed TX worker, which has nothing else to
         * do while a test runs. */
        BSP_LED_On(LED_AUX);
        bool bQueued = LORARADIO_bTxPacket(&pkt);
        osDelay(RSSI_TEST_LED_MS);
        BSP_LED_Off(LED_AUX);

        if (bQueued)
            DBG_LOG("RadioTest: RSSI TX seq=%lu\r\n", (unsigned long)u32Seq);
        else
            DBG_LOG("RadioTest: RSSI TX drop seq=%lu (queue full)\r\n",
                    (unsigned long)u32Seq);
        return u32Period;
    }

    u32Id = LORARADIO_u32GetUniqueId();

    memset(&pkt, 0, sizeof(pkt));
    pkt.buffer[0] = (uint8_t)MeshPktType_Reserved;

    n = snprintf((char *)&pkt.buffer[1],
                 (size_t)(LORA_MAX_PACKET_SIZE - 2),
                 RTM_TAG "%04lX %lu",
                 (unsigned long)u32Id, (unsigned long)u32Seq);

    if (n > 0)
    {
        pkt.length = (uint8_t)(n + 1);

        if (LORARADIO_bTxPacket(&pkt))
            DBG_LOG("RadioTest: TX seq=%lu\r\n", (unsigned long)u32Seq);
        else
            DBG_LOG("RadioTest: TX drop seq=%lu (queue full)\r\n",
                    (unsigned long)u32Seq);
    }

    return RTM_BEACON_MS;
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_vExit
 * -------------------------------------------------------------------------- */
void RADIOTESTMODE_vExit(const char *pcReason)
{
    uint32_t u32Rx     = 0U;
    uint32_t u32Missed = 0U;
    uint32_t u32Tx     = 0U;

    /* Clear the gates first so the mesh is live again the moment we stop, and
     * so nothing new is handed to a queue we are about to delete. It is also
     * what stops the next RADIOTESTMODE_u32ServiceBeacon on the mesh TX worker
     * dead, and puts that worker back on osWaitForever.
     *
     * Test-and-clear in one step: since the RSSI test's 1 h timeout, the mesh
     * TX worker can call this too, and a shake or a kernel 'Q' landing at the
     * same moment must not get past the check as well and release the sleep
     * lock a second time. */
    taskENTER_CRITICAL();
    if (!bActive)
    {
        taskEXIT_CRITICAL();
        return;
    }
    bActive = false;
    taskEXIT_CRITICAL();

    /* Hand the radio's power state back to its normal owners. Released here,
     * with bActive, rather than down beside SYSTEM_vSleepLockRelease() so the
     * two early returns below cannot leave the radio pinned awake for the rest
     * of the unit's uptime — that would cost real battery on a secondary and
     * would be invisible until someone noticed the drain. */
    LORARADIO_vSetKeepAwake(false);

    if (bIsListener)
        LORARADIO_vSetNoiseFloorSampling(false);

    /* Ask the task to wind itself down, then wait for it to say it has. We do
     * NOT terminate it out from under itself: it may be several seconds deep
     * in a Farmranger transaction, and killing it there would leave the fr9 AT
     * handler holding a thread handle it will later try to signal. The task
     * does its own final fr9 flush and clears bTaskRunning as its last act, so
     * by the time we free anything it is genuinely done with it.
     *
     * The bound is generous because that final flush is up to three AT+RTLOG
     * attempts. Listener only - a beaconing secondary has no task, so the
     * shake path (Movement task) skips all of this and returns at once. */
    if (xRtmTask != NULL)
    {
        uint32_t u32Start = osKernelGetTickCount();

        osThreadFlagsSet(xRtmTask, RTM_FLAG_STOP);

        while (bTaskRunning &&
               (osKernelGetTickCount() - u32Start) < RTM_STOP_WAIT_MS)
        {
            osDelay(20);
        }

        if (bTaskRunning)
        {
            /* Wedged somewhere we did not anticipate. Leaking the context is
             * strictly better than freeing memory a live task still writes to,
             * so say so loudly and leave it parked. */
            DBG_LOG("RadioTest: task did not stop in %lu ms - left parked\r\n",
                    (unsigned long)RTM_STOP_WAIT_MS);
            SYSTEM_vSleepLockRelease();
            return;
        }
    }

    if (pCtx != NULL)
    {
        u32Rx     = pCtx->u32RxCount;
        u32Missed = pCtx->u32Missed;
        u32Tx     = pCtx->u32TxSeq;
    }

    RTM_vTeardown();

    SYSTEM_vSleepLockRelease();

    /* Per role, because the other role's counters are all zero and printing
     * them reads as a test that measured nothing. On a secondary this line is
     * the whole result of the run - it is deaf for the duration, so the beacon
     * count is the only thing it can report, and a shake-exit is the only
     * place it gets reported. */
    if (bIsListener)
        DBG_LOG("RadioTest: EXITED (%s) - rx=%lu missed=%lu\r\n",
                (pcReason != NULL) ? pcReason : "?",
                (unsigned long)u32Rx, (unsigned long)u32Missed);
    else
        DBG_LOG("RadioTest: EXITED (%s) - tx=%lu\r\n",
                (pcReason != NULL) ? pcReason : "?",
                (unsigned long)u32Tx);
}

/* Free everything the mode allocated. The listener's task has already exited
 * by the time this runs from RADIOTESTMODE_vExit; from RADIOTESTMODE_bEnter's
 * failure path it either never started or is terminated here. Safe with any
 * subset built, which is what makes it usable as that failure path. */
static void RTM_vTeardown(void)
{
    RtmCtx_t *pDoomed;

    if (xRtmTask != NULL)
    {
        if (bTaskRunning)
        {
            osThreadTerminate(xRtmTask);
            bTaskRunning = false;
        }
        xRtmTask = NULL;
    }

    /* Unpublish before freeing, in one atomic step. RADIOTESTMODE_u32ServiceBeacon
     * runs on the mesh TX worker and reads pCtx under the same critical
     * section, so it either gets a context that is still whole or sees NULL -
     * there is no window in which it holds a pointer we are freeing. */
    taskENTER_CRITICAL();
    pDoomed = pCtx;
    pCtx    = NULL;
    taskEXIT_CRITICAL();

    if (pDoomed != NULL)
    {
        if (pDoomed->xRxQueue != NULL)
            osMessageQueueDelete(pDoomed->xRxQueue);

        vPortFree(pDoomed);
    }
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_vOnBeacon - parser-task context, keep it cheap
 *
 * "Cheap" is also what makes the pCtx reads below safe. Nothing in here
 * blocks or yields, so this function runs to completion against every task
 * that can call RADIOTESTMODE_vExit and free pCtx underneath it - FrKernel
 * and Movement are both osPriorityLow, the parser this runs on is Normal.
 * Anything added here that can block breaks that, and would need the same
 * critical section RADIOTESTMODE_u32ServiceBeacon uses.
 * -------------------------------------------------------------------------- */
/* Pull the id and sequence out of a beacon payload. Shared by the listener
 * and by the console's passive print below, so the wire format is decoded in
 * exactly one place — two copies of this would be two things to keep in step
 * with RADIOTESTMODE_u32ServiceBeacon's snprintf. Returns false for anything
 * that is not one of our beacons. */
static bool RTM_bParseBeacon(const uint8_t *pBuf, uint8_t u8Len,
                             uint32_t *pu32Id, uint32_t *pu32Seq)
{
    char     line[RTM_LINE_MAX];
    char    *pEnd = NULL;
    uint8_t  len  = u8Len;

    if (pBuf == NULL)
        return false;

    if (len >= (uint8_t)RTM_LINE_MAX)
        len = (uint8_t)(RTM_LINE_MAX - 1U);

    memcpy(line, pBuf, len);
    line[len] = '\0';

    if (len <= (uint8_t)RTM_TAG_LEN ||
        strncmp(line, RTM_TAG, RTM_TAG_LEN) != 0)
        return false;

    *pu32Id = strtoul(&line[RTM_TAG_LEN], &pEnd, 16);
    if (pEnd == &line[RTM_TAG_LEN])
        return false;

    *pu32Seq = strtoul(pEnd, NULL, 10);
    return true;
}

void RADIOTESTMODE_vOnBeacon(const uint8_t *pBuf, uint8_t u8Len,
                             int16_t i16Rssi, int8_t i8Snr, int16_t i16NoiseFloor)
{
    RtmBeacon_t tB;

    if (!bActive || !bIsListener || pCtx == NULL || pCtx->xRxQueue == NULL)
        return;

    if (!RTM_bParseBeacon(pBuf, u8Len, &tB.u32Id, &tB.u32Seq))
        return;

    tB.i16Rssi       = i16Rssi;
    tB.i8Snr         = i8Snr;
    tB.i16NoiseFloor = i16NoiseFloor;

    /* Non-blocking: if our task is mid-fr9-handshake and the queue is full,
     * drop this beacon rather than stall the parser. The gap shows up in the
     * missed count on the next one we do log, which is the honest outcome. */
    (void)osMessageQueuePut(pCtx->xRxQueue, &tB, 0U, 0U);
    osThreadFlagsSet(xRtmTask, RTM_FLAG_RX);
}

/* --------------------------------------------------------------------------
 * RADIOTESTMODE_bLogBeacon - passive observer, no mode, no state
 * -------------------------------------------------------------------------- */
bool RADIOTESTMODE_bLogBeacon(const uint8_t *pBuf, uint8_t u8Len,
                              int16_t i16Rssi, int8_t i8Snr, int16_t i16NoiseFloor)
{
    uint32_t u32Id;
    uint32_t u32Seq;

    /* RSSI test frame (kernel B). pBuf starts after the type byte, so the
     * frame's byte N is pBuf[N - 1] here, and the CRC has to fold the 0x00
     * type byte back in. Longer frames are later format versions and fine. */
    if (pBuf != NULL && u8Len >= (uint8_t)(RSSI_TEST_FRAME_LEN - 1U) &&
        pBuf[0] == RSSI_TEST_ID_0 && pBuf[1] == RSSI_TEST_ID_1)
    {
        const uint8_t u8Type = (uint8_t)MeshPktType_Reserved;
        uint16_t u16Crc = CRC16_u16CcittUpdate(CRC16_CCITT_INIT, &u8Type, 1U);
        u16Crc = CRC16_u16CcittUpdate(u16Crc, pBuf, RSSI_TEST_FRAME_LEN - 3U);
        const uint16_t u16Got = (uint16_t)(pBuf[21] | ((uint16_t)pBuf[22] << 8));

        DBG_LOG("RadioTest: RSSI RX id=%08lX seq=%u v%u rssi=%d dBm snr=%d dB "
                "bat=%u acc=%d,%d,%d flags=%02X%s\r\n",
                (unsigned long)((uint32_t)pBuf[3] | ((uint32_t)pBuf[4] << 8) |
                                ((uint32_t)pBuf[5] << 16) | ((uint32_t)pBuf[6] << 24)),
                (unsigned)(pBuf[7] | ((uint16_t)pBuf[8] << 8)), (unsigned)pBuf[2],
                i16Rssi, i8Snr,
                (unsigned)(pBuf[14] | ((uint16_t)pBuf[15] << 8)),
                (int)(int8_t)pBuf[17], (int)(int8_t)pBuf[18], (int)(int8_t)pBuf[19],
                (unsigned)pBuf[20], (u16Crc == u16Got) ? "" : " CRC-BAD");
        (void)i16NoiseFloor;
        return true;
    }

    if (!RTM_bParseBeacon(pBuf, u8Len, &u32Id, &u32Seq))
        return false;

    /* Deliberately stateless: no rx/missed tallies like the listener keeps.
     * Those would need statics, and this build links with well under 100 bytes
     * of RAM to spare. Nothing is lost - the sequence numbers are printed, so
     * a gap is visible directly in the lines themselves, which is what the
     * tally was summarising anyway. */
    if (i16NoiseFloor == LORA_NOISE_FLOOR_INVALID)
        DBG_LOG("RadioTest: RX id=%04lX seq=%lu rssi=%d dBm snr=%d dB nf=n/a\r\n",
                (unsigned long)u32Id, (unsigned long)u32Seq, i16Rssi, i8Snr);
    else
        DBG_LOG("RadioTest: RX id=%04lX seq=%lu rssi=%d dBm snr=%d dB "
                "nf=%d dBm margin=%d dB\r\n",
                (unsigned long)u32Id, (unsigned long)u32Seq, i16Rssi, i8Snr,
                i16NoiseFloor, (int)(i16Rssi - i16NoiseFloor));

    return true;
}

/* --------------------------------------------------------------------------
 * RTM_vTask - listener only
 * -------------------------------------------------------------------------- */
static void RTM_vTask(void *arg)
{
    (void)arg;

    for (;;)
    {
        RtmBeacon_t tB;

        /* The 1 s timeout is only a periodic liveness check - nothing is timed
         * off it. Real work arrives as RTM_FLAG_RX from the parser task. */
        uint32_t flags = osThreadFlagsWait(RTM_FLAG_RX | RTM_FLAG_STOP,
                                           osFlagsWaitAny, 1000U);
        if (flags & osFlagsError)
            flags = 0U;

        if (!bActive || (flags & RTM_FLAG_STOP) || pCtx == NULL)
            break;

        while (pCtx->xRxQueue != NULL &&
               osMessageQueueGet(pCtx->xRxQueue, &tB, NULL, 0U) == osOK)
        {
            RTM_vHandleBeacon(&tB);
        }
    }

    /* Winding down. Anything already measured is already on the fr9 - that is
     * the point of logging per beacon - but the queue may still hold beacons
     * the parser handed us in the last moments, and the last beacons of a
     * range walk are the ones from the far end. Log those before going.
     *
     * No session to close: each beacon opens and drops its own, so by here the
     * attention line is already low.
     *
     * Done here rather than in RADIOTESTMODE_vExit so it runs on this task's
     * stack and cannot race the loop above. */
    if (pCtx != NULL)
    {
        RtmBeacon_t tB;

        while (pCtx->xRxQueue != NULL &&
               osMessageQueueGet(pCtx->xRxQueue, &tB, NULL, 0U) == osOK)
        {
            RTM_vHandleBeacon(&tB);
        }

        DBG_LOG("RadioTest: fr9 logged=%lu failed=%lu\r\n",
                (unsigned long)pCtx->u32Fr9Logged,
                (unsigned long)pCtx->u32Fr9Failed);
    }

    /* Last act: tell RADIOTESTMODE_vExit it is safe to free the context.
     * Returning from a CMSIS-RTOS2 thread function exits the thread. */
    bTaskRunning = false;
}

/* --------------------------------------------------------------------------
 * Primary: account for and log one beacon
 * -------------------------------------------------------------------------- */
static void RTM_vHandleBeacon(const RtmBeacon_t *pB)
{
    bool bHaveNf;

    if (pCtx == NULL)
        return;

    bHaveNf = (pB->i16NoiseFloor != LORA_NOISE_FLOOR_INVALID);

    if (pCtx->bHaveLast && (pB->u32Id == pCtx->u32LastId))
    {
        if (pB->u32Seq > pCtx->u32LastSeq)
            pCtx->u32Missed += (pB->u32Seq - pCtx->u32LastSeq - 1U);
    }
    else if (pCtx->bHaveLast)
    {
        /* A different secondary took over the test - the old tally is not
         * comparable, so restart it rather than report nonsense. */
        pCtx->u32Missed = 0U;
    }

    pCtx->u32LastId  = pB->u32Id;
    pCtx->u32LastSeq = pB->u32Seq;
    pCtx->bHaveLast  = true;
    pCtx->u32RxCount++;

    if (bHaveNf)
    {
        DBG_LOG("RadioTest: RX id=%04lX seq=%lu rssi=%d dBm snr=%d dB "
                "nf=%d dBm margin=%d dB rx=%lu missed=%lu\r\n",
                (unsigned long)pB->u32Id, (unsigned long)pB->u32Seq,
                pB->i16Rssi, pB->i8Snr, pB->i16NoiseFloor,
                (int)(pB->i16Rssi - pB->i16NoiseFloor),
                (unsigned long)pCtx->u32RxCount, (unsigned long)pCtx->u32Missed);
    }
    else
    {
        DBG_LOG("RadioTest: RX id=%04lX seq=%lu rssi=%d dBm snr=%d dB "
                "nf=n/a rx=%lu missed=%lu\r\n",
                (unsigned long)pB->u32Id, (unsigned long)pB->u32Seq,
                pB->i16Rssi, pB->i8Snr,
                (unsigned long)pCtx->u32RxCount, (unsigned long)pCtx->u32Missed);
    }

    /* Straight to the fr9, now, while the beacon is in hand. This blocks for
     * the AT+RTLOG exchange - tens of milliseconds normally - which is why it
     * runs on this task and not on the parser that handed us the beacon.
     * Anything that arrives meanwhile waits in the RX queue. */
    {
        FarmrangerRtBeacon_t tRow;
        tRow.u32DeviceId   = pB->u32Id;
        tRow.u32Seq        = pB->u32Seq;
        tRow.u32Missed     = pCtx->u32Missed;
        tRow.i16Rssi       = pB->i16Rssi;
        tRow.i16NoiseFloor = pB->i16NoiseFloor;
        tRow.i8Snr         = pB->i8Snr;

        RTM_vLogToFr9(&tRow);
    }
}

/* --------------------------------------------------------------------------
 * Primary: push one beacon to the fr9, as it is measured
 *
 * One beacon, one session: assert the attention line, wait for RDY, send the
 * single AT+RTLOG, de-assert. Every beacon therefore starts from a known
 * state, which is the whole point - a session held across beacons has to stay
 * healthy for the length of a range walk, and when it does not (the fr9's own
 * 600 s cap, a reset, a cable knocked loose) the failure is silent and every
 * subsequent beacon is lost. That is what the held-session version did in
 * practice: it logged the first beacon and then nothing.
 *
 * The cost is the fr9's fixed ~1 s wait before it answers RDY, once per
 * beacon. Against a 5 s cadence that is affordable, and it buys a design with
 * no session lifetime to manage, no recycle timer, and nothing to go stale.
 * -------------------------------------------------------------------------- */
static void RTM_vLogToFr9(const FarmrangerRtBeacon_t *pRow)
{
    bool bOk;

    if (pCtx == NULL || pRow == NULL)
        return;

    if (!FARMRANGER_bDeviceOn())
    {
        DBG_LOG("RadioTest: fr9 did not answer RDY\r\n");
        pCtx->u32Fr9Failed++;
        FARMRANGER_vDeviceOff();   /* drop the line again; no session to keep */
        return;
    }

    bOk = FARMRANGER_bLogRadioTestData(pRow, 1U);

    FARMRANGER_vDeviceOff();

    /* Hold the attention line low long enough for the fr9 to see the falling
     * edge before the next beacon raises it again. It debounces GPIO1 for
     * 20 ms and starts a session on the RISING edge, so back-to-back sessions
     * that never showed a clean low would look like no edge at all. There are
     * ~5 s until the next beacon, so this costs nothing. FARMRANGER_bDeviceOn()
     * now enforces the same minimum low time itself (FR_WAKE_MIN_LOW_MS), so
     * this hold is belt-and-braces rather than load-bearing. */
    osDelay(50);

    if (bOk)
        pCtx->u32Fr9Logged++;
    else
        pCtx->u32Fr9Failed++;
}
