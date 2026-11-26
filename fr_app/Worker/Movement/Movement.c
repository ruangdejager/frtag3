/*
 * Movement.c
 *
 * Movement worker — sensor sampling, HPF, MOVING/STILL state, high-activity
 * trigger counter and gravity sequence detector. Runs as an RTOS task
 * subscribed to the 1 Hz platform heartbeat; each tick drains the
 * accelerometer FIFO (25 Hz, so ~25 samples) and runs every sample through
 * the algorithm in order, so the per-sample logic is unchanged by the 1 s
 * batching.
 *
 * High-activity triggers
 * ----------------------
 * The trigger strategy is a port of the fr9's move.c (fr9_application,
 * MOVE_vEvalMovementLevel + MOVE_vWindowATick), sample for sample, against the
 * same movementAlarm settings and the same compile-time defaults. The tag
 * holds the fr9's default movementAlarm values in its own Settings_EE and
 * nothing on the tag writes them, so today the algorithm runs on defaults; if
 * the fr9 ever distributes these settings, writing them into Settings_EE is
 * all it takes, because every read goes through MOVE_u8GetAlarmSettingNow().
 *
 * Per sample:
 *   - The envelope level (HPF, below) is compared with the day-time-zone's
 *     maxLevel. At or above it, and only while the movementAlarm.dayTimeActive
 *     bit for the current weekday + DTZ is set, a window counter counts up;
 *     below it the counter counts back down. The counter therefore measures
 *     sustained activity, not a single jolt.
 *   - When it reaches maxWindow seconds' worth of samples it is a trigger.
 *     The counter restarts, Window A (movementAlarm.windowSensitive seconds)
 *     is (re)started, and the trigger is numbered within that window.
 *   - The first and/or second trigger in a window are HELD BACK when the
 *     DTZ's holdFirst / holdSecond setting says so: on the fr9 those never
 *     raise an alarm, they are only remembered in case activity continues.
 *     Every trigger that is not held back is where the fr9 queues a movement
 *     alarm. The tag does not alarm; it counts those instead, in
 *     u16DailyTriggerCnt, and the beacon reports the count.
 *   - Window A expiring with no further trigger closes the episode, so the
 *     next activity starts again at trigger 1 and is held back again.
 *
 * Deliberate differences from move.c:
 *   - No alarm queue, holdback-event log, LED or buzzer feedback. The fr9 also
 *     refuses to close Window A while an alarm is being reported
 *     (MOVE_ALARM_bIsActive); the tag never has one in flight.
 *   - u8TriggerCnt saturates at 255 instead of wrapping. On the fr9 a wrap
 *     after ~64 min of continuous activity would make trigger 257 "trigger 1"
 *     again and silently hold it back; saturating keeps counting.
 *   - Triggers are evaluated only while the unit is in service (Settings_EE
 *     inService). The fr9 runs the algorithm only in nominal unit mode; the
 *     tag's equivalent of not-nominal is production sleep, where the unit is
 *     in a box or being shaken awake on purpose.
 *   - The fr9's no-activity alarm (minLevel / minWindow) is not ported. The
 *     tag's own MOVING/STILL state (MOVE_vUpdateState) already plays that role
 *     in the beacon.
 *
 * Kept as in move.c even though it looks wrong: the NIGHT holdSecond read
 * takes middayZoneLevels.holdSecond. The fr9 has done that for years, and it
 * now also repurposes nightZoneLevels.holdSecond as the discovery-mode flag
 * (see DiscoveryMode_e), so night really does hold back two triggers there.
 * Same algorithm means same answer.
 *
 * Daily counter
 * -------------
 * u16DailyTriggerCnt counts non-held-back triggers since local midnight and
 * saturates at 0xFFFF. It is cleared when the LOCAL calendar date changes
 * (TIME_vTick runs just before this task is woken, so the date is current).
 * A TimeSync that moves the RTC across a date — notably the first one after a
 * cold boot, when the RTC still holds an invalid epoch — clears it as well;
 * counts gathered against an unknown date cannot be attributed to a day
 * anyway. It is RAM only: a reset loses the day's count.
 *
 * Filtered-output stream
 * ----------------------
 * Kernel "g" prints the per-sample filter signals (DC levels, high-passed
 * magnitude, envelope, max-level window counter) as CSV on the debug UART,
 * for tuning maxLevel / maxWindow against real movement on the bench or on an
 * animal with a cable. It only observes: the algorithm runs identically with
 * the stream on or off. Format in Movement.h (MOVE_vStartStream).
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hal_bsp.h"
#include "Movement.h"
#include "Movement_Driver.h"

#include "platform.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "dbg_log.h"
#include "math_func.h"
#include "time_driver.h"
#include "settings.h"        /* movementAlarm.*, inService */
#include "Acc.h"
#include "DeviceDiscovery.h"

#include "build_config.h"

/* --------------------------------------------------------------------------
 * Internal types
 * -------------------------------------------------------------------------- */
typedef struct {
    int8_t  ai8DcBuf[MOVE_ALG_DC_BUF_SIZE];
    uint8_t u8DcBufIdx;
    int8_t  i8DcLevel;
} move_alg_axis_t;

typedef struct {
    move_alg_axis_t X;
    move_alg_axis_t Y;
    move_alg_axis_t Z;
    uint8_t au8EnvBuf[MOVE_ALG_ENV_BUF_SIZE];
    uint8_t u8EnvBufIdx;
    uint8_t u8EnvLevel;
} move_alg_t;

typedef struct {
    uint8_t  u8DevIdErrorSeqCnt;
    uint16_t u16SampleDeltaErrorCnt;
    uint8_t  u8NoSampleErrorCnt;
    acc_t    AccPrevSample;
    bool     bErrorFlag;
} acc_health_t;

/* Window A — one high-activity episode (move.c's move_window_a_t). The fr9
 * runs it on an RTC-second timer; here it is a kernel-tick start stamp, which
 * keeps counting across STOP2 (tickless idle compensates the tick count). */
typedef struct {
    uint32_t u32StartTick;  /* osKernelGetTickCount() at the last trigger   */
    uint16_t u16TmrHwm;     /* highest window-counter value since a trigger */
    uint8_t  u8HoldBackCnt; /* hold-backs allowed, locked in at the trigger */
    uint8_t  u8TriggerCnt;  /* triggers this episode, held-back included;
                             * 0 = no episode open                           */
} move_window_a_t;

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */
static move_alg_t  MoveAlg;
static acc_health_t AccHealth;

static MoveState_e eState        = MOVE_STATE_MOVING;
static uint32_t    u32StillTicks = 0U;

/* High-activity triggers */
static move_window_a_t   MoveWinA;
static uint16_t          u16MaxLevelWindowTmr = 0U; /* samples, see header */
static volatile uint16_t u16DailyTriggerCnt   = 0U; /* read by the MeshTx task */
static uint8_t           u8CountDate          = 0xFFU; /* local date the count belongs to */

/* Filtered-output stream (kernel 'g'). The kernel task starts and stops it;
 * this task does all the printing. Start writes the end tick and raises
 * bStreamRestart before bStreamOn, so this task never sees a stream that is
 * on without its end tick. */
typedef struct {
    uint8_t u8MaxLevel;
    uint8_t u8MaxWindowS;
    bool    bActive;
    bool    bInService;
} move_stream_ctx_t;

static volatile bool     bStreamOn        = false;
static volatile bool     bStreamRestart   = false;
static volatile uint32_t u32StreamEndTick = 0U;
static uint32_t          u32StreamSamples = 0U;
static move_stream_ctx_t StreamCtx;

/* Gravity sequence detector */
static uint8_t u8SeqStep         = 0U;
static uint8_t u8HoldCounter     = 0U;
static uint8_t u8TransitionTicks = 0U;

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */
static bool MOVE_bUpdateMovementLevel(uint8_t *pu8Level);
static void MOVE_vEvalMovementLevel(void);
static void MOVE_vEvalTrigger(uint8_t u8Level);
static void MOVE_vWindowATick(void);
static void MOVE_vDailyRollover(void);
static bool MOVE_bIsAlarmActiveNow(void);
static void MOVE_vUpdateState(void);
static bool MOVE_bCheckPosition(uint8_t step);
static void MOVE_vSequenceTick(void);
static void MOVE_vStreamTick(void);
static void MOVE_vStreamSample(void);
static void MOVE_vTask(void *arg);

/* --------------------------------------------------------------------------
 * MOVE_vInit
 * -------------------------------------------------------------------------- */
void MOVE_vInit(void)
{
    MoveAlg.X.u8DcBufIdx = 0;
    MoveAlg.Y.u8DcBufIdx = 0;
    MoveAlg.Z.u8DcBufIdx = 0;
    MoveAlg.u8EnvBufIdx  = 0;

    memset(&AccHealth, 0, sizeof(acc_health_t));

    eState            = MOVE_STATE_MOVING;
    u32StillTicks     = 0U;
    u8SeqStep         = 0U;
    u8HoldCounter     = 0U;
    u8TransitionTicks = 0U;

    memset(&MoveWinA, 0, sizeof(MoveWinA));
    u16MaxLevelWindowTmr = 0U;
    u16DailyTriggerCnt   = 0U;
    u8CountDate          = 0xFFU;   /* adopted on the first tick, see MOVE_vDailyRollover */

    static const osThreadAttr_t attr = {
        .name       = "Movement",
        .stack_size = configMINIMAL_STACK_SIZE * 8U,
        .priority   = osPriorityLow,
    };
    osThreadNew(MOVE_vTask, NULL, &attr);
}

/* --------------------------------------------------------------------------
 * MOVE_eGetState
 * -------------------------------------------------------------------------- */
MoveState_e MOVE_eGetState(void) { return eState; }

/* --------------------------------------------------------------------------
 * MOVE_u16GetDailyTriggerCount — non-held-back triggers since local midnight.
 * A single aligned halfword, so a reader on another task gets either the old
 * or the new value, never a torn one. 0 on a primary (Movement never runs).
 * -------------------------------------------------------------------------- */
uint16_t MOVE_u16GetDailyTriggerCount(void) { return u16DailyTriggerCnt; }

/* --------------------------------------------------------------------------
 * MOVE_vTask — 1 Hz heartbeat-driven task
 * -------------------------------------------------------------------------- */
static void MOVE_vTask(void *arg)
{
    (void)arg;
    PLATFORM_bSubscribeToHeartbeat(osThreadGetId(), HB_ALLOW_IN_RECOVERY);

    for (;;)
    {
        osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);

        /* Midnight first, so a trigger landing in this tick's samples is
         * counted against the new day rather than cleared with the old. */
        MOVE_vDailyRollover();

        /* Stream bookkeeping before this tick's samples, so its header and
         * any settings change print ahead of the samples they apply to. */
        MOVE_vStreamTick();

        MOVE_vEvalMovementLevel();
        MOVE_vWindowATick();

        MOVE_vUpdateState();
        MOVE_vSequenceTick();

        /* Sensor health check #1: device ID */
        if (AccHealth.u8DevIdErrorSeqCnt < MOVE_SENSOR_ID_ERR_CNT_ALERT)
        {
            if (MOVE_DRIVER_u8GetAccelDeviceId() != ACC_WHO_AM_I_VALUE)
            {
                AccHealth.u8DevIdErrorSeqCnt++;
                if (AccHealth.u8DevIdErrorSeqCnt == MOVE_SENSOR_ID_ERR_CNT_ALERT &&
                    !AccHealth.bErrorFlag)
                {
                    AccHealth.bErrorFlag = true;
                    DBG("+++ MOVE SENSOR ERROR (DEVICE_ID) +++");
                }
            }
            else
            {
                AccHealth.u8DevIdErrorSeqCnt = 0;
            }
        }

        /* Sensor health check #2: no sample */
        if (AccHealth.u8NoSampleErrorCnt < MOVE_SENSOR_SAMPLE_ERR_CNT_ALERT)
        {
            AccHealth.u8NoSampleErrorCnt++;
            if (AccHealth.u8NoSampleErrorCnt == MOVE_SENSOR_SAMPLE_ERR_CNT_RESET)
            {
                ACC_vInit();
                DBG("+++ MOVE SENSOR ERROR (NO_SAMPLE) - reset +++");
            }
            if (AccHealth.u8NoSampleErrorCnt == MOVE_SENSOR_SAMPLE_ERR_CNT_ALERT &&
                !AccHealth.bErrorFlag)
            {
                AccHealth.bErrorFlag = true;
                DBG("+++ MOVE SENSOR ERROR (NO_SAMPLE) - alert +++");
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * MOVE_vDailyRollover — clear the trigger count when the local date changes
 * -------------------------------------------------------------------------- */
static void MOVE_vDailyRollover(void)
{
    uint8_t u8Date = TIME_pGetTimeDateLocalNow()->date;

    if (u8CountDate == 0xFFU)
    {
        /* First tick after boot: the count is already 0, just adopt the date. */
        u8CountDate = u8Date;
        return;
    }

    if (u8Date != u8CountDate)
    {
        DBG_LOG("Movement: new day, trigger count %u -> 0\r\n",
                (unsigned)u16DailyTriggerCnt);
        u16DailyTriggerCnt = 0U;
        u8CountDate        = u8Date;
    }
}

/* --------------------------------------------------------------------------
 * MOVE_u8GetAlarmSettingNow — movementAlarm setting for the current DTZ
 * (move.c's function of the same name, including its night holdSecond read;
 * see the file header).
 * -------------------------------------------------------------------------- */
uint8_t MOVE_u8GetAlarmSettingNow(MoveAlarmSetting_e eSetting)
{
    uint32_t u32 = 0U;

    switch (TIME_tGetDayTimeZoneNow())
    {
        case TIME_OF_DAY_MORNING:
            switch (eSetting)
            {
                case MOVE_ALARM_SETTING_MAX_LEVEL:   u32 = SETTINGS_u32GetWord(movementAlarm.morningZoneLevels.maxLevel);   break;
                case MOVE_ALARM_SETTING_MAX_WINDOW:  u32 = SETTINGS_u32GetWord(movementAlarm.morningZoneLevels.maxWindow);  break;
                case MOVE_ALARM_SETTING_FIRST_HOLD:  u32 = SETTINGS_u32GetWord(movementAlarm.morningZoneLevels.holdFirst);  break;
                case MOVE_ALARM_SETTING_SECOND_HOLD: u32 = SETTINGS_u32GetWord(movementAlarm.morningZoneLevels.holdSecond); break;
                default: break;
            }
            break;

        case TIME_OF_DAY_MIDDAY:
            switch (eSetting)
            {
                case MOVE_ALARM_SETTING_MAX_LEVEL:   u32 = SETTINGS_u32GetWord(movementAlarm.middayZoneLevels.maxLevel);   break;
                case MOVE_ALARM_SETTING_MAX_WINDOW:  u32 = SETTINGS_u32GetWord(movementAlarm.middayZoneLevels.maxWindow);  break;
                case MOVE_ALARM_SETTING_FIRST_HOLD:  u32 = SETTINGS_u32GetWord(movementAlarm.middayZoneLevels.holdFirst);  break;
                case MOVE_ALARM_SETTING_SECOND_HOLD: u32 = SETTINGS_u32GetWord(movementAlarm.middayZoneLevels.holdSecond); break;
                default: break;
            }
            break;

        case TIME_OF_DAY_AFTERNOON:
            switch (eSetting)
            {
                case MOVE_ALARM_SETTING_MAX_LEVEL:   u32 = SETTINGS_u32GetWord(movementAlarm.afternoonZoneLevels.maxLevel);   break;
                case MOVE_ALARM_SETTING_MAX_WINDOW:  u32 = SETTINGS_u32GetWord(movementAlarm.afternoonZoneLevels.maxWindow);  break;
                case MOVE_ALARM_SETTING_FIRST_HOLD:  u32 = SETTINGS_u32GetWord(movementAlarm.afternoonZoneLevels.holdFirst);  break;
                case MOVE_ALARM_SETTING_SECOND_HOLD: u32 = SETTINGS_u32GetWord(movementAlarm.afternoonZoneLevels.holdSecond); break;
                default: break;
            }
            break;

        case TIME_OF_DAY_NIGHT:
            switch (eSetting)
            {
                case MOVE_ALARM_SETTING_MAX_LEVEL:   u32 = SETTINGS_u32GetWord(movementAlarm.nightZoneLevels.maxLevel);   break;
                case MOVE_ALARM_SETTING_MAX_WINDOW:  u32 = SETTINGS_u32GetWord(movementAlarm.nightZoneLevels.maxWindow);  break;
                case MOVE_ALARM_SETTING_FIRST_HOLD:  u32 = SETTINGS_u32GetWord(movementAlarm.nightZoneLevels.holdFirst);  break;
                /* midday, not night: as move.c — see the file header. */
                case MOVE_ALARM_SETTING_SECOND_HOLD: u32 = SETTINGS_u32GetWord(movementAlarm.middayZoneLevels.holdSecond); break;
                default: break;
            }
            break;

        default:
            break;
    }

    return (uint8_t)u32;
}

/* --------------------------------------------------------------------------
 * MOVE_bIsAlarmActiveNow — movementAlarm.dayTimeActive bit for now.
 * Bit (4 * dayOfWeek + DTZ), Sunday = 0, MORNING = 0 .. NIGHT = 3; the default
 * 0x0CCCCCCC enables AFTERNOON and NIGHT on every day.
 * -------------------------------------------------------------------------- */
static bool MOVE_bIsAlarmActiveNow(void)
{
    uint32_t u32Bit = 4U * (uint32_t)TIME_u8GetDayOfWeek() +
                      (uint32_t)TIME_tGetDayTimeZoneNow();
    if (u32Bit > 31U) return false;
    return (SETTINGS_u32GetWord(movementAlarm.dayTimeActive) & (UINT32_C(1) << u32Bit)) != 0U;
}

/* --------------------------------------------------------------------------
 * MOVE_vEvalTrigger — one envelope sample through the move.c trigger logic
 * -------------------------------------------------------------------------- */
static void MOVE_vEvalTrigger(uint8_t u8Level)
{
    if (u8Level >= MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_LEVEL))
    {
        /* Above max level but outside the active day-time zones: the fr9
         * leaves the window counter exactly where it is. */
        if (!MOVE_bIsAlarmActiveNow()) return;

        if (u16MaxLevelWindowTmr < UINT16_MAX) u16MaxLevelWindowTmr++;
        MoveWinA.u16TmrHwm = max(MoveWinA.u16TmrHwm, u16MaxLevelWindowTmr);

        uint8_t u8MaxWindowS = MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_WINDOW);
        if (u16MaxLevelWindowTmr < (uint16_t)(MOVE_ACC_SAMPLE_RATE_HZ * u8MaxWindowS))
            return;

        /* Trigger. Restart the max-level window and (re)start Window A. */
        bool bHoldFirst  = MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_FIRST_HOLD)  != 0U;
        bool bHoldSecond = MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_SECOND_HOLD) != 0U;

        u16MaxLevelWindowTmr   = 0U;
        MoveWinA.u16TmrHwm     = 0U;
        if (MoveWinA.u8TriggerCnt < UINT8_MAX) MoveWinA.u8TriggerCnt++;
        MoveWinA.u8HoldBackCnt = (uint8_t)bHoldFirst + (uint8_t)bHoldSecond;
        MoveWinA.u32StartTick  = osKernelGetTickCount();

        if (((MoveWinA.u8TriggerCnt == 1U) && bHoldFirst) ||
            ((MoveWinA.u8TriggerCnt == 2U) && bHoldSecond))
        {
            DBG_LOG("Movement: high act trigger | hold (%u/%u)\r\n",
                    MoveWinA.u8TriggerCnt, MoveWinA.u8HoldBackCnt);
        }
        else
        {
            if (u16DailyTriggerCnt < UINT16_MAX) u16DailyTriggerCnt++;
            DBG_LOG("Movement: high act trigger | count (trig=%u, today=%u)\r\n",
                    MoveWinA.u8TriggerCnt, (unsigned)u16DailyTriggerCnt);
        }
    }
    else
    {
        if (u16MaxLevelWindowTmr) u16MaxLevelWindowTmr--;
    }
}

/* --------------------------------------------------------------------------
 * MOVE_vWindowATick — close the episode once Window A runs out
 * -------------------------------------------------------------------------- */
static void MOVE_vWindowATick(void)
{
    if (MoveWinA.u8TriggerCnt == 0U) return;

    uint32_t u32WindowMs = SETTINGS_u32GetWord(movementAlarm.windowSensitive) * 1000U;
    if ((osKernelGetTickCount() - MoveWinA.u32StartTick) < u32WindowMs) return;

    if (MoveWinA.u8TriggerCnt <= MoveWinA.u8HoldBackCnt)
    {
        DBG_LOG("Movement: high act episode | cancel (hold=%u/%u, hwm=%u/%us)\r\n",
                MoveWinA.u8TriggerCnt, MoveWinA.u8HoldBackCnt,
                (unsigned)(MoveWinA.u16TmrHwm / MOVE_ACC_SAMPLE_RATE_HZ),
                MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_WINDOW));
    }
    else
    {
        DBG_LOG("Movement: high act episode | done (trig=%u, hold=%u)\r\n",
                MoveWinA.u8TriggerCnt, MoveWinA.u8HoldBackCnt);
    }
    memset(&MoveWinA, 0, sizeof(MoveWinA));
}

/* --------------------------------------------------------------------------
 * MOVE_vUpdateState — MOVING / STILL bookkeeping
 * -------------------------------------------------------------------------- */
static void MOVE_vUpdateState(void)
{
    if (MoveAlg.u8EnvLevel > MOVE_STILL_ENV_THRESHOLD)
    {
        if (eState != MOVE_STATE_MOVING)
            DBG_LOG("Movement: state -> MOVING (env=%u)\r\n", MoveAlg.u8EnvLevel);

        eState        = MOVE_STATE_MOVING;
        u32StillTicks = 0U;
    }
    else
    {
        if (u32StillTicks < MOVE_STILL_TIMEOUT_S)
            u32StillTicks++;

        if (u32StillTicks >= MOVE_STILL_TIMEOUT_S && eState != MOVE_STATE_STILL)
        {
            eState = MOVE_STATE_STILL;
            DBG_LOG("Movement: state -> STILL (env=%u)\r\n", MoveAlg.u8EnvLevel);
        }
    }
}

/* --------------------------------------------------------------------------
 * MOVE_bCheckPosition — is the device in the given sequence position?
 * -------------------------------------------------------------------------- */
static bool MOVE_bCheckPosition(uint8_t step)
{
    int8_t x = MoveAlg.X.i8DcLevel;
    int8_t y = MoveAlg.Y.i8DcLevel;
    int8_t z = MoveAlg.Z.i8DcLevel;

    /* 5-position sequence: flat-down -> up -> tilt X -> tilt Y -> up.
     * (Previously 6 positions with a redundant "return to up" required
     * between the X and Y tilts; removed so tilt X can transition straight
     * into tilt Y.) */
    switch (step)
    {
        case 0: return (z < -MOVE_SEQ_GRAV_TH) &&
                       (x > -MOVE_SEQ_GRAV_NULL && x < MOVE_SEQ_GRAV_NULL) &&
                       (y > -MOVE_SEQ_GRAV_NULL && y < MOVE_SEQ_GRAV_NULL);
        case 1: return (z > +MOVE_SEQ_GRAV_TH);
        case 2: return ((x > MOVE_SEQ_GRAV_TH || x < -MOVE_SEQ_GRAV_TH)) &&
                       (z > -MOVE_SEQ_GRAV_NULL && z < MOVE_SEQ_GRAV_NULL);
        case 3: return ((y > MOVE_SEQ_GRAV_TH || y < -MOVE_SEQ_GRAV_TH)) &&
                       (z > -MOVE_SEQ_GRAV_NULL && z < MOVE_SEQ_GRAV_NULL);
        case 4: return (z > +MOVE_SEQ_GRAV_TH);
        default: return false;
    }
}

/* --------------------------------------------------------------------------
 * MOVE_vSequenceTick — gravity orientation sequence detector
 * -------------------------------------------------------------------------- */
static void MOVE_vSequenceTick(void)
{
    if (MOVE_bCheckPosition(u8SeqStep))
    {
        u8HoldCounter++;
        u8TransitionTicks = 0U;

        if (u8HoldCounter >= MOVE_SEQ_HOLD_TICKS)
        {
            MOVE_DRIVER_vLedOn();
            osDelay(100);
            MOVE_DRIVER_vLedOff();

            u8SeqStep++;
            u8HoldCounter     = 0U;
            u8TransitionTicks = 0U;

            DBG_LOG("Movement: shake-sequence step %u/5 accepted\r\n", u8SeqStep);

            if (u8SeqStep >= 5U)
            {
                /* Sequence complete — flash 5 times then trigger kernel wakeup */
                DBG_LOG("Movement: shake-sequence COMPLETE -> kernel wakeup\r\n");
                DEVICE_DISCOVERY_vTriggerKernelWakeup();
                for (uint8_t i = 0U; i < 20U; i++)
                {
                    MOVE_DRIVER_vLedOn();
                    osDelay(25);
                    MOVE_DRIVER_vLedOff();
                    osDelay(75);
                }
                u8SeqStep = 0U;
            }
        }
    }
    else if (u8SeqStep != 0U)
    {
        /* Mid-sequence: not yet in the next required position. Allow a few
         * seconds to physically move into place before giving up. */
        u8HoldCounter = 0U;
        u8TransitionTicks++;

        if (u8TransitionTicks >= MOVE_SEQ_TRANSITION_TICKS)
        {
            DBG_LOG("Movement: shake-sequence reset (was step %u/5)\r\n", u8SeqStep);
            u8SeqStep         = 0U;
            u8TransitionTicks = 0U;
        }
    }
    else
    {
        u8HoldCounter     = 0U;
        u8TransitionTicks = 0U;
    }
}

/* --------------------------------------------------------------------------
 * MOVE_bUpdateMovementLevel — drain one sample from ACC FIFO, run HPF
 * -------------------------------------------------------------------------- */
static bool MOVE_bUpdateMovementLevel(uint8_t *pu8Level)
{
    uint8_t u8;
    uint8_t u8Idx;
    int16_t i16Sum;
    acc_t   AccMoveDev;

    if (ACC_u8NumSamplesInFifo() > 0)
    {
        AccHealth.u8NoSampleErrorCnt = 0;

        ACC_vGetAccSample(&AccMoveDev);


        AccMoveDev.i16.i16OutX /= MOVE_ACC_RAW_SCALE_VALUE;
        AccMoveDev.i16.i16OutY /= MOVE_ACC_RAW_SCALE_VALUE;
        AccMoveDev.i16.i16OutZ /= MOVE_ACC_RAW_SCALE_VALUE;

        AccMoveDev.i16.i16OutX = min(AccMoveDev.i16.i16OutX,  127);
        AccMoveDev.i16.i16OutX = max(AccMoveDev.i16.i16OutX, -128);
        AccMoveDev.i16.i16OutY = min(AccMoveDev.i16.i16OutY,  127);
        AccMoveDev.i16.i16OutY = max(AccMoveDev.i16.i16OutY, -128);
        AccMoveDev.i16.i16OutZ = min(AccMoveDev.i16.i16OutZ,  127);
        AccMoveDev.i16.i16OutZ = max(AccMoveDev.i16.i16OutZ, -128);

        u8 = SETTINGS_u8GetByte(movementAlarm.accelerometerAxes);
        MoveAlg.X.ai8DcBuf[MoveAlg.X.u8DcBufIdx++] = ((u8 & 0x01) ? (int8_t)AccMoveDev.i16.i16OutX : 0);
        MoveAlg.X.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;
        MoveAlg.Y.ai8DcBuf[MoveAlg.Y.u8DcBufIdx++] = ((u8 & 0x02) ? (int8_t)AccMoveDev.i16.i16OutY : 0);
        MoveAlg.Y.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;
        MoveAlg.Z.ai8DcBuf[MoveAlg.Z.u8DcBufIdx++] = ((u8 & 0x04) ? (int8_t)AccMoveDev.i16.i16OutZ : 0);
        MoveAlg.Z.u8DcBufIdx %= MOVE_ALG_DC_BUF_SIZE;

        i16Sum = 0;
        for (u8Idx = 0; u8Idx < MOVE_ALG_DC_BUF_SIZE; u8Idx++)
            i16Sum += MoveAlg.X.ai8DcBuf[u8Idx];
        MoveAlg.X.i8DcLevel = (i16Sum / MOVE_ALG_DC_BUF_SIZE);

        i16Sum = 0;
        for (u8Idx = 0; u8Idx < MOVE_ALG_DC_BUF_SIZE; u8Idx++)
            i16Sum += MoveAlg.Y.ai8DcBuf[u8Idx];
        MoveAlg.Y.i8DcLevel = (i16Sum / MOVE_ALG_DC_BUF_SIZE);

        i16Sum = 0;
        for (u8Idx = 0; u8Idx < MOVE_ALG_DC_BUF_SIZE; u8Idx++)
            i16Sum += MoveAlg.Z.ai8DcBuf[u8Idx];
        MoveAlg.Z.i8DcLevel = (i16Sum / MOVE_ALG_DC_BUF_SIZE);

        u8 = SETTINGS_u8GetByte(movementAlarm.accelerometerAxes);
        MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx]  = (uint8_t)abs((int8_t)((u8 & 0x01) ? (int8_t)AccMoveDev.i16.i16OutX : 0) - MoveAlg.X.i8DcLevel);
        MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx] += (uint8_t)abs((int8_t)((u8 & 0x02) ? (int8_t)AccMoveDev.i16.i16OutY : 0) - MoveAlg.Y.i8DcLevel);
        MoveAlg.au8EnvBuf[MoveAlg.u8EnvBufIdx] += (uint8_t)abs((int8_t)((u8 & 0x04) ? (int8_t)AccMoveDev.i16.i16OutZ : 0) - MoveAlg.Z.i8DcLevel);
        MoveAlg.u8EnvBufIdx = (MoveAlg.u8EnvBufIdx + 1) % MOVE_ALG_ENV_BUF_SIZE;

        i16Sum = 0;
        for (u8Idx = 0; u8Idx < MOVE_ALG_ENV_BUF_SIZE; u8Idx++)
            i16Sum += MoveAlg.au8EnvBuf[u8Idx];
        MoveAlg.u8EnvLevel = (uint8_t)(i16Sum / MOVE_ALG_ENV_BUF_SIZE);

        *pu8Level = MoveAlg.u8EnvLevel;

        /* Health check #3: no delta */
        for (u8 = 0; u8 < sizeof(acc_t); u8++)
        {
            if (AccHealth.AccPrevSample.au8Data[u8] != AccMoveDev.au8Data[u8])
            {
                memcpy(&AccHealth.AccPrevSample, &AccMoveDev, sizeof(acc_t));
                AccHealth.u16SampleDeltaErrorCnt = 0;
                break;
            }
        }
        if (u8 == sizeof(acc_t) &&
            AccHealth.u16SampleDeltaErrorCnt < MOVE_SENSOR_DELTA_ERR_CNT_ALERT)
        {
            AccHealth.u16SampleDeltaErrorCnt++;
            if (AccHealth.u16SampleDeltaErrorCnt == MOVE_SENSOR_DELTA_ERR_CNT_RESET)
            {
                ACC_vInit();
                DBG("+++ MOVE SENSOR ERROR (NO_DELTA) - reset +++");
            }
            else if (AccHealth.u16SampleDeltaErrorCnt == MOVE_SENSOR_DELTA_ERR_CNT_ALERT &&
                     !AccHealth.bErrorFlag)
            {
                AccHealth.bErrorFlag = true;
                DBG("+++ MOVE SENSOR ERROR (NO_DELTA) - alert +++");
            }
        }

        return true;
    }
    return false;
}

/* --------------------------------------------------------------------------
 * MOVE_vEvalMovementLevel — drain full ACC FIFO, every sample through the
 * trigger logic in order. Trigger evaluation is skipped (the HPF still runs,
 * so it is settled the moment the unit goes into service) while the unit is
 * not in service; see the file header.
 * -------------------------------------------------------------------------- */
static void MOVE_vEvalMovementLevel(void)
{
    uint8_t u8Level;
    bool    bInService = SETTINGS_bGetBool(inService);

    while (MOVE_bUpdateMovementLevel(&u8Level))
    {
        if (bInService)
            MOVE_vEvalTrigger(u8Level);
        if (bStreamOn)
            MOVE_vStreamSample();
    }
}

/* --------------------------------------------------------------------------
 * Filtered-output stream (kernel 'g'). Format and columns: Movement.h.
 *
 * Output goes through _DBG_ (debug UART only, never the flash log - 25 lines
 * a second would fill it in hours). The DbgLog consumer runs above this task
 * and is woken by every line, so each line leaves the 1 KB ring as soon as it
 * is written and a 25-line burst never backs it up; at 115200 baud the
 * stream uses under a tenth of the UART.
 * -------------------------------------------------------------------------- */
void MOVE_vStartStream(uint16_t u16Minutes)
{
    if (u16Minutes == 0U)                 u16Minutes = MOVE_STREAM_DEFAULT_MIN;
    if (u16Minutes > MOVE_STREAM_MAX_MIN) u16Minutes = MOVE_STREAM_MAX_MIN;

    u32StreamEndTick = osKernelGetTickCount() + (uint32_t)u16Minutes * 60U * 1000U;
    bStreamRestart   = true;
    bStreamOn        = true;
}

void MOVE_vStopStream(void)
{
    bStreamOn      = false;
    bStreamRestart = false;
}

bool MOVE_bStreamActive(void) { return bStreamOn; }

/* Once per heartbeat tick, before the samples: header on (re)start, the time
 * limit, and a context line whenever a setting the 'win' column depends on
 * changes. */
static void MOVE_vStreamTick(void)
{
    if (!bStreamOn)
        return;

    if (bStreamRestart)
    {
        bStreamRestart   = false;
        u32StreamSamples = 0U;
        memset(&StreamCtx, 0xFF, sizeof(StreamCtx));   /* forces the context line */
        _DBG_("# movement stream: %u Hz, axes=0x%02X, columns MV,n,dx,dy,dz,hp,env,win\r\n",
              (unsigned)MOVE_ACC_SAMPLE_RATE_HZ,
              (unsigned)SETTINGS_u8GetByte(movementAlarm.accelerometerAxes));
    }

    if ((int32_t)(osKernelGetTickCount() - u32StreamEndTick) >= 0)
    {
        bStreamOn = false;
        _DBG_("# movement stream end: time limit, %lu samples\r\n",
              (unsigned long)u32StreamSamples);
        return;
    }

    move_stream_ctx_t tNow;
    memset(&tNow, 0, sizeof(tNow));   /* padding too, so memcmp is meaningful */
    tNow.u8MaxLevel   = MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_LEVEL);
    tNow.u8MaxWindowS = MOVE_u8GetAlarmSettingNow(MOVE_ALARM_SETTING_MAX_WINDOW);
    tNow.bActive      = MOVE_bIsAlarmActiveNow();
    tNow.bInService   = SETTINGS_bGetBool(inService);

    if (memcmp(&tNow, &StreamCtx, sizeof(tNow)) != 0)
    {
        StreamCtx = tNow;
        /* win only moves while both flags are 1: not in service, triggers are
         * not evaluated at all; outside an active zone, an above-maxLevel
         * sample leaves it where it is. */
        _DBG_("# maxLevel=%u maxWindow=%us (%u samples) zoneActive=%u inService=%u\r\n",
              (unsigned)tNow.u8MaxLevel, (unsigned)tNow.u8MaxWindowS,
              (unsigned)(MOVE_ACC_SAMPLE_RATE_HZ * tNow.u8MaxWindowS),
              (unsigned)tNow.bActive, (unsigned)tNow.bInService);
    }
}

/* One line for the sample MOVE_bUpdateMovementLevel has just filtered (and
 * MOVE_vEvalTrigger has just judged). hp is the envelope-buffer slot that
 * sample wrote, one behind the advanced index. */
static void MOVE_vStreamSample(void)
{
    uint8_t u8Hp = MoveAlg.au8EnvBuf[(MoveAlg.u8EnvBufIdx + MOVE_ALG_ENV_BUF_SIZE - 1U) %
                                     MOVE_ALG_ENV_BUF_SIZE];

    _DBG_("MV,%lu,%d,%d,%d,%u,%u,%u\r\n",
          (unsigned long)u32StreamSamples,
          (int)MoveAlg.X.i8DcLevel, (int)MoveAlg.Y.i8DcLevel, (int)MoveAlg.Z.i8DcLevel,
          (unsigned)u8Hp, (unsigned)MoveAlg.u8EnvLevel,
          (unsigned)u16MaxLevelWindowTmr);
    u32StreamSamples++;
}

#ifdef ENABLE_MOVEMENT_SENSOR_DATA_API
int8_t MOVE_i8SensorDataX(void) { return (int8_t)AccHealth.AccPrevSample.i16.i16OutX; }
int8_t MOVE_i8SensorDataY(void) { return (int8_t)AccHealth.AccPrevSample.i16.i16OutY; }
int8_t MOVE_i8SensorDataZ(void) { return (int8_t)AccHealth.AccPrevSample.i16.i16OutZ; }
#endif
