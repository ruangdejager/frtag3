/*
 * Battery.c
 *
 * Battery voltage measurement — CMSIS-RTOS v2 task layer.
 *
 * Two tasks:
 *   BatSampleTask       — triggered by thread flags; performs one ADC sample
 *                         and owns every write to the averaging buffer.
 *   BatSchedTask        — subscribes to 1-second heartbeat; triggers a
 *                         delta-limited sample every BAT_SAMPLE_INTERVAL ticks.
 *
 * Thread flags used on BatSampleTask:
 *   BAT_NOTIFY_DELTA_LIMIT    — take a delta-limited sample
 *   BAT_NOTIFY_PURGE_REQUEST  — take an immediate sample and refill the whole
 *                               averaging buffer from the next VALID sample
 *
 * Sample validity and seeding. A sample is valid only when both conversions
 * completed and the VREFINT-derived VDDA is plausible
 * (HAL_ADC_u16VddaFromVrefint() returns non-zero). An invalid sample never
 * enters the averaging buffer; it is tallied in u16BatSampleFails instead.
 * Until the first valid sample after boot the buffer is empty,
 * BAT_u16GetVoltage() returns 0 and BAT_bHasValidSample() returns false; the
 * first valid sample fills all BAT_AVG_FACTOR slots at once and opens that
 * gate for good.
 * This replaces a one-shot BatPurgeTask that asked for a sample and then
 * filled the buffer with u16BatPreviousSample whether or not the sample had
 * succeeded. A failed boot conversion therefore seeded the average with 0 mV,
 * which dropped a secondary to LOW and then RECOVERY within two heartbeats
 * and took ~70-80 s of good samples to climb back out. Removing that task also
 * frees its stack and TCB at the exact point in the boot sequence
 * (POWER_vInit) where heap exhaustion has been hit before (FreeRTOSConfig.h).
 *
 * No logging in BatSampleTask. It runs on a configMINIMAL_STACK_SIZE (512 B)
 * stack and a DBG_LOG call chain needs about 536 B (measured from the frame
 * sizes in Debug/frtag3.list), so a log line here is a stack overflow waiting
 * to happen. Failures are counted instead and read back with
 * BAT_u16GetAndClearSampleFails().
 */

#include "Battery.h"
#include "hal_bsp.h"
#include "hal_adc.h"
#include "hal_system.h"
#include "Battery_Driver.h"
#include "Battery_Config.h"

#include "platform.h"
#include "math_func.h"
#include "hard_timers.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"   /* configMINIMAL_STACK_SIZE, taskENTER/EXIT_CRITICAL */
#include "task.h"

#include <limits.h>

/* ---- Thread flags ---- */
#define BAT_NOTIFY_DELTA_LIMIT    (1UL << 0)
#define BAT_NOTIFY_PURGE_REQUEST  (1UL << 1)

/* ---- Task stack / priority ---- */
#define BAT_SAMPLETASK_STACK_SIZE    (configMINIMAL_STACK_SIZE)
#define BAT_SCHEDULETASK_STACK_SIZE  (configMINIMAL_STACK_SIZE)

/* ---- CMSIS-RTOS v2 handles ---- */
static osThreadId_t BAT_vSampleTask_handle;
static osThreadId_t BAT_vCheckSampleScheduleTask_handle;

/* ---- Averaging buffer ----
 * Written only by BatSampleTask. BAT_u16GetVoltage() reads it lock-free from
 * any task (DbgLog's line prefix calls it inside every DBG_LOG, including from
 * timer callbacks). A single-slot update is therefore kept to one aligned
 * 16-bit store, so a reader sees the old or the new slot value but never a
 * torn one, and the whole-buffer seed runs inside a critical section. */
static uint16_t au16BatAvgBuf[BAT_AVG_FACTOR];
static uint8_t  u8BatAvgIdx;
static uint16_t u16BatPreviousSample;

/* Monotonic validity gate: false from boot until the first valid sample has
 * seeded the whole averaging buffer, then never cleared. A later failed
 * sample only skips its slot, so the average keeps the last good values; only
 * a success opens the gate. It is set in the same critical section as the
 * seeding fill, so no reader can see it true over a half-filled buffer. */
static volatile bool     bBatSeeded;

/* Invalid samples (EOC timeout, or a VDDA outside the ADC supply range) since
 * the last BAT_u16GetAndClearSampleFails(). Tallied rather than logged
 * because BatSampleTask's stack cannot afford DBG_LOG; saturates at
 * UINT16_MAX. */
static volatile uint16_t u16BatSampleFails;

/* ---- Forward declarations ---- */
static void BAT_vSampleTask(void *pvParameters);
static void BAT_vCheckSampleScheduleTask(void *pvParameters);
static bool BAT_bConvert(hal_adc_channel_t channel, uint16_t *pu16Result);

/* --------------------------------------------------------------------------
 * BAT_bConvert — one blocking ADC conversion on the given channel.
 * The ADC must already be enabled. Returns true and writes *pu16Result on
 * success; false if the end-of-conversion flag never arrived.
 * -------------------------------------------------------------------------- */
static bool BAT_bConvert(hal_adc_channel_t channel, uint16_t *pu16Result)
{
    uint8_t u8DelayMs = 3;

    BAT_DRIVER_vCleanInterrupt();
    HAL_ADC_vStartConversion(channel);

    while (!BAT_DRIVER_bGetInterruptFlag() && u8DelayMs--)
        osDelay(1);

    if (!BAT_DRIVER_bGetInterruptFlag())
        return false;

    *pu16Result = BAT_DRIVER_u16GetResult();
    return true;
}

/* --------------------------------------------------------------------------
 * BAT_vInit
 * -------------------------------------------------------------------------- */
void BAT_vInit(void)
{
    static const osThreadAttr_t sample_attr = {
        .name       = "BatSampleTask",
        .stack_size = BAT_SAMPLETASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };
    static const osThreadAttr_t sched_attr = {
        .name       = "BatSchedTask",
        .stack_size = BAT_SCHEDULETASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };

    BAT_vSampleTask_handle =
        osThreadNew(BAT_vSampleTask, NULL, &sample_attr);
    BAT_vCheckSampleScheduleTask_handle =
        osThreadNew(BAT_vCheckSampleScheduleTask, NULL, &sched_attr);

    configASSERT(BAT_vSampleTask_handle              != NULL);
    configASSERT(BAT_vCheckSampleScheduleTask_handle != NULL);

    /* Request an immediate sample; the first valid one fills the averaging
     * buffer. Must come after BatSampleTask exists, since the request is a
     * thread flag on it. */
    BAT_vPurgeBuffer();
}

/* --------------------------------------------------------------------------
 * BAT_vSampleTask — triggered by thread flags, performs ADC sampling
 * -------------------------------------------------------------------------- */
static void BAT_vSampleTask(void *pvParameters)
{
    (void)pvParameters;

    /* True while the next valid sample must refill the whole averaging
     * buffer. It starts true, so the first valid sample after boot seeds the
     * buffer whichever flag woke the task, and BAT_NOTIFY_PURGE_REQUEST
     * re-arms it. An invalid sample leaves it armed, so a failed conversion
     * can never seed 0 mV; the next valid sample (normally the next scheduled
     * one, BAT_SAMPLE_INTERVAL heartbeats later) seeds instead. */
    bool bSeedPending = true;

    for (;;)
    {
        uint32_t ulNotifiedValue = osThreadFlagsWait(
            BAT_NOTIFY_DELTA_LIMIT | BAT_NOTIFY_PURGE_REQUEST,
            osFlagsWaitAny,
            osWaitForever);

        bool bDeltaLimit = (ulNotifiedValue & BAT_NOTIFY_DELTA_LIMIT) != 0U;
        if ((ulNotifiedValue & BAT_NOTIFY_PURGE_REQUEST) != 0U)
            bSeedPending = true;

        HAL_ADC_vLock();
        SYSTEM_vSleepLockAcquire();

        /* Enable bias and wait for the divider to settle */
        BAT_DRIVER_vEnableBiasCircuit();
        osDelay(BAT_MEAS_BIAS_SETTLE_TIME);

        /* The ADC should already be disabled here, because every ADC user
         * disables it before HAL_ADC_vUnlock(). Give a disable that has not
         * completed a bounded window, then force it (HAL_ADC_vDisable stops a
         * pending conversion first). The loop tests the condition, not the
         * counter. The old form, while (enabled && u8DelayMs--), left the
         * post-decremented counter at 255 when the ADC stayed enabled, so the
         * timeout was never seen and the forced disable was skipped. When the
         * ADC went idle on the last wait it left the counter at 0 and reported
         * a false error. */
        uint8_t u8WaitMs = 0U;
        while (BAT_DRIVER_bIsEnabled() && (u8WaitMs < BAT_ADC_IDLE_WAIT_MS))
        {
            osDelay(1);
            u8WaitMs++;
        }

        if (BAT_DRIVER_bIsEnabled())
            BAT_DRIVER_vDisable();

        BAT_DRIVER_vEnable();

        /* Read both the battery divider and VREFINT in the same enabled window.
         * VREFINT yields the true VDDA, making the battery result independent
         * of the actual rail voltage (2.0 V nominal on Tagskaap V3 RevA). */
        uint16_t u16BatRaw  = 0;
        uint16_t u16VrefRaw = 0;
        bool bConvOk = BAT_bConvert(BAT_VOLTAGE_CHANNEL, &u16BatRaw)
                    && BAT_bConvert(VREFINT_CHANNEL,     &u16VrefRaw);

        BAT_DRIVER_vDisable();
        BAT_DRIVER_vDisableBiasCircuit();

        /* ADC and sleep no longer needed — the VDDA/divider math below is
         * pure arithmetic. Release before computing so the shared ADC and
         * STOP2 are freed as early as possible. */
        SYSTEM_vSleepLockRelease();
        HAL_ADC_vUnlock();

        /* A sample is valid only if both conversions completed AND the
         * VREFINT-derived VDDA is plausible: HAL_ADC_u16VddaFromVrefint()
         * returns 0 for a raw 0 or a VDDA outside the ADC supply range. The
         * battery result shares the VDDA, so an invalid sample is dropped
         * whole. It never enters the buffer (which would drag the average
         * towards 0 mV or a wrapped garbage value). It is tallied instead of
         * logged (no stack for DBG_LOG here), and bSeedPending stays armed. */
        uint16_t u16Vdda = bConvOk ? HAL_ADC_u16VddaFromVrefint(u16VrefRaw) : 0U;
        if (u16Vdda == 0U)
        {
            taskENTER_CRITICAL();
            if (u16BatSampleFails < UINT16_MAX)
                u16BatSampleFails++;
            taskEXIT_CRITICAL();
            continue;
        }

        /* Vbatt = adc_bat * VDDA / full_scale * divider_ratio.
         * Computed in one 64-bit expression to avoid intermediate
         * truncation; the divider ratio comes from BAT_DIVIDER_NUM/DEN. With
         * VDDA capped at HAL_ADC_VDDA_MAX_MV the result is at most 15480 mV,
         * so the uint16_t cast cannot wrap. */
        uint16_t u16BatMv = (uint16_t)
            (((uint64_t)u16BatRaw * u16Vdda * BAT_DIVIDER_NUM)
             / ((uint64_t)BAT_ADC_FULL_SCALE * BAT_DIVIDER_DEN));

        if (bSeedPending)
        {
            /* First valid sample since boot or since a purge request: fill
             * every slot with it so the average is meaningful at once. The
             * fill and the gate store share one critical section. That is the
             * compiler/memory barrier that keeps a lock-free reader from
             * seeing bBatSeeded before the buffer is filled, and it stops any
             * reader from running during the fill loop itself. It cannot stop
             * a torn mean: a lower-priority reader that this task preempts
             * part-way through its sum in BAT_u16GetVoltage() (for example the
             * DbgLog prefix on a low-priority task) can return a one-off mix
             * of pre-seed and seeded slots. That is acceptable for a log or
             * report value; Power.c, which acts on the voltage, reads only
             * after BAT_bHasValidSample(). A seed is not delta-limited:
             * there is no trustworthy previous sample to limit it against. */
            taskENTER_CRITICAL();
            for (uint8_t i = 0; i < BAT_AVG_FACTOR; i++)
                au16BatAvgBuf[i] = u16BatMv;
            u8BatAvgIdx          = 0U;
            u16BatPreviousSample = u16BatMv;
            bBatSeeded           = true;
            taskEXIT_CRITICAL();

            bSeedPending = false;
            continue;
        }

        /* Delta limit: a scheduled sample may fall at most
         * BAT_SAMPLE_VALUE_DELTA_MAX below the previous one (rises are not
         * limited). Integer form of the old float comparison, with the same
         * result: for a previous sample of 20 mV or less the float version
         * compared against zero or a negative number and never clamped
         * either, which the u16BatPreviousSample > BAT_SAMPLE_VALUE_DELTA_MAX
         * guard reproduces without an unsigned wrap. */
        if (bDeltaLimit
            && (u16BatPreviousSample > BAT_SAMPLE_VALUE_DELTA_MAX)
            && (u16BatMv < (uint16_t)(u16BatPreviousSample - BAT_SAMPLE_VALUE_DELTA_MAX)))
        {
            u16BatMv = (uint16_t)(u16BatPreviousSample - BAT_SAMPLE_VALUE_DELTA_MAX);
        }

        au16BatAvgBuf[u8BatAvgIdx] = u16BatMv;
        u16BatPreviousSample       = u16BatMv;
        u8BatAvgIdx = (uint8_t)((u8BatAvgIdx + 1U) % BAT_AVG_FACTOR);
    }
}

/* --------------------------------------------------------------------------
 * BAT_vCheckSampleScheduleTask — heartbeat-driven sample scheduler
 * -------------------------------------------------------------------------- */
static void BAT_vCheckSampleScheduleTask(void *pvParameters)
{
    (void)pvParameters;

    PLATFORM_bSubscribeToHeartbeat(osThreadGetId(), HB_ALLOW_IN_RECOVERY);

    uint8_t u8UpdateCnt = 0;

    for (;;)
    {
        /* Wait for 1-second heartbeat notification */
        osThreadFlagsWait(0x7FFFFFFFU, osFlagsWaitAny, osWaitForever);

        if (u8UpdateCnt % BAT_SAMPLE_INTERVAL == 0)
        {
            u8UpdateCnt = 0;
            osThreadFlagsSet(BAT_vSampleTask_handle, BAT_NOTIFY_DELTA_LIMIT);
        }
        u8UpdateCnt++;
    }
}

/* --------------------------------------------------------------------------
 * BAT_vPurgeBuffer — request an immediate sample that refills the whole
 * averaging buffer.
 *
 * Only sets BAT_NOTIFY_PURGE_REQUEST on BatSampleTask, which takes the sample
 * and does the fill itself. If that sample fails, the next valid sample
 * (normally the next scheduled one, BAT_SAMPLE_INTERVAL heartbeats later)
 * refills the buffer instead, so a failed conversion never seeds 0 mV. Call
 * only after BAT_vInit() has created BatSampleTask.
 * -------------------------------------------------------------------------- */
void BAT_vPurgeBuffer(void)
{
    (void)osThreadFlagsSet(BAT_vSampleTask_handle, BAT_NOTIFY_PURGE_REQUEST);
}

/* --------------------------------------------------------------------------
 * BAT_u16GetVoltage — average of last BAT_AVG_FACTOR samples, in mV
 *
 * Returns 0 until the first valid sample has seeded the buffer. A caller that
 * acts on the value, rather than just reporting it, must check
 * BAT_bHasValidSample() first. Lock-free on purpose: DbgLog's line prefix calls
 * it inside every DBG_LOG, from any task and from timer callbacks.
 * -------------------------------------------------------------------------- */
uint16_t BAT_u16GetVoltage(void)
{
    /* The averaging buffer already holds VREFINT-compensated millivolts
     * (computed per sample in BAT_vSampleTask), so just return the mean. */
    uint32_t u32Temp = 0;
    for (uint8_t i = 0; i < BAT_AVG_FACTOR; i++)
        u32Temp += au16BatAvgBuf[i];

    return (uint16_t)(u32Temp / (uint32_t)BAT_AVG_FACTOR);
}

/* --------------------------------------------------------------------------
 * BAT_bHasValidSample — true once a valid sample has seeded the averaging
 * buffer. Monotonic: it never goes false again, because later failed samples
 * leave the last good average in place. Lock-free.
 * -------------------------------------------------------------------------- */
bool BAT_bHasValidSample(void)
{
    return bBatSeeded;
}

/* --------------------------------------------------------------------------
 * BAT_u16GetAndClearSampleFails — number of invalid battery samples (EOC
 * timeout or implausible VDDA) since the previous call, saturating at
 * UINT16_MAX. Task context only (critical section).
 * -------------------------------------------------------------------------- */
uint16_t BAT_u16GetAndClearSampleFails(void)
{
    taskENTER_CRITICAL();
    uint16_t u16Fails = u16BatSampleFails;
    u16BatSampleFails = 0U;
    taskEXIT_CRITICAL();

    return u16Fails;
}

/* --------------------------------------------------------------------------
 * BAT_u8ConvertVoltageToPercentage
 * -------------------------------------------------------------------------- */
uint8_t BAT_u8ConvertVoltageToPercentage(uint16_t u16Voltage)
{
    uint32_t u32Temp = max(u16Voltage, 3350);
    u32Temp -= 3350;
    u32Temp  = 100 * u32Temp;
    u32Temp  = min(u32Temp / (4100 - 3350), 100);
    return (uint8_t)u32Temp;
}

/* --------------------------------------------------------------------------
 * BAT_u8GetPercentage
 * -------------------------------------------------------------------------- */
uint8_t BAT_u8GetPercentage(void)
{
    return BAT_u8ConvertVoltageToPercentage(BAT_u16GetVoltage());
}
