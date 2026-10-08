/*
 * SolarPower.c
 *
 * Solar panel voltage measurement — CMSIS-RTOS v2 task layer.
 *
 * Two tasks:
 *   SOLAR_vSampleTask            — triggered by thread flag; performs one
 *                                  VSOLAR ADC sample.
 *   SOLAR_vCheckSampleScheduleTask — subscribes to the 1-second heartbeat;
 *                                  triggers a sample every 10 ticks.
 *
 * The ADC is shared with the Battery worker. HAL_ADC_vLock / vUnlock
 * serialize access so only one measurement runs at a time.
 */

#include "SolarPower.h"
#include "SolarPower_Driver.h"
#include "SolarPower_Config.h"

#include "hal_system.h"
#include "platform.h"
#include "math_func.h"
#include "dbg_log.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* ---- Thread flags ---- */
#define SOLAR_NOTIFY_SAMPLE     (1UL << 0)

/* ---- Task stack / priority ---- */
#define SOLAR_SAMPLETASK_STACK_SIZE     (configMINIMAL_STACK_SIZE * 3)
#define SOLAR_SCHEDTASK_STACK_SIZE      (configMINIMAL_STACK_SIZE)

/* ---- CMSIS-RTOS v2 handles ---- */
static osThreadId_t SOLAR_vSampleTask_handle;
static osThreadId_t SOLAR_vCheckSampleScheduleTask_handle;

/* ---- Last measured values (mV) ---- */
static volatile uint16_t u16VsolarLastMV;

/* ---- Forward declarations ---- */
static void SOLAR_vSampleTask(void *pvParameters);
static void SOLAR_vCheckSampleScheduleTask(void *pvParameters);
static bool SOLAR_bConvert(hal_adc_channel_t channel, uint16_t *pu16Result);

/* --------------------------------------------------------------------------
 * SOLAR_bConvert — one blocking ADC conversion on the given channel.
 * The ADC must already be enabled. Returns true and writes *pu16Result on
 * success; false if the end-of-conversion flag never arrived.
 * -------------------------------------------------------------------------- */
static bool SOLAR_bConvert(hal_adc_channel_t channel, uint16_t *pu16Result)
{
    uint8_t u8DelayMs = 3;

    SOLAR_DRIVER_vCleanInterrupt();
    HAL_ADC_vStartConversion(channel);

    while (!SOLAR_DRIVER_bGetInterruptFlag() && u8DelayMs--)
        osDelay(1);

    if (!SOLAR_DRIVER_bGetInterruptFlag())
        return false;

    *pu16Result = SOLAR_DRIVER_u16GetResult();
    return true;
}

/* --------------------------------------------------------------------------
 * SOLAR_vInit
 * -------------------------------------------------------------------------- */
void SOLAR_vInit(void)
{
    static const osThreadAttr_t sample_attr = {
        .name       = "SolarSampleTask",
        .stack_size = SOLAR_SAMPLETASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };
    static const osThreadAttr_t sched_attr = {
        .name       = "SolarSchedTask",
        .stack_size = SOLAR_SCHEDTASK_STACK_SIZE * sizeof(StackType_t),
        .priority   = osPriorityNormal,
    };

    SOLAR_vSampleTask_handle =
        osThreadNew(SOLAR_vSampleTask, NULL, &sample_attr);
    SOLAR_vCheckSampleScheduleTask_handle =
        osThreadNew(SOLAR_vCheckSampleScheduleTask, NULL, &sched_attr);

    configASSERT(SOLAR_vSampleTask_handle              != NULL);
    configASSERT(SOLAR_vCheckSampleScheduleTask_handle != NULL);
}

/* --------------------------------------------------------------------------
 * SOLAR_vSampleTask — triggered by thread flag, performs ADC sampling
 * -------------------------------------------------------------------------- */
static void SOLAR_vSampleTask(void *pvParameters)
{
    (void)pvParameters;

    uint16_t u16RawVsolar = 0;
    uint16_t u16RawVref   = 0;

    for (;;)
    {
        osThreadFlagsWait(SOLAR_NOTIFY_SAMPLE, osFlagsWaitAny, osWaitForever);

        SOLAR_DRIVER_vLock();
        SYSTEM_vSleepLockAcquire();

        SOLAR_DRIVER_vEnable();

        /* Read VSOLAR plus VREFINT in one enabled window. VREFINT gives the
         * true VDDA so results are independent of the actual rail (2.0 V
         * nominal on Tagskaap V3 RevA). */
        bool bConvOk = SOLAR_bConvert(VSOLAR_VOLTAGE_CHANNEL, &u16RawVsolar)
                    && SOLAR_bConvert(VREFINT_CHANNEL,        &u16RawVref);

        SOLAR_DRIVER_vDisable();

        SYSTEM_vSleepLockRelease();
        SOLAR_DRIVER_vUnlock();

        if (!bConvOk)
        {
            DBG_LOG("solar: ADC conversion timeout — sample skipped\r\n");
            continue;
        }

        /* ---- Convert to millivolts using the measured VDDA ---- */
        uint16_t u16Vdda = HAL_ADC_u16VddaFromVrefint(u16RawVref);

        if (u16Vdda == 0U)
        {
            /* Implausible VREFINT (raw 0, or a VDDA outside the ADC supply
             * range): the VSOLAR conversion shares that VDDA, so skip the
             * whole sample and keep the last good Vsolar. A VDDA above the
             * ADC supply range must never reach u16VsolarLastMV, because it
             * could read a dark panel as >= SOLAR_ACTIVATION_VSOLAR_MV and
             * pull a shelved tag out of ProductionSleep. This guard is the
             * silicon range (HAL_ADC_VDDA_MIN_MV..HAL_ADC_VDDA_MAX_MV,
             * 1.62-3.6 V), not a window for this board: a wrong VDDA that
             * still falls inside it (say 3.0 V reported on the 2.0 V rail) is
             * not caught and scales Vsolar up in proportion. Until the first valid sample
             * u16VsolarLastMV stays 0, which is the safe direction (the tag
             * stays asleep), so no validity gate is needed here. This task has
             * a 3x stack, so it can afford the log line. */
            DBG_LOG("solar: implausible VREFINT (raw %u) — sample skipped\r\n",
                    (unsigned)u16RawVref);
            continue;
        }

        /* Vsolar = adc * VDDA / full_scale * divider_ratio (64-bit, no
         * truncation). With VDDA capped at HAL_ADC_VDDA_MAX_MV the result is
         * at most 15480 mV, so the uint16_t cast cannot wrap. */
        u16VsolarLastMV = (uint16_t)
            (((uint64_t)u16RawVsolar * u16Vdda * SOLAR_VSOLAR_DIV_NUM)
             / ((uint64_t)SOLAR_ADC_FULL_SCALE * SOLAR_VSOLAR_DIV_DEN));

#ifdef ENABLE_SOLAR_PERIODIC_LOG
        DBG_LOG("solar: Vsolar=%u mV  Vdda=%u mV\r\n", u16VsolarLastMV, u16Vdda);
#endif
    }
}

/* --------------------------------------------------------------------------
 * SOLAR_vCheckSampleScheduleTask — heartbeat-driven sample scheduler
 * -------------------------------------------------------------------------- */
static void SOLAR_vCheckSampleScheduleTask(void *pvParameters)
{
    (void)pvParameters;

    PLATFORM_bSubscribeToHeartbeat(osThreadGetId(), HB_ALLOW_IN_RECOVERY);

    uint8_t u8UpdateCnt = 0;

    for (;;)
    {
        osThreadFlagsWait(0x7FFFFFFFU, osFlagsWaitAny, osWaitForever);

        if (u8UpdateCnt % SOLAR_SAMPLE_INTERVAL == 0)
        {
            u8UpdateCnt = 0;
            osThreadFlagsSet(SOLAR_vSampleTask_handle, SOLAR_NOTIFY_SAMPLE);
        }
        u8UpdateCnt++;
    }
}

/* --------------------------------------------------------------------------
 * Public getters
 * -------------------------------------------------------------------------- */
uint16_t SOLAR_u16GetVSolarMV(void)
{
    return u16VsolarLastMV;
}

