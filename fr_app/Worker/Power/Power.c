/*
 * Power.c
 *
 * Power class management — battery-voltage driven normal/recovery switching.
 *
 * Power state is stored in an osEventFlags object so that any task can
 * query (POWER_tGetState) or block on a class (POWER_vWaitForClass).
 *
 * The PowerStateManager task only runs on secondary devices; it subscribes
 * to the 1-second platform heartbeat and adjusts the power class based on
 * the measured battery voltage, starting at the first valid battery sample.
 * A primary never creates it and stays in NORMAL for good.
 *
 * Uses CMSIS-RTOS v2 throughout.
 */

#include "Power.h"
#include "Power_Config.h"
#include "platform.h"
#include "Battery.h"
#include "DeviceDiscovery.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"   /* configMINIMAL_STACK_SIZE */
#include "task.h"

#include <limits.h>

/* ---- CMSIS-RTOS v2 objects ---- */
static osEventFlagsId_t gPowerEvents           = NULL;
static osThreadId_t     xPowerStateManagerTaskHandle = NULL;

/* ---- Forward declarations ---- */
static void POWER_vStateManagerTask(void *arg);

/* --------------------------------------------------------------------------
 * POWER_vInit
 * -------------------------------------------------------------------------- */
void POWER_vInit(void)
{
    gPowerEvents = osEventFlagsNew(NULL);
    configASSERT(gPowerEvents != NULL);

    POWER_vSetModeNormal();

    if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
    {
        static const osThreadAttr_t mgr_attr = {
            .name       = "PowerStateMgr",
            .stack_size = configMINIMAL_STACK_SIZE * sizeof(StackType_t),
            .priority   = osPriorityNormal,
        };
        xPowerStateManagerTaskHandle =
            osThreadNew(POWER_vStateManagerTask, NULL, &mgr_attr);
        configASSERT(xPowerStateManagerTaskHandle != NULL);
    }
}

/* --------------------------------------------------------------------------
 * POWER_vSetModeNormal
 * -------------------------------------------------------------------------- */
void POWER_vSetModeNormal(void)
{
    osEventFlagsSet(gPowerEvents,   POWER_CLASS_NORMAL | POWER_CLASS_ALWAYS);
    osEventFlagsClear(gPowerEvents, POWER_CLASS_LOW | POWER_CLASS_RECOVERY);
}

/* --------------------------------------------------------------------------
 * POWER_vSetModeLow
 * -------------------------------------------------------------------------- */
void POWER_vSetModeLow(void)
{
    osEventFlagsSet(gPowerEvents,   POWER_CLASS_LOW | POWER_CLASS_ALWAYS);
    osEventFlagsClear(gPowerEvents, POWER_CLASS_NORMAL | POWER_CLASS_RECOVERY);
}

/* --------------------------------------------------------------------------
 * POWER_vSetModeRecovery
 * -------------------------------------------------------------------------- */
void POWER_vSetModeRecovery(void)
{
    osEventFlagsSet(gPowerEvents,   POWER_CLASS_RECOVERY | POWER_CLASS_ALWAYS);
    osEventFlagsClear(gPowerEvents, POWER_CLASS_NORMAL | POWER_CLASS_LOW);
}

/* --------------------------------------------------------------------------
 * POWER_vWaitForClass — block until the requested power class is active
 * -------------------------------------------------------------------------- */
void POWER_vWaitForClass(uint32_t classMask)
{
    osEventFlagsWait(gPowerEvents,
                     classMask,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
}

/* --------------------------------------------------------------------------
 * POWER_tGetState — return current power class flags (non-blocking)
 * -------------------------------------------------------------------------- */
uint32_t POWER_tGetState(void)
{
    return osEventFlagsGet(gPowerEvents);
}

/* --------------------------------------------------------------------------
 * POWER_vStateManagerTask — heartbeat-driven voltage check (secondary only)
 *
 * Three-class state machine driven by battery voltage:
 *
 *   NORMAL   ──(V ≤ ENTER_LOW_MV)──►  LOW       ──(V ≤ ENTER_RECOVERY_MV)──►  RECOVERY
 *   NORMAL  ◄──(V ≥ EXIT_LOW_MV) ──   LOW      ◄──(V ≥ EXIT_RECOVERY_MV) ──   RECOVERY
 *
 * Hysteresis windows prevent oscillation:
 *   LOW ↔ NORMAL boundary  : 20 mV (3500 → 3520)
 *   LOW ↔ RECOVERY boundary: 50 mV (3400 → 3450)
 *
 * Evaluation starts at the first valid battery sample (BAT_bHasValidSample()).
 * Until then the class stays at the NORMAL that POWER_vInit() set. Reading the
 * still-empty average as 0 mV used to push a freshly booted secondary into a
 * bogus low-power class in two ways: a 1 s drop to LOW (GPS refused) when the
 * first heartbeat beat the boot sample, or - when the boot sample failed and
 * seeded the buffer with 0 mV - LOW and then RECOVERY (scheduled wakes and
 * beacons suppressed) for ~70-80 s until good samples refilled the average.
 * Consequence, by design: if samples keep failing at the conversion or
 * plausibility level, a secondary stays in NORMAL indefinitely with an unknown battery voltage. It fails
 * operational (still discoverable, GPS allowed) instead of failing into
 * RECOVERY as it did before. Failed samples are counted in Battery
 * (BAT_u16GetAndClearSampleFails()), not here. An ADC enable/stop/disable
 * timeout counts the same way: the HAL error bit it latches is cleared on the
 * next lazy HAL_ADC_vInit() once the peripheral is confirmed idle (see
 * hal_adc.c). Only an ADC that cannot be stopped at all still ends in
 * Error_Handler() and a watchdog reset.
 *
 * No logging in this task: it runs on a configMINIMAL_STACK_SIZE (512 B) stack
 * and a DBG_LOG call chain needs about 536 B.
 * -------------------------------------------------------------------------- */
static void POWER_vStateManagerTask(void *arg)
{
    (void)arg;

    PLATFORM_bSubscribeToHeartbeat(osThreadGetId(), HB_ALLOW_IN_RECOVERY);

    for (;;)
    {
        osThreadFlagsWait(0x7FFFFFFFU, osFlagsWaitAny, osWaitForever);

        /* No battery sample has succeeded since boot: hold the NORMAL class
         * POWER_vInit() set (nothing else changes the class) rather than read
         * the empty average as 0 mV and walk down to RECOVERY.
         * BAT_bHasValidSample() is monotonic, so once it opens, later failed
         * conversions leave the last good average in place and evaluation
         * never pauses again. */
        if (!BAT_bHasValidSample())
            continue;

        uint16_t u16Bat   = BAT_u16GetVoltage();
        uint32_t u32State = POWER_tGetState();

        if (u32State & POWER_CLASS_NORMAL)
        {
            /* From NORMAL, drop to LOW when at/below ENTER_LOW_MV.
             * A direct NORMAL→RECOVERY transition is also covered: LOW will
             * collapse to RECOVERY on the next tick if voltage stays low. */
            if (u16Bat <= ENTER_LOW_MV)
                POWER_vSetModeLow();
        }
        else if (u32State & POWER_CLASS_LOW)
        {
            if (u16Bat <= ENTER_RECOVERY_MV)
                POWER_vSetModeRecovery();
            else if (u16Bat >= EXIT_LOW_MV)
                POWER_vSetModeNormal();
        }
        else if (u32State & POWER_CLASS_RECOVERY)
        {
            /* RECOVERY only releases up to LOW; LOW→NORMAL is a separate
             * step that requires another EXIT_LOW_MV crossing. This makes
             * the climb out of a deep brown-out conservative. */
            if (u16Bat >= EXIT_RECOVERY_MV)
                POWER_vSetModeLow();
        }
    }
}
