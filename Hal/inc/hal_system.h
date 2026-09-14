/*
 * hal_system.h
 *
 * System clock, power, and sleep management for the frtag board.
 */

#ifndef INC_HAL_SYSTEM_H_
#define INC_HAL_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>

void SystemClock_Config(void);
bool SYSTEM_bCheckSleepModeStatus(void);

/* Sleep-lock API — any task that needs the system to stay out of STOP2
 * (deep sleep) calls SYSTEM_vSleepLockAcquire() and, once it no longer
 * needs the system awake, SYSTEM_vSleepLockRelease(). Acquire/release
 * pairs may come from independent tasks/modules and nest via a shared
 * counter; deep sleep is only re-armed once the count returns to zero. */
void SYSTEM_vSleepLockAcquire(void);
void SYSTEM_vSleepLockRelease(void);

void Error_Handler(void);

/* Sub-millisecond busy-wait for hardware settle times, calibrated off
 * SystemCoreClock. Deliberately NOT built on HAL_Delay/HAL_vDelayUs or osDelay:
 * the HAL delays depend on a tick this project does not drive the way they
 * expect, and osDelay's 1 ms granularity is both far too coarse and a yield
 * where the caller needs the pin state held. This is a spin — keep the
 * durations to the tens of microseconds the settle actually needs. */
void HAL_SYSTEM_vBusyWaitUs(uint32_t u32Us);

/* Enters STOP2 and restores clocks/peripherals on wake. Called from the
 * tickless-idle sleep path (vPortSuppressTicksAndSleep) with interrupts
 * already masked and deep-sleep eligibility already decided. */
void HAL_SYSTEM_vEnterStop2(void);

/* Which LED indicates STOP2 residency (off in STOP2, on otherwise). Red
 * is the default; DeviceDiscovery swaps it to yellow while the secondary
 * is in ProductionSleep so the bench can distinguish "asleep waiting for
 * solar / kernel wakeup" from "asleep between normal scheduled wakes".
 *
 * The setter turns the previously-selected LED off before switching so
 * two indicator LEDs are never lit at once. Safe to call from any task
 * context; the switch takes effect on the next STOP2 entry/exit. */
typedef enum { HAL_SYSTEM_SLEEP_LED_RED = 0, HAL_SYSTEM_SLEEP_LED_YELLOW = 1 } HalSystemSleepLed_e;
void HAL_SYSTEM_vSetSleepIndicatorLed(HalSystemSleepLed_e eLed);

/* Current sleep-indicator LED. Movement's shake-sequence confirmation flash
 * (Movement_Driver.h) reads this to always blink the OTHER LED, so it's never
 * fighting the sleep indicator for the same physical LED — e.g. during
 * ProductionSleep/SolarSleep the indicator owns yellow, so the shake flash
 * moves to red instead of being stomped by the STOP2 entry/exit and 2 Hz
 * lock-held pulsing that also drive yellow in that mode. */
HalSystemSleepLed_e HAL_SYSTEM_eGetSleepIndicatorLed(void);

/* GPS-powered indication. While set, the companion LED (whichever one is NOT
 * the sleep indicator) flashes in step with the indicator, so "GNSS module is
 * on" reads as both LEDs blinking together. Pushed in by the GPS module on
 * power-on/off so this HAL layer needs no device-layer dependency; the
 * falling edge forces the companion LED off so it can't be left lit. */
void HAL_SYSTEM_vSetGpsActive(bool bActive);

#endif /* INC_HAL_SYSTEM_H_ */
