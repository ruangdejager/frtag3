/*
 * tag_hal.h
 *
 * Top-level HAL include aggregator for the frtag board.
 */

#ifndef INC_TAG_HAL_H_
#define INC_TAG_HAL_H_

#include <stdint.h>

#include "stm32wlxx.h"
#include "hal_gpio.h"
#include "hal_crc.h"
#include "hal_delay.h"
#include "hal_rtc.h"
#include "hal_timer.h"
#include "hal_uart.h"
#include "hal_system.h"
#include "hal_wdt.h"

/* There is deliberately no combined "HAL_vInit()" here any more. main() calls
 * HAL_Init(), HAL_GPIO_vInit() and SystemClock_Config() explicitly, in that
 * order, so the constraint that the board pins reach their safe state before
 * the LSE start (multi-second only when this image runs without the
 * bootloader; behind the bootloader the LSE is already running) is visible where it matters. */

#define HAL_vResetMCU()  NVIC_SystemReset()

#endif /* INC_TAG_HAL_H_ */
