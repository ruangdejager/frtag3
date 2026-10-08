/*
 * hal_gpio.h
 *
 * GPIO safe-state initialisation, STOP2 sleep/wake helpers, pin
 * configuration helpers, and LED permission control.
 */

#ifndef HAL_GPIO_H_
#define HAL_GPIO_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32wlxx.h"
#include "hal_bsp.h"

typedef enum _leds_active_t
{
    LEDS_ACTIVE_RESERVED = 0,
    LEDS_ACTIVE_ENABLE,
    LEDS_ACTIVE_DISABLE,
} leds_active_t;

/* Board GPIO safe state. Called from main() right after HAL_Init() and before
 * SystemClock_Config(), so the pins with no external pull are driven before
 * the LSE start (up to 5 s when the image runs without the bootloader; behind
 * the bootloader the LSE is already running). It therefore runs before the kernel
 * exists, at the reset MSI clock: keep it free of RTOS calls, logging and
 * delays. */
void HAL_GPIO_vInit(void);

/* STOP2 entry: parks the UART pins analog and gates the GPIO port clocks.
 * HAL_SPI_vOnSleep() must already have parked the SPI pins (it needs the
 * clocks this function turns off). */
void HAL_GPIO_vOnSleep(void);

/* STOP2 exit: re-enables the GPIO port clocks and nothing else. */
void HAL_GPIO_OnWake(void);

void HAL_GPIO_vInitInput(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, uint32_t gpio_pull);
void HAL_GPIO_vInitIntPullup(GPIO_TypeDef *GPIOx, uint32_t gpio_pin);

/* Push-pull output, no pull, low speed. Writes the level BEFORE switching
 * MODER to output, so the pin never drives the stale ODR value; the port clock
 * must already be on or that write is lost. Accepts a multi-pin mask. */
void HAL_GPIO_vInitOutput(GPIO_TypeDef *GPIOx, uint32_t gpio_pin, GPIO_PinState gpio_pinstate);

/* Analog mode with every pull in the mask cleared. The vendor HAL leaves
 * PUPDR untouched in analog mode, so this clears it explicitly (see
 * hal_gpio.c). Accepts a multi-pin mask. */
void HAL_GPIO_vInitAnalogNoPull(GPIO_TypeDef *GPIOx, uint32_t gpio_pin);

bool HAL_GPIO_bLedsAllowed(void);
void HAL_GPIO_vAllowLeds(bool allow);

#endif /* HAL_GPIO_H_ */
