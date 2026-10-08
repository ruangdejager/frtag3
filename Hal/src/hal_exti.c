/*
 * hal_exti.c
 *
 * EXTI callback dispatch for GPIO interrupt lines.
 * Register application callbacks via HAL_EXTI_vRegisterCallback().
 *
 * No EXTI line is armed on this board today, so nothing here is reached at
 * run time (the linker discards both functions). The only candidate is
 * ACC_INT (PC13), routed from the LIS2DH INT1 output on a secondary but unused
 * and parked analog by HAL_GPIO_vInit(). No charger status line (the old
 * CHG_nPG power-good input) is wired to an MCU pin on frtag2 or frtag3, so the
 * dispatcher entry that used to be named after it is now named after the line
 * it would actually serve.
 */

#include <stddef.h>
#include "hal_exti.h"
#include "stm32wlxx.h"
#include "hal_bsp.h"

static callback_function_t accIntCallback = NULL;

void HAL_EXTI_vRegisterCallback(hal_exti_pin_t pin, callback_function_t callback_function)
{
    switch (pin)
    {
        case ACC_INT_EXTI_PIN:
            accIntCallback = callback_function;
            break;
        default:
            break;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Not wired to any EXTI line: ACC_INT (PC13) is routed but unused and
     * parked analog. To use it, configure it as an EXTI input with no pull (the
     * LIS2DH drives INT1 itself), keep it out of every STOP2 park mask, add an
     * EXTI15_10_IRQHandler that calls HAL_GPIO_EXTI_IRQHandler(BSP_ACC_INT_PIN)
     * - the startup file maps that vector to Default_Handler, an infinite loop,
     * so arming the line without a handler hangs the MCU - and dispatch to
     * accIntCallback from here. */
    (void)GPIO_Pin;
}
