/*
 * hal_exti.h
 *
 * EXTI callback registration for GPIO interrupt sources.
 */

#ifndef INC_HAL_EXTI_H_
#define INC_HAL_EXTI_H_

#include <stdint.h>

/* EXTI sources the dispatcher knows about. ACC_INT (PC13, LIS2DH INT1) is
 * routed on the board but unused, and no EXTI line is armed for it; see
 * hal_exti.c for what using it would take. */
typedef enum {
    ACC_INT_EXTI_PIN = 0,
} hal_exti_pin_t;

typedef void (*callback_function_t)(void);

void HAL_EXTI_vRegisterCallback(hal_exti_pin_t pin, callback_function_t callback_function);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_HAL_EXTI_H_ */
