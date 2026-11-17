/*
 * hal_subghz.h
 *
 * Sub-GHz radio SPI peripheral driver.
 */

#ifndef INC_HAL_SUBGHZ_H_
#define INC_HAL_SUBGHZ_H_

#include "stm32wlxx_hal.h"

extern SUBGHZ_HandleTypeDef hsubghz;

void HAL_vSUBGHZ_Init(void);
/* Copies the MCU's 96-bit unique ID into pau32Uid[0..2] = UID word 0..2
 * (RM0461, "Unique device ID register"). Interpreting it is not this layer's
 * job; LORARADIO_u32GetUniqueId() builds the radio id from it. */
void HAL_SUBGHZ_vGetUid96(uint32_t pau32Uid[3]);

#endif /* INC_HAL_SUBGHZ_H_ */
