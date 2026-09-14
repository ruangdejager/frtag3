/*
 * hal_subghz.h
 *
 * Sub-GHz radio SPI peripheral driver.
 */

#ifndef INC_HAL_SUBGHZ_H_
#define INC_HAL_SUBGHZ_H_

#include <stdbool.h>

#include "stm32wlxx_hal.h"

extern SUBGHZ_HandleTypeDef hsubghz;

void HAL_vSUBGHZ_Init(void);
void HAL_SUBGHZ_vSetUniqueId(uint8_t *id);

/* --------------------------------------------------------------------------
 * RF switch supply rail (RF_SW_VDD, see hal_bsp.h)
 *
 * These follow the RADIO's power state, not individual TX/RX operations, and
 * are called from exactly two places: the radio's wake path and its sleep
 * path (fr_app/Device/LoraRadio/drivers/LoraRadio_Driver.c). The rail stays
 * up across every RX<->TX turnaround in between.
 *
 * Task context only — both busy-wait on a settle time.
 * -------------------------------------------------------------------------- */
void HAL_SUBGHZ_vRfSwitchPowerOn(void);
void HAL_SUBGHZ_vRfSwitchPowerOff(void);

/* True while RF_SW_VDD is asserted, i.e. while the radio is out of sleep.
 * The path-select call checks this, and the STOP2 entry path uses it to decide
 * whether the rail has to survive the sleep. */
bool HAL_SUBGHZ_bRfSwitchPowered(void);

#endif /* INC_HAL_SUBGHZ_H_ */
