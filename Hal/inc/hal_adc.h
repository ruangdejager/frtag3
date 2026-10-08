/*
 * hal_adc.h
 *
 * ADC driver — battery and VSOLAR voltage measurement.
 *
 * The ADC peripheral is shared across multiple workers. Callers must
 * acquire the ADC mutex (HAL_ADC_vLock / HAL_ADC_vUnlock) around the
 * full enable → convert → disable sequence to prevent concurrent access.
 */

#ifndef INC_HAL_ADC_H_
#define INC_HAL_ADC_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32wlxx_hal.h"
#include "cmsis_os2.h"

typedef enum {
    BAT_VOLTAGE_CHANNEL    = 0,
    VSOLAR_VOLTAGE_CHANNEL = 1,
    VREFINT_CHANNEL        = 2,
} hal_adc_channel_t;

extern ADC_HandleTypeDef hadc;

void     HAL_ADC_vInit(void);
/* Flag the ADC as torn down by STOP2; the next HAL_ADC_vLock() re-inits it
 * lazily. Call from the wake path instead of HAL_ADC_vInit() so idle 1 Hz
 * wakes don't pay the init + calibration cost. */
void     HAL_ADC_vMarkParked(void);
void     HAL_ADC_vEnable(void);
bool     HAL_ADC_bIsEnabled(void);
/* Stop any conversion still pending, then disable the ADC. Task context only,
 * with the ADC lock held (it uses the vendor HAL's tick-based timeouts and
 * handle lock). */
void     HAL_ADC_vDisable(void);
bool     HAL_ADC_bGetInterruptFlag(void);
void     HAL_ADC_vClearInterruptFlag(void);
void     HAL_ADC_vSelectChannel(hal_adc_channel_t channel);
void     HAL_ADC_vStartConversion(hal_adc_channel_t channel);
uint16_t HAL_ADC_u16GetResult(void);

/* Serialize access to the shared ADC across workers (battery, solar). Acquire
 * around the full enable → convert → disable sequence. */
void     HAL_ADC_vLock(void);
void     HAL_ADC_vUnlock(void);

/* Plausibility window for a VREFINT-derived VDDA, in mV. These are the STM32WL
 * ADC supply limits ("ADC supply requirements: 1.62 V to 3.6 V" in the vendor
 * stm32wlxx_hal_adc.c), not a board option, which is why they live here and
 * not in build_config.h. A computed VDDA outside this window is not a real
 * operating point: below it the ADC is out of spec, and with the BOR option
 * byte at level 0 the part resets near 1.6 V anyway, so running code cannot
 * observe it. It means the VREFINT conversion itself was bad (a garbage or
 * stuck result, or an unprogrammed VREFINT_CAL that reads 0xFFFF), and the
 * paired external-channel result taken in the same window cannot be trusted
 * either. The nominal rail on Tagskaap V3 RevA is 2.0 V, well inside the
 * window. */
#define HAL_ADC_VDDA_MIN_MV     1620U
#define HAL_ADC_VDDA_MAX_MV     3600U

/* Convert a raw VREFINT conversion (12-bit) into the actual analog supply
 * voltage VDDA in mV, using the chip's factory VREFINT calibration. Returns
 * 0 when the raw value is 0 (conversion failed) or when the computed VDDA is
 * outside [HAL_ADC_VDDA_MIN_MV, HAL_ADC_VDDA_MAX_MV]. Callers must treat 0 as
 * "sample invalid" and discard the channel result taken alongside it, rather
 * than scale it by 0 mV or by an implausible VDDA. */
uint16_t HAL_ADC_u16VddaFromVrefint(uint16_t u16VrefintRaw);

#endif /* INC_HAL_ADC_H_ */
