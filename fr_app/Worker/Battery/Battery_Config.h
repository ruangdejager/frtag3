/*
 * Battery_Config.h
 *
 * Battery measurement parameters.
 */

#ifndef WORKER_BATTERY_BATTERY_CONFIG_H_
#define WORKER_BATTERY_BATTERY_CONFIG_H_

/* Settling time (ms) from raising PA4 BAT_MEAS_BIAS (the divider's load
 * switch, active high) to the first conversion. 40 ms is what BatSampleTask
 * has always used: it was a literal osDelay(40) while this constant said 20
 * and was never referenced. It covers load-switch turn-on plus settling of
 * the divider node, and has not been re-characterised on Tagskaap V3 RevA.
 * The ADC lock and the sleep lock are held for the whole delay. */
#define BAT_MEAS_BIAS_SETTLE_TIME   40

/* Bounded wait (ms) for the ADC to be found disabled at the start of a
 * battery sample. Every ADC user disables the ADC before HAL_ADC_vUnlock(), so
 * this only matters after a disable that did not complete; once the wait runs
 * out BatSampleTask forces the disable itself. Same 3 ms the code has always
 * allowed. */
#define BAT_ADC_IDLE_WAIT_MS        3U

/* Averaging factor */
#define BAT_AVG_FACTOR              8

/* Sample interval, in 1 Hz heartbeats (s): BatSchedTask requests one
 * delta-limited sample every BAT_SAMPLE_INTERVAL heartbeats. */
#define BAT_SAMPLE_INTERVAL         10

/* Max fall (mV) a delta-limited sample may show below the previous one; a
 * larger drop is clamped to previous - BAT_SAMPLE_VALUE_DELTA_MAX. Rises are
 * not limited. Integer mV: this used to be the float 20.f, which pulled
 * soft-float calls into BatSampleTask on this FPU-less core for an identical
 * result. */
#define BAT_SAMPLE_VALUE_DELTA_MAX  20U

/* ADC full-scale code (12-bit) */
#define BAT_ADC_FULL_SCALE          4095U

/* Resistor-divider ratio on the battery sense input (PA12):
 * (R_top + R_bottom)/R_bottom = (33k + 10k)/10k = 4.30. This file used to say
 * 330k/100k, which is the same ratio; the fitted parts are 33k top and 10k
 * bottom (see hal_bsp.h), so the source impedance seen by the ADC is
 * 33k || 10k = ~7.7 kOhm. Battery voltage is reconstructed as:
 *   Vbatt = adc_bat * VDDA_measured / BAT_ADC_FULL_SCALE * BAT_DIVIDER_NUM/DEN
 * VDDA_measured comes from the calibrated VREFINT channel, so the result does
 * not depend on the rail being exactly its nominal 2.0 V (VDD = VDDA on
 * Tagskaap V3 RevA). Full scale is VDDA x 4.3 = 8.6 V at 2.0 V; a 4.2 V cell
 * puts ~0.98 V on PA12 (~2000 counts). The 4.300 ratio was confirmed on frtag2
 * hardware against a calibrated reference (true 4050 mV / measured pin
 * 941.9 mV = 4.300). */
#define BAT_DIVIDER_NUM             4300U
#define BAT_DIVIDER_DEN             1000U

#endif /* WORKER_BATTERY_BATTERY_CONFIG_H_ */
