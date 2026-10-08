/*
 * SolarPower_Config.h
 *
 * Solar panel voltage (Vsolar) measurement parameters.
 *
 * This board measures panel VOLTAGE only. There is no shunt and no current,
 * power or coulomb measurement: Vsolar exists to decide when the panel is
 * illuminated enough to leave ProductionSleep.
 */

#ifndef WORKER_SOLARPOWER_SOLARPOWER_CONFIG_H_
#define WORKER_SOLARPOWER_SOLARPOWER_CONFIG_H_

#include "build_config.h"   /* ENABLE_SOLAR_PERIODIC_LOG */

/* Sample interval, in 1 Hz heartbeats (s): SolarSchedTask requests one
 * measurement every SOLAR_SAMPLE_INTERVAL heartbeats. */
#define SOLAR_SAMPLE_INTERVAL           10

/* ADC full-scale code (12-bit) */
#define SOLAR_ADC_FULL_SCALE            4095U

/* VSOLAR resistor divider: 33k top / 10k bottom → (33+10)/10 = 4.3.
 *   Vpin   = adc_vsolar * VDDA_measured / SOLAR_ADC_FULL_SCALE
 *   Vsolar = Vpin * SOLAR_VSOLAR_DIV_NUM / SOLAR_VSOLAR_DIV_DEN
 * VDDA comes from the calibrated VREFINT channel (same as the battery worker),
 * so the result does not assume the rail is exactly its nominal 2.0 V
 * (Tagskaap V3 RevA). Full scale is VDDA x 4.3 = 8.6 V at 2.0 V. */
#define SOLAR_VSOLAR_DIV_NUM            43U
#define SOLAR_VSOLAR_DIV_DEN            10U

/* Minimum Vsolar (mV) that counts as "panel illuminated / charging" and so
 * ends ProductionSleep.
 * NOTE: tune to this board's behaviour — Vsolar tracks the charge rail, so it
 * sits near the battery voltage and rises when the panel actively drives it.
 * Both that description and the 3000 mV value were carried over from frtag2
 * and have not been re-checked on Tagskaap V3 RevA. */
#define SOLAR_ACTIVATION_VSOLAR_MV      3000U

#endif /* WORKER_SOLARPOWER_SOLARPOWER_CONFIG_H_ */
