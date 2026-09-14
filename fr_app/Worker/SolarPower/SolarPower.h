/*
 * SolarPower.h
 *
 * Solar panel voltage measurement — public interface.
 *
 * VSOLAR is sampled once every 10 seconds and read back with
 * SOLAR_u16GetVSolarMV(). Panel current and power are not measured on this
 * board; the voltage alone decides when to leave ProductionSleep.
 */

#ifndef WORKER_SOLARPOWER_SOLARPOWER_H_
#define WORKER_SOLARPOWER_SOLARPOWER_H_

#include <stdint.h>

#include "SolarPower_Config.h"

void     SOLAR_vInit(void);

uint16_t SOLAR_u16GetVSolarMV(void);    /* last solar panel voltage, mV          */


#endif /* WORKER_SOLARPOWER_SOLARPOWER_H_ */
