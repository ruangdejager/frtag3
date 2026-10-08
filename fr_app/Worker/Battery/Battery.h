/*
 * Battery.h
 *
 * Battery voltage measurement — public interface.
 */

#ifndef WORKER_BATTERY_BATTERY_H_
#define WORKER_BATTERY_BATTERY_H_

#include <stdbool.h>
#include <stdint.h>

void     BAT_vInit(void);
/* Request an immediate sample that refills the whole averaging buffer; a
 * failed sample defers the refill to the next valid one. */
void     BAT_vPurgeBuffer(void);
/* Averaged battery voltage in mV. Returns 0 until the first valid sample after
 * boot: check BAT_bHasValidSample() before acting on it. Lock-free. */
uint16_t BAT_u16GetVoltage(void);
/* True once a valid sample has seeded the average; never goes false again. */
bool     BAT_bHasValidSample(void);
/* Invalid samples (EOC timeout or implausible VDDA) since the last call,
 * saturating at UINT16_MAX; the count is cleared by the read. Task context. */
uint16_t BAT_u16GetAndClearSampleFails(void);
uint8_t  BAT_u8GetPercentage(void);
uint8_t  BAT_u8ConvertVoltageToPercentage(uint16_t u16Voltage);

#endif /* WORKER_BATTERY_BATTERY_H_ */
