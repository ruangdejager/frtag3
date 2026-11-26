/*
 * Movement.h
 *
 * Movement worker — public interface.
 * Runs as an RTOS task subscribed to the 1 Hz platform heartbeat. Reports a
 * MOVING/STILL state (STILL after MOVE_STILL_TIMEOUT_S of no motion) and a
 * daily count of high-activity triggers, both carried in the beacon.
 */

#ifndef WORKER_MOVEMENT_MOVEMENT_H_
#define WORKER_MOVEMENT_MOVEMENT_H_

#include <stdint.h>
#include <stdbool.h>
#include "build_config.h"   /* ENABLE_MOVEMENT_SENSOR_DATA_API */
#include "Movement_Config.h"

/* ---- Motion state ---- */
typedef enum {
    MOVE_STATE_MOVING = 0,
    MOVE_STATE_STILL  = 1,
} MoveState_e;

/* ---- movementAlarm settings, per day-time zone (fr9 move.c's
 *      move_alarm_setting_t, minus the unported no-activity pair) ---- */
typedef enum {
    MOVE_ALARM_SETTING_MAX_LEVEL = 0,
    MOVE_ALARM_SETTING_MAX_WINDOW,
    MOVE_ALARM_SETTING_FIRST_HOLD,
    MOVE_ALARM_SETTING_SECOND_HOLD,
} MoveAlarmSetting_e;

/* ---- Public API ---- */
void        MOVE_vInit(void);
MoveState_e MOVE_eGetState(void);

/* High-activity triggers counted since local midnight (the fr9 move.c trigger
 * strategy; held-back triggers are not counted). Saturates at 0xFFFF, cleared
 * when the local date changes, RAM only. Always 0 on a primary. Carried in
 * every beacon this node originates. See Movement.c. */
uint16_t    MOVE_u16GetDailyTriggerCount(void);

/* Current value of a movementAlarm setting for the day-time zone in force now. */
uint8_t     MOVE_u8GetAlarmSettingNow(MoveAlarmSetting_e eSetting);

/* Filtered-output stream (kernel "g [min|off]", UART only).
 *
 * Prints the algorithm's internal signals for every accelerometer sample, as
 * plain CSV on the debug UART, so the trigger thresholds can be tuned against
 * real animal movement. One line per sample, 25 lines/s:
 *
 *   MV,<n>,<dx>,<dy>,<dz>,<hp>,<env>,<win>
 *
 *   n           sample number since the stream started
 *   dx,dy,dz    DC level per axis: the 32-sample moving average, i.e. the
 *               gravity/orientation estimate (the shake-sequence detector
 *               reads these), in raw counts / MOVE_ACC_RAW_SCALE_VALUE
 *   hp          this sample's high-passed magnitude, |x-dx|+|y-dy|+|z-dz|
 *   env         envelope level: the 8-sample moving average of hp. This is
 *               the value compared with maxLevel.
 *   win         the max-level window counter after this sample: up one per
 *               sample at/above maxLevel (in an active zone), down one below.
 *               A trigger fires when it reaches maxWindow * 25, and it
 *               restarts at 0.
 *
 * Lines starting with '#' carry context: the column list when the stream
 * starts, and maxLevel / maxWindow / zone-active / in-service whenever they
 * change (the day-time zone can roll over mid-stream). Trigger and episode
 * log lines appear in place, between the samples that caused them. Other
 * debug output is not muted, so a parser should keep only "MV," lines.
 *
 * Start (or restart) for u16Minutes, held to MOVE_STREAM_MAX_MIN; 0 means
 * MOVE_STREAM_DEFAULT_MIN. Stop ends it at once. Both may be called from any
 * task; the Movement task does all the printing. A no-op on a primary, where
 * the Movement task does not run - the caller refuses it there. */
void        MOVE_vStartStream(uint16_t u16Minutes);
void        MOVE_vStopStream(void);
bool        MOVE_bStreamActive(void);

#ifdef ENABLE_MOVEMENT_SENSOR_DATA_API
int8_t MOVE_i8SensorDataX(void);
int8_t MOVE_i8SensorDataY(void);
int8_t MOVE_i8SensorDataZ(void);
#endif

#endif /* WORKER_MOVEMENT_MOVEMENT_H_ */
