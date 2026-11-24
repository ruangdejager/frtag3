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

#ifdef ENABLE_MOVEMENT_SENSOR_DATA_API
int8_t MOVE_i8SensorDataX(void);
int8_t MOVE_i8SensorDataY(void);
int8_t MOVE_i8SensorDataZ(void);
#endif

#endif /* WORKER_MOVEMENT_MOVEMENT_H_ */
