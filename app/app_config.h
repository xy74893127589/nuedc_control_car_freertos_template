#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "vehicle_calibration.h"

/* Task periods. Keep fast sampling short; put blocking work in telemetry/tools. */
#define APP_FAST_TASK_PERIOD_MS       5u
#define APP_CONTROL_TASK_PERIOD_MS    VEHICLE_SPEED_CONTROL_PERIOD_MS
#define APP_TELEM_TASK_PERIOD_MS      10u

/* Chassis values verified on the assembled car. */
#define APP_LEFT_PULSES_PER_CM        VEHICLE_ENCODER_LEFT_PULSES_PER_CM
#define APP_RIGHT_PULSES_PER_CM       VEHICLE_ENCODER_RIGHT_PULSES_PER_CM
#define APP_WHEEL_BASE_CM             13.5f
#define APP_OPEN_LOOP_CM_PER_PWM_MS   0.00022f
#define APP_SPEED_OUT_LIMIT           VEHICLE_SPEED_PWM_LIMIT
#define APP_SPEED_CORRECTION_LIMIT    VEHICLE_SPEED_PID_CORRECTION_LIMIT
#define APP_FORWARD_MIN_PULSE         0.03f
#define APP_SPEED_LEFT_FF_SLOPE       VEHICLE_SPEED_LEFT_FF_SLOPE
#define APP_SPEED_LEFT_FF_OFFSET      VEHICLE_SPEED_LEFT_FF_OFFSET
#define APP_SPEED_RIGHT_FF_SLOPE      VEHICLE_SPEED_RIGHT_FF_SLOPE
#define APP_SPEED_RIGHT_FF_OFFSET     VEHICLE_SPEED_RIGHT_FF_OFFSET
#define APP_SPEED_INTEGRAL_DELAY_MS   VEHICLE_SPEED_INTEGRAL_DELAY_MS

#define APP_SPEED_PID_KP              VEHICLE_SPEED_PID_KP
#define APP_SPEED_PID_KI              VEHICLE_SPEED_PID_KI
#define APP_SPEED_PID_KD              VEHICLE_SPEED_PID_KD

#define APP_LINE_PID_KP               0.18f
#define APP_LINE_PID_KI               0.0f
#define APP_LINE_PID_KD               0.02f

/* Low competition band validated by the 2026-07-13 wheel-speed tests. */
#define APP_BASE_SPEED_CM_S           VEHICLE_SPEED_LOW_CM_S
#define APP_LINE_TURN_LIMIT           40.0f

#endif /* APP_CONFIG_H */
