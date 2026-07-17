#ifndef VEHICLE_CALIBRATION_H
#define VEHICLE_CALIBRATION_H

/* Physical calibration verified on the assembled car on 2026-07-11. */
#define VEHICLE_ENCODER_LEFT_PULSES_PER_CM       73.44f
#define VEHICLE_ENCODER_RIGHT_PULSES_PER_CM      73.14f

/* Straight-line calibration at nominal battery voltage. */
#define VEHICLE_STRAIGHT_PWM_LEFT                300
#define VEHICLE_STRAIGHT_PWM_RIGHT               300
#define VEHICLE_STRAIGHT_SYNC_KP                 0.08f
#define VEHICLE_STRAIGHT_SYNC_DEADBAND_PULSES    3
#define VEHICLE_STRAIGHT_SYNC_MAX_PWM            15
#define VEHICLE_STRAIGHT_DECEL_WINDOW_PULSES     900u
#define VEHICLE_STRAIGHT_MIN_PWM                 220
#define VEHICLE_STRAIGHT_RAMP_MS                 500u

/*
 * Wheel-speed baseline measured on the ground at regulated 12 V on
 * 2026-07-13. Revalidate the feedforward model with the final load and course
 * surface; encoder feedback is responsible for the remaining load error.
 */
#define VEHICLE_SPEED_CONTROL_PERIOD_MS           10u
#define VEHICLE_SPEED_PID_KP                       8.0f
#define VEHICLE_SPEED_PID_KI                       0.1f
#define VEHICLE_SPEED_PID_KD                       0.0f
#define VEHICLE_SPEED_PID_CORRECTION_LIMIT       120.0f
#define VEHICLE_SPEED_INTEGRAL_DELAY_MS           200u
#define VEHICLE_SPEED_PWM_LIMIT                   500.0f
#define VEHICLE_SPEED_LEFT_FF_SLOPE                0.069114f
#define VEHICLE_SPEED_LEFT_FF_OFFSET               5.751407f
#define VEHICLE_SPEED_RIGHT_FF_SLOPE               0.068201f
#define VEHICLE_SPEED_RIGHT_FF_OFFSET              5.486958f

/* Provisional competition speed bands; the high band still needs load tests. */
#define VEHICLE_SPEED_LOW_CM_S                     20.0f
#define VEHICLE_SPEED_MEDIUM_CM_S                  30.0f
#define VEHICLE_SPEED_HIGH_CM_S                    34.0f
#define VEHICLE_SPEED_MAX_TARGET_PULSES            25.0f

/* Stationary 90-degree yaw turn calibration. */
#define VEHICLE_TURN_ANGLE_DEG                    90.0f
#define VEHICLE_TURN_PRECISION_DEG                0.2f
#define VEHICLE_TURN_TIMEOUT_MS                   12000u
#define VEHICLE_TURN_FINE_MAX_ATTEMPTS            20u
#define VEHICLE_TURN_PWM_MAX                      300
#define VEHICLE_TURN_PWM_MIN                      220
#define VEHICLE_TURN_KP                           4.0f
#define VEHICLE_TURN_APPROACH_WINDOW_DEG          20.0f
#define VEHICLE_TURN_PWM_APPROACH_MIN             160
#define VEHICLE_TURN_FINE_ENTRY_DEG               3.0f
#define VEHICLE_TURN_SETTLE_RATE_DPS              1.0f
#define VEHICLE_TURN_SETTLE_SAMPLES               15u
#define VEHICLE_TURN_BRAKE_MIN_MS                 120u
#define VEHICLE_TURN_FINE_PWM_INITIAL             180
#define VEHICLE_TURN_FINE_PWM_MIN                 140
#define VEHICLE_TURN_FINE_PWM_MAX                 260
#define VEHICLE_TURN_FINE_PWM_STEP                20
#define VEHICLE_TURN_FINE_PULSE_MS                20u
#define VEHICLE_TURN_FINE_PROGRESS_MIN_DEG        0.05f
#define VEHICLE_TURN_DIRECTION_CHECK_MS           600u
#define VEHICLE_TURN_DIRECTION_PROGRESS_DEG       2.0f

#endif /* VEHICLE_CALIBRATION_H */
