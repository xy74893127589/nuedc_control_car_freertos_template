#include "chassis.h"
#include "bsp_motor.h"
#include "vehicle_calibration.h"
#ifndef NUEDC_NO_ENCODER
#include "bsp_encoder.h"
#endif
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CONTROL_CHASSIS_DEFAULT_LEFT_PULSES_PER_CM \
    VEHICLE_ENCODER_LEFT_PULSES_PER_CM
#define CONTROL_CHASSIS_DEFAULT_RIGHT_PULSES_PER_CM \
    VEHICLE_ENCODER_RIGHT_PULSES_PER_CM
#define CONTROL_CHASSIS_DEFAULT_WHEEL_BASE_CM      13.5f
#define CONTROL_CHASSIS_DEFAULT_OPEN_LOOP_CM_PWM   0.00022f
#define CONTROL_CHASSIS_DEFAULT_SPEED_OUT_LIMIT    VEHICLE_SPEED_PWM_LIMIT
#define CONTROL_CHASSIS_DEFAULT_CORRECTION_LIMIT   \
    VEHICLE_SPEED_PID_CORRECTION_LIMIT
#define CONTROL_CHASSIS_DEFAULT_FORWARD_MIN_PULSE  0.03f
#define CONTROL_CHASSIS_DEFAULT_LEFT_FF_SLOPE      \
    VEHICLE_SPEED_LEFT_FF_SLOPE
#define CONTROL_CHASSIS_DEFAULT_LEFT_FF_OFFSET     \
    VEHICLE_SPEED_LEFT_FF_OFFSET
#define CONTROL_CHASSIS_DEFAULT_RIGHT_FF_SLOPE     \
    VEHICLE_SPEED_RIGHT_FF_SLOPE
#define CONTROL_CHASSIS_DEFAULT_RIGHT_FF_OFFSET    \
    VEHICLE_SPEED_RIGHT_FF_OFFSET
#define CONTROL_CHASSIS_DEFAULT_INTEGRAL_DELAY_MS  \
    VEHICLE_SPEED_INTEGRAL_DELAY_MS

static float wrap_pi_local(float x)
{
    while (x > (float)M_PI)  x -= 2.0f * (float)M_PI;
    while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

#ifndef NUEDC_NO_ENCODER
static float feedforward_pwm(float target, float slope, float offset,
                             float minimum_target)
{
    if (target <= minimum_target || slope <= 0.0f) {
        return 0.0f;
    }
    return (target + offset) / slope;
}

static float update_speed_pid(PID_t *pid, float target, float measured,
                              uint8_t allow_integral)
{
    float ki;
    float output;

    if (allow_integral) {
        return PID_Update(pid, target, measured);
    }

    ki = pid->ki;
    pid->ki = 0.0f;
    output = PID_Update(pid, target, measured);
    pid->ki = ki;
    pid->integral = 0.0f;
    pid->last_i = 0.0f;
    return output;
}
#endif

static void normalize_config(ControlChassisConfig_t *config)
{
    if (config->left_pulses_per_cm <= 0.0f) {
        config->left_pulses_per_cm =
            CONTROL_CHASSIS_DEFAULT_LEFT_PULSES_PER_CM;
    }
    if (config->right_pulses_per_cm <= 0.0f) {
        config->right_pulses_per_cm =
            CONTROL_CHASSIS_DEFAULT_RIGHT_PULSES_PER_CM;
    }
    if (config->wheel_base_cm <= 0.0f) {
        config->wheel_base_cm = CONTROL_CHASSIS_DEFAULT_WHEEL_BASE_CM;
    }
    if (config->open_loop_cm_per_pwm_ms <= 0.0f) {
        config->open_loop_cm_per_pwm_ms = CONTROL_CHASSIS_DEFAULT_OPEN_LOOP_CM_PWM;
    }
    if (config->speed_out_limit <= 0.0f) {
        config->speed_out_limit = CONTROL_CHASSIS_DEFAULT_SPEED_OUT_LIMIT;
    }
    if (config->speed_correction_limit <= 0.0f) {
        config->speed_correction_limit =
            CONTROL_CHASSIS_DEFAULT_CORRECTION_LIMIT;
    }
    if (config->forward_min_pulse < 0.0f) {
        config->forward_min_pulse = CONTROL_CHASSIS_DEFAULT_FORWARD_MIN_PULSE;
    }
    if (config->feedforward_left_slope <= 0.0f) {
        config->feedforward_left_slope =
            CONTROL_CHASSIS_DEFAULT_LEFT_FF_SLOPE;
    }
    if (config->feedforward_left_offset <= 0.0f) {
        config->feedforward_left_offset =
            CONTROL_CHASSIS_DEFAULT_LEFT_FF_OFFSET;
    }
    if (config->feedforward_right_slope <= 0.0f) {
        config->feedforward_right_slope =
            CONTROL_CHASSIS_DEFAULT_RIGHT_FF_SLOPE;
    }
    if (config->feedforward_right_offset <= 0.0f) {
        config->feedforward_right_offset =
            CONTROL_CHASSIS_DEFAULT_RIGHT_FF_OFFSET;
    }
    if (config->integral_delay_ms == 0u) {
        config->integral_delay_ms =
            CONTROL_CHASSIS_DEFAULT_INTEGRAL_DELAY_MS;
    }
}

static void integrate_odometry(ControlChassis_t *chassis, float ds_left_cm, float ds_right_cm)
{
    float ds = 0.5f * (ds_left_cm + ds_right_cm);
    float dtheta = (ds_right_cm - ds_left_cm) / chassis->config.wheel_base_cm;
    float mid = chassis->pose.theta + 0.5f * dtheta;

    chassis->pose.x += ds * cosf(mid);
    chassis->pose.y += ds * sinf(mid);
    chassis->pose.theta = wrap_pi_local(chassis->pose.theta + dtheta);
    chassis->segment_dist_cm += fabsf(ds);
}

void Control_Chassis_Init(ControlChassis_t *chassis,
                          const ControlChassisConfig_t *config,
                          float speed_kp, float speed_ki, float speed_kd)
{
    ControlChassisConfig_t cfg;

    if (chassis == 0) {
        return;
    }

    memset(chassis, 0, sizeof(*chassis));
    if (config != 0) {
        cfg = *config;
    } else {
        memset(&cfg, 0, sizeof(cfg));
    }
    normalize_config(&cfg);
    chassis->config = cfg;

    PID_Init(&chassis->pid_left, speed_kp, speed_ki, speed_kd,
             -chassis->config.speed_correction_limit,
             chassis->config.speed_correction_limit);
    PID_Init(&chassis->pid_right, speed_kp, speed_ki, speed_kd,
             -chassis->config.speed_correction_limit,
             chassis->config.speed_correction_limit);
#ifndef NUEDC_NO_ENCODER
    BSP_Encoder_Init();
#endif
}

void Control_Chassis_PollFastInputs(ControlChassis_t *chassis)
{
    (void)chassis;
#ifndef NUEDC_NO_ENCODER
    BSP_Encoder_Poll();
#endif
}

void Control_Chassis_ResetOdometry(ControlChassis_t *chassis, ControlPose2D_t pose)
{
    if (chassis == 0) {
        return;
    }

    chassis->pose = pose;
    chassis->segment_dist_cm = 0.0f;
#ifndef NUEDC_NO_ENCODER
    BSP_Encoder_Reset(0);
    BSP_Encoder_Reset(1);
#endif
}

void Control_Chassis_ResetSegmentDistance(ControlChassis_t *chassis)
{
    if (chassis != 0) {
        chassis->segment_dist_cm = 0.0f;
    }
}

void Control_Chassis_SetTargetSpeeds(ControlChassis_t *chassis, float left, float right)
{
    if (chassis == 0) {
        return;
    }

    chassis->target_speed_left = left;
    chassis->target_speed_right = right;
}

void Control_Chassis_SetDeadZone(ControlChassis_t *chassis, float left, float right)
{
    if (chassis == 0) {
        return;
    }

    chassis->dead_zone_left = left;
    chassis->dead_zone_right = right;
}

void Control_Chassis_Tick(ControlChassis_t *chassis, uint8_t run, uint32_t now_ms)
{
    float limit;

    if (chassis == 0) {
        return;
    }

    limit = chassis->config.speed_out_limit;

#ifdef NUEDC_NO_ENCODER
    int16_t pwm_left = 0;
    int16_t pwm_right = 0;

    if (run) {
        pwm_left = (int16_t)clampf_local(chassis->target_speed_left, -limit, limit);
        pwm_right = (int16_t)clampf_local(chassis->target_speed_right, -limit, limit);
    }

    integrate_odometry(chassis,
                       (float)pwm_left * chassis->config.open_loop_cm_per_pwm_ms,
                       (float)pwm_right * chassis->config.open_loop_cm_per_pwm_ms);
    chassis->measured_speed_left = (float)pwm_left;
    chassis->measured_speed_right = (float)pwm_right;
    BSP_Motor_SetDuty(0, pwm_left);
    BSP_Motor_SetDuty(1, pwm_right);
#else
    int32_t delta_left = BSP_Encoder_GetDelta(0);
    int32_t delta_right = BSP_Encoder_GetDelta(1);
    float meas_left = fabsf((float)delta_left);
    float meas_right = fabsf((float)delta_right);
    int16_t pwm_left = 0;
    int16_t pwm_right = 0;

    integrate_odometry(chassis,
                       meas_left / chassis->config.left_pulses_per_cm,
                       meas_right / chassis->config.right_pulses_per_cm);

    chassis->measured_speed_left = meas_left;
    chassis->measured_speed_right = meas_right;

    if (run) {
        float ff_left;
        float ff_right;
        float correction_left = 0.0f;
        float correction_right = 0.0f;
        uint8_t allow_integral;

        if (!chassis->speed_control_active) {
            chassis->speed_control_active = 1u;
            chassis->speed_control_started_ms = now_ms;
            PID_Reset(&chassis->pid_left);
            PID_Reset(&chassis->pid_right);
        }

        allow_integral =
            ((uint32_t)(now_ms - chassis->speed_control_started_ms) >=
             chassis->config.integral_delay_ms) ? 1u : 0u;
        ff_left = feedforward_pwm(chassis->target_speed_left,
                                  chassis->config.feedforward_left_slope,
                                  chassis->config.feedforward_left_offset,
                                  chassis->config.forward_min_pulse);
        ff_right = feedforward_pwm(chassis->target_speed_right,
                                   chassis->config.feedforward_right_slope,
                                   chassis->config.feedforward_right_offset,
                                   chassis->config.forward_min_pulse);

        if (ff_left > 0.0f) {
            correction_left = update_speed_pid(&chassis->pid_left,
                                               chassis->target_speed_left,
                                               meas_left, allow_integral);
        } else {
            PID_Reset(&chassis->pid_left);
        }
        if (ff_right > 0.0f) {
            correction_right = update_speed_pid(&chassis->pid_right,
                                                chassis->target_speed_right,
                                                meas_right, allow_integral);
        } else {
            PID_Reset(&chassis->pid_right);
        }

        pwm_left = (int16_t)(clampf_local(
            ff_left + correction_left + chassis->dead_zone_left,
            0.0f, limit) + 0.5f);
        pwm_right = (int16_t)(clampf_local(
            ff_right + correction_right + chassis->dead_zone_right,
            0.0f, limit) + 0.5f);
        BSP_Motor_SetDuty(0, pwm_left);
        BSP_Motor_SetDuty(1, pwm_right);
    } else if (chassis->debug_pwm_active) {
        chassis->speed_control_active = 0u;
        PID_Reset(&chassis->pid_left);
        PID_Reset(&chassis->pid_right);
        if (now_ms >= chassis->debug_pwm_deadline_ms) {
            Control_Chassis_ClearDebugDuty(chassis);
            BSP_Motor_SetDuty(0, 0);
            BSP_Motor_SetDuty(1, 0);
        } else {
            BSP_Motor_SetDuty(0, chassis->debug_pwm_left);
            BSP_Motor_SetDuty(1, chassis->debug_pwm_right);
        }
    } else {
        chassis->speed_control_active = 0u;
        PID_Reset(&chassis->pid_left);
        PID_Reset(&chassis->pid_right);
        BSP_Motor_SetDuty(0, 0);
        BSP_Motor_SetDuty(1, 0);
    }
#endif
}

void Control_Chassis_SetDebugDuty(ControlChassis_t *chassis,
                                  int16_t left_pwm, int16_t right_pwm,
                                  uint32_t now_ms, uint32_t hold_ms)
{
    if (chassis == 0) {
        return;
    }

    if (hold_ms == 0u) {
        hold_ms = 800u;
    }

    chassis->debug_pwm_active = 1u;
    chassis->debug_pwm_left = left_pwm;
    chassis->debug_pwm_right = right_pwm;
    chassis->debug_pwm_deadline_ms = now_ms + hold_ms;
}

void Control_Chassis_ClearDebugDuty(ControlChassis_t *chassis)
{
    if (chassis == 0) {
        return;
    }

    chassis->debug_pwm_active = 0u;
    chassis->debug_pwm_left = 0;
    chassis->debug_pwm_right = 0;
    chassis->debug_pwm_deadline_ms = 0u;
}
