# Vehicle Calibration

The single source of truth for measured vehicle constants is
`config/vehicle_calibration.h`. Hardware tests and the production chassis use
the same definitions.

## Verified Values

| Item | Value | Verification |
| --- | ---: | --- |
| Left encoder | 73.44 pulse/cm | 100 cm straight trim |
| Right encoder | 73.14 pulse/cm | 99.8 cm measured over 100 cm |
| Straight PWM | left 300, right 300 | physical straight run |
| Encoder synchronization | Kp 0.08, deadband 3 pulses, limit 15 PWM | physical straight run |
| Distance deceleration | final 900 pulses, minimum PWM 220 | physical 100 cm run |
| Wheel-speed PID | Kp 8.0, Ki 0.1, Kd 0.0; integral enabled after 200 ms | 12 V ground runs at low/mid/high targets |
| Left wheel feedforward | `(target + 5.751407) / 0.069114` PWM | five-pass 12 V ground fit |
| Right wheel feedforward | `(target + 5.486958) / 0.068201` PWM | five-pass 12 V ground fit |
| Speed bands | 20, 30, 34 cm/s | low/medium/high ground validation; high is provisional |
| Stationary yaw turn | 90 degrees, 0.2 degree final tolerance | CW and CCW button test |
| Grayscale order | PB20, PB24, PB25, PA24, PA25, PA26, PA27 | left-to-right sweep |
| Grayscale type | seven digital channels | transition test |
| Motor polarity | both positive commands move forward | wheel test |
| ICM45688 | calibrated profile in `modules/imu/imu_profile.c` | OLED yaw and turn tests |

## Template-to-Calibrated Comparison

The clean GitHub template baseline is commit `8650fd4`. The table below lists
the production-control values changed after the 2026-07-13 calibration runs.

| Item | Clean template | Calibrated template |
| --- | ---: | ---: |
| Wheel-speed control | PID output only | per-wheel feedforward plus independent PID correction |
| Wheel-speed PID | Kp 18.0, Ki 0.0, Kd 0.0 | Kp 8.0, Ki 0.1, Kd 0.0 |
| Integral startup | integral available immediately | integral held at zero for first 200 ms |
| Default forward target | 80 pulse/10 ms | 20 cm/s: left 14.688, right 14.628 pulse/10 ms |
| Left feedforward | none | `(target + 5.751407) / 0.069114` PWM |
| Right feedforward | none | `(target + 5.486958) / 0.068201` PWM |
| Production speed PWM cap | 1000 | 500 |
| Competition speed bands | not defined | 20, 30 and provisional 34 cm/s |

The wheel-speed PID and per-wheel feedforward model are the current 12 V
baseline. They still require revalidation with the final competition load,
surface and battery-voltage range. The line-tracking PID remains an initial
template value and should be tuned after those conditions are fixed.

## Control Use

Production odometry converts each encoder with its own pulse-per-centimeter
value. Equal physical straight speed therefore uses a slightly higher left
pulse target than right pulse target. The production speed controller adds
the calibrated per-wheel feedforward PWM to an independently closed PID
correction and suppresses integral accumulation for the first 200 ms after a
run starts. The stationary yaw turn constants remain
available for precise stop-and-turn actions; a continuous racing turn should
use a separate non-stopping transition controller.
