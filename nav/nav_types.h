#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t q16_16_t;

typedef struct RobotSensors {
    q16_16_t ir_front_left_mm_q16;
    q16_16_t ir_front_right_mm_q16;
    q16_16_t ir_left_mm_q16;
    q16_16_t ir_right_mm_q16;
    q16_16_t ir_diag_left_mm_q16;
    q16_16_t ir_diag_right_mm_q16;
    q16_16_t yaw_deg_q16;
    q16_16_t yaw_rate_deg_s_q16;
    bool floor_front_black;
    bool floor_rear_black;
} RobotSensors;

typedef struct RobotCommand {
    int16_t left_motor_pwm;
    int16_t right_motor_pwm;
} RobotCommand;

#ifdef __cplusplus
}
#endif

#endif // NAV_TYPES_H
