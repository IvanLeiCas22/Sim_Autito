#include "nav_core.h"
#include "pid_controller.h"

enum {
    NAV_PWM_STOP = 0,
    NAV_PWM_FORWARD = 3000,
    NAV_PWM_SMOOTH_FAST_BASE = 3500,
    NAV_PWM_SMOOTH_SLOW_BASE = 1500,
    NAV_PWM_PIVOT_BASE = 1000,
    NAV_PWM_MIN = -9999,
    NAV_PWM_MAX = 9999,
    NAV_PID_OUTPUT_LIMIT_PWM = 4000,
    NAV_YAW_RATE_TARGET_ABS_DEG_S = 100,
    NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S = 100,
    NAV_SMOOTH_YAW_RATE_KP = 8,
    NAV_SMOOTH_YAW_RATE_KI_X100 = 50,
    Q16_90_DEG = 90 << 16,
    Q16_NEG_90_DEG = -(90 << 16),
    Q16_180_DEG = 180 << 16,
    Q16_TURN_TOLERANCE_DEG = 3 << 16
};

static NavState current_state = NAV_STATE_IDLE;
static NavAction current_action = NAV_ACTION_NONE;
static q16_16_t action_start_yaw_q16 = 0;
static q16_16_t action_target_yaw_q16 = 0;
static PID_Controller_t turn_yaw_rate_pid;

static q16_16_t abs_q16(q16_16_t value)
{
    return value < 0 ? -value : value;
}

static int16_t clamp_pwm(int32_t pwm)
{
    if (pwm > NAV_PWM_MAX) {
        return NAV_PWM_MAX;
    }
    if (pwm < NAV_PWM_MIN) {
        return NAV_PWM_MIN;
    }

    return (int16_t)pwm;
}

static RobotCommand smooth_turn_command(const RobotSensors *sensors, int direction)
{
    const int32_t pid_output_q16 = PID_Update_Fixed(&turn_yaw_rate_pid, sensors->yaw_rate_deg_s_q16, 10);
    const int32_t correction_pwm = FIXED_TO_INT(pid_output_q16);
    const int32_t base_left_pwm = direction > 0 ? NAV_PWM_SMOOTH_FAST_BASE : NAV_PWM_SMOOTH_SLOW_BASE;
    const int32_t base_right_pwm = direction > 0 ? NAV_PWM_SMOOTH_SLOW_BASE : NAV_PWM_SMOOTH_FAST_BASE;

    RobotCommand command = {
        clamp_pwm(base_left_pwm + correction_pwm),
        clamp_pwm(base_right_pwm - correction_pwm)
    };
    return command;
}

static RobotCommand pivot_turn_command(const RobotSensors *sensors, int direction)
{
    const int32_t pid_output_q16 = PID_Update_Fixed(&turn_yaw_rate_pid, sensors->yaw_rate_deg_s_q16, 10);
    const int32_t correction_pwm = FIXED_TO_INT(pid_output_q16);
    const int32_t base_left_pwm = direction * NAV_PWM_PIVOT_BASE;
    const int32_t base_right_pwm = -direction * NAV_PWM_PIVOT_BASE;

    RobotCommand command = {
        clamp_pwm(base_left_pwm + correction_pwm),
        clamp_pwm(base_right_pwm - correction_pwm)
    };
    return command;
}

void nav_core_init(void)
{
    current_state = NAV_STATE_IDLE;
    current_action = NAV_ACTION_NONE;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
    PID_Init(&turn_yaw_rate_pid,
             INT_TO_FIXED(NAV_SMOOTH_YAW_RATE_KP),
             HUNDREDTHS_TO_FIXED(NAV_SMOOTH_YAW_RATE_KI_X100),
             0);
    PID_Set_Output_Limits(&turn_yaw_rate_pid,
                          INT_TO_FIXED(-NAV_PID_OUTPUT_LIMIT_PWM),
                          INT_TO_FIXED(NAV_PID_OUTPUT_LIMIT_PWM));
}

void nav_core_start_advance_until_rear_black(void)
{
    current_state = NAV_STATE_ADVANCING_UNTIL_REAR_BLACK;
    current_action = NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
}

void nav_core_start_smooth_turn_left(const RobotSensors *sensors)
{
    if (sensors == 0) {
        current_state = NAV_STATE_IDLE;
        current_action = NAV_ACTION_NONE;
        action_start_yaw_q16 = 0;
        action_target_yaw_q16 = 0;
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_NEG_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, -INT_TO_FIXED(NAV_YAW_RATE_TARGET_ABS_DEG_S));
    current_state = NAV_STATE_SMOOTH_TURNING;
    current_action = NAV_ACTION_SMOOTH_TURN_LEFT;
}

void nav_core_start_smooth_turn_right(const RobotSensors *sensors)
{
    if (sensors == 0) {
        current_state = NAV_STATE_IDLE;
        current_action = NAV_ACTION_NONE;
        action_start_yaw_q16 = 0;
        action_target_yaw_q16 = 0;
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, INT_TO_FIXED(NAV_YAW_RATE_TARGET_ABS_DEG_S));
    current_state = NAV_STATE_SMOOTH_TURNING;
    current_action = NAV_ACTION_SMOOTH_TURN_RIGHT;
}

void nav_core_start_pivot_turn_left(const RobotSensors *sensors)
{
    if (sensors == 0) {
        current_state = NAV_STATE_IDLE;
        current_action = NAV_ACTION_NONE;
        action_start_yaw_q16 = 0;
        action_target_yaw_q16 = 0;
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_NEG_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, -INT_TO_FIXED(NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S));
    current_state = NAV_STATE_PIVOT_TURNING;
    current_action = NAV_ACTION_PIVOT_TURN_LEFT;
}

void nav_core_start_pivot_turn_right(const RobotSensors *sensors)
{
    if (sensors == 0) {
        current_state = NAV_STATE_IDLE;
        current_action = NAV_ACTION_NONE;
        action_start_yaw_q16 = 0;
        action_target_yaw_q16 = 0;
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, INT_TO_FIXED(NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S));
    current_state = NAV_STATE_PIVOT_TURNING;
    current_action = NAV_ACTION_PIVOT_TURN_RIGHT;
}

void nav_core_start_pivot_turn_180(const RobotSensors *sensors)
{
    if (sensors == 0) {
        current_state = NAV_STATE_IDLE;
        current_action = NAV_ACTION_NONE;
        action_start_yaw_q16 = 0;
        action_target_yaw_q16 = 0;
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_180_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, INT_TO_FIXED(NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S));
    current_state = NAV_STATE_PIVOT_TURNING;
    current_action = NAV_ACTION_PIVOT_TURN_180;
}

void nav_core_stop(void)
{
    current_state = NAV_STATE_IDLE;
    current_action = NAV_ACTION_NONE;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
}

NavState nav_core_state(void)
{
    return current_state;
}

NavAction nav_core_action(void)
{
    return current_action;
}

q16_16_t nav_core_action_start_yaw_q16(void)
{
    return action_start_yaw_q16;
}

q16_16_t nav_core_action_target_yaw_q16(void)
{
    return action_target_yaw_q16;
}

RobotCommand nav_core_update(const RobotSensors *sensors)
{
    if (sensors == 0) {
        RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
        return command;
    }

    if (current_action == NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK) {
        if (sensors->floor_rear_black) {
            current_state = NAV_STATE_DONE;
            current_action = NAV_ACTION_NONE;
            action_start_yaw_q16 = 0;
            action_target_yaw_q16 = 0;
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        RobotCommand command = {NAV_PWM_FORWARD, NAV_PWM_FORWARD};
        return command;
    }

    if (current_action == NAV_ACTION_SMOOTH_TURN_RIGHT) {
        if (sensors->yaw_deg_q16 >= action_target_yaw_q16 - Q16_TURN_TOLERANCE_DEG) {
            current_state = NAV_STATE_DONE;
            current_action = NAV_ACTION_NONE;
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        return smooth_turn_command(sensors, 1);
    }

    if (current_action == NAV_ACTION_SMOOTH_TURN_LEFT) {
        if (sensors->yaw_deg_q16 <= action_target_yaw_q16 + Q16_TURN_TOLERANCE_DEG) {
            current_state = NAV_STATE_DONE;
            current_action = NAV_ACTION_NONE;
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        return smooth_turn_command(sensors, -1);
    }

    if (current_action == NAV_ACTION_PIVOT_TURN_RIGHT) {
        if (sensors->yaw_deg_q16 >= Q16_90_DEG - Q16_TURN_TOLERANCE_DEG) {
            current_state = NAV_STATE_DONE;
            current_action = NAV_ACTION_NONE;
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        return pivot_turn_command(sensors, 1);
    }

    if (current_action == NAV_ACTION_PIVOT_TURN_LEFT) {
        if (sensors->yaw_deg_q16 <= Q16_NEG_90_DEG + Q16_TURN_TOLERANCE_DEG) {
            current_state = NAV_STATE_DONE;
            current_action = NAV_ACTION_NONE;
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        return pivot_turn_command(sensors, -1);
    }

    if (current_action == NAV_ACTION_PIVOT_TURN_180) {
        if (abs_q16(sensors->yaw_deg_q16) >= Q16_180_DEG - Q16_TURN_TOLERANCE_DEG) {
            current_state = NAV_STATE_DONE;
            current_action = NAV_ACTION_NONE;
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        return pivot_turn_command(sensors, 1);
    }

    RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
    return command;
}
