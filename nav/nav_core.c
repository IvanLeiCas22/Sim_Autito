#include "nav_core.h"
#include "pid_controller.h"

enum {
    NAV_PWM_STOP = 0,
    NAV_PWM_FORWARD = 3000,
    NAV_ADVANCE_BASE_LEFT_PWM = 3000,
    NAV_ADVANCE_BASE_RIGHT_PWM = 3370,
    NAV_SMOOTH_POST_YAW_BASE_LEFT_PWM = 2500,
    NAV_SMOOTH_POST_YAW_BASE_RIGHT_PWM = 2800,
    NAV_SMOOTH_POST_YAW_TIMEOUT_MS = 800,
    NAV_SMOOTH_TARGET_YAW_RATE_DEFAULT_DEG_S = 120,
    NAV_SMOOTH_TARGET_YAW_RATE_MIN_DEG_S = 60,
    NAV_SMOOTH_TARGET_YAW_RATE_MAX_DEG_S = 120,
    NAV_SMOOTH_CENTER_EFFECTIVE_PWM = 2500,
    /* Smooth bases are derived for the simulated motor imbalance:
       leftMotorGain = 1.0 and rightMotorGain = 0.89. The target yaw rate
       maps to effective wheel PWM around a fixed center; recalibrate these
       model constants if the simulator motor gains change. */
    NAV_SMOOTH_DELTA_PWM_PER_DEG_S_NUM = 191,
    NAV_SMOOTH_DELTA_PWM_PER_DEG_S_DEN = 10,
    NAV_SMOOTH_LEFT_GAIN_PERMILLE = 1000,
    NAV_SMOOTH_RIGHT_GAIN_PERMILLE = 890,
    NAV_PWM_PIVOT_BASE = 1000,
    NAV_PWM_MIN = -9999,
    NAV_PWM_MAX = 9999,
    NAV_PID_OUTPUT_LIMIT_PWM = 4000,
    NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S = 100,
    NAV_SMOOTH_YAW_RATE_KP = 8,
    NAV_SMOOTH_YAW_RATE_KI_X100 = 50,
    NAV_ADVANCE_YAW_KP = 50,
    NAV_ADVANCE_YAW_KD = 2,
    NAV_ADVANCE_YAW_OUTPUT_LIMIT_PWM = 1200,
    NAV_ADVANCE_WALL_TARGET_LEFT_MM_DEFAULT = 60,
    NAV_ADVANCE_WALL_TARGET_RIGHT_MM_DEFAULT = 60,
    NAV_ADVANCE_WALL_KP_PWM_PER_MM_DEFAULT = 12,
    NAV_ADVANCE_WALL_KD_PWM_PER_MM_PER_TICK_DEFAULT = 600,
    NAV_ADVANCE_WALL_KP_PWM_PER_MM_MAX = 600,
    NAV_ADVANCE_WALL_KD_PWM_PER_MM_PER_TICK_MAX = 600,
    NAV_ADVANCE_WALL_OUTPUT_LIMIT_PWM_DEFAULT = 4000,
    NAV_ADVANCE_WALL_OUTPUT_LIMIT_PWM_MAX = 4000,
    NAV_ADVANCE_WALL_ERROR_DEADBAND_MM_DEFAULT = 0,
    NAV_WALL_FRONT_THRESHOLD_MM_Q16 = 140 << 16,
    NAV_WALL_SIDE_THRESHOLD_MM_Q16 = 100 << 16,
    NAV_WALL_DIAG_THRESHOLD_MM_Q16 = 145 << 16,
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
static PID_Controller_t advance_yaw_pid;
static NavTurnPidConfig turn_pid_config = {0};
static NavAdvanceYawPidConfig advance_yaw_pid_config = {0};
static NavSmoothPhase smooth_phase = NAV_SMOOTH_PHASE_NONE;
static NavAdvancePhase advance_phase = NAV_ADVANCE_PHASE_NONE;
static NavTurnDebug turn_debug = {0};
static NavSmoothTurnConfig smooth_turn_config = {0};
static NavWallPerception wall_perception = {0};
static uint16_t smooth_post_yaw_elapsed_ms = 0;
static NavAdvanceGuidanceMode advance_guidance_mode = NAV_ADVANCE_GUIDANCE_WALL_ASSIST;
static NavAdvanceWallConfig advance_wall_config = {
    NAV_ADVANCE_WALL_KP_PWM_PER_MM_DEFAULT,
    NAV_ADVANCE_WALL_KD_PWM_PER_MM_PER_TICK_DEFAULT,
    NAV_ADVANCE_WALL_OUTPUT_LIMIT_PWM_DEFAULT,
    NAV_ADVANCE_WALL_ERROR_DEADBAND_MM_DEFAULT,
    NAV_ADVANCE_WALL_TARGET_LEFT_MM_DEFAULT,
    NAV_ADVANCE_WALL_TARGET_RIGHT_MM_DEFAULT
};
static q16_16_t advance_wall_previous_error_q16 = 0;
static bool advance_wall_has_previous_error = false;

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

static int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }

    return (int16_t)value;
}

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }

    return value;
}

static int16_t clamp_wall_correction(int32_t correction_pwm)
{
    if (correction_pwm > advance_wall_config.correction_limit_pwm) {
        return advance_wall_config.correction_limit_pwm;
    }
    if (correction_pwm < -advance_wall_config.correction_limit_pwm) {
        return -advance_wall_config.correction_limit_pwm;
    }

    return (int16_t)correction_pwm;
}

static q16_16_t mm_to_q16(int16_t mm)
{
    return (q16_16_t)mm << 16;
}

static q16_16_t apply_wall_deadband(q16_16_t error_q16)
{
    const q16_16_t deadband_q16 = mm_to_q16(advance_wall_config.error_deadband_mm);
    if (abs_q16(error_q16) <= deadband_q16) {
        return 0;
    }

    return error_q16;
}

static int32_t q16_to_pwm(q16_16_t value_q16, int32_t pwm_per_mm)
{
    return (int32_t)(((int64_t)value_q16 * pwm_per_mm) / 65536);
}

static void reset_advance_wall_pd(void)
{
    advance_wall_previous_error_q16 = 0;
    advance_wall_has_previous_error = false;
}

static void reset_advance_wall_config(void)
{
    advance_wall_config.kp_pwm_per_mm = NAV_ADVANCE_WALL_KP_PWM_PER_MM_DEFAULT;
    advance_wall_config.kd_pwm_per_mm_per_tick = NAV_ADVANCE_WALL_KD_PWM_PER_MM_PER_TICK_DEFAULT;
    advance_wall_config.correction_limit_pwm = NAV_ADVANCE_WALL_OUTPUT_LIMIT_PWM_DEFAULT;
    advance_wall_config.error_deadband_mm = NAV_ADVANCE_WALL_ERROR_DEADBAND_MM_DEFAULT;
    advance_wall_config.target_left_mm = NAV_ADVANCE_WALL_TARGET_LEFT_MM_DEFAULT;
    advance_wall_config.target_right_mm = NAV_ADVANCE_WALL_TARGET_RIGHT_MM_DEFAULT;
}

static void set_turn_pid_defaults(void)
{
    turn_pid_config.kp_q16 = INT_TO_FIXED(NAV_SMOOTH_YAW_RATE_KP);
    turn_pid_config.ki_q16 = HUNDREDTHS_TO_FIXED(NAV_SMOOTH_YAW_RATE_KI_X100);
    turn_pid_config.kd_q16 = 0;
    turn_pid_config.output_limit_pwm = NAV_PID_OUTPUT_LIMIT_PWM;
}

static void apply_turn_pid_config(bool reset_state)
{
    PID_Config_t config = {
        turn_pid_config.kp_q16,
        turn_pid_config.ki_q16,
        turn_pid_config.kd_q16,
        -INT_TO_FIXED(turn_pid_config.output_limit_pwm),
        INT_TO_FIXED(turn_pid_config.output_limit_pwm)
    };
    PID_ApplyConfig(&turn_yaw_rate_pid, &config, reset_state);
}

static void set_advance_yaw_pid_defaults(void)
{
    advance_yaw_pid_config.kp_q16 = INT_TO_FIXED(NAV_ADVANCE_YAW_KP);
    advance_yaw_pid_config.ki_q16 = 0;
    advance_yaw_pid_config.kd_q16 = INT_TO_FIXED(NAV_ADVANCE_YAW_KD);
    advance_yaw_pid_config.output_limit_pwm = NAV_ADVANCE_YAW_OUTPUT_LIMIT_PWM;
}

static void apply_advance_yaw_pid_config(bool reset_state)
{
    PID_Config_t config = {
        advance_yaw_pid_config.kp_q16,
        advance_yaw_pid_config.ki_q16,
        advance_yaw_pid_config.kd_q16,
        -INT_TO_FIXED(advance_yaw_pid_config.output_limit_pwm),
        INT_TO_FIXED(advance_yaw_pid_config.output_limit_pwm)
    };
    PID_ApplyConfig(&advance_yaw_pid, &config, reset_state);
}

static void clear_wall_perception(void)
{
    wall_perception.wall_front = false;
    wall_perception.wall_left = false;
    wall_perception.wall_right = false;
    wall_perception.wall_diag_left = false;
    wall_perception.wall_diag_right = false;
    wall_perception.front_left_mm_q16 = 0;
    wall_perception.front_right_mm_q16 = 0;
    wall_perception.left_mm_q16 = 0;
    wall_perception.right_mm_q16 = 0;
    wall_perception.diag_left_mm_q16 = 0;
    wall_perception.diag_right_mm_q16 = 0;
    wall_perception.front_threshold_mm_q16 = NAV_WALL_FRONT_THRESHOLD_MM_Q16;
    wall_perception.side_threshold_mm_q16 = NAV_WALL_SIDE_THRESHOLD_MM_Q16;
    wall_perception.diag_threshold_mm_q16 = NAV_WALL_DIAG_THRESHOLD_MM_Q16;
}

static void update_wall_perception(const RobotSensors *sensors)
{
    if (sensors == 0) {
        clear_wall_perception();
        return;
    }

    wall_perception.front_left_mm_q16 = sensors->ir_front_left_mm_q16;
    wall_perception.front_right_mm_q16 = sensors->ir_front_right_mm_q16;
    wall_perception.left_mm_q16 = sensors->ir_left_mm_q16;
    wall_perception.right_mm_q16 = sensors->ir_right_mm_q16;
    wall_perception.diag_left_mm_q16 = sensors->ir_diag_left_mm_q16;
    wall_perception.diag_right_mm_q16 = sensors->ir_diag_right_mm_q16;
    wall_perception.front_threshold_mm_q16 = NAV_WALL_FRONT_THRESHOLD_MM_Q16;
    wall_perception.side_threshold_mm_q16 = NAV_WALL_SIDE_THRESHOLD_MM_Q16;
    wall_perception.diag_threshold_mm_q16 = NAV_WALL_DIAG_THRESHOLD_MM_Q16;
    wall_perception.wall_front =
        sensors->ir_front_left_mm_q16 < NAV_WALL_FRONT_THRESHOLD_MM_Q16
        || sensors->ir_front_right_mm_q16 < NAV_WALL_FRONT_THRESHOLD_MM_Q16;
    wall_perception.wall_left =
        sensors->ir_left_mm_q16 < NAV_WALL_SIDE_THRESHOLD_MM_Q16;
    wall_perception.wall_right =
        sensors->ir_right_mm_q16 < NAV_WALL_SIDE_THRESHOLD_MM_Q16;
    wall_perception.wall_diag_left =
        sensors->ir_diag_left_mm_q16 < NAV_WALL_DIAG_THRESHOLD_MM_Q16;
    wall_perception.wall_diag_right =
        sensors->ir_diag_right_mm_q16 < NAV_WALL_DIAG_THRESHOLD_MM_Q16;
}

static int32_t clamp_smooth_target_yaw_rate(int32_t target_yaw_rate_deg_s)
{
    if (target_yaw_rate_deg_s < NAV_SMOOTH_TARGET_YAW_RATE_MIN_DEG_S) {
        return NAV_SMOOTH_TARGET_YAW_RATE_MIN_DEG_S;
    }
    if (target_yaw_rate_deg_s > NAV_SMOOTH_TARGET_YAW_RATE_MAX_DEG_S) {
        return NAV_SMOOTH_TARGET_YAW_RATE_MAX_DEG_S;
    }

    return target_yaw_rate_deg_s;
}

static int16_t pwm_from_effective_pwm(int32_t effective_pwm, int32_t gain_permille)
{
    if (gain_permille <= 0) {
        return clamp_pwm(effective_pwm);
    }

    return clamp_pwm((effective_pwm * 1000 + gain_permille / 2) / gain_permille);
}

static void recalculate_smooth_turn_bases(void)
{
    const int32_t delta_effective_pwm =
        (smooth_turn_config.target_yaw_rate_deg_s * NAV_SMOOTH_DELTA_PWM_PER_DEG_S_NUM
         + NAV_SMOOTH_DELTA_PWM_PER_DEG_S_DEN / 2)
        / NAV_SMOOTH_DELTA_PWM_PER_DEG_S_DEN;
    const int32_t half_delta_effective_pwm = delta_effective_pwm / 2;
    const int32_t fast_effective_pwm =
        NAV_SMOOTH_CENTER_EFFECTIVE_PWM + half_delta_effective_pwm;
    const int32_t slow_effective_pwm =
        NAV_SMOOTH_CENTER_EFFECTIVE_PWM - half_delta_effective_pwm;

    smooth_turn_config.right_left_base_pwm =
        pwm_from_effective_pwm(fast_effective_pwm, NAV_SMOOTH_LEFT_GAIN_PERMILLE);
    smooth_turn_config.right_right_base_pwm =
        pwm_from_effective_pwm(slow_effective_pwm, NAV_SMOOTH_RIGHT_GAIN_PERMILLE);
    smooth_turn_config.left_left_base_pwm =
        pwm_from_effective_pwm(slow_effective_pwm, NAV_SMOOTH_LEFT_GAIN_PERMILLE);
    smooth_turn_config.left_right_base_pwm =
        pwm_from_effective_pwm(fast_effective_pwm, NAV_SMOOTH_RIGHT_GAIN_PERMILLE);
}

static void apply_current_smooth_target_to_pid(void)
{
    if (current_action == NAV_ACTION_SMOOTH_TURN_LEFT) {
        PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid,
                               -INT_TO_FIXED(smooth_turn_config.target_yaw_rate_deg_s));
    } else if (current_action == NAV_ACTION_SMOOTH_TURN_RIGHT) {
        PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid,
                               INT_TO_FIXED(smooth_turn_config.target_yaw_rate_deg_s));
    }
}

static void clear_live_turn_debug(void)
{
    turn_debug.yaw_rate_setpoint_deg_s_q16 = 0;
    turn_debug.yaw_rate_measured_deg_s_q16 = 0;
    turn_debug.yaw_rate_error_deg_s_q16 = 0;
    turn_debug.pid_output_q16 = 0;
    turn_debug.correction_pwm = 0;
    turn_debug.integral_q16 = 0;
    turn_debug.turn_kp_q16 = turn_pid_config.kp_q16;
    turn_debug.turn_ki_q16 = turn_pid_config.ki_q16;
    turn_debug.turn_kd_q16 = turn_pid_config.kd_q16;
    turn_debug.turn_output_limit_pwm = (int16_t)turn_pid_config.output_limit_pwm;
    turn_debug.smooth_left_base_pwm = 0;
    turn_debug.smooth_right_base_pwm = 0;
    turn_debug.smooth_phase = NAV_SMOOTH_PHASE_NONE;
    turn_debug.smooth_done_reason = NAV_SMOOTH_DONE_NONE;
    turn_debug.smooth_post_yaw_elapsed_ms = 0;
    turn_debug.advance_phase = NAV_ADVANCE_PHASE_NONE;
    turn_debug.advance_done_reason = NAV_ADVANCE_DONE_NONE;
    turn_debug.advance_yaw_setpoint_deg_q16 = 0;
    turn_debug.advance_yaw_measured_deg_q16 = 0;
    turn_debug.advance_yaw_error_deg_q16 = 0;
    turn_debug.advance_yaw_pid_output_q16 = 0;
    turn_debug.advance_yaw_correction_pwm = 0;
    turn_debug.advance_yaw_kp_q16 = advance_yaw_pid_config.kp_q16;
    turn_debug.advance_yaw_ki_q16 = advance_yaw_pid_config.ki_q16;
    turn_debug.advance_yaw_kd_q16 = advance_yaw_pid_config.kd_q16;
    turn_debug.advance_yaw_output_limit_pwm = (int16_t)advance_yaw_pid_config.output_limit_pwm;
    turn_debug.advance_base_left_pwm = 0;
    turn_debug.advance_base_right_pwm = 0;
    turn_debug.advance_guidance_mode = advance_guidance_mode;
    turn_debug.advance_final_correction_source = NAV_ADVANCE_CORRECTION_YAW_PD;
    turn_debug.advance_wall_left_valid = false;
    turn_debug.advance_wall_right_valid = false;
    turn_debug.advance_diag_left_valid = false;
    turn_debug.advance_diag_right_valid = false;
    turn_debug.advance_follow_left_valid = false;
    turn_debug.advance_follow_right_valid = false;
    turn_debug.advance_wall_left_mm_q16 = 0;
    turn_debug.advance_wall_right_mm_q16 = 0;
    turn_debug.advance_wall_error_mm_q16 = 0;
    turn_debug.advance_wall_error_after_deadband_mm_q16 = 0;
    turn_debug.advance_wall_prev_error_mm_q16 = 0;
    turn_debug.advance_wall_error_delta_mm_q16 = 0;
    turn_debug.advance_wall_p_term_pwm = 0;
    turn_debug.advance_wall_d_term_pwm = 0;
    turn_debug.advance_wall_raw_correction_pwm = 0;
    turn_debug.advance_wall_limited_correction_pwm = 0;
    turn_debug.advance_wall_correction_pwm = 0;
    turn_debug.wall_kp_pwm_per_mm = advance_wall_config.kp_pwm_per_mm;
    turn_debug.wall_kd_pwm_per_mm_per_tick = advance_wall_config.kd_pwm_per_mm_per_tick;
    turn_debug.wall_error_deadband_mm_q16 = mm_to_q16(advance_wall_config.error_deadband_mm);
    turn_debug.wall_follow_target_left_mm_q16 = mm_to_q16(advance_wall_config.target_left_mm);
    turn_debug.wall_follow_target_right_mm_q16 = mm_to_q16(advance_wall_config.target_right_mm);
    turn_debug.wall_correction_limit_pwm = advance_wall_config.correction_limit_pwm;
}

static void clear_completed_turn_debug(void)
{
    turn_debug.last_completed_action = NAV_ACTION_NONE;
    turn_debug.last_smooth_done_reason = NAV_SMOOTH_DONE_NONE;
    turn_debug.last_smooth_final_yaw_deg_q16 = 0;
    turn_debug.last_smooth_final_floor_rear_black = false;
    turn_debug.last_advance_done_reason = NAV_ADVANCE_DONE_NONE;
    turn_debug.last_advance_final_yaw_deg_q16 = 0;
    turn_debug.last_advance_final_floor_rear_black = false;
}

static void reset_turn_debug(void)
{
    clear_live_turn_debug();
    clear_completed_turn_debug();
}

static void set_smooth_phase(NavSmoothPhase phase)
{
    smooth_phase = phase;
    turn_debug.smooth_phase = phase;
    turn_debug.smooth_done_reason = NAV_SMOOTH_DONE_NONE;
    if (phase != NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE) {
        smooth_post_yaw_elapsed_ms = 0;
        turn_debug.smooth_post_yaw_elapsed_ms = 0;
    }
}

static void set_advance_phase(NavAdvancePhase phase)
{
    advance_phase = phase;
    turn_debug.advance_phase = phase;
    turn_debug.advance_done_reason = NAV_ADVANCE_DONE_NONE;
}

static RobotCommand finish_advance_until_rear_black(const RobotSensors *sensors)
{
    const NavAction completed_action = current_action;

    current_state = NAV_STATE_DONE;
    current_action = NAV_ACTION_NONE;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    reset_advance_wall_pd();
    clear_live_turn_debug();
    turn_debug.advance_done_reason = NAV_ADVANCE_DONE_REAR_SENSOR_TARGET_LINE;
    turn_debug.last_completed_action = completed_action;
    turn_debug.last_advance_done_reason = NAV_ADVANCE_DONE_REAR_SENSOR_TARGET_LINE;
    turn_debug.last_advance_final_yaw_deg_q16 = sensors->yaw_deg_q16;
    turn_debug.last_advance_final_floor_rear_black = sensors->floor_rear_black;

    RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
    return command;
}

static RobotCommand advance_until_rear_black_command(const RobotSensors *sensors)
{
    const int32_t pid_output_q16 = PID_Update_Fixed(&advance_yaw_pid, sensors->yaw_deg_q16, 10);
    const int32_t yaw_correction_pwm = FIXED_TO_INT(pid_output_q16);
    const bool wall_left_valid = wall_perception.wall_left;
    const bool wall_right_valid = wall_perception.wall_right;
    const bool diag_left_valid = wall_perception.wall_diag_left;
    const bool diag_right_valid = wall_perception.wall_diag_right;
    const bool follow_left_valid = wall_left_valid && diag_left_valid;
    const bool follow_right_valid = wall_right_valid && diag_right_valid;
    q16_16_t wall_error_q16 = 0;
    NavAdvanceCorrectionSource correction_source = NAV_ADVANCE_CORRECTION_YAW_PD;

    if (advance_guidance_mode == NAV_ADVANCE_GUIDANCE_WALL_ASSIST) {
        if (follow_left_valid && follow_right_valid) {
            wall_error_q16 = wall_perception.right_mm_q16 - wall_perception.left_mm_q16;
            correction_source = NAV_ADVANCE_CORRECTION_WALL_CENTER;
        } else if (follow_left_valid) {
            wall_error_q16 = mm_to_q16(advance_wall_config.target_left_mm) - wall_perception.left_mm_q16;
            correction_source = NAV_ADVANCE_CORRECTION_WALL_LEFT;
        } else if (follow_right_valid) {
            wall_error_q16 = wall_perception.right_mm_q16 - mm_to_q16(advance_wall_config.target_right_mm);
            correction_source = NAV_ADVANCE_CORRECTION_WALL_RIGHT;
        }
    }

    const q16_16_t wall_error_after_deadband_q16 = apply_wall_deadband(wall_error_q16);
    q16_16_t wall_previous_error_q16 = 0;
    q16_16_t wall_error_delta_q16 = 0;
    int32_t wall_p_term_pwm = 0;
    int32_t wall_d_term_pwm = 0;
    int32_t wall_raw_correction_pwm = 0;
    int16_t wall_correction_pwm = 0;
    if (correction_source != NAV_ADVANCE_CORRECTION_YAW_PD) {
        wall_previous_error_q16 = advance_wall_has_previous_error ? advance_wall_previous_error_q16 : 0;
        wall_error_delta_q16 = advance_wall_has_previous_error
            ? wall_error_after_deadband_q16 - wall_previous_error_q16
            : 0;
        wall_p_term_pwm = q16_to_pwm(wall_error_after_deadband_q16,
                                     advance_wall_config.kp_pwm_per_mm);
        wall_d_term_pwm = q16_to_pwm(wall_error_delta_q16,
                                     advance_wall_config.kd_pwm_per_mm_per_tick);
        wall_raw_correction_pwm = wall_p_term_pwm + wall_d_term_pwm;
        wall_correction_pwm = clamp_wall_correction(wall_raw_correction_pwm);
        advance_wall_previous_error_q16 = wall_error_after_deadband_q16;
        advance_wall_has_previous_error = true;
    } else {
        reset_advance_wall_pd();
    }

    const int32_t final_correction_pwm = correction_source == NAV_ADVANCE_CORRECTION_YAW_PD
        ? yaw_correction_pwm
        : wall_correction_pwm;

    turn_debug.advance_phase = advance_phase;
    turn_debug.advance_done_reason = NAV_ADVANCE_DONE_NONE;
    turn_debug.advance_yaw_setpoint_deg_q16 = advance_yaw_pid.setpoint;
    turn_debug.advance_yaw_measured_deg_q16 = sensors->yaw_deg_q16;
    turn_debug.advance_yaw_error_deg_q16 = advance_yaw_pid.setpoint - sensors->yaw_deg_q16;
    turn_debug.advance_yaw_pid_output_q16 = pid_output_q16;
    turn_debug.advance_yaw_correction_pwm = clamp_pwm(yaw_correction_pwm);
    turn_debug.advance_yaw_kp_q16 = advance_yaw_pid_config.kp_q16;
    turn_debug.advance_yaw_ki_q16 = advance_yaw_pid_config.ki_q16;
    turn_debug.advance_yaw_kd_q16 = advance_yaw_pid_config.kd_q16;
    turn_debug.advance_yaw_output_limit_pwm = (int16_t)advance_yaw_pid_config.output_limit_pwm;
    turn_debug.advance_base_left_pwm = NAV_ADVANCE_BASE_LEFT_PWM;
    turn_debug.advance_base_right_pwm = NAV_ADVANCE_BASE_RIGHT_PWM;
    turn_debug.advance_guidance_mode = advance_guidance_mode;
    turn_debug.advance_final_correction_source = correction_source;
    turn_debug.advance_wall_left_valid = wall_left_valid;
    turn_debug.advance_wall_right_valid = wall_right_valid;
    turn_debug.advance_diag_left_valid = diag_left_valid;
    turn_debug.advance_diag_right_valid = diag_right_valid;
    turn_debug.advance_follow_left_valid = follow_left_valid;
    turn_debug.advance_follow_right_valid = follow_right_valid;
    turn_debug.advance_wall_left_mm_q16 = wall_perception.left_mm_q16;
    turn_debug.advance_wall_right_mm_q16 = wall_perception.right_mm_q16;
    turn_debug.advance_wall_error_mm_q16 = wall_error_q16;
    turn_debug.advance_wall_error_after_deadband_mm_q16 = wall_error_after_deadband_q16;
    turn_debug.advance_wall_prev_error_mm_q16 = wall_previous_error_q16;
    turn_debug.advance_wall_error_delta_mm_q16 = wall_error_delta_q16;
    turn_debug.advance_wall_p_term_pwm = wall_p_term_pwm;
    turn_debug.advance_wall_d_term_pwm = wall_d_term_pwm;
    turn_debug.advance_wall_raw_correction_pwm = wall_raw_correction_pwm;
    turn_debug.advance_wall_limited_correction_pwm = wall_correction_pwm;
    turn_debug.advance_wall_correction_pwm = wall_correction_pwm;
    turn_debug.wall_kp_pwm_per_mm = advance_wall_config.kp_pwm_per_mm;
    turn_debug.wall_kd_pwm_per_mm_per_tick = advance_wall_config.kd_pwm_per_mm_per_tick;
    turn_debug.wall_error_deadband_mm_q16 = mm_to_q16(advance_wall_config.error_deadband_mm);
    turn_debug.wall_follow_target_left_mm_q16 = mm_to_q16(advance_wall_config.target_left_mm);
    turn_debug.wall_follow_target_right_mm_q16 = mm_to_q16(advance_wall_config.target_right_mm);
    turn_debug.wall_correction_limit_pwm = advance_wall_config.correction_limit_pwm;

    RobotCommand command = {
        clamp_pwm(NAV_ADVANCE_BASE_LEFT_PWM + final_correction_pwm),
        clamp_pwm(NAV_ADVANCE_BASE_RIGHT_PWM - final_correction_pwm)
    };
    return command;
}

static RobotCommand finish_smooth_turn(const RobotSensors *sensors, NavSmoothDoneReason reason)
{
    const NavAction completed_action = current_action;
    const uint16_t final_post_yaw_elapsed_ms = smooth_post_yaw_elapsed_ms;

    current_state = NAV_STATE_DONE;
    current_action = NAV_ACTION_NONE;
    smooth_phase = NAV_SMOOTH_PHASE_DONE;
    clear_live_turn_debug();
    turn_debug.smooth_phase = NAV_SMOOTH_PHASE_DONE;
    turn_debug.smooth_done_reason = reason;
    turn_debug.smooth_post_yaw_elapsed_ms = final_post_yaw_elapsed_ms;
    turn_debug.last_completed_action = completed_action;
    turn_debug.last_smooth_done_reason = reason;
    turn_debug.last_smooth_final_yaw_deg_q16 = sensors->yaw_deg_q16;
    turn_debug.last_smooth_final_floor_rear_black = sensors->floor_rear_black;
    smooth_post_yaw_elapsed_ms = 0;

    RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
    return command;
}

static void enter_smooth_post_yaw_seek(void)
{
    smooth_phase = NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE;
    smooth_post_yaw_elapsed_ms = 0;
    turn_debug.smooth_phase = smooth_phase;
    turn_debug.smooth_done_reason = NAV_SMOOTH_DONE_NONE;
    turn_debug.smooth_post_yaw_elapsed_ms = smooth_post_yaw_elapsed_ms;
    PID_Reset(&advance_yaw_pid);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, action_target_yaw_q16);
}

static RobotCommand smooth_post_yaw_seek_command(const RobotSensors *sensors)
{
    const int32_t pid_output_q16 = PID_Update_Fixed(&advance_yaw_pid, sensors->yaw_deg_q16, 10);
    const int32_t correction_pwm = FIXED_TO_INT(pid_output_q16);

    turn_debug.yaw_rate_setpoint_deg_s_q16 = 0;
    turn_debug.yaw_rate_measured_deg_s_q16 = sensors->yaw_rate_deg_s_q16;
    turn_debug.yaw_rate_error_deg_s_q16 = 0;
    turn_debug.pid_output_q16 = pid_output_q16;
    turn_debug.correction_pwm = clamp_pwm(correction_pwm);
    turn_debug.integral_q16 = advance_yaw_pid.integral;
    turn_debug.smooth_left_base_pwm = NAV_SMOOTH_POST_YAW_BASE_LEFT_PWM;
    turn_debug.smooth_right_base_pwm = NAV_SMOOTH_POST_YAW_BASE_RIGHT_PWM;
    turn_debug.smooth_phase = smooth_phase;
    turn_debug.smooth_done_reason = NAV_SMOOTH_DONE_NONE;
    turn_debug.smooth_post_yaw_elapsed_ms = smooth_post_yaw_elapsed_ms;

    RobotCommand command = {
        clamp_pwm(NAV_SMOOTH_POST_YAW_BASE_LEFT_PWM + correction_pwm),
        clamp_pwm(NAV_SMOOTH_POST_YAW_BASE_RIGHT_PWM - correction_pwm)
    };
    return command;
}

static void update_turn_debug(const RobotSensors *sensors,
                              int32_t pid_output_q16,
                              int32_t correction_pwm,
                              int16_t base_left_pwm,
                              int16_t base_right_pwm)
{
    turn_debug.yaw_rate_setpoint_deg_s_q16 = turn_yaw_rate_pid.setpoint;
    turn_debug.yaw_rate_measured_deg_s_q16 = sensors->yaw_rate_deg_s_q16;
    turn_debug.yaw_rate_error_deg_s_q16 =
        turn_yaw_rate_pid.setpoint - sensors->yaw_rate_deg_s_q16;
    turn_debug.pid_output_q16 = pid_output_q16;
    turn_debug.correction_pwm = clamp_pwm(correction_pwm);
    turn_debug.integral_q16 = turn_yaw_rate_pid.integral;
    turn_debug.turn_kp_q16 = turn_pid_config.kp_q16;
    turn_debug.turn_ki_q16 = turn_pid_config.ki_q16;
    turn_debug.turn_kd_q16 = turn_pid_config.kd_q16;
    turn_debug.turn_output_limit_pwm = (int16_t)turn_pid_config.output_limit_pwm;
    turn_debug.smooth_left_base_pwm = base_left_pwm;
    turn_debug.smooth_right_base_pwm = base_right_pwm;
    turn_debug.smooth_phase = smooth_phase;
    turn_debug.smooth_done_reason = NAV_SMOOTH_DONE_NONE;
    turn_debug.smooth_post_yaw_elapsed_ms = smooth_post_yaw_elapsed_ms;
}

static RobotCommand smooth_turn_command(const RobotSensors *sensors, int direction)
{
    const int32_t pid_output_q16 = PID_Update_Fixed(&turn_yaw_rate_pid, sensors->yaw_rate_deg_s_q16, 10);
    const int32_t correction_pwm = FIXED_TO_INT(pid_output_q16);
    const int16_t base_left_pwm = direction > 0
        ? smooth_turn_config.right_left_base_pwm
        : smooth_turn_config.left_left_base_pwm;
    const int16_t base_right_pwm = direction > 0
        ? smooth_turn_config.right_right_base_pwm
        : smooth_turn_config.left_right_base_pwm;

    RobotCommand command = {
        clamp_pwm(base_left_pwm + correction_pwm),
        clamp_pwm(base_right_pwm - correction_pwm)
    };
    update_turn_debug(sensors, pid_output_q16, correction_pwm, base_left_pwm, base_right_pwm);
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
    update_turn_debug(sensors, pid_output_q16, correction_pwm, 0, 0);
    return command;
}

void nav_core_init(void)
{
    current_state = NAV_STATE_IDLE;
    current_action = NAV_ACTION_NONE;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    smooth_post_yaw_elapsed_ms = 0;
    clear_wall_perception();
    reset_advance_wall_pd();
    advance_guidance_mode = NAV_ADVANCE_GUIDANCE_WALL_ASSIST;
    reset_advance_wall_config();
    set_turn_pid_defaults();
    set_advance_yaw_pid_defaults();
    reset_turn_debug();
    smooth_turn_config.target_yaw_rate_deg_s = NAV_SMOOTH_TARGET_YAW_RATE_DEFAULT_DEG_S;
    recalculate_smooth_turn_bases();
    apply_turn_pid_config(true);
    turn_yaw_rate_pid.setpoint = 0;
    apply_advance_yaw_pid_config(true);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, 0);
}

void nav_core_start_advance_until_rear_black(void)
{
    current_state = NAV_STATE_ADVANCING_UNTIL_REAR_BLACK;
    current_action = NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    smooth_post_yaw_elapsed_ms = 0;
    reset_turn_debug();
    reset_advance_wall_pd();
    PID_Reset(&advance_yaw_pid);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, 0);
}

void nav_core_start_smooth_turn_left(const RobotSensors *sensors)
{
    if (sensors == 0) {
        current_state = NAV_STATE_IDLE;
        current_action = NAV_ACTION_NONE;
        action_start_yaw_q16 = 0;
        action_target_yaw_q16 = 0;
        smooth_phase = NAV_SMOOTH_PHASE_NONE;
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        smooth_post_yaw_elapsed_ms = 0;
        reset_turn_debug();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_NEG_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid,
                           -INT_TO_FIXED(smooth_turn_config.target_yaw_rate_deg_s));
    reset_turn_debug();
    set_smooth_phase(sensors->floor_rear_black
                         ? NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE
                         : NAV_SMOOTH_PHASE_SEEK_TARGET_LINE);
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
        smooth_phase = NAV_SMOOTH_PHASE_NONE;
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        smooth_post_yaw_elapsed_ms = 0;
        reset_turn_debug();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid,
                           INT_TO_FIXED(smooth_turn_config.target_yaw_rate_deg_s));
    reset_turn_debug();
    set_smooth_phase(sensors->floor_rear_black
                         ? NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE
                         : NAV_SMOOTH_PHASE_SEEK_TARGET_LINE);
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
        smooth_phase = NAV_SMOOTH_PHASE_NONE;
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        reset_turn_debug();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_NEG_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, -INT_TO_FIXED(NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S));
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    reset_turn_debug();
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
        smooth_phase = NAV_SMOOTH_PHASE_NONE;
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        reset_turn_debug();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, INT_TO_FIXED(NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S));
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    reset_turn_debug();
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
        smooth_phase = NAV_SMOOTH_PHASE_NONE;
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        reset_turn_debug();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_180_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, INT_TO_FIXED(NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S));
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    reset_turn_debug();
    current_state = NAV_STATE_PIVOT_TURNING;
    current_action = NAV_ACTION_PIVOT_TURN_180;
}

void nav_core_stop(void)
{
    current_state = NAV_STATE_IDLE;
    current_action = NAV_ACTION_NONE;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    smooth_post_yaw_elapsed_ms = 0;
    reset_turn_debug();
    reset_advance_wall_pd();
}

void nav_core_set_smooth_target_yaw_rate_deg_s(int32_t target_yaw_rate_deg_s)
{
    smooth_turn_config.target_yaw_rate_deg_s =
        clamp_smooth_target_yaw_rate(target_yaw_rate_deg_s);
    recalculate_smooth_turn_bases();
    apply_current_smooth_target_to_pid();
}

int32_t nav_core_get_smooth_target_yaw_rate_deg_s(void)
{
    return smooth_turn_config.target_yaw_rate_deg_s;
}

void nav_core_get_smooth_turn_config(NavSmoothTurnConfig *config)
{
    if (config == 0) {
        return;
    }

    *config = smooth_turn_config;
}

void nav_core_get_wall_perception(NavWallPerception *perception)
{
    if (perception == 0) {
        return;
    }

    *perception = wall_perception;
}

void nav_core_set_advance_guidance_mode(NavAdvanceGuidanceMode mode)
{
    if (mode != NAV_ADVANCE_GUIDANCE_YAW_ONLY
        && mode != NAV_ADVANCE_GUIDANCE_WALL_ASSIST) {
        return;
    }

    advance_guidance_mode = mode;
    reset_advance_wall_pd();
    turn_debug.advance_guidance_mode = advance_guidance_mode;
}

NavAdvanceGuidanceMode nav_core_get_advance_guidance_mode(void)
{
    return advance_guidance_mode;
}

void nav_core_set_advance_wall_config(const NavAdvanceWallConfig *config)
{
    if (config == 0) {
        return;
    }

    advance_wall_config.kp_pwm_per_mm =
        clamp_i16(config->kp_pwm_per_mm, 0, NAV_ADVANCE_WALL_KP_PWM_PER_MM_MAX);
    advance_wall_config.kd_pwm_per_mm_per_tick =
        clamp_i16(config->kd_pwm_per_mm_per_tick, 0, NAV_ADVANCE_WALL_KD_PWM_PER_MM_PER_TICK_MAX);
    advance_wall_config.correction_limit_pwm =
        clamp_i16(config->correction_limit_pwm, 0, NAV_ADVANCE_WALL_OUTPUT_LIMIT_PWM_MAX);
    advance_wall_config.error_deadband_mm = clamp_i16(config->error_deadband_mm, 0, 50);
    advance_wall_config.target_left_mm = clamp_i16(config->target_left_mm, 1, 200);
    advance_wall_config.target_right_mm = clamp_i16(config->target_right_mm, 1, 200);
    reset_advance_wall_pd();
    clear_live_turn_debug();
}

void nav_core_get_advance_wall_config(NavAdvanceWallConfig *config)
{
    if (config == 0) {
        return;
    }

    *config = advance_wall_config;
}

void nav_core_reset_advance_wall_defaults(void)
{
    reset_advance_wall_config();
    reset_advance_wall_pd();
    clear_live_turn_debug();
}

void nav_core_get_turn_pid_config(NavTurnPidConfig *config)
{
    if (config == 0) {
        return;
    }

    *config = turn_pid_config;
}

void nav_core_set_turn_pid_config(const NavTurnPidConfig *config)
{
    if (config == 0) {
        return;
    }

    turn_pid_config.kp_q16 = clamp_i32(config->kp_q16, 0, INT_TO_FIXED(200));
    turn_pid_config.ki_q16 = clamp_i32(config->ki_q16, 0, INT_TO_FIXED(200));
    turn_pid_config.kd_q16 = clamp_i32(config->kd_q16, 0, INT_TO_FIXED(200));
    turn_pid_config.output_limit_pwm = clamp_i32(config->output_limit_pwm, 0, NAV_PWM_MAX);
    apply_turn_pid_config(true);
    apply_current_smooth_target_to_pid();
    clear_live_turn_debug();
}

void nav_core_reset_turn_pid_defaults(void)
{
    set_turn_pid_defaults();
    apply_turn_pid_config(true);
    apply_current_smooth_target_to_pid();
    clear_live_turn_debug();
}

void nav_core_get_advance_yaw_pid_config(NavAdvanceYawPidConfig *config)
{
    if (config == 0) {
        return;
    }

    *config = advance_yaw_pid_config;
}

void nav_core_set_advance_yaw_pid_config(const NavAdvanceYawPidConfig *config)
{
    if (config == 0) {
        return;
    }

    advance_yaw_pid_config.kp_q16 = clamp_i32(config->kp_q16, 0, INT_TO_FIXED(200));
    advance_yaw_pid_config.ki_q16 = clamp_i32(config->ki_q16, 0, INT_TO_FIXED(200));
    advance_yaw_pid_config.kd_q16 = clamp_i32(config->kd_q16, 0, INT_TO_FIXED(200));
    advance_yaw_pid_config.output_limit_pwm = clamp_i32(config->output_limit_pwm, 0, NAV_PWM_MAX);
    apply_advance_yaw_pid_config(true);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, 0);
    clear_live_turn_debug();
}

void nav_core_reset_advance_yaw_pid_defaults(void)
{
    set_advance_yaw_pid_defaults();
    apply_advance_yaw_pid_config(true);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, 0);
    clear_live_turn_debug();
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

void nav_core_get_turn_debug(NavTurnDebug *debug)
{
    if (debug == 0) {
        return;
    }

    *debug = turn_debug;
}

RobotCommand nav_core_update(const RobotSensors *sensors)
{
    update_wall_perception(sensors);

    if (sensors == 0) {
        smooth_phase = NAV_SMOOTH_PHASE_NONE;
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        clear_live_turn_debug();
        RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
        return command;
    }

    if (current_action == NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK) {
        if (advance_phase == NAV_ADVANCE_PHASE_NONE) {
            set_advance_phase(sensors->floor_rear_black
                                  ? NAV_ADVANCE_PHASE_WAIT_LEAVE_START_LINE
                                  : NAV_ADVANCE_PHASE_SEEK_TARGET_LINE);
        }

        if (advance_phase == NAV_ADVANCE_PHASE_WAIT_LEAVE_START_LINE
            && !sensors->floor_rear_black) {
            set_advance_phase(NAV_ADVANCE_PHASE_SEEK_TARGET_LINE);
        }

        if (advance_phase == NAV_ADVANCE_PHASE_SEEK_TARGET_LINE
            && sensors->floor_rear_black) {
            return finish_advance_until_rear_black(sensors);
        }

        smooth_phase = NAV_SMOOTH_PHASE_NONE;
        return advance_until_rear_black_command(sensors);
    }

    if (current_action == NAV_ACTION_SMOOTH_TURN_RIGHT) {
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        if (smooth_phase == NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE
            && !sensors->floor_rear_black) {
            set_smooth_phase(NAV_SMOOTH_PHASE_SEEK_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE
            && sensors->floor_rear_black) {
            return finish_smooth_turn(sensors, NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE) {
            if (sensors->floor_rear_black) {
                return finish_smooth_turn(sensors, NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE);
            }
            if (smooth_post_yaw_elapsed_ms >= NAV_SMOOTH_POST_YAW_TIMEOUT_MS) {
                return finish_smooth_turn(sensors, NAV_SMOOTH_DONE_TIMEOUT);
            }
            smooth_post_yaw_elapsed_ms += 10;
            return smooth_post_yaw_seek_command(sensors);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE
            && sensors->yaw_deg_q16 >= action_target_yaw_q16 - Q16_TURN_TOLERANCE_DEG) {
            enter_smooth_post_yaw_seek();
            return smooth_post_yaw_seek_command(sensors);
        }

        return smooth_turn_command(sensors, 1);
    }

    if (current_action == NAV_ACTION_SMOOTH_TURN_LEFT) {
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        if (smooth_phase == NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE
            && !sensors->floor_rear_black) {
            set_smooth_phase(NAV_SMOOTH_PHASE_SEEK_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE
            && sensors->floor_rear_black) {
            return finish_smooth_turn(sensors, NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE) {
            if (sensors->floor_rear_black) {
                return finish_smooth_turn(sensors, NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE);
            }
            if (smooth_post_yaw_elapsed_ms >= NAV_SMOOTH_POST_YAW_TIMEOUT_MS) {
                return finish_smooth_turn(sensors, NAV_SMOOTH_DONE_TIMEOUT);
            }
            smooth_post_yaw_elapsed_ms += 10;
            return smooth_post_yaw_seek_command(sensors);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE
            && sensors->yaw_deg_q16 <= action_target_yaw_q16 + Q16_TURN_TOLERANCE_DEG) {
            enter_smooth_post_yaw_seek();
            return smooth_post_yaw_seek_command(sensors);
        }

        return smooth_turn_command(sensors, -1);
    }

    if (current_action == NAV_ACTION_PIVOT_TURN_RIGHT) {
        if (sensors->yaw_deg_q16 >= Q16_90_DEG - Q16_TURN_TOLERANCE_DEG) {
            current_state = NAV_STATE_DONE;
            current_action = NAV_ACTION_NONE;
            smooth_phase = NAV_SMOOTH_PHASE_NONE;
            advance_phase = NAV_ADVANCE_PHASE_NONE;
            clear_live_turn_debug();
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        return pivot_turn_command(sensors, 1);
    }

    if (current_action == NAV_ACTION_PIVOT_TURN_LEFT) {
        if (sensors->yaw_deg_q16 <= Q16_NEG_90_DEG + Q16_TURN_TOLERANCE_DEG) {
            current_state = NAV_STATE_DONE;
            current_action = NAV_ACTION_NONE;
            smooth_phase = NAV_SMOOTH_PHASE_NONE;
            advance_phase = NAV_ADVANCE_PHASE_NONE;
            clear_live_turn_debug();
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        return pivot_turn_command(sensors, -1);
    }

    if (current_action == NAV_ACTION_PIVOT_TURN_180) {
        if (abs_q16(sensors->yaw_deg_q16) >= Q16_180_DEG - Q16_TURN_TOLERANCE_DEG) {
            current_state = NAV_STATE_DONE;
            current_action = NAV_ACTION_NONE;
            smooth_phase = NAV_SMOOTH_PHASE_NONE;
            advance_phase = NAV_ADVANCE_PHASE_NONE;
            clear_live_turn_debug();
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        return pivot_turn_command(sensors, 1);
    }

    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    clear_live_turn_debug();
    RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
    return command;
}
