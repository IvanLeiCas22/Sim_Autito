#include "nav_core.h"
#include "nav_map.h"
#include "pid_controller.h"

enum {
    NAV_PWM_STOP = 0,
    NAV_PWM_FORWARD = 3000,
    NAV_ADVANCE_BASE_LEFT_PWM = 3000,
    NAV_ADVANCE_BASE_RIGHT_PWM = 3370,
    NAV_APPROACH_FRONT_BASE_LEFT_PWM = 1800,
    NAV_APPROACH_FRONT_BASE_RIGHT_PWM = 2020,
    NAV_APPROACH_FRONT_TARGET_MM_Q16 = 70 << 16,
    NAV_APPROACH_FRONT_TIMEOUT_MS = 2000,
    NAV_BRAKE_SETTLE_MS = 200,
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
    NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE = 2,
    NAV_SPECIAL_DETECT_MIN_MS = 100,
    NAV_SPECIAL_DETECT_MAX_MS = 800,
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
static NavApproachFrontPhase approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
static NavTurnDebug turn_debug = {0};
static NavSmoothTurnConfig smooth_turn_config = {0};
static NavWallPerception wall_perception = {0};
static uint16_t smooth_post_yaw_elapsed_ms = 0;
static uint16_t advance_elapsed_since_leave_start_line_ms = 0;
static uint16_t approach_front_elapsed_ms = 0;
static uint16_t approach_front_brake_elapsed_ms = 0;
static NavApproachFrontDoneReason approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
static bool special_ignore_rear_until_white = false;
static bool special_confirmed = false;
static NavAdvanceGuidanceMode advance_guidance_mode = NAV_ADVANCE_GUIDANCE_WALL_ASSIST;
static NavPolicy nav_policy = NAV_POLICY_RIGHT_HAND_RULE;
static NavMapCandidateDebug map_candidate_debug = {0};
static bool map_walls_recorded_for_current_pose = false;
static bool map_initial_wall_snapshot_pending = false;
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

typedef enum NavMapUpdatePolicy {
    NAV_MAP_UPDATE_NONE = 0,
    NAV_MAP_UPDATE_POSE_ONLY,
    NAV_MAP_UPDATE_WALLS_ONLY,
    NAV_MAP_UPDATE_POSE_AND_WALLS
} NavMapUpdatePolicy;

static NavMapDirection map_turn_left(NavMapDirection dir)
{
    return (NavMapDirection)((dir + 3) & 3);
}

static NavMapDirection map_turn_right(NavMapDirection dir)
{
    return (NavMapDirection)((dir + 1) & 3);
}

static void map_neighbor_for_dir(int8_t cell_x,
                                 int8_t cell_y,
                                 NavMapDirection dir,
                                 int8_t *neighbor_x,
                                 int8_t *neighbor_y)
{
    *neighbor_x = cell_x;
    *neighbor_y = cell_y;

    switch (dir) {
    case NAV_DIR_NORTH:
        --(*neighbor_y);
        break;
    case NAV_DIR_EAST:
        ++(*neighbor_x);
        break;
    case NAV_DIR_SOUTH:
        ++(*neighbor_y);
        break;
    case NAV_DIR_WEST:
        --(*neighbor_x);
        break;
    }
}

static bool map_cell_is_inside(const NavMapDebugSnapshot *map_debug, int8_t cell_x, int8_t cell_y)
{
    return map_debug != 0
        && map_debug->enabled
        && cell_x >= 0
        && cell_y >= 0
        && cell_x < (int8_t)map_debug->width
        && cell_y < (int8_t)map_debug->height;
}

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

static NavMapAction map_action_from_nav_action(NavAction action)
{
    switch (action) {
    case NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK:
        return NAV_MAP_ACTION_ADVANCE_LINE;
    case NAV_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT:
        return NAV_MAP_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT;
    case NAV_ACTION_SMOOTH_TURN_LEFT:
        return NAV_MAP_ACTION_SMOOTH_TURN_LEFT;
    case NAV_ACTION_SMOOTH_TURN_RIGHT:
        return NAV_MAP_ACTION_SMOOTH_TURN_RIGHT;
    case NAV_ACTION_PIVOT_TURN_LEFT:
        return NAV_MAP_ACTION_PIVOT_TURN_LEFT;
    case NAV_ACTION_PIVOT_TURN_RIGHT:
        return NAV_MAP_ACTION_PIVOT_TURN_RIGHT;
    case NAV_ACTION_PIVOT_TURN_180:
        return NAV_MAP_ACTION_PIVOT_TURN_180;
    case NAV_ACTION_NONE:
        break;
    }

    return NAV_MAP_ACTION_NONE;
}

static NavMapUpdatePolicy map_update_policy_from_nav_action(NavAction action)
{
    switch (action) {
    case NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK:
    case NAV_ACTION_SMOOTH_TURN_LEFT:
    case NAV_ACTION_SMOOTH_TURN_RIGHT:
        return NAV_MAP_UPDATE_POSE_AND_WALLS;
    case NAV_ACTION_PIVOT_TURN_LEFT:
    case NAV_ACTION_PIVOT_TURN_RIGHT:
    case NAV_ACTION_PIVOT_TURN_180:
        return NAV_MAP_UPDATE_POSE_ONLY;
    case NAV_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT:
    case NAV_ACTION_NONE:
        break;
    }

    return NAV_MAP_UPDATE_NONE;
}

static void record_current_map_cell_walls(NavMapAction action)
{
    nav_map_mark_visited_current();
    nav_map_update_current_cell_walls_from_relative_for_action(wall_perception.wall_front,
                                                               wall_perception.wall_left,
                                                               wall_perception.wall_right,
                                                               action);
    map_walls_recorded_for_current_pose = true;
    map_initial_wall_snapshot_pending = false;
}

static void apply_completed_action_to_map(NavAction action)
{
    const NavMapAction map_action = map_action_from_nav_action(action);
    const NavMapUpdatePolicy policy = map_update_policy_from_nav_action(action);
    if (map_action == NAV_MAP_ACTION_NONE || policy == NAV_MAP_UPDATE_NONE) {
        return;
    }

    if (policy == NAV_MAP_UPDATE_POSE_ONLY || policy == NAV_MAP_UPDATE_POSE_AND_WALLS) {
        nav_map_apply_completed_action(map_action);
    }
    if (policy == NAV_MAP_UPDATE_WALLS_ONLY || policy == NAV_MAP_UPDATE_POSE_AND_WALLS) {
        record_current_map_cell_walls(map_action);
    }
}

static void record_current_map_cell_if_ready(const RobotSensors *sensors)
{
    if (sensors == 0) {
        return;
    }
    if (map_initial_wall_snapshot_pending) {
        record_current_map_cell_walls(NAV_MAP_ACTION_INITIAL_SNAPSHOT);
        return;
    }
    if (!sensors->floor_rear_black) {
        return;
    }
    if (current_action != NAV_ACTION_NONE) {
        return;
    }
    if (map_walls_recorded_for_current_pose) {
        return;
    }

    record_current_map_cell_walls(NAV_MAP_ACTION_NONE);
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
    turn_debug.special_candidate = false;
    turn_debug.special_confirmed = special_confirmed;
    turn_debug.special_ignore_rear_until_white = special_ignore_rear_until_white;
    turn_debug.advance_elapsed_since_leave_start_line_ms =
        advance_elapsed_since_leave_start_line_ms;
    turn_debug.special_detect_min_ms = NAV_SPECIAL_DETECT_MIN_MS;
    turn_debug.special_detect_max_ms = NAV_SPECIAL_DETECT_MAX_MS;
    turn_debug.approach_front_phase = approach_front_phase;
    turn_debug.approach_front_done_reason = approach_front_done_reason;
    turn_debug.approach_front_target_mm_q16 = NAV_APPROACH_FRONT_TARGET_MM_Q16;
    turn_debug.approach_front_left_mm_q16 = 0;
    turn_debug.approach_front_right_mm_q16 = 0;
    turn_debug.approach_front_elapsed_ms = approach_front_elapsed_ms;
    turn_debug.approach_front_brake_elapsed_ms = approach_front_brake_elapsed_ms;
    turn_debug.approach_front_base_left_pwm = 0;
    turn_debug.approach_front_base_right_pwm = 0;
    turn_debug.approach_front_correction_pwm = 0;
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
    turn_debug.advance_wall_raw_error_mm_q16 = 0;
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
    turn_debug.wall_single_side_error_scale = NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE;
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
    turn_debug.last_approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
}

static void reset_turn_debug(void)
{
    clear_live_turn_debug();
    clear_completed_turn_debug();
}

static RobotCommand guided_forward_command(const RobotSensors *sensors,
                                           int16_t base_left_pwm,
                                           int16_t base_right_pwm);

static void reset_special_detection_state(void)
{
    advance_elapsed_since_leave_start_line_ms = 0;
    special_ignore_rear_until_white = false;
    special_confirmed = false;
    turn_debug.special_candidate = false;
    turn_debug.special_confirmed = false;
    turn_debug.special_ignore_rear_until_white = false;
    turn_debug.advance_elapsed_since_leave_start_line_ms = 0;
}

static bool rear_black_for_line(const RobotSensors *sensors)
{
    return sensors != 0
        && sensors->floor_rear_black
        && !special_ignore_rear_until_white;
}

static void update_special_detection(const RobotSensors *sensors, bool allow_confirm)
{
    if (sensors == 0) {
        return;
    }

    if (advance_elapsed_since_leave_start_line_ms < UINT16_MAX - 10) {
        advance_elapsed_since_leave_start_line_ms += 10;
    }

    if (special_ignore_rear_until_white && !sensors->floor_rear_black) {
        special_ignore_rear_until_white = false;
    }

    const bool special_candidate = sensors->floor_front_black && sensors->floor_rear_black;
    const bool in_special_window =
        advance_elapsed_since_leave_start_line_ms >= NAV_SPECIAL_DETECT_MIN_MS
        && advance_elapsed_since_leave_start_line_ms <= NAV_SPECIAL_DETECT_MAX_MS;

    special_confirmed = false;
    if (allow_confirm
        && special_candidate
        && !special_ignore_rear_until_white
        && in_special_window) {
        (void)nav_map_mark_current_cell_special();
        special_confirmed = true;
        special_ignore_rear_until_white = true;
    }

    turn_debug.special_candidate = special_candidate;
    turn_debug.special_confirmed = special_confirmed;
    turn_debug.special_ignore_rear_until_white = special_ignore_rear_until_white;
    turn_debug.advance_elapsed_since_leave_start_line_ms =
        advance_elapsed_since_leave_start_line_ms;
    turn_debug.special_detect_min_ms = NAV_SPECIAL_DETECT_MIN_MS;
    turn_debug.special_detect_max_ms = NAV_SPECIAL_DETECT_MAX_MS;
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
    if (phase == NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE) {
        reset_special_detection_state();
    } else if (phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE) {
        advance_elapsed_since_leave_start_line_ms = 0;
        special_ignore_rear_until_white = false;
        special_confirmed = false;
    }
}

static void set_advance_phase(NavAdvancePhase phase)
{
    advance_phase = phase;
    turn_debug.advance_phase = phase;
    turn_debug.advance_done_reason = NAV_ADVANCE_DONE_NONE;
    if (phase == NAV_ADVANCE_PHASE_WAIT_LEAVE_START_LINE) {
        reset_special_detection_state();
    } else if (phase == NAV_ADVANCE_PHASE_SEEK_TARGET_LINE) {
        advance_elapsed_since_leave_start_line_ms = 0;
        special_ignore_rear_until_white = false;
        special_confirmed = false;
    }
}

static void set_approach_front_phase(NavApproachFrontPhase phase)
{
    approach_front_phase = phase;
    turn_debug.approach_front_phase = phase;
    if (phase == NAV_APPROACH_FRONT_PHASE_DRIVE) {
        approach_front_brake_elapsed_ms = 0;
    }
}

static RobotCommand finish_advance_until_rear_black(const RobotSensors *sensors)
{
    const NavAction completed_action = current_action;

    apply_completed_action_to_map(completed_action);
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

static RobotCommand finish_approach_front_wall_for_pivot(const RobotSensors *sensors,
                                                         NavApproachFrontDoneReason reason)
{
    const NavAction completed_action = current_action;

    apply_completed_action_to_map(completed_action);
    current_state = NAV_STATE_DONE;
    current_action = NAV_ACTION_NONE;
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_DONE;
    approach_front_done_reason = reason;
    reset_advance_wall_pd();
    clear_live_turn_debug();
    turn_debug.approach_front_phase = approach_front_phase;
    turn_debug.approach_front_done_reason = reason;
    turn_debug.approach_front_target_mm_q16 = NAV_APPROACH_FRONT_TARGET_MM_Q16;
    turn_debug.approach_front_left_mm_q16 = sensors->ir_front_left_mm_q16;
    turn_debug.approach_front_right_mm_q16 = sensors->ir_front_right_mm_q16;
    turn_debug.approach_front_elapsed_ms = approach_front_elapsed_ms;
    turn_debug.approach_front_brake_elapsed_ms = approach_front_brake_elapsed_ms;
    turn_debug.approach_front_base_left_pwm = NAV_APPROACH_FRONT_BASE_LEFT_PWM;
    turn_debug.approach_front_base_right_pwm = NAV_APPROACH_FRONT_BASE_RIGHT_PWM;
    turn_debug.approach_front_correction_pwm = 0;
    turn_debug.last_completed_action = completed_action;
    turn_debug.last_approach_front_done_reason = reason;

    RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
    return command;
}

static bool approach_front_target_reached(const RobotSensors *sensors)
{
    return sensors->ir_front_left_mm_q16 <= NAV_APPROACH_FRONT_TARGET_MM_Q16
        && sensors->ir_front_right_mm_q16 <= NAV_APPROACH_FRONT_TARGET_MM_Q16;
}

static RobotCommand approach_front_wall_for_pivot_command(const RobotSensors *sensors)
{
    RobotCommand command = guided_forward_command(sensors,
                                                  NAV_APPROACH_FRONT_BASE_LEFT_PWM,
                                                  NAV_APPROACH_FRONT_BASE_RIGHT_PWM);
    turn_debug.approach_front_phase = approach_front_phase;
    turn_debug.approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    turn_debug.approach_front_target_mm_q16 = NAV_APPROACH_FRONT_TARGET_MM_Q16;
    turn_debug.approach_front_left_mm_q16 = sensors->ir_front_left_mm_q16;
    turn_debug.approach_front_right_mm_q16 = sensors->ir_front_right_mm_q16;
    turn_debug.approach_front_elapsed_ms = approach_front_elapsed_ms;
    turn_debug.approach_front_brake_elapsed_ms = approach_front_brake_elapsed_ms;
    turn_debug.approach_front_base_left_pwm = NAV_APPROACH_FRONT_BASE_LEFT_PWM;
    turn_debug.approach_front_base_right_pwm = NAV_APPROACH_FRONT_BASE_RIGHT_PWM;
    turn_debug.approach_front_correction_pwm =
        clamp_pwm(command.left_motor_pwm - NAV_APPROACH_FRONT_BASE_LEFT_PWM);
    return command;
}

static RobotCommand guided_forward_command(const RobotSensors *sensors,
                                           int16_t base_left_pwm,
                                           int16_t base_right_pwm)
{
    const int32_t pid_output_q16 = PID_Update_Fixed(&advance_yaw_pid, sensors->yaw_deg_q16, 10);
    const int32_t yaw_correction_pwm = FIXED_TO_INT(pid_output_q16);
    const bool wall_left_valid = wall_perception.wall_left;
    const bool wall_right_valid = wall_perception.wall_right;
    const bool diag_left_valid = wall_perception.wall_diag_left;
    const bool diag_right_valid = wall_perception.wall_diag_right;
    const bool follow_left_valid = wall_left_valid && diag_left_valid;
    const bool follow_right_valid = wall_right_valid && diag_right_valid;
    q16_16_t wall_raw_error_q16 = 0;
    q16_16_t wall_error_q16 = 0;
    NavAdvanceCorrectionSource correction_source = NAV_ADVANCE_CORRECTION_YAW_PD;

    if (advance_guidance_mode == NAV_ADVANCE_GUIDANCE_WALL_ASSIST) {
        if (follow_left_valid && follow_right_valid) {
            wall_raw_error_q16 = wall_perception.right_mm_q16 - wall_perception.left_mm_q16;
            wall_error_q16 = wall_raw_error_q16;
            correction_source = NAV_ADVANCE_CORRECTION_WALL_CENTER;
        } else if (follow_left_valid) {
            wall_raw_error_q16 =
                mm_to_q16(advance_wall_config.target_left_mm) - wall_perception.left_mm_q16;
            wall_error_q16 = wall_raw_error_q16 * NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE;
            correction_source = NAV_ADVANCE_CORRECTION_WALL_LEFT;
        } else if (follow_right_valid) {
            wall_raw_error_q16 =
                wall_perception.right_mm_q16 - mm_to_q16(advance_wall_config.target_right_mm);
            wall_error_q16 = wall_raw_error_q16 * NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE;
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
    turn_debug.advance_base_left_pwm = base_left_pwm;
    turn_debug.advance_base_right_pwm = base_right_pwm;
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
    turn_debug.advance_wall_raw_error_mm_q16 = wall_raw_error_q16;
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
    turn_debug.wall_single_side_error_scale = NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE;

    RobotCommand command = {
        clamp_pwm(base_left_pwm + final_correction_pwm),
        clamp_pwm(base_right_pwm - final_correction_pwm)
    };
    return command;
}

static RobotCommand advance_until_rear_black_command(const RobotSensors *sensors)
{
    return guided_forward_command(sensors, NAV_ADVANCE_BASE_LEFT_PWM, NAV_ADVANCE_BASE_RIGHT_PWM);
}

static RobotCommand finish_smooth_turn(const RobotSensors *sensors, NavSmoothDoneReason reason)
{
    const NavAction completed_action = current_action;
    const uint16_t final_post_yaw_elapsed_ms = smooth_post_yaw_elapsed_ms;

    apply_completed_action_to_map(completed_action);
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
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    smooth_post_yaw_elapsed_ms = 0;
    advance_elapsed_since_leave_start_line_ms = 0;
    approach_front_elapsed_ms = 0;
    approach_front_brake_elapsed_ms = 0;
    approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    special_ignore_rear_until_white = false;
    special_confirmed = false;
    map_walls_recorded_for_current_pose = false;
    map_initial_wall_snapshot_pending = false;
    nav_policy = NAV_POLICY_RIGHT_HAND_RULE;
    map_candidate_debug = (NavMapCandidateDebug){0};
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
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    smooth_post_yaw_elapsed_ms = 0;
    approach_front_elapsed_ms = 0;
    approach_front_brake_elapsed_ms = 0;
    approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    reset_turn_debug();
    reset_special_detection_state();
    reset_advance_wall_pd();
    PID_Reset(&advance_yaw_pid);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, 0);
}

void nav_core_start_approach_front_wall_for_pivot(void)
{
    current_state = NAV_STATE_APPROACHING_FRONT_WALL_FOR_PIVOT;
    current_action = NAV_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    approach_front_elapsed_ms = 0;
    approach_front_brake_elapsed_ms = 0;
    approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    reset_turn_debug();
    reset_special_detection_state();
    set_approach_front_phase(NAV_APPROACH_FRONT_PHASE_DRIVE);
    reset_advance_wall_pd();
    PID_Reset(&advance_yaw_pid);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, 0);
    /* TODO: add front-wall alignment using front_left - front_right before pivot. */
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
        approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
        smooth_post_yaw_elapsed_ms = 0;
        reset_turn_debug();
        reset_special_detection_state();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_NEG_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid,
                           -INT_TO_FIXED(smooth_turn_config.target_yaw_rate_deg_s));
    reset_turn_debug();
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
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
        approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
        smooth_post_yaw_elapsed_ms = 0;
        reset_turn_debug();
        reset_special_detection_state();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid,
                           INT_TO_FIXED(smooth_turn_config.target_yaw_rate_deg_s));
    reset_turn_debug();
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
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
        approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
        reset_turn_debug();
        reset_special_detection_state();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_NEG_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, -INT_TO_FIXED(NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S));
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    reset_turn_debug();
    reset_special_detection_state();
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
        approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
        reset_turn_debug();
        reset_special_detection_state();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_90_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, INT_TO_FIXED(NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S));
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    reset_turn_debug();
    reset_special_detection_state();
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
        approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
        reset_turn_debug();
        reset_special_detection_state();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_180_DEG;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid, INT_TO_FIXED(NAV_PIVOT_YAW_RATE_TARGET_ABS_DEG_S));
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    reset_turn_debug();
    reset_special_detection_state();
    /* TODO: add CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE before in-cell pivots outside dead ends. */
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
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    smooth_post_yaw_elapsed_ms = 0;
    advance_elapsed_since_leave_start_line_ms = 0;
    approach_front_elapsed_ms = 0;
    approach_front_brake_elapsed_ms = 0;
    approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    special_ignore_rear_until_white = false;
    special_confirmed = false;
    map_walls_recorded_for_current_pose = false;
    map_initial_wall_snapshot_pending = false;
    map_candidate_debug = (NavMapCandidateDebug){0};
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

void nav_core_map_init(uint8_t width,
                       uint8_t height,
                       int8_t start_cell_x,
                       int8_t start_cell_y,
                       NavMapDirection start_dir)
{
    nav_map_init(width, height, start_cell_x, start_cell_y, start_dir);
    map_walls_recorded_for_current_pose = false;
    map_initial_wall_snapshot_pending = true;
}

void nav_core_get_map_debug(NavMapDebugSnapshot *snapshot)
{
    nav_map_get_debug_snapshot(snapshot);
    if (snapshot != 0) {
        snapshot->initial_wall_snapshot_pending = map_initial_wall_snapshot_pending;
    }
}

bool nav_core_get_map_cell(int8_t cell_x, int8_t cell_y, NavMapCell *cell)
{
    return nav_map_get_cell(cell_x, cell_y, cell);
}

void nav_core_set_policy(NavPolicy policy)
{
    if (policy != NAV_POLICY_RIGHT_HAND_RULE && policy != NAV_POLICY_MAP_PREFER_UNVISITED) {
        policy = NAV_POLICY_RIGHT_HAND_RULE;
    }

    nav_policy = policy;
    map_candidate_debug = (NavMapCandidateDebug){0};
}

NavPolicy nav_core_get_policy(void)
{
    return nav_policy;
}

void nav_core_get_map_candidate_debug(NavMapCandidateDebug *debug)
{
    if (debug == 0) {
        return;
    }

    *debug = map_candidate_debug;
}

static NavRecommendedAction recommend_right_hand_rule(const RobotSensors *sensors)
{
    if (sensors == 0) {
        return NAV_RECOMMENDED_NONE;
    }

    if (!sensors->floor_rear_black) {
        return wall_perception.wall_front
            ? NAV_RECOMMENDED_RECOVERY_PIVOT_180_FRONT_BLOCKED
            : NAV_RECOMMENDED_ACQUIRE_REAR_LINE;
    }

    if (!wall_perception.wall_right) {
        return NAV_RECOMMENDED_SMOOTH_RIGHT;
    }
    if (!wall_perception.wall_front) {
        return NAV_RECOMMENDED_ADVANCE_LINE;
    }
    if (!wall_perception.wall_left) {
        return NAV_RECOMMENDED_SMOOTH_LEFT;
    }

    return NAV_RECOMMENDED_PIVOT_180;
}

static void fill_map_candidate(bool free_path,
                               NavMapDirection dir,
                               int8_t *cell_x,
                               int8_t *cell_y,
                               bool *valid,
                               bool *visited,
                               const NavMapDebugSnapshot *map_debug)
{
    *cell_x = -1;
    *cell_y = -1;
    *valid = false;
    *visited = false;

    if (!free_path || map_debug == 0 || !map_debug->enabled) {
        return;
    }

    int8_t candidate_x = map_debug->cell_x;
    int8_t candidate_y = map_debug->cell_y;
    map_neighbor_for_dir(map_debug->cell_x, map_debug->cell_y, dir, &candidate_x, &candidate_y);
    *cell_x = candidate_x;
    *cell_y = candidate_y;

    if (!map_cell_is_inside(map_debug, candidate_x, candidate_y)) {
        return;
    }

    NavMapCell cell = {0};
    if (!nav_map_get_cell(candidate_x, candidate_y, &cell)) {
        return;
    }

    *valid = true;
    *visited = cell.visited;
}

static NavRecommendedAction recommend_map_prefer_unvisited(const RobotSensors *sensors)
{
    map_candidate_debug = (NavMapCandidateDebug){0};

    if (sensors == 0 || !sensors->floor_rear_black) {
        return recommend_right_hand_rule(sensors);
    }

    NavMapDebugSnapshot map_debug = {0};
    nav_map_get_debug_snapshot(&map_debug);
    if (!map_debug.enabled) {
        return recommend_right_hand_rule(sensors);
    }

    const NavMapDirection right_dir = map_turn_right(map_debug.dir);
    const NavMapDirection front_dir = map_debug.dir;
    const NavMapDirection left_dir = map_turn_left(map_debug.dir);
    const bool right_free = !wall_perception.wall_right;
    const bool front_free = !wall_perception.wall_front;
    const bool left_free = !wall_perception.wall_left;

    fill_map_candidate(right_free,
                       right_dir,
                       &map_candidate_debug.right_cell_x,
                       &map_candidate_debug.right_cell_y,
                       &map_candidate_debug.right_cell_valid,
                       &map_candidate_debug.right_cell_visited,
                       &map_debug);
    fill_map_candidate(front_free,
                       front_dir,
                       &map_candidate_debug.front_cell_x,
                       &map_candidate_debug.front_cell_y,
                       &map_candidate_debug.front_cell_valid,
                       &map_candidate_debug.front_cell_visited,
                       &map_debug);
    fill_map_candidate(left_free,
                       left_dir,
                       &map_candidate_debug.left_cell_x,
                       &map_candidate_debug.left_cell_y,
                       &map_candidate_debug.left_cell_valid,
                       &map_candidate_debug.left_cell_visited,
                       &map_debug);

    if (map_candidate_debug.right_cell_valid && !map_candidate_debug.right_cell_visited) {
        map_candidate_debug.used_unvisited_preference = true;
        return NAV_RECOMMENDED_SMOOTH_RIGHT;
    }
    if (map_candidate_debug.front_cell_valid && !map_candidate_debug.front_cell_visited) {
        map_candidate_debug.used_unvisited_preference = true;
        return NAV_RECOMMENDED_ADVANCE_LINE;
    }
    if (map_candidate_debug.left_cell_valid && !map_candidate_debug.left_cell_visited) {
        map_candidate_debug.used_unvisited_preference = true;
        return NAV_RECOMMENDED_SMOOTH_LEFT;
    }

    if (right_free && map_candidate_debug.right_cell_valid) {
        return NAV_RECOMMENDED_SMOOTH_RIGHT;
    }
    if (front_free && map_candidate_debug.front_cell_valid) {
        return NAV_RECOMMENDED_ADVANCE_LINE;
    }
    if (left_free && map_candidate_debug.left_cell_valid) {
        return NAV_RECOMMENDED_SMOOTH_LEFT;
    }

    return NAV_RECOMMENDED_PIVOT_180;
}

NavRecommendedAction nav_core_recommend_basic_action(const RobotSensors *sensors)
{
    map_candidate_debug.used_unvisited_preference = false;
    if (nav_policy == NAV_POLICY_MAP_PREFER_UNVISITED) {
        return recommend_map_prefer_unvisited(sensors);
    }

    map_candidate_debug = (NavMapCandidateDebug){0};
    return recommend_right_hand_rule(sensors);
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
        approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
        clear_live_turn_debug();
        RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
        return command;
    }

    record_current_map_cell_if_ready(sensors);

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

        if (advance_phase == NAV_ADVANCE_PHASE_SEEK_TARGET_LINE) {
            update_special_detection(sensors, true);
        }

        if (advance_phase == NAV_ADVANCE_PHASE_SEEK_TARGET_LINE
            && rear_black_for_line(sensors)) {
            return finish_advance_until_rear_black(sensors);
        }

        smooth_phase = NAV_SMOOTH_PHASE_NONE;
        return advance_until_rear_black_command(sensors);
    }

    if (current_action == NAV_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT) {
        smooth_phase = NAV_SMOOTH_PHASE_NONE;
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        turn_debug.approach_front_target_mm_q16 = NAV_APPROACH_FRONT_TARGET_MM_Q16;
        turn_debug.approach_front_left_mm_q16 = sensors->ir_front_left_mm_q16;
        turn_debug.approach_front_right_mm_q16 = sensors->ir_front_right_mm_q16;

        if (approach_front_phase == NAV_APPROACH_FRONT_PHASE_NONE) {
            set_approach_front_phase(NAV_APPROACH_FRONT_PHASE_DRIVE);
        }

        if (approach_front_phase == NAV_APPROACH_FRONT_PHASE_DRIVE) {
            if (approach_front_target_reached(sensors)) {
                approach_front_done_reason = NAV_APPROACH_FRONT_DONE_TARGET_DISTANCE;
                set_approach_front_phase(NAV_APPROACH_FRONT_PHASE_BRAKE_SETTLE);
            } else if (approach_front_elapsed_ms >= NAV_APPROACH_FRONT_TIMEOUT_MS) {
                approach_front_done_reason = NAV_APPROACH_FRONT_DONE_TIMEOUT;
                set_approach_front_phase(NAV_APPROACH_FRONT_PHASE_BRAKE_SETTLE);
            } else {
                approach_front_elapsed_ms += 10;
                return approach_front_wall_for_pivot_command(sensors);
            }
        }

        if (approach_front_phase == NAV_APPROACH_FRONT_PHASE_BRAKE_SETTLE) {
            if (approach_front_brake_elapsed_ms >= NAV_BRAKE_SETTLE_MS) {
                return finish_approach_front_wall_for_pivot(sensors, approach_front_done_reason);
            }

            approach_front_brake_elapsed_ms += 10;
            turn_debug.approach_front_phase = approach_front_phase;
            turn_debug.approach_front_done_reason = approach_front_done_reason;
            turn_debug.approach_front_elapsed_ms = approach_front_elapsed_ms;
            turn_debug.approach_front_brake_elapsed_ms = approach_front_brake_elapsed_ms;
            turn_debug.approach_front_base_left_pwm = NAV_APPROACH_FRONT_BASE_LEFT_PWM;
            turn_debug.approach_front_base_right_pwm = NAV_APPROACH_FRONT_BASE_RIGHT_PWM;
            turn_debug.approach_front_correction_pwm = 0;
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        return finish_approach_front_wall_for_pivot(sensors, approach_front_done_reason);
    }

    if (current_action == NAV_ACTION_SMOOTH_TURN_RIGHT) {
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        if (smooth_phase == NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE
            && !sensors->floor_rear_black) {
            set_smooth_phase(NAV_SMOOTH_PHASE_SEEK_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE) {
            update_special_detection(sensors, true);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE
            && rear_black_for_line(sensors)) {
            return finish_smooth_turn(sensors, NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE) {
            update_special_detection(sensors, true);
            if (rear_black_for_line(sensors)) {
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

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE) {
            update_special_detection(sensors, true);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE
            && rear_black_for_line(sensors)) {
            return finish_smooth_turn(sensors, NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE) {
            update_special_detection(sensors, true);
            if (rear_black_for_line(sensors)) {
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
            apply_completed_action_to_map(current_action);
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
            apply_completed_action_to_map(current_action);
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
            apply_completed_action_to_map(current_action);
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
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    clear_live_turn_debug();
    RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
    return command;
}
