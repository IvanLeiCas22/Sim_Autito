#include "nav_core.h"
#include "nav_flood.h"
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
    NAV_CENTER_PIVOT_BASE_LEFT_PWM = 1800,
    NAV_CENTER_PIVOT_BASE_RIGHT_PWM = 2020,
    NAV_CENTER_PIVOT_FRONT_LINE_MIN_MS = 600,
    NAV_CENTER_PIVOT_TIMEOUT_MS = 2500,
    NAV_BRAKE_SETTLE_MS = 200,
    NAV_SMOOTH_POST_YAW_BASE_LEFT_PWM = 2500,
    NAV_SMOOTH_POST_YAW_BASE_RIGHT_PWM = 2800,
    NAV_SMOOTH_POST_YAW_TIMEOUT_MS = 800,
    NAV_DIAG_GUIDANCE_TARGET_MM_DEFAULT = 99,
    NAV_DIAG_GUIDANCE_ERROR_SCALE_NUM_DEFAULT = 1,
    NAV_DIAG_GUIDANCE_ERROR_SCALE_DEN_DEFAULT = 1,
    NAV_DIAG_GUIDANCE_KP_PWM_PER_MM_DEFAULT = 30,
    NAV_DIAG_GUIDANCE_KD_PWM_PER_MM_DEFAULT = 0,
    NAV_DIAG_GUIDANCE_OUTPUT_LIMIT_PWM_DEFAULT = 1000,
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
    NAV_WALL_CAUTION_TIMEOUT_MS_DEFAULT = 400,
    NAV_WALL_CAUTION_TIMEOUT_MS_MAX = 1000,
    NAV_WALL_CAUTION_DELTA_MAX_MM_DEFAULT = 10,
    NAV_WALL_CAUTION_OUTPUT_LIMIT_PWM_DEFAULT = 1000,
    NAV_WALL_CAUTION_KP_PWM_PER_MM_DEFAULT = 12,
    NAV_WALL_CAUTION_KD_PWM_PER_MM_PER_TICK_DEFAULT = 600,
    NAV_SMOOTH_YAW_CARRY_MAX_ABS_DEG_DEFAULT = 15,
    NAV_SMOOTH_YAW_CARRY_MIN_ABS_DEG_DEFAULT = 3,
    NAV_SMOOTH_YAW_CARRY_SCALE_NUM_DEFAULT = 1,
    NAV_SMOOTH_YAW_CARRY_SCALE_DEN_DEFAULT = 1,
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
static NavAdvanceCorrectionSource forward_guidance_last_source = NAV_ADVANCE_CORRECTION_YAW_PD;
static bool forward_guidance_yaw_hold_initialized = false;
static q16_16_t forward_guidance_yaw_hold_deg_q16 = 0;
static uint16_t forward_guidance_yaw_hold_recapture_count = 0;
static NavSmoothPhase smooth_phase = NAV_SMOOTH_PHASE_NONE;
static NavAdvancePhase advance_phase = NAV_ADVANCE_PHASE_NONE;
static NavApproachFrontPhase approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
static NavCenterPivotPhase center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
static NavTurnDebug turn_debug = {0};
static NavSmoothTurnConfig smooth_turn_config = {0};
static NavWallPerception wall_perception = {0};
static uint16_t smooth_post_yaw_elapsed_ms = 0;
static uint16_t advance_elapsed_since_leave_start_line_ms = 0;
static uint16_t approach_front_elapsed_ms = 0;
static uint16_t approach_front_brake_elapsed_ms = 0;
static NavApproachFrontDoneReason approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
static uint16_t center_pivot_elapsed_ms = 0;
static uint16_t center_pivot_brake_elapsed_ms = 0;
static NavCenterPivotDoneReason center_pivot_done_reason = NAV_CENTER_PIVOT_DONE_NONE;
static bool center_pivot_front_seen_white = false;
static bool special_ignore_rear_until_white = false;
static bool special_confirmed = false;
static bool special_detection_started_on_rear_line = false;
static bool special_detection_enabled_for_current_motion = false;
static NavSpecialDetectionContext special_detection_context = NAV_SPECIAL_DETECT_DISABLED;
static bool special_aux_detection_enabled = false;
static bool special_aux_started_after_rear_line_left = false;
static bool special_aux_waiting_rear_white = false;
static int8_t special_mark_target_cell_x = -1;
static int8_t special_mark_target_cell_y = -1;
static NavSpecialMarkTargetSource special_mark_target_source =
    NAV_SPECIAL_MARK_TARGET_INVALID;
static NavAction last_special_mark_action = NAV_ACTION_NONE;
static bool advance_started_on_rear_line = false;
static NavAdvanceStartMode advance_start_mode = NAV_ADVANCE_START_REAR_LINE;
static bool advance_from_centered_waiting_rear_white = false;
static bool advance_front_diag_preview_armed = false;
static bool advance_front_diag_preview_latched = false;
static NavAdvanceGuidanceMode advance_guidance_mode = NAV_ADVANCE_GUIDANCE_WALL_ASSIST;
static NavPolicy nav_policy = NAV_POLICY_RIGHT_HAND_RULE;
static NavMapCandidateDebug map_candidate_debug = {0};
static NavPlanAction plan_queue[NAV_PLAN_MAX_ACTIONS];
static uint8_t plan_head = 0;
static uint8_t plan_count = 0;
static bool plan_overflow = false;
static NavRouteDebugSnapshot route_debug = {0};
static NavRouteEvalDebugSnapshot route_eval_debug = {0};
enum {
    NAV_ROUTE_MAX_STATES = NAV_MAP_MAX_WIDTH * NAV_MAP_MAX_HEIGHT * 4
};
typedef struct NavRouteWorkspace {
    uint8_t visited[NAV_ROUTE_MAX_STATES];
    int16_t parent[NAV_ROUTE_MAX_STATES];
    NavPlanAction parent_action[NAV_ROUTE_MAX_STATES];
    uint16_t queue[NAV_ROUTE_MAX_STATES];
    NavPlanAction reverse_actions[NAV_PLAN_MAX_ACTIONS];
} NavRouteWorkspace;
/* Shared static planner workspace for STM32 portability. The route planner is not reentrant. */
static NavRouteWorkspace route_workspace = {0};
static bool map_walls_recorded_for_current_pose = false;
static bool map_initial_wall_snapshot_pending = false;
static bool initial_special_snapshot_pending = false;
static bool initial_special_snapshot_done = false;
static bool rear_line_trusted_for_decision = false;
static NavRearLineTrustSource rear_line_trust_source = NAV_REAR_LINE_TRUST_NONE;
static NavSmoothFinalGuidanceSource smooth_final_hold_source = NAV_SMOOTH_FINAL_GUIDANCE_NONE;
static bool smooth_final_hold_initialized = false;
static uint16_t smooth_final_hold_recapture_count = 0;
static q16_16_t smooth_final_left_hold_mm_q16 = 0;
static q16_16_t smooth_final_right_hold_mm_q16 = 0;
static q16_16_t smooth_final_center_diff_hold_mm_q16 = 0;
static bool smooth_final_diag_hold_initialized = false;
static q16_16_t smooth_final_diag_left_hold_mm_q16 = 0;
static q16_16_t smooth_final_diag_right_hold_mm_q16 = 0;
static q16_16_t smooth_final_diag_center_diff_hold_mm_q16 = 0;
static uint16_t smooth_final_diag_hold_recapture_count = 0;
static q16_16_t smooth_final_yaw_hold_deg_q16 = 0;
static bool smooth_final_yaw_hold_initialized = false;
static uint16_t smooth_final_yaw_hold_recapture_count = 0;
static NavSmoothYawCarryConfig smooth_yaw_carry_config = {
    false,
    true,
    true,
    true,
    0,
    NAV_SMOOTH_YAW_CARRY_MAX_ABS_DEG_DEFAULT << 16,
    NAV_SMOOTH_YAW_CARRY_SCALE_NUM_DEFAULT,
    NAV_SMOOTH_YAW_CARRY_SCALE_DEN_DEFAULT
};
static q16_16_t smooth_final_entry_yaw_deg_q16 = 0;
static q16_16_t smooth_final_exit_yaw_deg_q16 = 0;
static q16_16_t smooth_final_exit_yaw_offset_deg_q16 = 0;
static bool smooth_final_diag_used = false;
static q16_16_t yaw_carry_candidate_entry_yaw_deg_q16 = 0;
static q16_16_t yaw_carry_candidate_exit_yaw_deg_q16 = 0;
static q16_16_t yaw_carry_candidate_offset_deg_q16 = 0;
static bool yaw_carry_candidate_diag_used = false;
static NavYawCarryCandidateSource yaw_carry_candidate_source = NAV_YAW_CARRY_SOURCE_NONE;
static bool smooth_yaw_carry_candidate_available = false;
static bool smooth_yaw_carry_pending = false;
static bool smooth_yaw_carry_used = false;
static q16_16_t smooth_yaw_carry_offset_deg_q16 = 0;
static NavSmoothYawCarryRejectedReason smooth_yaw_carry_rejected_reason =
    NAV_SMOOTH_YAW_CARRY_REJECT_NONE;
static q16_16_t advance_diag_preview_entry_yaw_deg_q16 = 0;
static bool advance_front_diag_used_for_yaw_carry = false;
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
static NavDiagonalGuidanceConfig diagonal_guidance_config = {
    NAV_DIAG_GUIDANCE_KP_PWM_PER_MM_DEFAULT,
    NAV_DIAG_GUIDANCE_KD_PWM_PER_MM_DEFAULT,
    NAV_DIAG_GUIDANCE_OUTPUT_LIMIT_PWM_DEFAULT,
    NAV_DIAG_GUIDANCE_ERROR_SCALE_NUM_DEFAULT,
    NAV_DIAG_GUIDANCE_ERROR_SCALE_DEN_DEFAULT,
    NAV_DIAG_GUIDANCE_TARGET_MM_DEFAULT,
    NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT
};
static q16_16_t diagonal_guidance_previous_error_q16 = 0;
static bool diagonal_guidance_has_previous_error = false;
static NavWallCautionConfig wall_caution_config = {
    true,
    NAV_WALL_CAUTION_TIMEOUT_MS_DEFAULT,
    NAV_WALL_CAUTION_DELTA_MAX_MM_DEFAULT,
    NAV_WALL_CAUTION_KP_PWM_PER_MM_DEFAULT,
    NAV_WALL_CAUTION_KD_PWM_PER_MM_PER_TICK_DEFAULT,
    NAV_WALL_CAUTION_OUTPUT_LIMIT_PWM_DEFAULT
};
static NavWallCautionConfidence wall_left_confidence = NAV_WALL_CAUTION_CONFIDENCE_LOST;
static NavWallCautionConfidence wall_right_confidence = NAV_WALL_CAUTION_CONFIDENCE_LOST;
static uint16_t wall_left_caution_elapsed_ms = 0;
static uint16_t wall_right_caution_elapsed_ms = 0;
static q16_16_t wall_left_caution_hold_mm_q16 = 0;
static q16_16_t wall_right_caution_hold_mm_q16 = 0;
static q16_16_t wall_left_caution_delta_mm_q16 = 0;
static q16_16_t wall_right_caution_delta_mm_q16 = 0;
static q16_16_t wall_caution_previous_error_q16 = 0;
static bool wall_caution_has_previous_error = false;
static NavWallCautionLossReason wall_caution_loss_reason = NAV_WALL_CAUTION_LOSS_NONE;

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

static NavMapDirection map_turn_back(NavMapDirection dir)
{
    return (NavMapDirection)((dir + 2) & 3);
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

static void clear_special_mark_debug(void)
{
    special_mark_target_cell_x = -1;
    special_mark_target_cell_y = -1;
    special_mark_target_source = NAV_SPECIAL_MARK_TARGET_INVALID;
    last_special_mark_action = NAV_ACTION_NONE;
}

static void reset_smooth_final_hold(void)
{
    smooth_final_hold_source = NAV_SMOOTH_FINAL_GUIDANCE_NONE;
    smooth_final_hold_initialized = false;
    smooth_final_hold_recapture_count = 0;
    smooth_final_left_hold_mm_q16 = 0;
    smooth_final_right_hold_mm_q16 = 0;
    smooth_final_center_diff_hold_mm_q16 = 0;
    smooth_final_diag_hold_initialized = false;
    smooth_final_diag_left_hold_mm_q16 = 0;
    smooth_final_diag_right_hold_mm_q16 = 0;
    smooth_final_diag_center_diff_hold_mm_q16 = 0;
    smooth_final_diag_hold_recapture_count = 0;
    smooth_final_yaw_hold_deg_q16 = 0;
    smooth_final_yaw_hold_initialized = false;
    smooth_final_yaw_hold_recapture_count = 0;
}

static void sync_smooth_yaw_carry_debug(void)
{
    turn_debug.smooth_yaw_carry_enabled = smooth_yaw_carry_config.enabled;
    turn_debug.smooth_yaw_carry_pending = smooth_yaw_carry_pending;
    turn_debug.smooth_yaw_carry_used = smooth_yaw_carry_used;
    turn_debug.smooth_yaw_carry_offset_deg_q16 = smooth_yaw_carry_offset_deg_q16;
    turn_debug.smooth_yaw_carry_entry_yaw_deg_q16 = yaw_carry_candidate_entry_yaw_deg_q16;
    turn_debug.smooth_yaw_carry_exit_yaw_deg_q16 = yaw_carry_candidate_exit_yaw_deg_q16;
    turn_debug.smooth_yaw_carry_diag_used = yaw_carry_candidate_diag_used;
    turn_debug.smooth_yaw_carry_candidate_source = yaw_carry_candidate_source;
    turn_debug.smooth_yaw_carry_rejected_reason = smooth_yaw_carry_rejected_reason;
    turn_debug.smooth_yaw_carry_only_setpoint = smooth_yaw_carry_config.only_setpoint;
    turn_debug.smooth_yaw_carry_require_diag = smooth_yaw_carry_config.require_diag;
    turn_debug.smooth_yaw_carry_allow_advance_preview =
        smooth_yaw_carry_config.allow_advance_preview;
    turn_debug.smooth_yaw_carry_min_abs_deg_q16 = smooth_yaw_carry_config.min_abs_deg_q16;
    turn_debug.smooth_yaw_carry_max_abs_deg_q16 = smooth_yaw_carry_config.max_abs_deg_q16;
    turn_debug.smooth_yaw_carry_offset_scale_num =
        smooth_yaw_carry_config.offset_scale_num;
    turn_debug.smooth_yaw_carry_offset_scale_den =
        smooth_yaw_carry_config.offset_scale_den;
}

static void reset_smooth_yaw_carry_runtime(void)
{
    smooth_final_entry_yaw_deg_q16 = 0;
    smooth_final_exit_yaw_deg_q16 = 0;
    smooth_final_exit_yaw_offset_deg_q16 = 0;
    smooth_final_diag_used = false;
    yaw_carry_candidate_entry_yaw_deg_q16 = 0;
    yaw_carry_candidate_exit_yaw_deg_q16 = 0;
    yaw_carry_candidate_offset_deg_q16 = 0;
    yaw_carry_candidate_diag_used = false;
    yaw_carry_candidate_source = NAV_YAW_CARRY_SOURCE_NONE;
    smooth_yaw_carry_candidate_available = false;
    smooth_yaw_carry_pending = false;
    smooth_yaw_carry_used = false;
    smooth_yaw_carry_offset_deg_q16 = 0;
    smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_NONE;
    sync_smooth_yaw_carry_debug();
}

static void sync_special_mark_debug(void)
{
    turn_debug.special_mark_target_cell_x = special_mark_target_cell_x;
    turn_debug.special_mark_target_cell_y = special_mark_target_cell_y;
    turn_debug.special_mark_target_source = special_mark_target_source;
    turn_debug.last_special_mark_action = last_special_mark_action;
}

static void set_rear_line_trust(bool trusted, NavRearLineTrustSource source)
{
    rear_line_trusted_for_decision = trusted;
    rear_line_trust_source = trusted ? source : NAV_REAR_LINE_TRUST_NONE;
    turn_debug.rear_line_trusted_for_decision = rear_line_trusted_for_decision;
    turn_debug.rear_line_trust_source = rear_line_trust_source;
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

static int16_t clamp_diag_correction(int32_t correction_pwm)
{
    if (correction_pwm > diagonal_guidance_config.correction_limit_pwm) {
        return diagonal_guidance_config.correction_limit_pwm;
    }
    if (correction_pwm < -diagonal_guidance_config.correction_limit_pwm) {
        return -diagonal_guidance_config.correction_limit_pwm;
    }

    return (int16_t)correction_pwm;
}

static int16_t clamp_wall_caution_correction(int32_t correction_pwm)
{
    if (correction_pwm > wall_caution_config.correction_limit_pwm) {
        return wall_caution_config.correction_limit_pwm;
    }
    if (correction_pwm < -wall_caution_config.correction_limit_pwm) {
        return -wall_caution_config.correction_limit_pwm;
    }

    return (int16_t)correction_pwm;
}

static q16_16_t mm_to_q16(int16_t mm)
{
    return (q16_16_t)mm << 16;
}

static q16_16_t smooth_final_scale_diag_error(q16_16_t error_q16)
{
    return (q16_16_t)(((int64_t)error_q16 * diagonal_guidance_config.error_scale_num)
        / diagonal_guidance_config.error_scale_den);
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

static void reset_diagonal_guidance_pd(void)
{
    diagonal_guidance_previous_error_q16 = 0;
    diagonal_guidance_has_previous_error = false;
}

static void reset_wall_caution_pd(void)
{
    wall_caution_previous_error_q16 = 0;
    wall_caution_has_previous_error = false;
}

static void reset_wall_caution_state(NavWallCautionLossReason reason)
{
    wall_left_confidence = NAV_WALL_CAUTION_CONFIDENCE_LOST;
    wall_right_confidence = NAV_WALL_CAUTION_CONFIDENCE_LOST;
    wall_left_caution_elapsed_ms = 0;
    wall_right_caution_elapsed_ms = 0;
    wall_left_caution_hold_mm_q16 = 0;
    wall_right_caution_hold_mm_q16 = 0;
    wall_left_caution_delta_mm_q16 = 0;
    wall_right_caution_delta_mm_q16 = 0;
    wall_caution_loss_reason = reason;
    reset_wall_caution_pd();
}

static void reset_forward_guidance_yaw_hold(void)
{
    forward_guidance_last_source = NAV_ADVANCE_CORRECTION_YAW_PD;
    forward_guidance_yaw_hold_initialized = false;
    forward_guidance_yaw_hold_deg_q16 = 0;
    forward_guidance_yaw_hold_recapture_count = 0;
}

static void capture_forward_guidance_yaw_hold(q16_16_t yaw_deg_q16)
{
    forward_guidance_yaw_hold_initialized = true;
    forward_guidance_yaw_hold_deg_q16 = yaw_deg_q16;
    if (forward_guidance_yaw_hold_recapture_count < UINT16_MAX) {
        ++forward_guidance_yaw_hold_recapture_count;
    }
    PID_Reset(&advance_yaw_pid);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, forward_guidance_yaw_hold_deg_q16);
}

static void update_one_wall_caution_state(bool lateral_valid,
                                          bool confirmed_valid,
                                          q16_16_t lateral_mm_q16,
                                          NavWallCautionConfidence *confidence,
                                          uint16_t *elapsed_ms,
                                          q16_16_t *hold_mm_q16,
                                          q16_16_t *delta_mm_q16)
{
    if (!wall_caution_config.enabled) {
        *confidence = NAV_WALL_CAUTION_CONFIDENCE_LOST;
        *elapsed_ms = 0;
        *delta_mm_q16 = 0;
        wall_caution_loss_reason = NAV_WALL_CAUTION_LOSS_DISABLED;
        return;
    }

    if (confirmed_valid) {
        *confidence = NAV_WALL_CAUTION_CONFIDENCE_CONFIRMED;
        *elapsed_ms = 0;
        *hold_mm_q16 = lateral_mm_q16;
        *delta_mm_q16 = 0;
        wall_caution_loss_reason = NAV_WALL_CAUTION_LOSS_NONE;
        return;
    }

    if (*confidence == NAV_WALL_CAUTION_CONFIDENCE_CONFIRMED && lateral_valid) {
        *confidence = NAV_WALL_CAUTION_CONFIDENCE_CAUTION;
        *elapsed_ms = 0;
        *hold_mm_q16 = lateral_mm_q16;
        *delta_mm_q16 = 0;
        wall_caution_loss_reason = NAV_WALL_CAUTION_LOSS_NONE;
        reset_wall_caution_pd();
        return;
    }

    if (*confidence != NAV_WALL_CAUTION_CONFIDENCE_CAUTION) {
        *confidence = NAV_WALL_CAUTION_CONFIDENCE_LOST;
        *elapsed_ms = 0;
        *delta_mm_q16 = 0;
        return;
    }

    if (!lateral_valid) {
        *confidence = NAV_WALL_CAUTION_CONFIDENCE_LOST;
        *elapsed_ms = 0;
        *delta_mm_q16 = 0;
        wall_caution_loss_reason = NAV_WALL_CAUTION_LOSS_LATERAL_LOST;
        reset_wall_caution_pd();
        return;
    }

    *elapsed_ms = (uint16_t)clamp_i16((int16_t)(*elapsed_ms + 10),
                                      0,
                                      NAV_WALL_CAUTION_TIMEOUT_MS_MAX);
    *delta_mm_q16 = lateral_mm_q16 - *hold_mm_q16;
    if (abs_q16(*delta_mm_q16) > mm_to_q16(wall_caution_config.delta_max_mm)) {
        *confidence = NAV_WALL_CAUTION_CONFIDENCE_LOST;
        *elapsed_ms = 0;
        wall_caution_loss_reason = NAV_WALL_CAUTION_LOSS_DELTA_MAX;
        reset_wall_caution_pd();
        return;
    }

    if (*elapsed_ms > wall_caution_config.timeout_ms) {
        *confidence = NAV_WALL_CAUTION_CONFIDENCE_LOST;
        *elapsed_ms = 0;
        wall_caution_loss_reason = NAV_WALL_CAUTION_LOSS_TIMEOUT;
        reset_wall_caution_pd();
    }
}

static void update_wall_caution_state(bool wall_left_valid,
                                      bool wall_right_valid,
                                      bool follow_left_valid,
                                      bool follow_right_valid)
{
    if (current_action != NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK
        || advance_phase != NAV_ADVANCE_PHASE_SEEK_TARGET_LINE) {
        reset_wall_caution_state(NAV_WALL_CAUTION_LOSS_ACTION_END);
        return;
    }

    update_one_wall_caution_state(wall_left_valid,
                                  follow_left_valid,
                                  wall_perception.left_mm_q16,
                                  &wall_left_confidence,
                                  &wall_left_caution_elapsed_ms,
                                  &wall_left_caution_hold_mm_q16,
                                  &wall_left_caution_delta_mm_q16);
    update_one_wall_caution_state(wall_right_valid,
                                  follow_right_valid,
                                  wall_perception.right_mm_q16,
                                  &wall_right_confidence,
                                  &wall_right_caution_elapsed_ms,
                                  &wall_right_caution_hold_mm_q16,
                                  &wall_right_caution_delta_mm_q16);
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

static void reset_diagonal_guidance_config(void)
{
    diagonal_guidance_config.kp_pwm_per_mm = NAV_DIAG_GUIDANCE_KP_PWM_PER_MM_DEFAULT;
    diagonal_guidance_config.kd_pwm_per_mm_per_tick = NAV_DIAG_GUIDANCE_KD_PWM_PER_MM_DEFAULT;
    diagonal_guidance_config.correction_limit_pwm = NAV_DIAG_GUIDANCE_OUTPUT_LIMIT_PWM_DEFAULT;
    diagonal_guidance_config.error_scale_num = NAV_DIAG_GUIDANCE_ERROR_SCALE_NUM_DEFAULT;
    diagonal_guidance_config.error_scale_den = NAV_DIAG_GUIDANCE_ERROR_SCALE_DEN_DEFAULT;
    diagonal_guidance_config.target_mm = NAV_DIAG_GUIDANCE_TARGET_MM_DEFAULT;
    diagonal_guidance_config.smooth_final_mode = NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT;
}

static void reset_wall_caution_config(void)
{
    wall_caution_config.enabled = true;
    wall_caution_config.timeout_ms = NAV_WALL_CAUTION_TIMEOUT_MS_DEFAULT;
    wall_caution_config.delta_max_mm = NAV_WALL_CAUTION_DELTA_MAX_MM_DEFAULT;
    wall_caution_config.kp_pwm_per_mm = NAV_WALL_CAUTION_KP_PWM_PER_MM_DEFAULT;
    wall_caution_config.kd_pwm_per_mm_per_tick = NAV_WALL_CAUTION_KD_PWM_PER_MM_PER_TICK_DEFAULT;
    wall_caution_config.correction_limit_pwm = NAV_WALL_CAUTION_OUTPUT_LIMIT_PWM_DEFAULT;
}

static void reset_smooth_yaw_carry_config(void)
{
    smooth_yaw_carry_config.enabled = true;
    smooth_yaw_carry_config.only_setpoint = true;
    smooth_yaw_carry_config.require_diag = true;
    smooth_yaw_carry_config.allow_advance_preview = true;
    smooth_yaw_carry_config.min_abs_deg_q16 = NAV_SMOOTH_YAW_CARRY_MIN_ABS_DEG_DEFAULT << 16;
    smooth_yaw_carry_config.max_abs_deg_q16 =
        NAV_SMOOTH_YAW_CARRY_MAX_ABS_DEG_DEFAULT << 16;
    smooth_yaw_carry_config.offset_scale_num = NAV_SMOOTH_YAW_CARRY_SCALE_NUM_DEFAULT;
    smooth_yaw_carry_config.offset_scale_den = NAV_SMOOTH_YAW_CARRY_SCALE_DEN_DEFAULT;
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
    case NAV_ACTION_CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE:
        break;
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
    case NAV_ACTION_CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE:
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

static void record_initial_special_snapshot_if_ready(const RobotSensors *sensors)
{
    if (sensors == 0 || !initial_special_snapshot_pending) {
        return;
    }

    initial_special_snapshot_pending = false;
    initial_special_snapshot_done = true;
    if (sensors->floor_front_black && sensors->floor_rear_black) {
        NavMapDebugSnapshot map_debug = {0};
        nav_map_get_debug_snapshot(&map_debug);
        special_mark_target_cell_x = map_debug.cell_x;
        special_mark_target_cell_y = map_debug.cell_y;
        special_mark_target_source = NAV_SPECIAL_MARK_TARGET_CURRENT_CELL;
        last_special_mark_action = NAV_ACTION_NONE;
        (void)nav_map_mark_current_cell_special();
        special_confirmed = true;
        set_rear_line_trust(false, NAV_REAR_LINE_TRUST_NONE);
    } else if (sensors->floor_rear_black && !sensors->floor_front_black) {
        set_rear_line_trust(true, NAV_REAR_LINE_TRUST_INITIAL_REAR_LINE);
    } else {
        set_rear_line_trust(false, NAV_REAR_LINE_TRUST_NONE);
    }
    turn_debug.special_candidate = sensors->floor_front_black && sensors->floor_rear_black;
    turn_debug.special_confirmed = special_confirmed;
    turn_debug.initial_special_snapshot_pending = initial_special_snapshot_pending;
    turn_debug.initial_special_snapshot_done = initial_special_snapshot_done;
    sync_special_mark_debug();
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
    turn_debug.smooth_final_guidance_source = NAV_SMOOTH_FINAL_GUIDANCE_NONE;
    turn_debug.smooth_final_hold_initialized = smooth_final_hold_initialized;
    turn_debug.smooth_final_hold_source = smooth_final_hold_source;
    turn_debug.smooth_final_hold_recapture_count = smooth_final_hold_recapture_count;
    turn_debug.smooth_final_left_hold_mm_q16 = smooth_final_left_hold_mm_q16;
    turn_debug.smooth_final_right_hold_mm_q16 = smooth_final_right_hold_mm_q16;
    turn_debug.smooth_final_center_diff_hold_mm_q16 = smooth_final_center_diff_hold_mm_q16;
    turn_debug.smooth_final_wall_error_mm_q16 = 0;
    turn_debug.smooth_final_wall_correction_pwm = 0;
    turn_debug.smooth_final_yaw_correction_pwm = 0;
    turn_debug.smooth_final_yaw_hold_deg_q16 = smooth_final_yaw_hold_deg_q16;
    turn_debug.smooth_final_yaw_hold_initialized = smooth_final_yaw_hold_initialized;
    turn_debug.smooth_final_yaw_hold_recapture_count = smooth_final_yaw_hold_recapture_count;
    turn_debug.smooth_final_yaw_error_deg_q16 = 0;
    turn_debug.smooth_final_applied_correction_pwm = 0;
    turn_debug.smooth_final_diag_left_valid = false;
    turn_debug.smooth_final_diag_right_valid = false;
    turn_debug.smooth_final_diag_left_mm_q16 = 0;
    turn_debug.smooth_final_diag_right_mm_q16 = 0;
    turn_debug.smooth_final_diag_target_mm_q16 =
        mm_to_q16(diagonal_guidance_config.target_mm);
    turn_debug.smooth_final_diag_error_scale_q16 =
        INT_TO_FIXED(diagonal_guidance_config.error_scale_num)
        / diagonal_guidance_config.error_scale_den;
    turn_debug.smooth_final_diag_mode = diagonal_guidance_config.smooth_final_mode;
    turn_debug.smooth_final_diag_hold_initialized = smooth_final_diag_hold_initialized;
    turn_debug.smooth_final_diag_left_hold_mm_q16 = smooth_final_diag_left_hold_mm_q16;
    turn_debug.smooth_final_diag_right_hold_mm_q16 = smooth_final_diag_right_hold_mm_q16;
    turn_debug.smooth_final_diag_center_diff_hold_mm_q16 =
        smooth_final_diag_center_diff_hold_mm_q16;
    turn_debug.smooth_final_diag_hold_recapture_count =
        smooth_final_diag_hold_recapture_count;
    turn_debug.smooth_final_diag_raw_error_mm_q16 = 0;
    turn_debug.smooth_final_diag_error_mm_q16 = 0;
    turn_debug.smooth_final_follow_left_valid = false;
    turn_debug.smooth_final_follow_right_valid = false;
    sync_smooth_yaw_carry_debug();
    turn_debug.advance_phase = NAV_ADVANCE_PHASE_NONE;
    turn_debug.advance_done_reason = NAV_ADVANCE_DONE_NONE;
    turn_debug.rear_black_for_line = false;
    turn_debug.floor_rear_black = false;
    turn_debug.advance_started_on_rear_line = advance_started_on_rear_line;
    turn_debug.advance_start_mode = advance_start_mode;
    turn_debug.advance_from_centered_waiting_rear_white =
        advance_from_centered_waiting_rear_white;
    turn_debug.special_candidate = false;
    turn_debug.special_confirmed = special_confirmed;
    turn_debug.special_ignore_rear_until_white = special_ignore_rear_until_white;
    turn_debug.special_detection_started_on_rear_line =
        special_detection_started_on_rear_line;
    turn_debug.special_detection_enabled_for_current_motion =
        special_detection_enabled_for_current_motion;
    turn_debug.special_detection_context = special_detection_context;
    turn_debug.special_aux_detection_enabled = special_aux_detection_enabled;
    turn_debug.special_aux_started_after_rear_line_left =
        special_aux_started_after_rear_line_left;
    turn_debug.initial_special_snapshot_pending = initial_special_snapshot_pending;
    turn_debug.initial_special_snapshot_done = initial_special_snapshot_done;
    turn_debug.rear_line_trusted_for_decision = rear_line_trusted_for_decision;
    turn_debug.rear_line_trust_source = rear_line_trust_source;
    sync_special_mark_debug();
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
    turn_debug.center_pivot_phase = center_pivot_phase;
    turn_debug.center_pivot_done_reason = center_pivot_done_reason;
    turn_debug.center_pivot_elapsed_ms = center_pivot_elapsed_ms;
    turn_debug.center_pivot_brake_elapsed_ms = center_pivot_brake_elapsed_ms;
    turn_debug.center_pivot_base_left_pwm = 0;
    turn_debug.center_pivot_base_right_pwm = 0;
    turn_debug.center_pivot_correction_pwm = 0;
    turn_debug.center_pivot_front_black = false;
    turn_debug.center_pivot_rear_black = false;
    turn_debug.center_pivot_front_seen_white = center_pivot_front_seen_white;
    turn_debug.advance_yaw_setpoint_deg_q16 = 0;
    turn_debug.advance_yaw_measured_deg_q16 = 0;
    turn_debug.advance_yaw_error_deg_q16 = 0;
    turn_debug.advance_yaw_pid_output_q16 = 0;
    turn_debug.advance_yaw_correction_pwm = 0;
    turn_debug.advance_yaw_hold_deg_q16 = forward_guidance_yaw_hold_deg_q16;
    turn_debug.advance_yaw_hold_initialized = forward_guidance_yaw_hold_initialized;
    turn_debug.advance_yaw_hold_recapture_count = forward_guidance_yaw_hold_recapture_count;
    turn_debug.advance_yaw_hold_error_deg_q16 = 0;
    turn_debug.advance_guidance_last_source = forward_guidance_last_source;
    turn_debug.advance_yaw_kp_q16 = advance_yaw_pid_config.kp_q16;
    turn_debug.advance_yaw_ki_q16 = advance_yaw_pid_config.ki_q16;
    turn_debug.advance_yaw_kd_q16 = advance_yaw_pid_config.kd_q16;
    turn_debug.advance_yaw_output_limit_pwm = (int16_t)advance_yaw_pid_config.output_limit_pwm;
    turn_debug.advance_base_left_pwm = 0;
    turn_debug.advance_base_right_pwm = 0;
    turn_debug.advance_guidance_mode = advance_guidance_mode;
    turn_debug.advance_final_correction_source = NAV_ADVANCE_CORRECTION_YAW_PD;
    turn_debug.advance_front_diag_preview_armed = advance_front_diag_preview_armed;
    turn_debug.advance_front_diag_preview_latched = advance_front_diag_preview_latched;
    turn_debug.advance_front_diag_preview_active = false;
    turn_debug.advance_front_diag_source = NAV_ADVANCE_FRONT_DIAG_NONE;
    turn_debug.advance_front_diag_raw_error_mm_q16 = 0;
    turn_debug.advance_front_diag_error_mm_q16 = 0;
    turn_debug.advance_front_diag_left_valid = false;
    turn_debug.advance_front_diag_right_valid = false;
    turn_debug.advance_wall_left_valid = false;
    turn_debug.advance_wall_right_valid = false;
    turn_debug.advance_diag_left_valid = false;
    turn_debug.advance_diag_right_valid = false;
    turn_debug.advance_follow_left_valid = false;
    turn_debug.advance_follow_right_valid = false;
    turn_debug.wall_caution_enabled = wall_caution_config.enabled;
    turn_debug.wall_left_confidence = wall_left_confidence;
    turn_debug.wall_right_confidence = wall_right_confidence;
    turn_debug.wall_left_caution_elapsed_ms = wall_left_caution_elapsed_ms;
    turn_debug.wall_right_caution_elapsed_ms = wall_right_caution_elapsed_ms;
    turn_debug.wall_left_caution_hold_mm_q16 = wall_left_caution_hold_mm_q16;
    turn_debug.wall_right_caution_hold_mm_q16 = wall_right_caution_hold_mm_q16;
    turn_debug.wall_left_caution_delta_mm_q16 = wall_left_caution_delta_mm_q16;
    turn_debug.wall_right_caution_delta_mm_q16 = wall_right_caution_delta_mm_q16;
    turn_debug.wall_caution_correction_pwm = 0;
    turn_debug.wall_caution_loss_reason = wall_caution_loss_reason;
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
    turn_debug.diag_guidance_kp_pwm_per_mm = diagonal_guidance_config.kp_pwm_per_mm;
    turn_debug.diag_guidance_kd_pwm_per_mm_per_tick =
        diagonal_guidance_config.kd_pwm_per_mm_per_tick;
    turn_debug.diag_guidance_correction_limit_pwm =
        diagonal_guidance_config.correction_limit_pwm;
    turn_debug.diag_guidance_error_scale_num = diagonal_guidance_config.error_scale_num;
    turn_debug.diag_guidance_error_scale_den = diagonal_guidance_config.error_scale_den;
    turn_debug.diag_guidance_target_mm = diagonal_guidance_config.target_mm;
    turn_debug.diag_guidance_smooth_final_mode = diagonal_guidance_config.smooth_final_mode;
    turn_debug.diag_guidance_p_term_pwm = 0;
    turn_debug.diag_guidance_d_term_pwm = 0;
    turn_debug.diag_guidance_correction_pwm = 0;
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
    turn_debug.last_center_pivot_done_reason = NAV_CENTER_PIVOT_DONE_NONE;
}

static void reset_turn_debug(void)
{
    clear_live_turn_debug();
    clear_completed_turn_debug();
}

static RobotCommand guided_forward_command(const RobotSensors *sensors,
                                           int16_t base_left_pwm,
                                           int16_t base_right_pwm);
static void set_yaw_carry_candidate(NavYawCarryCandidateSource source,
                                    q16_16_t entry_yaw_deg_q16,
                                    q16_16_t exit_yaw_deg_q16,
                                    bool diag_used);

static void reset_special_detection_state(void)
{
    advance_elapsed_since_leave_start_line_ms = 0;
    special_ignore_rear_until_white = false;
    special_confirmed = false;
    special_detection_started_on_rear_line = false;
    special_detection_enabled_for_current_motion = false;
    special_detection_context = NAV_SPECIAL_DETECT_DISABLED;
    special_aux_detection_enabled = false;
    special_aux_started_after_rear_line_left = false;
    special_aux_waiting_rear_white = false;
    advance_started_on_rear_line = false;
    advance_from_centered_waiting_rear_white = false;
    turn_debug.special_candidate = false;
    turn_debug.special_confirmed = false;
    turn_debug.special_ignore_rear_until_white = false;
    turn_debug.special_detection_started_on_rear_line = false;
    turn_debug.special_detection_enabled_for_current_motion = false;
    turn_debug.special_detection_context = special_detection_context;
    turn_debug.special_aux_detection_enabled = false;
    turn_debug.special_aux_started_after_rear_line_left = false;
    turn_debug.initial_special_snapshot_pending = initial_special_snapshot_pending;
    turn_debug.initial_special_snapshot_done = initial_special_snapshot_done;
    turn_debug.rear_line_trusted_for_decision = rear_line_trusted_for_decision;
    turn_debug.rear_line_trust_source = rear_line_trust_source;
    turn_debug.rear_black_for_line = false;
    turn_debug.floor_rear_black = false;
    turn_debug.advance_started_on_rear_line = false;
    turn_debug.advance_start_mode = advance_start_mode;
    turn_debug.advance_from_centered_waiting_rear_white = false;
    turn_debug.advance_elapsed_since_leave_start_line_ms = 0;
}

static void begin_special_detection_for_motion(bool started_on_rear_line)
{
    special_detection_started_on_rear_line = started_on_rear_line;
    special_detection_enabled_for_current_motion = started_on_rear_line;
    special_detection_context = NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL;
    special_aux_detection_enabled = false;
    special_aux_started_after_rear_line_left = false;
    special_aux_waiting_rear_white = false;
    turn_debug.special_detection_started_on_rear_line = started_on_rear_line;
    turn_debug.special_detection_enabled_for_current_motion =
        special_detection_enabled_for_current_motion;
    turn_debug.special_detection_context = special_detection_context;
    turn_debug.special_aux_detection_enabled = special_aux_detection_enabled;
    turn_debug.special_aux_started_after_rear_line_left =
        special_aux_started_after_rear_line_left;
}

static void begin_special_detection_for_aux_translation(bool starts_on_rear_black)
{
    special_detection_started_on_rear_line = false;
    special_detection_enabled_for_current_motion = false;
    special_detection_context = NAV_SPECIAL_DETECT_IN_CELL_AUX_TRANSLATION;
    special_aux_waiting_rear_white = starts_on_rear_black;
    special_aux_detection_enabled = !starts_on_rear_black;
    special_aux_started_after_rear_line_left = !starts_on_rear_black;
    turn_debug.special_detection_started_on_rear_line = false;
    turn_debug.special_detection_enabled_for_current_motion = false;
    turn_debug.special_detection_context = special_detection_context;
    turn_debug.special_aux_detection_enabled = special_aux_detection_enabled;
    turn_debug.special_aux_started_after_rear_line_left =
        special_aux_started_after_rear_line_left;
}

void nav_core_plan_clear(void)
{
    for (uint8_t i = 0; i < NAV_PLAN_MAX_ACTIONS; ++i) {
        plan_queue[i] = NAV_PLAN_ACTION_NONE;
    }
    plan_head = 0;
    plan_count = 0;
    plan_overflow = false;
}

bool nav_core_plan_push(NavPlanAction action)
{
    if (action == NAV_PLAN_ACTION_NONE) {
        return false;
    }

    if (plan_count >= NAV_PLAN_MAX_ACTIONS) {
        plan_overflow = true;
        return false;
    }

    const uint8_t tail = (uint8_t)((plan_head + plan_count) % NAV_PLAN_MAX_ACTIONS);
    plan_queue[tail] = action;
    ++plan_count;
    return true;
}

uint8_t nav_core_plan_count(void)
{
    return plan_count;
}

bool nav_core_plan_is_empty(void)
{
    return plan_count == 0;
}

NavPlanAction nav_core_plan_peek_next(void)
{
    if (plan_count == 0) {
        return NAV_PLAN_ACTION_NONE;
    }
    return plan_queue[plan_head];
}

NavPlanAction nav_core_plan_pop_next(void)
{
    if (plan_count == 0) {
        return NAV_PLAN_ACTION_NONE;
    }

    const NavPlanAction action = plan_queue[plan_head];
    plan_queue[plan_head] = NAV_PLAN_ACTION_NONE;
    plan_head = (uint8_t)((plan_head + 1) % NAV_PLAN_MAX_ACTIONS);
    --plan_count;
    return action;
}

void nav_core_plan_debug_snapshot(NavPlanDebugSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    snapshot->capacity = NAV_PLAN_MAX_ACTIONS;
    snapshot->count = plan_count;
    snapshot->head = plan_head;
    snapshot->tail = (uint8_t)((plan_head + plan_count) % NAV_PLAN_MAX_ACTIONS);
    snapshot->next_action = nav_core_plan_peek_next();
    snapshot->overflow = plan_overflow;
}

void nav_core_route_clear_debug(void)
{
    route_debug = (NavRouteDebugSnapshot){0};
    route_debug.status = NAV_ROUTE_STATUS_IDLE;
    route_debug.target_cell_x = -1;
    route_debug.target_cell_y = -1;
    route_debug.start_cell_x = -1;
    route_debug.start_cell_y = -1;
    route_debug.start_dir = NAV_DIR_NORTH;
    route_debug.first_action = NAV_PLAN_ACTION_NONE;
    route_debug.last_action = NAV_PLAN_ACTION_NONE;
    route_debug.frontier_target_cell_x = -1;
    route_debug.frontier_target_cell_y = -1;
    route_debug.frontier_target_dir = NAV_DIR_NORTH;
    route_debug.frontier_exit_dir_absolute = NAV_DIR_NORTH;
    route_debug.frontier_exit_relative = NAV_FRONTIER_EXIT_NONE;
    route_debug.frontier_neighbor_cell_x = -1;
    route_debug.frontier_neighbor_cell_y = -1;
}

static void nav_core_route_eval_clear_debug(void)
{
    route_eval_debug = (NavRouteEvalDebugSnapshot){0};
    route_eval_debug.status = NAV_ROUTE_STATUS_IDLE;
    route_eval_debug.target_cell_x = -1;
    route_eval_debug.target_cell_y = -1;
    route_eval_debug.target_dir_mask = 0u;
    route_eval_debug.found_target_dir = -1;
    route_eval_debug.first_action = NAV_PLAN_ACTION_NONE;
    route_eval_debug.last_action = NAV_PLAN_ACTION_NONE;
}

void nav_core_get_route_debug(NavRouteDebugSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    *snapshot = route_debug;
}

void nav_core_route_eval_get_debug(NavRouteEvalDebugSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    *snapshot = route_eval_debug;
}

static uint16_t route_state_index(int8_t cell_x,
                                  int8_t cell_y,
                                  NavMapDirection dir,
                                  uint8_t width)
{
    return (uint16_t)((((uint16_t)cell_y * width) + (uint8_t)cell_x) * 4u + (uint8_t)dir);
}

static void route_decode_state(uint16_t index,
                               uint8_t width,
                               int8_t *cell_x,
                               int8_t *cell_y,
                               NavMapDirection *dir)
{
    const uint16_t cell_index = (uint16_t)(index / 4u);
    if (dir != 0) {
        *dir = (NavMapDirection)(index & 3u);
    }
    if (cell_x != 0) {
        *cell_x = (int8_t)(cell_index % width);
    }
    if (cell_y != 0) {
        *cell_y = (int8_t)(cell_index / width);
    }
}

static uint8_t route_wall_bit(NavMapDirection dir)
{
    return (uint8_t)(1u << (uint8_t)dir);
}

static bool route_can_move_from_cell(int8_t cell_x,
                                     int8_t cell_y,
                                     NavMapDirection move_dir,
                                     const NavMapDebugSnapshot *map_debug)
{
    int8_t next_x = cell_x;
    int8_t next_y = cell_y;
    map_neighbor_for_dir(cell_x, cell_y, move_dir, &next_x, &next_y);
    if (!map_cell_is_inside(map_debug, next_x, next_y)) {
        return false;
    }

    NavMapCell next_cell = {0};
    if (!nav_map_get_cell(next_x, next_y, &next_cell) || !next_cell.visited) {
        return false;
    }

    NavMapCell cell = {0};
    if (!nav_map_get_cell(cell_x, cell_y, &cell)) {
        return false;
    }

    const uint8_t bit = route_wall_bit(move_dir);
    return (cell.walls_known & bit) != 0
        && (cell.walls_present & bit) == 0;
}

static bool route_next_state_for_action(int8_t cell_x,
                                        int8_t cell_y,
                                        NavMapDirection dir,
                                        NavPlanAction action,
                                        const NavMapDebugSnapshot *map_debug,
                                        int8_t *next_x,
                                        int8_t *next_y,
                                        NavMapDirection *next_dir)
{
    NavMapDirection move_dir = dir;
    switch (action) {
    case NAV_PLAN_ACTION_ADVANCE_LINE:
        move_dir = dir;
        break;
    case NAV_PLAN_ACTION_SMOOTH_LEFT:
        move_dir = map_turn_left(dir);
        break;
    case NAV_PLAN_ACTION_SMOOTH_RIGHT:
        move_dir = map_turn_right(dir);
        break;
    case NAV_PLAN_ACTION_CENTER_AND_PIVOT_180:
        *next_x = cell_x;
        *next_y = cell_y;
        *next_dir = map_turn_back(dir);
        return true;
    case NAV_PLAN_ACTION_NONE:
    case NAV_PLAN_ACTION_PIVOT_180:
    case NAV_PLAN_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT:
        return false;
    }

    if (!route_can_move_from_cell(cell_x, cell_y, move_dir, map_debug)) {
        return false;
    }

    *next_x = cell_x;
    *next_y = cell_y;
    map_neighbor_for_dir(cell_x, cell_y, move_dir, next_x, next_y);
    *next_dir = move_dir;
    return true;
}

static bool route_cell_has_known_open_exit_to_unvisited(int8_t cell_x,
                                                        int8_t cell_y,
                                                        NavMapDirection exit_dir,
                                                        const NavMapDebugSnapshot *map_debug,
                                                        int8_t *neighbor_x,
                                                        int8_t *neighbor_y)
{
    int8_t next_x = cell_x;
    int8_t next_y = cell_y;
    map_neighbor_for_dir(cell_x, cell_y, exit_dir, &next_x, &next_y);
    if (!map_cell_is_inside(map_debug, next_x, next_y)) {
        return false;
    }

    NavMapCell cell = {0};
    if (!nav_map_get_cell(cell_x, cell_y, &cell) || !cell.visited) {
        return false;
    }

    const uint8_t bit = route_wall_bit(exit_dir);
    if ((cell.walls_known & bit) == 0 || (cell.walls_present & bit) != 0) {
        return false;
    }

    NavMapCell neighbor = {0};
    if (!nav_map_get_cell(next_x, next_y, &neighbor) || neighbor.visited) {
        return false;
    }

    if (neighbor_x != 0) {
        *neighbor_x = next_x;
    }
    if (neighbor_y != 0) {
        *neighbor_y = next_y;
    }
    return true;
}

static bool route_state_is_usable_frontier(int8_t cell_x,
                                           int8_t cell_y,
                                           NavMapDirection dir,
                                           const NavMapDebugSnapshot *map_debug,
                                           NavMapDirection *exit_dir_absolute,
                                           NavFrontierExitRelative *exit_relative,
                                           int8_t *neighbor_x,
                                           int8_t *neighbor_y)
{
    const NavMapDirection dirs[] = {
        map_turn_right(dir),
        dir,
        map_turn_left(dir)
    };
    const NavFrontierExitRelative relatives[] = {
        NAV_FRONTIER_EXIT_RIGHT,
        NAV_FRONTIER_EXIT_FRONT,
        NAV_FRONTIER_EXIT_LEFT
    };

    for (uint8_t i = 0; i < 3; ++i) {
        int8_t next_x = -1;
        int8_t next_y = -1;
        if (!route_cell_has_known_open_exit_to_unvisited(cell_x,
                                                         cell_y,
                                                         dirs[i],
                                                         map_debug,
                                                         &next_x,
                                                         &next_y)) {
            continue;
        }

        if (exit_dir_absolute != 0) {
            *exit_dir_absolute = dirs[i];
        }
        if (exit_relative != 0) {
            *exit_relative = relatives[i];
        }
        if (neighbor_x != 0) {
            *neighbor_x = next_x;
        }
        if (neighbor_y != 0) {
            *neighbor_y = next_y;
        }
        return true;
    }

    return false;
}

static uint16_t route_count_visited_frontier_cells(const NavMapDebugSnapshot *map_debug)
{
    uint16_t count = 0;
    for (int8_t y = 0; y < (int8_t)map_debug->height; ++y) {
        for (int8_t x = 0; x < (int8_t)map_debug->width; ++x) {
            NavMapCell cell = {0};
            if (!nav_map_get_cell(x, y, &cell) || !cell.visited) {
                continue;
            }

            bool is_frontier = false;
            for (uint8_t dir = 0; dir < 4; ++dir) {
                if (route_cell_has_known_open_exit_to_unvisited(x,
                                                                y,
                                                                (NavMapDirection)dir,
                                                                map_debug,
                                                                0,
                                                                0)) {
                    is_frontier = true;
                    break;
                }
            }
            if (is_frontier) {
                ++count;
            }
        }
    }
    return count;
}

static void route_workspace_reset(void)
{
    for (uint16_t i = 0; i < NAV_ROUTE_MAX_STATES; ++i) {
        route_workspace.visited[i] = 0u;
        route_workspace.parent[i] = -1;
        route_workspace.parent_action[i] = NAV_PLAN_ACTION_NONE;
        route_workspace.queue[i] = 0u;
    }
    for (uint8_t i = 0; i < NAV_PLAN_MAX_ACTIONS; ++i) {
        route_workspace.reverse_actions[i] = NAV_PLAN_ACTION_NONE;
    }
}

static NavRouteStatus route_collect_plan_from_found_state(int16_t found_index,
                                                          uint16_t start_index,
                                                          const int16_t *parent,
                                                          const NavPlanAction *parent_action,
                                                          bool load_queue,
                                                          uint16_t *route_length_out,
                                                          NavPlanAction *first_action_out,
                                                          NavPlanAction *last_action_out)
{
    uint8_t route_length = 0;
    int16_t cursor = found_index;
    while (cursor >= 0 && cursor != (int16_t)start_index) {
        if (route_length >= NAV_PLAN_MAX_ACTIONS) {
            if (route_length_out != 0) {
                *route_length_out = route_length;
            }
            return NAV_ROUTE_STATUS_ROUTE_TOO_LONG;
        }

        route_workspace.reverse_actions[route_length++] = parent_action[cursor];
        cursor = parent[cursor];
    }

    if (load_queue) {
        for (uint8_t i = 0; i < route_length; ++i) {
            const NavPlanAction action = route_workspace.reverse_actions[route_length - 1u - i];
            if (!nav_core_plan_push(action)) {
                if (route_length_out != 0) {
                    *route_length_out = i;
                }
                return NAV_ROUTE_STATUS_QUEUE_OVERFLOW;
            }
        }
    }

    if (route_length_out != 0) {
        *route_length_out = route_length;
    }
    if (first_action_out != 0) {
        *first_action_out = route_length > 0
            ? route_workspace.reverse_actions[route_length - 1u]
            : NAV_PLAN_ACTION_NONE;
    }
    if (last_action_out != 0) {
        *last_action_out = route_length > 0
            ? route_workspace.reverse_actions[0]
            : NAV_PLAN_ACTION_NONE;
    }
    return NAV_ROUTE_STATUS_FOUND;
}

static NavRouteStatus route_load_plan_from_found_state(int16_t found_index,
                                                       uint16_t start_index,
                                                       const int16_t *parent,
                                                       const NavPlanAction *parent_action)
{
    uint16_t route_length = 0;
    NavPlanAction first_action = NAV_PLAN_ACTION_NONE;
    NavPlanAction last_action = NAV_PLAN_ACTION_NONE;
    const NavRouteStatus status =
        route_collect_plan_from_found_state(found_index,
                                            start_index,
                                            parent,
                                            parent_action,
                                            true,
                                            &route_length,
                                            &first_action,
                                            &last_action);
    route_debug.status = status;
    route_debug.route_length = (uint8_t)route_length;
    route_debug.loaded_into_plan_queue =
        status == NAV_ROUTE_STATUS_FOUND && route_length > 0u;
    route_debug.first_action = first_action;
    route_debug.last_action = last_action;
    return status;
}

static bool route_target_dir_mask_is_valid(uint8_t target_dir_mask)
{
    return target_dir_mask != 0u && (target_dir_mask & 0xF0u) == 0u;
}

static NavRouteStatus route_search_to_cell_with_dir_mask(int8_t target_cell_x,
                                                         int8_t target_cell_y,
                                                         uint8_t target_dir_mask,
                                                         bool load_queue,
                                                         bool update_eval_debug,
                                                         int16_t *found_index_out,
                                                         uint16_t *start_index_out,
                                                         NavMapDirection *found_dir_out)
{
    NavMapDebugSnapshot map_debug = {0};
    nav_map_get_debug_snapshot(&map_debug);
    if (update_eval_debug) {
        nav_core_route_eval_clear_debug();
        route_eval_debug.target_cell_x = target_cell_x;
        route_eval_debug.target_cell_y = target_cell_y;
        route_eval_debug.target_dir_mask = target_dir_mask;
    } else {
        nav_core_route_clear_debug();
        route_debug.target_cell_x = target_cell_x;
        route_debug.target_cell_y = target_cell_y;
        route_debug.start_cell_x = map_debug.cell_x;
        route_debug.start_cell_y = map_debug.cell_y;
        route_debug.start_dir = map_debug.dir;
    }

    if (!map_debug.enabled
        || target_cell_x < 0
        || target_cell_y < 0
        || target_cell_x >= map_debug.width
        || target_cell_y >= map_debug.height) {
        if (update_eval_debug) {
            route_eval_debug.status = NAV_ROUTE_STATUS_TARGET_OUT_OF_BOUNDS;
        } else {
            route_debug.status = NAV_ROUTE_STATUS_TARGET_OUT_OF_BOUNDS;
        }
        return NAV_ROUTE_STATUS_TARGET_OUT_OF_BOUNDS;
    }

    if (!route_target_dir_mask_is_valid(target_dir_mask)) {
        if (update_eval_debug) {
            route_eval_debug.status = NAV_ROUTE_STATUS_INVALID_TARGET_DIR_MASK;
        } else {
            route_debug.status = NAV_ROUTE_STATUS_INVALID_TARGET_DIR_MASK;
        }
        return NAV_ROUTE_STATUS_INVALID_TARGET_DIR_MASK;
    }

    NavMapCell target_cell = {0};
    if (!nav_map_get_cell(target_cell_x, target_cell_y, &target_cell)
        || !target_cell.visited) {
        if (update_eval_debug) {
            route_eval_debug.status = NAV_ROUTE_STATUS_TARGET_NOT_VISITED;
        } else {
            route_debug.status = NAV_ROUTE_STATUS_TARGET_NOT_VISITED;
        }
        return NAV_ROUTE_STATUS_TARGET_NOT_VISITED;
    }

    route_workspace_reset();

    const uint16_t start_index =
        route_state_index(map_debug.cell_x, map_debug.cell_y, map_debug.dir, map_debug.width);
    if (start_index_out != 0) {
        *start_index_out = start_index;
    }
    route_workspace.visited[start_index] = 1u;
    route_workspace.queue[0] = start_index;
    uint16_t queue_head = 0;
    uint16_t queue_tail = 1;
    if (update_eval_debug) {
        route_eval_debug.reached_count = 1u;
    }
    int16_t found_index = -1;
    NavMapDirection found_dir = NAV_DIR_NORTH;

    while (queue_head < queue_tail) {
        const uint16_t current_index = route_workspace.queue[queue_head++];
        if (update_eval_debug) {
            ++route_eval_debug.expanded_count;
        } else {
            ++route_debug.expanded_states;
        }

        int8_t cell_x = 0;
        int8_t cell_y = 0;
        NavMapDirection dir = NAV_DIR_NORTH;
        route_decode_state(current_index, map_debug.width, &cell_x, &cell_y, &dir);

        if (cell_x == target_cell_x
            && cell_y == target_cell_y
            && ((uint8_t)(1u << (uint8_t)dir) & target_dir_mask) != 0u) {
            found_index = (int16_t)current_index;
            found_dir = dir;
            break;
        }

        const NavPlanAction actions[] = {
            NAV_PLAN_ACTION_ADVANCE_LINE,
            NAV_PLAN_ACTION_SMOOTH_RIGHT,
            NAV_PLAN_ACTION_SMOOTH_LEFT,
            NAV_PLAN_ACTION_CENTER_AND_PIVOT_180
        };
        for (uint8_t i = 0; i < 4; ++i) {
            if (route_workspace.parent_action[current_index] == NAV_PLAN_ACTION_CENTER_AND_PIVOT_180
                && (actions[i] == NAV_PLAN_ACTION_SMOOTH_RIGHT
                    || actions[i] == NAV_PLAN_ACTION_SMOOTH_LEFT)) {
                continue;
            }

            int8_t next_x = 0;
            int8_t next_y = 0;
            NavMapDirection next_dir = NAV_DIR_NORTH;
            if (!route_next_state_for_action(cell_x,
                                             cell_y,
                                             dir,
                                             actions[i],
                                             &map_debug,
                                             &next_x,
                                             &next_y,
                                             &next_dir)) {
                continue;
            }

            const uint16_t next_index =
                route_state_index(next_x, next_y, next_dir, map_debug.width);
            if (route_workspace.visited[next_index] != 0u) {
                continue;
            }

            route_workspace.visited[next_index] = 1u;
            route_workspace.parent[next_index] = (int16_t)current_index;
            route_workspace.parent_action[next_index] = actions[i];
            route_workspace.queue[queue_tail++] = next_index;
            if (update_eval_debug) {
                route_eval_debug.reached_count = queue_tail;
            }
        }
    }

    if (found_index < 0) {
        if (update_eval_debug) {
            route_eval_debug.status = NAV_ROUTE_STATUS_NO_PATH;
            route_eval_debug.reached_count = queue_tail;
        } else {
            route_debug.status = NAV_ROUTE_STATUS_NO_PATH;
        }
        return NAV_ROUTE_STATUS_NO_PATH;
    }

    if (found_index_out != 0) {
        *found_index_out = found_index;
    }
    if (found_dir_out != 0) {
        *found_dir_out = found_dir;
    }

    uint16_t route_length = 0;
    NavPlanAction first_action = NAV_PLAN_ACTION_NONE;
    NavPlanAction last_action = NAV_PLAN_ACTION_NONE;
    const NavRouteStatus status =
        route_collect_plan_from_found_state(found_index,
                                            start_index,
                                            route_workspace.parent,
                                            route_workspace.parent_action,
                                            load_queue,
                                            &route_length,
                                            &first_action,
                                            &last_action);
    if (update_eval_debug) {
        route_eval_debug.status = status;
        route_eval_debug.found_target_dir =
            status == NAV_ROUTE_STATUS_FOUND ? (int8_t)found_dir : -1;
        route_eval_debug.route_length = route_length;
        route_eval_debug.first_action = first_action;
        route_eval_debug.last_action = last_action;
        route_eval_debug.loaded_into_plan_queue = false;
        route_eval_debug.reached_count = queue_tail;
    } else {
        route_debug.status = status;
        route_debug.route_length = (uint8_t)route_length;
        route_debug.first_action = first_action;
        route_debug.last_action = last_action;
        route_debug.loaded_into_plan_queue =
            status == NAV_ROUTE_STATUS_FOUND && route_length > 0u && load_queue;
    }
    return status;
}

NavRouteStatus nav_core_route_eval_to_cell_with_dir_mask(int8_t target_x,
                                                         int8_t target_y,
                                                         uint8_t target_dir_mask)
{
    return route_search_to_cell_with_dir_mask(target_x,
                                              target_y,
                                              target_dir_mask,
                                              false,
                                              true,
                                              0,
                                              0,
                                              0);
}

NavRouteStatus nav_core_route_plan_to_cell_with_dir_mask(int8_t target_x,
                                                         int8_t target_y,
                                                         uint8_t target_dir_mask)
{
    nav_core_plan_clear();
    return route_search_to_cell_with_dir_mask(target_x,
                                              target_y,
                                              target_dir_mask,
                                              true,
                                              false,
                                              0,
                                              0,
                                              0);
}

NavRouteStatus nav_core_route_plan_to_cell(int16_t target_cell_x, int16_t target_cell_y)
{
    if (target_cell_x < INT8_MIN
        || target_cell_x > INT8_MAX
        || target_cell_y < INT8_MIN
        || target_cell_y > INT8_MAX) {
        nav_core_route_clear_debug();
        nav_core_plan_clear();
        NavMapDebugSnapshot map_debug = {0};
        nav_map_get_debug_snapshot(&map_debug);
        route_debug.target_cell_x = -1;
        route_debug.target_cell_y = -1;
        route_debug.start_cell_x = map_debug.cell_x;
        route_debug.start_cell_y = map_debug.cell_y;
        route_debug.start_dir = map_debug.dir;
        route_debug.status = NAV_ROUTE_STATUS_TARGET_OUT_OF_BOUNDS;
        return route_debug.status;
    }

    const NavRouteStatus status =
        nav_core_route_plan_to_cell_with_dir_mask((int8_t)target_cell_x,
                                                  (int8_t)target_cell_y,
                                                  0x0Fu);
    if (status == NAV_ROUTE_STATUS_FOUND) {
        route_debug.loaded_into_plan_queue = true;
    }
    return status;
}

NavRouteStatus nav_core_route_plan_to_nearest_frontier(void)
{
    nav_core_route_clear_debug();
    nav_core_plan_clear();
    route_debug.frontier_mode = true;

    NavMapDebugSnapshot map_debug = {0};
    nav_map_get_debug_snapshot(&map_debug);
    route_debug.start_cell_x = map_debug.cell_x;
    route_debug.start_cell_y = map_debug.cell_y;
    route_debug.start_dir = map_debug.dir;

    if (!map_debug.enabled
        || !map_cell_is_inside(&map_debug, map_debug.cell_x, map_debug.cell_y)) {
        route_debug.status = NAV_ROUTE_STATUS_TARGET_OUT_OF_BOUNDS;
        return route_debug.status;
    }

    route_debug.frontier_count_found = route_count_visited_frontier_cells(&map_debug);
    if (route_debug.frontier_count_found == 0) {
        route_debug.status = NAV_ROUTE_STATUS_NO_FRONTIER;
        return route_debug.status;
    }

    route_workspace_reset();

    const uint16_t start_index =
        route_state_index(map_debug.cell_x, map_debug.cell_y, map_debug.dir, map_debug.width);
    route_workspace.visited[start_index] = 1u;
    route_workspace.queue[0] = start_index;
    uint16_t queue_head = 0;
    uint16_t queue_tail = 1;
    int16_t found_index = -1;

    while (queue_head < queue_tail) {
        const uint16_t current_index = route_workspace.queue[queue_head++];
        ++route_debug.expanded_states;

        int8_t cell_x = 0;
        int8_t cell_y = 0;
        NavMapDirection dir = NAV_DIR_NORTH;
        route_decode_state(current_index, map_debug.width, &cell_x, &cell_y, &dir);

        NavMapDirection exit_dir = NAV_DIR_NORTH;
        NavFrontierExitRelative exit_relative = NAV_FRONTIER_EXIT_NONE;
        int8_t neighbor_x = -1;
        int8_t neighbor_y = -1;
        if (route_state_is_usable_frontier(cell_x,
                                           cell_y,
                                           dir,
                                           &map_debug,
                                           &exit_dir,
                                           &exit_relative,
                                           &neighbor_x,
                                           &neighbor_y)) {
            found_index = (int16_t)current_index;
            route_debug.frontier_target_cell_x = cell_x;
            route_debug.frontier_target_cell_y = cell_y;
            route_debug.frontier_target_dir = dir;
            route_debug.frontier_exit_dir_absolute = exit_dir;
            route_debug.frontier_exit_relative = exit_relative;
            route_debug.frontier_neighbor_cell_x = neighbor_x;
            route_debug.frontier_neighbor_cell_y = neighbor_y;
            route_debug.target_cell_x = cell_x;
            route_debug.target_cell_y = cell_y;
            break;
        }

        const NavPlanAction actions[] = {
            NAV_PLAN_ACTION_ADVANCE_LINE,
            NAV_PLAN_ACTION_SMOOTH_RIGHT,
            NAV_PLAN_ACTION_SMOOTH_LEFT,
            NAV_PLAN_ACTION_CENTER_AND_PIVOT_180
        };
        for (uint8_t i = 0; i < 4; ++i) {
            if (route_workspace.parent_action[current_index] == NAV_PLAN_ACTION_CENTER_AND_PIVOT_180
                && (actions[i] == NAV_PLAN_ACTION_SMOOTH_RIGHT
                    || actions[i] == NAV_PLAN_ACTION_SMOOTH_LEFT)) {
                continue;
            }

            int8_t next_x = 0;
            int8_t next_y = 0;
            NavMapDirection next_dir = NAV_DIR_NORTH;
            if (!route_next_state_for_action(cell_x,
                                             cell_y,
                                             dir,
                                             actions[i],
                                             &map_debug,
                                             &next_x,
                                             &next_y,
                                             &next_dir)) {
                continue;
            }

            const uint16_t next_index =
                route_state_index(next_x, next_y, next_dir, map_debug.width);
            if (route_workspace.visited[next_index] != 0u) {
                continue;
            }

            route_workspace.visited[next_index] = 1u;
            route_workspace.parent[next_index] = (int16_t)current_index;
            route_workspace.parent_action[next_index] = actions[i];
            route_workspace.queue[queue_tail++] = next_index;
        }
    }

    if (found_index < 0) {
        route_debug.status = NAV_ROUTE_STATUS_NO_FRONTIER;
        return route_debug.status;
    }

    route_debug.status =
        route_load_plan_from_found_state(found_index,
                                         start_index,
                                         route_workspace.parent,
                                         route_workspace.parent_action);
    if (route_debug.status != NAV_ROUTE_STATUS_FOUND) {
        return route_debug.status;
    }
    if (route_debug.route_length == 0) {
        route_debug.status = NAV_ROUTE_STATUS_FRONTIER_ALREADY_HERE;
        route_debug.loaded_into_plan_queue = false;
    }
    return route_debug.status;
}

static bool rear_black_for_line(const RobotSensors *sensors)
{
    if (advance_start_mode == NAV_ADVANCE_START_CENTERED_POSE) {
        return sensors != 0
            && sensors->floor_rear_black
            && !advance_from_centered_waiting_rear_white;
    }

    return sensors != 0
        && sensors->floor_rear_black
        && !special_ignore_rear_until_white;
}

static void update_floor_line_debug(const RobotSensors *sensors)
{
    turn_debug.floor_rear_black = sensors != 0 && sensors->floor_rear_black;
    turn_debug.rear_black_for_line = rear_black_for_line(sensors);
    turn_debug.advance_started_on_rear_line = advance_started_on_rear_line;
    turn_debug.advance_start_mode = advance_start_mode;
    turn_debug.advance_from_centered_waiting_rear_white =
        advance_from_centered_waiting_rear_white;
}

static void update_special_detection(const RobotSensors *sensors,
                                     NavSpecialDetectionContext context,
                                     bool allow_confirm)
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
    const bool after_special_min_time =
        advance_elapsed_since_leave_start_line_ms >= NAV_SPECIAL_DETECT_MIN_MS;
    bool can_confirm = false;

    special_detection_context = context;
    if (context == NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL) {
        can_confirm = allow_confirm
            && special_detection_enabled_for_current_motion
            && !special_ignore_rear_until_white
            && in_special_window;
    } else if (context == NAV_SPECIAL_DETECT_IN_CELL_AUX_TRANSLATION) {
        if (special_aux_waiting_rear_white && !sensors->floor_rear_black) {
            special_aux_waiting_rear_white = false;
            special_aux_detection_enabled = true;
            special_aux_started_after_rear_line_left = true;
        }
        can_confirm = allow_confirm
            && special_aux_detection_enabled
            && after_special_min_time;
    }

    special_confirmed = false;
    if (can_confirm
        && special_candidate) {
        NavMapDebugSnapshot map_debug = {0};
        nav_map_get_debug_snapshot(&map_debug);
        special_mark_target_cell_x = map_debug.cell_x;
        special_mark_target_cell_y = map_debug.cell_y;
        special_mark_target_source =
            context == NAV_SPECIAL_DETECT_IN_CELL_AUX_TRANSLATION
                ? NAV_SPECIAL_MARK_TARGET_AUX_CURRENT_CELL
                : NAV_SPECIAL_MARK_TARGET_CURRENT_CELL;
        last_special_mark_action = current_action;
        (void)nav_map_mark_current_cell_special();
        special_confirmed = true;
        if (context == NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL) {
            special_ignore_rear_until_white = true;
        }
    }

    turn_debug.special_candidate = special_candidate;
    turn_debug.special_confirmed = special_confirmed;
    turn_debug.special_ignore_rear_until_white = special_ignore_rear_until_white;
    turn_debug.special_detection_started_on_rear_line = special_detection_started_on_rear_line;
    turn_debug.special_detection_enabled_for_current_motion =
        special_detection_enabled_for_current_motion;
    turn_debug.special_detection_context = special_detection_context;
    turn_debug.special_aux_detection_enabled = special_aux_detection_enabled;
    turn_debug.special_aux_started_after_rear_line_left =
        special_aux_started_after_rear_line_left;
    sync_special_mark_debug();
    update_floor_line_debug(sensors);
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
        reset_smooth_final_hold();
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
    if (phase == NAV_ADVANCE_PHASE_NONE) {
        advance_front_diag_preview_armed = false;
        advance_front_diag_preview_latched = false;
    }
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

static void set_center_pivot_phase(NavCenterPivotPhase phase)
{
    center_pivot_phase = phase;
    turn_debug.center_pivot_phase = phase;
    if (phase == NAV_CENTER_PIVOT_PHASE_INIT) {
        center_pivot_elapsed_ms = 0;
        center_pivot_brake_elapsed_ms = 0;
        center_pivot_done_reason = NAV_CENTER_PIVOT_DONE_NONE;
        center_pivot_front_seen_white = false;
    } else if (phase == NAV_CENTER_PIVOT_PHASE_BRAKE_SETTLE) {
        center_pivot_brake_elapsed_ms = 0;
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
    set_yaw_carry_candidate(NAV_YAW_CARRY_SOURCE_ADVANCE_FRONT_DIAG_PREVIEW,
                            advance_diag_preview_entry_yaw_deg_q16,
                            sensors->yaw_deg_q16,
                            advance_front_diag_used_for_yaw_carry);
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    advance_front_diag_preview_armed = false;
    advance_front_diag_preview_latched = false;
    advance_diag_preview_entry_yaw_deg_q16 = 0;
    advance_front_diag_used_for_yaw_carry = false;
    reset_wall_caution_state(NAV_WALL_CAUTION_LOSS_ACTION_END);
    reset_advance_wall_pd();
    clear_live_turn_debug();
    turn_debug.advance_done_reason = NAV_ADVANCE_DONE_REAR_SENSOR_TARGET_LINE;
    turn_debug.last_completed_action = completed_action;
    turn_debug.last_advance_done_reason = NAV_ADVANCE_DONE_REAR_SENSOR_TARGET_LINE;
    turn_debug.last_advance_final_yaw_deg_q16 = sensors->yaw_deg_q16;
    turn_debug.last_advance_final_floor_rear_black = sensors->floor_rear_black;
    set_rear_line_trust(
        true,
        advance_start_mode == NAV_ADVANCE_START_CENTERED_POSE
            ? NAV_REAR_LINE_TRUST_CENTERED_ADVANCE_DONE
            : NAV_REAR_LINE_TRUST_ADVANCE_DONE);

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

static RobotCommand finish_center_in_cell_for_pivot_by_front_line(
    const RobotSensors *sensors,
    NavCenterPivotDoneReason reason)
{
    const NavAction completed_action = current_action;

    apply_completed_action_to_map(completed_action);
    current_state = NAV_STATE_DONE;
    current_action = NAV_ACTION_NONE;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_DONE;
    center_pivot_done_reason = reason;
    reset_advance_wall_pd();
    clear_live_turn_debug();
    turn_debug.center_pivot_phase = center_pivot_phase;
    turn_debug.center_pivot_done_reason = reason;
    turn_debug.center_pivot_elapsed_ms = center_pivot_elapsed_ms;
    turn_debug.center_pivot_brake_elapsed_ms = center_pivot_brake_elapsed_ms;
    turn_debug.center_pivot_base_left_pwm = NAV_CENTER_PIVOT_BASE_LEFT_PWM;
    turn_debug.center_pivot_base_right_pwm = NAV_CENTER_PIVOT_BASE_RIGHT_PWM;
    turn_debug.center_pivot_correction_pwm = 0;
    turn_debug.center_pivot_front_black = sensors != 0 && sensors->floor_front_black;
    turn_debug.center_pivot_rear_black = sensors != 0 && sensors->floor_rear_black;
    turn_debug.center_pivot_front_seen_white = center_pivot_front_seen_white;
    turn_debug.last_completed_action = completed_action;
    turn_debug.last_center_pivot_done_reason = reason;

    RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
    return command;
}

static RobotCommand center_in_cell_for_pivot_command(const RobotSensors *sensors)
{
    RobotCommand command = guided_forward_command(sensors,
                                                  NAV_CENTER_PIVOT_BASE_LEFT_PWM,
                                                  NAV_CENTER_PIVOT_BASE_RIGHT_PWM);
    turn_debug.center_pivot_phase = center_pivot_phase;
    turn_debug.center_pivot_done_reason = center_pivot_done_reason;
    turn_debug.center_pivot_elapsed_ms = center_pivot_elapsed_ms;
    turn_debug.center_pivot_brake_elapsed_ms = center_pivot_brake_elapsed_ms;
    turn_debug.center_pivot_base_left_pwm = NAV_CENTER_PIVOT_BASE_LEFT_PWM;
    turn_debug.center_pivot_base_right_pwm = NAV_CENTER_PIVOT_BASE_RIGHT_PWM;
    turn_debug.center_pivot_correction_pwm =
        clamp_pwm(command.left_motor_pwm - NAV_CENTER_PIVOT_BASE_LEFT_PWM);
    turn_debug.center_pivot_front_black = sensors->floor_front_black;
    turn_debug.center_pivot_rear_black = sensors->floor_rear_black;
    turn_debug.center_pivot_front_seen_white = center_pivot_front_seen_white;
    return command;
}

static RobotCommand guided_forward_command(const RobotSensors *sensors,
                                           int16_t base_left_pwm,
                                           int16_t base_right_pwm)
{
    const bool wall_left_valid = wall_perception.wall_left;
    const bool wall_right_valid = wall_perception.wall_right;
    const bool diag_left_valid = wall_perception.wall_diag_left;
    const bool diag_right_valid = wall_perception.wall_diag_right;
    const bool follow_left_valid = wall_left_valid && diag_left_valid;
    const bool follow_right_valid = wall_right_valid && diag_right_valid;
    q16_16_t wall_raw_error_q16 = 0;
    q16_16_t wall_error_q16 = 0;
    NavAdvanceCorrectionSource correction_source = NAV_ADVANCE_CORRECTION_YAW_PD;
    bool front_diag_preview_active = false;
    NavAdvanceFrontDiagSource front_diag_source = NAV_ADVANCE_FRONT_DIAG_NONE;
    q16_16_t front_diag_raw_error_q16 = 0;
    q16_16_t front_diag_error_q16 = 0;
    int16_t wall_caution_correction_pwm = 0;

    update_wall_caution_state(wall_left_valid,
                              wall_right_valid,
                              follow_left_valid,
                              follow_right_valid);

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
        if (correction_source == NAV_ADVANCE_CORRECTION_YAW_PD) {
            if (wall_left_confidence == NAV_WALL_CAUTION_CONFIDENCE_CAUTION) {
                wall_raw_error_q16 = wall_left_caution_hold_mm_q16 - wall_perception.left_mm_q16;
                wall_error_q16 = wall_raw_error_q16;
                correction_source = NAV_ADVANCE_CORRECTION_WALL_LEFT_CAUTION;
            } else if (wall_right_confidence == NAV_WALL_CAUTION_CONFIDENCE_CAUTION) {
                wall_raw_error_q16 = wall_perception.right_mm_q16 - wall_right_caution_hold_mm_q16;
                wall_error_q16 = wall_raw_error_q16;
                correction_source = NAV_ADVANCE_CORRECTION_WALL_RIGHT_CAUTION;
            }
        }
    }
    if (current_action == NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK
        && advance_phase == NAV_ADVANCE_PHASE_SEEK_TARGET_LINE
        && !sensors->floor_front_black) {
        advance_front_diag_preview_armed = true;
    }
    if (current_action == NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK
        && advance_phase == NAV_ADVANCE_PHASE_SEEK_TARGET_LINE
        && advance_front_diag_preview_armed
        && sensors->floor_front_black
        && !sensors->floor_rear_black) {
        advance_front_diag_preview_latched = true;
    }
    if (correction_source == NAV_ADVANCE_CORRECTION_YAW_PD
        && advance_front_diag_preview_latched) {
        if (diag_left_valid && diag_right_valid) {
            front_diag_raw_error_q16 =
                wall_perception.diag_right_mm_q16 - wall_perception.diag_left_mm_q16;
            front_diag_error_q16 = smooth_final_scale_diag_error(front_diag_raw_error_q16);
            wall_raw_error_q16 = front_diag_raw_error_q16;
            wall_error_q16 = front_diag_error_q16;
            correction_source = NAV_ADVANCE_CORRECTION_DIAG_CENTER;
            front_diag_source = NAV_ADVANCE_FRONT_DIAG_CENTER;
            front_diag_preview_active = true;
        } else if (diag_left_valid) {
            front_diag_raw_error_q16 =
                (mm_to_q16(diagonal_guidance_config.target_mm) - wall_perception.diag_left_mm_q16)
                * NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE;
            front_diag_error_q16 = smooth_final_scale_diag_error(front_diag_raw_error_q16);
            wall_raw_error_q16 = front_diag_raw_error_q16;
            wall_error_q16 = front_diag_error_q16;
            correction_source = NAV_ADVANCE_CORRECTION_DIAG_LEFT;
            front_diag_source = NAV_ADVANCE_FRONT_DIAG_LEFT;
            front_diag_preview_active = true;
        } else if (diag_right_valid) {
            front_diag_raw_error_q16 =
                (wall_perception.diag_right_mm_q16 - mm_to_q16(diagonal_guidance_config.target_mm))
                * NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE;
            front_diag_error_q16 = smooth_final_scale_diag_error(front_diag_raw_error_q16);
            wall_raw_error_q16 = front_diag_raw_error_q16;
            wall_error_q16 = front_diag_error_q16;
            correction_source = NAV_ADVANCE_CORRECTION_DIAG_RIGHT;
            front_diag_source = NAV_ADVANCE_FRONT_DIAG_RIGHT;
            front_diag_preview_active = true;
        }
    }
    if (front_diag_preview_active) {
        advance_front_diag_used_for_yaw_carry = true;
    }

    if (correction_source == NAV_ADVANCE_CORRECTION_YAW_PD
        && (!forward_guidance_yaw_hold_initialized
            || forward_guidance_last_source != NAV_ADVANCE_CORRECTION_YAW_PD)) {
        capture_forward_guidance_yaw_hold(sensors->yaw_deg_q16);
    }
    if (correction_source != NAV_ADVANCE_CORRECTION_YAW_PD) {
        forward_guidance_last_source = correction_source;
    }

    const q16_16_t yaw_setpoint_q16 = forward_guidance_yaw_hold_initialized
        ? forward_guidance_yaw_hold_deg_q16
        : sensors->yaw_deg_q16;
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, yaw_setpoint_q16);
    const int32_t pid_output_q16 = PID_Update_Fixed(&advance_yaw_pid, sensors->yaw_deg_q16, 10);
    const int32_t yaw_correction_pwm = FIXED_TO_INT(pid_output_q16);
    const q16_16_t yaw_error_q16 = yaw_setpoint_q16 - sensors->yaw_deg_q16;
    if (correction_source == NAV_ADVANCE_CORRECTION_YAW_PD) {
        forward_guidance_last_source = NAV_ADVANCE_CORRECTION_YAW_PD;
    }

    const q16_16_t wall_error_after_deadband_q16 = apply_wall_deadband(wall_error_q16);
    q16_16_t wall_previous_error_q16 = 0;
    q16_16_t wall_error_delta_q16 = 0;
    q16_16_t diag_previous_error_q16 = 0;
    q16_16_t diag_error_delta_q16 = 0;
    q16_16_t caution_previous_error_q16 = 0;
    q16_16_t caution_error_delta_q16 = 0;
    int32_t wall_p_term_pwm = 0;
    int32_t wall_d_term_pwm = 0;
    int32_t wall_raw_correction_pwm = 0;
    int16_t wall_correction_pwm = 0;
    int32_t diag_p_term_pwm = 0;
    int32_t diag_d_term_pwm = 0;
    int16_t diag_correction_pwm = 0;
    int32_t caution_p_term_pwm = 0;
    int32_t caution_d_term_pwm = 0;
    const bool use_diag_guidance =
        correction_source == NAV_ADVANCE_CORRECTION_DIAG_CENTER
        || correction_source == NAV_ADVANCE_CORRECTION_DIAG_LEFT
        || correction_source == NAV_ADVANCE_CORRECTION_DIAG_RIGHT;
    const bool use_wall_guidance =
        correction_source == NAV_ADVANCE_CORRECTION_WALL_CENTER
        || correction_source == NAV_ADVANCE_CORRECTION_WALL_LEFT
        || correction_source == NAV_ADVANCE_CORRECTION_WALL_RIGHT;
    const bool use_wall_caution_guidance =
        correction_source == NAV_ADVANCE_CORRECTION_WALL_LEFT_CAUTION
        || correction_source == NAV_ADVANCE_CORRECTION_WALL_RIGHT_CAUTION;
    if (use_wall_guidance) {
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
        reset_wall_caution_pd();
        reset_diagonal_guidance_pd();
    } else if (use_wall_caution_guidance) {
        caution_previous_error_q16 = wall_caution_has_previous_error
            ? wall_caution_previous_error_q16
            : 0;
        caution_error_delta_q16 = wall_caution_has_previous_error
            ? wall_error_after_deadband_q16 - caution_previous_error_q16
            : 0;
        caution_p_term_pwm = q16_to_pwm(wall_error_after_deadband_q16,
                                        wall_caution_config.kp_pwm_per_mm);
        caution_d_term_pwm = q16_to_pwm(caution_error_delta_q16,
                                        wall_caution_config.kd_pwm_per_mm_per_tick);
        wall_caution_correction_pwm =
            clamp_wall_caution_correction(caution_p_term_pwm + caution_d_term_pwm);
        wall_p_term_pwm = caution_p_term_pwm;
        wall_d_term_pwm = caution_d_term_pwm;
        wall_raw_correction_pwm = caution_p_term_pwm + caution_d_term_pwm;
        wall_correction_pwm = wall_caution_correction_pwm;
        wall_caution_previous_error_q16 = wall_error_after_deadband_q16;
        wall_caution_has_previous_error = true;
        reset_advance_wall_pd();
        reset_diagonal_guidance_pd();
    } else if (use_diag_guidance) {
        diag_previous_error_q16 = diagonal_guidance_has_previous_error
            ? diagonal_guidance_previous_error_q16
            : 0;
        diag_error_delta_q16 = diagonal_guidance_has_previous_error
            ? wall_error_after_deadband_q16 - diag_previous_error_q16
            : 0;
        diag_p_term_pwm = q16_to_pwm(wall_error_after_deadband_q16,
                                     diagonal_guidance_config.kp_pwm_per_mm);
        diag_d_term_pwm = q16_to_pwm(diag_error_delta_q16,
                                     diagonal_guidance_config.kd_pwm_per_mm_per_tick);
        diag_correction_pwm = clamp_diag_correction(diag_p_term_pwm + diag_d_term_pwm);
        wall_p_term_pwm = diag_p_term_pwm;
        wall_d_term_pwm = diag_d_term_pwm;
        wall_raw_correction_pwm = diag_p_term_pwm + diag_d_term_pwm;
        wall_correction_pwm = diag_correction_pwm;
        diagonal_guidance_previous_error_q16 = wall_error_after_deadband_q16;
        diagonal_guidance_has_previous_error = true;
        reset_advance_wall_pd();
        reset_wall_caution_pd();
    } else {
        reset_advance_wall_pd();
        reset_diagonal_guidance_pd();
        reset_wall_caution_pd();
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
    turn_debug.advance_yaw_hold_deg_q16 = forward_guidance_yaw_hold_deg_q16;
    turn_debug.advance_yaw_hold_initialized = forward_guidance_yaw_hold_initialized;
    turn_debug.advance_yaw_hold_recapture_count = forward_guidance_yaw_hold_recapture_count;
    turn_debug.advance_yaw_hold_error_deg_q16 = yaw_error_q16;
    turn_debug.advance_guidance_last_source = forward_guidance_last_source;
    turn_debug.advance_yaw_kp_q16 = advance_yaw_pid_config.kp_q16;
    turn_debug.advance_yaw_ki_q16 = advance_yaw_pid_config.ki_q16;
    turn_debug.advance_yaw_kd_q16 = advance_yaw_pid_config.kd_q16;
    turn_debug.advance_yaw_output_limit_pwm = (int16_t)advance_yaw_pid_config.output_limit_pwm;
    turn_debug.advance_base_left_pwm = base_left_pwm;
    turn_debug.advance_base_right_pwm = base_right_pwm;
    turn_debug.advance_guidance_mode = advance_guidance_mode;
    turn_debug.advance_final_correction_source = correction_source;
    turn_debug.advance_front_diag_preview_armed = advance_front_diag_preview_armed;
    turn_debug.advance_front_diag_preview_latched = advance_front_diag_preview_latched;
    turn_debug.advance_front_diag_preview_active = front_diag_preview_active;
    turn_debug.advance_front_diag_source = front_diag_source;
    turn_debug.advance_front_diag_raw_error_mm_q16 = front_diag_raw_error_q16;
    turn_debug.advance_front_diag_error_mm_q16 = front_diag_error_q16;
    turn_debug.advance_front_diag_left_valid = diag_left_valid;
    turn_debug.advance_front_diag_right_valid = diag_right_valid;
    turn_debug.advance_wall_left_valid = wall_left_valid;
    turn_debug.advance_wall_right_valid = wall_right_valid;
    turn_debug.advance_diag_left_valid = diag_left_valid;
    turn_debug.advance_diag_right_valid = diag_right_valid;
    turn_debug.advance_follow_left_valid = follow_left_valid;
    turn_debug.advance_follow_right_valid = follow_right_valid;
    turn_debug.wall_caution_enabled = wall_caution_config.enabled;
    turn_debug.wall_left_confidence = wall_left_confidence;
    turn_debug.wall_right_confidence = wall_right_confidence;
    turn_debug.wall_left_caution_elapsed_ms = wall_left_caution_elapsed_ms;
    turn_debug.wall_right_caution_elapsed_ms = wall_right_caution_elapsed_ms;
    turn_debug.wall_left_caution_hold_mm_q16 = wall_left_caution_hold_mm_q16;
    turn_debug.wall_right_caution_hold_mm_q16 = wall_right_caution_hold_mm_q16;
    turn_debug.wall_left_caution_delta_mm_q16 = wall_left_caution_delta_mm_q16;
    turn_debug.wall_right_caution_delta_mm_q16 = wall_right_caution_delta_mm_q16;
    turn_debug.wall_caution_correction_pwm = wall_caution_correction_pwm;
    turn_debug.wall_caution_loss_reason = wall_caution_loss_reason;
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
    turn_debug.diag_guidance_kp_pwm_per_mm = diagonal_guidance_config.kp_pwm_per_mm;
    turn_debug.diag_guidance_kd_pwm_per_mm_per_tick =
        diagonal_guidance_config.kd_pwm_per_mm_per_tick;
    turn_debug.diag_guidance_correction_limit_pwm =
        diagonal_guidance_config.correction_limit_pwm;
    turn_debug.diag_guidance_error_scale_num = diagonal_guidance_config.error_scale_num;
    turn_debug.diag_guidance_error_scale_den = diagonal_guidance_config.error_scale_den;
    turn_debug.diag_guidance_target_mm = diagonal_guidance_config.target_mm;
    turn_debug.diag_guidance_p_term_pwm = diag_p_term_pwm;
    turn_debug.diag_guidance_d_term_pwm = diag_d_term_pwm;
    turn_debug.diag_guidance_correction_pwm = diag_correction_pwm;
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

static NavSmoothFinalGuidanceSource smooth_final_source_from_walls(bool follow_left_valid,
                                                                   bool follow_right_valid)
{
    if (follow_left_valid && follow_right_valid) {
        return NAV_SMOOTH_FINAL_GUIDANCE_WALL_CENTER_HOLD;
    }
    if (follow_left_valid) {
        return NAV_SMOOTH_FINAL_GUIDANCE_WALL_LEFT_HOLD;
    }
    if (follow_right_valid) {
        return NAV_SMOOTH_FINAL_GUIDANCE_WALL_RIGHT_HOLD;
    }
    return NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY;
}

static NavSmoothFinalGuidanceSource smooth_final_source_from_sensors(bool diag_left_valid,
                                                                     bool diag_right_valid,
                                                                     bool follow_left_valid,
                                                                     bool follow_right_valid)
{
    if (diag_left_valid && diag_right_valid) {
        return diagonal_guidance_config.smooth_final_mode == NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT
            ? NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER
            : NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER_HOLD;
    }
    if (diag_left_valid) {
        return diagonal_guidance_config.smooth_final_mode == NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT
            ? NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT
            : NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT_HOLD;
    }
    if (diag_right_valid) {
        return diagonal_guidance_config.smooth_final_mode == NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT
            ? NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT
            : NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT_HOLD;
    }
    return smooth_final_source_from_walls(follow_left_valid, follow_right_valid);
}

static void capture_smooth_final_hold_snapshot(NavSmoothFinalGuidanceSource source)
{
    const bool diag_left_valid = wall_perception.wall_diag_left;
    const bool diag_right_valid = wall_perception.wall_diag_right;
    const bool follow_left_valid = wall_perception.wall_left && wall_perception.wall_diag_left;
    const bool follow_right_valid = wall_perception.wall_right && wall_perception.wall_diag_right;
    const bool recapturing =
        !smooth_final_hold_initialized || source != smooth_final_hold_source;

    smooth_final_hold_initialized = true;
    smooth_final_hold_source = source;
    if (source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER_HOLD
        && diag_left_valid
        && diag_right_valid) {
        smooth_final_diag_hold_initialized = true;
        smooth_final_diag_left_hold_mm_q16 = wall_perception.diag_left_mm_q16;
        smooth_final_diag_right_hold_mm_q16 = wall_perception.diag_right_mm_q16;
        smooth_final_diag_center_diff_hold_mm_q16 =
            wall_perception.diag_right_mm_q16 - wall_perception.diag_left_mm_q16;
        if (smooth_final_diag_hold_recapture_count < UINT16_MAX) {
            ++smooth_final_diag_hold_recapture_count;
        }
    } else if (source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT_HOLD && diag_left_valid) {
        smooth_final_diag_hold_initialized = true;
        smooth_final_diag_left_hold_mm_q16 = wall_perception.diag_left_mm_q16;
        if (smooth_final_diag_hold_recapture_count < UINT16_MAX) {
            ++smooth_final_diag_hold_recapture_count;
        }
    } else if (source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT_HOLD && diag_right_valid) {
        smooth_final_diag_hold_initialized = true;
        smooth_final_diag_right_hold_mm_q16 = wall_perception.diag_right_mm_q16;
        if (smooth_final_diag_hold_recapture_count < UINT16_MAX) {
            ++smooth_final_diag_hold_recapture_count;
        }
    } else if (source == NAV_SMOOTH_FINAL_GUIDANCE_WALL_CENTER_HOLD
        && follow_left_valid
        && follow_right_valid) {
        smooth_final_left_hold_mm_q16 = wall_perception.left_mm_q16;
        smooth_final_right_hold_mm_q16 = wall_perception.right_mm_q16;
        smooth_final_center_diff_hold_mm_q16 =
            wall_perception.right_mm_q16 - wall_perception.left_mm_q16;
    } else if (source == NAV_SMOOTH_FINAL_GUIDANCE_WALL_LEFT_HOLD && follow_left_valid) {
        smooth_final_left_hold_mm_q16 = wall_perception.left_mm_q16;
    } else if (source == NAV_SMOOTH_FINAL_GUIDANCE_WALL_RIGHT_HOLD && follow_right_valid) {
        smooth_final_right_hold_mm_q16 = wall_perception.right_mm_q16;
    }

    if (smooth_final_hold_recapture_count < UINT16_MAX) {
        ++smooth_final_hold_recapture_count;
    }
    if (recapturing) {
        reset_advance_wall_pd();
    }
}

static void capture_smooth_final_yaw_hold_snapshot(q16_16_t yaw_deg_q16)
{
    smooth_final_yaw_hold_initialized = true;
    smooth_final_yaw_hold_deg_q16 = yaw_deg_q16;
    if (smooth_final_yaw_hold_recapture_count < UINT16_MAX) {
        ++smooth_final_yaw_hold_recapture_count;
    }
    PID_Reset(&advance_yaw_pid);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, smooth_final_yaw_hold_deg_q16);
}

static bool smooth_source_is_setpoint_diag(NavSmoothFinalGuidanceSource source)
{
    return source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER
        || source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT
        || source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT;
}

static void set_yaw_carry_candidate(NavYawCarryCandidateSource source,
                                    q16_16_t entry_yaw_deg_q16,
                                    q16_16_t exit_yaw_deg_q16,
                                    bool diag_used)
{
    yaw_carry_candidate_source = source;
    yaw_carry_candidate_entry_yaw_deg_q16 = entry_yaw_deg_q16;
    yaw_carry_candidate_exit_yaw_deg_q16 = exit_yaw_deg_q16;
    yaw_carry_candidate_offset_deg_q16 = exit_yaw_deg_q16 - entry_yaw_deg_q16;
    yaw_carry_candidate_diag_used = diag_used;
    smooth_yaw_carry_candidate_available = diag_used;
    smooth_yaw_carry_pending = false;
    smooth_yaw_carry_used = false;
    smooth_yaw_carry_offset_deg_q16 = 0;
    smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_NONE;
    sync_smooth_yaw_carry_debug();
}

static q16_16_t scale_smooth_yaw_carry_offset(q16_16_t offset_q16)
{
    const int16_t den = smooth_yaw_carry_config.offset_scale_den <= 0
        ? 1
        : smooth_yaw_carry_config.offset_scale_den;
    return (q16_16_t)(((int64_t)offset_q16 * smooth_yaw_carry_config.offset_scale_num) / den);
}

static bool evaluate_smooth_yaw_carry_candidate(bool next_action_is_smooth)
{
    smooth_yaw_carry_pending = false;
    smooth_yaw_carry_offset_deg_q16 = 0;

    if (!smooth_yaw_carry_config.enabled) {
        smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_DISABLED;
        sync_smooth_yaw_carry_debug();
        return false;
    }
    if (!next_action_is_smooth) {
        smooth_yaw_carry_candidate_available = false;
        smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_NOT_NEXT_SMOOTH;
        sync_smooth_yaw_carry_debug();
        return false;
    }
    if (!smooth_yaw_carry_candidate_available) {
        smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_NOT_NEXT_SMOOTH;
        sync_smooth_yaw_carry_debug();
        return false;
    }
    if (smooth_yaw_carry_config.only_setpoint
        && yaw_carry_candidate_source == NAV_YAW_CARRY_SOURCE_SMOOTH_FINAL_DIAG
        && diagonal_guidance_config.smooth_final_mode != NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT) {
        smooth_yaw_carry_candidate_available = false;
        smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_NOT_SETPOINT;
        sync_smooth_yaw_carry_debug();
        return false;
    }
    if (yaw_carry_candidate_source == NAV_YAW_CARRY_SOURCE_ADVANCE_FRONT_DIAG_PREVIEW
        && !smooth_yaw_carry_config.allow_advance_preview) {
        smooth_yaw_carry_candidate_available = false;
        smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_NO_DIAG_USED;
        sync_smooth_yaw_carry_debug();
        return false;
    }
    if (smooth_yaw_carry_config.require_diag && !yaw_carry_candidate_diag_used) {
        smooth_yaw_carry_candidate_available = false;
        smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_NO_DIAG_USED;
        sync_smooth_yaw_carry_debug();
        return false;
    }

    const q16_16_t abs_offset_q16 = abs_q16(yaw_carry_candidate_offset_deg_q16);
    if (abs_offset_q16 < smooth_yaw_carry_config.min_abs_deg_q16) {
        smooth_yaw_carry_candidate_available = false;
        smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_OFFSET_TOO_SMALL;
        sync_smooth_yaw_carry_debug();
        return false;
    }
    if (abs_offset_q16 > smooth_yaw_carry_config.max_abs_deg_q16) {
        smooth_yaw_carry_candidate_available = false;
        smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_OFFSET_TOO_LARGE;
        sync_smooth_yaw_carry_debug();
        return false;
    }

    smooth_yaw_carry_offset_deg_q16 =
        scale_smooth_yaw_carry_offset(yaw_carry_candidate_offset_deg_q16);
    smooth_yaw_carry_pending = true;
    smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_NONE;
    sync_smooth_yaw_carry_debug();
    return true;
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
    smooth_final_exit_yaw_deg_q16 = sensors->yaw_deg_q16;
    smooth_final_exit_yaw_offset_deg_q16 =
        smooth_final_exit_yaw_deg_q16 - smooth_final_entry_yaw_deg_q16;
    set_yaw_carry_candidate(NAV_YAW_CARRY_SOURCE_SMOOTH_FINAL_DIAG,
                            smooth_final_entry_yaw_deg_q16,
                            smooth_final_exit_yaw_deg_q16,
                            reason == NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE
                                && smooth_final_diag_used);
    if (reason != NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE) {
        smooth_yaw_carry_candidate_available = false;
        smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_NOT_NEXT_SMOOTH;
    }
    sync_smooth_yaw_carry_debug();
    if (reason == NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE) {
        set_rear_line_trust(true, NAV_REAR_LINE_TRUST_SMOOTH_DONE);
    }
    smooth_post_yaw_elapsed_ms = 0;

    RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
    return command;
}

static void enter_smooth_post_yaw_seek(const RobotSensors *sensors)
{
    smooth_phase = NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE;
    smooth_post_yaw_elapsed_ms = 0;
    smooth_final_entry_yaw_deg_q16 = sensors->yaw_deg_q16;
    smooth_final_exit_yaw_deg_q16 = sensors->yaw_deg_q16;
    smooth_final_exit_yaw_offset_deg_q16 = 0;
    smooth_final_diag_used = false;
    smooth_yaw_carry_candidate_available = false;
    smooth_yaw_carry_pending = false;
    smooth_yaw_carry_offset_deg_q16 = 0;
    smooth_yaw_carry_rejected_reason = NAV_SMOOTH_YAW_CARRY_REJECT_NONE;
    const bool diag_left_valid = wall_perception.wall_diag_left;
    const bool diag_right_valid = wall_perception.wall_diag_right;
    const bool follow_left_valid = wall_perception.wall_left && wall_perception.wall_diag_left;
    const bool follow_right_valid = wall_perception.wall_right && wall_perception.wall_diag_right;
    const NavSmoothFinalGuidanceSource initial_source =
        smooth_final_source_from_sensors(diag_left_valid,
                                         diag_right_valid,
                                         follow_left_valid,
                                         follow_right_valid);
    capture_smooth_final_hold_snapshot(initial_source);
    if (initial_source == NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY) {
        capture_smooth_final_yaw_hold_snapshot(sensors->yaw_deg_q16);
    }
    turn_debug.smooth_phase = smooth_phase;
    turn_debug.smooth_done_reason = NAV_SMOOTH_DONE_NONE;
    turn_debug.smooth_post_yaw_elapsed_ms = smooth_post_yaw_elapsed_ms;
    turn_debug.smooth_final_guidance_source = smooth_final_hold_source;
    turn_debug.smooth_final_hold_initialized = smooth_final_hold_initialized;
    turn_debug.smooth_final_hold_source = smooth_final_hold_source;
    turn_debug.smooth_final_hold_recapture_count = smooth_final_hold_recapture_count;
    turn_debug.smooth_final_left_hold_mm_q16 = smooth_final_left_hold_mm_q16;
    turn_debug.smooth_final_right_hold_mm_q16 = smooth_final_right_hold_mm_q16;
    turn_debug.smooth_final_center_diff_hold_mm_q16 = smooth_final_center_diff_hold_mm_q16;
    turn_debug.smooth_final_yaw_hold_deg_q16 = smooth_final_yaw_hold_deg_q16;
    turn_debug.smooth_final_yaw_hold_initialized = smooth_final_yaw_hold_initialized;
    turn_debug.smooth_final_yaw_hold_recapture_count = smooth_final_yaw_hold_recapture_count;
    turn_debug.smooth_final_yaw_error_deg_q16 = 0;
    turn_debug.smooth_final_diag_left_valid = diag_left_valid;
    turn_debug.smooth_final_diag_right_valid = diag_right_valid;
    turn_debug.smooth_final_diag_left_mm_q16 = wall_perception.diag_left_mm_q16;
    turn_debug.smooth_final_diag_right_mm_q16 = wall_perception.diag_right_mm_q16;
    turn_debug.smooth_final_diag_target_mm_q16 =
        mm_to_q16(diagonal_guidance_config.target_mm);
    turn_debug.smooth_final_diag_error_scale_q16 =
        INT_TO_FIXED(diagonal_guidance_config.error_scale_num)
        / diagonal_guidance_config.error_scale_den;
    turn_debug.smooth_final_diag_mode = diagonal_guidance_config.smooth_final_mode;
    turn_debug.smooth_final_diag_hold_initialized = smooth_final_diag_hold_initialized;
    turn_debug.smooth_final_diag_left_hold_mm_q16 = smooth_final_diag_left_hold_mm_q16;
    turn_debug.smooth_final_diag_right_hold_mm_q16 = smooth_final_diag_right_hold_mm_q16;
    turn_debug.smooth_final_diag_center_diff_hold_mm_q16 =
        smooth_final_diag_center_diff_hold_mm_q16;
    turn_debug.smooth_final_diag_hold_recapture_count =
        smooth_final_diag_hold_recapture_count;
    turn_debug.smooth_final_diag_raw_error_mm_q16 = 0;
    turn_debug.smooth_final_diag_error_mm_q16 = 0;
    sync_smooth_yaw_carry_debug();
    if (initial_source != NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY) {
        PID_Reset(&advance_yaw_pid);
        PID_Set_Setpoint_Fixed(&advance_yaw_pid, sensors->yaw_deg_q16);
    }
    reset_advance_wall_pd();
}

static RobotCommand smooth_post_yaw_seek_command(const RobotSensors *sensors)
{
    const bool diag_left_valid = wall_perception.wall_diag_left;
    const bool diag_right_valid = wall_perception.wall_diag_right;
    const bool follow_left_valid = wall_perception.wall_left && wall_perception.wall_diag_left;
    const bool follow_right_valid = wall_perception.wall_right && wall_perception.wall_diag_right;
    const NavSmoothFinalGuidanceSource requested_source =
        smooth_final_source_from_sensors(diag_left_valid,
                                         diag_right_valid,
                                         follow_left_valid,
                                         follow_right_valid);
    NavSmoothFinalGuidanceSource active_source = requested_source;
    q16_16_t wall_error_q16 = 0;
    q16_16_t diag_raw_error_q16 = 0;
    q16_16_t diag_error_q16 = 0;

    if (!smooth_final_hold_initialized || requested_source != smooth_final_hold_source) {
        capture_smooth_final_hold_snapshot(requested_source);
        if (requested_source == NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY) {
            capture_smooth_final_yaw_hold_snapshot(sensors->yaw_deg_q16);
        }
    } else if (requested_source == NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY
               && !smooth_final_yaw_hold_initialized) {
        capture_smooth_final_yaw_hold_snapshot(sensors->yaw_deg_q16);
    }

    switch (requested_source) {
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER:
        if (diag_left_valid && diag_right_valid) {
            diag_raw_error_q16 =
                wall_perception.diag_right_mm_q16 - wall_perception.diag_left_mm_q16;
            diag_error_q16 = smooth_final_scale_diag_error(diag_raw_error_q16);
            wall_error_q16 = diag_error_q16;
        } else {
            active_source = NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK;
        }
        break;
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT:
        if (diag_left_valid) {
            diag_raw_error_q16 =
                (mm_to_q16(diagonal_guidance_config.target_mm)
                 - wall_perception.diag_left_mm_q16)
                * NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE;
            diag_error_q16 = smooth_final_scale_diag_error(diag_raw_error_q16);
            wall_error_q16 = diag_error_q16;
        } else {
            active_source = NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK;
        }
        break;
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT:
        if (diag_right_valid) {
            diag_raw_error_q16 =
                (wall_perception.diag_right_mm_q16
                 - mm_to_q16(diagonal_guidance_config.target_mm))
                * NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE;
            diag_error_q16 = smooth_final_scale_diag_error(diag_raw_error_q16);
            wall_error_q16 = diag_error_q16;
        } else {
            active_source = NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK;
        }
        break;
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER_HOLD:
        if (diag_left_valid && diag_right_valid) {
            diag_raw_error_q16 =
                (wall_perception.diag_right_mm_q16 - wall_perception.diag_left_mm_q16)
                - smooth_final_diag_center_diff_hold_mm_q16;
            diag_error_q16 = smooth_final_scale_diag_error(diag_raw_error_q16);
            wall_error_q16 = diag_error_q16;
        } else {
            active_source = NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK;
        }
        break;
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT_HOLD:
        if (diag_left_valid) {
            diag_raw_error_q16 =
                smooth_final_diag_left_hold_mm_q16 - wall_perception.diag_left_mm_q16;
            diag_error_q16 = smooth_final_scale_diag_error(diag_raw_error_q16);
            wall_error_q16 = diag_error_q16;
        } else {
            active_source = NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK;
        }
        break;
    case NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT_HOLD:
        if (diag_right_valid) {
            diag_raw_error_q16 =
                wall_perception.diag_right_mm_q16 - smooth_final_diag_right_hold_mm_q16;
            diag_error_q16 = smooth_final_scale_diag_error(diag_raw_error_q16);
            wall_error_q16 = diag_error_q16;
        } else {
            active_source = NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK;
        }
        break;
    case NAV_SMOOTH_FINAL_GUIDANCE_WALL_CENTER_HOLD:
        if (follow_left_valid && follow_right_valid) {
            wall_error_q16 =
                (wall_perception.right_mm_q16 - wall_perception.left_mm_q16)
                - smooth_final_center_diff_hold_mm_q16;
        } else {
            active_source = NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK;
        }
        break;
    case NAV_SMOOTH_FINAL_GUIDANCE_WALL_LEFT_HOLD:
        if (follow_left_valid) {
            wall_error_q16 =
                (smooth_final_left_hold_mm_q16 - wall_perception.left_mm_q16)
                * NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE;
        } else {
            active_source = NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK;
        }
        break;
    case NAV_SMOOTH_FINAL_GUIDANCE_WALL_RIGHT_HOLD:
        if (follow_right_valid) {
            wall_error_q16 =
                (wall_perception.right_mm_q16 - smooth_final_right_hold_mm_q16)
                * NAV_ADVANCE_WALL_SINGLE_SIDE_ERROR_SCALE;
        } else {
            active_source = NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK;
        }
        break;
    case NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY:
    case NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK:
    case NAV_SMOOTH_FINAL_GUIDANCE_NONE:
        active_source = NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY;
        break;
    }

    const q16_16_t yaw_setpoint_q16 = smooth_final_yaw_hold_initialized
        ? smooth_final_yaw_hold_deg_q16
        : sensors->yaw_deg_q16;
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, yaw_setpoint_q16);
    const int32_t pid_output_q16 = PID_Update_Fixed(&advance_yaw_pid, sensors->yaw_deg_q16, 10);
    const int32_t yaw_correction_pwm = FIXED_TO_INT(pid_output_q16);
    const q16_16_t yaw_error_q16 = yaw_setpoint_q16 - sensors->yaw_deg_q16;

    const q16_16_t wall_error_after_deadband_q16 = apply_wall_deadband(wall_error_q16);
    q16_16_t wall_previous_error_q16 = 0;
    q16_16_t wall_error_delta_q16 = 0;
    q16_16_t diag_previous_error_q16 = 0;
    q16_16_t diag_error_delta_q16 = 0;
    int32_t wall_p_term_pwm = 0;
    int32_t wall_d_term_pwm = 0;
    int32_t wall_raw_correction_pwm = 0;
    int16_t wall_correction_pwm = 0;
    int32_t diag_p_term_pwm = 0;
    int32_t diag_d_term_pwm = 0;
    int16_t diag_correction_pwm = 0;
    const bool use_diag_guidance =
        active_source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER
        || active_source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT
        || active_source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT
        || active_source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER_HOLD
        || active_source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT_HOLD
        || active_source == NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT_HOLD;
    if (smooth_source_is_setpoint_diag(active_source)) {
        smooth_final_diag_used = true;
    }
    const bool use_wall_hold =
        active_source == NAV_SMOOTH_FINAL_GUIDANCE_WALL_CENTER_HOLD
        || active_source == NAV_SMOOTH_FINAL_GUIDANCE_WALL_LEFT_HOLD
        || active_source == NAV_SMOOTH_FINAL_GUIDANCE_WALL_RIGHT_HOLD;
    if (use_wall_hold) {
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
    } else if (use_diag_guidance) {
        diag_previous_error_q16 = diagonal_guidance_has_previous_error
            ? diagonal_guidance_previous_error_q16
            : 0;
        diag_error_delta_q16 = diagonal_guidance_has_previous_error
            ? wall_error_after_deadband_q16 - diag_previous_error_q16
            : 0;
        diag_p_term_pwm = q16_to_pwm(wall_error_after_deadband_q16,
                                     diagonal_guidance_config.kp_pwm_per_mm);
        diag_d_term_pwm = q16_to_pwm(diag_error_delta_q16,
                                     diagonal_guidance_config.kd_pwm_per_mm_per_tick);
        diag_correction_pwm = clamp_diag_correction(diag_p_term_pwm + diag_d_term_pwm);
        wall_p_term_pwm = diag_p_term_pwm;
        wall_d_term_pwm = diag_d_term_pwm;
        wall_raw_correction_pwm = diag_p_term_pwm + diag_d_term_pwm;
        wall_correction_pwm = diag_correction_pwm;
        diagonal_guidance_previous_error_q16 = wall_error_after_deadband_q16;
        diagonal_guidance_has_previous_error = true;
        reset_advance_wall_pd();
    } else {
        reset_advance_wall_pd();
        reset_diagonal_guidance_pd();
    }

    const int32_t final_correction_pwm =
        (use_wall_hold || use_diag_guidance) ? wall_correction_pwm : yaw_correction_pwm;
    RobotCommand command = {
        clamp_pwm(NAV_SMOOTH_POST_YAW_BASE_LEFT_PWM + final_correction_pwm),
        clamp_pwm(NAV_SMOOTH_POST_YAW_BASE_RIGHT_PWM - final_correction_pwm)
    };

    turn_debug.advance_yaw_setpoint_deg_q16 = advance_yaw_pid.setpoint;
    turn_debug.advance_yaw_measured_deg_q16 = sensors->yaw_deg_q16;
    turn_debug.advance_yaw_error_deg_q16 = advance_yaw_pid.setpoint - sensors->yaw_deg_q16;
    turn_debug.advance_yaw_pid_output_q16 = pid_output_q16;
    turn_debug.advance_yaw_correction_pwm = clamp_pwm(yaw_correction_pwm);
    turn_debug.advance_yaw_kp_q16 = advance_yaw_pid_config.kp_q16;
    turn_debug.advance_yaw_ki_q16 = advance_yaw_pid_config.ki_q16;
    turn_debug.advance_yaw_kd_q16 = advance_yaw_pid_config.kd_q16;
    turn_debug.advance_yaw_output_limit_pwm = (int16_t)advance_yaw_pid_config.output_limit_pwm;
    turn_debug.smooth_phase = smooth_phase;
    turn_debug.smooth_done_reason = NAV_SMOOTH_DONE_NONE;
    turn_debug.smooth_post_yaw_elapsed_ms = smooth_post_yaw_elapsed_ms;
    turn_debug.smooth_left_base_pwm = NAV_SMOOTH_POST_YAW_BASE_LEFT_PWM;
    turn_debug.smooth_right_base_pwm = NAV_SMOOTH_POST_YAW_BASE_RIGHT_PWM;
    turn_debug.smooth_final_guidance_source = active_source;
    turn_debug.smooth_final_hold_initialized = smooth_final_hold_initialized;
    turn_debug.smooth_final_hold_source = smooth_final_hold_source;
    turn_debug.smooth_final_hold_recapture_count = smooth_final_hold_recapture_count;
    turn_debug.smooth_final_left_hold_mm_q16 = smooth_final_left_hold_mm_q16;
    turn_debug.smooth_final_right_hold_mm_q16 = smooth_final_right_hold_mm_q16;
    turn_debug.smooth_final_center_diff_hold_mm_q16 = smooth_final_center_diff_hold_mm_q16;
    turn_debug.smooth_final_wall_error_mm_q16 = wall_error_q16;
    turn_debug.smooth_final_wall_correction_pwm = wall_correction_pwm;
    turn_debug.smooth_final_yaw_correction_pwm = clamp_pwm(yaw_correction_pwm);
    turn_debug.smooth_final_yaw_hold_deg_q16 = smooth_final_yaw_hold_deg_q16;
    turn_debug.smooth_final_yaw_hold_initialized = smooth_final_yaw_hold_initialized;
    turn_debug.smooth_final_yaw_hold_recapture_count = smooth_final_yaw_hold_recapture_count;
    turn_debug.smooth_final_yaw_error_deg_q16 = yaw_error_q16;
    turn_debug.smooth_final_diag_left_valid = diag_left_valid;
    turn_debug.smooth_final_diag_right_valid = diag_right_valid;
    turn_debug.smooth_final_diag_left_mm_q16 = wall_perception.diag_left_mm_q16;
    turn_debug.smooth_final_diag_right_mm_q16 = wall_perception.diag_right_mm_q16;
    turn_debug.smooth_final_diag_target_mm_q16 =
        mm_to_q16(diagonal_guidance_config.target_mm);
    turn_debug.smooth_final_diag_error_scale_q16 =
        INT_TO_FIXED(diagonal_guidance_config.error_scale_num)
        / diagonal_guidance_config.error_scale_den;
    turn_debug.smooth_final_diag_mode = diagonal_guidance_config.smooth_final_mode;
    turn_debug.smooth_final_diag_hold_initialized = smooth_final_diag_hold_initialized;
    turn_debug.smooth_final_diag_left_hold_mm_q16 = smooth_final_diag_left_hold_mm_q16;
    turn_debug.smooth_final_diag_right_hold_mm_q16 = smooth_final_diag_right_hold_mm_q16;
    turn_debug.smooth_final_diag_center_diff_hold_mm_q16 =
        smooth_final_diag_center_diff_hold_mm_q16;
    turn_debug.smooth_final_diag_hold_recapture_count =
        smooth_final_diag_hold_recapture_count;
    turn_debug.smooth_final_diag_raw_error_mm_q16 = diag_raw_error_q16;
    turn_debug.smooth_final_diag_error_mm_q16 = diag_error_q16;
    turn_debug.smooth_final_follow_left_valid = follow_left_valid;
    turn_debug.smooth_final_follow_right_valid = follow_right_valid;
    sync_smooth_yaw_carry_debug();
    turn_debug.smooth_final_applied_correction_pwm = clamp_pwm(final_correction_pwm);
    turn_debug.diag_guidance_kp_pwm_per_mm = diagonal_guidance_config.kp_pwm_per_mm;
    turn_debug.diag_guidance_kd_pwm_per_mm_per_tick =
        diagonal_guidance_config.kd_pwm_per_mm_per_tick;
    turn_debug.diag_guidance_correction_limit_pwm =
        diagonal_guidance_config.correction_limit_pwm;
    turn_debug.diag_guidance_error_scale_num = diagonal_guidance_config.error_scale_num;
    turn_debug.diag_guidance_error_scale_den = diagonal_guidance_config.error_scale_den;
    turn_debug.diag_guidance_target_mm = diagonal_guidance_config.target_mm;
    turn_debug.diag_guidance_smooth_final_mode = diagonal_guidance_config.smooth_final_mode;
    turn_debug.diag_guidance_p_term_pwm = diag_p_term_pwm;
    turn_debug.diag_guidance_d_term_pwm = diag_d_term_pwm;
    turn_debug.diag_guidance_correction_pwm = diag_correction_pwm;
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
    advance_front_diag_preview_armed = false;
    advance_front_diag_preview_latched = false;
    advance_diag_preview_entry_yaw_deg_q16 = 0;
    advance_front_diag_used_for_yaw_carry = false;
    advance_diag_preview_entry_yaw_deg_q16 = 0;
    advance_front_diag_used_for_yaw_carry = false;
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
    smooth_post_yaw_elapsed_ms = 0;
    advance_elapsed_since_leave_start_line_ms = 0;
    approach_front_elapsed_ms = 0;
    approach_front_brake_elapsed_ms = 0;
    approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    center_pivot_elapsed_ms = 0;
    center_pivot_brake_elapsed_ms = 0;
    center_pivot_done_reason = NAV_CENTER_PIVOT_DONE_NONE;
    center_pivot_front_seen_white = false;
    special_ignore_rear_until_white = false;
    special_confirmed = false;
    special_detection_started_on_rear_line = false;
    special_detection_enabled_for_current_motion = false;
    advance_started_on_rear_line = false;
    advance_start_mode = NAV_ADVANCE_START_REAR_LINE;
    advance_from_centered_waiting_rear_white = false;
    advance_front_diag_preview_armed = false;
    advance_front_diag_preview_latched = false;
    map_walls_recorded_for_current_pose = false;
    map_initial_wall_snapshot_pending = false;
    initial_special_snapshot_pending = false;
    initial_special_snapshot_done = false;
    set_rear_line_trust(false, NAV_REAR_LINE_TRUST_NONE);
    nav_policy = NAV_POLICY_RIGHT_HAND_RULE;
    map_candidate_debug = (NavMapCandidateDebug){0};
    clear_special_mark_debug();
    nav_core_plan_clear();
    nav_core_route_clear_debug();
    nav_core_route_eval_clear_debug();
    clear_wall_perception();
    reset_advance_wall_pd();
    reset_wall_caution_state(NAV_WALL_CAUTION_LOSS_ACTION_END);
    reset_diagonal_guidance_pd();
    reset_forward_guidance_yaw_hold();
    advance_guidance_mode = NAV_ADVANCE_GUIDANCE_WALL_ASSIST;
    reset_advance_wall_config();
    reset_diagonal_guidance_config();
    reset_wall_caution_config();
    reset_smooth_yaw_carry_config();
    reset_smooth_yaw_carry_runtime();
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
    advance_start_mode = NAV_ADVANCE_START_REAR_LINE;
    advance_from_centered_waiting_rear_white = false;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    advance_front_diag_preview_armed = false;
    advance_front_diag_preview_latched = false;
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
    smooth_post_yaw_elapsed_ms = 0;
    approach_front_elapsed_ms = 0;
    approach_front_brake_elapsed_ms = 0;
    approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    center_pivot_done_reason = NAV_CENTER_PIVOT_DONE_NONE;
    reset_turn_debug();
    reset_special_detection_state();
    reset_advance_wall_pd();
    reset_wall_caution_state(NAV_WALL_CAUTION_LOSS_NONE);
    reset_forward_guidance_yaw_hold();
    PID_Reset(&advance_yaw_pid);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, 0);
}

void nav_core_start_advance_until_rear_black_from_centered_pose(void)
{
    current_state = NAV_STATE_ADVANCING_UNTIL_REAR_BLACK;
    current_action = NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK;
    advance_start_mode = NAV_ADVANCE_START_CENTERED_POSE;
    advance_from_centered_waiting_rear_white = false;
    advance_front_diag_preview_armed = false;
    advance_front_diag_preview_latched = false;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
    smooth_post_yaw_elapsed_ms = 0;
    approach_front_elapsed_ms = 0;
    approach_front_brake_elapsed_ms = 0;
    approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    center_pivot_done_reason = NAV_CENTER_PIVOT_DONE_NONE;
    reset_turn_debug();
    reset_special_detection_state();
    advance_start_mode = NAV_ADVANCE_START_CENTERED_POSE;
    advance_from_centered_waiting_rear_white = false;
    special_ignore_rear_until_white = false;
    special_detection_started_on_rear_line = false;
    special_detection_enabled_for_current_motion = false;
    turn_debug.advance_start_mode = advance_start_mode;
    turn_debug.advance_from_centered_waiting_rear_white =
        advance_from_centered_waiting_rear_white;
    reset_advance_wall_pd();
    reset_wall_caution_state(NAV_WALL_CAUTION_LOSS_NONE);
    reset_forward_guidance_yaw_hold();
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
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
    approach_front_elapsed_ms = 0;
    approach_front_brake_elapsed_ms = 0;
    approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    reset_turn_debug();
    reset_special_detection_state();
    set_approach_front_phase(NAV_APPROACH_FRONT_PHASE_DRIVE);
    reset_advance_wall_pd();
    reset_wall_caution_state(NAV_WALL_CAUTION_LOSS_ACTION_END);
    reset_forward_guidance_yaw_hold();
    PID_Reset(&advance_yaw_pid);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, 0);
    /* TODO: add front-wall alignment using front_left - front_right before pivot. */
}

void nav_core_start_center_in_cell_for_pivot_by_front_line(void)
{
    current_state = NAV_STATE_CENTERING_IN_CELL_FOR_PIVOT;
    current_action = NAV_ACTION_CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE;
    action_start_yaw_q16 = 0;
    action_target_yaw_q16 = 0;
    smooth_phase = NAV_SMOOTH_PHASE_NONE;
    advance_phase = NAV_ADVANCE_PHASE_NONE;
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    reset_turn_debug();
    reset_special_detection_state();
    set_center_pivot_phase(NAV_CENTER_PIVOT_PHASE_INIT);
    reset_advance_wall_pd();
    reset_wall_caution_state(NAV_WALL_CAUTION_LOSS_ACTION_END);
    reset_forward_guidance_yaw_hold();
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
        approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
        center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
        smooth_post_yaw_elapsed_ms = 0;
        reset_turn_debug();
        reset_special_detection_state();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_NEG_90_DEG;
    advance_start_mode = NAV_ADVANCE_START_REAR_LINE;
    advance_from_centered_waiting_rear_white = false;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid,
                           -INT_TO_FIXED(smooth_turn_config.target_yaw_rate_deg_s));
    reset_turn_debug();
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    set_smooth_phase(sensors->floor_rear_black
                         ? NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE
                         : NAV_SMOOTH_PHASE_SEEK_TARGET_LINE);
    begin_special_detection_for_motion(sensors->floor_rear_black);
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
        center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
        smooth_post_yaw_elapsed_ms = 0;
        reset_turn_debug();
        reset_special_detection_state();
        return;
    }

    action_start_yaw_q16 = sensors->yaw_deg_q16;
    action_target_yaw_q16 = Q16_90_DEG;
    advance_start_mode = NAV_ADVANCE_START_REAR_LINE;
    advance_from_centered_waiting_rear_white = false;
    PID_Reset(&turn_yaw_rate_pid);
    PID_Set_Setpoint_Fixed(&turn_yaw_rate_pid,
                           INT_TO_FIXED(smooth_turn_config.target_yaw_rate_deg_s));
    reset_turn_debug();
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
    approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
    set_smooth_phase(sensors->floor_rear_black
                         ? NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE
                         : NAV_SMOOTH_PHASE_SEEK_TARGET_LINE);
    begin_special_detection_for_motion(sensors->floor_rear_black);
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
        center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
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
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
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
        center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
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
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
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
        center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
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
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
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
    center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
    smooth_post_yaw_elapsed_ms = 0;
    advance_elapsed_since_leave_start_line_ms = 0;
    approach_front_elapsed_ms = 0;
    approach_front_brake_elapsed_ms = 0;
    approach_front_done_reason = NAV_APPROACH_FRONT_DONE_NONE;
    center_pivot_elapsed_ms = 0;
    center_pivot_brake_elapsed_ms = 0;
    center_pivot_done_reason = NAV_CENTER_PIVOT_DONE_NONE;
    center_pivot_front_seen_white = false;
    special_ignore_rear_until_white = false;
    special_confirmed = false;
    special_detection_started_on_rear_line = false;
    special_detection_enabled_for_current_motion = false;
    map_walls_recorded_for_current_pose = false;
    map_initial_wall_snapshot_pending = false;
    initial_special_snapshot_pending = false;
    initial_special_snapshot_done = false;
    set_rear_line_trust(false, NAV_REAR_LINE_TRUST_NONE);
    map_candidate_debug = (NavMapCandidateDebug){0};
    clear_special_mark_debug();
    reset_smooth_yaw_carry_runtime();
    reset_turn_debug();
    reset_advance_wall_pd();
    reset_wall_caution_state(NAV_WALL_CAUTION_LOSS_ACTION_END);
    reset_diagonal_guidance_pd();
    reset_forward_guidance_yaw_hold();
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
    reset_forward_guidance_yaw_hold();
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
    reset_diagonal_guidance_pd();
    reset_forward_guidance_yaw_hold();
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
    reset_diagonal_guidance_pd();
    reset_forward_guidance_yaw_hold();
    clear_live_turn_debug();
}

void nav_core_set_diagonal_guidance_config(const NavDiagonalGuidanceConfig *config)
{
    if (config == 0) {
        return;
    }

    diagonal_guidance_config.kp_pwm_per_mm =
        clamp_i16(config->kp_pwm_per_mm, 0, NAV_ADVANCE_WALL_KP_PWM_PER_MM_MAX);
    diagonal_guidance_config.kd_pwm_per_mm_per_tick =
        clamp_i16(config->kd_pwm_per_mm_per_tick, 0, NAV_ADVANCE_WALL_KD_PWM_PER_MM_PER_TICK_MAX);
    diagonal_guidance_config.correction_limit_pwm =
        clamp_i16(config->correction_limit_pwm, 0, NAV_ADVANCE_WALL_OUTPUT_LIMIT_PWM_MAX);
    diagonal_guidance_config.error_scale_num = clamp_i16(config->error_scale_num, 0, 100);
    diagonal_guidance_config.error_scale_den = clamp_i16(config->error_scale_den, 1, 100);
    diagonal_guidance_config.target_mm = clamp_i16(config->target_mm, 50, 160);
    diagonal_guidance_config.smooth_final_mode =
        config->smooth_final_mode == NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT
            ? NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT
            : NAV_SMOOTH_FINAL_DIAG_MODE_HOLD_RELATIVE;
    reset_diagonal_guidance_pd();
    clear_live_turn_debug();
}

void nav_core_get_diagonal_guidance_config(NavDiagonalGuidanceConfig *config)
{
    if (config == 0) {
        return;
    }

    *config = diagonal_guidance_config;
}

void nav_core_reset_diagonal_guidance_defaults(void)
{
    reset_diagonal_guidance_config();
    reset_diagonal_guidance_pd();
    clear_live_turn_debug();
}

void nav_core_set_wall_caution_config(const NavWallCautionConfig *config)
{
    if (config == 0) {
        return;
    }

    wall_caution_config.enabled = config->enabled;
    wall_caution_config.timeout_ms =
        (uint16_t)clamp_i16((int16_t)config->timeout_ms,
                            0,
                            NAV_WALL_CAUTION_TIMEOUT_MS_MAX);
    wall_caution_config.delta_max_mm = clamp_i16(config->delta_max_mm, 0, 100);
    wall_caution_config.kp_pwm_per_mm =
        clamp_i16(config->kp_pwm_per_mm, 0, NAV_ADVANCE_WALL_KP_PWM_PER_MM_MAX);
    wall_caution_config.kd_pwm_per_mm_per_tick =
        clamp_i16(config->kd_pwm_per_mm_per_tick, 0, NAV_ADVANCE_WALL_KD_PWM_PER_MM_PER_TICK_MAX);
    wall_caution_config.correction_limit_pwm =
        clamp_i16(config->correction_limit_pwm, 0, NAV_ADVANCE_WALL_OUTPUT_LIMIT_PWM_MAX);
    reset_wall_caution_state(NAV_WALL_CAUTION_LOSS_NONE);
    clear_live_turn_debug();
}

void nav_core_get_wall_caution_config(NavWallCautionConfig *config)
{
    if (config == 0) {
        return;
    }

    *config = wall_caution_config;
}

void nav_core_reset_wall_caution_defaults(void)
{
    reset_wall_caution_config();
    reset_wall_caution_state(NAV_WALL_CAUTION_LOSS_NONE);
    clear_live_turn_debug();
}

void nav_core_set_smooth_yaw_carry_config(const NavSmoothYawCarryConfig *config)
{
    if (config == 0) {
        return;
    }

    smooth_yaw_carry_config.enabled = config->enabled;
    smooth_yaw_carry_config.only_setpoint = config->only_setpoint;
    smooth_yaw_carry_config.require_diag = config->require_diag;
    smooth_yaw_carry_config.allow_advance_preview = config->allow_advance_preview;
    smooth_yaw_carry_config.min_abs_deg_q16 = abs_q16(config->min_abs_deg_q16);
    smooth_yaw_carry_config.max_abs_deg_q16 = abs_q16(config->max_abs_deg_q16);
    if (smooth_yaw_carry_config.max_abs_deg_q16 < smooth_yaw_carry_config.min_abs_deg_q16) {
        smooth_yaw_carry_config.max_abs_deg_q16 = smooth_yaw_carry_config.min_abs_deg_q16;
    }
    smooth_yaw_carry_config.offset_scale_num =
        clamp_i16(config->offset_scale_num, 0, 100);
    smooth_yaw_carry_config.offset_scale_den =
        clamp_i16(config->offset_scale_den, 1, 100);
    sync_smooth_yaw_carry_debug();
    clear_live_turn_debug();
}

void nav_core_get_smooth_yaw_carry_config(NavSmoothYawCarryConfig *config)
{
    if (config == 0) {
        return;
    }

    *config = smooth_yaw_carry_config;
}

void nav_core_reset_smooth_yaw_carry_defaults(void)
{
    reset_smooth_yaw_carry_config();
    reset_smooth_yaw_carry_runtime();
    clear_live_turn_debug();
}

bool nav_core_prepare_smooth_yaw_carry_for_next_action(bool next_action_is_smooth)
{
    return evaluate_smooth_yaw_carry_candidate(next_action_is_smooth);
}

bool nav_core_has_smooth_yaw_carry_pending(void)
{
    return smooth_yaw_carry_pending;
}

q16_16_t nav_core_consume_smooth_yaw_carry_offset_q16(void)
{
    if (!smooth_yaw_carry_pending) {
        smooth_yaw_carry_used = false;
        sync_smooth_yaw_carry_debug();
        return 0;
    }

    const q16_16_t offset_q16 = smooth_yaw_carry_offset_deg_q16;
    smooth_yaw_carry_pending = false;
    smooth_yaw_carry_candidate_available = false;
    smooth_yaw_carry_used = true;
    sync_smooth_yaw_carry_debug();
    return offset_q16;
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
    reset_forward_guidance_yaw_hold();
    clear_live_turn_debug();
}

void nav_core_reset_advance_yaw_pid_defaults(void)
{
    set_advance_yaw_pid_defaults();
    apply_advance_yaw_pid_config(true);
    PID_Set_Setpoint_Fixed(&advance_yaw_pid, 0);
    reset_forward_guidance_yaw_hold();
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
    initial_special_snapshot_pending = true;
    initial_special_snapshot_done = false;
    set_rear_line_trust(false, NAV_REAR_LINE_TRUST_NONE);
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
    if (policy != NAV_POLICY_RIGHT_HAND_RULE
        && policy != NAV_POLICY_MAP_PREFER_UNVISITED
        && policy != NAV_POLICY_SMART_RECOGNITION) {
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

NavFloodStatus nav_core_flood_fill_to_cell(int8_t goal_x, int8_t goal_y)
{
    return nav_flood_fill_to_cell(goal_x, goal_y);
}

uint16_t nav_core_flood_get_cost(int8_t cell_x, int8_t cell_y)
{
    return nav_flood_get_cost(cell_x, cell_y);
}

void nav_core_flood_get_debug(NavFloodDebugSnapshot *snapshot)
{
    nav_flood_get_debug_snapshot(snapshot);
}

void nav_core_flood_clear(void)
{
    nav_flood_clear();
}

bool nav_core_flood_is_valid(void)
{
    return nav_flood_is_valid();
}

bool nav_core_rear_line_trusted_for_decision(void)
{
    return rear_line_trusted_for_decision;
}

NavRearLineTrustSource nav_core_rear_line_trust_source(void)
{
    return rear_line_trust_source;
}

static NavRecommendedAction recommend_right_hand_rule(const RobotSensors *sensors)
{
    if (sensors == 0) {
        return NAV_RECOMMENDED_NONE;
    }

    if (!sensors->floor_rear_black || !rear_line_trusted_for_decision) {
        if (sensors->floor_rear_black && !rear_line_trusted_for_decision) {
            return NAV_RECOMMENDED_ACQUIRE_REAR_LINE;
        }
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

    if (sensors == 0 || !sensors->floor_rear_black || !rear_line_trusted_for_decision) {
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
    if (nav_policy == NAV_POLICY_MAP_PREFER_UNVISITED
        || nav_policy == NAV_POLICY_SMART_RECOGNITION) {
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
        advance_front_diag_preview_armed = false;
        advance_front_diag_preview_latched = false;
        approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
        center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
        clear_live_turn_debug();
        RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
        return command;
    }

    record_current_map_cell_if_ready(sensors);
    record_initial_special_snapshot_if_ready(sensors);
    update_floor_line_debug(sensors);

    if (current_action == NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK) {
        center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
        if (advance_phase == NAV_ADVANCE_PHASE_NONE) {
            if (advance_start_mode == NAV_ADVANCE_START_CENTERED_POSE) {
                advance_started_on_rear_line = false;
                advance_from_centered_waiting_rear_white = sensors->floor_rear_black;
                special_ignore_rear_until_white = false;
                special_detection_started_on_rear_line = false;
                special_detection_enabled_for_current_motion = false;
                set_advance_phase(NAV_ADVANCE_PHASE_SEEK_TARGET_LINE);
            } else {
                set_advance_phase(sensors->floor_rear_black
                                      ? NAV_ADVANCE_PHASE_WAIT_LEAVE_START_LINE
                                      : NAV_ADVANCE_PHASE_SEEK_TARGET_LINE);
                advance_started_on_rear_line = sensors->floor_rear_black;
                begin_special_detection_for_motion(sensors->floor_rear_black);
            }
            advance_diag_preview_entry_yaw_deg_q16 = sensors->yaw_deg_q16;
            advance_front_diag_used_for_yaw_carry = false;
            update_floor_line_debug(sensors);
        }

        if (advance_start_mode == NAV_ADVANCE_START_CENTERED_POSE
            && advance_from_centered_waiting_rear_white
            && !sensors->floor_rear_black) {
            advance_from_centered_waiting_rear_white = false;
            update_floor_line_debug(sensors);
        }

        if (advance_start_mode == NAV_ADVANCE_START_REAR_LINE
            && advance_phase == NAV_ADVANCE_PHASE_WAIT_LEAVE_START_LINE
            && !sensors->floor_rear_black) {
            set_advance_phase(NAV_ADVANCE_PHASE_SEEK_TARGET_LINE);
        }

        if (advance_start_mode == NAV_ADVANCE_START_REAR_LINE
            && advance_phase == NAV_ADVANCE_PHASE_SEEK_TARGET_LINE) {
            update_special_detection(
                sensors,
                NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL,
                true);
        } else {
            special_confirmed = false;
            turn_debug.special_candidate = sensors->floor_front_black && sensors->floor_rear_black;
            turn_debug.special_confirmed = false;
            turn_debug.special_ignore_rear_until_white = special_ignore_rear_until_white;
            turn_debug.special_detection_started_on_rear_line =
                special_detection_started_on_rear_line;
            turn_debug.special_detection_enabled_for_current_motion =
                special_detection_enabled_for_current_motion;
            update_floor_line_debug(sensors);
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
        center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
        turn_debug.approach_front_target_mm_q16 = NAV_APPROACH_FRONT_TARGET_MM_Q16;
        turn_debug.approach_front_left_mm_q16 = sensors->ir_front_left_mm_q16;
        turn_debug.approach_front_right_mm_q16 = sensors->ir_front_right_mm_q16;

        if (approach_front_phase == NAV_APPROACH_FRONT_PHASE_NONE) {
            set_approach_front_phase(NAV_APPROACH_FRONT_PHASE_DRIVE);
        }
        if (special_detection_context != NAV_SPECIAL_DETECT_IN_CELL_AUX_TRANSLATION) {
            begin_special_detection_for_aux_translation(sensors->floor_rear_black);
        }

        if (approach_front_phase == NAV_APPROACH_FRONT_PHASE_DRIVE) {
            update_special_detection(
                sensors,
                NAV_SPECIAL_DETECT_IN_CELL_AUX_TRANSLATION,
                true);
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

    if (current_action == NAV_ACTION_CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE) {
        smooth_phase = NAV_SMOOTH_PHASE_NONE;
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        approach_front_phase = NAV_APPROACH_FRONT_PHASE_NONE;
        turn_debug.center_pivot_front_black = sensors->floor_front_black;
        turn_debug.center_pivot_rear_black = sensors->floor_rear_black;
        turn_debug.center_pivot_front_seen_white = center_pivot_front_seen_white;

        if (center_pivot_phase == NAV_CENTER_PIVOT_PHASE_NONE) {
            set_center_pivot_phase(NAV_CENTER_PIVOT_PHASE_INIT);
        }

        if (center_pivot_phase == NAV_CENTER_PIVOT_PHASE_INIT) {
            if (!sensors->floor_rear_black) {
                return finish_center_in_cell_for_pivot_by_front_line(
                    sensors,
                    NAV_CENTER_PIVOT_DONE_START_NOT_ON_REAR_LINE);
            }
            set_center_pivot_phase(NAV_CENTER_PIVOT_PHASE_WAIT_LEAVE_START_LINE);
            begin_special_detection_for_aux_translation(true);
        }

        if (center_pivot_phase == NAV_CENTER_PIVOT_PHASE_WAIT_LEAVE_START_LINE
            && !sensors->floor_rear_black) {
            set_center_pivot_phase(NAV_CENTER_PIVOT_PHASE_WAIT_FRONT_WHITE);
        }

        if (center_pivot_phase == NAV_CENTER_PIVOT_PHASE_WAIT_FRONT_WHITE
            || center_pivot_phase == NAV_CENTER_PIVOT_PHASE_SEEK_FRONT_LINE) {
            update_special_detection(
                sensors,
                NAV_SPECIAL_DETECT_IN_CELL_AUX_TRANSLATION,
                true);
        }

        if (center_pivot_phase == NAV_CENTER_PIVOT_PHASE_WAIT_FRONT_WHITE
            && !sensors->floor_front_black) {
            center_pivot_front_seen_white = true;
            set_center_pivot_phase(NAV_CENTER_PIVOT_PHASE_SEEK_FRONT_LINE);
        }

        if (center_pivot_phase == NAV_CENTER_PIVOT_PHASE_SEEK_FRONT_LINE
            && sensors->floor_front_black
            /* Avoid treating the central special-cell marker as the exit boundary. */
            && center_pivot_elapsed_ms >= NAV_CENTER_PIVOT_FRONT_LINE_MIN_MS) {
            center_pivot_done_reason = NAV_CENTER_PIVOT_DONE_FRONT_LINE;
            set_center_pivot_phase(NAV_CENTER_PIVOT_PHASE_BRAKE_SETTLE);
        } else if (center_pivot_phase != NAV_CENTER_PIVOT_PHASE_BRAKE_SETTLE
                   && center_pivot_elapsed_ms >= NAV_CENTER_PIVOT_TIMEOUT_MS) {
            center_pivot_done_reason = NAV_CENTER_PIVOT_DONE_TIMEOUT;
            set_center_pivot_phase(NAV_CENTER_PIVOT_PHASE_BRAKE_SETTLE);
        }

        if (center_pivot_phase == NAV_CENTER_PIVOT_PHASE_BRAKE_SETTLE) {
            if (center_pivot_brake_elapsed_ms >= NAV_BRAKE_SETTLE_MS) {
                return finish_center_in_cell_for_pivot_by_front_line(
                    sensors,
                    center_pivot_done_reason);
            }

            center_pivot_brake_elapsed_ms += 10;
            turn_debug.center_pivot_phase = center_pivot_phase;
            turn_debug.center_pivot_done_reason = center_pivot_done_reason;
            turn_debug.center_pivot_elapsed_ms = center_pivot_elapsed_ms;
            turn_debug.center_pivot_brake_elapsed_ms = center_pivot_brake_elapsed_ms;
            turn_debug.center_pivot_base_left_pwm = NAV_CENTER_PIVOT_BASE_LEFT_PWM;
            turn_debug.center_pivot_base_right_pwm = NAV_CENTER_PIVOT_BASE_RIGHT_PWM;
            turn_debug.center_pivot_correction_pwm = 0;
            RobotCommand command = {NAV_PWM_STOP, NAV_PWM_STOP};
            return command;
        }

        center_pivot_elapsed_ms += 10;
        return center_in_cell_for_pivot_command(sensors);
    }

    if (current_action == NAV_ACTION_SMOOTH_TURN_RIGHT) {
        center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        if (smooth_phase == NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE
            && !sensors->floor_rear_black) {
            set_smooth_phase(NAV_SMOOTH_PHASE_SEEK_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE) {
            update_special_detection(
                sensors,
                NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL,
                true);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE
            && rear_black_for_line(sensors)) {
            return finish_smooth_turn(sensors, NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE) {
            update_special_detection(
                sensors,
                NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL,
                true);
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
            enter_smooth_post_yaw_seek(sensors);
            return smooth_post_yaw_seek_command(sensors);
        }

        return smooth_turn_command(sensors, 1);
    }

    if (current_action == NAV_ACTION_SMOOTH_TURN_LEFT) {
        center_pivot_phase = NAV_CENTER_PIVOT_PHASE_NONE;
        advance_phase = NAV_ADVANCE_PHASE_NONE;
        if (smooth_phase == NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE
            && !sensors->floor_rear_black) {
            set_smooth_phase(NAV_SMOOTH_PHASE_SEEK_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE) {
            update_special_detection(
                sensors,
                NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL,
                true);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_SEEK_TARGET_LINE
            && rear_black_for_line(sensors)) {
            return finish_smooth_turn(sensors, NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE);
        }

        if (smooth_phase == NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE) {
            update_special_detection(
                sensors,
                NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL,
                true);
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
            enter_smooth_post_yaw_seek(sensors);
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
