#ifndef NAV_CORE_H
#define NAV_CORE_H

#include "nav_types.h"
#include "nav_map.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum NavState {
    NAV_STATE_IDLE = 0,
    NAV_STATE_ADVANCING_UNTIL_REAR_BLACK,
    NAV_STATE_APPROACHING_FRONT_WALL_FOR_PIVOT,
    NAV_STATE_CENTERING_IN_CELL_FOR_PIVOT,
    NAV_STATE_SMOOTH_TURNING,
    NAV_STATE_PIVOT_TURNING,
    NAV_STATE_DONE
} NavState;

typedef enum NavAction {
    NAV_ACTION_NONE = 0,
    NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK,
    NAV_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT,
    NAV_ACTION_CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE,
    NAV_ACTION_SMOOTH_TURN_LEFT,
    NAV_ACTION_SMOOTH_TURN_RIGHT,
    NAV_ACTION_PIVOT_TURN_LEFT,
    NAV_ACTION_PIVOT_TURN_RIGHT,
    NAV_ACTION_PIVOT_TURN_180
} NavAction;

typedef enum NavSmoothPhase {
    NAV_SMOOTH_PHASE_NONE = 0,
    NAV_SMOOTH_PHASE_WAIT_LEAVE_START_LINE,
    NAV_SMOOTH_PHASE_SEEK_TARGET_LINE,
    NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE,
    NAV_SMOOTH_PHASE_DONE
} NavSmoothPhase;

typedef enum NavSmoothDoneReason {
    NAV_SMOOTH_DONE_NONE = 0,
    NAV_SMOOTH_DONE_REAR_SENSOR_TARGET_LINE,
    NAV_SMOOTH_DONE_YAW_FALLBACK,
    NAV_SMOOTH_DONE_TIMEOUT
} NavSmoothDoneReason;

typedef enum NavAdvancePhase {
    NAV_ADVANCE_PHASE_NONE = 0,
    NAV_ADVANCE_PHASE_WAIT_LEAVE_START_LINE,
    NAV_ADVANCE_PHASE_SEEK_TARGET_LINE
} NavAdvancePhase;

typedef enum NavAdvanceDoneReason {
    NAV_ADVANCE_DONE_NONE = 0,
    NAV_ADVANCE_DONE_REAR_SENSOR_TARGET_LINE
} NavAdvanceDoneReason;

typedef enum NavAdvanceStartMode {
    NAV_ADVANCE_START_REAR_LINE = 0,
    NAV_ADVANCE_START_CENTERED_POSE
} NavAdvanceStartMode;

typedef enum NavRearLineTrustSource {
    NAV_REAR_LINE_TRUST_NONE = 0,
    NAV_REAR_LINE_TRUST_INITIAL_REAR_LINE,
    NAV_REAR_LINE_TRUST_ADVANCE_DONE,
    NAV_REAR_LINE_TRUST_SMOOTH_DONE,
    NAV_REAR_LINE_TRUST_CENTERED_ADVANCE_DONE,
    NAV_REAR_LINE_TRUST_OTHER
} NavRearLineTrustSource;

typedef enum NavApproachFrontPhase {
    NAV_APPROACH_FRONT_PHASE_NONE = 0,
    NAV_APPROACH_FRONT_PHASE_DRIVE,
    NAV_APPROACH_FRONT_PHASE_BRAKE_SETTLE,
    NAV_APPROACH_FRONT_PHASE_DONE
} NavApproachFrontPhase;

typedef enum NavApproachFrontDoneReason {
    NAV_APPROACH_FRONT_DONE_NONE = 0,
    NAV_APPROACH_FRONT_DONE_TARGET_DISTANCE,
    NAV_APPROACH_FRONT_DONE_TIMEOUT
} NavApproachFrontDoneReason;

typedef enum NavCenterPivotPhase {
    NAV_CENTER_PIVOT_PHASE_NONE = 0,
    NAV_CENTER_PIVOT_PHASE_INIT,
    NAV_CENTER_PIVOT_PHASE_WAIT_LEAVE_START_LINE,
    NAV_CENTER_PIVOT_PHASE_WAIT_FRONT_WHITE,
    NAV_CENTER_PIVOT_PHASE_SEEK_FRONT_LINE,
    NAV_CENTER_PIVOT_PHASE_BRAKE_SETTLE,
    NAV_CENTER_PIVOT_PHASE_DONE
} NavCenterPivotPhase;

typedef enum NavCenterPivotDoneReason {
    NAV_CENTER_PIVOT_DONE_NONE = 0,
    NAV_CENTER_PIVOT_DONE_FRONT_LINE,
    NAV_CENTER_PIVOT_DONE_TIMEOUT,
    NAV_CENTER_PIVOT_DONE_START_NOT_ON_REAR_LINE
} NavCenterPivotDoneReason;

typedef enum NavAdvanceGuidanceMode {
    NAV_ADVANCE_GUIDANCE_YAW_ONLY = 0,
    NAV_ADVANCE_GUIDANCE_WALL_ASSIST
} NavAdvanceGuidanceMode;

typedef enum NavAdvanceCorrectionSource {
    NAV_ADVANCE_CORRECTION_YAW_PD = 0,
    NAV_ADVANCE_CORRECTION_WALL_LEFT,
    NAV_ADVANCE_CORRECTION_WALL_RIGHT,
    NAV_ADVANCE_CORRECTION_WALL_CENTER,
    NAV_ADVANCE_CORRECTION_WALL_LEFT_CAUTION,
    NAV_ADVANCE_CORRECTION_WALL_RIGHT_CAUTION,
    NAV_ADVANCE_CORRECTION_DIAG_LEFT,
    NAV_ADVANCE_CORRECTION_DIAG_RIGHT,
    NAV_ADVANCE_CORRECTION_DIAG_CENTER
} NavAdvanceCorrectionSource;

typedef enum NavWallCautionConfidence {
    NAV_WALL_CAUTION_CONFIDENCE_LOST = 0,
    NAV_WALL_CAUTION_CONFIDENCE_CONFIRMED,
    NAV_WALL_CAUTION_CONFIDENCE_CAUTION
} NavWallCautionConfidence;

typedef enum NavWallCautionLossReason {
    NAV_WALL_CAUTION_LOSS_NONE = 0,
    NAV_WALL_CAUTION_LOSS_DISABLED,
    NAV_WALL_CAUTION_LOSS_LATERAL_LOST,
    NAV_WALL_CAUTION_LOSS_DELTA_MAX,
    NAV_WALL_CAUTION_LOSS_TIMEOUT,
    NAV_WALL_CAUTION_LOSS_ACTION_END
} NavWallCautionLossReason;

typedef enum NavAdvanceFrontDiagSource {
    NAV_ADVANCE_FRONT_DIAG_NONE = 0,
    NAV_ADVANCE_FRONT_DIAG_LEFT,
    NAV_ADVANCE_FRONT_DIAG_RIGHT,
    NAV_ADVANCE_FRONT_DIAG_CENTER
} NavAdvanceFrontDiagSource;

typedef enum NavSmoothFinalGuidanceSource {
    NAV_SMOOTH_FINAL_GUIDANCE_NONE = 0,
    NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER,
    NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT,
    NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT,
    NAV_SMOOTH_FINAL_GUIDANCE_DIAG_CENTER_HOLD,
    NAV_SMOOTH_FINAL_GUIDANCE_DIAG_LEFT_HOLD,
    NAV_SMOOTH_FINAL_GUIDANCE_DIAG_RIGHT_HOLD,
    NAV_SMOOTH_FINAL_GUIDANCE_WALL_CENTER_HOLD,
    NAV_SMOOTH_FINAL_GUIDANCE_WALL_LEFT_HOLD,
    NAV_SMOOTH_FINAL_GUIDANCE_WALL_RIGHT_HOLD,
    NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY,
    NAV_SMOOTH_FINAL_GUIDANCE_YAW_ONLY_FALLBACK
} NavSmoothFinalGuidanceSource;

typedef enum NavSmoothFinalDiagonalMode {
    NAV_SMOOTH_FINAL_DIAG_MODE_HOLD_RELATIVE = 0,
    NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT = 1
} NavSmoothFinalDiagonalMode;

typedef enum NavSmoothYawCarryRejectedReason {
    NAV_SMOOTH_YAW_CARRY_REJECT_NONE = 0,
    NAV_SMOOTH_YAW_CARRY_REJECT_DISABLED,
    NAV_SMOOTH_YAW_CARRY_REJECT_NOT_NEXT_SMOOTH,
    NAV_SMOOTH_YAW_CARRY_REJECT_NOT_SETPOINT,
    NAV_SMOOTH_YAW_CARRY_REJECT_NO_DIAG_USED,
    NAV_SMOOTH_YAW_CARRY_REJECT_OFFSET_TOO_SMALL,
    NAV_SMOOTH_YAW_CARRY_REJECT_OFFSET_TOO_LARGE
} NavSmoothYawCarryRejectedReason;

typedef enum NavRecommendedAction {
    NAV_RECOMMENDED_NONE = 0,
    NAV_RECOMMENDED_ACQUIRE_REAR_LINE,
    NAV_RECOMMENDED_ADVANCE_LINE,
    NAV_RECOMMENDED_SMOOTH_LEFT,
    NAV_RECOMMENDED_SMOOTH_RIGHT,
    NAV_RECOMMENDED_PIVOT_180,
    NAV_RECOMMENDED_RECOVERY_PIVOT_180_FRONT_BLOCKED
} NavRecommendedAction;

typedef enum NavPolicy {
    NAV_POLICY_RIGHT_HAND_RULE = 0,
    NAV_POLICY_MAP_PREFER_UNVISITED,
    NAV_POLICY_SMART_RECOGNITION
} NavPolicy;

enum {
    NAV_PLAN_MAX_ACTIONS = 64
};

typedef enum NavPlanAction {
    NAV_PLAN_ACTION_NONE = 0,
    NAV_PLAN_ACTION_ADVANCE_LINE,
    NAV_PLAN_ACTION_SMOOTH_LEFT,
    NAV_PLAN_ACTION_SMOOTH_RIGHT,
    NAV_PLAN_ACTION_PIVOT_180,
    NAV_PLAN_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT,
    NAV_PLAN_ACTION_CENTER_AND_PIVOT_180
} NavPlanAction;

typedef struct NavPlanDebugSnapshot {
    uint8_t capacity;
    uint8_t count;
    uint8_t head;
    uint8_t tail;
    NavPlanAction next_action;
    bool overflow;
} NavPlanDebugSnapshot;

typedef enum NavRouteStatus {
    NAV_ROUTE_STATUS_IDLE = 0,
    NAV_ROUTE_STATUS_FOUND,
    NAV_ROUTE_STATUS_FRONTIER_ALREADY_HERE,
    NAV_ROUTE_STATUS_NO_PATH,
    NAV_ROUTE_STATUS_NO_FRONTIER,
    NAV_ROUTE_STATUS_TARGET_OUT_OF_BOUNDS,
    NAV_ROUTE_STATUS_TARGET_NOT_VISITED,
    NAV_ROUTE_STATUS_ROUTE_TOO_LONG,
    NAV_ROUTE_STATUS_QUEUE_OVERFLOW
} NavRouteStatus;

typedef enum NavFrontierExitRelative {
    NAV_FRONTIER_EXIT_NONE = 0,
    NAV_FRONTIER_EXIT_FRONT,
    NAV_FRONTIER_EXIT_RIGHT,
    NAV_FRONTIER_EXIT_LEFT
} NavFrontierExitRelative;

typedef struct NavRouteDebugSnapshot {
    NavRouteStatus status;
    int8_t target_cell_x;
    int8_t target_cell_y;
    int8_t start_cell_x;
    int8_t start_cell_y;
    NavMapDirection start_dir;
    uint8_t route_length;
    uint16_t expanded_states;
    NavPlanAction first_action;
    NavPlanAction last_action;
    bool loaded_into_plan_queue;
    bool frontier_mode;
    int8_t frontier_target_cell_x;
    int8_t frontier_target_cell_y;
    NavMapDirection frontier_target_dir;
    NavMapDirection frontier_exit_dir_absolute;
    NavFrontierExitRelative frontier_exit_relative;
    int8_t frontier_neighbor_cell_x;
    int8_t frontier_neighbor_cell_y;
    uint16_t frontier_count_found;
} NavRouteDebugSnapshot;

typedef struct NavMapCandidateDebug {
    int8_t right_cell_x;
    int8_t right_cell_y;
    int8_t front_cell_x;
    int8_t front_cell_y;
    int8_t left_cell_x;
    int8_t left_cell_y;
    bool right_cell_valid;
    bool front_cell_valid;
    bool left_cell_valid;
    bool right_cell_visited;
    bool front_cell_visited;
    bool left_cell_visited;
    bool used_unvisited_preference;
} NavMapCandidateDebug;

typedef struct NavAdvanceWallConfig {
    int16_t kp_pwm_per_mm;
    int16_t kd_pwm_per_mm_per_tick;
    int16_t correction_limit_pwm;
    int16_t error_deadband_mm;
    int16_t target_left_mm;
    int16_t target_right_mm;
} NavAdvanceWallConfig;

typedef struct NavDiagonalGuidanceConfig {
    int16_t kp_pwm_per_mm;
    int16_t kd_pwm_per_mm_per_tick;
    int16_t correction_limit_pwm;
    int16_t error_scale_num;
    int16_t error_scale_den;
    int16_t target_mm;
    NavSmoothFinalDiagonalMode smooth_final_mode;
} NavDiagonalGuidanceConfig;

typedef struct NavWallCautionConfig {
    bool enabled;
    uint16_t timeout_ms;
    int16_t delta_max_mm;
    int16_t kp_pwm_per_mm;
    int16_t kd_pwm_per_mm_per_tick;
    int16_t correction_limit_pwm;
} NavWallCautionConfig;

typedef struct NavSmoothYawCarryConfig {
    bool enabled;
    bool only_setpoint;
    bool require_diag;
    q16_16_t min_abs_deg_q16;
    q16_16_t max_abs_deg_q16;
    int16_t offset_scale_num;
    int16_t offset_scale_den;
} NavSmoothYawCarryConfig;

typedef struct NavTurnPidConfig {
    int32_t kp_q16;
    int32_t ki_q16;
    int32_t kd_q16;
    int32_t output_limit_pwm;
} NavTurnPidConfig;

typedef struct NavAdvanceYawPidConfig {
    int32_t kp_q16;
    int32_t ki_q16;
    int32_t kd_q16;
    int32_t output_limit_pwm;
} NavAdvanceYawPidConfig;

typedef struct NavSmoothTurnConfig {
    int32_t target_yaw_rate_deg_s;
    int16_t right_left_base_pwm;
    int16_t right_right_base_pwm;
    int16_t left_left_base_pwm;
    int16_t left_right_base_pwm;
} NavSmoothTurnConfig;

typedef struct NavWallPerception {
    bool wall_front;
    bool wall_left;
    bool wall_right;
    bool wall_diag_left;
    bool wall_diag_right;
    q16_16_t front_left_mm_q16;
    q16_16_t front_right_mm_q16;
    q16_16_t left_mm_q16;
    q16_16_t right_mm_q16;
    q16_16_t diag_left_mm_q16;
    q16_16_t diag_right_mm_q16;
    q16_16_t front_threshold_mm_q16;
    q16_16_t side_threshold_mm_q16;
    q16_16_t diag_threshold_mm_q16;
} NavWallPerception;

typedef enum NavSpecialMarkTargetSource {
    NAV_SPECIAL_MARK_TARGET_INVALID = 0,
    NAV_SPECIAL_MARK_TARGET_CURRENT_CELL,
    NAV_SPECIAL_MARK_TARGET_SMOOTH_DESTINATION,
    NAV_SPECIAL_MARK_TARGET_AUX_CURRENT_CELL
} NavSpecialMarkTargetSource;

typedef enum NavSpecialDetectionContext {
    NAV_SPECIAL_DETECT_DISABLED = 0,
    NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL,
    NAV_SPECIAL_DETECT_IN_CELL_AUX_TRANSLATION
} NavSpecialDetectionContext;

typedef struct NavTurnDebug {
    q16_16_t yaw_rate_setpoint_deg_s_q16;
    q16_16_t yaw_rate_measured_deg_s_q16;
    q16_16_t yaw_rate_error_deg_s_q16;
    q16_16_t pid_output_q16;
    int16_t correction_pwm;
    q16_16_t integral_q16;
    q16_16_t turn_kp_q16;
    q16_16_t turn_ki_q16;
    q16_16_t turn_kd_q16;
    int16_t turn_output_limit_pwm;
    int16_t smooth_left_base_pwm;
    int16_t smooth_right_base_pwm;
    NavSmoothPhase smooth_phase;
    NavSmoothDoneReason smooth_done_reason;
    uint16_t smooth_post_yaw_elapsed_ms;
    NavSmoothFinalGuidanceSource smooth_final_guidance_source;
    bool smooth_final_hold_initialized;
    NavSmoothFinalGuidanceSource smooth_final_hold_source;
    uint16_t smooth_final_hold_recapture_count;
    q16_16_t smooth_final_left_hold_mm_q16;
    q16_16_t smooth_final_right_hold_mm_q16;
    q16_16_t smooth_final_center_diff_hold_mm_q16;
    q16_16_t smooth_final_wall_error_mm_q16;
    int16_t smooth_final_wall_correction_pwm;
    int16_t smooth_final_yaw_correction_pwm;
    q16_16_t smooth_final_yaw_hold_deg_q16;
    bool smooth_final_yaw_hold_initialized;
    uint16_t smooth_final_yaw_hold_recapture_count;
    q16_16_t smooth_final_yaw_error_deg_q16;
    int16_t smooth_final_applied_correction_pwm;
    bool smooth_final_diag_left_valid;
    bool smooth_final_diag_right_valid;
    q16_16_t smooth_final_diag_left_mm_q16;
    q16_16_t smooth_final_diag_right_mm_q16;
    q16_16_t smooth_final_diag_target_mm_q16;
    q16_16_t smooth_final_diag_error_scale_q16;
    NavSmoothFinalDiagonalMode smooth_final_diag_mode;
    bool smooth_final_diag_hold_initialized;
    q16_16_t smooth_final_diag_left_hold_mm_q16;
    q16_16_t smooth_final_diag_right_hold_mm_q16;
    q16_16_t smooth_final_diag_center_diff_hold_mm_q16;
    uint16_t smooth_final_diag_hold_recapture_count;
    q16_16_t smooth_final_diag_raw_error_mm_q16;
    q16_16_t smooth_final_diag_error_mm_q16;
    bool smooth_final_follow_left_valid;
    bool smooth_final_follow_right_valid;
    bool smooth_yaw_carry_enabled;
    bool smooth_yaw_carry_pending;
    bool smooth_yaw_carry_used;
    q16_16_t smooth_yaw_carry_offset_deg_q16;
    q16_16_t smooth_yaw_carry_entry_yaw_deg_q16;
    q16_16_t smooth_yaw_carry_exit_yaw_deg_q16;
    bool smooth_yaw_carry_diag_used;
    NavSmoothYawCarryRejectedReason smooth_yaw_carry_rejected_reason;
    bool smooth_yaw_carry_only_setpoint;
    bool smooth_yaw_carry_require_diag;
    q16_16_t smooth_yaw_carry_min_abs_deg_q16;
    q16_16_t smooth_yaw_carry_max_abs_deg_q16;
    int16_t smooth_yaw_carry_offset_scale_num;
    int16_t smooth_yaw_carry_offset_scale_den;
    NavAdvancePhase advance_phase;
    NavAdvanceDoneReason advance_done_reason;
    bool rear_black_for_line;
    bool floor_rear_black;
    bool advance_started_on_rear_line;
    NavAdvanceStartMode advance_start_mode;
    bool advance_from_centered_waiting_rear_white;
    bool special_candidate;
    bool special_confirmed;
    bool special_ignore_rear_until_white;
    bool special_detection_started_on_rear_line;
    bool special_detection_enabled_for_current_motion;
    NavSpecialDetectionContext special_detection_context;
    bool special_aux_detection_enabled;
    bool special_aux_started_after_rear_line_left;
    bool initial_special_snapshot_pending;
    bool initial_special_snapshot_done;
    bool rear_line_trusted_for_decision;
    NavRearLineTrustSource rear_line_trust_source;
    int8_t special_mark_target_cell_x;
    int8_t special_mark_target_cell_y;
    NavSpecialMarkTargetSource special_mark_target_source;
    NavAction last_special_mark_action;
    uint16_t advance_elapsed_since_leave_start_line_ms;
    uint16_t special_detect_min_ms;
    uint16_t special_detect_max_ms;
    NavApproachFrontPhase approach_front_phase;
    NavApproachFrontDoneReason approach_front_done_reason;
    q16_16_t approach_front_target_mm_q16;
    q16_16_t approach_front_left_mm_q16;
    q16_16_t approach_front_right_mm_q16;
    uint16_t approach_front_elapsed_ms;
    uint16_t approach_front_brake_elapsed_ms;
    int16_t approach_front_base_left_pwm;
    int16_t approach_front_base_right_pwm;
    int16_t approach_front_correction_pwm;
    NavCenterPivotPhase center_pivot_phase;
    NavCenterPivotDoneReason center_pivot_done_reason;
    uint16_t center_pivot_elapsed_ms;
    uint16_t center_pivot_brake_elapsed_ms;
    int16_t center_pivot_base_left_pwm;
    int16_t center_pivot_base_right_pwm;
    int16_t center_pivot_correction_pwm;
    bool center_pivot_front_black;
    bool center_pivot_rear_black;
    bool center_pivot_front_seen_white;
    q16_16_t advance_yaw_setpoint_deg_q16;
    q16_16_t advance_yaw_measured_deg_q16;
    q16_16_t advance_yaw_error_deg_q16;
    q16_16_t advance_yaw_pid_output_q16;
    int16_t advance_yaw_correction_pwm;
    q16_16_t advance_yaw_hold_deg_q16;
    bool advance_yaw_hold_initialized;
    uint16_t advance_yaw_hold_recapture_count;
    q16_16_t advance_yaw_hold_error_deg_q16;
    NavAdvanceCorrectionSource advance_guidance_last_source;
    q16_16_t advance_yaw_kp_q16;
    q16_16_t advance_yaw_ki_q16;
    q16_16_t advance_yaw_kd_q16;
    int16_t advance_yaw_output_limit_pwm;
    int16_t advance_base_left_pwm;
    int16_t advance_base_right_pwm;
    NavAdvanceGuidanceMode advance_guidance_mode;
    NavAdvanceCorrectionSource advance_final_correction_source;
    bool advance_front_diag_preview_armed;
    bool advance_front_diag_preview_latched;
    bool advance_front_diag_preview_active;
    NavAdvanceFrontDiagSource advance_front_diag_source;
    q16_16_t advance_front_diag_raw_error_mm_q16;
    q16_16_t advance_front_diag_error_mm_q16;
    bool advance_front_diag_left_valid;
    bool advance_front_diag_right_valid;
    bool advance_wall_left_valid;
    bool advance_wall_right_valid;
    bool advance_diag_left_valid;
    bool advance_diag_right_valid;
    bool advance_follow_left_valid;
    bool advance_follow_right_valid;
    bool wall_caution_enabled;
    NavWallCautionConfidence wall_left_confidence;
    NavWallCautionConfidence wall_right_confidence;
    uint16_t wall_left_caution_elapsed_ms;
    uint16_t wall_right_caution_elapsed_ms;
    q16_16_t wall_left_caution_hold_mm_q16;
    q16_16_t wall_right_caution_hold_mm_q16;
    q16_16_t wall_left_caution_delta_mm_q16;
    q16_16_t wall_right_caution_delta_mm_q16;
    int16_t wall_caution_correction_pwm;
    NavWallCautionLossReason wall_caution_loss_reason;
    q16_16_t advance_wall_left_mm_q16;
    q16_16_t advance_wall_right_mm_q16;
    q16_16_t advance_wall_raw_error_mm_q16;
    q16_16_t advance_wall_error_mm_q16;
    q16_16_t advance_wall_error_after_deadband_mm_q16;
    q16_16_t advance_wall_prev_error_mm_q16;
    q16_16_t advance_wall_error_delta_mm_q16;
    int32_t advance_wall_p_term_pwm;
    int32_t advance_wall_d_term_pwm;
    int32_t advance_wall_raw_correction_pwm;
    int16_t advance_wall_limited_correction_pwm;
    int16_t advance_wall_correction_pwm;
    int16_t diag_guidance_kp_pwm_per_mm;
    int16_t diag_guidance_kd_pwm_per_mm_per_tick;
    int16_t diag_guidance_correction_limit_pwm;
    int16_t diag_guidance_error_scale_num;
    int16_t diag_guidance_error_scale_den;
    int16_t diag_guidance_target_mm;
    NavSmoothFinalDiagonalMode diag_guidance_smooth_final_mode;
    int32_t diag_guidance_p_term_pwm;
    int32_t diag_guidance_d_term_pwm;
    int16_t diag_guidance_correction_pwm;
    int16_t wall_kp_pwm_per_mm;
    int16_t wall_kd_pwm_per_mm_per_tick;
    q16_16_t wall_error_deadband_mm_q16;
    q16_16_t wall_follow_target_left_mm_q16;
    q16_16_t wall_follow_target_right_mm_q16;
    int16_t wall_correction_limit_pwm;
    int16_t wall_single_side_error_scale;
    NavAction last_completed_action;
    NavSmoothDoneReason last_smooth_done_reason;
    q16_16_t last_smooth_final_yaw_deg_q16;
    bool last_smooth_final_floor_rear_black;
    NavAdvanceDoneReason last_advance_done_reason;
    q16_16_t last_advance_final_yaw_deg_q16;
    bool last_advance_final_floor_rear_black;
    NavApproachFrontDoneReason last_approach_front_done_reason;
    NavCenterPivotDoneReason last_center_pivot_done_reason;
} NavTurnDebug;

void nav_core_init(void);
void nav_core_start_advance_until_rear_black(void);
void nav_core_start_advance_until_rear_black_from_centered_pose(void);
void nav_core_start_approach_front_wall_for_pivot(void);
void nav_core_start_center_in_cell_for_pivot_by_front_line(void);
void nav_core_start_smooth_turn_left(const RobotSensors *sensors);
void nav_core_start_smooth_turn_right(const RobotSensors *sensors);
void nav_core_start_pivot_turn_left(const RobotSensors *sensors);
void nav_core_start_pivot_turn_right(const RobotSensors *sensors);
void nav_core_start_pivot_turn_180(const RobotSensors *sensors);
void nav_core_stop(void);
void nav_core_set_smooth_target_yaw_rate_deg_s(int32_t target_yaw_rate_deg_s);
int32_t nav_core_get_smooth_target_yaw_rate_deg_s(void);
void nav_core_get_smooth_turn_config(NavSmoothTurnConfig *config);
void nav_core_get_wall_perception(NavWallPerception *perception);
void nav_core_set_advance_guidance_mode(NavAdvanceGuidanceMode mode);
NavAdvanceGuidanceMode nav_core_get_advance_guidance_mode(void);
void nav_core_set_advance_wall_config(const NavAdvanceWallConfig *config);
void nav_core_get_advance_wall_config(NavAdvanceWallConfig *config);
void nav_core_reset_advance_wall_defaults(void);
void nav_core_set_diagonal_guidance_config(const NavDiagonalGuidanceConfig *config);
void nav_core_get_diagonal_guidance_config(NavDiagonalGuidanceConfig *config);
void nav_core_reset_diagonal_guidance_defaults(void);
void nav_core_set_wall_caution_config(const NavWallCautionConfig *config);
void nav_core_get_wall_caution_config(NavWallCautionConfig *config);
void nav_core_reset_wall_caution_defaults(void);
void nav_core_set_smooth_yaw_carry_config(const NavSmoothYawCarryConfig *config);
void nav_core_get_smooth_yaw_carry_config(NavSmoothYawCarryConfig *config);
void nav_core_reset_smooth_yaw_carry_defaults(void);
bool nav_core_prepare_smooth_yaw_carry_for_next_action(bool next_action_is_smooth);
bool nav_core_has_smooth_yaw_carry_pending(void);
q16_16_t nav_core_consume_smooth_yaw_carry_offset_q16(void);
void nav_core_get_turn_pid_config(NavTurnPidConfig *config);
void nav_core_set_turn_pid_config(const NavTurnPidConfig *config);
void nav_core_reset_turn_pid_defaults(void);
void nav_core_get_advance_yaw_pid_config(NavAdvanceYawPidConfig *config);
void nav_core_set_advance_yaw_pid_config(const NavAdvanceYawPidConfig *config);
void nav_core_reset_advance_yaw_pid_defaults(void);
void nav_core_map_init(uint8_t width,
                       uint8_t height,
                       int8_t start_cell_x,
                       int8_t start_cell_y,
                       NavMapDirection start_dir);
bool nav_core_get_map_cell(int8_t cell_x, int8_t cell_y, NavMapCell *cell);
void nav_core_get_map_debug(NavMapDebugSnapshot *snapshot);
void nav_core_set_policy(NavPolicy policy);
NavPolicy nav_core_get_policy(void);
void nav_core_get_map_candidate_debug(NavMapCandidateDebug *debug);
NavRecommendedAction nav_core_recommend_basic_action(const RobotSensors *sensors);
bool nav_core_rear_line_trusted_for_decision(void);
NavRearLineTrustSource nav_core_rear_line_trust_source(void);
void nav_core_plan_clear(void);
bool nav_core_plan_push(NavPlanAction action);
uint8_t nav_core_plan_count(void);
bool nav_core_plan_is_empty(void);
NavPlanAction nav_core_plan_peek_next(void);
NavPlanAction nav_core_plan_pop_next(void);
void nav_core_plan_debug_snapshot(NavPlanDebugSnapshot *snapshot);
NavRouteStatus nav_core_route_plan_to_cell(int16_t target_cell_x, int16_t target_cell_y);
NavRouteStatus nav_core_route_plan_to_nearest_frontier(void);
void nav_core_get_route_debug(NavRouteDebugSnapshot *snapshot);
void nav_core_route_clear_debug(void);
NavState nav_core_state(void);
NavAction nav_core_action(void);
q16_16_t nav_core_action_start_yaw_q16(void);
q16_16_t nav_core_action_target_yaw_q16(void);
void nav_core_get_turn_debug(NavTurnDebug *debug);
RobotCommand nav_core_update(const RobotSensors *sensors);

#ifdef __cplusplus
}
#endif

#endif // NAV_CORE_H
