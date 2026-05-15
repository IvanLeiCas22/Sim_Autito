#ifndef NAV_SUPERVISOR_H
#define NAV_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#include "nav_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_SUPERVISOR_REQUIRED_SPECIAL_COUNT_DEFAULT 3u
#define NAV_SUPERVISOR_REQUIRED_SPECIAL_COUNT_MAX 16u

typedef enum NavSupervisorState {
    NAV_SUPERVISOR_STATE_IDLE = 0,
    NAV_SUPERVISOR_STATE_SEARCH_SPECIALS,
    NAV_SUPERVISOR_STATE_FOUND_REQUIRED_SPECIALS_WAIT_ACTION_DONE,
    NAV_SUPERVISOR_STATE_RETURN_SAFE_PLAN,
    NAV_SUPERVISOR_STATE_RETURN_SAFE_EXECUTE,
    NAV_SUPERVISOR_STATE_RETURN_SMART_DECIDE,
    NAV_SUPERVISOR_STATE_RETURN_FRONTIER_PLAN,
    NAV_SUPERVISOR_STATE_RETURN_FRONTIER_EXECUTE,
    NAV_SUPERVISOR_STATE_RETURN_FRONTIER_ENTER,
    NAV_SUPERVISOR_STATE_DONE,
    NAV_SUPERVISOR_STATE_ERROR,
    NAV_SUPERVISOR_STATE_CANCELLED
} NavSupervisorState;

typedef enum NavSupervisorDoneReason {
    NAV_SUPERVISOR_DONE_REASON_NONE = 0,
    NAV_SUPERVISOR_DONE_REASON_FOUND_REQUIRED_SPECIALS_AND_RETURNED,
    NAV_SUPERVISOR_DONE_REASON_NO_RETURN_ROUTE,
    NAV_SUPERVISOR_DONE_REASON_RETURN_ROUTE_TOO_LONG,
    NAV_SUPERVISOR_DONE_REASON_RETURN_QUEUE_OVERFLOW,
    NAV_SUPERVISOR_DONE_REASON_START_CELL_INVALID,
    NAV_SUPERVISOR_DONE_REASON_NO_FRONTIER_BEFORE_REQUIRED_SPECIALS,
    NAV_SUPERVISOR_DONE_REASON_CANCELLED
} NavSupervisorDoneReason;

typedef enum NavSupervisorReturnStrategy {
    NAV_SUPERVISOR_RETURN_STRATEGY_SAFE_KNOWN_RETURN = 0,
    NAV_SUPERVISOR_RETURN_STRATEGY_GOAL_DIRECTED_RETURN
} NavSupervisorReturnStrategy;

typedef enum NavSupervisorSmartState {
    NAV_SUPERVISOR_SMART_STATE_IDLE = 0,
    NAV_SUPERVISOR_SMART_STATE_LOCAL_UNVISITED,
    NAV_SUPERVISOR_SMART_STATE_PLAN_TO_FRONTIER,
    NAV_SUPERVISOR_SMART_STATE_EXECUTING_FRONTIER_ROUTE,
    NAV_SUPERVISOR_SMART_STATE_FRONTIER_ALREADY_HERE,
    NAV_SUPERVISOR_SMART_STATE_NO_FRONTIER,
    NAV_SUPERVISOR_SMART_STATE_ERROR,
    NAV_SUPERVISOR_SMART_STATE_BLOCKED_BY_MISSION,
    NAV_SUPERVISOR_SMART_STATE_WAIT_NAV_READY
} NavSupervisorSmartState;

typedef enum NavSupervisorSmartDecisionReason {
    NAV_SUPERVISOR_SMART_DECISION_REASON_NONE = 0,
    NAV_SUPERVISOR_SMART_DECISION_REASON_AUTONOMY_DISABLED,
    NAV_SUPERVISOR_SMART_DECISION_REASON_POLICY_NOT_SMART,
    NAV_SUPERVISOR_SMART_DECISION_REASON_BLOCKED_BY_MISSION,
    NAV_SUPERVISOR_SMART_DECISION_REASON_PLAN_EXECUTION_ACTIVE,
    NAV_SUPERVISOR_SMART_DECISION_REASON_NAV_NOT_READY,
    NAV_SUPERVISOR_SMART_DECISION_REASON_LOCAL_ACTION_AVAILABLE,
    NAV_SUPERVISOR_SMART_DECISION_REASON_PLAN_FRONTIER_REQUESTED,
    NAV_SUPERVISOR_SMART_DECISION_REASON_FRONTIER_ROUTE_FOUND,
    NAV_SUPERVISOR_SMART_DECISION_REASON_FRONTIER_ALREADY_HERE,
    NAV_SUPERVISOR_SMART_DECISION_REASON_NO_FRONTIER,
    NAV_SUPERVISOR_SMART_DECISION_REASON_FRONTIER_ERROR
} NavSupervisorSmartDecisionReason;

typedef enum NavSupervisorRequestedAction {
    NAV_SUPERVISOR_REQUESTED_ACTION_NONE = 0,
    NAV_SUPERVISOR_REQUESTED_ACTION_ACQUIRE_REAR_LINE,
    NAV_SUPERVISOR_REQUESTED_ACTION_ADVANCE_LINE,
    NAV_SUPERVISOR_REQUESTED_ACTION_SMOOTH_LEFT,
    NAV_SUPERVISOR_REQUESTED_ACTION_SMOOTH_RIGHT,
    NAV_SUPERVISOR_REQUESTED_ACTION_PIVOT_180,
    NAV_SUPERVISOR_REQUESTED_ACTION_RECOVERY_PIVOT_180_FRONT_BLOCKED
} NavSupervisorRequestedAction;

typedef struct NavSupervisorConfig {
    bool mission_enabled;
    uint8_t required_special_count;
    NavSupervisorReturnStrategy return_strategy;
} NavSupervisorConfig;

typedef struct NavSupervisorDebugSnapshot {
    NavSupervisorState state;
    NavSupervisorDoneReason done_reason;
    NavSupervisorConfig config;
    bool mission_enabled;
    uint8_t required_special_count;
    uint8_t found_special_count;
    int8_t start_cell_x;
    int8_t start_cell_y;
    int8_t start_dir;
    bool start_cell_valid;
    bool required_specials_reached;
    bool return_requested;
    bool waiting_action_done;
    bool return_to_start_active;
    bool at_start_cell;
    uint16_t safe_return_cost;
    uint16_t flood_best_score;
    NavSupervisorReturnStrategy return_strategy;
    NavSupervisorSmartState smart_state;
    NavSupervisorSmartDecisionReason smart_decision_reason;
    NavSupervisorRequestedAction smart_requested_action;
    bool smart_request_plan_to_frontier;
    bool smart_request_execute_frontier_plan;
    bool smart_blocked_by_mission;
    NavRecommendedAction smart_local_action;
    NavRouteStatus smart_frontier_status;
} NavSupervisorDebugSnapshot;

typedef struct NavSupervisorInput {
    bool mission_enabled;
    uint8_t found_special_count;
    bool nav_ready;
    bool at_start_cell;
    bool plan_execution_enabled;
    uint8_t plan_queue_count;
    bool smart_no_frontier;
    bool return_plan_loaded;
    int16_t return_route_status;
    int8_t current_cell_x;
    int8_t current_cell_y;
    int8_t current_dir;
    bool start_cell_valid;
    int8_t start_cell_x;
    int8_t start_cell_y;
    int8_t start_dir;
} NavSupervisorInput;

typedef struct NavSupervisorOutput {
    bool block_smart_actions;
    bool request_clear_exploration_plan;
    bool request_plan_return_to_start;
    bool request_execute_return_plan;
    bool request_stop_autonomy;
    bool request_stop_motors;
} NavSupervisorOutput;

typedef struct NavSupervisorSmartInput {
    bool autonomy_enabled;
    NavPolicy policy;
    bool nav_ready;
    bool mission_block_smart_actions;
    bool plan_execution_enabled;
    uint8_t plan_queue_count;
    bool decision_point_valid;
    bool floor_rear_black;
    bool rear_line_trusted_for_decision;
    bool wall_front;
    bool wall_left;
    bool wall_right;
    NavRecommendedAction recommended_action;
    bool candidate_right_valid;
    bool candidate_right_visited;
    bool candidate_front_valid;
    bool candidate_front_visited;
    bool candidate_left_valid;
    bool candidate_left_visited;
    NavRouteStatus frontier_route_status;
    bool frontier_plan_loaded;
} NavSupervisorSmartInput;

typedef struct NavSupervisorSmartOutput {
    bool block_new_actions;
    bool request_start_action;
    NavSupervisorRequestedAction requested_action;
    bool request_plan_to_frontier;
    bool request_execute_frontier_plan;
    bool request_stop_autonomy;
    NavSupervisorSmartState smart_state;
    NavSupervisorSmartDecisionReason decision_reason;
} NavSupervisorSmartOutput;

void nav_supervisor_init(void);
void nav_supervisor_reset(void);
void nav_supervisor_set_config(const NavSupervisorConfig *config);
NavSupervisorConfig nav_supervisor_get_config(void);
void nav_supervisor_get_debug(NavSupervisorDebugSnapshot *snapshot);
void nav_supervisor_cancel(void);
void nav_supervisor_set_start_cell(int8_t x, int8_t y, int8_t dir);
void nav_supervisor_update(const NavSupervisorInput *input, NavSupervisorOutput *output);
void nav_supervisor_notify_return_route_status(int16_t route_status, bool plan_loaded);
void nav_supervisor_update_smart_shadow(const NavSupervisorSmartInput *input,
                                        NavSupervisorSmartOutput *output);

#ifdef __cplusplus
}
#endif

#endif // NAV_SUPERVISOR_H
