#include "nav_supervisor.h"

#define NAV_SUPERVISOR_COST_INF 0xFFFFu
#define NAV_SUPERVISOR_ROUTE_STATUS_TARGET_OUT_OF_BOUNDS 5
#define NAV_SUPERVISOR_ROUTE_STATUS_TARGET_NOT_VISITED 6
#define NAV_SUPERVISOR_ROUTE_STATUS_ROUTE_TOO_LONG 7
#define NAV_SUPERVISOR_ROUTE_STATUS_QUEUE_OVERFLOW 8

typedef struct NavSupervisorStateData {
    NavSupervisorState state;
    NavSupervisorDoneReason done_reason;
    NavSupervisorConfig config;
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
    int16_t return_route_status;
    bool return_plan_loaded;
    bool clear_plan_requested;
    bool return_plan_requested;
    bool execute_return_requested;
    NavSupervisorSmartOutput smart_output;
    NavRecommendedAction smart_local_action;
    NavRouteStatus smart_frontier_status;
    bool smart_blocked_by_mission;
    bool smart_frontier_plan_requested;
    bool smart_frontier_plan_result_available;
    bool smart_frontier_plan_loaded;
    uint16_t smart_frontier_plan_request_pulse_count;
    bool final_safe_scan_return_attempted;
    bool final_safe_scan_return_active;
    bool final_safe_scan_return_plan_requested;
    bool final_safe_scan_return_execute_requested;
    bool final_safe_scan_return_completed;
    bool final_safe_scan_return_success;
    bool final_safe_scan_return_found_required_during_return;
    bool final_safe_scan_return_ready;
    NavSupervisorFinalSafeScanReturnWaitReason final_safe_scan_return_wait_reason;
    bool goal_directed_shadow_evaluated;
    bool goal_directed_shadow_valid;
    NavSupervisorGoalDirectedDecision goal_directed_shadow_decision;
    NavSupervisorGoalDirectedReason goal_directed_shadow_reason;
    NavRouteStatus goal_directed_safe_return_status;
    uint16_t goal_directed_safe_return_cost;
    NavGoalReturnEvalStatus goal_directed_optimistic_eval_status;
    uint16_t goal_directed_optimistic_any_cost;
    uint16_t goal_directed_optimistic_shortcut_cost;
    uint16_t goal_directed_optimistic_return_cost;
    bool goal_directed_unknown_used_path_found;
    uint8_t goal_directed_unknown_cells_on_path;
    uint8_t goal_directed_unknown_edges_on_path;
    NavFrontierEvalStatus goal_directed_frontier_eval_status;
    bool goal_directed_best_found;
    int8_t goal_directed_best_cell_x;
    int8_t goal_directed_best_cell_y;
    int8_t goal_directed_best_neighbor_x;
    int8_t goal_directed_best_neighbor_y;
    NavMapDirection goal_directed_best_exit_dir;
    uint8_t goal_directed_supported_arrival_dir_mask;
    NavRouteStatus goal_directed_frontier_route_status;
    uint16_t goal_directed_frontier_route_cost;
    int8_t goal_directed_frontier_found_arrival_dir;
    NavFrontierEntryAction goal_directed_entry_action;
    bool goal_directed_entry_supported;
    uint16_t goal_directed_estimated_after_entry_to_start;
    uint16_t goal_directed_attempt_total_score;
    int32_t goal_directed_score_improvement;
} NavSupervisorStateData;

static NavSupervisorStateData supervisor_state;

static void clear_goal_directed_shadow_debug(void)
{
    supervisor_state.goal_directed_shadow_evaluated = false;
    supervisor_state.goal_directed_shadow_valid = false;
    supervisor_state.goal_directed_shadow_decision =
        NAV_SUPERVISOR_GOAL_DIRECTED_DECISION_NONE;
    supervisor_state.goal_directed_shadow_reason =
        NAV_SUPERVISOR_GOAL_DIRECTED_REASON_NONE;
    supervisor_state.goal_directed_safe_return_status = NAV_ROUTE_STATUS_IDLE;
    supervisor_state.goal_directed_safe_return_cost = NAV_SUPERVISOR_COST_INF;
    supervisor_state.goal_directed_optimistic_eval_status =
        NAV_GOAL_RETURN_EVAL_STATUS_IDLE;
    supervisor_state.goal_directed_optimistic_any_cost = NAV_SUPERVISOR_COST_INF;
    supervisor_state.goal_directed_optimistic_shortcut_cost = NAV_SUPERVISOR_COST_INF;
    supervisor_state.goal_directed_optimistic_return_cost = NAV_SUPERVISOR_COST_INF;
    supervisor_state.goal_directed_unknown_used_path_found = false;
    supervisor_state.goal_directed_unknown_cells_on_path = 0u;
    supervisor_state.goal_directed_unknown_edges_on_path = 0u;
    supervisor_state.goal_directed_frontier_eval_status =
        NAV_FRONTIER_EVAL_STATUS_IDLE;
    supervisor_state.goal_directed_best_found = false;
    supervisor_state.goal_directed_best_cell_x = -1;
    supervisor_state.goal_directed_best_cell_y = -1;
    supervisor_state.goal_directed_best_neighbor_x = -1;
    supervisor_state.goal_directed_best_neighbor_y = -1;
    supervisor_state.goal_directed_best_exit_dir = NAV_DIR_NORTH;
    supervisor_state.goal_directed_supported_arrival_dir_mask = 0u;
    supervisor_state.goal_directed_frontier_route_status = NAV_ROUTE_STATUS_IDLE;
    supervisor_state.goal_directed_frontier_route_cost = NAV_SUPERVISOR_COST_INF;
    supervisor_state.goal_directed_frontier_found_arrival_dir = -1;
    supervisor_state.goal_directed_entry_action = NAV_FRONTIER_ENTRY_ACTION_NONE;
    supervisor_state.goal_directed_entry_supported = false;
    supervisor_state.goal_directed_estimated_after_entry_to_start = 0u;
    supervisor_state.goal_directed_attempt_total_score = NAV_SUPERVISOR_COST_INF;
    supervisor_state.goal_directed_score_improvement = 0;
}

static NavSupervisorConfig nav_supervisor_default_config(void)
{
    NavSupervisorConfig config;
    config.mission_enabled = false;
    config.required_special_count = NAV_SUPERVISOR_REQUIRED_SPECIAL_COUNT_DEFAULT;
    config.return_strategy = NAV_SUPERVISOR_RETURN_STRATEGY_SAFE_KNOWN_RETURN;
    config.goal_directed_score_margin = 0u;
    config.goal_directed_min_safe_return_cost_to_try = 4u;
    config.goal_directed_max_frontier_attempts = 1u;
    config.goal_directed_allow_back_entry = false;
    config.goal_directed_unknown_wall_penalty = 0u;
    config.goal_directed_unknown_cell_penalty = 0u;
    config.goal_directed_max_unknown_cells = 32u;
    config.goal_directed_max_unknown_edges = 32u;
    return config;
}

static NavSupervisorReturnStrategy sanitize_return_strategy(NavSupervisorReturnStrategy strategy)
{
    switch (strategy) {
    case NAV_SUPERVISOR_RETURN_STRATEGY_SAFE_KNOWN_RETURN:
    case NAV_SUPERVISOR_RETURN_STRATEGY_GOAL_DIRECTED_RETURN:
        return strategy;
    }

    return NAV_SUPERVISOR_RETURN_STRATEGY_SAFE_KNOWN_RETURN;
}

static NavSupervisorConfig sanitize_config(const NavSupervisorConfig *config)
{
    NavSupervisorConfig sanitized = nav_supervisor_default_config();
    if (config == 0) {
        return sanitized;
    }

    sanitized.mission_enabled = config->mission_enabled;
    sanitized.required_special_count = config->required_special_count;
    if (sanitized.required_special_count == 0u) {
        sanitized.required_special_count = 1u;
    } else if (sanitized.required_special_count > NAV_SUPERVISOR_REQUIRED_SPECIAL_COUNT_MAX) {
        sanitized.required_special_count = NAV_SUPERVISOR_REQUIRED_SPECIAL_COUNT_MAX;
    }
    sanitized.return_strategy = sanitize_return_strategy(config->return_strategy);
    sanitized.goal_directed_score_margin = config->goal_directed_score_margin;
    sanitized.goal_directed_min_safe_return_cost_to_try =
        config->goal_directed_min_safe_return_cost_to_try;
    sanitized.goal_directed_max_frontier_attempts =
        config->goal_directed_max_frontier_attempts;
    sanitized.goal_directed_allow_back_entry = config->goal_directed_allow_back_entry;
    sanitized.goal_directed_unknown_wall_penalty =
        config->goal_directed_unknown_wall_penalty;
    sanitized.goal_directed_unknown_cell_penalty =
        config->goal_directed_unknown_cell_penalty;
    sanitized.goal_directed_max_unknown_cells =
        config->goal_directed_max_unknown_cells;
    sanitized.goal_directed_max_unknown_edges =
        config->goal_directed_max_unknown_edges;
    return sanitized;
}

void nav_supervisor_reset(void)
{
    const NavSupervisorConfig config = supervisor_state.config;
    supervisor_state = (NavSupervisorStateData){0};
    supervisor_state.state = NAV_SUPERVISOR_STATE_IDLE;
    supervisor_state.done_reason = NAV_SUPERVISOR_DONE_REASON_NONE;
    supervisor_state.config = config;
    supervisor_state.start_cell_x = -1;
    supervisor_state.start_cell_y = -1;
    supervisor_state.start_dir = 0;
    supervisor_state.safe_return_cost = NAV_SUPERVISOR_COST_INF;
    supervisor_state.flood_best_score = NAV_SUPERVISOR_COST_INF;
    supervisor_state.return_route_status = 0;
    clear_goal_directed_shadow_debug();
}

void nav_supervisor_init(void)
{
    supervisor_state = (NavSupervisorStateData){0};
    supervisor_state.config = nav_supervisor_default_config();
    nav_supervisor_reset();
}

void nav_supervisor_set_config(const NavSupervisorConfig *config)
{
    supervisor_state.config = sanitize_config(config);
}

NavSupervisorConfig nav_supervisor_get_config(void)
{
    return supervisor_state.config;
}

void nav_supervisor_get_debug(NavSupervisorDebugSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    *snapshot = (NavSupervisorDebugSnapshot){0};
    snapshot->state = supervisor_state.state;
    snapshot->done_reason = supervisor_state.done_reason;
    snapshot->config = supervisor_state.config;
    snapshot->mission_enabled = supervisor_state.config.mission_enabled;
    snapshot->required_special_count = supervisor_state.config.required_special_count;
    snapshot->found_special_count = supervisor_state.found_special_count;
    snapshot->start_cell_x = supervisor_state.start_cell_x;
    snapshot->start_cell_y = supervisor_state.start_cell_y;
    snapshot->start_dir = supervisor_state.start_dir;
    snapshot->start_cell_valid = supervisor_state.start_cell_valid;
    snapshot->required_specials_reached = supervisor_state.required_specials_reached;
    snapshot->return_requested = supervisor_state.return_requested;
    snapshot->waiting_action_done = supervisor_state.waiting_action_done;
    snapshot->return_to_start_active = supervisor_state.return_to_start_active;
    snapshot->at_start_cell = supervisor_state.at_start_cell;
    snapshot->safe_return_cost = supervisor_state.safe_return_cost;
    snapshot->flood_best_score = supervisor_state.flood_best_score;
    snapshot->return_strategy = supervisor_state.config.return_strategy;
    snapshot->smart_state = supervisor_state.smart_output.smart_state;
    snapshot->smart_decision_reason = supervisor_state.smart_output.decision_reason;
    snapshot->smart_requested_action = supervisor_state.smart_output.requested_action;
    snapshot->smart_request_plan_to_frontier =
        supervisor_state.smart_output.request_plan_to_frontier;
    snapshot->smart_request_execute_frontier_plan =
        supervisor_state.smart_output.request_execute_frontier_plan;
    snapshot->smart_blocked_by_mission = supervisor_state.smart_blocked_by_mission;
    snapshot->smart_local_action = supervisor_state.smart_local_action;
    snapshot->smart_frontier_status = supervisor_state.smart_frontier_status;
    snapshot->smart_frontier_plan_notified =
        supervisor_state.smart_frontier_plan_result_available;
    snapshot->smart_frontier_plan_loaded = supervisor_state.smart_frontier_plan_loaded;
    snapshot->smart_frontier_plan_request_pulse_count =
        supervisor_state.smart_frontier_plan_request_pulse_count;
    snapshot->final_safe_scan_return_attempted =
        supervisor_state.final_safe_scan_return_attempted;
    snapshot->final_safe_scan_return_active =
        supervisor_state.final_safe_scan_return_active;
    snapshot->final_safe_scan_return_plan_requested =
        supervisor_state.final_safe_scan_return_plan_requested;
    snapshot->final_safe_scan_return_execute_requested =
        supervisor_state.final_safe_scan_return_execute_requested;
    snapshot->final_safe_scan_return_completed =
        supervisor_state.final_safe_scan_return_completed;
    snapshot->final_safe_scan_return_success =
        supervisor_state.final_safe_scan_return_success;
    snapshot->final_safe_scan_return_found_required_during_return =
        supervisor_state.final_safe_scan_return_found_required_during_return;
    snapshot->final_safe_scan_return_ready = supervisor_state.final_safe_scan_return_ready;
    snapshot->final_safe_scan_return_wait_reason =
        supervisor_state.final_safe_scan_return_wait_reason;
    snapshot->goal_directed_shadow_enabled =
        supervisor_state.config.return_strategy
        == NAV_SUPERVISOR_RETURN_STRATEGY_GOAL_DIRECTED_RETURN;
    snapshot->goal_directed_shadow_evaluated =
        supervisor_state.goal_directed_shadow_evaluated;
    snapshot->goal_directed_shadow_valid = supervisor_state.goal_directed_shadow_valid;
    snapshot->goal_directed_shadow_decision =
        supervisor_state.goal_directed_shadow_decision;
    snapshot->goal_directed_shadow_reason = supervisor_state.goal_directed_shadow_reason;
    snapshot->goal_directed_safe_return_status =
        supervisor_state.goal_directed_safe_return_status;
    snapshot->goal_directed_safe_return_cost =
        supervisor_state.goal_directed_safe_return_cost;
    snapshot->goal_directed_optimistic_eval_status =
        supervisor_state.goal_directed_optimistic_eval_status;
    snapshot->goal_directed_optimistic_any_cost =
        supervisor_state.goal_directed_optimistic_any_cost;
    snapshot->goal_directed_optimistic_shortcut_cost =
        supervisor_state.goal_directed_optimistic_shortcut_cost;
    snapshot->goal_directed_optimistic_return_cost =
        supervisor_state.goal_directed_optimistic_return_cost;
    snapshot->goal_directed_unknown_used_path_found =
        supervisor_state.goal_directed_unknown_used_path_found;
    snapshot->goal_directed_unknown_cells_on_path =
        supervisor_state.goal_directed_unknown_cells_on_path;
    snapshot->goal_directed_unknown_edges_on_path =
        supervisor_state.goal_directed_unknown_edges_on_path;
    snapshot->goal_directed_frontier_eval_status =
        supervisor_state.goal_directed_frontier_eval_status;
    snapshot->goal_directed_best_found = supervisor_state.goal_directed_best_found;
    snapshot->goal_directed_best_cell_x = supervisor_state.goal_directed_best_cell_x;
    snapshot->goal_directed_best_cell_y = supervisor_state.goal_directed_best_cell_y;
    snapshot->goal_directed_best_neighbor_x =
        supervisor_state.goal_directed_best_neighbor_x;
    snapshot->goal_directed_best_neighbor_y =
        supervisor_state.goal_directed_best_neighbor_y;
    snapshot->goal_directed_best_exit_dir = supervisor_state.goal_directed_best_exit_dir;
    snapshot->goal_directed_supported_arrival_dir_mask =
        supervisor_state.goal_directed_supported_arrival_dir_mask;
    snapshot->goal_directed_frontier_route_status =
        supervisor_state.goal_directed_frontier_route_status;
    snapshot->goal_directed_frontier_route_cost =
        supervisor_state.goal_directed_frontier_route_cost;
    snapshot->goal_directed_frontier_found_arrival_dir =
        supervisor_state.goal_directed_frontier_found_arrival_dir;
    snapshot->goal_directed_entry_action = supervisor_state.goal_directed_entry_action;
    snapshot->goal_directed_entry_supported = supervisor_state.goal_directed_entry_supported;
    snapshot->goal_directed_estimated_after_entry_to_start =
        supervisor_state.goal_directed_estimated_after_entry_to_start;
    snapshot->goal_directed_attempt_total_score =
        supervisor_state.goal_directed_attempt_total_score;
    snapshot->goal_directed_score_margin =
        supervisor_state.config.goal_directed_score_margin;
    snapshot->goal_directed_score_improvement =
        supervisor_state.goal_directed_score_improvement;
}

void nav_supervisor_cancel(void)
{
    supervisor_state.state = NAV_SUPERVISOR_STATE_CANCELLED;
    supervisor_state.done_reason = NAV_SUPERVISOR_DONE_REASON_CANCELLED;
    supervisor_state.return_requested = false;
    supervisor_state.waiting_action_done = false;
    supervisor_state.return_to_start_active = false;
    supervisor_state.final_safe_scan_return_active = false;
}

void nav_supervisor_set_start_cell(int8_t x, int8_t y, int8_t dir)
{
    supervisor_state.start_cell_x = x;
    supervisor_state.start_cell_y = y;
    supervisor_state.start_dir = dir;
    supervisor_state.start_cell_valid = true;
}

static void clear_output(NavSupervisorOutput *output)
{
    if (output != 0) {
        *output = (NavSupervisorOutput){0};
    }
}

static void set_inactive_state_from_input(const NavSupervisorInput *input)
{
    supervisor_state.state = NAV_SUPERVISOR_STATE_IDLE;
    supervisor_state.done_reason = NAV_SUPERVISOR_DONE_REASON_NONE;
    supervisor_state.found_special_count = input != 0 ? input->found_special_count : 0u;
    supervisor_state.required_specials_reached = false;
    supervisor_state.return_requested = false;
    supervisor_state.waiting_action_done = false;
    supervisor_state.return_to_start_active = false;
    supervisor_state.at_start_cell = input != 0 ? input->at_start_cell : false;
    supervisor_state.clear_plan_requested = false;
    supervisor_state.return_plan_requested = false;
    supervisor_state.execute_return_requested = false;
    supervisor_state.final_safe_scan_return_attempted = false;
    supervisor_state.final_safe_scan_return_active = false;
    supervisor_state.final_safe_scan_return_plan_requested = false;
    supervisor_state.final_safe_scan_return_execute_requested = false;
    supervisor_state.final_safe_scan_return_completed = false;
    supervisor_state.final_safe_scan_return_success = false;
    supervisor_state.final_safe_scan_return_found_required_during_return = false;
    supervisor_state.final_safe_scan_return_ready = false;
    supervisor_state.final_safe_scan_return_wait_reason = NAV_SUPERVISOR_FINAL_SAFE_SCAN_WAIT_NONE;
    clear_goal_directed_shadow_debug();
}

static void sync_input_snapshot(const NavSupervisorInput *input)
{
    supervisor_state.found_special_count = input->found_special_count;
    supervisor_state.at_start_cell = input->at_start_cell;
    supervisor_state.start_cell_x = input->start_cell_x;
    supervisor_state.start_cell_y = input->start_cell_y;
    supervisor_state.start_dir = input->start_dir;
    supervisor_state.start_cell_valid = input->start_cell_valid;
    supervisor_state.return_route_status = input->return_route_status;
    supervisor_state.return_plan_loaded = input->return_plan_loaded;
    supervisor_state.final_safe_scan_return_ready = input->final_safe_scan_return_ready;
    supervisor_state.final_safe_scan_return_wait_reason =
        input->final_safe_scan_return_wait_reason;
}

static NavFrontierEntryAction goal_directed_entry_action_for_dirs(
    NavMapDirection arrival_dir,
    NavMapDirection exit_dir)
{
    const uint8_t delta =
        (uint8_t)(((uint8_t)exit_dir - (uint8_t)arrival_dir) & 3u);
    switch (delta) {
    case 0u:
        return NAV_FRONTIER_ENTRY_ACTION_ADVANCE_LINE;
    case 1u:
        return NAV_FRONTIER_ENTRY_ACTION_SMOOTH_RIGHT;
    case 2u:
        return NAV_FRONTIER_ENTRY_ACTION_UNSUPPORTED_BACK_EXIT;
    case 3u:
        return NAV_FRONTIER_ENTRY_ACTION_SMOOTH_LEFT;
    }

    return NAV_FRONTIER_ENTRY_ACTION_NONE;
}

static bool goal_directed_entry_action_is_supported(NavFrontierEntryAction action,
                                                    bool allow_back_entry)
{
    return action == NAV_FRONTIER_ENTRY_ACTION_ADVANCE_LINE
        || action == NAV_FRONTIER_ENTRY_ACTION_SMOOTH_RIGHT
        || action == NAV_FRONTIER_ENTRY_ACTION_SMOOTH_LEFT
        || (allow_back_entry
            && action == NAV_FRONTIER_ENTRY_ACTION_UNSUPPORTED_BACK_EXIT);
}

static uint8_t goal_directed_supported_arrival_dir_mask(NavMapDirection exit_dir,
                                                        bool allow_back_entry)
{
    uint8_t mask = 0u;
    for (uint8_t dir_value = 0u; dir_value < 4u; ++dir_value) {
        const NavMapDirection arrival_dir = (NavMapDirection)dir_value;
        const NavFrontierEntryAction action =
            goal_directed_entry_action_for_dirs(arrival_dir, exit_dir);
        if (goal_directed_entry_action_is_supported(action, allow_back_entry)) {
            mask |= (uint8_t)(1u << dir_value);
        }
    }
    return mask;
}

static void goal_directed_set_fallback(NavSupervisorGoalDirectedReason reason)
{
    supervisor_state.goal_directed_shadow_decision =
        NAV_SUPERVISOR_GOAL_DIRECTED_DECISION_FALLBACK_SAFE;
    supervisor_state.goal_directed_shadow_reason = reason;
}

static NavSupervisorGoalDirectedReason goal_directed_reason_from_eval(
    NavGoalReturnReason reason)
{
    switch (reason) {
    case NAV_GOAL_RETURN_REASON_NO_PATH:
        return NAV_SUPERVISOR_GOAL_DIRECTED_REASON_NO_FRONTIER;
    case NAV_GOAL_RETURN_REASON_NO_SHORTCUT_FRONTIER:
        return NAV_SUPERVISOR_GOAL_DIRECTED_REASON_NO_SHORTCUT_FRONTIER;
    case NAV_GOAL_RETURN_REASON_UNKNOWN_BUDGET_EXCEEDED:
        return NAV_SUPERVISOR_GOAL_DIRECTED_REASON_UNKNOWN_BUDGET_EXCEEDED;
    case NAV_GOAL_RETURN_REASON_OPTIMISTIC_NOT_BETTER_THAN_SAFE:
        return NAV_SUPERVISOR_GOAL_DIRECTED_REASON_OPTIMISTIC_NOT_BETTER_THAN_SAFE_RETURN;
    case NAV_GOAL_RETURN_REASON_OPTIMISTIC_BETTER_THAN_SAFE:
        return NAV_SUPERVISOR_GOAL_DIRECTED_REASON_OPTIMISTIC_BETTER_THAN_SAFE_RETURN;
    case NAV_GOAL_RETURN_REASON_SAFE_RETURN_TOO_SHORT:
        return NAV_SUPERVISOR_GOAL_DIRECTED_REASON_SAFE_RETURN_TOO_SHORT;
    case NAV_GOAL_RETURN_REASON_ATTEMPT_BUDGET_EXHAUSTED:
        return NAV_SUPERVISOR_GOAL_DIRECTED_REASON_ATTEMPT_BUDGET_EXHAUSTED;
    case NAV_GOAL_RETURN_REASON_NONE:
        break;
    }

    return NAV_SUPERVISOR_GOAL_DIRECTED_REASON_NONE;
}

static void evaluate_goal_directed_return_shadow(const NavSupervisorInput *input)
{
    clear_goal_directed_shadow_debug();
    supervisor_state.goal_directed_shadow_evaluated = true;

    if (supervisor_state.config.return_strategy
        != NAV_SUPERVISOR_RETURN_STRATEGY_GOAL_DIRECTED_RETURN) {
        goal_directed_set_fallback(NAV_SUPERVISOR_GOAL_DIRECTED_REASON_DISABLED);
        return;
    }

    if (input == 0 || !input->start_cell_valid) {
        goal_directed_set_fallback(NAV_SUPERVISOR_GOAL_DIRECTED_REASON_NO_START_CELL);
        return;
    }

    if (supervisor_state.config.goal_directed_max_frontier_attempts == 0u) {
        goal_directed_set_fallback(
            NAV_SUPERVISOR_GOAL_DIRECTED_REASON_ATTEMPT_BUDGET_EXHAUSTED);
        return;
    }

    const NavRouteStatus safe_status =
        nav_core_route_eval_to_cell_with_dir_mask(input->start_cell_x,
                                                  input->start_cell_y,
                                                  0x0Fu);
    NavRouteEvalDebugSnapshot safe_debug = {0};
    nav_core_route_eval_get_debug(&safe_debug);
    supervisor_state.goal_directed_safe_return_status = safe_status;
    if (safe_status == NAV_ROUTE_STATUS_FOUND) {
        supervisor_state.goal_directed_safe_return_cost = safe_debug.route_length;
    }

    if (safe_status != NAV_ROUTE_STATUS_FOUND) {
        goal_directed_set_fallback(
            NAV_SUPERVISOR_GOAL_DIRECTED_REASON_SAFE_RETURN_NOT_FOUND);
        return;
    }

    NavGoalReturnEvalConfig eval_config = {0};
    eval_config.start_cell_x = input->start_cell_x;
    eval_config.start_cell_y = input->start_cell_y;
    eval_config.current_cell_x = input->current_cell_x;
    eval_config.current_cell_y = input->current_cell_y;
    eval_config.current_dir = (NavMapDirection)input->current_dir;
    eval_config.safe_return_cost = safe_debug.route_length;
    eval_config.score_margin = supervisor_state.config.goal_directed_score_margin;
    eval_config.min_safe_return_cost_to_try =
        supervisor_state.config.goal_directed_min_safe_return_cost_to_try;
    eval_config.unknown_wall_penalty =
        supervisor_state.config.goal_directed_unknown_wall_penalty;
    eval_config.unknown_cell_penalty =
        supervisor_state.config.goal_directed_unknown_cell_penalty;
    eval_config.max_unknown_cells =
        supervisor_state.config.goal_directed_max_unknown_cells;
    eval_config.max_unknown_edges =
        supervisor_state.config.goal_directed_max_unknown_edges;
    eval_config.max_shortcut_attempts =
        supervisor_state.config.goal_directed_max_frontier_attempts;
    eval_config.allow_back_entry = supervisor_state.config.goal_directed_allow_back_entry;

    NavGoalReturnEvalResult eval_result = {0};
    const NavGoalReturnEvalStatus eval_status =
        nav_goal_return_eval_evaluate(&eval_config, &eval_result);
    supervisor_state.goal_directed_optimistic_eval_status = eval_status;
    supervisor_state.goal_directed_optimistic_any_cost =
        eval_result.optimistic_any_cost;
    supervisor_state.goal_directed_optimistic_shortcut_cost =
        eval_result.optimistic_shortcut_cost;
    supervisor_state.goal_directed_optimistic_return_cost =
        eval_result.optimistic_return_cost;
    supervisor_state.goal_directed_unknown_used_path_found =
        eval_result.unknown_used_path_found;
    supervisor_state.goal_directed_unknown_cells_on_path =
        eval_result.unknown_cells_on_path;
    supervisor_state.goal_directed_unknown_edges_on_path =
        eval_result.unknown_edges_on_path;
    supervisor_state.goal_directed_best_found = eval_result.first_frontier_found;
    supervisor_state.goal_directed_best_cell_x = eval_result.frontier_cell_x;
    supervisor_state.goal_directed_best_cell_y = eval_result.frontier_cell_y;
    supervisor_state.goal_directed_best_neighbor_x = eval_result.frontier_neighbor_x;
    supervisor_state.goal_directed_best_neighbor_y = eval_result.frontier_neighbor_y;
    supervisor_state.goal_directed_best_exit_dir = eval_result.exit_dir;
    supervisor_state.goal_directed_estimated_after_entry_to_start =
        eval_result.optimistic_return_cost;
    supervisor_state.goal_directed_attempt_total_score =
        eval_result.optimistic_return_cost;
    supervisor_state.goal_directed_score_improvement = eval_result.score_improvement;
    supervisor_state.goal_directed_shadow_valid = eval_result.valid;

    if (eval_result.decision != NAV_GOAL_RETURN_DECISION_TRY_SHORTCUT
        || !eval_result.first_frontier_found) {
        goal_directed_set_fallback(goal_directed_reason_from_eval(eval_result.reason));
        return;
    }

    const uint8_t arrival_mask =
        goal_directed_supported_arrival_dir_mask(
            eval_result.exit_dir,
            supervisor_state.config.goal_directed_allow_back_entry);
    supervisor_state.goal_directed_supported_arrival_dir_mask = arrival_mask;
    if (arrival_mask == 0u) {
        goal_directed_set_fallback(
            NAV_SUPERVISOR_GOAL_DIRECTED_REASON_ENTRY_UNSUPPORTED);
        return;
    }

    const NavRouteStatus frontier_route_status =
        nav_core_route_eval_to_cell_with_dir_mask(eval_result.frontier_cell_x,
                                                  eval_result.frontier_cell_y,
                                                  arrival_mask);
    NavRouteEvalDebugSnapshot frontier_route_debug = {0};
    nav_core_route_eval_get_debug(&frontier_route_debug);
    supervisor_state.goal_directed_frontier_route_status = frontier_route_status;
    supervisor_state.goal_directed_frontier_found_arrival_dir =
        frontier_route_debug.found_target_dir;
    if (frontier_route_status == NAV_ROUTE_STATUS_FOUND) {
        supervisor_state.goal_directed_frontier_route_cost =
            frontier_route_debug.route_length;
    }

    if (frontier_route_status != NAV_ROUTE_STATUS_FOUND
        || frontier_route_debug.found_target_dir < 0
        || frontier_route_debug.found_target_dir > 3) {
        goal_directed_set_fallback(
            NAV_SUPERVISOR_GOAL_DIRECTED_REASON_FRONTIER_ROUTE_NOT_FOUND);
        return;
    }

    const NavMapDirection found_arrival_dir =
        (NavMapDirection)frontier_route_debug.found_target_dir;
    const NavFrontierEntryAction entry_action =
        goal_directed_entry_action_for_dirs(found_arrival_dir,
                                            eval_result.exit_dir);
    supervisor_state.goal_directed_entry_action = entry_action;
    supervisor_state.goal_directed_entry_supported =
        goal_directed_entry_action_is_supported(
            entry_action,
            supervisor_state.config.goal_directed_allow_back_entry);
    if (!supervisor_state.goal_directed_entry_supported) {
        goal_directed_set_fallback(
            NAV_SUPERVISOR_GOAL_DIRECTED_REASON_ENTRY_UNSUPPORTED);
        return;
    }

    supervisor_state.goal_directed_shadow_valid = true;
    supervisor_state.goal_directed_shadow_decision =
        NAV_SUPERVISOR_GOAL_DIRECTED_DECISION_TRY_FRONTIER;
    supervisor_state.goal_directed_shadow_reason =
        NAV_SUPERVISOR_GOAL_DIRECTED_REASON_OPTIMISTIC_BETTER_THAN_SAFE_RETURN;
}

static NavSupervisorDoneReason done_reason_from_return_route_status(int16_t route_status)
{
    switch (route_status) {
    case NAV_SUPERVISOR_ROUTE_STATUS_TARGET_OUT_OF_BOUNDS:
    case NAV_SUPERVISOR_ROUTE_STATUS_TARGET_NOT_VISITED:
        return NAV_SUPERVISOR_DONE_REASON_START_CELL_INVALID;
    case NAV_SUPERVISOR_ROUTE_STATUS_ROUTE_TOO_LONG:
        return NAV_SUPERVISOR_DONE_REASON_RETURN_ROUTE_TOO_LONG;
    case NAV_SUPERVISOR_ROUTE_STATUS_QUEUE_OVERFLOW:
        return NAV_SUPERVISOR_DONE_REASON_RETURN_QUEUE_OVERFLOW;
    default:
        return NAV_SUPERVISOR_DONE_REASON_NO_RETURN_ROUTE;
    }
}

static void start_required_specials_return_wait(const NavSupervisorInput *input,
                                                NavSupervisorOutput *output)
{
    supervisor_state.required_specials_reached = true;
    supervisor_state.return_requested = true;
    supervisor_state.waiting_action_done = !input->nav_ready;
    supervisor_state.clear_plan_requested = true;
    supervisor_state.return_plan_requested = false;
    supervisor_state.execute_return_requested = false;
    supervisor_state.final_safe_scan_return_active = false;
    supervisor_state.state = NAV_SUPERVISOR_STATE_FOUND_REQUIRED_SPECIALS_WAIT_ACTION_DONE;
    if (output != 0) {
        output->block_smart_actions = true;
        output->request_clear_exploration_plan = true;
    }
}

static void enter_no_frontier_before_required_error(NavSupervisorOutput *output)
{
    supervisor_state.state = NAV_SUPERVISOR_STATE_ERROR;
    supervisor_state.done_reason =
        NAV_SUPERVISOR_DONE_REASON_NO_FRONTIER_BEFORE_REQUIRED_SPECIALS;
    supervisor_state.final_safe_scan_return_active = false;
    if (output != 0) {
        output->request_stop_autonomy = true;
        output->request_stop_motors = true;
    }
}

void nav_supervisor_update(const NavSupervisorInput *input, NavSupervisorOutput *output)
{
    clear_output(output);
    if (input == 0) {
        return;
    }

    sync_input_snapshot(input);

    if (!input->mission_enabled && supervisor_state.state != NAV_SUPERVISOR_STATE_CANCELLED) {
        set_inactive_state_from_input(input);
        return;
    }

    if (supervisor_state.state == NAV_SUPERVISOR_STATE_CANCELLED) {
        return;
    }

    if (supervisor_state.state == NAV_SUPERVISOR_STATE_IDLE) {
        supervisor_state.state = NAV_SUPERVISOR_STATE_SEARCH_SPECIALS;
        supervisor_state.done_reason = NAV_SUPERVISOR_DONE_REASON_NONE;
    }

    if (supervisor_state.state == NAV_SUPERVISOR_STATE_SEARCH_SPECIALS) {
        if (input->smart_no_frontier
            && input->found_special_count < supervisor_state.config.required_special_count) {
            if (supervisor_state.final_safe_scan_return_attempted) {
                enter_no_frontier_before_required_error(output);
                return;
            }

            supervisor_state.final_safe_scan_return_attempted = true;
            supervisor_state.final_safe_scan_return_active = true;
            supervisor_state.final_safe_scan_return_plan_requested = false;
            supervisor_state.final_safe_scan_return_execute_requested = false;
            supervisor_state.final_safe_scan_return_completed = false;
            supervisor_state.final_safe_scan_return_success = false;
            supervisor_state.final_safe_scan_return_found_required_during_return = false;
            supervisor_state.final_safe_scan_return_wait_reason =
                input->final_safe_scan_return_wait_reason;
            supervisor_state.waiting_action_done = !input->final_safe_scan_return_ready;
            supervisor_state.clear_plan_requested = true;
            supervisor_state.return_to_start_active = true;
            supervisor_state.return_plan_requested = false;
            supervisor_state.execute_return_requested = false;
            supervisor_state.state = NAV_SUPERVISOR_STATE_FINAL_SAFE_SCAN_RETURN_PLAN;
            if (output != 0) {
                output->block_smart_actions = true;
                output->request_clear_exploration_plan = true;
            }
            return;
        }

        if (input->found_special_count < supervisor_state.config.required_special_count) {
            return;
        }

        start_required_specials_return_wait(input, output);
        return;
    }

    if (supervisor_state.state == NAV_SUPERVISOR_STATE_FINAL_SAFE_SCAN_RETURN_PLAN) {
        if (input->found_special_count >= supervisor_state.config.required_special_count) {
            supervisor_state.final_safe_scan_return_success = true;
            supervisor_state.final_safe_scan_return_found_required_during_return = true;
        }

        supervisor_state.waiting_action_done = !input->final_safe_scan_return_ready;
        supervisor_state.final_safe_scan_return_active = true;
        supervisor_state.return_to_start_active = true;
        if (output != 0) {
            output->block_smart_actions = true;
        }

        if (input->at_start_cell) {
            supervisor_state.final_safe_scan_return_active = false;
            supervisor_state.final_safe_scan_return_completed = true;
            supervisor_state.return_to_start_active = false;
            if (input->found_special_count >= supervisor_state.config.required_special_count) {
                supervisor_state.final_safe_scan_return_success = true;
                supervisor_state.state = NAV_SUPERVISOR_STATE_DONE;
                supervisor_state.done_reason =
                    NAV_SUPERVISOR_DONE_REASON_FOUND_REQUIRED_SPECIALS_AND_RETURNED;
            } else {
                supervisor_state.final_safe_scan_return_success = false;
                supervisor_state.state = NAV_SUPERVISOR_STATE_ERROR;
                supervisor_state.done_reason =
                    NAV_SUPERVISOR_DONE_REASON_NO_FRONTIER_BEFORE_REQUIRED_SPECIALS;
            }
            if (output != 0) {
                output->request_stop_autonomy = true;
                output->request_stop_motors = true;
            }
            return;
        }

        if (!input->final_safe_scan_return_ready) {
            return;
        }

        supervisor_state.waiting_action_done = false;
        if (!input->start_cell_valid) {
            supervisor_state.final_safe_scan_return_active = false;
            supervisor_state.return_to_start_active = false;
            supervisor_state.state = NAV_SUPERVISOR_STATE_ERROR;
            supervisor_state.done_reason = NAV_SUPERVISOR_DONE_REASON_START_CELL_INVALID;
            if (output != 0) {
                output->request_stop_autonomy = true;
                output->request_stop_motors = true;
            }
            return;
        }

        if (!supervisor_state.final_safe_scan_return_plan_requested) {
            supervisor_state.final_safe_scan_return_plan_requested = true;
            supervisor_state.return_plan_requested = true;
            supervisor_state.final_safe_scan_return_wait_reason =
                NAV_SUPERVISOR_FINAL_SAFE_SCAN_WAIT_PLAN_REQUESTED;
            if (output != 0) {
                output->request_plan_return_to_start = true;
            }
            return;
        }

        if (input->return_plan_loaded) {
            supervisor_state.final_safe_scan_return_wait_reason =
                NAV_SUPERVISOR_FINAL_SAFE_SCAN_WAIT_PLAN_LOADED;
            supervisor_state.state = NAV_SUPERVISOR_STATE_FINAL_SAFE_SCAN_RETURN_EXECUTE;
            if (!supervisor_state.final_safe_scan_return_execute_requested) {
                supervisor_state.final_safe_scan_return_execute_requested = true;
                supervisor_state.execute_return_requested = true;
                if (output != 0) {
                    output->request_execute_return_plan = true;
                }
            }
        } else if (input->return_route_status != 0) {
            supervisor_state.final_safe_scan_return_wait_reason =
                NAV_SUPERVISOR_FINAL_SAFE_SCAN_WAIT_PLAN_FAILED;
            supervisor_state.final_safe_scan_return_active = false;
            supervisor_state.return_to_start_active = false;
            supervisor_state.state = NAV_SUPERVISOR_STATE_ERROR;
            supervisor_state.done_reason =
                done_reason_from_return_route_status(input->return_route_status);
            if (output != 0) {
                output->request_stop_autonomy = true;
                output->request_stop_motors = true;
            }
        } else {
            supervisor_state.final_safe_scan_return_wait_reason =
                NAV_SUPERVISOR_FINAL_SAFE_SCAN_WAIT_PLAN_REQUESTED;
        }
        return;
    }

    if (supervisor_state.state == NAV_SUPERVISOR_STATE_FINAL_SAFE_SCAN_RETURN_EXECUTE) {
        supervisor_state.final_safe_scan_return_active = true;
        supervisor_state.return_to_start_active = true;
        if (input->found_special_count >= supervisor_state.config.required_special_count) {
            supervisor_state.final_safe_scan_return_success = true;
            supervisor_state.final_safe_scan_return_found_required_during_return = true;
        }
        if (output != 0) {
            output->block_smart_actions = true;
        }
        if (!input->plan_execution_enabled) {
            supervisor_state.final_safe_scan_return_active = false;
            supervisor_state.final_safe_scan_return_completed = true;
            supervisor_state.return_to_start_active = false;
            if (input->at_start_cell
                && input->found_special_count >= supervisor_state.config.required_special_count) {
                supervisor_state.final_safe_scan_return_success = true;
                supervisor_state.state = NAV_SUPERVISOR_STATE_DONE;
                supervisor_state.done_reason =
                    NAV_SUPERVISOR_DONE_REASON_FOUND_REQUIRED_SPECIALS_AND_RETURNED;
            } else if (input->at_start_cell) {
                supervisor_state.final_safe_scan_return_success = false;
                supervisor_state.state = NAV_SUPERVISOR_STATE_ERROR;
                supervisor_state.done_reason =
                    NAV_SUPERVISOR_DONE_REASON_NO_FRONTIER_BEFORE_REQUIRED_SPECIALS;
            } else {
                supervisor_state.final_safe_scan_return_success = false;
                supervisor_state.state = NAV_SUPERVISOR_STATE_ERROR;
                supervisor_state.done_reason = NAV_SUPERVISOR_DONE_REASON_NO_RETURN_ROUTE;
            }
            if (output != 0) {
                output->request_stop_autonomy = true;
                output->request_stop_motors = true;
            }
        }
        return;
    }

    if (supervisor_state.state
        == NAV_SUPERVISOR_STATE_FOUND_REQUIRED_SPECIALS_WAIT_ACTION_DONE) {
        supervisor_state.waiting_action_done = !input->nav_ready;
        if (output != 0) {
            output->block_smart_actions = true;
        }
        if (!input->nav_ready) {
            return;
        }

        supervisor_state.waiting_action_done = false;
        if (!supervisor_state.return_plan_requested
            && !supervisor_state.goal_directed_shadow_evaluated
            && supervisor_state.config.return_strategy
                == NAV_SUPERVISOR_RETURN_STRATEGY_GOAL_DIRECTED_RETURN) {
            evaluate_goal_directed_return_shadow(input);
        }
        supervisor_state.state = NAV_SUPERVISOR_STATE_RETURN_SAFE_PLAN;
        supervisor_state.return_to_start_active = true;
        if (!supervisor_state.return_plan_requested) {
            supervisor_state.return_plan_requested = true;
            if (output != 0) {
                output->request_plan_return_to_start = true;
            }
        }
        return;
    }

    if (supervisor_state.state == NAV_SUPERVISOR_STATE_RETURN_SAFE_PLAN) {
        supervisor_state.return_to_start_active = true;
        if (output != 0) {
            output->block_smart_actions = true;
        }

        if (!input->start_cell_valid) {
            supervisor_state.state = NAV_SUPERVISOR_STATE_ERROR;
            supervisor_state.done_reason = NAV_SUPERVISOR_DONE_REASON_START_CELL_INVALID;
            if (output != 0) {
                output->request_stop_autonomy = true;
                output->request_stop_motors = true;
            }
            return;
        }

        if (input->at_start_cell) {
            supervisor_state.state = NAV_SUPERVISOR_STATE_DONE;
            supervisor_state.done_reason =
                NAV_SUPERVISOR_DONE_REASON_FOUND_REQUIRED_SPECIALS_AND_RETURNED;
            supervisor_state.return_to_start_active = false;
            if (output != 0) {
                output->request_stop_autonomy = true;
                output->request_stop_motors = true;
            }
            return;
        }

        if (input->return_plan_loaded) {
            supervisor_state.state = NAV_SUPERVISOR_STATE_RETURN_SAFE_EXECUTE;
            if (!supervisor_state.execute_return_requested) {
                supervisor_state.execute_return_requested = true;
                if (output != 0) {
                    output->request_execute_return_plan = true;
                }
            }
        } else if (input->return_route_status != 0) {
            supervisor_state.state = NAV_SUPERVISOR_STATE_ERROR;
            supervisor_state.done_reason =
                done_reason_from_return_route_status(input->return_route_status);
            supervisor_state.return_to_start_active = false;
            if (output != 0) {
                output->request_stop_autonomy = true;
                output->request_stop_motors = true;
            }
        }
        return;
    }

    if (supervisor_state.state == NAV_SUPERVISOR_STATE_RETURN_SAFE_EXECUTE) {
        supervisor_state.return_to_start_active = true;
        if (output != 0) {
            output->block_smart_actions = true;
        }
        if (!input->plan_execution_enabled) {
            supervisor_state.return_to_start_active = false;
            if (input->at_start_cell) {
                supervisor_state.state = NAV_SUPERVISOR_STATE_DONE;
                supervisor_state.done_reason =
                    NAV_SUPERVISOR_DONE_REASON_FOUND_REQUIRED_SPECIALS_AND_RETURNED;
            } else {
                supervisor_state.state = NAV_SUPERVISOR_STATE_ERROR;
                supervisor_state.done_reason = NAV_SUPERVISOR_DONE_REASON_NO_RETURN_ROUTE;
            }
            if (output != 0) {
                output->request_stop_autonomy = true;
                output->request_stop_motors = true;
            }
        }
    }
}

void nav_supervisor_notify_return_route_status(int16_t route_status, bool plan_loaded)
{
    supervisor_state.return_route_status = route_status;
    supervisor_state.return_plan_loaded = plan_loaded;
}

void nav_supervisor_notify_frontier_route_status(NavRouteStatus status, bool plan_loaded)
{
    supervisor_state.smart_frontier_plan_requested = true;
    supervisor_state.smart_frontier_plan_result_available = true;
    supervisor_state.smart_frontier_status = status;
    supervisor_state.smart_frontier_plan_loaded = plan_loaded;
}

static void clear_smart_output(NavSupervisorSmartOutput *output)
{
    if (output != 0) {
        *output = (NavSupervisorSmartOutput){0};
    }
}

static NavSupervisorRequestedAction requested_action_from_recommended(
    NavRecommendedAction action)
{
    switch (action) {
    case NAV_RECOMMENDED_ACQUIRE_REAR_LINE:
        return NAV_SUPERVISOR_REQUESTED_ACTION_ACQUIRE_REAR_LINE;
    case NAV_RECOMMENDED_ADVANCE_LINE:
        return NAV_SUPERVISOR_REQUESTED_ACTION_ADVANCE_LINE;
    case NAV_RECOMMENDED_SMOOTH_LEFT:
        return NAV_SUPERVISOR_REQUESTED_ACTION_SMOOTH_LEFT;
    case NAV_RECOMMENDED_SMOOTH_RIGHT:
        return NAV_SUPERVISOR_REQUESTED_ACTION_SMOOTH_RIGHT;
    case NAV_RECOMMENDED_PIVOT_180:
        return NAV_SUPERVISOR_REQUESTED_ACTION_PIVOT_180;
    case NAV_RECOMMENDED_RECOVERY_PIVOT_180_FRONT_BLOCKED:
        return NAV_SUPERVISOR_REQUESTED_ACTION_RECOVERY_PIVOT_180_FRONT_BLOCKED;
    case NAV_RECOMMENDED_NONE:
    default:
        return NAV_SUPERVISOR_REQUESTED_ACTION_NONE;
    }
}

static bool recommended_action_uses_unvisited_candidate(
    const NavSupervisorSmartInput *input)
{
    if (input == 0) {
        return false;
    }

    switch (input->recommended_action) {
    case NAV_RECOMMENDED_SMOOTH_RIGHT:
        return input->candidate_right_valid && !input->candidate_right_visited;
    case NAV_RECOMMENDED_ADVANCE_LINE:
        return input->candidate_front_valid && !input->candidate_front_visited;
    case NAV_RECOMMENDED_SMOOTH_LEFT:
        return input->candidate_left_valid && !input->candidate_left_visited;
    default:
        return false;
    }
}

static bool recommended_action_is_local_for_smart(
    const NavSupervisorSmartInput *input)
{
    if (input == 0 || input->recommended_action == NAV_RECOMMENDED_NONE) {
        return false;
    }

    if (!input->decision_point_valid) {
        return true;
    }

    if (input->recommended_action == NAV_RECOMMENDED_ACQUIRE_REAR_LINE
        || input->recommended_action
            == NAV_RECOMMENDED_RECOVERY_PIVOT_180_FRONT_BLOCKED) {
        return true;
    }

    return recommended_action_uses_unvisited_candidate(input);
}

static void clear_smart_frontier_plan_state(void)
{
    supervisor_state.smart_frontier_plan_requested = false;
    supervisor_state.smart_frontier_plan_result_available = false;
    supervisor_state.smart_frontier_plan_loaded = false;
    supervisor_state.smart_frontier_status = NAV_ROUTE_STATUS_IDLE;
}

static void store_smart_output(const NavSupervisorSmartInput *input,
                               const NavSupervisorSmartOutput *output)
{
    supervisor_state.smart_output = output != 0 ? *output : (NavSupervisorSmartOutput){0};
    supervisor_state.smart_local_action =
        input != 0 ? input->recommended_action : NAV_RECOMMENDED_NONE;
    supervisor_state.smart_blocked_by_mission =
        input != 0 ? input->mission_block_smart_actions : false;
}

static void set_frontier_result_from_input_if_present(const NavSupervisorSmartInput *input)
{
    if (input == 0 || input->frontier_route_status == NAV_ROUTE_STATUS_IDLE) {
        return;
    }

    supervisor_state.smart_frontier_plan_requested = true;
    supervisor_state.smart_frontier_plan_result_available = true;
    supervisor_state.smart_frontier_status = input->frontier_route_status;
    supervisor_state.smart_frontier_plan_loaded = input->frontier_plan_loaded;
}

void nav_supervisor_update_smart(const NavSupervisorSmartInput *input,
                                 NavSupervisorSmartOutput *output)
{
    clear_smart_output(output);
    if (input == 0) {
        clear_smart_frontier_plan_state();
        store_smart_output(0, output);
        return;
    }

    set_frontier_result_from_input_if_present(input);

    NavSupervisorSmartOutput local_output = {0};
    if (!input->autonomy_enabled) {
        clear_smart_frontier_plan_state();
        local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_IDLE;
        local_output.decision_reason =
            NAV_SUPERVISOR_SMART_DECISION_REASON_AUTONOMY_DISABLED;
    } else if (input->policy != NAV_POLICY_SMART_RECOGNITION) {
        clear_smart_frontier_plan_state();
        local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_IDLE;
        local_output.decision_reason =
            NAV_SUPERVISOR_SMART_DECISION_REASON_POLICY_NOT_SMART;
    } else if (input->mission_block_smart_actions) {
        clear_smart_frontier_plan_state();
        local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_BLOCKED_BY_MISSION;
        local_output.decision_reason =
            NAV_SUPERVISOR_SMART_DECISION_REASON_BLOCKED_BY_MISSION;
        local_output.block_new_actions = true;
    } else if (input->plan_execution_enabled) {
        supervisor_state.smart_frontier_plan_requested = false;
        supervisor_state.smart_frontier_plan_result_available = false;
        local_output.smart_state =
            NAV_SUPERVISOR_SMART_STATE_EXECUTING_FRONTIER_ROUTE;
        local_output.decision_reason =
            NAV_SUPERVISOR_SMART_DECISION_REASON_PLAN_EXECUTION_ACTIVE;
        local_output.request_execute_frontier_plan = true;
    } else if (!input->nav_ready) {
        local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_WAIT_NAV_READY;
        local_output.decision_reason =
            NAV_SUPERVISOR_SMART_DECISION_REASON_NAV_NOT_READY;
    } else if (recommended_action_is_local_for_smart(input)) {
        clear_smart_frontier_plan_state();
        local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_LOCAL_UNVISITED;
        local_output.decision_reason =
            NAV_SUPERVISOR_SMART_DECISION_REASON_LOCAL_ACTION_AVAILABLE;
        local_output.request_start_action = true;
        local_output.requested_action =
            requested_action_from_recommended(input->recommended_action);
    } else {
        local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_PLAN_TO_FRONTIER;
        local_output.decision_reason =
            NAV_SUPERVISOR_SMART_DECISION_REASON_PLAN_FRONTIER_REQUESTED;

        if (!supervisor_state.smart_frontier_plan_requested) {
            supervisor_state.smart_frontier_plan_requested = true;
            ++supervisor_state.smart_frontier_plan_request_pulse_count;
            local_output.request_plan_to_frontier = true;
        }

        if (!supervisor_state.smart_frontier_plan_result_available) {
            if (output != 0) {
                *output = local_output;
            }
            store_smart_output(input, &local_output);
            return;
        }

        switch (supervisor_state.smart_frontier_status) {
        case NAV_ROUTE_STATUS_FOUND:
            if (supervisor_state.smart_frontier_plan_loaded) {
                local_output.smart_state =
                    NAV_SUPERVISOR_SMART_STATE_EXECUTING_FRONTIER_ROUTE;
                local_output.decision_reason =
                    NAV_SUPERVISOR_SMART_DECISION_REASON_FRONTIER_ROUTE_FOUND;
                local_output.request_execute_frontier_plan = true;
            } else {
                local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_ERROR;
                local_output.decision_reason =
                    NAV_SUPERVISOR_SMART_DECISION_REASON_FRONTIER_ERROR;
                local_output.request_stop_autonomy = true;
            }
            break;
        case NAV_ROUTE_STATUS_FRONTIER_ALREADY_HERE:
            local_output.smart_state =
                NAV_SUPERVISOR_SMART_STATE_FRONTIER_ALREADY_HERE;
            local_output.decision_reason =
                NAV_SUPERVISOR_SMART_DECISION_REASON_FRONTIER_ALREADY_HERE;
            break;
        case NAV_ROUTE_STATUS_NO_FRONTIER:
            local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_NO_FRONTIER;
            local_output.decision_reason =
                NAV_SUPERVISOR_SMART_DECISION_REASON_NO_FRONTIER;
            local_output.request_stop_autonomy = true;
            break;
        case NAV_ROUTE_STATUS_IDLE:
            break;
        default:
            local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_ERROR;
            local_output.decision_reason =
                NAV_SUPERVISOR_SMART_DECISION_REASON_FRONTIER_ERROR;
            local_output.request_stop_autonomy = true;
            break;
        }
    }

    if (output != 0) {
        *output = local_output;
    }
    store_smart_output(input, &local_output);
}
