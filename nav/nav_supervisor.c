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
} NavSupervisorStateData;

static NavSupervisorStateData supervisor_state;

static NavSupervisorConfig nav_supervisor_default_config(void)
{
    NavSupervisorConfig config;
    config.mission_enabled = false;
    config.required_special_count = NAV_SUPERVISOR_REQUIRED_SPECIAL_COUNT_DEFAULT;
    config.return_strategy = NAV_SUPERVISOR_RETURN_STRATEGY_SAFE_KNOWN_RETURN;
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
}

void nav_supervisor_cancel(void)
{
    supervisor_state.state = NAV_SUPERVISOR_STATE_CANCELLED;
    supervisor_state.done_reason = NAV_SUPERVISOR_DONE_REASON_CANCELLED;
    supervisor_state.return_requested = false;
    supervisor_state.waiting_action_done = false;
    supervisor_state.return_to_start_active = false;
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
            supervisor_state.state = NAV_SUPERVISOR_STATE_ERROR;
            supervisor_state.done_reason =
                NAV_SUPERVISOR_DONE_REASON_NO_FRONTIER_BEFORE_REQUIRED_SPECIALS;
            if (output != 0) {
                output->request_stop_autonomy = true;
                output->request_stop_motors = true;
            }
            return;
        }

        if (input->found_special_count < supervisor_state.config.required_special_count) {
            return;
        }

        supervisor_state.required_specials_reached = true;
        supervisor_state.return_requested = true;
        supervisor_state.waiting_action_done = !input->nav_ready;
        supervisor_state.clear_plan_requested = true;
        supervisor_state.return_plan_requested = false;
        supervisor_state.execute_return_requested = false;
        supervisor_state.state = NAV_SUPERVISOR_STATE_FOUND_REQUIRED_SPECIALS_WAIT_ACTION_DONE;
        if (output != 0) {
            output->block_smart_actions = true;
            output->request_clear_exploration_plan = true;
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

static void store_smart_output(const NavSupervisorSmartInput *input,
                               const NavSupervisorSmartOutput *output)
{
    supervisor_state.smart_output = output != 0 ? *output : (NavSupervisorSmartOutput){0};
    supervisor_state.smart_local_action =
        input != 0 ? input->recommended_action : NAV_RECOMMENDED_NONE;
    supervisor_state.smart_frontier_status =
        input != 0 ? input->frontier_route_status : NAV_ROUTE_STATUS_IDLE;
    supervisor_state.smart_blocked_by_mission =
        input != 0 ? input->mission_block_smart_actions : false;
}

void nav_supervisor_update_smart_shadow(const NavSupervisorSmartInput *input,
                                        NavSupervisorSmartOutput *output)
{
    clear_smart_output(output);
    if (input == 0) {
        store_smart_output(0, output);
        return;
    }

    NavSupervisorSmartOutput local_output = {0};
    if (!input->autonomy_enabled) {
        local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_IDLE;
        local_output.decision_reason =
            NAV_SUPERVISOR_SMART_DECISION_REASON_AUTONOMY_DISABLED;
    } else if (input->policy != NAV_POLICY_SMART_RECOGNITION) {
        local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_IDLE;
        local_output.decision_reason =
            NAV_SUPERVISOR_SMART_DECISION_REASON_POLICY_NOT_SMART;
    } else if (input->mission_block_smart_actions) {
        local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_BLOCKED_BY_MISSION;
        local_output.decision_reason =
            NAV_SUPERVISOR_SMART_DECISION_REASON_BLOCKED_BY_MISSION;
        local_output.block_new_actions = true;
    } else if (input->plan_execution_enabled) {
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
        local_output.request_plan_to_frontier = true;

        switch (input->frontier_route_status) {
        case NAV_ROUTE_STATUS_FOUND:
            if (input->frontier_plan_loaded) {
                local_output.smart_state =
                    NAV_SUPERVISOR_SMART_STATE_EXECUTING_FRONTIER_ROUTE;
                local_output.decision_reason =
                    NAV_SUPERVISOR_SMART_DECISION_REASON_FRONTIER_ROUTE_FOUND;
                local_output.request_execute_frontier_plan = true;
            } else {
                local_output.smart_state = NAV_SUPERVISOR_SMART_STATE_ERROR;
                local_output.decision_reason =
                    NAV_SUPERVISOR_SMART_DECISION_REASON_FRONTIER_ERROR;
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
            break;
        }
    }

    if (output != 0) {
        *output = local_output;
    }
    store_smart_output(input, &local_output);
}
