#include "nav_supervisor.h"

#define NAV_SUPERVISOR_COST_INF 0xFFFFu

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
