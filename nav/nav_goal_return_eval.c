#include "nav_goal_return_eval.h"

#define NAV_GOAL_RETURN_MAX_CELLS (NAV_MAP_MAX_WIDTH * NAV_MAP_MAX_HEIGHT)
#define NAV_GOAL_RETURN_MAX_STATES (NAV_GOAL_RETURN_MAX_CELLS * 2u)
#define NAV_GOAL_RETURN_NO_PARENT (-1)

typedef struct NavGoalReturnWorkspace {
    uint16_t cost[NAV_GOAL_RETURN_MAX_STATES];
    uint8_t closed[NAV_GOAL_RETURN_MAX_STATES];
    int16_t parent[NAV_GOAL_RETURN_MAX_STATES];
    uint8_t parent_dir[NAV_GOAL_RETURN_MAX_STATES];
    uint8_t unknown_cells[NAV_GOAL_RETURN_MAX_STATES];
    uint8_t unknown_edges[NAV_GOAL_RETURN_MAX_STATES];
    uint16_t reverse_path[NAV_GOAL_RETURN_MAX_STATES];
    uint16_t path[NAV_GOAL_RETURN_MAX_STATES];
    NavGoalReturnEvalResult debug;
} NavGoalReturnWorkspace;

typedef struct NavGoalReturnStepInfo {
    bool blocked;
    bool known_open;
    bool unknown_edge;
    bool target_unvisited;
} NavGoalReturnStepInfo;

static NavGoalReturnWorkspace goal_return_workspace = {0};

static uint16_t goal_return_index(int8_t cell_x, int8_t cell_y, uint8_t width)
{
    return (uint16_t)((uint16_t)cell_y * (uint16_t)width + (uint16_t)cell_x);
}

static uint16_t goal_return_state_index(uint16_t cell_index, bool unknown_used)
{
    return (uint16_t)(cell_index * 2u + (unknown_used ? 1u : 0u));
}

static uint16_t goal_return_state_cell_index(uint16_t state_index)
{
    return (uint16_t)(state_index / 2u);
}

static bool goal_return_state_unknown_used(uint16_t state_index)
{
    return (state_index & 1u) != 0u;
}

static bool goal_return_cell_is_inside(const NavMapDebugSnapshot *map_debug,
                                       int8_t cell_x,
                                       int8_t cell_y)
{
    return map_debug != 0
        && map_debug->enabled
        && cell_x >= 0
        && cell_y >= 0
        && cell_x < (int8_t)map_debug->width
        && cell_y < (int8_t)map_debug->height;
}

static uint8_t goal_return_wall_bit(NavMapDirection dir)
{
    return (uint8_t)(1u << (uint8_t)dir);
}

static NavMapDirection goal_return_opposite_dir(NavMapDirection dir)
{
    return (NavMapDirection)(((uint8_t)dir + 2u) & 3u);
}

static void goal_return_neighbor_for_dir(int8_t cell_x,
                                         int8_t cell_y,
                                         NavMapDirection dir,
                                         int8_t *next_x,
                                         int8_t *next_y)
{
    static const int8_t dx[4] = {0, 1, 0, -1};
    static const int8_t dy[4] = {-1, 0, 1, 0};

    if (next_x != 0) {
        *next_x = (int8_t)(cell_x + dx[(uint8_t)dir]);
    }
    if (next_y != 0) {
        *next_y = (int8_t)(cell_y + dy[(uint8_t)dir]);
    }
}

static bool goal_return_dir_between_cells(int8_t from_x,
                                          int8_t from_y,
                                          int8_t to_x,
                                          int8_t to_y,
                                          NavMapDirection *dir_out)
{
    for (uint8_t dir_value = 0u; dir_value < 4u; ++dir_value) {
        int8_t next_x = 0;
        int8_t next_y = 0;
        goal_return_neighbor_for_dir(from_x,
                                     from_y,
                                     (NavMapDirection)dir_value,
                                     &next_x,
                                     &next_y);
        if (next_x == to_x && next_y == to_y) {
            if (dir_out != 0) {
                *dir_out = (NavMapDirection)dir_value;
            }
            return true;
        }
    }

    return false;
}

static bool goal_return_cell_wall_present(const NavMapCell *cell, NavMapDirection dir)
{
    const uint8_t bit = goal_return_wall_bit(dir);
    return cell != 0
        && (cell->walls_known & bit) != 0u
        && (cell->walls_present & bit) != 0u;
}

static bool goal_return_cell_wall_known_open(const NavMapCell *cell, NavMapDirection dir)
{
    const uint8_t bit = goal_return_wall_bit(dir);
    return cell != 0
        && (cell->walls_known & bit) != 0u
        && (cell->walls_present & bit) == 0u;
}

static NavGoalReturnStepInfo goal_return_step_info(
    const NavMapDebugSnapshot *map_debug,
    int8_t from_x,
    int8_t from_y,
    NavMapDirection dir)
{
    NavGoalReturnStepInfo info = {0};
    int8_t to_x = 0;
    int8_t to_y = 0;
    goal_return_neighbor_for_dir(from_x, from_y, dir, &to_x, &to_y);
    if (!goal_return_cell_is_inside(map_debug, to_x, to_y)) {
        info.blocked = true;
        return info;
    }

    NavMapCell from_cell = {0};
    NavMapCell to_cell = {0};
    if (!nav_map_get_cell(from_x, from_y, &from_cell)
        || !nav_map_get_cell(to_x, to_y, &to_cell)) {
        info.blocked = true;
        return info;
    }

    const NavMapDirection opposite_dir = goal_return_opposite_dir(dir);
    if (goal_return_cell_wall_present(&from_cell, dir)
        || goal_return_cell_wall_present(&to_cell, opposite_dir)) {
        info.blocked = true;
        return info;
    }

    info.known_open =
        goal_return_cell_wall_known_open(&from_cell, dir)
        || goal_return_cell_wall_known_open(&to_cell, opposite_dir);
    info.unknown_edge = !info.known_open;
    info.target_unvisited = !to_cell.visited;
    return info;
}

static void goal_return_set_default_result(NavGoalReturnEvalResult *result)
{
    if (result == 0) {
        return;
    }

    *result = (NavGoalReturnEvalResult){0};
    result->status = NAV_GOAL_RETURN_EVAL_STATUS_IDLE;
    result->decision = NAV_GOAL_RETURN_DECISION_NONE;
    result->reason = NAV_GOAL_RETURN_REASON_NONE;
    result->safe_return_cost = NAV_GOAL_RETURN_COST_INF;
    result->optimistic_any_cost = NAV_GOAL_RETURN_COST_INF;
    result->optimistic_shortcut_cost = NAV_GOAL_RETURN_COST_INF;
    result->optimistic_return_cost = NAV_GOAL_RETURN_COST_INF;
    result->unknown_used_path_found = false;
    result->frontier_cell_x = -1;
    result->frontier_cell_y = -1;
    result->frontier_neighbor_x = -1;
    result->frontier_neighbor_y = -1;
    result->exit_dir = NAV_DIR_NORTH;
    result->supported_arrival_dir_mask = 0u;
}

void nav_goal_return_eval_clear(void)
{
    for (uint16_t i = 0u; i < NAV_GOAL_RETURN_MAX_STATES; ++i) {
        goal_return_workspace.cost[i] = NAV_GOAL_RETURN_COST_INF;
        goal_return_workspace.closed[i] = 0u;
        goal_return_workspace.parent[i] = NAV_GOAL_RETURN_NO_PARENT;
        goal_return_workspace.parent_dir[i] = 0u;
        goal_return_workspace.unknown_cells[i] = 0u;
        goal_return_workspace.unknown_edges[i] = 0u;
        goal_return_workspace.reverse_path[i] = 0u;
        goal_return_workspace.path[i] = 0u;
    }
    goal_return_set_default_result(&goal_return_workspace.debug);
}

static void goal_return_set_fallback(NavGoalReturnEvalStatus status,
                                     NavGoalReturnReason reason)
{
    goal_return_workspace.debug.status = status;
    goal_return_workspace.debug.decision = NAV_GOAL_RETURN_DECISION_FALLBACK_SAFE;
    goal_return_workspace.debug.reason = reason;
}

static int16_t goal_return_find_open_lowest_cost(uint16_t state_count)
{
    uint16_t best_cost = NAV_GOAL_RETURN_COST_INF;
    int16_t best_index = -1;
    for (uint16_t i = 0u; i < state_count; ++i) {
        if (goal_return_workspace.closed[i] != 0u
            || goal_return_workspace.cost[i] == NAV_GOAL_RETURN_COST_INF) {
            continue;
        }
        if (goal_return_workspace.cost[i] < best_cost) {
            best_cost = goal_return_workspace.cost[i];
            best_index = (int16_t)i;
        }
    }

    return best_index;
}

static uint8_t goal_return_clamp_u8(uint16_t value)
{
    return value > 0xFFu ? 0xFFu : (uint8_t)value;
}

static void goal_return_decode_index(uint16_t index,
                                     uint8_t width,
                                     int8_t *cell_x,
                                     int8_t *cell_y)
{
    if (cell_x != 0) {
        *cell_x = (int8_t)(index % width);
    }
    if (cell_y != 0) {
        *cell_y = (int8_t)(index / width);
    }
}

static uint8_t goal_return_supported_arrival_dir_mask(NavMapDirection exit_dir,
                                                      bool allow_back_entry)
{
    uint8_t mask = 0u;
    for (uint8_t arrival_value = 0u; arrival_value < 4u; ++arrival_value) {
        const uint8_t delta =
            (uint8_t)(((uint8_t)exit_dir - arrival_value) & 3u);
        if (delta == 2u && !allow_back_entry) {
            continue;
        }
        mask |= (uint8_t)(1u << arrival_value);
    }
    return mask;
}

static bool goal_return_reconstruct_path(uint16_t start_index,
                                         uint16_t goal_index,
                                         uint16_t *path_out,
                                         uint16_t *path_count_out)
{
    uint16_t reverse_count = 0u;
    int16_t cursor = (int16_t)goal_index;
    while (cursor >= 0) {
        if (reverse_count >= NAV_GOAL_RETURN_MAX_STATES) {
            return false;
        }
        goal_return_workspace.reverse_path[reverse_count++] = (uint16_t)cursor;
        if ((uint16_t)cursor == start_index) {
            break;
        }
        cursor = goal_return_workspace.parent[(uint16_t)cursor];
    }

    if (reverse_count == 0u
        || goal_return_workspace.reverse_path[reverse_count - 1u] != start_index) {
        return false;
    }

    for (uint16_t i = 0u; i < reverse_count; ++i) {
        path_out[i] = goal_return_workspace.reverse_path[reverse_count - 1u - i];
    }
    if (path_count_out != 0) {
        *path_count_out = reverse_count;
    }
    return true;
}

static void goal_return_find_first_frontier(const NavMapDebugSnapshot *map_debug,
                                            const uint16_t *path,
                                            uint16_t path_count,
                                            uint8_t width,
                                            bool allow_back_entry)
{
    for (uint16_t i = 0u; i + 1u < path_count; ++i) {
        int8_t from_x = 0;
        int8_t from_y = 0;
        int8_t to_x = 0;
        int8_t to_y = 0;
        goal_return_decode_index(goal_return_state_cell_index(path[i]),
                                 width,
                                 &from_x,
                                 &from_y);
        goal_return_decode_index(goal_return_state_cell_index(path[i + 1u]),
                                 width,
                                 &to_x,
                                 &to_y);

        NavMapDirection exit_dir = NAV_DIR_NORTH;
        if (!goal_return_dir_between_cells(from_x, from_y, to_x, to_y, &exit_dir)) {
            continue;
        }

        const NavGoalReturnStepInfo info =
            goal_return_step_info(map_debug, from_x, from_y, exit_dir);
        if (info.blocked) {
            continue;
        }

        if (info.unknown_edge || info.target_unvisited) {
            goal_return_workspace.debug.first_frontier_found = true;
            goal_return_workspace.debug.frontier_cell_x = from_x;
            goal_return_workspace.debug.frontier_cell_y = from_y;
            goal_return_workspace.debug.frontier_neighbor_x = to_x;
            goal_return_workspace.debug.frontier_neighbor_y = to_y;
            goal_return_workspace.debug.exit_dir = exit_dir;
            goal_return_workspace.debug.supported_arrival_dir_mask =
                goal_return_supported_arrival_dir_mask(exit_dir, allow_back_entry);
            return;
        }
    }
}

NavGoalReturnEvalStatus nav_goal_return_eval_evaluate(
    const NavGoalReturnEvalConfig *config,
    NavGoalReturnEvalResult *result)
{
    nav_goal_return_eval_clear();

    if (config == 0) {
        goal_return_set_fallback(NAV_GOAL_RETURN_EVAL_STATUS_INVALID_INPUT,
                                 NAV_GOAL_RETURN_REASON_NO_PATH);
        if (result != 0) {
            *result = goal_return_workspace.debug;
        }
        return goal_return_workspace.debug.status;
    }

    goal_return_workspace.debug.valid = true;
    goal_return_workspace.debug.safe_return_cost = config->safe_return_cost;

    NavMapDebugSnapshot map_debug = {0};
    nav_map_get_debug_snapshot(&map_debug);
    if (!map_debug.enabled
        || !goal_return_cell_is_inside(&map_debug,
                                       config->current_cell_x,
                                       config->current_cell_y)
        || !goal_return_cell_is_inside(&map_debug,
                                       config->start_cell_x,
                                       config->start_cell_y)) {
        goal_return_set_fallback(NAV_GOAL_RETURN_EVAL_STATUS_INVALID_INPUT,
                                 NAV_GOAL_RETURN_REASON_NO_PATH);
        if (result != 0) {
            *result = goal_return_workspace.debug;
        }
        return goal_return_workspace.debug.status;
    }

    if (config->max_shortcut_attempts == 0u) {
        goal_return_set_fallback(NAV_GOAL_RETURN_EVAL_STATUS_OK,
                                 NAV_GOAL_RETURN_REASON_ATTEMPT_BUDGET_EXHAUSTED);
        if (result != 0) {
            *result = goal_return_workspace.debug;
        }
        return goal_return_workspace.debug.status;
    }

    if (config->safe_return_cost < config->min_safe_return_cost_to_try) {
        goal_return_set_fallback(NAV_GOAL_RETURN_EVAL_STATUS_OK,
                                 NAV_GOAL_RETURN_REASON_SAFE_RETURN_TOO_SHORT);
        if (result != 0) {
            *result = goal_return_workspace.debug;
        }
        return goal_return_workspace.debug.status;
    }

    const uint16_t cell_count =
        (uint16_t)map_debug.width * (uint16_t)map_debug.height;
    const uint16_t state_count = (uint16_t)(cell_count * 2u);
    const uint16_t source_index =
        goal_return_index(config->current_cell_x, config->current_cell_y, map_debug.width);
    const uint16_t target_index =
        goal_return_index(config->start_cell_x, config->start_cell_y, map_debug.width);
    const uint16_t source_state = goal_return_state_index(source_index, false);
    const uint16_t target_known_state = goal_return_state_index(target_index, false);
    const uint16_t target_shortcut_state = goal_return_state_index(target_index, true);

    goal_return_workspace.cost[source_state] = 0u;

    bool budget_blocked_any_transition = false;
    bool unknown_transition_seen = false;
    while (true) {
        const int16_t current_state_signed = goal_return_find_open_lowest_cost(state_count);
        if (current_state_signed < 0) {
            break;
        }

        const uint16_t current_state = (uint16_t)current_state_signed;
        const uint16_t current_index = goal_return_state_cell_index(current_state);
        const bool current_unknown_used = goal_return_state_unknown_used(current_state);
        goal_return_workspace.closed[current_state] = 1u;
        if (current_index == target_index) {
            continue;
        }

        int8_t current_x = 0;
        int8_t current_y = 0;
        goal_return_decode_index(current_index, map_debug.width, &current_x, &current_y);

        for (uint8_t dir_value = 0u; dir_value < 4u; ++dir_value) {
            const NavMapDirection dir = (NavMapDirection)dir_value;
            int8_t next_x = 0;
            int8_t next_y = 0;
            goal_return_neighbor_for_dir(current_x, current_y, dir, &next_x, &next_y);
            if (!goal_return_cell_is_inside(&map_debug, next_x, next_y)) {
                continue;
            }

            const NavGoalReturnStepInfo info =
                goal_return_step_info(&map_debug, current_x, current_y, dir);
            if (info.blocked) {
                continue;
            }

            const uint16_t next_index =
                goal_return_index(next_x, next_y, map_debug.width);
            const bool step_uses_unknown = info.unknown_edge || info.target_unvisited;
            const bool next_unknown_used = current_unknown_used || step_uses_unknown;
            const uint16_t next_state =
                goal_return_state_index(next_index, next_unknown_used);
            if (goal_return_workspace.closed[next_state] != 0u) {
                continue;
            }
            if (step_uses_unknown) {
                unknown_transition_seen = true;
            }

            const uint16_t next_unknown_cells =
                (uint16_t)goal_return_workspace.unknown_cells[current_state]
                + (info.target_unvisited ? 1u : 0u);
            const uint16_t next_unknown_edges =
                (uint16_t)goal_return_workspace.unknown_edges[current_state]
                + (info.unknown_edge ? 1u : 0u);
            if (next_unknown_cells > config->max_unknown_cells
                || next_unknown_edges > config->max_unknown_edges) {
                budget_blocked_any_transition = true;
                continue;
            }

            uint32_t step_cost = 1u;
            if (info.unknown_edge) {
                step_cost += config->unknown_wall_penalty;
            }
            if (info.target_unvisited) {
                step_cost += config->unknown_cell_penalty;
            }

            const uint32_t candidate_cost =
                (uint32_t)goal_return_workspace.cost[current_state] + step_cost;
            if (candidate_cost >= NAV_GOAL_RETURN_COST_INF
                || candidate_cost >= goal_return_workspace.cost[next_state]) {
                continue;
            }

            goal_return_workspace.cost[next_state] = (uint16_t)candidate_cost;
            goal_return_workspace.parent[next_state] = (int16_t)current_state;
            goal_return_workspace.parent_dir[next_state] = dir_value;
            goal_return_workspace.unknown_cells[next_state] =
                goal_return_clamp_u8(next_unknown_cells);
            goal_return_workspace.unknown_edges[next_state] =
                goal_return_clamp_u8(next_unknown_edges);
        }
    }

    const uint16_t target_known_cost = goal_return_workspace.cost[target_known_state];
    const uint16_t target_shortcut_cost =
        goal_return_workspace.cost[target_shortcut_state];
    goal_return_workspace.debug.optimistic_any_cost =
        target_known_cost < target_shortcut_cost ? target_known_cost : target_shortcut_cost;
    goal_return_workspace.debug.optimistic_shortcut_cost = target_shortcut_cost;
    goal_return_workspace.debug.optimistic_return_cost = target_shortcut_cost;
    goal_return_workspace.debug.unknown_used_path_found =
        target_shortcut_cost != NAV_GOAL_RETURN_COST_INF;

    if (goal_return_workspace.debug.optimistic_any_cost == NAV_GOAL_RETURN_COST_INF) {
        goal_return_set_fallback(
            budget_blocked_any_transition
                ? NAV_GOAL_RETURN_EVAL_STATUS_BUDGET_EXCEEDED
                : NAV_GOAL_RETURN_EVAL_STATUS_NO_PATH,
            budget_blocked_any_transition
                ? NAV_GOAL_RETURN_REASON_UNKNOWN_BUDGET_EXCEEDED
                : NAV_GOAL_RETURN_REASON_NO_PATH);
        if (result != 0) {
            *result = goal_return_workspace.debug;
        }
        return goal_return_workspace.debug.status;
    }

    if (target_shortcut_cost == NAV_GOAL_RETURN_COST_INF) {
        goal_return_set_fallback(
            budget_blocked_any_transition
                ? NAV_GOAL_RETURN_EVAL_STATUS_BUDGET_EXCEEDED
                : NAV_GOAL_RETURN_EVAL_STATUS_NO_SHORTCUT_FRONTIER,
            budget_blocked_any_transition
                ? NAV_GOAL_RETURN_REASON_UNKNOWN_BUDGET_EXCEEDED
                : NAV_GOAL_RETURN_REASON_NO_SHORTCUT_FRONTIER);
        if (!unknown_transition_seen
            && goal_return_workspace.debug.status
                == NAV_GOAL_RETURN_EVAL_STATUS_BUDGET_EXCEEDED) {
            goal_return_set_fallback(NAV_GOAL_RETURN_EVAL_STATUS_NO_SHORTCUT_FRONTIER,
                                     NAV_GOAL_RETURN_REASON_NO_SHORTCUT_FRONTIER);
        }
        if (result != 0) {
            *result = goal_return_workspace.debug;
        }
        return goal_return_workspace.debug.status;
    }

    goal_return_workspace.debug.status = NAV_GOAL_RETURN_EVAL_STATUS_OK;
    goal_return_workspace.debug.unknown_cells_on_path =
        goal_return_workspace.unknown_cells[target_shortcut_state];
    goal_return_workspace.debug.unknown_edges_on_path =
        goal_return_workspace.unknown_edges[target_shortcut_state];

    uint16_t path_count = 0u;
    if (!goal_return_reconstruct_path(source_state,
                                      target_shortcut_state,
                                      goal_return_workspace.path,
                                      &path_count)) {
        goal_return_set_fallback(NAV_GOAL_RETURN_EVAL_STATUS_NO_PATH,
                                 NAV_GOAL_RETURN_REASON_NO_PATH);
        if (result != 0) {
            *result = goal_return_workspace.debug;
        }
        return goal_return_workspace.debug.status;
    }

    goal_return_find_first_frontier(&map_debug,
                                    goal_return_workspace.path,
                                    path_count,
                                    map_debug.width,
                                    config->allow_back_entry);

    if (!goal_return_workspace.debug.first_frontier_found) {
        goal_return_set_fallback(NAV_GOAL_RETURN_EVAL_STATUS_NO_SHORTCUT_FRONTIER,
                                 NAV_GOAL_RETURN_REASON_NO_SHORTCUT_FRONTIER);
        if (result != 0) {
            *result = goal_return_workspace.debug;
        }
        return goal_return_workspace.debug.status;
    }

    goal_return_workspace.debug.score_improvement =
        (int32_t)config->safe_return_cost
        - (int32_t)goal_return_workspace.debug.optimistic_shortcut_cost;

    if (goal_return_workspace.debug.score_improvement >= (int32_t)config->score_margin) {
        goal_return_workspace.debug.decision =
            NAV_GOAL_RETURN_DECISION_TRY_SHORTCUT;
        goal_return_workspace.debug.reason =
            NAV_GOAL_RETURN_REASON_OPTIMISTIC_BETTER_THAN_SAFE;
    } else {
        goal_return_set_fallback(
            NAV_GOAL_RETURN_EVAL_STATUS_OK,
            NAV_GOAL_RETURN_REASON_OPTIMISTIC_NOT_BETTER_THAN_SAFE);
    }

    if (result != 0) {
        *result = goal_return_workspace.debug;
    }
    return goal_return_workspace.debug.status;
}

void nav_goal_return_eval_get_debug(NavGoalReturnEvalResult *result)
{
    if (result == 0) {
        return;
    }

    *result = goal_return_workspace.debug;
}
