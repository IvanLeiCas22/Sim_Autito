#include "nav_frontier_eval.h"

#define NAV_FRONTIER_EVAL_NEIGHBOR_SEEN_BYTES ((NAV_FLOOD_MAX_CELLS + 7u) / 8u)

typedef struct NavFrontierEvalWorkspace {
    NavFrontierEvalDebugSnapshot debug;
    uint8_t neighbor_seen[NAV_FRONTIER_EVAL_NEIGHBOR_SEEN_BYTES];
} NavFrontierEvalWorkspace;

static NavFrontierEvalWorkspace frontier_eval_workspace = {0};

static uint8_t frontier_wall_bit(NavMapDirection dir)
{
    return (uint8_t)(1u << (uint8_t)dir);
}

static bool frontier_cell_is_inside(const NavMapDebugSnapshot *map_debug,
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

static void frontier_neighbor_for_dir(int8_t cell_x,
                                      int8_t cell_y,
                                      NavMapDirection dir,
                                      int8_t *neighbor_x,
                                      int8_t *neighbor_y)
{
    int8_t dx = 0;
    int8_t dy = 0;
    switch (dir) {
    case NAV_DIR_NORTH:
        dy = -1;
        break;
    case NAV_DIR_EAST:
        dx = 1;
        break;
    case NAV_DIR_SOUTH:
        dy = 1;
        break;
    case NAV_DIR_WEST:
        dx = -1;
        break;
    }

    if (neighbor_x != 0) {
        *neighbor_x = (int8_t)(cell_x + dx);
    }
    if (neighbor_y != 0) {
        *neighbor_y = (int8_t)(cell_y + dy);
    }
}

static NavFrontierEntryRelative frontier_entry_relative(NavMapDirection arrival_dir,
                                                        NavMapDirection exit_dir)
{
    const uint8_t delta = (uint8_t)(((uint8_t)exit_dir - (uint8_t)arrival_dir) & 3u);
    switch (delta) {
    case 0u:
        return NAV_FRONTIER_ENTRY_REL_FRONT;
    case 1u:
        return NAV_FRONTIER_ENTRY_REL_RIGHT;
    case 2u:
        return NAV_FRONTIER_ENTRY_REL_BACK;
    case 3u:
        return NAV_FRONTIER_ENTRY_REL_LEFT;
    }

    return NAV_FRONTIER_ENTRY_REL_NONE;
}

static NavFrontierEntryAction frontier_entry_action_for_relative(
    NavFrontierEntryRelative relative)
{
    switch (relative) {
    case NAV_FRONTIER_ENTRY_REL_FRONT:
        return NAV_FRONTIER_ENTRY_ACTION_ADVANCE_LINE;
    case NAV_FRONTIER_ENTRY_REL_RIGHT:
        return NAV_FRONTIER_ENTRY_ACTION_SMOOTH_RIGHT;
    case NAV_FRONTIER_ENTRY_REL_LEFT:
        return NAV_FRONTIER_ENTRY_ACTION_SMOOTH_LEFT;
    case NAV_FRONTIER_ENTRY_REL_BACK:
        return NAV_FRONTIER_ENTRY_ACTION_UNSUPPORTED_BACK_EXIT;
    case NAV_FRONTIER_ENTRY_REL_NONE:
        break;
    }

    return NAV_FRONTIER_ENTRY_ACTION_NONE;
}

static bool frontier_entry_action_is_supported(NavFrontierEntryAction action,
                                               bool allow_back_entry)
{
    return action == NAV_FRONTIER_ENTRY_ACTION_ADVANCE_LINE
        || action == NAV_FRONTIER_ENTRY_ACTION_SMOOTH_RIGHT
        || action == NAV_FRONTIER_ENTRY_ACTION_SMOOTH_LEFT
        || (allow_back_entry
            && action == NAV_FRONTIER_ENTRY_ACTION_UNSUPPORTED_BACK_EXIT);
}

static void frontier_reset_neighbor_seen(void)
{
    for (uint16_t i = 0; i < NAV_FRONTIER_EVAL_NEIGHBOR_SEEN_BYTES; ++i) {
        frontier_eval_workspace.neighbor_seen[i] = 0u;
    }
}

static bool frontier_neighbor_seen_get(uint16_t index)
{
    if (index >= NAV_FLOOD_MAX_CELLS) {
        return false;
    }

    const uint16_t byte_index = (uint16_t)(index >> 3u);
    const uint8_t bit = (uint8_t)(1u << (index & 7u));
    return (frontier_eval_workspace.neighbor_seen[byte_index] & bit) != 0u;
}

static void frontier_neighbor_seen_set(uint16_t index)
{
    if (index >= NAV_FLOOD_MAX_CELLS) {
        return;
    }

    const uint16_t byte_index = (uint16_t)(index >> 3u);
    const uint8_t bit = (uint8_t)(1u << (index & 7u));
    frontier_eval_workspace.neighbor_seen[byte_index] |= bit;
}

void nav_frontier_eval_clear(void)
{
    frontier_eval_workspace.debug.valid = false;
    frontier_eval_workspace.debug.status = NAV_FRONTIER_EVAL_STATUS_IDLE;
    frontier_eval_workspace.debug.candidate_edge_count = 0;
    frontier_eval_workspace.debug.candidate_cell_count = 0;
    frontier_eval_workspace.debug.candidate_neighbor_cell_count = 0;
    frontier_eval_workspace.debug.best_found = false;
    frontier_eval_workspace.debug.best_cell_x = -1;
    frontier_eval_workspace.debug.best_cell_y = -1;
    frontier_eval_workspace.debug.best_neighbor_cell_x = -1;
    frontier_eval_workspace.debug.best_neighbor_cell_y = -1;
    frontier_eval_workspace.debug.best_exit_dir = NAV_DIR_NORTH;
    frontier_eval_workspace.debug.best_cost_to_start = NAV_FLOOD_COST_INF;
    frontier_eval_workspace.debug.best_neighbor_manhattan = 0;
    frontier_eval_workspace.debug.best_score = NAV_FLOOD_COST_INF;
    frontier_eval_workspace.debug.safe_return_cost = NAV_FLOOD_COST_INF;
    frontier_eval_workspace.debug.score_improvement = 0;
    frontier_eval_workspace.debug.score_margin = 0;
    frontier_eval_workspace.debug.decision = NAV_FRONTIER_EVAL_DECISION_NONE;
    frontier_eval_workspace.debug.decision_reason =
        NAV_FRONTIER_EVAL_DECISION_REASON_NONE;
    frontier_eval_workspace.debug.entry_required_dir = NAV_DIR_NORTH;
    frontier_eval_workspace.debug.entry_relative_from_current_dir =
        NAV_FRONTIER_ENTRY_REL_NONE;
    frontier_eval_workspace.debug.entry_action_from_current_dir =
        NAV_FRONTIER_ENTRY_ACTION_NONE;
    frontier_eval_workspace.debug.entry_supported_from_current_dir = false;
    for (uint8_t i = 0; i < 4u; ++i) {
        frontier_eval_workspace.debug.entry_relative_by_arrival_dir[i] =
            NAV_FRONTIER_ENTRY_REL_NONE;
        frontier_eval_workspace.debug.entry_action_by_arrival_dir[i] =
            NAV_FRONTIER_ENTRY_ACTION_NONE;
    }
    frontier_eval_workspace.debug.preferred_arrival_dir = NAV_DIR_NORTH;
    frontier_eval_workspace.debug.preferred_entry_action =
        NAV_FRONTIER_ENTRY_ACTION_NONE;
    frontier_eval_workspace.debug.preferred_supported = false;
    frontier_reset_neighbor_seen();
}

static uint16_t frontier_neighbor_index(int8_t cell_x, int8_t cell_y)
{
    return (uint16_t)((uint16_t)cell_y * (uint16_t)NAV_MAP_MAX_WIDTH
                      + (uint16_t)cell_x);
}

static uint16_t frontier_manhattan_to_start(int8_t cell_x,
                                            int8_t cell_y,
                                            int8_t start_x,
                                            int8_t start_y)
{
    const int dx = (int)cell_x - (int)start_x;
    const int dy = (int)cell_y - (int)start_y;
    const int abs_dx = dx < 0 ? -dx : dx;
    const int abs_dy = dy < 0 ? -dy : dy;
    return (uint16_t)(abs_dx + abs_dy);
}

static void frontier_update_entry_debug(const NavFrontierEvalConfig *config,
                                        NavMapDirection current_dir)
{
    NavFrontierEvalDebugSnapshot *debug = &frontier_eval_workspace.debug;
    debug->entry_required_dir = debug->best_exit_dir;
    debug->entry_relative_from_current_dir =
        frontier_entry_relative(current_dir, debug->entry_required_dir);
    debug->entry_action_from_current_dir =
        frontier_entry_action_for_relative(debug->entry_relative_from_current_dir);
    debug->entry_supported_from_current_dir =
        frontier_entry_action_is_supported(debug->entry_action_from_current_dir,
                                           config->allow_back_entry);

    for (uint8_t dir_value = 0; dir_value < 4u; ++dir_value) {
        const NavMapDirection arrival_dir = (NavMapDirection)dir_value;
        debug->entry_relative_by_arrival_dir[dir_value] =
            frontier_entry_relative(arrival_dir, debug->entry_required_dir);
        debug->entry_action_by_arrival_dir[dir_value] =
            frontier_entry_action_for_relative(
                debug->entry_relative_by_arrival_dir[dir_value]);
    }

    const NavFrontierEntryAction preferred_actions[] = {
        NAV_FRONTIER_ENTRY_ACTION_ADVANCE_LINE,
        NAV_FRONTIER_ENTRY_ACTION_SMOOTH_RIGHT,
        NAV_FRONTIER_ENTRY_ACTION_SMOOTH_LEFT
    };
    for (uint8_t action_index = 0u; action_index < 3u; ++action_index) {
        const NavFrontierEntryAction preferred_action =
            preferred_actions[action_index];
        for (uint8_t dir_value = 0u; dir_value < 4u; ++dir_value) {
            if (debug->entry_action_by_arrival_dir[dir_value] == preferred_action) {
                debug->preferred_arrival_dir = (NavMapDirection)dir_value;
                debug->preferred_entry_action = preferred_action;
                debug->preferred_supported = true;
                return;
            }
        }
    }
}

NavFrontierEvalStatus nav_frontier_eval_evaluate(const NavFrontierEvalConfig *config)
{
    nav_frontier_eval_clear();

    if (config == 0) {
        frontier_eval_workspace.debug.decision =
            NAV_FRONTIER_EVAL_DECISION_FALLBACK_SAFE;
        frontier_eval_workspace.debug.decision_reason =
            NAV_FRONTIER_EVAL_DECISION_REASON_NO_FLOOD;
        frontier_eval_workspace.debug.status = NAV_FRONTIER_EVAL_STATUS_INVALID_START;
        return frontier_eval_workspace.debug.status;
    }
    frontier_eval_workspace.debug.score_margin = config->score_margin;

    if (!nav_flood_is_valid()) {
        frontier_eval_workspace.debug.decision =
            NAV_FRONTIER_EVAL_DECISION_FALLBACK_SAFE;
        frontier_eval_workspace.debug.decision_reason =
            NAV_FRONTIER_EVAL_DECISION_REASON_NO_FLOOD;
        frontier_eval_workspace.debug.status = NAV_FRONTIER_EVAL_STATUS_NO_FLOOD;
        return frontier_eval_workspace.debug.status;
    }

    NavMapDebugSnapshot map_debug = {0};
    nav_map_get_debug_snapshot(&map_debug);
    if (!map_debug.enabled) {
        frontier_eval_workspace.debug.decision =
            NAV_FRONTIER_EVAL_DECISION_FALLBACK_SAFE;
        frontier_eval_workspace.debug.decision_reason =
            NAV_FRONTIER_EVAL_DECISION_REASON_NO_FLOOD;
        frontier_eval_workspace.debug.status = NAV_FRONTIER_EVAL_STATUS_MAP_DISABLED;
        return frontier_eval_workspace.debug.status;
    }

    if (!frontier_cell_is_inside(&map_debug,
                                 config->start_cell_x,
                                 config->start_cell_y)) {
        frontier_eval_workspace.debug.decision =
            NAV_FRONTIER_EVAL_DECISION_FALLBACK_SAFE;
        frontier_eval_workspace.debug.decision_reason =
            NAV_FRONTIER_EVAL_DECISION_REASON_NO_FLOOD;
        frontier_eval_workspace.debug.status = NAV_FRONTIER_EVAL_STATUS_INVALID_START;
        return frontier_eval_workspace.debug.status;
    }

    NavFrontierEvalDebugSnapshot *debug = &frontier_eval_workspace.debug;
    debug->valid = true;
    debug->status = NAV_FRONTIER_EVAL_STATUS_OK;
    debug->safe_return_cost = nav_flood_get_cost(map_debug.cell_x, map_debug.cell_y);

    for (int8_t y = 0; y < (int8_t)map_debug.height; ++y) {
        for (int8_t x = 0; x < (int8_t)map_debug.width; ++x) {
            NavMapCell cell = {0};
            if (!nav_map_get_cell(x, y, &cell) || !cell.visited) {
                continue;
            }

            const uint16_t flood_cost = nav_flood_get_cost(x, y);
            if (flood_cost == NAV_FLOOD_COST_INF) {
                continue;
            }

            bool cell_has_candidate_edge = false;
            for (uint8_t dir_value = 0u; dir_value < 4u; ++dir_value) {
                const NavMapDirection exit_dir = (NavMapDirection)dir_value;
                const uint8_t wall_bit = frontier_wall_bit(exit_dir);
                if ((cell.walls_known & wall_bit) == 0u
                    || (cell.walls_present & wall_bit) != 0u) {
                    continue;
                }

                int8_t neighbor_x = 0;
                int8_t neighbor_y = 0;
                frontier_neighbor_for_dir(x, y, exit_dir, &neighbor_x, &neighbor_y);
                if (!frontier_cell_is_inside(&map_debug, neighbor_x, neighbor_y)) {
                    continue;
                }

                NavMapCell neighbor = {0};
                if (!nav_map_get_cell(neighbor_x, neighbor_y, &neighbor)
                    || neighbor.visited) {
                    continue;
                }

                const uint16_t neighbor_manhattan =
                    frontier_manhattan_to_start(neighbor_x,
                                                neighbor_y,
                                                config->start_cell_x,
                                                config->start_cell_y);
                /* Pre-score/debug only. C5D will combine this with oriented route eval. */
                const uint32_t score =
                    (uint32_t)flood_cost + 1u + (uint32_t)neighbor_manhattan;

                ++debug->candidate_edge_count;
                if (!cell_has_candidate_edge) {
                    ++debug->candidate_cell_count;
                    cell_has_candidate_edge = true;
                }

                const uint16_t neighbor_index =
                    frontier_neighbor_index(neighbor_x, neighbor_y);
                if (!frontier_neighbor_seen_get(neighbor_index)) {
                    frontier_neighbor_seen_set(neighbor_index);
                    ++debug->candidate_neighbor_cell_count;
                }

                if (!debug->best_found || score < (uint32_t)debug->best_score) {
                    debug->best_found = true;
                    debug->best_cell_x = x;
                    debug->best_cell_y = y;
                    debug->best_neighbor_cell_x = neighbor_x;
                    debug->best_neighbor_cell_y = neighbor_y;
                    debug->best_exit_dir = exit_dir;
                    debug->best_cost_to_start = flood_cost;
                    debug->best_neighbor_manhattan = neighbor_manhattan;
                    debug->best_score =
                        score > NAV_FLOOD_COST_INF
                            ? NAV_FLOOD_COST_INF
                            : (uint16_t)score;
                }
            }
        }
    }

    if (debug->safe_return_cost == NAV_FLOOD_COST_INF) {
        debug->decision = NAV_FRONTIER_EVAL_DECISION_FALLBACK_SAFE;
        debug->decision_reason =
            NAV_FRONTIER_EVAL_DECISION_REASON_CURRENT_CELL_UNREACHABLE;
        return debug->status;
    }

    if (!debug->best_found) {
        debug->status = NAV_FRONTIER_EVAL_STATUS_NO_CANDIDATES;
        debug->decision = NAV_FRONTIER_EVAL_DECISION_FALLBACK_SAFE;
        debug->decision_reason = NAV_FRONTIER_EVAL_DECISION_REASON_NO_FRONTIER;
        return debug->status;
    }

    frontier_update_entry_debug(config, map_debug.dir);

    debug->score_improvement =
        (int32_t)debug->safe_return_cost - (int32_t)debug->best_score;
    if ((uint32_t)debug->best_score + (uint32_t)config->score_margin
        < (uint32_t)debug->safe_return_cost) {
        debug->decision = NAV_FRONTIER_EVAL_DECISION_TRY_FRONTIER;
        debug->decision_reason =
            NAV_FRONTIER_EVAL_DECISION_REASON_FRONTIER_BETTER_THAN_SAFE_RETURN;
    } else {
        debug->decision = NAV_FRONTIER_EVAL_DECISION_FALLBACK_SAFE;
        debug->decision_reason =
            NAV_FRONTIER_EVAL_DECISION_REASON_FRONTIER_NOT_BETTER_THAN_SAFE_RETURN;
    }

    return debug->status;
}

void nav_frontier_eval_get_debug(NavFrontierEvalDebugSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    *snapshot = frontier_eval_workspace.debug;
}
