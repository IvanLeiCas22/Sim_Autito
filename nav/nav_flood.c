#include "nav_flood.h"

typedef struct NavFloodWorkspace {
    uint16_t cost[NAV_FLOOD_MAX_CELLS];
    uint16_t queue[NAV_FLOOD_MAX_CELLS];
    NavFloodStatus status;
    bool valid;
    uint8_t width;
    uint8_t height;
    int8_t goal_x;
    int8_t goal_y;
    uint16_t reached_count;
    uint16_t expanded_count;
} NavFloodWorkspace;

static NavFloodWorkspace flood_workspace = {0};

static uint16_t flood_index(int8_t cell_x, int8_t cell_y, uint8_t width)
{
    return (uint16_t)((uint16_t)cell_y * (uint16_t)width + (uint16_t)cell_x);
}

static bool flood_cell_is_inside(const NavMapDebugSnapshot *map_debug,
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

static uint8_t flood_wall_bit(NavMapDirection dir)
{
    return (uint8_t)(1u << (uint8_t)dir);
}

static NavMapDirection flood_opposite_dir(NavMapDirection dir)
{
    return (NavMapDirection)(((uint8_t)dir + 2u) & 3u);
}

static void flood_neighbor_for_dir(int8_t cell_x,
                                   int8_t cell_y,
                                   NavMapDirection dir,
                                   int8_t *next_x,
                                   int8_t *next_y)
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

    if (next_x != 0) {
        *next_x = (int8_t)(cell_x + dx);
    }
    if (next_y != 0) {
        *next_y = (int8_t)(cell_y + dy);
    }
}

static void flood_reset_costs(void)
{
    for (uint16_t i = 0; i < NAV_FLOOD_MAX_CELLS; ++i) {
        flood_workspace.cost[i] = NAV_FLOOD_COST_INF;
        flood_workspace.queue[i] = 0;
    }
    flood_workspace.reached_count = 0;
    flood_workspace.expanded_count = 0;
}

void nav_flood_clear(void)
{
    flood_reset_costs();
    flood_workspace.status = NAV_FLOOD_STATUS_IDLE;
    flood_workspace.valid = false;
    flood_workspace.width = 0;
    flood_workspace.height = 0;
    flood_workspace.goal_x = -1;
    flood_workspace.goal_y = -1;
}

static bool flood_can_step_toward_current(int8_t neighbor_x,
                                          int8_t neighbor_y,
                                          NavMapDirection neighbor_to_current_dir)
{
    NavMapCell neighbor = {0};
    if (!nav_map_get_cell(neighbor_x, neighbor_y, &neighbor) || !neighbor.visited) {
        return false;
    }

    const uint8_t bit = flood_wall_bit(neighbor_to_current_dir);
    return (neighbor.walls_known & bit) != 0
        && (neighbor.walls_present & bit) == 0;
}

NavFloodStatus nav_flood_fill_to_cell(int8_t goal_x, int8_t goal_y)
{
    nav_flood_clear();

    NavMapDebugSnapshot map_debug = {0};
    nav_map_get_debug_snapshot(&map_debug);
    flood_workspace.width = map_debug.width;
    flood_workspace.height = map_debug.height;
    flood_workspace.goal_x = goal_x;
    flood_workspace.goal_y = goal_y;

    if (!map_debug.enabled) {
        flood_workspace.status = NAV_FLOOD_STATUS_MAP_DISABLED;
        return flood_workspace.status;
    }

    if (!flood_cell_is_inside(&map_debug, goal_x, goal_y)) {
        flood_workspace.status = NAV_FLOOD_STATUS_GOAL_OUT_OF_BOUNDS;
        return flood_workspace.status;
    }

    NavMapCell goal_cell = {0};
    if (!nav_map_get_cell(goal_x, goal_y, &goal_cell) || !goal_cell.visited) {
        flood_workspace.status = NAV_FLOOD_STATUS_GOAL_NOT_VISITED;
        return flood_workspace.status;
    }

    const uint16_t goal_index = flood_index(goal_x, goal_y, map_debug.width);
    flood_workspace.cost[goal_index] = 0;
    flood_workspace.queue[0] = goal_index;
    flood_workspace.reached_count = 1;

    uint16_t queue_head = 0;
    uint16_t queue_tail = 1;
    while (queue_head < queue_tail) {
        const uint16_t current_index = flood_workspace.queue[queue_head++];
        const int8_t cell_x = (int8_t)(current_index % map_debug.width);
        const int8_t cell_y = (int8_t)(current_index / map_debug.width);
        const uint16_t current_cost = flood_workspace.cost[current_index];
        ++flood_workspace.expanded_count;

        for (uint8_t dir_value = 0; dir_value < 4u; ++dir_value) {
            const NavMapDirection dir_from_current = (NavMapDirection)dir_value;
            int8_t neighbor_x = cell_x;
            int8_t neighbor_y = cell_y;
            flood_neighbor_for_dir(cell_x, cell_y, dir_from_current, &neighbor_x, &neighbor_y);
            if (!flood_cell_is_inside(&map_debug, neighbor_x, neighbor_y)) {
                continue;
            }

            const NavMapDirection neighbor_to_current_dir =
                flood_opposite_dir(dir_from_current);
            if (!flood_can_step_toward_current(neighbor_x,
                                               neighbor_y,
                                               neighbor_to_current_dir)) {
                continue;
            }

            const uint16_t neighbor_index =
                flood_index(neighbor_x, neighbor_y, map_debug.width);
            if (flood_workspace.cost[neighbor_index] != NAV_FLOOD_COST_INF) {
                continue;
            }

            flood_workspace.cost[neighbor_index] = (uint16_t)(current_cost + 1u);
            flood_workspace.queue[queue_tail++] = neighbor_index;
            ++flood_workspace.reached_count;
        }
    }

    flood_workspace.status = NAV_FLOOD_STATUS_OK;
    flood_workspace.valid = true;
    return flood_workspace.status;
}

uint16_t nav_flood_get_cost(int8_t cell_x, int8_t cell_y)
{
    if (!flood_workspace.valid
        || cell_x < 0
        || cell_y < 0
        || cell_x >= (int8_t)flood_workspace.width
        || cell_y >= (int8_t)flood_workspace.height) {
        return NAV_FLOOD_COST_INF;
    }

    return flood_workspace.cost[flood_index(cell_x, cell_y, flood_workspace.width)];
}

void nav_flood_get_debug_snapshot(NavFloodDebugSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    NavMapDebugSnapshot map_debug = {0};
    nav_map_get_debug_snapshot(&map_debug);

    snapshot->status = flood_workspace.status;
    snapshot->valid = flood_workspace.valid;
    snapshot->width = flood_workspace.width;
    snapshot->height = flood_workspace.height;
    snapshot->goal_x = flood_workspace.goal_x;
    snapshot->goal_y = flood_workspace.goal_y;
    snapshot->current_cell_cost = NAV_FLOOD_COST_INF;
    snapshot->reached_count = flood_workspace.reached_count;
    snapshot->expanded_count = flood_workspace.expanded_count;

    if (flood_workspace.valid
        && flood_cell_is_inside(&map_debug, map_debug.cell_x, map_debug.cell_y)) {
        snapshot->current_cell_cost = nav_flood_get_cost(map_debug.cell_x, map_debug.cell_y);
    }
}

bool nav_flood_is_valid(void)
{
    return flood_workspace.valid;
}
