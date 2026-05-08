#include "nav_map.h"

typedef struct NavMapState {
    bool enabled;
    uint8_t width;
    uint8_t height;
    int8_t cell_x;
    int8_t cell_y;
    NavMapDirection dir;
    NavMapAction last_pose_update_action;
    NavMapAction last_wall_update_action;
    uint32_t update_count;
    uint32_t wall_update_count;
    NavMapCell cells[NAV_MAP_MAX_HEIGHT][NAV_MAP_MAX_WIDTH];
} NavMapState;

static NavMapState map_state = {0};

static uint8_t clamp_size(uint8_t value, uint8_t max_value)
{
    if (value == 0) {
        return 1;
    }
    if (value > max_value) {
        return max_value;
    }

    return value;
}

static bool is_inside(int8_t cell_x, int8_t cell_y)
{
    return map_state.enabled
        && cell_x >= 0
        && cell_y >= 0
        && cell_x < (int8_t)map_state.width
        && cell_y < (int8_t)map_state.height;
}

static NavMapDirection turn_left(NavMapDirection dir)
{
    return (NavMapDirection)((dir + 3) & 3);
}

static NavMapDirection turn_right(NavMapDirection dir)
{
    return (NavMapDirection)((dir + 1) & 3);
}

static NavMapDirection turn_back(NavMapDirection dir)
{
    return (NavMapDirection)((dir + 2) & 3);
}

static uint8_t wall_bit(NavMapDirection dir)
{
    return (uint8_t)(1u << dir);
}

static void neighbor_for_dir(int8_t cell_x,
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

static void move_forward_one_cell(void)
{
    int8_t next_x = map_state.cell_x;
    int8_t next_y = map_state.cell_y;
    neighbor_for_dir(map_state.cell_x, map_state.cell_y, map_state.dir, &next_x, &next_y);

    if (is_inside(next_x, next_y)) {
        map_state.cell_x = next_x;
        map_state.cell_y = next_y;
    }
}

static void set_absolute_wall(int8_t cell_x,
                              int8_t cell_y,
                              NavMapDirection dir,
                              bool present)
{
    if (!is_inside(cell_x, cell_y)) {
        return;
    }

    NavMapCell *cell = &map_state.cells[(uint8_t)cell_y][(uint8_t)cell_x];
    const uint8_t bit = wall_bit(dir);
    cell->walls_known |= bit;
    if (present) {
        cell->walls_present |= bit;
    } else {
        cell->walls_present &= (uint8_t)~bit;
    }

    int8_t neighbor_x = cell_x;
    int8_t neighbor_y = cell_y;
    neighbor_for_dir(cell_x, cell_y, dir, &neighbor_x, &neighbor_y);
    if (is_inside(neighbor_x, neighbor_y)) {
        NavMapCell *neighbor = &map_state.cells[(uint8_t)neighbor_y][(uint8_t)neighbor_x];
        const uint8_t opposite_bit = wall_bit(turn_back(dir));
        neighbor->walls_known |= opposite_bit;
        if (present) {
            neighbor->walls_present |= opposite_bit;
        } else {
            neighbor->walls_present &= (uint8_t)~opposite_bit;
        }
    }
}

void nav_map_init(uint8_t width,
                  uint8_t height,
                  int8_t start_cell_x,
                  int8_t start_cell_y,
                  NavMapDirection start_dir)
{
    map_state = (NavMapState){0};
    map_state.enabled = true;
    map_state.width = clamp_size(width, NAV_MAP_MAX_WIDTH);
    map_state.height = clamp_size(height, NAV_MAP_MAX_HEIGHT);
    nav_map_set_pose(start_cell_x, start_cell_y, start_dir);
    nav_map_mark_visited_current();
    map_state.last_pose_update_action = NAV_MAP_ACTION_NONE;
    map_state.last_wall_update_action = NAV_MAP_ACTION_NONE;
    map_state.update_count = 0;
    map_state.wall_update_count = 0;
}

void nav_map_set_pose(int8_t cell_x, int8_t cell_y, NavMapDirection dir)
{
    if (!map_state.enabled) {
        return;
    }

    if (cell_x < 0) {
        cell_x = 0;
    } else if (cell_x >= (int8_t)map_state.width) {
        cell_x = (int8_t)(map_state.width - 1);
    }

    if (cell_y < 0) {
        cell_y = 0;
    } else if (cell_y >= (int8_t)map_state.height) {
        cell_y = (int8_t)(map_state.height - 1);
    }

    map_state.cell_x = cell_x;
    map_state.cell_y = cell_y;
    map_state.dir = (NavMapDirection)(dir & 3);
}

void nav_map_get_pose(int8_t *cell_x, int8_t *cell_y, NavMapDirection *dir)
{
    if (cell_x != 0) {
        *cell_x = map_state.cell_x;
    }
    if (cell_y != 0) {
        *cell_y = map_state.cell_y;
    }
    if (dir != 0) {
        *dir = map_state.dir;
    }
}

void nav_map_mark_visited_current(void)
{
    if (!is_inside(map_state.cell_x, map_state.cell_y)) {
        return;
    }

    map_state.cells[(uint8_t)map_state.cell_y][(uint8_t)map_state.cell_x].visited = true;
}

void nav_map_update_current_cell_walls_from_relative(bool front, bool left, bool right)
{
    nav_map_update_current_cell_walls_from_relative_for_action(front,
                                                               left,
                                                               right,
                                                               NAV_MAP_ACTION_NONE);
}

void nav_map_update_current_cell_walls_from_relative_for_action(bool front,
                                                                bool left,
                                                                bool right,
                                                                NavMapAction action)
{
    if (!is_inside(map_state.cell_x, map_state.cell_y)) {
        return;
    }

    set_absolute_wall(map_state.cell_x, map_state.cell_y, map_state.dir, front);
    set_absolute_wall(map_state.cell_x, map_state.cell_y, turn_left(map_state.dir), left);
    set_absolute_wall(map_state.cell_x, map_state.cell_y, turn_right(map_state.dir), right);
    map_state.last_wall_update_action = action;
    map_state.wall_update_count++;
}

void nav_map_apply_completed_action(NavMapAction action)
{
    if (!map_state.enabled) {
        return;
    }

    const int8_t previous_cell_x = map_state.cell_x;
    const int8_t previous_cell_y = map_state.cell_y;
    const NavMapDirection previous_dir = map_state.dir;

    switch (action) {
    case NAV_MAP_ACTION_ADVANCE_LINE:
        move_forward_one_cell();
        break;
    case NAV_MAP_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT:
        break;
    case NAV_MAP_ACTION_SMOOTH_TURN_LEFT:
        map_state.dir = turn_left(map_state.dir);
        move_forward_one_cell();
        break;
    case NAV_MAP_ACTION_SMOOTH_TURN_RIGHT:
        map_state.dir = turn_right(map_state.dir);
        move_forward_one_cell();
        break;
    case NAV_MAP_ACTION_PIVOT_TURN_LEFT:
        map_state.dir = turn_left(map_state.dir);
        break;
    case NAV_MAP_ACTION_PIVOT_TURN_RIGHT:
        map_state.dir = turn_right(map_state.dir);
        break;
    case NAV_MAP_ACTION_PIVOT_TURN_180:
        map_state.dir = turn_back(map_state.dir);
        break;
    case NAV_MAP_ACTION_INITIAL_SNAPSHOT:
    case NAV_MAP_ACTION_NONE:
        break;
    }

    if (map_state.cell_x == previous_cell_x
        && map_state.cell_y == previous_cell_y
        && map_state.dir == previous_dir) {
        return;
    }

    map_state.last_pose_update_action = action;
    map_state.update_count++;
}

bool nav_map_get_cell(int8_t cell_x, int8_t cell_y, NavMapCell *cell)
{
    if (!is_inside(cell_x, cell_y) || cell == 0) {
        return false;
    }

    *cell = map_state.cells[(uint8_t)cell_y][(uint8_t)cell_x];
    return true;
}

void nav_map_get_debug_snapshot(NavMapDebugSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    *snapshot = (NavMapDebugSnapshot){0};
    snapshot->enabled = map_state.enabled;
    snapshot->width = map_state.width;
    snapshot->height = map_state.height;
    snapshot->cell_x = map_state.cell_x;
    snapshot->cell_y = map_state.cell_y;
    snapshot->dir = map_state.dir;
    snapshot->last_pose_update_action = map_state.last_pose_update_action;
    snapshot->last_wall_update_action = map_state.last_wall_update_action;
    snapshot->update_count = map_state.update_count;
    snapshot->wall_update_count = map_state.wall_update_count;

    NavMapCell current = {0};
    if (nav_map_get_cell(map_state.cell_x, map_state.cell_y, &current)) {
        snapshot->current_cell_visited = current.visited;
        snapshot->current_cell_walls_known = current.walls_known;
        snapshot->current_cell_walls_present = current.walls_present;
    }
}
