#ifndef NAV_MAP_H
#define NAV_MAP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_MAP_MAX_WIDTH 16
#define NAV_MAP_MAX_HEIGHT 16

typedef enum NavMapDirection {
    NAV_DIR_NORTH = 0,
    NAV_DIR_EAST,
    NAV_DIR_SOUTH,
    NAV_DIR_WEST
} NavMapDirection;

typedef enum NavMapAction {
    NAV_MAP_ACTION_NONE = 0,
    NAV_MAP_ACTION_INITIAL_SNAPSHOT,
    NAV_MAP_ACTION_ADVANCE_LINE,
    NAV_MAP_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT,
    NAV_MAP_ACTION_SMOOTH_TURN_LEFT,
    NAV_MAP_ACTION_SMOOTH_TURN_RIGHT,
    NAV_MAP_ACTION_PIVOT_TURN_LEFT,
    NAV_MAP_ACTION_PIVOT_TURN_RIGHT,
    NAV_MAP_ACTION_PIVOT_TURN_180
} NavMapAction;

typedef enum NavMapWall {
    NAV_MAP_WALL_NORTH = 1 << NAV_DIR_NORTH,
    NAV_MAP_WALL_EAST = 1 << NAV_DIR_EAST,
    NAV_MAP_WALL_SOUTH = 1 << NAV_DIR_SOUTH,
    NAV_MAP_WALL_WEST = 1 << NAV_DIR_WEST
} NavMapWall;

typedef struct NavMapCell {
    uint8_t walls_known;
    uint8_t walls_present;
    bool visited;
} NavMapCell;

typedef struct NavMapDebugSnapshot {
    bool enabled;
    uint8_t width;
    uint8_t height;
    int8_t cell_x;
    int8_t cell_y;
    NavMapDirection dir;
    bool current_cell_visited;
    uint8_t current_cell_walls_known;
    uint8_t current_cell_walls_present;
    NavMapAction last_pose_update_action;
    NavMapAction last_wall_update_action;
    bool initial_wall_snapshot_pending;
    uint32_t update_count;
    uint32_t wall_update_count;
} NavMapDebugSnapshot;

void nav_map_init(uint8_t width,
                  uint8_t height,
                  int8_t start_cell_x,
                  int8_t start_cell_y,
                  NavMapDirection start_dir);
void nav_map_set_pose(int8_t cell_x, int8_t cell_y, NavMapDirection dir);
void nav_map_get_pose(int8_t *cell_x, int8_t *cell_y, NavMapDirection *dir);
void nav_map_mark_visited_current(void);
void nav_map_update_current_cell_walls_from_relative(bool front, bool left, bool right);
void nav_map_update_current_cell_walls_from_relative_for_action(bool front,
                                                                bool left,
                                                                bool right,
                                                                NavMapAction action);
void nav_map_apply_completed_action(NavMapAction action);
bool nav_map_get_cell(int8_t cell_x, int8_t cell_y, NavMapCell *cell);
void nav_map_get_debug_snapshot(NavMapDebugSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif // NAV_MAP_H
