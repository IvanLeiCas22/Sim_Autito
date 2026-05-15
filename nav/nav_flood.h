#ifndef NAV_FLOOD_H
#define NAV_FLOOD_H

#include <stdbool.h>
#include <stdint.h>

#include "nav_map.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_FLOOD_MAX_CELLS (NAV_MAP_MAX_WIDTH * NAV_MAP_MAX_HEIGHT)
#define NAV_FLOOD_COST_INF 0xFFFFu

typedef enum NavFloodStatus {
    NAV_FLOOD_STATUS_IDLE = 0,
    NAV_FLOOD_STATUS_OK,
    NAV_FLOOD_STATUS_GOAL_OUT_OF_BOUNDS,
    NAV_FLOOD_STATUS_GOAL_NOT_VISITED,
    NAV_FLOOD_STATUS_MAP_DISABLED
} NavFloodStatus;

typedef struct NavFloodDebugSnapshot {
    NavFloodStatus status;
    bool valid;
    uint8_t width;
    uint8_t height;
    int8_t goal_x;
    int8_t goal_y;
    uint16_t current_cell_cost;
    uint16_t reached_count;
    uint16_t expanded_count;
} NavFloodDebugSnapshot;

void nav_flood_clear(void);
NavFloodStatus nav_flood_fill_to_cell(int8_t goal_x, int8_t goal_y);
uint16_t nav_flood_get_cost(int8_t cell_x, int8_t cell_y);
void nav_flood_get_debug_snapshot(NavFloodDebugSnapshot *snapshot);
bool nav_flood_is_valid(void);

#ifdef __cplusplus
}
#endif

#endif // NAV_FLOOD_H
