#ifndef NAV_SUPERVISOR_H
#define NAV_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

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
} NavSupervisorDebugSnapshot;

void nav_supervisor_init(void);
void nav_supervisor_reset(void);
void nav_supervisor_set_config(const NavSupervisorConfig *config);
NavSupervisorConfig nav_supervisor_get_config(void);
void nav_supervisor_get_debug(NavSupervisorDebugSnapshot *snapshot);
void nav_supervisor_cancel(void);
void nav_supervisor_set_start_cell(int8_t x, int8_t y, int8_t dir);

#ifdef __cplusplus
}
#endif

#endif // NAV_SUPERVISOR_H
