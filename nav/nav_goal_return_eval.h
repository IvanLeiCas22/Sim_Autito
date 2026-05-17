#ifndef NAV_GOAL_RETURN_EVAL_H
#define NAV_GOAL_RETURN_EVAL_H

#include <stdbool.h>
#include <stdint.h>

#include "nav_map.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_GOAL_RETURN_COST_INF 0xFFFFu

typedef enum NavGoalReturnEvalStatus {
    NAV_GOAL_RETURN_EVAL_STATUS_IDLE = 0,
    NAV_GOAL_RETURN_EVAL_STATUS_OK,
    NAV_GOAL_RETURN_EVAL_STATUS_INVALID_INPUT,
    NAV_GOAL_RETURN_EVAL_STATUS_NO_PATH,
    NAV_GOAL_RETURN_EVAL_STATUS_NO_SHORTCUT_FRONTIER,
    NAV_GOAL_RETURN_EVAL_STATUS_BUDGET_EXCEEDED
} NavGoalReturnEvalStatus;

typedef enum NavGoalReturnDecision {
    NAV_GOAL_RETURN_DECISION_NONE = 0,
    NAV_GOAL_RETURN_DECISION_TRY_SHORTCUT,
    NAV_GOAL_RETURN_DECISION_FALLBACK_SAFE
} NavGoalReturnDecision;

typedef enum NavGoalReturnReason {
    NAV_GOAL_RETURN_REASON_NONE = 0,
    NAV_GOAL_RETURN_REASON_NO_PATH,
    NAV_GOAL_RETURN_REASON_NO_SHORTCUT_FRONTIER,
    NAV_GOAL_RETURN_REASON_UNKNOWN_BUDGET_EXCEEDED,
    NAV_GOAL_RETURN_REASON_OPTIMISTIC_NOT_BETTER_THAN_SAFE,
    NAV_GOAL_RETURN_REASON_OPTIMISTIC_BETTER_THAN_SAFE,
    NAV_GOAL_RETURN_REASON_SAFE_RETURN_TOO_SHORT,
    NAV_GOAL_RETURN_REASON_ATTEMPT_BUDGET_EXHAUSTED
} NavGoalReturnReason;

typedef struct NavGoalReturnEvalConfig {
    int8_t start_cell_x;
    int8_t start_cell_y;
    int8_t current_cell_x;
    int8_t current_cell_y;
    NavMapDirection current_dir;

    uint16_t safe_return_cost;
    uint16_t score_margin;
    uint16_t min_safe_return_cost_to_try;

    uint16_t unknown_wall_penalty;
    uint16_t unknown_cell_penalty;
    uint8_t max_unknown_cells;
    uint8_t max_unknown_edges;
    uint8_t max_shortcut_attempts;
    bool allow_back_entry;
} NavGoalReturnEvalConfig;

typedef struct NavGoalReturnEvalResult {
    bool valid;
    NavGoalReturnEvalStatus status;
    NavGoalReturnDecision decision;
    NavGoalReturnReason reason;

    uint16_t safe_return_cost;
    uint16_t optimistic_return_cost;
    int32_t score_improvement;

    uint8_t unknown_cells_on_path;
    uint8_t unknown_edges_on_path;

    bool first_frontier_found;
    int8_t frontier_cell_x;
    int8_t frontier_cell_y;
    int8_t frontier_neighbor_x;
    int8_t frontier_neighbor_y;
    NavMapDirection exit_dir;

    uint8_t supported_arrival_dir_mask;
} NavGoalReturnEvalResult;

void nav_goal_return_eval_clear(void);
NavGoalReturnEvalStatus nav_goal_return_eval_evaluate(
    const NavGoalReturnEvalConfig *config,
    NavGoalReturnEvalResult *result);
void nav_goal_return_eval_get_debug(NavGoalReturnEvalResult *result);

#ifdef __cplusplus
}
#endif

#endif // NAV_GOAL_RETURN_EVAL_H
