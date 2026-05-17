#ifndef NAV_FRONTIER_EVAL_H
#define NAV_FRONTIER_EVAL_H

#include <stdbool.h>
#include <stdint.h>

#include "nav_flood.h"
#include "nav_map.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum NavFrontierEvalStatus {
    NAV_FRONTIER_EVAL_STATUS_IDLE = 0,
    NAV_FRONTIER_EVAL_STATUS_OK,
    NAV_FRONTIER_EVAL_STATUS_NO_FLOOD,
    NAV_FRONTIER_EVAL_STATUS_MAP_DISABLED,
    NAV_FRONTIER_EVAL_STATUS_INVALID_START,
    NAV_FRONTIER_EVAL_STATUS_NO_CANDIDATES
} NavFrontierEvalStatus;

typedef enum NavFrontierEvalDecision {
    NAV_FRONTIER_EVAL_DECISION_NONE = 0,
    NAV_FRONTIER_EVAL_DECISION_TRY_FRONTIER,
    NAV_FRONTIER_EVAL_DECISION_FALLBACK_SAFE
} NavFrontierEvalDecision;

typedef enum NavFrontierEvalDecisionReason {
    NAV_FRONTIER_EVAL_DECISION_REASON_NONE = 0,
    NAV_FRONTIER_EVAL_DECISION_REASON_NO_FLOOD,
    NAV_FRONTIER_EVAL_DECISION_REASON_CURRENT_CELL_UNREACHABLE,
    NAV_FRONTIER_EVAL_DECISION_REASON_NO_FRONTIER,
    NAV_FRONTIER_EVAL_DECISION_REASON_FRONTIER_BETTER_THAN_SAFE_RETURN,
    NAV_FRONTIER_EVAL_DECISION_REASON_FRONTIER_NOT_BETTER_THAN_SAFE_RETURN
} NavFrontierEvalDecisionReason;

typedef enum NavFrontierEntryRelative {
    NAV_FRONTIER_ENTRY_REL_NONE = 0,
    NAV_FRONTIER_ENTRY_REL_FRONT,
    NAV_FRONTIER_ENTRY_REL_RIGHT,
    NAV_FRONTIER_ENTRY_REL_LEFT,
    NAV_FRONTIER_ENTRY_REL_BACK
} NavFrontierEntryRelative;

typedef enum NavFrontierEntryAction {
    NAV_FRONTIER_ENTRY_ACTION_NONE = 0,
    NAV_FRONTIER_ENTRY_ACTION_ADVANCE_LINE,
    NAV_FRONTIER_ENTRY_ACTION_SMOOTH_RIGHT,
    NAV_FRONTIER_ENTRY_ACTION_SMOOTH_LEFT,
    NAV_FRONTIER_ENTRY_ACTION_UNSUPPORTED_BACK_EXIT
} NavFrontierEntryAction;

typedef struct NavFrontierEvalConfig {
    int8_t start_cell_x;
    int8_t start_cell_y;
    uint16_t score_margin;
    bool allow_back_entry;
} NavFrontierEvalConfig;

typedef struct NavFrontierEvalDebugSnapshot {
    bool valid;
    NavFrontierEvalStatus status;

    uint16_t candidate_edge_count;
    uint16_t candidate_cell_count;
    uint16_t candidate_neighbor_cell_count;

    bool best_found;
    int8_t best_cell_x;
    int8_t best_cell_y;
    int8_t best_neighbor_cell_x;
    int8_t best_neighbor_cell_y;
    NavMapDirection best_exit_dir;
    uint16_t best_cost_to_start;
    uint16_t best_neighbor_manhattan;
    uint16_t best_score;

    uint16_t safe_return_cost;
    int32_t score_improvement;
    uint16_t score_margin;
    NavFrontierEvalDecision decision;
    NavFrontierEvalDecisionReason decision_reason;

    NavMapDirection entry_required_dir;
    NavFrontierEntryRelative entry_relative_from_current_dir;
    NavFrontierEntryAction entry_action_from_current_dir;
    bool entry_supported_from_current_dir;

    NavFrontierEntryRelative entry_relative_by_arrival_dir[4];
    NavFrontierEntryAction entry_action_by_arrival_dir[4];
    NavMapDirection preferred_arrival_dir;
    NavFrontierEntryAction preferred_entry_action;
    bool preferred_supported;
} NavFrontierEvalDebugSnapshot;

void nav_frontier_eval_clear(void);
NavFrontierEvalStatus nav_frontier_eval_evaluate(const NavFrontierEvalConfig *config);
void nav_frontier_eval_get_debug(NavFrontierEvalDebugSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif // NAV_FRONTIER_EVAL_H
