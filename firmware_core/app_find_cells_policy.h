#ifndef INC_APP_FIND_CELLS_POLICY_H_
#define INC_APP_FIND_CELLS_POLICY_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_maze_types.h"
#include "app_nav_types.h"

/*
 * FIND_CELLS high-level decision policy.
 *
 * This module is production navigation logic, not shadow/debug code.
 *
 * Current implementation stage:
 * - prefer immediate unvisited neighbors in relative priority order:
 *   front -> right -> left;
 * - if no immediate unvisited neighbor is available, return false so the
 *   supervisor can fall back to the current local App_Nav_RecommendAction().
 *
 * Future stages:
 * - route to the nearest exploration frontier using flood fill/BFS;
 * - support route backtracking when the best next step is behind the robot.
 */

typedef enum
{
    APP_FIND_CELLS_DECISION_REASON_NONE = 0,
    APP_FIND_CELLS_DECISION_REASON_IMMEDIATE_UNVISITED
} AppFindCellsDecisionReason;

typedef struct
{
    AppNavRecommendedAction action;
    HeadingTypeDef desired_dir;
    uint8_t target_x;
    uint8_t target_y;
    AppFindCellsDecisionReason reason;
} AppFindCellsDecision;

/*
 * Evaluate the current FIND_CELLS decision.
 *
 * Returns true only when this policy found a concrete action to execute.
 * Returns false when the supervisor should use the existing local fallback.
 */
bool App_FindCellsPolicy_Evaluate(AppFindCellsDecision *decision_out);

#endif /* INC_APP_FIND_CELLS_POLICY_H_ */
