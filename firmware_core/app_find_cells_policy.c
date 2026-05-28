#include "app_find_cells_policy.h"

#include "app_maze.h"
#include <stddef.h>

#define APP_FIND_CELLS_INVALID_COORD 0xFFU

typedef struct
{
    HeadingTypeDef dir;
    AppNavRecommendedAction action;
} AppFindCellsCandidate;

static HeadingTypeDef App_FindCellsPolicy_RotateRight(HeadingTypeDef heading)
{
    return (HeadingTypeDef)((heading + 1) % 4);
}

static HeadingTypeDef App_FindCellsPolicy_RotateLeft(HeadingTypeDef heading)
{
    return (HeadingTypeDef)((heading + 3) % 4);
}

static void App_FindCellsPolicy_ClearDecision(AppFindCellsDecision *decision)
{
    if (decision == NULL)
    {
        return;
    }

    decision->action = APP_NAV_ACTION_NONE;
    decision->desired_dir = HEADING_NORTH;
    decision->target_x = APP_FIND_CELLS_INVALID_COORD;
    decision->target_y = APP_FIND_CELLS_INVALID_COORD;
    decision->reason = APP_FIND_CELLS_DECISION_REASON_NONE;
}

static bool App_FindCellsPolicy_IsReachableUnvisitedNeighbor(uint8_t x,
                                                             uint8_t y,
                                                             HeadingTypeDef dir,
                                                             uint8_t *neighbor_x,
                                                             uint8_t *neighbor_y)
{
    uint8_t nx = 0U;
    uint8_t ny = 0U;

    if ((neighbor_x == NULL) || (neighbor_y == NULL))
    {
        return false;
    }

    if (!App_Maze_IsKnownOpenEdge(x, y, dir))
    {
        return false;
    }

    if (!App_Maze_GetNeighbor(x, y, dir, &nx, &ny))
    {
        return false;
    }

    if (App_Maze_IsCellVisited(nx, ny))
    {
        return false;
    }

    *neighbor_x = nx;
    *neighbor_y = ny;
    return true;
}

bool App_FindCellsPolicy_Evaluate(AppFindCellsDecision *decision_out)
{
    uint8_t x = 0U;
    uint8_t y = 0U;
    HeadingTypeDef heading = HEADING_NORTH;

    if (decision_out == NULL)
    {
        return false;
    }

    App_FindCellsPolicy_ClearDecision(decision_out);

    if (!App_Maze_GetRobotPose(&x, &y, &heading))
    {
        return false;
    }

    /*
     * Tie-break priority intentionally matches the current local navigation
     * style: front -> right -> left.
     *
     * Back is not considered in this stage because route backtracking in an
     * open cell still needs the future CENTER_BY_FRONT_TAPE_FOR_PIVOT primitive.
     */
    const AppFindCellsCandidate candidates[3] =
    {
        {heading, APP_NAV_ACTION_GO_FRONT_NAVIGATING},
        {App_FindCellsPolicy_RotateRight(heading), APP_NAV_ACTION_SMOOTH_RIGHT},
        {App_FindCellsPolicy_RotateLeft(heading), APP_NAV_ACTION_SMOOTH_LEFT}
    };

    for (uint8_t i = 0U; i < 3U; i++)
    {
        uint8_t target_x = 0U;
        uint8_t target_y = 0U;

        if (App_FindCellsPolicy_IsReachableUnvisitedNeighbor(x,
                                                             y,
                                                             candidates[i].dir,
                                                             &target_x,
                                                             &target_y))
        {
            decision_out->action = candidates[i].action;
            decision_out->desired_dir = candidates[i].dir;
            decision_out->target_x = target_x;
            decision_out->target_y = target_y;
            decision_out->reason = APP_FIND_CELLS_DECISION_REASON_IMMEDIATE_UNVISITED;
            return true;
        }
    }

    return false;
}
