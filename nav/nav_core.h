#ifndef NAV_CORE_H
#define NAV_CORE_H

#include "nav_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum NavState {
    NAV_STATE_IDLE = 0,
    NAV_STATE_ADVANCING_UNTIL_REAR_BLACK,
    NAV_STATE_SMOOTH_TURNING,
    NAV_STATE_DONE
} NavState;

typedef enum NavAction {
    NAV_ACTION_NONE = 0,
    NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK,
    NAV_ACTION_SMOOTH_TURN_LEFT,
    NAV_ACTION_SMOOTH_TURN_RIGHT,
    NAV_ACTION_PIVOT_TURN_LEFT,
    NAV_ACTION_PIVOT_TURN_RIGHT,
    NAV_ACTION_PIVOT_TURN_180
} NavAction;

void nav_core_init(void);
void nav_core_start_advance_until_rear_black(void);
void nav_core_start_smooth_turn_left(const RobotSensors *sensors);
void nav_core_start_smooth_turn_right(const RobotSensors *sensors);
void nav_core_stop(void);
NavState nav_core_state(void);
NavAction nav_core_action(void);
q16_16_t nav_core_action_start_yaw_q16(void);
q16_16_t nav_core_action_target_yaw_q16(void);
RobotCommand nav_core_update(const RobotSensors *sensors);

#ifdef __cplusplus
}
#endif

#endif // NAV_CORE_H
