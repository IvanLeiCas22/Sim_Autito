#include "nav_core.h"

void nav_core_init(void)
{
    // TODO: Initialize navigation state in a later step.
}

NavRobotCommand nav_core_update(const NavSensorReadings *sensors)
{
    (void)sensors;

    // TODO: Replace with navigation logic once sensors and maze state exist.
    NavRobotCommand command = {0.0f, 0.0f};
    return command;
}
