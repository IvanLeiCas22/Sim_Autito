#ifndef NAV_CORE_H
#define NAV_CORE_H

#include "nav_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void nav_core_init(void);
NavRobotCommand nav_core_update(const NavSensorReadings *sensors);

#ifdef __cplusplus
}
#endif

#endif // NAV_CORE_H
