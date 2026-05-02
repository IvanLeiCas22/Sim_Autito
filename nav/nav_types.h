#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct NavSensorReadings {
    float front_mm;
    float left_mm;
    float right_mm;
} NavSensorReadings;

typedef struct NavRobotCommand {
    float linear_mm_s;
    float angular_rad_s;
} NavRobotCommand;

#ifdef __cplusplus
}
#endif

#endif // NAV_TYPES_H
