#include "sim_robot.h"

SimRobot::SimRobot() = default;

double SimRobot::xMm() const
{
    return xMm_;
}

double SimRobot::yMm() const
{
    return yMm_;
}

double SimRobot::headingRad() const
{
    return headingRad_;
}
