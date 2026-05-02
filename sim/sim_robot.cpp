#include "sim_robot.h"

#include <cmath>

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kInitialXmm = 100.0;
constexpr double kInitialYmm = 100.0;
constexpr double kInitialYawDeg = 0.0;
}

SimRobot::SimRobot() = default;

void SimRobot::setPose(double x_mm, double y_mm, double yaw_deg)
{
    xMm_ = x_mm;
    yMm_ = y_mm;
    yawDeg_ = normalizeYawDeg(yaw_deg);
}

void SimRobot::moveForward(double distance_mm)
{
    const double yawRad = yawDeg_ * kDegToRad;
    xMm_ += std::cos(yawRad) * distance_mm;
    yMm_ += std::sin(yawRad) * distance_mm;
}

void SimRobot::rotate(double delta_yaw_deg)
{
    yawDeg_ = normalizeYawDeg(yawDeg_ + delta_yaw_deg);
}

void SimRobot::resetPose()
{
    setPose(kInitialXmm, kInitialYmm, kInitialYawDeg);
}

double SimRobot::xMm() const
{
    return xMm_;
}

double SimRobot::yMm() const
{
    return yMm_;
}

double SimRobot::yawDeg() const
{
    return yawDeg_;
}

double SimRobot::lengthMm() const
{
    return lengthMm_;
}

double SimRobot::widthMm() const
{
    return widthMm_;
}

double SimRobot::normalizeYawDeg(double yaw_deg)
{
    double normalized = std::fmod(yaw_deg, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }

    return normalized;
}
