#include "sim_robot.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kInitialXmm = 100.0;
constexpr double kInitialYmm = 100.0;
constexpr double kInitialYawDeg = 0.0;
constexpr int kMaxMotorPwm = 9999;
constexpr double kReferencePwm = 3000.0;
constexpr double kReferenceVelocityMmS = 200.0;
constexpr double kWheelBaseMm = 73.0;
}

SimRobot::SimRobot() = default;

void SimRobot::setPose(double x_mm, double y_mm, double yaw_deg)
{
    xMm_ = x_mm;
    yMm_ = y_mm;
    yawDeg_ = normalizeYawDeg(yaw_deg);
    yawRateDegS_ = 0.0;
}

void SimRobot::moveForward(double distance_mm)
{
    const double yawRad = yawDeg_ * kDegToRad;
    xMm_ += std::cos(yawRad) * distance_mm;
    yMm_ += std::sin(yawRad) * distance_mm;
    yawRateDegS_ = 0.0;
}

void SimRobot::rotate(double delta_yaw_deg)
{
    yawDeg_ = normalizeYawDeg(yawDeg_ + delta_yaw_deg);
    yawRateDegS_ = 0.0;
}

void SimRobot::resetPose()
{
    setPose(kInitialXmm, kInitialYmm, kInitialYawDeg);
    yawRateDegS_ = 0.0;
}

void SimRobot::applyDifferentialDrive(int16_t left_motor_pwm,
                                      int16_t right_motor_pwm,
                                      double dt_s)
{
    const int leftPwm = std::clamp(static_cast<int>(left_motor_pwm), -kMaxMotorPwm, kMaxMotorPwm);
    const int rightPwm = std::clamp(static_cast<int>(right_motor_pwm), -kMaxMotorPwm, kMaxMotorPwm);

    const double vLeft = static_cast<double>(leftPwm) / kReferencePwm * kReferenceVelocityMmS;
    const double vRight = static_cast<double>(rightPwm) / kReferencePwm * kReferenceVelocityMmS;
    const double linearVelocityMmS = (vLeft + vRight) / 2.0;
    const double angularVelocityRadS = (vLeft - vRight) / kWheelBaseMm;
    const double yawRad = yawDeg_ * kDegToRad;
    yawRateDegS_ = angularVelocityRadS * kRadToDeg;

    xMm_ += std::cos(yawRad) * linearVelocityMmS * dt_s;
    yMm_ += std::sin(yawRad) * linearVelocityMmS * dt_s;
    yawDeg_ = normalizeYawDeg(yawDeg_ + yawRateDegS_ * dt_s);
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

double SimRobot::yawRateDegS() const
{
    return yawRateDegS_;
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
