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
constexpr int kPivotLikeMinAbsPwm = 500;
constexpr double kPivotLikeMaxSumRatio = 0.35;
constexpr double kReferencePwm = 3000.0;
constexpr double kReferenceVelocityMmS = 200.0;
constexpr double kWheelBaseMm = 73.0;
constexpr double kAngularVelocityEpsilon = 1.0e-9;
}

SimRobot::SimRobot() = default;

void SimRobot::setPose(double x_mm, double y_mm, double yaw_deg)
{
    xMm_ = x_mm;
    yMm_ = y_mm;
    yawDeg_ = normalizeYawDeg(yaw_deg);
    yawRateDegS_ = 0.0;
    lastMotionWasPivotLike_ = false;
}

void SimRobot::moveForward(double distance_mm)
{
    const double yawRad = yawDeg_ * kDegToRad;
    xMm_ += std::cos(yawRad) * distance_mm;
    yMm_ += std::sin(yawRad) * distance_mm;
    yawRateDegS_ = 0.0;
    lastMotionWasPivotLike_ = false;
}

void SimRobot::rotate(double delta_yaw_deg)
{
    yawDeg_ = normalizeYawDeg(yawDeg_ + delta_yaw_deg);
    yawRateDegS_ = 0.0;
    lastMotionWasPivotLike_ = false;
}

void SimRobot::resetPose()
{
    setPose(kInitialXmm, kInitialYmm, kInitialYawDeg);
    yawRateDegS_ = 0.0;
}

void SimRobot::setMotorGains(double left_motor_gain, double right_motor_gain)
{
    leftMotorGain_ = left_motor_gain;
    rightMotorGain_ = right_motor_gain;
}

void SimRobot::setUsePivotCenterCorrection(bool enabled)
{
    usePivotCenterCorrection_ = enabled;
}

void SimRobot::setPivotCenterLocal(double x_mm, double y_mm)
{
    pivotCenterLocalXmm_ = x_mm;
    pivotCenterLocalYmm_ = y_mm;
}

void SimRobot::applyDifferentialDrive(int16_t left_motor_pwm,
                                      int16_t right_motor_pwm,
                                      double dt_s)
{
    const int leftPwm = std::clamp(static_cast<int>(left_motor_pwm), -kMaxMotorPwm, kMaxMotorPwm);
    const int rightPwm = std::clamp(static_cast<int>(right_motor_pwm), -kMaxMotorPwm, kMaxMotorPwm);
    const double effectiveLeftPwm = std::clamp(static_cast<double>(leftPwm) * leftMotorGain_,
                                               -static_cast<double>(kMaxMotorPwm),
                                               static_cast<double>(kMaxMotorPwm));
    const double effectiveRightPwm = std::clamp(static_cast<double>(rightPwm) * rightMotorGain_,
                                                -static_cast<double>(kMaxMotorPwm),
                                                static_cast<double>(kMaxMotorPwm));

    const double vLeft = effectiveLeftPwm / kReferencePwm * kReferenceVelocityMmS;
    const double vRight = effectiveRightPwm / kReferencePwm * kReferenceVelocityMmS;
    const double linearVelocityMmS = (vLeft + vRight) / 2.0;
    // Positive yaw in the simulator points toward +Y on screen, so the existing
    // control convention is left wheel faster => positive yaw.
    const double angularVelocityRadS = (vLeft - vRight) / kWheelBaseMm;
    const double yawRad = yawDeg_ * kDegToRad;
    const bool pivotLike = isPivotLikeCommand(leftPwm, rightPwm);
    lastMotionWasPivotLike_ = pivotLike;
    yawRateDegS_ = angularVelocityRadS * kRadToDeg;

    const double pivotLocalX = usePivotCenterCorrection_ ? pivotCenterLocalXmm_ : 0.0;
    const double pivotLocalY = usePivotCenterCorrection_ ? pivotCenterLocalYmm_ : 0.0;
    const double pivotGlobalX = xMm_
        + std::cos(yawRad) * pivotLocalX
        - std::sin(yawRad) * pivotLocalY;
    const double pivotGlobalY = yMm_
        + std::sin(yawRad) * pivotLocalX
        + std::cos(yawRad) * pivotLocalY;

    const double yawNewRad = yawRad + angularVelocityRadS * dt_s;
    double pivotNewX = pivotGlobalX;
    double pivotNewY = pivotGlobalY;

    if (std::abs(angularVelocityRadS) < kAngularVelocityEpsilon) {
        pivotNewX += std::cos(yawRad) * linearVelocityMmS * dt_s;
        pivotNewY += std::sin(yawRad) * linearVelocityMmS * dt_s;
    } else {
        const double arcRadiusMm = linearVelocityMmS / angularVelocityRadS;
        pivotNewX += arcRadiusMm * (std::sin(yawNewRad) - std::sin(yawRad));
        pivotNewY -= arcRadiusMm * (std::cos(yawNewRad) - std::cos(yawRad));
    }

    xMm_ = pivotNewX
        - std::cos(yawNewRad) * pivotLocalX
        + std::sin(yawNewRad) * pivotLocalY;
    yMm_ = pivotNewY
        - std::sin(yawNewRad) * pivotLocalX
        - std::cos(yawNewRad) * pivotLocalY;
    yawDeg_ = normalizeYawDeg(yawNewRad * kRadToDeg);
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

double SimRobot::leftMotorGain() const
{
    return leftMotorGain_;
}

double SimRobot::rightMotorGain() const
{
    return rightMotorGain_;
}

bool SimRobot::usePivotCenterCorrection() const
{
    return usePivotCenterCorrection_;
}

double SimRobot::pivotCenterLocalXmm() const
{
    return pivotCenterLocalXmm_;
}

double SimRobot::pivotCenterLocalYmm() const
{
    return pivotCenterLocalYmm_;
}

bool SimRobot::lastMotionWasPivotLike() const
{
    return lastMotionWasPivotLike_;
}

double SimRobot::normalizeYawDeg(double yaw_deg)
{
    double normalized = std::fmod(yaw_deg, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }

    return normalized;
}

bool SimRobot::isPivotLikeCommand(int left_pwm, int right_pwm)
{
    if (left_pwm == 0 || right_pwm == 0) {
        return false;
    }
    if ((left_pwm > 0) == (right_pwm > 0)) {
        return false;
    }

    const int absLeft = std::abs(left_pwm);
    const int absRight = std::abs(right_pwm);
    if (absLeft < kPivotLikeMinAbsPwm || absRight < kPivotLikeMinAbsPwm) {
        return false;
    }

    const int totalAbs = absLeft + absRight;
    const int absSum = std::abs(left_pwm + right_pwm);
    return static_cast<double>(absSum) <= kPivotLikeMaxSumRatio * static_cast<double>(totalAbs);
}
