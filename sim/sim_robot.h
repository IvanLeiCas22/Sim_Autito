#ifndef SIM_ROBOT_H
#define SIM_ROBOT_H

#include <cstdint>

class SimRobot
{
public:
    SimRobot();

    void setPose(double x_mm, double y_mm, double yaw_deg);
    void moveForward(double distance_mm);
    void rotate(double delta_yaw_deg);
    void resetPose();
    void applyDifferentialDrive(int16_t left_motor_pwm,
                                int16_t right_motor_pwm,
                                double dt_s);
    double xMm() const;
    double yMm() const;
    double yawDeg() const;
    double yawRateDegS() const;
    double lengthMm() const;
    double widthMm() const;

private:
    double xMm_ = 100.0;
    double yMm_ = 100.0;
    double yawDeg_ = 0.0;
    double yawRateDegS_ = 0.0;
    double lengthMm_ = 90.0;
    double widthMm_ = 80.0;

    static double normalizeYawDeg(double yaw_deg);

    // TODO: Add robot dimensions, kinematics, sensors, and controller state later.
};

#endif // SIM_ROBOT_H
