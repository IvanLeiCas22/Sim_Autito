#ifndef SIM_ROBOT_H
#define SIM_ROBOT_H

class SimRobot
{
public:
    SimRobot();

    double xMm() const;
    double yMm() const;
    double headingRad() const;

private:
    double xMm_ = 0.0;
    double yMm_ = 0.0;
    double headingRad_ = 0.0;

    // TODO: Add robot dimensions, kinematics, sensors, and controller state later.
};

#endif // SIM_ROBOT_H
