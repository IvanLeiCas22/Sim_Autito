#ifndef SIM_WORLD_H
#define SIM_WORLD_H

#include "sim_robot.h"

class SimWorld
{
public:
    SimWorld();

    const SimRobot &robot() const;

private:
    SimRobot robot_;

    // TODO: Add maze loading and collision geometry in a later step.
};

#endif // SIM_WORLD_H
