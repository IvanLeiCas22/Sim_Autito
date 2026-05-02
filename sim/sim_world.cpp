#include "sim_world.h"

SimWorld::SimWorld() = default;

const SimRobot &SimWorld::robot() const
{
    return robot_;
}
