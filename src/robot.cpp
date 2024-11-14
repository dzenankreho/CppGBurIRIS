#include "robot.hpp"

#include <stdexcept>

GBurIRIS::robots::Robot::Robot(
    const drake::planning::CollisionChecker& collisionChecker,
    const std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>>& jointChildAndEndEffectorLinks,
    const std::vector<double>& linkGeometryCompensation
) : collisionChecker{ collisionChecker },
    jointChildAndEndEffectorLinks{ jointChildAndEndEffectorLinks },
    linkGeometryCompensation{ linkGeometryCompensation } {

    if (jointChildAndEndEffectorLinks.size() != linkGeometryCompensation.size() + 1) {
        throw std::invalid_argument("Invalid vector sizes!");
    }
}
