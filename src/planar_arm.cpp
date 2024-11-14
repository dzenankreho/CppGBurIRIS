#include "planar_arm.hpp"



GBurIRIS::robots::PlanarArm::PlanarArm(
    const drake::planning::CollisionChecker& collisionChecker,
    const std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>>& jointChildAndEndEffectorLinks,
    const std::vector<double>& linkGeometryCompensation
) : Robot{collisionChecker, jointChildAndEndEffectorLinks, linkGeometryCompensation} { }



std::vector<Eigen::Vector2d> GBurIRIS::robots::PlanarArm::getLinkPositions(const Eigen::VectorXd& qk) const {
    auto&& plant{ collisionChecker.plant() };
    auto&& plantContext{ collisionChecker.plant_context() };

    auto&& previousConfig{ plant.GetPositions(plantContext) };

    std::vector<Eigen::Vector2d> radii;

    auto&& newContext{ collisionChecker.UpdatePositions(qk) };

    for (auto&& linkBody : jointChildAndEndEffectorLinks) {
        radii.push_back(plant.EvalBodyPoseInWorld(newContext, linkBody).translation()(Eigen::seq(0, 1)));
    }

    collisionChecker.UpdatePositions(previousConfig);

    return radii;
}
