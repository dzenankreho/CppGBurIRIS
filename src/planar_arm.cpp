#include "planar_arm.hpp"

#include <algorithm>
#include <iostream>



GBurIRIS::robots::PlanarArm::PlanarArm(
    const drake::planning::CollisionChecker& collisionChecker,
    const std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>>& jointChildAndEndEffectorLinks,
    const std::vector<double>& linkGeometryCompensation
) : Robot{collisionChecker, jointChildAndEndEffectorLinks, linkGeometryCompensation} { }



std::vector<Eigen::VectorXd> GBurIRIS::robots::PlanarArm::getLinkPositions(const Eigen::VectorXd& qk) {
    auto&& previousConfig{ getCurrentConfiguration() };

    std::vector<Eigen::VectorXd> linkPositions;

    setConfiguration(qk);

    for (const auto& linkBody : jointChildAndEndEffectorLinks) {
        linkPositions.push_back(plant.EvalBodyPoseInWorld(plantContext, linkBody).translation()(Eigen::seq(0, 1)));
    }

    setConfiguration(previousConfig);

    return linkPositions;
}



std::vector<double> GBurIRIS::robots::PlanarArm::getEnclosingRadii(const Eigen::VectorXd& qk) {
    auto&& linkPositions{ getLinkPositions(qk) };

    std::vector<double> radii(linkPositions.size() - 1, 0);

    for (int i{}; i + 1 < linkPositions.size(); ++i) {
        for (int j{ i + 1 }; j < linkPositions.size(); ++j) {
            radii.at(i) = std::max(
                radii.at(i),
                (linkPositions.at(i) - linkPositions.at(j)).norm() +
                    std::max(
                        linkGeometryCompensation.at(j - 1),
                        linkGeometryCompensation.at(((j < linkGeometryCompensation.size()) ? (j) : (j - 1)))
                    )
            );
        }
    }


    return radii;
}



double GBurIRIS::robots::PlanarArm::getMaxDisplacement(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) {
    auto&& linkPositions1{ getLinkPositions(q1) }, linkPositions2{ getLinkPositions(q2) };

    double maxDisplacement{};

    for (int i{ 1 }; i < linkPositions1.size(); ++i) {
        maxDisplacement = std::max(
            maxDisplacement,
            (linkPositions1.at(i) - linkPositions2.at(i)).norm()
        );
    }

    return maxDisplacement;
}


