#include "anthropomorphic_arm.hpp"

#include <algorithm>
#include <iostream>



GBurIRIS::robots::AnthropomorphicArm::AnthropomorphicArm(
    const drake::planning::CollisionChecker& collisionChecker,
    const std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>>& jointChildAndEndEffectorLinks,
    const std::vector<double>& linkGeometryCompensation
) : Robot{collisionChecker, jointChildAndEndEffectorLinks, linkGeometryCompensation} { }



std::vector<Eigen::VectorXd> GBurIRIS::robots::AnthropomorphicArm::getLinkPositions(const Eigen::VectorXd& qk) {
    auto&& previousConfig{ getCurrentConfiguration() };

    std::vector<Eigen::VectorXd> linkPositions;

    setConfiguration(qk);

    for (auto&& linkBody : jointChildAndEndEffectorLinks) {
        linkPositions.push_back(plant.EvalBodyPoseInWorld(plantContext, linkBody).translation());
    }

    setConfiguration(previousConfig);

    return linkPositions;
}



std::vector<double> GBurIRIS::robots::AnthropomorphicArm::getEnclosingRadii(const Eigen::VectorXd& qk) {
    auto&& linkPositions{ getLinkPositions(qk) };

    std::vector<double> radii(linkPositions.size() - 1, 0);

    for (int i{ 1 }; i < linkPositions.size(); ++i) {
        radii.at(0) = std::max(
            radii.at(0),
            (linkPositions.at(0)(Eigen::seq(0, 1)) - linkPositions.at(i)(Eigen::seq(0, 1))).norm() +
                std::max(
                    linkGeometryCompensation.at(i - 1),
                    linkGeometryCompensation.at(((i < linkGeometryCompensation.size()) ? (i) : (i - 1)))
                )
        );
    }

    for (int i{ 1 }; i + 1 < linkPositions.size(); ++i) {
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


double GBurIRIS::robots::AnthropomorphicArm::getMaxDisplacement(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) {
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


