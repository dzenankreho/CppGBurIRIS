#pragma once

#include "robot.hpp"

namespace GBurIRIS {
namespace robots {

    class PlanarArm final : public Robot {

    public:
        PlanarArm(
            const drake::planning::CollisionChecker& collisionChecker,
            const std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>>& jointChildAndEndEffectorLinks,
            const std::vector<double>& linkGeometryCompensation
        );
        std::vector<Eigen::Vector2d> getLinkPositions(const Eigen::VectorXd& qk) override;
        std::vector<double> getEnclosingRadii(const Eigen::VectorXd& qk) override;
        double getMaxDisplacement(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) override;
    };

}
}
