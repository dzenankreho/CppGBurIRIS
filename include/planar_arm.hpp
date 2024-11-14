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

        std::vector<Eigen::Vector2d> getLinkPositions(const Eigen::VectorXd& qk) const override;

//         std::vector<double> getEnclosingRadii(const Eigen::VectorXd& qk) const override;
//
//         double getMaxDisplacement(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) const override;
//
//         std::vector<double> compensateForLinkGeometry(
//             const std::vector<double>& distances,
//             LinkGeometryCompensationType linkGeometryCompensationType
//         ) const override;
//
//         double compensateForLinkGeometry(
//             int linkNumber,
//             double distances,
//             LinkGeometryCompensationType linkGeometryCompensationType
//         ) const override;
    };

}
}
