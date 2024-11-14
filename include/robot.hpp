#pragma once

#include <vector>
#include <functional>

#include <drake/planning/collision_checker.h>
#include <drake/multibody/tree/rigid_body.h>

#include <Eigen/Dense>

namespace GBurIRIS {
namespace robots {

    enum class LinkGeometryCompensationType { positive, negative };

    class Robot {

    protected:
        const drake::planning::CollisionChecker& collisionChecker;
        const std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>>& jointChildAndEndEffectorLinks;
        const std::vector<double> linkGeometryCompensation;

    public:
        Robot(
            const drake::planning::CollisionChecker& collisionChecker,
            const std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>>& jointChildAndEndEffectorLinks,
            const std::vector<double>& linkGeometryCompensation
        );

        virtual ~Robot() = default;

        virtual std::vector<Eigen::Vector2d> getLinkPositions(const Eigen::VectorXd& qk) const = 0;

//         virtual std::vector<double> getEnclosingRadii(const Eigen::VectorXd& qk) const = 0;
//
//         virtual double getMaxDisplacement(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) const = 0;
//
//         virtual std::vector<double> compensateForLinkGeometry(
//             const std::vector<double>& distances,
//             LinkGeometryCompensationType linkGeometryCompensationType
//         ) const = 0;
//
//         virtual double compensateForLinkGeometry(
//             int linkNumber,
//             double distances,
//             LinkGeometryCompensationType linkGeometryCompensationType
//         ) const = 0;
    };

}
}
