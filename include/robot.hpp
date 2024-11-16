#pragma once

#include <vector>
#include <functional>

#include <drake/systems/framework/context.h>
#include <drake/planning/collision_checker.h>
#include <drake/multibody/tree/rigid_body.h>

#include <Eigen/Dense>

namespace GBurIRIS::robots {

    enum class LinkGeometryCompensationType { positive, negative };

    class Robot {

    public:
        Robot(
            const drake::planning::CollisionChecker& collisionChecker,
            const std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>>& jointChildAndEndEffectorLinks,
            const std::vector<double>& linkGeometryCompensation
        );
        virtual ~Robot() = default;
        virtual std::vector<Eigen::Vector2d> getLinkPositions(const Eigen::VectorXd& qk) = 0;
        virtual std::vector<double> getEnclosingRadii(const Eigen::VectorXd& qk) = 0;
        virtual double getMaxDisplacement(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) = 0;

        Eigen::VectorXd getCurrentConfiguration() const;
        void setConfiguration(const Eigen::VectorXd& q);
        const drake::planning::CollisionChecker& getCollisionChecker() const;
        const drake::multibody::MultibodyPlant<double>& getPlant() const;
        const std::reference_wrapper<const drake::systems::Context<double>>& getPlantContext() const;
        const std::vector<double>& getLinkGeometryCompensation() const;

    protected:
        const std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>>& jointChildAndEndEffectorLinks;
        const std::vector<double> linkGeometryCompensation;
        const drake::planning::CollisionChecker& collisionChecker;
        const drake::multibody::MultibodyPlant<double>& plant;
        std::reference_wrapper<const drake::systems::Context<double>> plantContext;
    };


    inline Eigen::VectorXd Robot::getCurrentConfiguration() const {
        return plant.GetPositions(plantContext);
    }


    inline void Robot::setConfiguration(const Eigen::VectorXd& q) {
        plantContext = collisionChecker.UpdatePositions(q);
    }

    inline const drake::planning::CollisionChecker& Robot::getCollisionChecker() const {
        return collisionChecker;
    }

    inline const drake::multibody::MultibodyPlant<double>& Robot::getPlant() const {
        return plant;
    }

    inline const std::reference_wrapper<const drake::systems::Context<double>>& Robot::getPlantContext() const {
        return plantContext;
    }

    inline const std::vector<double>& Robot::getLinkGeometryCompensation() const {
        return linkGeometryCompensation;
    }

}
