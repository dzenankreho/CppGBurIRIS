#pragma once

#include "robot.hpp"

#include <Eigen/Dense>

#include <functional>
#include <optional>
#include <vector>
#include <tuple>


#include <iostream>

namespace GBurIRIS::GBur {

    struct GeneralizedBurConfig {
        int numOfSpines{ 7 };
        int burOrder{ 3 };
        double minDistanceTol{ 1e-5 };
        double phiTol{ 0.1 };
    };


    class GeneralizedBur {

    public:
        GeneralizedBur(
            const Eigen::VectorXd& qCenter,
            const GeneralizedBurConfig& generalizedBurConfig,
            robots::Robot& robot,
            const std::vector<Eigen::VectorXd>& randomConfigs
        );
        GeneralizedBur(
            const Eigen::VectorXd& qCenter,
            const GeneralizedBurConfig& generalizedBurConfig,
            robots::Robot& robot,
            const std::function<Eigen::VectorXd ()>& randomConfigGenerator
        );
        GeneralizedBur(
            const Eigen::VectorXd& qCenter,
            const GeneralizedBurConfig& generalizedBurConfig,
            robots::Robot& robot,
            const Eigen::MatrixXd& rotationMatrix
        );
        double getMinDistanceToCollision();
        std::tuple<std::vector<Eigen::VectorXd>, std::vector<std::vector<Eigen::VectorXd>>> calculateBur();
        GeneralizedBurConfig getGeneralizedBurConfig() const;
        std::vector<std::vector<Eigen::VectorXd>> getLayers() const;
        void setRandomConfigs(const std::vector<Eigen::VectorXd>& randomConfigs);
        Eigen::VectorXd getCenter() const;

    private:
        const Eigen::VectorXd qCenter;
        const GeneralizedBurConfig generalizedBurConfig;
        robots::Robot& robot;
        std::optional<std::function<Eigen::VectorXd ()>> randomConfigGenerator{ std::nullopt };
        std::optional<Eigen::MatrixXd> rotationMatrix{ std::nullopt };
        std::optional<std::vector<Eigen::VectorXd>> randomConfigs{ std::nullopt };
        std::optional<double> minDistance{ std::nullopt };
        std::optional<std::vector<
            std::tuple<drake::multibody::BodyIndex, drake::multibody::BodyIndex, Eigen::Vector3d, Eigen::Vector3d, double>
        >> linkObstacleDistancePairs{ std::nullopt };
        std::optional<std::vector<Eigen::Vector4d>> linkObstaclePlanes{ std::nullopt };
        std::vector<std::vector<Eigen::VectorXd>> layers;

        void approximateObstaclesWithPlanes();
        double getMinDistanceUnderestimation(const Eigen::VectorXd& q);
        double evaluatePhiFunction(
            double distance,
            double t,
            const Eigen::VectorXd& qe,
            const Eigen::VectorXd& startingPoint
        );
    };

    inline double GeneralizedBur::evaluatePhiFunction(
        double distance,
        double t,
        const Eigen::VectorXd& qe,
        const Eigen::VectorXd& startingPoint
    ) {
        return distance - robot.getMaxDisplacement(startingPoint, startingPoint + t * (qe - startingPoint));
    }


    inline GeneralizedBurConfig GeneralizedBur::getGeneralizedBurConfig() const {
        return generalizedBurConfig;
    }

    inline std::vector<std::vector<Eigen::VectorXd>> GeneralizedBur::getLayers() const {
        return layers;
    }

    inline void GeneralizedBur::setRandomConfigs(const std::vector<Eigen::VectorXd>& randomConfigs) {
        this->randomConfigs = randomConfigs;
    }

    inline Eigen::VectorXd GeneralizedBur::getCenter() const {
        return qCenter;
    }
}
