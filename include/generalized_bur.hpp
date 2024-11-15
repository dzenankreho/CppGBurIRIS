#pragma once

#include "robot.hpp"

#include <Eigen/Dense>

#include <functional>
#include <optional>
#include <vector>

namespace GBurIRIS {
namespace GBur {

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
            const robots::Robot& robot,
            const std::function<Eigen::VectorXd ()>& randomConfigGenerator
        );
        double getMinDistanceToCollision();
        void calculateBur();

    private:
        const Eigen::VectorXd& qCenter;
        const GeneralizedBurConfig& generalizedBurConfig;
        const robots::Robot& robot;
        const std::function<Eigen::VectorXd ()>& randomConfigGenerator;
        std::optional<std::vector<Eigen::VectorXd>> randomConfigs{ std::nullopt };
        std::optional<double> minDistance{ std::nullopt };

        void approximateObstaclesWithPlanes();
        double getMinDistanceUnderestimation(const Eigen::VectorXd& q);
        double evaluatePhiFunction(
            double distance,
            double t,
            const Eigen::VectorXd& qe,
            const Eigen::VectorXd& startingPoint
        );
    };

}
}
