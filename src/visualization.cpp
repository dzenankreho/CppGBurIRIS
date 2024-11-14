#include "visualization.hpp" 

#include <vector>
#include <cstdlib>

#include "matplotlibcpp.h"


visualization::Figure::Figure() : figureNumber{ matplotlibcpp::figure() } { }


void visualization::Figure::visualize2dConfigurationSpace(
    const drake::planning::CollisionChecker& collisionChecker,
    const int numOfSamples,
    const std::optional<std::tuple<double, double, double, double>>& plotColor
) const {

    const drake::multibody::MultibodyPlant<double>& plant{ collisionChecker.plant() };

    auto qLowerBounds{ plant.GetPositionLowerLimits() };
    auto qUpperBounds{ plant.GetPositionUpperLimits() };

    std::vector<float> configurationSpace;

    double delta0{(qUpperBounds(0) - qLowerBounds(0)) / (numOfSamples - 1)},
        delta1{(qUpperBounds(1) - qLowerBounds(1)) / (numOfSamples - 1)};


    std::vector<double> plotColorVec;
    if (plotColor) {
        auto& [r, g, b, a] = *plotColor;
        plotColorVec = {r, g, b, a};
    } else {
        plotColorVec = {0, 0, 0, 1};
    }

    for (int i{ numOfSamples - 1 }; i >= 0; --i) {
        for (int j{}; j < numOfSamples; ++j) {
            for (auto&& rgba : plotColorVec) {
                configurationSpace.push_back(rgba * !collisionChecker.CheckConfigCollisionFree(
                    Eigen::Vector2d(qLowerBounds(1) + delta1 * j,
                                    qLowerBounds(0) + delta0 * i)
                ));
            }
        }
    }

    matplotlibcpp::figure(figureNumber);
    matplotlibcpp::imshow(configurationSpace.data(), numOfSamples, numOfSamples, 4);
}


void visualization::Figure::visualize2dConvexSet(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::geometry::optimization::ConvexSet& set,
    const int numOfSamples,
    const std::optional<std::tuple<double, double, double, double>>& plotColor
) const {

    auto qLowerBounds{ plant.GetPositionLowerLimits() };
    auto qUpperBounds{ plant.GetPositionUpperLimits() };

    std::vector<float> polytope;

    double delta0{(qUpperBounds(0) - qLowerBounds(0)) / (numOfSamples - 1)},
        delta1{(qUpperBounds(1) - qLowerBounds(1)) / (numOfSamples - 1)};


    std::vector<double> plotColorVec;
    if (plotColor) {
        auto& [r, g, b, a] = *plotColor;
        plotColorVec = {r, g, b, a};
    } else {
        plotColorVec = {
            double(std::rand()) / RAND_MAX,
            double(std::rand()) / RAND_MAX,
            double(std::rand()) / RAND_MAX,
            0.5
        };
    }

    for (int i{ numOfSamples - 1 }; i >= 0; --i) {
        for (int j{}; j < numOfSamples; ++j) {
            for (auto&& rgba : plotColorVec) {
                polytope.push_back(rgba * set.PointInSet(
                    Eigen::Vector2d(qLowerBounds(1) + delta1 * j,
                                    qLowerBounds(0) + delta0 * i)
                ));
            }
        }
    }

    matplotlibcpp::figure(figureNumber);
    matplotlibcpp::imshow(polytope.data(), numOfSamples, numOfSamples, 4);
}


void visualization::Figure::showFigures() {
    matplotlibcpp::show();
}
