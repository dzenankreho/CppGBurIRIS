#include "visualization.hpp" 

#include <vector>
#include <cstdlib>
#include <sstream>
#include <ios>
#include <iomanip>

#include <iostream>


#include "matplotlibcpp.h"


GBurIRIS::visualization::Figure::Figure() : figureNumber{ matplotlibcpp::figure() } { }


void GBurIRIS::visualization::Figure::visualize2dConfigurationSpace(
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


void GBurIRIS::visualization::Figure::visualize2dConvexSet(
    const drake::planning::CollisionChecker& collisionChecker,
    const drake::geometry::optimization::ConvexSet& set,
    const int numOfSamples,
    const std::optional<std::tuple<double, double, double, double>>& plotColor
) const {

    auto qLowerBounds{ collisionChecker.plant().GetPositionLowerLimits() };
    auto qUpperBounds{ collisionChecker.plant().GetPositionUpperLimits() };

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


void GBurIRIS::visualization::Figure::showFigures() {
    matplotlibcpp::show();
}


void GBurIRIS::visualization::Figure::visualize2dGeneralizedBur(
    const drake::planning::CollisionChecker& collisionChecker,
    const GBur::GeneralizedBur& gBur,
    const int numOfSamples,
    std::optional<std::vector<std::tuple<double, double, double, double>>> plotColor
) const {

    matplotlibcpp::figure(figureNumber);

    auto&& qLowerBounds{ collisionChecker.plant().GetPositionLowerLimits() };
    auto&& qUpperBounds{ collisionChecker.plant().GetPositionUpperLimits() };

    GBur::GeneralizedBurConfig generalizedBurConfig{ gBur.getGeneralizedBurConfig() };
    auto&& layers{ gBur.getLayers() };


    if (!plotColor) {
        plotColor = std::vector<std::tuple<double, double, double, double>>(
            generalizedBurConfig.burOrder + 1,
            {1, 0, 0, 1}
        );
    }

    std::optional<std::vector<std::vector<double>>> prevBurLayerInPixels{ std::nullopt };

    for (int j{ generalizedBurConfig.burOrder }; j >= 0; --j) {
        std::vector<std::vector<double>> burLayerInPixels(
            generalizedBurConfig.numOfSpines, std::vector<double>(2)
        );
        for (int i{}; i < generalizedBurConfig.numOfSpines; ++i) {
            burLayerInPixels.at(i) = {
                (layers.at(i).at(j)(0) - qLowerBounds(0)) / (qUpperBounds(0) - qLowerBounds(0)) * numOfSamples,
                (layers.at(i).at(j)(1) - qUpperBounds(1)) / (qLowerBounds(1) - qUpperBounds(1)) * numOfSamples
            };
        }


        if (prevBurLayerInPixels) {
            for (int i{}; i < generalizedBurConfig.numOfSpines; ++i) {
                matplotlibcpp::plot(
                    { prevBurLayerInPixels->at(i).at(0), burLayerInPixels.at(i).at(0) },
                    { prevBurLayerInPixels->at(i).at(1), burLayerInPixels.at(i).at(1) },
                    {
                        {"color", rgbFloatToString(std::get<0>(*(plotColor->rbegin() + j)),
                                                   std::get<1>(*(plotColor->rbegin() + j)),
                                                   std::get<2>(*(plotColor->rbegin() + j)))},
                        {"alpha", std::to_string(std::get<3>(*(plotColor->rbegin() + j)))}
                    }
                );
            }
        }

        for (int i{}; i < generalizedBurConfig.numOfSpines; ++i) {
            matplotlibcpp::plot(
                { burLayerInPixels.at(i).at(0) },
                { burLayerInPixels.at(i).at(1) },
                {
                    {"color", rgbFloatToString(std::get<0>(*(plotColor->rbegin() + j)),
                                                std::get<1>(*(plotColor->rbegin() + j)),
                                                std::get<2>(*(plotColor->rbegin() + j)))},
                    {"alpha", std::to_string(std::get<3>(*(plotColor->rbegin() + j)))},
                    {"marker", "."}
                }
            );
        }

        prevBurLayerInPixels = burLayerInPixels;
    }

    auto qCenter{ gBur.getqCenter() };

    std::vector<double> burCenterInPixels {
        (qCenter(0) - qLowerBounds(0)) / (qUpperBounds(0) - qLowerBounds(0)) * numOfSamples,
        (qCenter(1) - qUpperBounds(1)) / (qLowerBounds(1) - qUpperBounds(1)) * numOfSamples
    };

    matplotlibcpp::plot(
        { burCenterInPixels.at(0) },
        { burCenterInPixels.at(1) },
        {
            {"color", rgbFloatToString(std::get<0>(*(plotColor->rbegin())),
                                       std::get<1>(*(plotColor->rbegin())),
                                       std::get<2>(*(plotColor->rbegin())))},
            {"alpha", std::to_string(std::get<3>(*(plotColor->rbegin())))},
            {"marker", "."}
        }
    );

    for (int i{}; i < generalizedBurConfig.numOfSpines; ++i) {
        matplotlibcpp::plot(
            { burCenterInPixels.at(0), prevBurLayerInPixels->at(i).at(0) },
            { burCenterInPixels.at(1), prevBurLayerInPixels->at(i).at(1) },
            {
                {"color", rgbFloatToString(std::get<0>(*(plotColor->rbegin())),
                                        std::get<1>(*(plotColor->rbegin())),
                                        std::get<2>(*(plotColor->rbegin())))},
                {"alpha", std::to_string(std::get<3>(*(plotColor->rbegin())))},
            }
        );
    }
}


std::string GBurIRIS::visualization::rgbFloatToString(float r, float g, float b) {
    return (std::stringstream() << "#" << std::hex <<
        std::setw(2) << std::setfill('0') << int(r * 0xff) <<
        std::setw(2) << std::setfill('0') << int(g * 0xff) <<
        std::setw(2) << std::setfill('0') << int(b * 0xff)).str();
}
