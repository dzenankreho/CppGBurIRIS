#include "generalized_bur.hpp"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <stdexcept>

GBurIRIS::GBur::GeneralizedBur::GeneralizedBur(
    const Eigen::VectorXd& qCenter,
    const GeneralizedBurConfig& generalizedBurConfig,
    robots::Robot& robot,
    const std::function<Eigen::VectorXd ()>& randomConfigGenerator
) : qCenter{ qCenter },
    generalizedBurConfig{ generalizedBurConfig },
    robot{ robot },
    randomConfigGenerator{ randomConfigGenerator } {

    layers.resize(generalizedBurConfig.numOfSpines, std::vector<Eigen::VectorXd>(generalizedBurConfig.burOrder + 1));
}


void GBurIRIS::GBur::GeneralizedBur::approximateObstaclesWithPlanes() {
    if (linkObstacleDistancePairs && linkObstaclePlanes) {
        return;
    }

    auto&& collisionChecker{ robot.getCollisionChecker() };
    auto&& plant{ robot.getPlant() };
    auto&& plantContext{ collisionChecker.UpdatePositions(qCenter) };

    auto&& queryObject{ collisionChecker.model_context().GetQueryObject() };
    auto&& distancePairs{ queryObject.ComputeSignedDistancePairwiseClosestPoints() };
    auto&& inspector{ queryObject.inspector() };

    for (int i{}; i < distancePairs.size(); ++i) {
        auto&& bodyA{ plant.GetBodyFromFrameId(inspector.GetFrameId(distancePairs.at(i).id_A)) };
        auto&& bodyB{ plant.GetBodyFromFrameId(inspector.GetFrameId(distancePairs.at(i).id_B)) };

        if (collisionChecker.IsPartOfRobot(*bodyA) && collisionChecker.IsPartOfRobot(*bodyB)) {
            continue;
        }

        Eigen::Vector4d pointOnAInAFrameHomogCord;
        pointOnAInAFrameHomogCord << distancePairs.at(i).p_ACa, 1;
        Eigen::Vector3d pointOnA{ (bodyA->EvalPoseInWorld(plantContext).GetAsMatrix4() *
            (inspector.GetPoseInFrame(distancePairs.at(i).id_A).GetAsMatrix4() * pointOnAInAFrameHomogCord))(Eigen::seq(0, 2)) };

        Eigen::Vector4d pointOnBInBFrameHomogCord;
        pointOnBInBFrameHomogCord << distancePairs.at(i).p_BCb, 1;
        Eigen::Vector3d pointOnB{ (bodyB->EvalPoseInWorld(plantContext).GetAsMatrix4() *
            (inspector.GetPoseInFrame(distancePairs.at(i).id_B).GetAsMatrix4() * pointOnBInBFrameHomogCord))(Eigen::seq(0, 2)) };

        if (std::find_if(
                linkObstacleDistancePairs->begin(),
                linkObstacleDistancePairs->end(),
                [bodyA, bodyB](auto&& linkObstacleDistancePair) -> bool {
                    return std::get<0>(linkObstacleDistancePair) == bodyA->index() &&
                        std::get<1>(linkObstacleDistancePair) == bodyB->index();
                }
            ) == linkObstacleDistancePairs->end()) {

            linkObstacleDistancePairs->emplace_back(bodyA->index(), bodyB->index(), pointOnA, pointOnB, distancePairs.at(i).distance);
        }
    }

    for (auto&& linkObstacleDistancePair : *linkObstacleDistancePairs) {
        Eigen::Vector3d normalVector{ std::get<3>(linkObstacleDistancePair) - std::get<2>(linkObstacleDistancePair) };
        Eigen::Vector4d obstaclePlane;
        obstaclePlane << normalVector, -normalVector.dot(std::get<3>(linkObstacleDistancePair));
        linkObstaclePlanes->push_back(obstaclePlane);
    }
}


double GBurIRIS::GBur::GeneralizedBur::getMinDistanceToCollision() {
    if (minDistance) {
        return *minDistance;
    }

    if (!linkObstacleDistancePairs && !linkObstaclePlanes) {
        approximateObstaclesWithPlanes();
    }

    minDistance = std::numeric_limits<double>::max();

    for (auto&& linkObstacleDistancePair : *linkObstacleDistancePairs) {
        minDistance = std::min(*minDistance, std::get<4>(linkObstacleDistancePair));
    }

    return *minDistance;
}



double GBurIRIS::GBur::GeneralizedBur::getMinDistanceUnderestimation(const Eigen::VectorXd& q) {
    if (!linkObstacleDistancePairs && !linkObstaclePlanes) {
        approximateObstaclesWithPlanes();
    }


    double minDistance{ std::numeric_limits<double>::max() };
    int minBodyIndex{ int(std::get<0>(linkObstacleDistancePairs->at(0))) };

    auto&& linkPositions{ robot.getLinkPositions(q) };
    auto&& linkGeometryCompensation{ robot.getLinkGeometryCompensation() };


    for (int i{}; i < linkObstacleDistancePairs->size(); ++i) {
        auto&& linkObstacleDistancePair{ linkObstacleDistancePairs->at(i) };
        auto&& linkObstaclePlane{ linkObstaclePlanes->at(i) };

        int linkNumber{ int(std::get<0>(linkObstacleDistancePair)) - minBodyIndex };

        Eigen::VectorXd proximalLinkPoint{ linkPositions.at(linkNumber) }, distalLinkPoint{ linkPositions.at(linkNumber + 1) };


        Eigen::Vector4d proximalLinkPointInHomogCord, distalLinkPointInHomogCord;

        if (proximalLinkPoint.size() == 2) {
            proximalLinkPointInHomogCord << proximalLinkPoint, 0, 1;
            distalLinkPointInHomogCord << distalLinkPoint, 0, 1;
        } else {
            proximalLinkPointInHomogCord << proximalLinkPoint, 1;
            distalLinkPointInHomogCord << distalLinkPoint, 1;
        }

        double proximalToPlaneHelper{ linkObstaclePlane.dot(proximalLinkPointInHomogCord) },
            distalToPlaneHelper{ linkObstaclePlane.dot(distalLinkPointInHomogCord) };

        double proximalToPlaneDistance{
            (std::abs(proximalToPlaneHelper) / linkObstaclePlane(Eigen::seq(0, 2)).norm()) -
                linkGeometryCompensation.at(linkNumber)
        };

        double distalToPlaneDistance{
             std::abs(distalToPlaneHelper) / linkObstaclePlane(Eigen::seq(0, 2)).norm() -
                linkGeometryCompensation.at(linkNumber)
        };

        if (proximalToPlaneHelper < 0 && proximalToPlaneDistance > 0 && proximalToPlaneDistance < minDistance) {
            minDistance = proximalToPlaneDistance;
        }

        if (distalToPlaneHelper < 0 && distalToPlaneDistance > 0 && distalToPlaneDistance < minDistance) {
            minDistance = distalToPlaneDistance;
        }
    }


    if (std::abs(minDistance - std::numeric_limits<double>::max()) < 1e-5) {
        throw std::runtime_error("Min. distance is infinite (robot may be in collision with a obstacle plane)!");
    }

    return minDistance;
}



std::tuple<std::vector<Eigen::VectorXd>, std::vector<std::vector<Eigen::VectorXd>>> GBurIRIS::GBur::GeneralizedBur::calculateBur() {
    auto&& qLowerBounds{ robot.getPlant().GetPositionLowerLimits() };
    auto&& qUpperBounds{ robot.getPlant().GetPositionUpperLimits() };

    if (!randomConfigs) {
        auto&& qSpaceWidth{ qUpperBounds - qLowerBounds };
        double maxDistanceConfigSpace{};

        for (int i{}; i < qSpaceWidth.size(); ++i) {
            maxDistanceConfigSpace = std::max(maxDistanceConfigSpace, qSpaceWidth(i));
        }

        while (generalizedBurConfig.numOfSpines > randomConfigs->size()) {
            auto&& q{ randomConfigGenerator() };
            auto&& unitVec{ (q - qCenter) / (q - qCenter).norm() };
            randomConfigs->push_back(qCenter + unitVec * 2 * maxDistanceConfigSpace);
        }

        double initMinDistance{ getMinDistanceToCollision() };

        for (int i{}; i < randomConfigs->size(); ++i) {
            auto&& qe{ randomConfigs->at(i) };
            auto startingPoint{ qCenter };
            double minDistance{ initMinDistance };

            for (int j{}; j < generalizedBurConfig.burOrder + 1; ++j) {
                double tk{};
                auto qk{ startingPoint };

                while (minDistance > generalizedBurConfig.minDistanceTol &&
                    evaluatePhiFunction(minDistance, tk, qe, startingPoint) >= generalizedBurConfig.phiTol * minDistance) {

                    auto&& radii{ robot.getEnclosingRadii(qk) };

                    double prevTk{ tk }, weightSum{};
                    auto&& helperVec{ qe - qk };
                    for (int k{}; k < radii.size(); ++k) {
                        weightSum += radii.at(k) * std::abs(helperVec(k));
                    }
                    tk += (evaluatePhiFunction(minDistance, tk, qe, startingPoint) / weightSum) * (1 - tk);
                    qk = startingPoint + tk * (qe - startingPoint);

                    bool configInsideLimits{ true };
                    for (int k{}; k < qk.size() && configInsideLimits; ++k) {
                        configInsideLimits = (qk(k) >= qLowerBounds(k)) && (qk(k) <= qUpperBounds(k));
                    }

                    if (!configInsideLimits) {
                        tk = prevTk;
                        qk = startingPoint + tk * (qe - startingPoint);
                        break;
                    }
                }

                layers.at(i).at(j) = qk;
                startingPoint = qk;
                minDistance = getMinDistanceUnderestimation(qk);
            }
        }
    }

    return std::make_tuple(*randomConfigs, layers);
}
