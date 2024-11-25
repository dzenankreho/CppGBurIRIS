#include "gbur_iris.hpp"

#include <drake/geometry/optimization/iris.h>
#include <drake/geometry/optimization/affine_ball.h>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <random>


const char* irisCenterMarginErrorStr{
    "The current center of the IRIS region is within "
    "options.configuration_space_margin of being infeasible.  Check your "
    "sample point and/or any additional constraints you've passed in via "
    "the options. The configuration space surrounding the sample point "
    "must have an interior."
};

const char* minVolEllipsoidRankErrorStr{
    "The numerical rank of the points appears to be less than the "
    "ambient dimension. The smallest singular value is {}, which is "
    "below rank_tol = {}. Decrease rank_tol or consider using "
    "AffineBall::MinimumVolumeCircumscribedEllipsoid instead."
};


drake::geometry::optimization::Hyperellipsoid GBurIRIS::MinVolumeEllipsoid(
    const drake::planning::CollisionChecker& collisionChecker,
    const std::vector<Eigen::VectorXd>& points
) {

    Eigen::MatrixXd pointsMatrix(points.begin()->size(), points.size());
    for (int i{}; i < points.size(); ++i) {
        pointsMatrix.col(i) = points.at(i);
    }

    auto ellipsoid{
        drake::geometry::optimization::Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid(pointsMatrix)
    };

    if (collisionChecker.CheckConfigCollisionFree(ellipsoid.center())) {
        return ellipsoid;
    }

    int closestPoint{};
    double minDistance{ (points.at(closestPoint) - ellipsoid.center()).norm() };

    for (int i{ 1 }; i < points.size(); ++i) {
        if (double currentDistance{ (points.at(i) - ellipsoid.center()).norm() }; currentDistance < minDistance &&
            collisionChecker.CheckConfigCollisionFree(points.at(i))
        ) {
            minDistance = currentDistance;
            closestPoint = i;
        }
    }

    return drake::geometry::optimization::Hyperellipsoid(ellipsoid.A(), points.at(closestPoint));
}


drake::geometry::optimization::HPolyhedron GBurIRIS::InflatePolytope(
    const drake::planning::CollisionChecker& collisionChecker,
    const drake::geometry::optimization::Hyperellipsoid& ellipsoid,
    int numOfIrisIterations
) {

    drake::geometry::optimization::IrisOptions irisOptions;
    irisOptions.iteration_limit = numOfIrisIterations;

    irisOptions.starting_ellipse = ellipsoid;
    auto&& plantContext{ collisionChecker.UpdatePositions(ellipsoid.center()) };

    return IrisInConfigurationSpace(collisionChecker.plant(), plantContext, irisOptions);
}


double GBurIRIS::CheckCoverage(
    const drake::planning::CollisionChecker& collisionChecker,
    const std::vector<drake::geometry::optimization::HPolyhedron>& sets,
    int numSamplesCoverageCheck,
    const std::function<Eigen::VectorXd ()>& randomConfigGenerator
) {

    int numSamplesGenerated{}, numCoveredPoints{};

    while (numSamplesCoverageCheck > numSamplesGenerated) {
        if (auto&& q{ randomConfigGenerator() }; collisionChecker.CheckConfigCollisionFree(q)) {
            ++numSamplesGenerated;

            numCoveredPoints += std::any_of(
                sets.begin(),
                sets.end(),
                [q](auto&& set) { return set.PointInSet(q); }
            );
        }
    }

    return double(numCoveredPoints) / double(numSamplesCoverageCheck);
}

std::tuple<
    std::vector<drake::geometry::optimization::HPolyhedron>,
    double,
    std::vector<GBurIRIS::GBur::GeneralizedBur>
> GBurIRIS::GBurIRIS(
    robots::Robot& robot,
    const GBurIRISConfig& gBurIRISConfig,
    const std::function<Eigen::VectorXd ()>& randomConfigGenerator
) {

    std::vector<drake::geometry::optimization::HPolyhedron> regions;
    std::vector<GBur::GeneralizedBur> burs;
    double coverage;

    auto&& collisionChecker{ robot.getCollisionChecker() };

    for (int i{}; i < gBurIRISConfig.numOfIter; ++i) {
        coverage = CheckCoverage(
            collisionChecker,
            regions,
            gBurIRISConfig.numPointsCoverageCheck,
            randomConfigGenerator
        );


        if (coverage >= gBurIRISConfig.coverage) {
            break;
        }


        Eigen::VectorXd burCenter;

        for (
            burCenter = randomConfigGenerator();
            !collisionChecker.CheckConfigCollisionFree(burCenter) ||
                std::any_of(
                    regions.begin(),
                    regions.end(),
                    [burCenter](auto&& region) { return region.PointInSet(burCenter); }
                );
            burCenter = randomConfigGenerator()
        );


        GBurIRIS::GBur::GeneralizedBur bur(
            burCenter,
            GBurIRIS::GBur::GeneralizedBurConfig{
                gBurIRISConfig.numOfSpines,
                gBurIRISConfig.burOrder,
                gBurIRISConfig.minDistanceTol,
                gBurIRISConfig.phiTol
            },
            robot,
            randomConfigGenerator
        );

        if (bur.getMinDistanceToCollision() < gBurIRISConfig.minDistanceTol) {
            --i;
            continue;
        }


        auto [burRandomConfigs, layers] = bur.calculateBur();

        burs.push_back(bur);

        std::vector<Eigen::VectorXd> outerLayer;
        for (auto&& spine : layers) {
            outerLayer.push_back(*(spine.end() - 1));
        }

        drake::geometry::optimization::Hyperellipsoid ellipsoid;
        try {
            ellipsoid = GBurIRIS::MinVolumeEllipsoid(collisionChecker, outerLayer);
        } catch (const std::runtime_error& exception) {
            if (std::string(exception.what()) != minVolEllipsoidRankErrorStr) {
                throw;
            }

            burs.pop_back();
            --i;
            continue;
        }

        try {
            regions.push_back(GBurIRIS::InflatePolytope(collisionChecker, ellipsoid));
        } catch (const std::logic_error& exception) {
            if (!gBurIRISConfig.ignoreDeltaExceptionFromIRISNP ||
                    std::string(exception.what()) != irisCenterMarginErrorStr
            ) {
                throw;
            }

            burs.pop_back();
            --i;
            continue;
        }
    }

    return std::make_tuple(regions, coverage, burs);
}


std::tuple<
    std::vector<drake::geometry::optimization::HPolyhedron>,
    double,
    std::vector<GBurIRIS::GBur::GeneralizedBur>
> GBurIRIS::GBurIRIS(
    robots::Robot& robot,
    GBurIRISConfig gBurIRISConfig,
    const std::function<Eigen::VectorXd ()>& randomConfigGenerator,
    const std::function<Eigen::MatrixXd ()>& generateRandomRotationMatrix
) {

    std::vector<drake::geometry::optimization::HPolyhedron> regions;
    std::vector<GBur::GeneralizedBur> burs;
    double coverage;

    auto&& collisionChecker{ robot.getCollisionChecker() };

    for (int i{}; i < gBurIRISConfig.numOfIter; ++i) {
        coverage = CheckCoverage(
            collisionChecker,
            regions,
            gBurIRISConfig.numPointsCoverageCheck,
            randomConfigGenerator
        );


        if (coverage >= gBurIRISConfig.coverage) {
            break;
        }


        Eigen::VectorXd burCenter;

        for (
            burCenter = randomConfigGenerator();
            !collisionChecker.CheckConfigCollisionFree(burCenter) ||
                std::any_of(
                    regions.begin(),
                    regions.end(),
                    [burCenter](auto&& region) { return region.PointInSet(burCenter); }
                );
            burCenter = randomConfigGenerator()
        );


        GBurIRIS::GBur::GeneralizedBur bur(
            burCenter,
            GBurIRIS::GBur::GeneralizedBurConfig{
                gBurIRISConfig.numOfSpines,
                gBurIRISConfig.burOrder,
                gBurIRISConfig.minDistanceTol,
                gBurIRISConfig.phiTol
            },
            robot,
            generateRandomRotationMatrix()
        );

        if (bur.getMinDistanceToCollision() < gBurIRISConfig.minDistanceTol) {
            --i;
            continue;
        }


        auto [burRandomConfigs, layers] = bur.calculateBur();

        burs.push_back(bur);

        std::vector<Eigen::VectorXd> outerLayer;
        for (auto&& spine : layers) {
            outerLayer.push_back(*(spine.end() - 1));
        }

        drake::geometry::optimization::Hyperellipsoid ellipsoid;
        try {
            ellipsoid = GBurIRIS::MinVolumeEllipsoid(collisionChecker, outerLayer);
        } catch (const std::runtime_error& exception) {
            if (std::string(exception.what()) != minVolEllipsoidRankErrorStr) {
                throw;
            }

            burs.pop_back();
            --i;
            continue;
        }

        try {
            regions.push_back(GBurIRIS::InflatePolytope(collisionChecker, ellipsoid));
        } catch(const std::logic_error& exception) {
            if (!gBurIRISConfig.ignoreDeltaExceptionFromIRISNP ||
                    std::string(exception.what()) != irisCenterMarginErrorStr
            ) {
                throw;
            }

            burs.pop_back();
            --i;
            continue;
        }
    }

    return std::make_tuple(regions, coverage, burs);
}
