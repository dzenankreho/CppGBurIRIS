#pragma once

#include <drake/geometry/optimization/hyperellipsoid.h>
#include <drake/planning/collision_checker.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <vector>
#include <Eigen/Dense>
#include "generalized_bur.hpp"
#include <tuple>

namespace GBurIRIS {

    drake::geometry::optimization::Hyperellipsoid MinVolumeEllipsoid(
        const drake::planning::CollisionChecker& collisionChecker,
        const std::vector<Eigen::VectorXd>& points
    );

    drake::geometry::optimization::HPolyhedron InflatePolytope(
        const drake::planning::CollisionChecker& collisionChecker,
        const drake::geometry::optimization::Hyperellipsoid& ellipsoid,
        int numOfIrisIterations = 1
    );


    struct GBurIRISConfig {
        int numOfSpines{ 7 };
        int burOrder{ 4 };
        double minDistanceTol{ 1e-5 };
        double phiTol{ 0.1 };
        int numPointsCoverageCheck{ 5000 };
        double coverage{ 0.7 };
        int numOfIter{ 100 };
        int numOfIterIRIS{ 1 };
        bool ignoreDeltaExceptionFromIRISNP{ true };
    };


    double CheckCoverage(
        const drake::planning::CollisionChecker& collisionChecker,
        const std::vector<drake::geometry::optimization::HPolyhedron>& sets,
        int numSamplesCoverageCheck,
        const std::function<Eigen::VectorXd ()>& randomConfigGenerator
    );


    std::tuple<
        std::vector<drake::geometry::optimization::HPolyhedron>,
        double,
        std::vector<GBur::GeneralizedBur>
    > GBurIRIS(
        robots::Robot& robot,
        const GBurIRISConfig& gBurIRISConfig,
        const std::function<Eigen::VectorXd ()>& randomConfigGenerator
    );

}
