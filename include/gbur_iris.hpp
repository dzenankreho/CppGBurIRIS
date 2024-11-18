#pragma once

#include <drake/geometry/optimization/hyperellipsoid.h>
#include <drake/planning/collision_checker.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <vector>
#include <Eigen/Dense>


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


// def InflatePolytopes(
//         collisionChecker: CollisionChecker,
//         ellipsoids: list[Hyperellipsoid],
//         numOfIrisIterations: int = 1
//         ) -> list[HPolyhedron]:
//
//
//     vccIrisOptions = IrisOptions()
//     # VCC only requires 1 iteration
//     vccIrisOptions.iteration_limit = numOfIrisIterations
//
//     plant = collisionChecker.plant()
//     plantContext = collisionChecker.plant_context()
//
//     vccRegions = []
//     for ellipsoid in ellipsoids:
//         vccIrisOptions.starting_ellipse = ellipsoid
//         plant.SetPositions(plantContext, ellipsoid.center())
//         vccRegions.append(
//             IrisInConfigurationSpace(plant, plantContext, vccIrisOptions)
//         )
//
//     return vccRegions
}
