#include "gbur_iris.hpp"

// #include <drake/geometry/optimization/affine_ball.h>
#include <drake/geometry/optimization/iris.h>

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
        if (double currentDistance{ (points.at(i) - ellipsoid.center()).norm() }; currentDistance < minDistance) {
            minDistance = currentDistance;
            closestPoint = i;
        }
    }

    return drake::geometry::optimization::Hyperellipsoid(points.at(closestPoint), ellipsoid.A());

//     auto&& affineBall{
//         drake::geometry::optimization::AffineBall::MinimumVolumeCircumscribedEllipsoid(pointsMatrix)
//     };
//
//
//     auto svd{ Eigen::JacobiSVD<Eigen::MatrixXd>(affineBall.B(), Eigen::ComputeFullU | Eigen::ComputeFullV) };
//     auto S{ svd.singularValues() };
//
//     for (int i{}; i < S.size(); ++i) {
//         if (S(i) < 1e-4) {
//             S(i) = 1e-4;
//         }
//     }
//
//     auto newBall{
//         svd.matrixU() * (S.asDiagonal() * svd.matrixV().transpose())
//     };
//
// U, S, Vt = np.linalg.svd(affineBall.B())
//
// # Force the ellipsoid to have volume, i.e., at length to small axes
// for i in range(S.shape[0]):
//     if S[i] < 1e-4:
//         S[i] = 1e-4
// newB = U @ np.diag(S) @ Vt
//
// ellipsoidCenter = affineBall.center()
//
// # Move the ellipsoid center if in collision
// if not collisionChecker.CheckConfigCollisionFree(ellipsoidCenter):
//     ellipsoidCenter = clique[np.argmin([np.linalg.norm(ellipsoidCenter - node) for node in clique])]
//
// ellipsoids.append(Hyperellipsoid(AffineBall(newB, ellipsoidCenter)))
//
//
//     return drake::geometry::optimization::Hyperellipsoid(drake::geometry::optimization::AffineBall(newBall, affineBall.center()));
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

