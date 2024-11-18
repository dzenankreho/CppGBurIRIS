#pragma once

#include <tuple>
#include <optional>

#include <drake/planning/collision_checker.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/geometry/optimization/hpolyhedron.h>

#include "generalized_bur.hpp"

namespace GBurIRIS::visualization {

    class Figure {
        long figureNumber;

    public:
        Figure();

        void visualize2dConfigurationSpace(
            const drake::planning::CollisionChecker& collisionChecker,
            const int numOfSamples,
            const std::optional<std::tuple<double, double, double, double>>& plotColor = std::nullopt
        ) const;

        void visualize2dConvexSet(
            const drake::planning::CollisionChecker& collisionChecker,
            const drake::geometry::optimization::ConvexSet& set,
            const int numOfSamples,
            const std::optional<std::tuple<double, double, double, double>>& plotColor = std::nullopt
        ) const;

        void visualize2dGeneralizedBur(
            const drake::planning::CollisionChecker& collisionChecker,
            const GBur::GeneralizedBur& gBur,
            const int numOfSamples,
            std::optional<std::vector<std::tuple<double, double, double, double>>> plotColors = std::nullopt
        ) const;

        static void showFigures();
    };


    std::string rgbFloatToString(float r, float g, float b);

}
