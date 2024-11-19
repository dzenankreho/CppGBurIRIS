#pragma once
#include <ctime>
#include <tuple>
#include <vector>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/planning/iris/iris_from_clique_cover.h>
#include "robot.hpp"
#include "gbur_iris.hpp"

namespace GBurIRIS::testing {

    class Test {

    public:
        Test(robots::Robot& robot, unsigned int randomSeed);
        virtual std::tuple<
            std::vector<std::size_t>,
            std::vector<std::size_t>,
            std::vector<double>
        > run(int numOfRuns) const = 0;

    protected:
        robots::Robot& robot;
        const unsigned int randomSeed;

    };


    class TestGBurIRIS final : public Test {

    public:
        TestGBurIRIS(
            robots::Robot& robot,
            const GBurIRISConfig& gBurIRISConfig,
            unsigned int randomSeed = std::time(nullptr)
        );

        std::tuple<
            std::vector<std::size_t>,
            std::vector<std::size_t>,
            std::vector<double>
        > run(int numOfRuns) const override;

    private:
        const GBurIRISConfig gBurIRISConfig;

    };



    class TestVCC final : public Test {

    public:
        TestVCC(
            robots::Robot& robot,
            const drake::planning::IrisFromCliqueCoverOptions& irisFromCliqueCoverOptions,
            unsigned int randomSeed = std::time(nullptr)
        );

        std::tuple<
            std::vector<std::size_t>,
            std::vector<std::size_t>,
            std::vector<double>
        > run(int numOfRuns) const override;

    private:
        const drake::planning::IrisFromCliqueCoverOptions irisFromCliqueCoverOptions;

    };

};
