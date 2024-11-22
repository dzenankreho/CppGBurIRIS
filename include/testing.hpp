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
        template <typename T>
        static double calculateMean(const std::vector<T>& data);
        template <typename T>
        static double calculateStandardDeviation(const std::vector<T>& data);

    protected:
        robots::Robot& robot;
        const unsigned int randomSeed;

    };


    class TestGBurIRIS final : public Test {

    public:
        enum class GBurDistantConfigOption{ individualConfigs, rotationMatrix };

        TestGBurIRIS(
            robots::Robot& robot,
            const GBurIRISConfig& gBurIRISConfig,
            const GBurDistantConfigOption& gBurDistantConfigOption = GBurDistantConfigOption::individualConfigs,
            unsigned int randomSeed = std::time(nullptr)
        );

        std::tuple<
            std::vector<std::size_t>,
            std::vector<std::size_t>,
            std::vector<double>
        > run(int numOfRuns) const override;

    private:
        const GBurIRISConfig gBurIRISConfig;
        const GBurDistantConfigOption gBurDistantConfigOption;

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


    template <typename T>
    double Test::calculateMean(const std::vector<T>& data) {
        return double(std::accumulate(data.begin(), data.end(), T{})) / data.size();
    }

    template <typename T>
    double Test::calculateStandardDeviation(const std::vector<T>& data) {
        double mean{ calculateMean(data) };

        std::vector<double> diff(data.size());
        std::transform(data.begin(), data.end(), diff.begin(), [mean](const T& t) { return double(t) - mean; });

        return std::sqrt(std::inner_product(diff.begin(), diff.end(), diff.begin(), T{}) / data.size());
    }
};
