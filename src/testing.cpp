#include "testing.hpp"
#include <chrono>
#include <drake/common/random.h>
#include <numeric>
#include <algorithm>
#include <cmath>

GBurIRIS::testing::Test::Test(robots::Robot& robot, unsigned int randomSeed)
    : robot{ robot }, randomSeed{ randomSeed } {

    std::srand(randomSeed);
}


GBurIRIS::testing::TestGBurIRIS::TestGBurIRIS(
    robots::Robot& robot,
    const GBurIRISConfig& gBurIRISConfig,
    unsigned int randomSeed
) : Test{ robot, randomSeed }, gBurIRISConfig{ gBurIRISConfig } {}


GBurIRIS::testing::TestVCC::TestVCC(
    robots::Robot& robot,
    const drake::planning::IrisFromCliqueCoverOptions& irisFromCliqueCoverOptions,
    unsigned int randomSeed
) : Test{ robot, randomSeed }, irisFromCliqueCoverOptions{ irisFromCliqueCoverOptions } {}




class RandomGenerator {
public:
    RandomGenerator(
        const drake::geometry::optimization::HPolyhedron& domain,
        drake::RandomGenerator& randomGenerator
    ) : domain{ domain },
        randomGenerator{ randomGenerator },
        lastSample{ std::nullopt } {}

    Eigen::VectorXd randomConfig() {
        if (!lastSample) {
            lastSample = domain.UniformSample(&randomGenerator);
        } else {
            lastSample = domain.UniformSample(&randomGenerator, *lastSample);
        }

        return *lastSample;
    }

private:
    const drake::geometry::optimization::HPolyhedron& domain;
    drake::RandomGenerator& randomGenerator;
    std::optional<Eigen::VectorXd> lastSample;

};


std::tuple<
    std::vector<std::size_t>,
    std::vector<std::size_t>,
    std::vector<double>
> GBurIRIS::testing::TestGBurIRIS::run(int numOfRuns) const {

    auto&& plant{ robot.getPlant() };
    auto domain = drake::geometry::optimization::HPolyhedron::MakeBox(
        plant.GetPositionLowerLimits(),
        plant.GetPositionUpperLimits()
    );

    std::vector<std::size_t> numOfRegions(numOfRuns), execTime(numOfRuns);
    std::vector<double> coverage(numOfRuns);


    for (int i{}; i < numOfRuns; ++i) {
        drake::RandomGenerator drakeRandomGenerator(std::rand());
        RandomGenerator randomGenerator(domain, drakeRandomGenerator);

        auto startTime{ std::chrono::steady_clock::now() };
        auto [regionsGBurIRIS, coverageGBurIRIS, burs] = GBurIRIS::GBurIRIS(
            robot,
            gBurIRISConfig,
            std::bind(&RandomGenerator::randomConfig, &randomGenerator)
        );
        auto endTime{ std::chrono::steady_clock::now() };

        numOfRegions.at(i) = regionsGBurIRIS.size();
        coverage.at(i) = coverageGBurIRIS;
        execTime.at(i) = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
    }


    return std::make_tuple(execTime, numOfRegions, coverage);
}


std::tuple<
    std::vector<std::size_t>,
    std::vector<std::size_t>,
    std::vector<double>
> GBurIRIS::testing::TestVCC::run(int numOfRuns) const {

    auto&& plant{ robot.getPlant() };
    auto domain = drake::geometry::optimization::HPolyhedron::MakeBox(
        plant.GetPositionLowerLimits(),
        plant.GetPositionUpperLimits()
    );
    auto&& collisionChecker{ robot.getCollisionChecker() };

    std::vector<std::size_t> numOfRegions(numOfRuns), execTime(numOfRuns);
    std::vector<double> coverage(numOfRuns);

    for (int i{}; i < numOfRuns; ++i) {
        drake::RandomGenerator drakeRandomGenerator(std::rand());
        std::vector<drake::geometry::optimization::HPolyhedron> regionsVCC;

        auto startTime{ std::chrono::steady_clock::now() };
        while(true) {
            try {
                drake::planning::IrisInConfigurationSpaceFromCliqueCover(
                    collisionChecker,
                    irisFromCliqueCoverOptions,
                    &drakeRandomGenerator,
                    &regionsVCC
                );

                break;
            } catch (...) {}
        }
        auto endTime{ std::chrono::steady_clock::now() };

        RandomGenerator randomGenerator(domain, drakeRandomGenerator);
        double coverageVCC{
            GBurIRIS::CheckCoverage(
                collisionChecker,
                regionsVCC,
                irisFromCliqueCoverOptions.num_points_per_coverage_check,
                std::bind(&RandomGenerator::randomConfig, &randomGenerator)
            )
        };

        numOfRegions.at(i) = regionsVCC.size();
        coverage.at(i) = coverageVCC;
        execTime.at(i) = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
    }


    return std::make_tuple(execTime, numOfRegions, coverage);
}
