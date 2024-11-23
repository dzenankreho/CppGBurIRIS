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
    const GBurDistantConfigOption& gBurDistantConfigOption,
    unsigned int randomSeed
) : Test{ robot, randomSeed },
    gBurIRISConfig{ gBurIRISConfig },
    gBurDistantConfigOption{gBurDistantConfigOption} {}


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


Eigen::MatrixXd generateRandomRotationMatrix(const int matrixSize) {
    // https://scicomp.stackexchange.com/a/34974
    // https://en.cppreference.com/w/cpp/numeric/random/normal_distribution

    static std::random_device randomDevice;
    static std::mt19937 randomNumEngine{ randomDevice() };
    static std::normal_distribution<double> normalDistribution{ 0, 1 };

    Eigen::MatrixXd X{
        Eigen::MatrixXd::Zero(matrixSize, matrixSize).unaryExpr(
            [](double) { return normalDistribution(randomNumEngine); }
        )
    };

    Eigen::MatrixXd XtX{ X.transpose() * X };

    Eigen::MatrixXd invSqrt{
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(XtX).operatorInverseSqrt()
    };

    Eigen::MatrixXd R{ X * invSqrt };

    if (R.determinant() < 0) {
        R.col(0) *= -1;
    }

    return R;
}


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

    long numOfDof{ plant.GetPositionLowerLimits().size() };

    std::vector<std::size_t> numOfRegions(numOfRuns), execTime(numOfRuns);
    std::vector<double> coverage(numOfRuns);

    for (int i{}; i < numOfRuns; ++i) {
        if (gBurDistantConfigOption == GBurDistantConfigOption::individualConfigs) {
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

        if (gBurDistantConfigOption == GBurDistantConfigOption::rotationMatrix) {
            drake::RandomGenerator drakeRandomGenerator(std::rand());
            RandomGenerator randomGenerator(domain, drakeRandomGenerator);

            auto startTime{ std::chrono::steady_clock::now() };
            auto [regionsGBurIRIS, coverageGBurIRIS, burs] = GBurIRIS::GBurIRIS(
                robot,
                gBurIRISConfig,
                std::bind(&RandomGenerator::randomConfig, &randomGenerator),
                std::bind(&generateRandomRotationMatrix, numOfDof)
            );
            auto endTime{ std::chrono::steady_clock::now() };

            numOfRegions.at(i) = regionsGBurIRIS.size();
            coverage.at(i) = coverageGBurIRIS;
            execTime.at(i) = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
        }
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
        drake::planning::IrisInConfigurationSpaceFromCliqueCover(
            collisionChecker,
            irisFromCliqueCoverOptions,
            &drakeRandomGenerator,
            &regionsVCC
        );
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
