#include <iostream>
#include <memory>
#include <tuple>
#include <vector>
#include <functional>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include <Eigen/Dense>

#include <drake/planning/robot_diagram_builder.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/planning/iris/iris_from_clique_cover.h>
#include <drake/common/random.h>
#include <drake/geometry/optimization/hpolyhedron.h>

#include "visualization.hpp"
#include "planar_arm.hpp"
#include "generalized_bur.hpp"
#include "gbur_iris.hpp"
#include "testing.hpp"
#include "anthropomorphic_arm.hpp"


template <typename T>
std::string to_string_with_precision(const T value, const int n = 4) {
// https://stackoverflow.com/questions/16605967/set-precision-of-stdto-string-when-converting-floating-point-values
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << value;
    return std::move(out).str();
}


enum class RobotType { Planar2DofArm = 2, AnthropomorphicArm = 3, Planar6DofArm = 6 };
enum class ObstacleCount { OneToTwo = 1, ThreeToFour = 2, EightOrMore = 3 };


void runTests(RobotType robotType, ObstacleCount obstacleCount, int numOfRuns) {
    drake::planning::RobotDiagramBuilder<double> robotDiagramBuilder;
    drake::multibody::MultibodyPlant<double>& plant{ robotDiagramBuilder.plant() };

    std::string projectPath{ std::filesystem::current_path().parent_path().string() };
    std::string scenePath{
        std::string{"/scenes/"} + std::to_string(int(robotType)) +
        std::string{"dofScene"} + std::to_string(int(obstacleCount)) + std::string{".dmd.yaml"}
    };

    drake::multibody::Parser parser(&plant);
    parser.package_map().Add("assets", projectPath + "/assets");
    parser.AddModels(projectPath + scenePath);
    plant.Finalize();

    std::unique_ptr<drake::planning::RobotDiagram<double>> diagram{ robotDiagramBuilder.Build() };
    std::unique_ptr<drake::systems::Context<double>> diagramContext{ diagram->CreateDefaultContext() };

    drake::planning::CollisionCheckerParams collisionCheckerParams;
    collisionCheckerParams.model = std::move(diagram);
    collisionCheckerParams.edge_step_size = 0.1;
    switch (robotType) {
        case RobotType::Planar2DofArm:
            collisionCheckerParams.robot_model_instances.push_back(plant.GetModelInstanceByName("2dofPlanarArm"));
            break;

        case RobotType::AnthropomorphicArm:
            collisionCheckerParams.robot_model_instances.push_back(plant.GetModelInstanceByName("AnthropomorphicArm"));
            break;

        case RobotType::Planar6DofArm:
            collisionCheckerParams.robot_model_instances.push_back(plant.GetModelInstanceByName("6dofPlanarArm"));
            break;
    }

    std::unique_ptr<drake::planning::CollisionChecker> collisionChecker{
        std::make_unique<drake::planning::SceneGraphCollisionChecker>(std::move(collisionCheckerParams))
    };

    if (robotType == RobotType::AnthropomorphicArm) {
        auto bodyIndices{ plant.GetBodyIndices(plant.GetModelInstanceByName("AnthropomorphicArm")) };
        for (int i{}; i < bodyIndices.size(); ++i) {
            for (int j{i + 1}; j < bodyIndices.size(); ++j) {
                collisionChecker->SetCollisionFilteredBetween(bodyIndices.at(i), bodyIndices.at(j), true);
            }
        }
    }

    std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>> jointChildAndEndEffectorLinks;
    switch (robotType) {
        case RobotType::Planar2DofArm:
            for (
                const std::string& rigidBodyName : {
                    "2dofPlanarLink1", "2dofPlanarLink2", "2dofPlanarEndEffector"
                }
            ) {
                jointChildAndEndEffectorLinks.push_back(plant.GetBodyByName(rigidBodyName, plant.GetModelInstanceByName("2dofPlanarArm")));
            }
            break;

        case RobotType::AnthropomorphicArm:
            for (
                const std::string& rigidBodyName : {
                    "AnthropomorphicArmLink1", "AnthropomorphicArmLink2",
                    "AnthropomorphicArmLink3", "AnthropomorphicArmEndEffector"
                }
            ) {
                jointChildAndEndEffectorLinks.push_back(plant.GetBodyByName(rigidBodyName, plant.GetModelInstanceByName("AnthropomorphicArm")));
            }
            break;

        case RobotType::Planar6DofArm:
            for (
                const std::string& rigidBodyName : {
                    "6dofPlanarLink1", "6dofPlanarLink2", "6dofPlanarLink3", "6dofPlanarLink4",
                    "6dofPlanarLink5", "6dofPlanarLink6", "6dofPlanarEndEffector"
                }
            ) {
                jointChildAndEndEffectorLinks.push_back(plant.GetBodyByName(rigidBodyName, plant.GetModelInstanceByName("6dofPlanarArm")));
            }
            break;
    }

    std::vector<double> linkGeometryCompensation(int(robotType), 0.1);


    std::unique_ptr<GBurIRIS::robots::Robot> robot;

    if (robotType == RobotType::AnthropomorphicArm) {
        robot = std::make_unique<GBurIRIS::robots::AnthropomorphicArm>(
            *collisionChecker,
            jointChildAndEndEffectorLinks,
            linkGeometryCompensation
        );
    } else {
        robot = std::make_unique<GBurIRIS::robots::PlanarArm>(
            *collisionChecker,
            jointChildAndEndEffectorLinks,
            linkGeometryCompensation
        );
    }

    GBurIRIS::GBurIRISConfig gBurIRISConfig;
    gBurIRISConfig.numOfSpines = 2 * int(robotType);
    switch (robotType) {
        case RobotType::Planar2DofArm:
            gBurIRISConfig.coverage = 0.9;
            break;

        case RobotType::AnthropomorphicArm:
            gBurIRISConfig.coverage = 0.6;
            break;

        case RobotType::Planar6DofArm:
            gBurIRISConfig.coverage = 0.4;
            break;
    }


    GBurIRIS::testing::TestGBurIRIS testGBurIRIS(*robot, gBurIRISConfig);
    auto [execTimeGBurIRIS, numOfRegionsGBurIRIS, coverageGBurIRIS] = testGBurIRIS.run(numOfRuns);


    GBurIRIS::testing::TestGBurIRIS testGBurIRIS2(
        *robot,
        gBurIRISConfig,
        GBurIRIS::testing::TestGBurIRIS::GBurDistantConfigOption::rotationMatrix
    );
    auto [execTimeGBurIRIS2, numOfRegionsGBurIRIS2, coverageGBurIRIS2] = testGBurIRIS2.run(numOfRuns);


    drake::planning::IrisFromCliqueCoverOptions irisFromCliqueCoverOptions;
    irisFromCliqueCoverOptions.coverage_termination_threshold = gBurIRISConfig.coverage;
    irisFromCliqueCoverOptions.num_points_per_visibility_round = 500;
    irisFromCliqueCoverOptions.num_points_per_coverage_check = 5000;
    irisFromCliqueCoverOptions.minimum_clique_size = 10;

    collisionChecker->SetPaddingAllRobotEnvironmentPairs(5e-3);

    GBurIRIS::testing::TestVCC testVCC(*robot, irisFromCliqueCoverOptions);
    auto [execTimeVCC, numOfRegionsVCC, coverageVCC] = testVCC.run(numOfRuns);

    std::cout << std::endl << std::left
              << std::string(120, '-') << std::endl
              << std::setw(30) << "Algorithm"
              << std::setw(30) << "Runtime [s]"
              << std::setw(30) << " Num of regions"
              << std::setw(30) << " Coverage [%]" << std::endl
              << std::string(120, '-') << std::endl
              << std::setw(30) << "GBur-IRIS (individ. configs)"
              << std::setw(30) << to_string_with_precision(testGBurIRIS.calculateMean(execTimeGBurIRIS)) + "+-"
                + to_string_with_precision(testGBurIRIS.calculateStandardDeviation(execTimeGBurIRIS))
              << std::setw(30) << to_string_with_precision(testGBurIRIS.calculateMean(numOfRegionsGBurIRIS)) + "+-"
                + to_string_with_precision(testGBurIRIS.calculateStandardDeviation(numOfRegionsGBurIRIS))
              << std::setw(30) << to_string_with_precision(testGBurIRIS.calculateMean(coverageGBurIRIS) * 100) + "+-"
                + to_string_with_precision(testGBurIRIS.calculateStandardDeviation(coverageGBurIRIS) * 100) << std::endl
              << std::setw(30) << "GBur-IRIS (rotation matrix)"
              << std::setw(30) << to_string_with_precision(testGBurIRIS2.calculateMean(execTimeGBurIRIS2)) + "+-"
                + to_string_with_precision(testGBurIRIS2.calculateStandardDeviation(execTimeGBurIRIS2))
              << std::setw(30) << to_string_with_precision(testGBurIRIS2.calculateMean(numOfRegionsGBurIRIS2)) + "+-"
                + to_string_with_precision(testGBurIRIS2.calculateStandardDeviation(numOfRegionsGBurIRIS2))
              << std::setw(30) << to_string_with_precision(testGBurIRIS2.calculateMean(coverageGBurIRIS2) * 100) + "+-"
                + to_string_with_precision(testGBurIRIS2.calculateStandardDeviation(coverageGBurIRIS2) * 100) << std::endl
              << std::setw(30) << "VCC"
              << std::setw(30) << to_string_with_precision(testVCC.calculateMean(execTimeVCC)) + "+-"
                + to_string_with_precision(testVCC.calculateStandardDeviation(execTimeVCC))
              << std::setw(30) << to_string_with_precision(testVCC.calculateMean(numOfRegionsVCC)) + "+-"
                + to_string_with_precision(testVCC.calculateStandardDeviation(numOfRegionsVCC))
              << std::setw(30) << to_string_with_precision(testVCC.calculateMean(coverageVCC) * 100) + "+-"
                + to_string_with_precision(testVCC.calculateStandardDeviation(coverageVCC) * 100) << std::endl << std::endl;
}


void runAndVisualizeVCC() {
    drake::planning::RobotDiagramBuilder<double> robotDiagramBuilder;
    drake::multibody::MultibodyPlant<double>& plant{ robotDiagramBuilder.plant() };

    std::string projectPath{ std::filesystem::current_path().parent_path().string() };

    drake::multibody::Parser parser(&plant);
    parser.package_map().Add("assets", projectPath + "/assets");
    parser.AddModels(projectPath + "/scenes/2dofScene0.dmd.yaml");
    plant.Finalize();

    std::unique_ptr<drake::planning::RobotDiagram<double>> diagram{ robotDiagramBuilder.Build() };
    std::unique_ptr<drake::systems::Context<double>> diagramContext{ diagram->CreateDefaultContext() };

    drake::planning::CollisionCheckerParams collisionCheckerParams;
    collisionCheckerParams.model = std::move(diagram);
    collisionCheckerParams.edge_step_size = 0.1;
    collisionCheckerParams.robot_model_instances.push_back(plant.GetModelInstanceByName("2dofPlanarArm"));

    std::unique_ptr<drake::planning::CollisionChecker> collisionChecker{
        std::make_unique<drake::planning::SceneGraphCollisionChecker>(std::move(collisionCheckerParams))
    };


    drake::planning::IrisFromCliqueCoverOptions irisFromCliqueCoverOptions;
    irisFromCliqueCoverOptions.coverage_termination_threshold = 0.8;
    irisFromCliqueCoverOptions.num_points_per_visibility_round = 500;
    irisFromCliqueCoverOptions.num_points_per_coverage_check = 5000;
    irisFromCliqueCoverOptions.minimum_clique_size = 10;

    drake::RandomGenerator randomGenerator(1337);
    std::vector<drake::geometry::optimization::HPolyhedron> sets;

    drake::planning::IrisInConfigurationSpaceFromCliqueCover(
        *collisionChecker,
        irisFromCliqueCoverOptions,
        &randomGenerator,
        &sets
    );


    std::srand(0);
    GBurIRIS::visualization::Figure figure;
    figure.visualize2dConfigurationSpace(*collisionChecker, 250);
    for (const auto& set : sets) {
        figure.visualize2dConvexSet(*collisionChecker, set, 250);
    }

    GBurIRIS::visualization::Figure::showFigures();
}


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


void runAndVisualizeGBurIRISIndividConfigs() {
    drake::planning::RobotDiagramBuilder<double> robotDiagramBuilder;
    drake::multibody::MultibodyPlant<double>& plant{ robotDiagramBuilder.plant() };

    std::string projectPath{ std::filesystem::current_path().parent_path().string() };

    drake::multibody::Parser parser(&plant);
    parser.package_map().Add("assets", projectPath + "/assets");
    parser.AddModels(projectPath + "/scenes/2dofScene0.dmd.yaml");
    plant.Finalize();

    std::unique_ptr<drake::planning::RobotDiagram<double>> diagram{ robotDiagramBuilder.Build() };
    std::unique_ptr<drake::systems::Context<double>> diagramContext{ diagram->CreateDefaultContext() };

    drake::planning::CollisionCheckerParams collisionCheckerParams;
    collisionCheckerParams.model = std::move(diagram);
    collisionCheckerParams.edge_step_size = 0.1;
    collisionCheckerParams.robot_model_instances.push_back(plant.GetModelInstanceByName("2dofPlanarArm"));

    std::unique_ptr<drake::planning::CollisionChecker> collisionChecker{
        std::make_unique<drake::planning::SceneGraphCollisionChecker>(std::move(collisionCheckerParams))
    };

    std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>> jointChildAndEndEffectorLinks;
    for (const std::string& rigidBodyName : {"2dofPlanarLink1", "2dofPlanarLink2", "2dofPlanarEndEffector"}) {
        jointChildAndEndEffectorLinks.push_back(plant.GetBodyByName(rigidBodyName, plant.GetModelInstanceByName("2dofPlanarArm")));
    }

    std::vector<double> linkGeometryCompensation(2, 0.1);

    GBurIRIS::robots::PlanarArm planarArm{
        *collisionChecker,
        jointChildAndEndEffectorLinks,
        linkGeometryCompensation
    };

    auto domain{
        drake::geometry::optimization::HPolyhedron::MakeBox(
            plant.GetPositionLowerLimits(),
            plant.GetPositionUpperLimits()
        )
    };

    drake::RandomGenerator drakeRandomGenerator(0);
    RandomGenerator randomGenerator(domain, drakeRandomGenerator);

    GBurIRIS::GBur::GeneralizedBur gBur{
        Eigen::Vector2d{ -0.5, -0.5 },
        GBurIRIS::GBur::GeneralizedBurConfig{ 20, 2, 0.00001, 0.01 },
        planarArm,
        std::bind(&RandomGenerator::randomConfig, &randomGenerator)
    };

    auto [burRandomConfigs, layers]{ gBur.calculateBur() };

    std::vector<Eigen::VectorXd> outerLayer;
    for (auto&& spine : layers) {
        outerLayer.push_back(*(spine.end() - 1));
    }

    auto ellipsoid{ GBurIRIS::MinVolumeEllipsoid(*collisionChecker, outerLayer) };

    auto polytope{ GBurIRIS::InflatePolytope(*collisionChecker, ellipsoid) };

    int numOfSamples{ 250 };
    GBurIRIS::visualization::Figure figure;
    figure.visualize2dConfigurationSpace(*collisionChecker, numOfSamples);

    figure.visualize2dConvexSet(
        *collisionChecker,
        polytope,
        numOfSamples,
        std::make_tuple(0, 1, 1, 1)
    );

    figure.visualize2dConvexSet(
        *collisionChecker,
        ellipsoid,
        numOfSamples,
        std::make_tuple(0.5, 0.5, 0.5, 1)
    );

    figure.visualize2dGeneralizedBur(
        *collisionChecker,
        gBur,
        numOfSamples,
        std::vector<std::tuple<double, double, double, double>>{
            {0.75, 0, 0, 1},
            {0, 0.75, 0, 1},
            {0, 0, 0.75, 1}
        }
    );

    figure.showFigures();
}


Eigen::MatrixXd genRandRotMat(const int matrixSize) {
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


void runAndVisualizeGBurIRISRotationMatrix() {
    drake::planning::RobotDiagramBuilder<double> robotDiagramBuilder;
    drake::multibody::MultibodyPlant<double>& plant{ robotDiagramBuilder.plant() };

    std::string projectPath{ std::filesystem::current_path().parent_path().string() };

    drake::multibody::Parser parser(&plant);
    parser.package_map().Add("assets", projectPath + "/assets");
    parser.AddModels(projectPath + "/scenes/2dofScene0.dmd.yaml");
    plant.Finalize();

    std::unique_ptr<drake::planning::RobotDiagram<double>> diagram{ robotDiagramBuilder.Build() };
    std::unique_ptr<drake::systems::Context<double>> diagramContext{ diagram->CreateDefaultContext() };

    drake::planning::CollisionCheckerParams collisionCheckerParams;
    collisionCheckerParams.model = std::move(diagram);
    collisionCheckerParams.edge_step_size = 0.1;
    collisionCheckerParams.robot_model_instances.push_back(plant.GetModelInstanceByName("2dofPlanarArm"));

    std::unique_ptr<drake::planning::CollisionChecker> collisionChecker{
        std::make_unique<drake::planning::SceneGraphCollisionChecker>(std::move(collisionCheckerParams))
    };

    std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>> jointChildAndEndEffectorLinks;
    for (const std::string& rigidBodyName : {"2dofPlanarLink1", "2dofPlanarLink2", "2dofPlanarEndEffector"}) {
        jointChildAndEndEffectorLinks.push_back(plant.GetBodyByName(rigidBodyName, plant.GetModelInstanceByName("2dofPlanarArm")));
    }

    std::vector<double> linkGeometryCompensation(2, 0.1);

    GBurIRIS::robots::PlanarArm planarArm{
        *collisionChecker,
        jointChildAndEndEffectorLinks,
        linkGeometryCompensation
    };

    auto domain{
        drake::geometry::optimization::HPolyhedron::MakeBox(
            plant.GetPositionLowerLimits(),
            plant.GetPositionUpperLimits()
        )
    };

    drake::RandomGenerator drakeRandomGenerator(0);
    RandomGenerator randomGenerator(domain, drakeRandomGenerator);

    GBurIRIS::GBur::GeneralizedBur gBur{
        Eigen::Vector2d{ -0.5, -0.5 },
        GBurIRIS::GBur::GeneralizedBurConfig{ 4, 2, 0.00001, 0.01 },
        planarArm,
        genRandRotMat(2)
    };

    auto [burRandomConfigs, layers]{ gBur.calculateBur() };

    std::vector<Eigen::VectorXd> outerLayer;
    for (auto&& spine : layers) {
        outerLayer.push_back(*(spine.end() - 1));
    }

    auto ellipsoid{ GBurIRIS::MinVolumeEllipsoid(*collisionChecker, outerLayer) };

    auto polytope{ GBurIRIS::InflatePolytope(*collisionChecker, ellipsoid) };

    int numOfSamples{ 250 };
    GBurIRIS::visualization::Figure figure;
    figure.visualize2dConfigurationSpace(*collisionChecker, numOfSamples);

    figure.visualize2dConvexSet(
        *collisionChecker,
        polytope,
        numOfSamples,
        std::make_tuple(0, 1, 1, 1)
    );

    figure.visualize2dConvexSet(
        *collisionChecker,
        ellipsoid,
        numOfSamples,
        std::make_tuple(0.5, 0.5, 0.5, 1)
    );

    figure.visualize2dGeneralizedBur(
        *collisionChecker,
        gBur,
        numOfSamples,
        std::vector<std::tuple<double, double, double, double>>{
            {0.75, 0, 0, 1},
            {0, 0.75, 0, 1},
            {0, 0, 0.75, 1}
        }
    );

    figure.showFigures();
}


int main() {

//     runTests(RobotType::Planar2DofArm, ObstacleCount::EightOrMore, 10);
//     runAndVisualizeVCC();
//     runAndVisualizeGBurIRISIndividConfigs();
    runAndVisualizeGBurIRISRotationMatrix();

    return 0;
}
