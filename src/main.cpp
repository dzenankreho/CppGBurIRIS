#include <iostream>
#include <memory>
#include <tuple>
#include <vector>
#include <functional>
#include <chrono>
#include <filesystem>


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


class DrakeRandomGenerator {
public:
    DrakeRandomGenerator(
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


// #include<random>
// #include<ctime>
//
// static std::default_random_engine e(std::time(0));
// static std::normal_distribution<double> gaussian(0,1);
//
// Eigen::MatrixXd randomOrthogonalMatrix(const unsigned long n){
//   Eigen::MatrixXd X = Eigen::MatrixXd::Zero(n,n).unaryExpr([](double dummy){return gaussian(e);});
//   Eigen::MatrixXd XtX = X.transpose() * X;
//   Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(XtX);
//   Eigen::MatrixXd S = es.operatorInverseSqrt();
//   return X * S;
// }
//
// int main() {
//     constexpr int n{ 4 };
//
//     auto mat{ randomOrthogonalMatrix(n) };
//     std::cout << mat << std::endl << std::endl << mat.determinant() << std::endl << std::endl;
//
//     if (mat.determinant() < 0) {
//         mat.col(0) *= -1;
//     }
//
//     std::cout << mat << std::endl << std::endl << mat.determinant() << std::endl << std::endl;
//
//     return 0;
// }


int main() {
    drake::planning::RobotDiagramBuilder<double> robotDiagramBuilder;
    drake::multibody::MultibodyPlant<double>& plant{ robotDiagramBuilder.plant() };

    std::string projectPath{ std::filesystem::current_path().parent_path().string() };

    drake::multibody::Parser parser(&plant);
    parser.package_map().Add("assets", projectPath + "/assets");
    parser.AddModels(projectPath + "/scenes/2dofScene3.dmd.yaml");
    plant.Finalize();

    // drake::visualization::AddDefaultVisualization(&robotDiagramBuilder.builder());

    std::unique_ptr<drake::planning::RobotDiagram<double>> diagram{ robotDiagramBuilder.Build() };
    std::unique_ptr<drake::systems::Context<double>> diagramContext{ diagram->CreateDefaultContext() };
    // drake::systems::Context<double>& plantContext{ plant.GetMyMutableContextFromRoot(diagramContext.get()) };

    // diagram->ForcedPublish(*diagramContext);

    drake::planning::CollisionCheckerParams collisionCheckerParams;
    collisionCheckerParams.model = std::move(diagram);
    collisionCheckerParams.edge_step_size = 0.1;
    collisionCheckerParams.robot_model_instances.push_back(plant.GetModelInstanceByName("2dofPlanarArm"));

    std::unique_ptr<drake::planning::CollisionChecker> collisionChecker{
        std::make_unique<drake::planning::SceneGraphCollisionChecker>(std::move(collisionCheckerParams))
    };

    std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>> jointChildAndEndEffectorLinks {
        plant.GetBodyByName("2dofPlanarLink1", plant.GetModelInstanceByName("2dofPlanarArm")),
        plant.GetBodyByName("2dofPlanarLink2", plant.GetModelInstanceByName("2dofPlanarArm")),
        plant.GetBodyByName("2dofPlanarEndEffector", plant.GetModelInstanceByName("2dofPlanarArm"))
    };

    std::vector<double> linkGeometryCompensation{ 0.1, 0.1 };

    GBurIRIS::robots::PlanarArm planarArm(*collisionChecker, jointChildAndEndEffectorLinks, linkGeometryCompensation);

//     auto&& config1{ Eigen::Vector2d(0, 3.14) }, config2{ Eigen::Vector2d(0, -1.57) };
//
//     for (auto&& pos : planarArm.getLinkPositions(config1)) {
//         std::cout << pos << std::endl;
//     }
//
//     for (auto&& radius : planarArm.getEnclosingRadii(config1)) {
//         std::cout << radius << " ";
//     }
//     std::cout << std::endl;
//
//     std::cout << planarArm.getMaxDisplacement(config1, config2) << std::endl;


    int numOfRuns{ 10 };

    GBurIRIS::GBurIRISConfig gBurIRISConfig;
    gBurIRISConfig.coverage = 0.9;

    GBurIRIS::testing::TestGBurIRIS testGBurIRIS(planarArm, gBurIRISConfig);
    auto [execTimeGBurIRIS, numOfRegionsGBurIRIS, coverageGBurIRIS] = testGBurIRIS.run(numOfRuns);

    drake::planning::IrisFromCliqueCoverOptions irisFromCliqueCoverOptions;
    irisFromCliqueCoverOptions.coverage_termination_threshold = 0.9;
    irisFromCliqueCoverOptions.num_points_per_visibility_round = 500;
    irisFromCliqueCoverOptions.num_points_per_coverage_check = 5000;
    irisFromCliqueCoverOptions.minimum_clique_size = 10;


    GBurIRIS::testing::TestVCC testVCC(planarArm, irisFromCliqueCoverOptions);
    auto [execTimeVCC, numOfRegionsVCC, coverageVCC] = testVCC.run(numOfRuns);


    std::cout << testGBurIRIS.calculateMean(execTimeGBurIRIS) << " "
              << testGBurIRIS.calculateMean(numOfRegionsGBurIRIS) << " "
              << testGBurIRIS.calculateMean(coverageGBurIRIS) << std::endl
              << testVCC.calculateMean(execTimeVCC) << " "
              << testVCC.calculateMean(numOfRegionsVCC) << " "
              << testVCC.calculateMean(coverageVCC) << std::endl;



/*
    drake::RandomGenerator randomGenerator(0);
    auto domain = drake::geometry::optimization::HPolyhedron::MakeBox(
        plant.GetPositionLowerLimits(),
        plant.GetPositionUpperLimits()
    );
    DrakeRandomGenerator drakeRandomGenerator(domain, randomGenerator);

    GBurIRIS::GBur::GeneralizedBur gBur(
        Eigen::Vector2d(0, 0),
        GBurIRIS::GBur::GeneralizedBurConfig{ 20, 2, 1e-5, 0.01 },
        planarArm,
        std::bind(&DrakeRandomGenerator::randomConfig, &drakeRandomGenerator)
    );


    std::vector<Eigen::VectorXd> randomConfigs {
        Eigen::Vector2d(2.41516081, -12.3320883),
        Eigen::Vector2d(-2.68751996, 12.2756116),
        Eigen::Vector2d(-9.98523574, -7.62944761),
        Eigen::Vector2d(5.83260261, 11.13077492),
        Eigen::Vector2d(-12.30621474,   2.54371429),
        Eigen::Vector2d(10.08631485, -7.49530897),
        Eigen::Vector2d(-7.87107069, -9.79589964),
        Eigen::Vector2d(-6.35049916, 10.84364164),
        Eigen::Vector2d(8.65546941, -9.11022793),
        Eigen::Vector2d(10.22488112, -7.30514954),
        Eigen::Vector2d(-1.92731445, 12.41768347),
        Eigen::Vector2d(11.16758691, -5.76180582),
        Eigen::Vector2d(-11.70043926,  -4.58400751),
        Eigen::Vector2d(-11.23589189,  -5.62744498),
        Eigen::Vector2d(6.5435321 , -10.72826133),
        Eigen::Vector2d(8.66562355, 9.10056988),
        Eigen::Vector2d(-12.0397344 ,  -3.59974987),
        Eigen::Vector2d(-9.61137696,  8.09535895),
        Eigen::Vector2d(9.27088909, -8.48316091),
        Eigen::Vector2d(12.32960736,  2.42779449)
    };

    gBur.setRandomConfigs(randomConfigs);

    auto [randomConfigs_, layers] = gBur.calculateBur();


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
        numOfSamples,e
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

    figure.showFigures();*/



//     robots::Robot robot(*collisionChecker, std::vector<std::shared>);


//     drake::planning::RobotDiagramBuilder<double> robotDiagramBuilder;
//     drake::multibody::MultibodyPlant<double>& plant{ robotDiagramBuilder.plant() };
//
//     drake::multibody::Parser parser(&plant);
//     parser.package_map().Add("assets", "/home/dzenan/Desktop/TestDrake/assets");
//     parser.AddModels("/home/dzenan/Desktop/TestDrake/scenes/2dofScene0.dmd.yaml");
//     plant.Finalize();
//
//     // drake::visualization::AddDefaultVisualization(&robotDiagramBuilder.builder());
//
//     std::unique_ptr<drake::planning::RobotDiagram<double>> diagram{ robotDiagramBuilder.Build() };
//     std::unique_ptr<drake::systems::Context<double>> diagramContext{ diagram->CreateDefaultContext() };
//     // drake::systems::Context<double>& plantContext{ plant.GetMyMutableContextFromRoot(diagramContext.get()) };
//
//     // diagram->ForcedPublish(*diagramContext);
//
//     drake::planning::CollisionCheckerParams collisionCheckerParams;
//     collisionCheckerParams.model = std::move(diagram);
//     collisionCheckerParams.edge_step_size = 0.1;
//     collisionCheckerParams.robot_model_instances.push_back(plant.GetModelInstanceByName("2dofPlanarArm"));
//
//     std::unique_ptr<drake::planning::CollisionChecker> collisionChecker{
//         new drake::planning::SceneGraphCollisionChecker(std::move(collisionCheckerParams))
//     };
//
//     drake::planning::IrisFromCliqueCoverOptions irisFromCliqueCoverOptions;
//     irisFromCliqueCoverOptions.coverage_termination_threshold = 0.8;
//     irisFromCliqueCoverOptions.num_points_per_visibility_round = 500;
//     irisFromCliqueCoverOptions.num_points_per_coverage_check = 5000;
//     irisFromCliqueCoverOptions.minimum_clique_size = 10;
//
//     drake::RandomGenerator randomGenerator(1337);
//     std::vector<drake::geometry::optimization::HPolyhedron> sets;
//
//     drake::planning::IrisInConfigurationSpaceFromCliqueCover(
//         *collisionChecker,
//         irisFromCliqueCoverOptions,
//         &randomGenerator,
//         &sets
//     );
//
//     std::srand(0);
//     visualization::Figure figure;
//     figure.visualize2dConfigurationSpace(*collisionChecker, 250);
//     for (auto&& set : sets) {
//         figure.visualize2dConvexSet(collisionChecker->plant(), set, 250);
//     }
//
//     visualization::Figure::showFigures();


    return 0;
}
