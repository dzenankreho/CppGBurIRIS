#include <iostream>
#include <memory>
#include <tuple>
#include <vector>
#include <functional>


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


int main() {
    drake::planning::RobotDiagramBuilder<double> robotDiagramBuilder;
    drake::multibody::MultibodyPlant<double>& plant{ robotDiagramBuilder.plant() };

    drake::multibody::Parser parser(&plant);
    parser.package_map().Add("assets", "/home/dzenan/Desktop/TestDrake/assets");
    parser.AddModels("/home/dzenan/Desktop/TestDrake/scenes/2dofScene0.dmd.yaml");
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

// jointChildAndEndEffectorLinks = [
//     plant.GetBodyByName("2dofPlanarLink1", plant.GetModelInstanceByName("2dofPlanarArm")),
//     plant.GetBodyByName("2dofPlanarLink2", plant.GetModelInstanceByName("2dofPlanarArm")),
//     plant.GetBodyByName("2dofPlanarEndEffector", plant.GetModelInstanceByName("2dofPlanarArm"))
// ]

    std::vector<std::reference_wrapper<const drake::multibody::RigidBody<double>>> jointChildAndEndEffectorLinks {
        plant.GetBodyByName("2dofPlanarLink1", plant.GetModelInstanceByName("2dofPlanarArm")),
        plant.GetBodyByName("2dofPlanarLink2", plant.GetModelInstanceByName("2dofPlanarArm")),
        plant.GetBodyByName("2dofPlanarEndEffector", plant.GetModelInstanceByName("2dofPlanarArm"))
    };

    std::vector<double> linkGeometryCompensation{ 0.1, 0.1 };

    GBurIRIS::robots::PlanarArm planarArm(*collisionChecker, jointChildAndEndEffectorLinks, linkGeometryCompensation);

    auto&& config1{ Eigen::Vector2d(0, 3.14) }, config2{ Eigen::Vector2d(0, -1.57) };

    for (auto&& pos : planarArm.getLinkPositions(config1)) {
        std::cout << pos << std::endl;
    }

    for (auto&& radius : planarArm.getEnclosingRadii(config1)) {
        std::cout << radius << " ";
    }
    std::cout << std::endl;

    std::cout << planarArm.getMaxDisplacement(config1, config2) << std::endl;


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
