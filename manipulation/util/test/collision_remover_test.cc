#include "drake/manipulation/util/collision_remover.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace manipulation {
namespace util {
namespace {

/// This test is based on a URDF which has a rod on a revolute joint, a box on
/// a prismatic joint, and a free body sphere.
///
///     ^^^      -- ------------
///    joint    /  |\       box |
///   *==============|          |  --> joint
///     rod     \  |/           |
///              -- ------------
///             sphere
///
/// When all three are at their zero positions, they collide with each other;
/// all three can adjust out of collision.
const char* urdf_file =
    "drake/manipulation/util/test/collision_remover_test.urdf";

class CollisionRemoverTest : public ::testing::Test {
 protected:
  void SetUp() override {
    plant = builder.template AddSystem<drake::multibody::MultibodyPlant>(1e-3);
    plant->set_name("plant");

    const std::string urdf_name = FindResourceOrThrow(urdf_file);
    multibody::Parser parser(plant);
    parser.AddModelFromFile(urdf_name, "collision_remover_test");

    scene_graph = builder.template AddSystem<drake::geometry::SceneGraph>();
    scene_graph->set_name("scene_graph");
    plant->RegisterAsSourceForSceneGraph(scene_graph);
    plant->Finalize();

    builder.Connect(
        plant->get_geometry_poses_output_port(),
        scene_graph->get_source_pose_port(plant->get_source_id().value()));
    builder.Connect(
        scene_graph->get_query_output_port(),
        plant->get_geometry_query_input_port());
    diagram = builder.Build();

    root_context = diagram->CreateDefaultContext();
    plant_context =
        &diagram->GetMutableSubsystemContext(*plant, root_context.get());

    dut = std::make_unique<CollisionRemover>(diagram.get(), plant, scene_graph);
  }

  drake::geometry::SceneGraph<double>* scene_graph;
  drake::systems::DiagramBuilder<double> builder;
  std::unique_ptr<drake::systems::Diagram<double>> diagram;
  drake::multibody::MultibodyPlant<double>* plant;
  std::unique_ptr<drake::systems::Context<double>> root_context;
  drake::systems::Context<double>* plant_context;
  std::unique_ptr<CollisionRemover> dut;
};

TEST_F(CollisionRemoverTest, SmokeTest) {
  // Do nothing; test that the fixture can life-cycle the object.
}

TEST_F(CollisionRemoverTest, RevoluteTest) {
}

}  // namespace
}  // namespace util
}  // namespace manipulation
}  // namespace drake
