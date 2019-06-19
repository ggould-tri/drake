#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {

// We make an interesting world (from a multibody topological perspective)
// with two Atlas, a table, and a mug.
class MultibodyPlantFixture: public ::testing::Test {
 public:
  void SetUp() override {
    atlas_path =
        FindResourceOrThrow("drake/examples/atlas/urdf/atlas_convex_hull.urdf");

    table_sdf_path = FindResourceOrThrow(
        "drake/examples/kuka_iiwa_arm/models/table/"
        "extra_heavy_duty_table_surface_only_collision.sdf");

    mug_sdf_path =
        FindResourceOrThrow("drake/examples/simple_gripper/simple_mug.sdf");

    // Load a model of a table for the environment around the robot.
    multibody::Parser parser(&plant, &scene_graph);
    robot_table_model = parser.AddModelFromFile(table_sdf_path, "robot_table");
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("link", robot_table_model));

    // Load two Atlas robots.
    atlas_model1 = parser.AddModelFromFile(atlas_path, "Atlas1");
    atlas_model2 = parser.AddModelFromFile(atlas_path, "Atlas2");
    pelvis1 = &plant.GetBodyByName("pelvis", atlas_model1);
    pelvis2 = &plant.GetBodyByName("pelvis", atlas_model2);

    // Add a floating mug.
    mug_model = parser.AddModelFromFile(mug_sdf_path);
    mug = &plant.GetBodyByName("main_body", mug_model);
  }

 protected:
  std::string atlas_path;
  std::string table_sdf_path;
  std::string mug_sdf_path;
  MultibodyPlant<double> plant;
  geometry::SceneGraph<double> scene_graph;
  ModelInstanceIndex robot_table_model;
  ModelInstanceIndex atlas_model1;
  ModelInstanceIndex atlas_model2;
  const Body<double>* pelvis1;
  const Body<double>* pelvis2;
  ModelInstanceIndex mug_model;
  const Body<double>* mug;
};

}  // namespace multibody
}  // namespace drake
