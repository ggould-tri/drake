
#include <iostream>

#include <Eigen/Dense>

#include "drake/Path.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/cascade_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"

using namespace std;
using namespace Drake;
using namespace Eigen;


template <typename ScalarType = double>
class ZeroInput {
  ZeroInput(void) {}

  friend Eigen::Vector3d toEigen(const ZeroInput<ScalarType>& in) {
    return Eigen::Matrix<ScalarType, 3, 1>::Zero();
  }
}


int main(int argc, char* argv[]) {
  SimulationOptions options;

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if (!lcm->good()) return 1;

  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(
      getDrakePath() + "/examples/Sailboat/sailboat.sdf", floating_base_type);

  auto const& tree = rigid_body_sys->getRigidBodyTree();
  RigidBody& world = tree->world();
  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);

  auto zero_input = ZeroInput();
  auto sys_with_zero_input = cascade(zero_input, rigid_body_sys);
  auto sys_with_vis = cascade(sys_with_zero_input, visualizer);

  VectorXd zero_state = VectorXd::Zero(rigid_body_sys->getNumStates());

  runLCM(sys_with_vis, lcm, 0, std::numeric_limits<double>::infinity(),
         zero_state, options);
}
