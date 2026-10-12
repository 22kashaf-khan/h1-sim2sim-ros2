#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "h1_mujoco_bridge/h1_sim.hpp"

namespace
{

double joint_value(
  const h1_mujoco_bridge::H1Sim & sim, const std::vector<double> & values,
  const std::string & name)
{
  const std::vector<std::string> & names = sim.joint_names();
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i] == name) {
      return values[i];
    }
  }
  return NAN;
}

}  // namespace

TEST(H1Pd, JointMovesToTargetWithoutGravity)
{
  h1_mujoco_bridge::H1Sim sim(H1_SCENE, 0.005);
  sim.model().opt.gravity[2] = 0.0;
  sim.reset({"left_knee"}, {0.79}, 2.0);
  sim.set_gains({"left_knee"}, {200.0}, {5.0});
  sim.set_targets({"left_knee"}, {1.0});

  sim.step(200);

  EXPECT_NEAR(sim.time(), 1.0, 1e-9);
  EXPECT_NEAR(joint_value(sim, sim.joint_positions(), "left_knee"), 1.0, 0.01);
}

TEST(H1Pd, TorqueIsLimitedToTheConfiguredRange)
{
  h1_mujoco_bridge::H1Sim sim(H1_SCENE, 0.005);
  sim.set_gains({"left_knee"}, {200.0}, {5.0});
  sim.set_torque_limits({"left_knee"}, {1.0});
  sim.set_targets({"left_knee"}, {2.0});

  sim.step(1);

  const int actuator = mj_name2id(&sim.model(), mjOBJ_ACTUATOR, "left_knee");
  EXPECT_DOUBLE_EQ(sim.data().ctrl[actuator], 1.0);
  EXPECT_DOUBLE_EQ(joint_value(sim, sim.torque_limits(), "left_knee"), 1.0);
}

TEST(H1Pd, ModelTorqueLimitsAreTheMenagerieValues)
{
  h1_mujoco_bridge::H1Sim sim(H1_SCENE, 0.005);
  const std::vector<double> limits = sim.torque_limits();
  EXPECT_DOUBLE_EQ(joint_value(sim, limits, "left_knee"), 300.0);
  EXPECT_DOUBLE_EQ(joint_value(sim, limits, "left_ankle"), 40.0);
  EXPECT_DOUBLE_EQ(joint_value(sim, limits, "left_elbow"), 18.0);
}