#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "h1_mujoco_bridge/h1_sim.hpp"

TEST(H1Sim, LoadsWithRequestedTimestepAndJoints)
{
  h1_mujoco_bridge::H1Sim sim(H1_SCENE, 0.005);
  ASSERT_EQ(sim.joint_names().size(), 19u);
  EXPECT_EQ(sim.joint_names().front(), "left_hip_yaw");
  EXPECT_EQ(sim.joint_names().back(), "right_elbow");
}

TEST(H1Sim, ResetPlacesRobotInRequestedPose)
{
  h1_mujoco_bridge::H1Sim sim(H1_SCENE, 0.005);
  sim.reset({"left_knee", "right_ankle"}, {0.79, -0.52}, 1.05);

  const std::vector<std::string> & names = sim.joint_names();
  const std::vector<double> q = sim.joint_positions();
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i] == "left_knee") {
      EXPECT_DOUBLE_EQ(q[i], 0.79);
    } else if (names[i] == "right_ankle") {
      EXPECT_DOUBLE_EQ(q[i], -0.52);
    } else {
      EXPECT_DOUBLE_EQ(q[i], 0.0) << names[i];
    }
  }
  EXPECT_DOUBLE_EQ(sim.data().qpos[2], 1.05);
  EXPECT_DOUBLE_EQ(sim.base_quat()[0], 1.0);
}

TEST(H1Sim, ResetRejectsUnknownJoint)
{
  h1_mujoco_bridge::H1Sim sim(H1_SCENE, 0.005);
  EXPECT_THROW(sim.reset({"left_wing"}, {0.0}, 1.05), std::invalid_argument);
}

TEST(H1Sim, LinearVelocityIsReportedInBodyFrame)
{
  h1_mujoco_bridge::H1Sim sim(H1_SCENE, 0.005);
  sim.reset({}, {}, 1.05);

  // Turn the robot 90 degrees left (yaw), then move it along world +x.
  const double half = M_PI / 4.0;
  mjData & data = sim.data();
  data.qpos[3] = std::cos(half);
  data.qpos[4] = 0.0;
  data.qpos[5] = 0.0;
  data.qpos[6] = std::sin(half);
  data.qvel[0] = 1.0;

  // Facing world +y, a motion along world +x is to the robot's right: body -y.
  const std::array<double, 3> v = sim.base_lin_vel_body();
  EXPECT_NEAR(v[0], 0.0, 1e-12);
  EXPECT_NEAR(v[1], -1.0, 1e-12);
  EXPECT_NEAR(v[2], 0.0, 1e-12);
}