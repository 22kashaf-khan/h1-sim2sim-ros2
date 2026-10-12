#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "h1_policy/deploy_config.hpp"

TEST(DeployConfig, LoadsBaselinePolicyConfig)
{
  const h1_policy::DeployConfig config =
    h1_policy::load_deploy_config(std::string(POLICY_DIR) + "/deploy.yaml");

  EXPECT_EQ(config.task, "Isaac-Velocity-Flat-H1-v0");
  EXPECT_EQ(config.observation_dim, 69);
  EXPECT_EQ(config.action_dim, 19);
  EXPECT_EQ(config.observation_terms.size(), 7u);
  EXPECT_DOUBLE_EQ(config.action_scale, 0.5);
  EXPECT_DOUBLE_EQ(config.control_period, 0.02);

  ASSERT_EQ(config.joint_names.size(), 19u);
  EXPECT_EQ(config.joint_names[0], "left_hip_yaw");
  EXPECT_EQ(config.joint_names[2], "torso");
  EXPECT_EQ(config.joint_names[11], "left_knee");
  EXPECT_DOUBLE_EQ(config.default_joint_pos[11], 0.79);
  EXPECT_DOUBLE_EQ(config.stiffness[11], 200.0);
}

TEST(DeployConfig, MissingFileThrows)
{
  EXPECT_THROW(h1_policy::load_deploy_config("does_not_exist.yaml"), std::exception);
}