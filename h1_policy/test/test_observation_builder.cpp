#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "h1_policy/deploy_config.hpp"
#include "h1_policy/observation_builder.hpp"
#include "h1_policy/onnx_policy.hpp"

namespace
{
const std::string kPolicyDir = POLICY_DIR;
}  // namespace

TEST(ProjectedGravity, UprightPointsStraightDown)
{
  const std::array<double, 3> g = h1_policy::projected_gravity({1.0, 0.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(g[0], 0.0);
  EXPECT_DOUBLE_EQ(g[1], 0.0);
  EXPECT_DOUBLE_EQ(g[2], -1.0);
}

TEST(ProjectedGravity, PitchedForwardGivesPositiveX)
{
  const double angle = 0.3;  // rad, rotation about the body y axis (nose down)
  const std::array<double, 3> g =
    h1_policy::projected_gravity({std::cos(angle / 2), 0.0, std::sin(angle / 2), 0.0});
  EXPECT_NEAR(g[0], std::sin(angle), 1e-12);
  EXPECT_NEAR(g[1], 0.0, 1e-12);
  EXPECT_NEAR(g[2], -std::cos(angle), 1e-12);
}

TEST(ObservationBuilder, StandingCasesMatchPythonAndPolicyOutputs)
{
  const h1_policy::DeployConfig config =
    h1_policy::load_deploy_config(kPolicyDir + "/deploy.yaml");
  h1_policy::ObservationBuilder builder(config);
  h1_policy::OnnxPolicy policy(
    kPolicyDir + "/policy.onnx", config.observation_dim, config.action_dim);

  const std::map<std::string, h1_policy::VelocityCommand> commands = {
    {"standing_still", {0.0, 0.0, 0.0}},
    {"walk_forward", {1.0, 0.0, 0.0}},
    {"turn_left", {0.0, 0.0, 1.0}}};

  h1_policy::RobotState state;
  state.joint_pos = config.default_joint_pos;
  state.joint_vel.assign(config.action_dim, 0.0);
  const std::vector<float> last_action(config.action_dim, 0.0f);

  int checked = 0;
  for (const YAML::Node & test_case : YAML::LoadFile(kPolicyDir + "/test_cases.yaml")["cases"]) {
    const auto command = commands.find(test_case["name"].as<std::string>());
    if (command == commands.end()) {
      continue;
    }
    const std::vector<float> expected_obs = test_case["obs"].as<std::vector<float>>();
    const std::vector<float> expected_actions = test_case["actions"].as<std::vector<float>>();

    const std::vector<float> & obs = builder.build(state, command->second, last_action);
    ASSERT_EQ(obs.size(), expected_obs.size());
    for (size_t i = 0; i < obs.size(); ++i) {
      EXPECT_FLOAT_EQ(obs[i], expected_obs[i]) << command->first << ", obs " << i;
    }

    const std::vector<float> & actions = policy.infer(obs);
    for (size_t i = 0; i < actions.size(); ++i) {
      EXPECT_NEAR(actions[i], expected_actions[i], 1e-5f) << command->first << ", action " << i;
    }
    ++checked;
  }
  EXPECT_EQ(checked, 3);
}