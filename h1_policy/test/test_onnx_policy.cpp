#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "h1_policy/onnx_policy.hpp"

namespace
{
const std::string kPolicyDir = POLICY_DIR;
}  // namespace

TEST(OnnxPolicy, RejectsDimensionsThatDoNotMatchTheModel)
{
  EXPECT_THROW(
    h1_policy::OnnxPolicy(kPolicyDir + "/policy.onnx", 70, 19), std::runtime_error);
}

TEST(OnnxPolicy, RejectsObservationOfWrongSize)
{
  h1_policy::OnnxPolicy policy(kPolicyDir + "/policy.onnx", 69, 19);
  EXPECT_THROW(policy.infer(std::vector<float>(68, 0.0f)), std::invalid_argument);
}

TEST(OnnxPolicy, ReproducesPythonReferenceOutputs)
{
  h1_policy::OnnxPolicy policy(kPolicyDir + "/policy.onnx", 69, 19);
  const YAML::Node cases = YAML::LoadFile(kPolicyDir + "/test_cases.yaml")["cases"];
  ASSERT_GT(cases.size(), 0u);

  float max_diff = 0.0f;
  for (const YAML::Node & test_case : cases) {
    const std::string name = test_case["name"].as<std::string>();
    const std::vector<float> obs = test_case["obs"].as<std::vector<float>>();
    const std::vector<float> expected = test_case["actions"].as<std::vector<float>>();

    const std::vector<float> & actions = policy.infer(obs);
    ASSERT_EQ(actions.size(), expected.size()) << name;
    for (size_t i = 0; i < expected.size(); ++i) {
      EXPECT_NEAR(actions[i], expected[i], 1e-5f) << name << ", action " << i;
      max_diff = std::max(max_diff, std::abs(actions[i] - expected[i]));
    }
  }
  std::cout << "cases: " << cases.size() << ", max |C++ - Python| = " << max_diff << std::endl;
}