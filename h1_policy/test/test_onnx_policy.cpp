#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

namespace
{
const std::string kModelPath = std::string(POLICY_DIR) + "/policy.onnx";
}  // namespace


TEST(OnnxPolicy, ModelHasExpectedInputAndOutput)
{
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test_onnx_policy");
  Ort::SessionOptions options;
  Ort::Session session(env, kModelPath.c_str(), options);

  ASSERT_EQ(session.GetInputCount(), 1u);
  ASSERT_EQ(session.GetOutputCount(), 1u);

  Ort::AllocatorWithDefaultOptions allocator;
  EXPECT_EQ(std::string(session.GetInputNameAllocated(0, allocator).get()), "obs");
  EXPECT_EQ(std::string(session.GetOutputNameAllocated(0, allocator).get()), "actions");

  const std::vector<int64_t> input_shape =
    session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  const std::vector<int64_t> output_shape =
    session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  EXPECT_EQ(input_shape, (std::vector<int64_t>{1, 69}));
  EXPECT_EQ(output_shape, (std::vector<int64_t>{1, 19}));
}

TEST(OnnxPolicy, ReproducesPythonReferenceOutputs)
{
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test_onnx_policy");
  Ort::SessionOptions options;
  Ort::Session session(env, kModelPath.c_str(), options);

  const YAML::Node cases =
    YAML::LoadFile(std::string(POLICY_DIR) + "/test_cases.yaml")["cases"];
  ASSERT_GT(cases.size(), 0u);

  const Ort::MemoryInfo memory_info =
    Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  const std::array<int64_t, 2> input_shape{1, 69};
  const char * input_names[] = {"obs"};
  const char * output_names[] = {"actions"};

  float max_diff = 0.0f;
  for (const YAML::Node & test_case : cases) {
    const std::string name = test_case["name"].as<std::string>();
    std::vector<float> obs = test_case["obs"].as<std::vector<float>>();
    const std::vector<float> expected = test_case["actions"].as<std::vector<float>>();
    ASSERT_EQ(obs.size(), 69u) << name;

    Ort::Value input = Ort::Value::CreateTensor<float>(
      memory_info, obs.data(), obs.size(), input_shape.data(), input_shape.size());
    std::vector<Ort::Value> outputs = session.Run(
      Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);

    ASSERT_EQ(outputs[0].GetTensorTypeAndShapeInfo().GetElementCount(), expected.size()) << name;
    const float * actions = outputs[0].GetTensorData<float>();

    for (size_t i = 0; i < expected.size(); ++i) {
      EXPECT_NEAR(actions[i], expected[i], 1e-5f) << name << ", action " << i;
      max_diff = std::max(max_diff, std::abs(actions[i] - expected[i]));
    }
  }
  std::cout << "cases: " << cases.size() << ", max |C++ - Python| = " << max_diff << std::endl;
}