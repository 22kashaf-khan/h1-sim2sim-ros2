#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <onnxruntime_cxx_api.h>

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