#include "h1_policy/onnx_policy.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace h1_policy
{
namespace
{

Ort::SessionOptions make_session_options()
{
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(1);
  return options;
}

}  // namespace

OnnxPolicy::OnnxPolicy(const std::string & model_path, int observation_dim, int action_dim)
: env_(ORT_LOGGING_LEVEL_WARNING, "h1_policy"),
  session_(env_, model_path.c_str(), make_session_options()),
  memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
  input_buffer_(observation_dim, 0.0f),
  output_buffer_(action_dim, 0.0f),
  input_shape_{1, observation_dim},
  output_shape_{1, action_dim}
{
  if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
    throw std::runtime_error(model_path + ": expected exactly one input and one output");
  }

  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session_.GetInputNameAllocated(0, allocator).get();
  output_name_ = session_.GetOutputNameAllocated(0, allocator).get();

  const std::vector<int64_t> model_input =
    session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  const std::vector<int64_t> model_output =
    session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  if (model_input != std::vector<int64_t>(input_shape_.begin(), input_shape_.end()) ||
    model_output != std::vector<int64_t>(output_shape_.begin(), output_shape_.end()))
  {
    throw std::runtime_error(model_path + ": model shapes do not match the deploy config");
  }

  input_tensor_ = Ort::Value::CreateTensor<float>(
    memory_info_, input_buffer_.data(), input_buffer_.size(),
    input_shape_.data(), input_shape_.size());
  output_tensor_ = Ort::Value::CreateTensor<float>(
    memory_info_, output_buffer_.data(), output_buffer_.size(),
    output_shape_.data(), output_shape_.size());
}

const std::vector<float> & OnnxPolicy::infer(const std::vector<float> & observation)
{
  if (observation.size() != input_buffer_.size()) {
    throw std::invalid_argument("observation size does not match the model input");
  }
  std::copy(observation.begin(), observation.end(), input_buffer_.begin());

  const char * input_names[] = {input_name_.c_str()};
  const char * output_names[] = {output_name_.c_str()};
  session_.Run(
    Ort::RunOptions{nullptr}, input_names, &input_tensor_, 1,
    output_names, &output_tensor_, 1);
  return output_buffer_;
}

}  // namespace h1_policy