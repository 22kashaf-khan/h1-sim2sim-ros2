#ifndef H1_POLICY__ONNX_POLICY_HPP_
#define H1_POLICY__ONNX_POLICY_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace h1_policy
{

class OnnxPolicy
{
public:
  OnnxPolicy(const std::string & model_path, int observation_dim, int action_dim);

  const std::vector<float> & infer(const std::vector<float> & observation);

private:
  Ort::Env env_;
  Ort::Session session_;
  Ort::MemoryInfo memory_info_;
  std::string input_name_;
  std::string output_name_;
  std::vector<float> input_buffer_;
  std::vector<float> output_buffer_;
  std::array<int64_t, 2> input_shape_;
  std::array<int64_t, 2> output_shape_;
  Ort::Value input_tensor_{nullptr};
  Ort::Value output_tensor_{nullptr};
};

}  // namespace h1_policy

#endif  // H1_POLICY__ONNX_POLICY_HPP_