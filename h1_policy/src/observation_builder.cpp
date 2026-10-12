#include "h1_policy/observation_builder.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace h1_policy
{

std::array<double, 3> projected_gravity(const std::array<double, 4> & quat_wxyz)
{
  const double w = quat_wxyz[0];
  const double x = quat_wxyz[1];
  const double y = quat_wxyz[2];
  const double z = quat_wxyz[3];
  return {-2.0 * (x * z - w * y), -2.0 * (y * z + w * x), -(1.0 - 2.0 * (x * x + y * y))};
}

ObservationBuilder::ObservationBuilder(const DeployConfig & config)
: default_joint_pos_(config.default_joint_pos),
  observation_(config.observation_dim, 0.0f)
{
  const int n = config.action_dim;
  const std::vector<std::pair<std::string, int>> expected = {
    {"base_lin_vel", 3}, {"base_ang_vel", 3}, {"projected_gravity", 3},
    {"velocity_commands", 3}, {"joint_pos", n}, {"joint_vel", n}, {"actions", n}};

  bool matches = config.observation_terms.size() == expected.size();
  for (size_t i = 0; matches && i < expected.size(); ++i) {
    matches = config.observation_terms[i].name == expected[i].first &&
      config.observation_terms[i].size == expected[i].second;
  }
  if (!matches) {
    throw std::runtime_error("observation layout in deploy config is not the supported one");
  }
}

const std::vector<float> & ObservationBuilder::build(
  const RobotState & state, const VelocityCommand & command,
  const std::vector<float> & last_action)
{
  const size_t n = default_joint_pos_.size();
  if (state.joint_pos.size() != n || state.joint_vel.size() != n || last_action.size() != n) {
    throw std::invalid_argument("joint vectors must have one entry per policy joint");
  }

  size_t i = 0;
  auto put = [this, &i](double value) {observation_[i++] = static_cast<float>(value);};

  for (double v : state.base_lin_vel) {put(v);}
  for (double v : state.base_ang_vel) {put(v);}
  for (double v : projected_gravity(state.base_quat)) {put(v);}
  put(command.lin_vel_x);
  put(command.lin_vel_y);
  put(command.ang_vel_z);
  for (size_t j = 0; j < n; ++j) {put(state.joint_pos[j] - default_joint_pos_[j]);}
  for (size_t j = 0; j < n; ++j) {put(state.joint_vel[j]);}
  for (size_t j = 0; j < n; ++j) {put(last_action[j]);}

  return observation_;
}

}  // namespace h1_policy