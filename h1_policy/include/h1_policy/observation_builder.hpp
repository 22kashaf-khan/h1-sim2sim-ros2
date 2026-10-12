#ifndef H1_POLICY__OBSERVATION_BUILDER_HPP_
#define H1_POLICY__OBSERVATION_BUILDER_HPP_

#include <array>
#include <vector>

#include "h1_policy/deploy_config.hpp"

namespace h1_policy
{

struct RobotState
{
  std::array<double, 3> base_lin_vel{};                 // body frame, m/s
  std::array<double, 3> base_ang_vel{};                 // body frame, rad/s
  std::array<double, 4> base_quat{1.0, 0.0, 0.0, 0.0};  // body to world, (w, x, y, z)
  std::vector<double> joint_pos;                        // policy joint order, rad
  std::vector<double> joint_vel;                        // policy joint order, rad/s
};

struct VelocityCommand
{
  double lin_vel_x = 0.0;
  double lin_vel_y = 0.0;
  double ang_vel_z = 0.0;
};

std::array<double, 3> projected_gravity(const std::array<double, 4> & quat_wxyz);

class ObservationBuilder
{
public:
  explicit ObservationBuilder(const DeployConfig & config);

  const std::vector<float> & build(
    const RobotState & state, const VelocityCommand & command,
    const std::vector<float> & last_action);

private:
  std::vector<double> default_joint_pos_;
  std::vector<float> observation_;
};

}  // namespace h1_policy

#endif  // H1_POLICY__OBSERVATION_BUILDER_HPP_