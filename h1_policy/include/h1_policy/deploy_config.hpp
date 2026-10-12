#ifndef H1_POLICY__DEPLOY_CONFIG_HPP_
#define H1_POLICY__DEPLOY_CONFIG_HPP_

#include <string>
#include <vector>

namespace h1_policy
{

struct ObservationTerm
{
  std::string name;
  int size;
};

struct DeployConfig
{
  std::string task;
  double physics_dt;
  int decimation;
  double control_period;
  double action_scale;
  int observation_dim;
  int action_dim;
  std::vector<ObservationTerm> observation_terms;
  std::vector<std::string> joint_names;
  std::vector<double> default_joint_pos;
  std::vector<double> stiffness;
  std::vector<double> damping;
};

DeployConfig load_deploy_config(const std::string & path);

}  // namespace h1_policy

#endif  // H1_POLICY__DEPLOY_CONFIG_HPP_