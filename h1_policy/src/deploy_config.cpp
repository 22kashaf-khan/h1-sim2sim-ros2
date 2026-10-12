#include "h1_policy/deploy_config.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace h1_policy
{
namespace
{

void validate(const DeployConfig & config, const std::string & path)
{
  int total = 0;
  for (const ObservationTerm & term : config.observation_terms) {
    total += term.size;
  }
  if (total != config.observation_dim) {
    throw std::runtime_error(
      path + ": observation terms sum to " + std::to_string(total) +
      ", expected observation_dim " + std::to_string(config.observation_dim));
  }

  const size_t joints = static_cast<size_t>(config.action_dim);
  if (config.joint_names.size() != joints || config.default_joint_pos.size() != joints ||
    config.stiffness.size() != joints || config.damping.size() != joints)
  {
    throw std::runtime_error(
      path + ": joint lists must all have action_dim (" + std::to_string(joints) + ") entries");
  }
}

}  // namespace

DeployConfig load_deploy_config(const std::string & path)
{
  const YAML::Node yaml = YAML::LoadFile(path);

  DeployConfig config;
  config.task = yaml["task"].as<std::string>();
  config.physics_dt = yaml["physics_dt"].as<double>();
  config.decimation = yaml["decimation"].as<int>();
  config.control_period = yaml["control_period"].as<double>();
  config.action_scale = yaml["action_scale"].as<double>();
  config.observation_dim = yaml["observation_dim"].as<int>();
  config.action_dim = yaml["action_dim"].as<int>();
  for (const YAML::Node & term : yaml["observation_terms"]) {
    config.observation_terms.push_back({term["name"].as<std::string>(), term["size"].as<int>()});
  }
  config.joint_names = yaml["joint_names"].as<std::vector<std::string>>();
  config.default_joint_pos = yaml["default_joint_pos"].as<std::vector<double>>();
  config.stiffness = yaml["stiffness"].as<std::vector<double>>();
  config.damping = yaml["damping"].as<std::vector<double>>();

  validate(config, path);
  return config;
}

}  // namespace h1_policy