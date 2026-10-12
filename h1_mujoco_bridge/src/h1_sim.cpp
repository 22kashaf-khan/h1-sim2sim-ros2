#include "h1_mujoco_bridge/h1_sim.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace h1_mujoco_bridge
{

H1Sim::H1Sim(const std::string & scene_path, double timestep)
{
  char error[1000] = "";
  model_.reset(mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error)));
  if (!model_) {
    throw std::runtime_error("failed to load " + scene_path + ": " + error);
  }
  if (model_->njnt < 1 || model_->jnt_type[0] != mjJNT_FREE) {
    throw std::runtime_error(scene_path + ": expected a free joint as joint 0");
  }
  model_->opt.timestep = timestep;
  data_.reset(mj_makeData(model_.get()));

  for (int j = 1; j < model_->njnt; ++j) {
    if (model_->jnt_type[j] != mjJNT_HINGE) {
      continue;
    }
    joint_index_[mj_id2name(model_.get(), mjOBJ_JOINT, j)] = joint_names_.size();
    joint_names_.push_back(mj_id2name(model_.get(), mjOBJ_JOINT, j));
    qpos_adr_.push_back(model_->jnt_qposadr[j]);
    dof_adr_.push_back(model_->jnt_dofadr[j]);
  }
}

void H1Sim::reset(
  const std::vector<std::string> & names, const std::vector<double> & positions,
  double base_height)
{
  if (names.size() != positions.size()) {
    throw std::invalid_argument("names and positions must have the same length");
  }
  mj_resetData(model_.get(), data_.get());
  data_->qpos[2] = base_height;
  for (size_t i = 0; i < names.size(); ++i) {
    const auto it = joint_index_.find(names[i]);
    if (it == joint_index_.end()) {
      throw std::invalid_argument("unknown joint: " + names[i]);
    }
    data_->qpos[qpos_adr_[it->second]] = positions[i];
  }
  mj_forward(model_.get(), data_.get());
}

std::vector<double> H1Sim::joint_positions() const
{
  std::vector<double> result;
  for (int adr : qpos_adr_) {
    result.push_back(data_->qpos[adr]);
  }
  return result;
}

std::vector<double> H1Sim::joint_velocities() const
{
  std::vector<double> result;
  for (int adr : dof_adr_) {
    result.push_back(data_->qvel[adr]);
  }
  return result;
}

std::array<double, 4> H1Sim::base_quat() const
{
  return {data_->qpos[3], data_->qpos[4], data_->qpos[5], data_->qpos[6]};
}

std::array<double, 3> H1Sim::base_ang_vel_body() const
{
  return {data_->qvel[3], data_->qvel[4], data_->qvel[5]};
}

std::array<double, 3> H1Sim::base_lin_vel_body() const
{
  mjtNum inverse[4];
  mju_negQuat(inverse, data_->qpos + 3);
  mjtNum body[3];
  mju_rotVecQuat(body, data_->qvel, inverse);
  return {body[0], body[1], body[2]};
}

}  // namespace h1_mujoco_bridge