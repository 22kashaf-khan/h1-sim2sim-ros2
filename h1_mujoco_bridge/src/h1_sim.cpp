#include "h1_mujoco_bridge/h1_sim.hpp"

#include <algorithm>
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

    int actuator = -1;
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] == mjTRN_JOINT && model_->actuator_trnid[2 * a] == j) {
        actuator = a;
      }
    }
    if (actuator < 0 || model_->actuator_gear[6 * actuator] != 1.0) {
      throw std::runtime_error(
        scene_path + ": joint " + joint_names_.back() + " needs one motor with gear 1");
    }
    actuator_id_.push_back(actuator);
  }

  kp_.assign(joint_names_.size(), 0.0);
  kd_.assign(joint_names_.size(), 0.0);
  target_.assign(joint_names_.size(), 0.0);
}

size_t H1Sim::index_of(const std::string & name) const
{
  const auto it = joint_index_.find(name);
  if (it == joint_index_.end()) {
    throw std::invalid_argument("unknown joint: " + name);
  }
  return it->second;
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
    const size_t j = index_of(names[i]);
    data_->qpos[qpos_adr_[j]] = positions[i];
    target_[j] = positions[i];
  }
  mj_forward(model_.get(), data_.get());
}

double H1Sim::lower_to_ground(double step)
{
  while (data_->qpos[2] > 0.0) {
    data_->qpos[2] -= step;
    mj_forward(model_.get(), data_.get());
    if (data_->ncon > 0) {
      data_->qpos[2] += step;
      mj_forward(model_.get(), data_.get());
      return data_->qpos[2];
    }
  }
  throw std::runtime_error("robot never touched the ground");
}

void H1Sim::set_gains(
  const std::vector<std::string> & names, const std::vector<double> & kp,
  const std::vector<double> & kd)
{
  if (names.size() != kp.size() || names.size() != kd.size()) {
    throw std::invalid_argument("names, kp and kd must have the same length");
  }
  for (size_t i = 0; i < names.size(); ++i) {
    const size_t j = index_of(names[i]);
    kp_[j] = kp[i];
    kd_[j] = kd[i];
  }
}

void H1Sim::set_torque_limits(
  const std::vector<std::string> & names, const std::vector<double> & limits)
{
  if (names.size() != limits.size()) {
    throw std::invalid_argument("names and limits must have the same length");
  }
  for (size_t i = 0; i < names.size(); ++i) {
    const int a = actuator_id_[index_of(names[i])];
    model_->actuator_ctrlrange[2 * a] = -limits[i];
    model_->actuator_ctrlrange[2 * a + 1] = limits[i];
  }
}

void H1Sim::set_targets(
  const std::vector<std::string> & names, const std::vector<double> & targets)
{
  if (names.size() != targets.size()) {
    throw std::invalid_argument("names and targets must have the same length");
  }
  for (size_t i = 0; i < names.size(); ++i) {
    target_[index_of(names[i])] = targets[i];
  }
}

void H1Sim::apply_pd()
{
  for (size_t j = 0; j < joint_names_.size(); ++j) {
    const int a = actuator_id_[j];
    const double q = data_->qpos[qpos_adr_[j]];
    const double qd = data_->qvel[dof_adr_[j]];
    const double torque = kp_[j] * (target_[j] - q) - kd_[j] * qd;
    data_->ctrl[a] = std::clamp(
      torque, model_->actuator_ctrlrange[2 * a], model_->actuator_ctrlrange[2 * a + 1]);
  }
}

void H1Sim::step(int count)
{
  for (int i = 0; i < count; ++i) {
    apply_pd();
    mj_step(model_.get(), data_.get());
  }
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

std::vector<double> H1Sim::torque_limits() const
{
  std::vector<double> result;
  for (int a : actuator_id_) {
    result.push_back(model_->actuator_ctrlrange[2 * a + 1]);
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