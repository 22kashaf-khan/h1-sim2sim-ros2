#ifndef H1_MUJOCO_BRIDGE__H1_SIM_HPP_
#define H1_MUJOCO_BRIDGE__H1_SIM_HPP_

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>

namespace h1_mujoco_bridge
{

class H1Sim
{
public:
  H1Sim(const std::string & scene_path, double timestep);

  void reset(
    const std::vector<std::string> & names, const std::vector<double> & positions,
    double base_height);

  const std::vector<std::string> & joint_names() const {return joint_names_;}
  std::vector<double> joint_positions() const;
  std::vector<double> joint_velocities() const;
  std::array<double, 4> base_quat() const;          // body to world, (w, x, y, z)
  std::array<double, 3> base_ang_vel_body() const;  // rad/s
  std::array<double, 3> base_lin_vel_body() const;  // m/s
  double time() const {return data_->time;}

  mjData & data() {return *data_;}

private:
  struct ModelDeleter
  {
    void operator()(mjModel * model) const {mj_deleteModel(model);}
  };
  struct DataDeleter
  {
    void operator()(mjData * data) const {mj_deleteData(data);}
  };

  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> joint_index_;
  std::vector<int> qpos_adr_;
  std::vector<int> dof_adr_;
};

}  // namespace h1_mujoco_bridge

#endif  // H1_MUJOCO_BRIDGE__H1_SIM_HPP_