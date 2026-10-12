#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "h1_policy/deploy_config.hpp"
#include "h1_policy/observation_builder.hpp"
#include "h1_policy/onnx_policy.hpp"

class PolicyNode : public rclcpp::Node
{
public:
  PolicyNode()
  : Node("policy_node")
  {
    const std::string policy_dir = declare_parameter<std::string>("policy_dir", "");
    if (policy_dir.empty()) {
      throw std::runtime_error("parameter 'policy_dir' is required");
    }
    config_ = h1_policy::load_deploy_config(policy_dir + "/deploy.yaml");
    policy_ = std::make_unique<h1_policy::OnnxPolicy>(
      policy_dir + "/policy.onnx", config_.observation_dim, config_.action_dim);
    builder_ = std::make_unique<h1_policy::ObservationBuilder>(config_);

    for (size_t i = 0; i < config_.joint_names.size(); ++i) {
      joint_index_[config_.joint_names[i]] = i;
    }
    state_.joint_pos.assign(config_.action_dim, 0.0);
    state_.joint_vel.assign(config_.action_dim, 0.0);
    last_action_.assign(config_.action_dim, 0.0f);
    command_msg_.name = config_.joint_names;
    command_msg_.position.assign(config_.action_dim, 0.0);

    RCLCPP_INFO(
      get_logger(), "loaded %s: %d observations, %d joints, control period %.3f s",
      config_.task.c_str(), config_.observation_dim, config_.action_dim, config_.control_period);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        latest_joint_state_ = msg;
        last_joint_state_time_ = now();
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        latest_imu_ = msg;
        last_imu_time_ = now();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        latest_odom_ = msg;
        last_odom_time_ = now();
      });
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        command_.lin_vel_x = msg->linear.x;
        command_.lin_vel_y = msg->linear.y;
        command_.ang_vel_z = msg->angular.z;
      });

    joint_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_command", rclcpp::QoS(1));

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(config_.control_period), [this]() {on_control_tick();});
  }

private:
  static constexpr std::chrono::milliseconds kStateTimeout{100};

  void on_control_tick()
  {
    if (!latest_joint_state_ || !latest_imu_ || !latest_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "waiting for joint_states, imu and odom");
      return;
    }
    const rclcpp::Time tick_time = now();
    if (is_stale(last_joint_state_time_, tick_time, "joint_states") ||
      is_stale(last_imu_time_, tick_time, "imu") ||
      is_stale(last_odom_time_, tick_time, "odom"))
    {
      return;
    }
    if (!fill_joints(*latest_joint_state_)) {
      return;
    }

    const auto & q = latest_imu_->orientation;
    state_.base_quat = {q.w, q.x, q.y, q.z};
    const auto & w = latest_imu_->angular_velocity;
    state_.base_ang_vel = {w.x, w.y, w.z};
    const auto & v = latest_odom_->twist.twist.linear;
    state_.base_lin_vel = {v.x, v.y, v.z};

    const std::vector<float> & observation = builder_->build(state_, command_, last_action_);
    const std::vector<float> & action = policy_->infer(observation);
    last_action_ = action;

    command_msg_.header.stamp = tick_time;
    for (size_t i = 0; i < action.size(); ++i) {
      command_msg_.position[i] = config_.default_joint_pos[i] + config_.action_scale * action[i];
    }
    joint_command_pub_->publish(command_msg_);
  }

  bool is_stale(const rclcpp::Time & stamp, const rclcpp::Time & tick_time, const char * topic)
  {
    const rclcpp::Duration age = tick_time - stamp;
    if (age > rclcpp::Duration(kStateTimeout)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s is stale (%.0f ms old), not publishing", topic, age.seconds() * 1000.0);
      return true;
    }
    return false;
  }

  bool fill_joints(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size() || msg.velocity.size() != msg.name.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "joint_states needs position and velocity per name");
      return false;
    }
    size_t found = 0;
    for (size_t k = 0; k < msg.name.size(); ++k) {
      const auto it = joint_index_.find(msg.name[k]);
      if (it == joint_index_.end()) {
        continue;
      }
      state_.joint_pos[it->second] = msg.position[k];
      state_.joint_vel[it->second] = msg.velocity[k];
      ++found;
    }
    if (found != joint_index_.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "joint_states has %zu of the %zu policy joints", found, joint_index_.size());
      return false;
    }
    return true;
  }

  h1_policy::DeployConfig config_;
  std::unique_ptr<h1_policy::OnnxPolicy> policy_;
  std::unique_ptr<h1_policy::ObservationBuilder> builder_;
  std::unordered_map<std::string, size_t> joint_index_;

  h1_policy::RobotState state_;
  h1_policy::VelocityCommand command_;
  std::vector<float> last_action_;
  sensor_msgs::msg::JointState command_msg_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;
  sensor_msgs::msg::Imu::ConstSharedPtr latest_imu_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  rclcpp::Time last_joint_state_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_odom_time_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolicyNode>());
  rclcpp::shutdown();
  return 0;
}