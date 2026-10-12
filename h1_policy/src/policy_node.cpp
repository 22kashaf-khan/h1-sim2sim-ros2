#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "h1_policy/deploy_config.hpp"
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
    RCLCPP_INFO(
      get_logger(), "loaded %s: %d observations, %d joints, control period %.3f s",
      config_.task.c_str(), config_.observation_dim, config_.action_dim, config_.control_period);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joint_state(msg);});

    joint_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_command", rclcpp::QoS(1));

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(config_.control_period), [this]() {on_control_tick();});
  }

private:
  static constexpr std::chrono::milliseconds kStateTimeout{100};

  void on_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    latest_joint_state_ = msg;
    last_joint_state_time_ = now();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "received joint_states with %zu joints", msg->position.size());
  }

  void on_control_tick()
  {
    if (!latest_joint_state_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "waiting for first joint_states message");
      return;
    }

    const rclcpp::Duration age = now() - last_joint_state_time_;
    if (age > rclcpp::Duration(kStateTimeout)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "joint_states is stale (%.0f ms old), not publishing", age.seconds() * 1000.0);
      return;
    }

    sensor_msgs::msg::JointState command;
    command.header.stamp = now();
    command.name = latest_joint_state_->name;
    command.position = latest_joint_state_->position;
    joint_command_pub_->publish(command);
  }

  h1_policy::DeployConfig config_;
  std::unique_ptr<h1_policy::OnnxPolicy> policy_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;
  rclcpp::Time last_joint_state_time_;
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