#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class PolicyNode : public rclcpp::Node
{
public:
  PolicyNode()
  : Node("policy_node")
  {
    RCLCPP_INFO(get_logger(), "policy_node started");

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joint_state(msg);});

    joint_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_command", rclcpp::QoS(1));

    control_timer_ = create_wall_timer(kControlPeriod, [this]() {on_control_tick();});
  }

private:
  static constexpr std::chrono::milliseconds kControlPeriod{20};

  void on_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    latest_joint_state_ = msg;
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
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;
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