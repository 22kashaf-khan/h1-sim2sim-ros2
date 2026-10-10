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
  }

        private:
         void on_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr msg)
     {
       latest_joint_state_ = msg;
       RCLCPP_INFO_THROTTLE(
         get_logger(), *get_clock(), 1000,
         "received joint_states with %zu joints", msg->position.size());
     }

     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
     sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_state_;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolicyNode>());
  rclcpp::shutdown();
  return 0;
}