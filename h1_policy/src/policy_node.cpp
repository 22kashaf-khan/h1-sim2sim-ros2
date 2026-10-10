#include <memory>

#include <rclcpp/rclcpp.hpp>

class PolicyNode : public rclcpp::Node
{
public:
  PolicyNode()
  : Node("policy_node")
  {
    RCLCPP_INFO(get_logger(), "policy_node started");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolicyNode>());
  rclcpp::shutdown();
  return 0;
}