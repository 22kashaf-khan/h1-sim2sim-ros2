#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

#include "h1_mujoco_bridge/h1_sim.hpp"

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode()
  : Node("mujoco_bridge")
  {
    const std::string scene = declare_parameter<std::string>("scene", "");
    const std::string deploy_path = declare_parameter<std::string>("deploy_config", "");
    const double base_height = declare_parameter<double>("base_height", 1.05);
    const std::vector<double> torque_limits =
      declare_parameter<std::vector<double>>("torque_limits", std::vector<double>{});
    if (scene.empty() || deploy_path.empty()) {
      throw std::runtime_error("parameters 'scene' and 'deploy_config' are required");
    }

    const YAML::Node deploy = YAML::LoadFile(deploy_path);
    const auto names = deploy["joint_names"].as<std::vector<std::string>>();
    const auto default_pos = deploy["default_joint_pos"].as<std::vector<double>>();
    const auto kp = deploy["stiffness"].as<std::vector<double>>();
    const auto kd = deploy["damping"].as<std::vector<double>>();
    const double physics_dt = deploy["physics_dt"].as<double>();
    decimation_ = deploy["decimation"].as<int>();

    sim_ = std::make_unique<h1_mujoco_bridge::H1Sim>(scene, physics_dt);
    sim_->reset(names, default_pos, base_height);
        const double start_height = sim_->lower_to_ground(0.001);
    sim_->set_gains(names, kp, kd);
    if (!torque_limits.empty()) {
      sim_->set_torque_limits(names, torque_limits);
    }
    RCLCPP_INFO(
      get_logger(), "H1 loaded: dt %.3f s x %d steps, torque limits from %s, start height %.3f m",
      physics_dt, decimation_, torque_limits.empty() ? "model" : "parameter", start_height);


    joint_msg_.name = sim_->joint_names();

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());

    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_command", rclcpp::QoS(1),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_command(*msg);});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(physics_dt * decimation_), [this]() {on_tick();});
  }

private:
  void on_command(const sensor_msgs::msg::JointState & msg)
  {
    started_ = true;
    try {
      sim_->set_targets(msg.name, msg.position);
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "ignoring joint_command: %s", error.what());
    }
  }

    void on_tick()
  {
    if (started_) {
      sim_->step(decimation_);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "holding still until the first joint_command");
    }
    publish_state();
  }

  void publish_state()
  {
    const rclcpp::Time stamp = now();

    joint_msg_.header.stamp = stamp;
    joint_msg_.position = sim_->joint_positions();
    joint_msg_.velocity = sim_->joint_velocities();
    joint_pub_->publish(joint_msg_);

    const std::array<double, 4> q = sim_->base_quat();
    const std::array<double, 3> w = sim_->base_ang_vel_body();
    const std::array<double, 3> v = sim_->base_lin_vel_body();

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "pelvis";
    imu.orientation.w = q[0];
    imu.orientation.x = q[1];
    imu.orientation.y = q[2];
    imu.orientation.z = q[3];
    imu.angular_velocity.x = w[0];
    imu.angular_velocity.y = w[1];
    imu.angular_velocity.z = w[2];
    imu_pub_->publish(imu);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "world";
    odom.child_frame_id = "pelvis";
    odom.pose.pose.position.x = sim_->data().qpos[0];
    odom.pose.pose.position.y = sim_->data().qpos[1];
    odom.pose.pose.position.z = sim_->data().qpos[2];
    odom.pose.pose.orientation = imu.orientation;
    odom.twist.twist.linear.x = v[0];
    odom.twist.twist.linear.y = v[1];
    odom.twist.twist.linear.z = v[2];
    odom.twist.twist.angular = imu.angular_velocity;
    odom_pub_->publish(odom);
  }

  std::unique_ptr<h1_mujoco_bridge::H1Sim> sim_;
  int decimation_ = 1;
bool started_ = false;
  sensor_msgs::msg::JointState joint_msg_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeNode>());
  rclcpp::shutdown();
  return 0;
}