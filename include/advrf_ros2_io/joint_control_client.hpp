#pragma once

#include "joint_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace advrf_ros2_io
{
// ROS bridge to advrf_ros2_motors_controller. Use a single-threaded executor.
// Any input adapter can call event() and axis() on this object.
class JointControlClient
{
public:
  explicit JointControlClient(rclcpp::Node &node);
  ~JointControlClient();
  void event(JointEvent event);
  void axis(double value);
  bool ready() const { return static_cast<bool>(control_); }
  const std::string &selected_joint() const;

private:
  rclcpp::Node &node_;
  JointControl::Config config_;
  std::unique_ptr<JointControl> control_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr selection_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_;
  rclcpp::TimerBase::SharedPtr timer_;
  JointControl::Clock::time_point last_tick_;
  void tick();
  void publish_selection();
  void initialize();
};
} // namespace advrf_ros2_io
