#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rviz_default_plugins/displays/robot_model/robot_model_display.hpp>

namespace rviz_common::properties {class StringProperty; class ColorProperty;}

namespace advrf_ros2_io
{
class SelectedJointRobot : public rviz_default_plugins::displays::RobotModelDisplay
{
  Q_OBJECT
public:
  SelectedJointRobot();
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
protected:
  void updateHighlight();
  void onEnable() override;
  void onDisable() override;
private Q_SLOTS:
  void subscribeSelection();
private:
  struct Selection {std::mutex mutex; std::string joint;};
  std::shared_ptr<Selection> selection_ = std::make_shared<Selection>();
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rviz_common::properties::StringProperty * selection_topic_;
  rviz_common::properties::ColorProperty * highlight_color_;
  std::string highlighted_link_;
  void restoreColor();
};
}
