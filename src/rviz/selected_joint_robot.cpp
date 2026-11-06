#include "advrf_ros2_io/selected_joint_robot.hpp"
#include <QColor>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_joint.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>

namespace advrf_ros2_io
{
SelectedJointRobot::SelectedJointRobot()
{
  selection_topic_ = new rviz_common::properties::StringProperty(
    "Selection Topic", "inputs/selected_joint", "JointState with the selected joint name.",
    this, SLOT(subscribeSelection()));
  highlight_color_ = new rviz_common::properties::ColorProperty(
    "Highlight Color", QColor(255, 150, 0), "Color of the selected joint's child link.", this);
}

void SelectedJointRobot::onInitialize()
{
  RobotModelDisplay::onInitialize();
  subscribeSelection();
}

void SelectedJointRobot::subscribeSelection()
{
  subscription_.reset();
  {
    std::lock_guard<std::mutex> lock(selection_->mutex);
    selection_->joint.clear();
  }
  if (!context_ || !isEnabled()) {return;}
  auto abstraction = context_->getRosNodeAbstraction().lock();
  if (!abstraction) {return;}
  const auto topic = selection_topic_->getStdString();
  if (topic.empty()) {return;}
  try {
    subscription_ = abstraction->get_raw_node()->create_subscription<sensor_msgs::msg::JointState>(
      topic, rclcpp::QoS(1).reliable().transient_local(),
      [selection = selection_](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(selection->mutex);
        selection->joint = message->name.size() == 1 ? message->name.front() : "";
      });
  } catch (const std::exception & error) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Selection", error.what());
  }
}

void SelectedJointRobot::restoreColor()
{
  if (robot_ && !highlighted_link_.empty()) {
    if (auto * link = robot_->getLink(highlighted_link_)) {link->unsetColor();}
  }
  highlighted_link_.clear();
}

void SelectedJointRobot::update(float wall_dt, float ros_dt)
{
  RobotModelDisplay::update(wall_dt, ros_dt);
  updateHighlight();
}

void SelectedJointRobot::updateHighlight()
{
  if (!robot_ || !isEnabled()) {return;}
  std::string selected;
  {
    std::lock_guard<std::mutex> lock(selection_->mutex);
    selected = selection_->joint;
  }
  auto * joint = selected.empty() ? nullptr : robot_->getJoint(selected);
  auto * link = joint ? robot_->getLink(joint->getChildLinkName()) : nullptr;
  const std::string next = link ? link->getName() : "";
  if (next != highlighted_link_) {restoreColor();}
  if (link) {
    const auto color = highlight_color_->getColor();
    // Apply on the GUI thread, also after a RobotModel reload or color edit.
    link->setColor(color.redF(), color.greenF(), color.blueF());
    highlighted_link_ = next;
    setStatus(rviz_common::properties::StatusProperty::Ok, "Selection",
      QString::fromStdString(selected + " → " + next));
  } else {
    setStatus(selected.empty() ? rviz_common::properties::StatusProperty::Ok :
      rviz_common::properties::StatusProperty::Warn, "Selection",
      selected.empty() ? "Waiting for joint selection" : "Selected joint is not in the loaded robot");
  }
}

void SelectedJointRobot::onEnable()
{
  RobotModelDisplay::onEnable();
  subscribeSelection();
}

void SelectedJointRobot::onDisable()
{
  subscription_.reset();
  restoreColor();
  RobotModelDisplay::onDisable();
}
}
PLUGINLIB_EXPORT_CLASS(advrf_ros2_io::SelectedJointRobot, rviz_common::Display)
