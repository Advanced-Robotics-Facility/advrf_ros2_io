#include "advrf_ros2_io/joint_control_client.hpp"

#include <advrf_ros2_interfaces/msg/joint_control_event.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <optional>

namespace advrf_ros2_io
{
class JointControlNode : public rclcpp::Node
{
public:
  JointControlNode() : Node("joint_control_node"), client_(*this)
  {
    events_ = create_subscription<Event>(
        "joint_control/events", rclcpp::QoS(10),
        [this](Event::ConstSharedPtr message)
        {
          handle_event(message->action);
        });

    axis_ = create_subscription<std_msgs::msg::Float64>(
        "joint_control/axis", rclcpp::QoS(1),
        [this](std_msgs::msg::Float64::ConstSharedPtr message)
        {
          handle_axis(message->data);
        });
  }

private:
  using Event = advrf_ros2_interfaces::msg::JointControlEvent;

  static std::optional<JointEvent> decode_event(uint8_t action)
  {
    switch (action)
    {
      case Event::NEXT:
        return JointEvent::Next;
      case Event::PREVIOUS:
        return JointEvent::Previous;
      case Event::POWER_ON:
        return JointEvent::PowerOn;
      case Event::POWER_OFF:
        return JointEvent::PowerOff;
      case Event::POSITION_PLUS:
        return JointEvent::PositionPlus;
      case Event::POSITION_MINUS:
        return JointEvent::PositionMinus;
      case Event::SYNC:
        return JointEvent::Sync;
      default:
        return std::nullopt;
    }
  }

  void handle_event(uint8_t action)
  {
    const auto event = decode_event(action);
    if (!event)
    {
      RCLCPP_WARN(get_logger(), "Unknown joint control action: %u", static_cast<unsigned>(action));
      return;
    }

    if (*event != JointEvent::PositionPlus && *event != JointEvent::PositionMinus)
    {
      axis_armed_ = false;
    }

    try
    {
      client_.event(*event);
    }
    catch (const std::exception &error)
    {
      RCLCPP_WARN(get_logger(), "%s", error.what());
    }
  }

  void handle_axis(double value)
  {
    if (!std::isfinite(value) || std::abs(value) > 1.0)
    {
      axis_armed_ = false;
      client_.axis(0.0);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Axis outside [-1, 1]");
      return;
    }

    // Every input adapter must send neutral before it can start a movement.
    if (value == 0.0)
    {
      axis_armed_ = true;
    }

    if (axis_armed_)
    {
      client_.axis(value);
    }
  }

  JointControlClient client_;
  bool axis_armed_ = false;
  rclcpp::Subscription<Event>::SharedPtr events_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr axis_;
};
} // namespace advrf_ros2_io

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::spin(std::make_shared<advrf_ros2_io::JointControlNode>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("joint_control_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
