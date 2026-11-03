#include <rclcpp/rclcpp.hpp>
#include <advrf_ros2_interfaces/msg/joint_control_event.hpp>
#include <chrono>
#include <optional>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <array>
#include <algorithm>
#include <cmath>
#include <set>

namespace advrf_ros2_io
{
class JoyControlNode : public rclcpp::Node
{
public:
  JoyControlNode() : Node("joystick_input_node")
  {
    const std::array<std::string, 6> names{
        "next_button", "previous_button", "power_on_button", "power_off_button",
        "position_plus_button", "position_minus_button"};
    const std::array<int, 6> defaults{5, 4, 0, 1, -1, -1};
    std::set<int> used;

    for (size_t i = 0; i < names.size(); ++i)
    {
      buttons_[i] = declare_parameter(names[i], defaults[i]);
      if (buttons_[i] < -1 || (buttons_[i] >= 0 && !used.insert(buttons_[i]).second))
      {
        throw std::invalid_argument("Joy buttons must be distinct indices or -1 to disable");
      }
    }
    axis_index_ = declare_parameter("position_axis", 1);
    speed_axis_ = declare_parameter("speed_axis", -1);
    if (speed_axis_ < -1)
    {
      throw std::invalid_argument("speed_axis must be an index or -1");
    }

    speed_axis_inverted_ = declare_parameter("speed_axis_inverted", false);
    trigger_axes_ = declare_parameter("position_trigger_axes", false);
    minus_axis_ = declare_parameter("position_minus_axis", -1);
    plus_axis_ = declare_parameter("position_plus_axis", -1);

    if (minus_axis_ < -1 || plus_axis_ < -1 ||
        ((minus_axis_ >= 0) != (plus_axis_ >= 0)) ||
        (minus_axis_ >= 0 && minus_axis_ == plus_axis_))
    {
      throw std::invalid_argument("Split position axes must be two distinct indices, or both -1");
    }

    deadzone_ = declare_parameter("axis_deadzone", 0.15);
    axis_scale_ = declare_parameter("axis_scale", 1.0);
    if (axis_index_ < -1 || !std::isfinite(deadzone_) || deadzone_ < 0 || deadzone_ >= 1 ||
        !std::isfinite(axis_scale_) || std::abs(axis_scale_) != 1)
    {
      throw std::invalid_argument("Invalid axis index, deadzone or scale (must be +/-1)");
    }
    input_timeout_ = declare_parameter("input_timeout", 0.25);
    if (!std::isfinite(input_timeout_) || input_timeout_ <= 0.0)
    {
      throw std::invalid_argument("input_timeout must be finite and positive");
    }

    events_ = create_publisher<Event>("joint_control/events", rclcpp::QoS(10));
    axis_ = create_publisher<std_msgs::msg::Float64>("joint_control/axis", rclcpp::QoS(1));

    joy_ = create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Joy::ConstSharedPtr message)
        {
          joy(*message);
        });
  }

private:
  using Event = advrf_ros2_interfaces::msg::JointControlEvent;

  void dispatch(uint8_t action)
  {
    if (action != Event::POSITION_PLUS && action != Event::POSITION_MINUS)
    {
      axis_armed_ = false;
    }

    Event message;
    message.action = action;
    events_->publish(message);
  }

  void publish_axis(double value)
  {
    std_msgs::msg::Float64 message;
    message.data = value;
    axis_->publish(message);
  }

  void input_axis(double value, double speed = 1.0)
  {
    try
    {
      if (!std::isfinite(speed) || speed < 0 || speed > 1)
      {
        throw std::invalid_argument("Speed outside [0, 1]");
      }

      if (!std::isfinite(value) || std::abs(value) > 1)
      {
        throw std::invalid_argument("Axis outside [-1, 1]");
      }

      if (std::abs(value) <= deadzone_)
      {
        axis_armed_ = true;
        publish_axis(0);
      }
      else if (axis_armed_)
      {
        const double magnitude = (std::abs(value) - deadzone_) / (1.0 - deadzone_);
        publish_axis(speed * axis_scale_ * std::copysign(magnitude, value));
      }
    }
    catch (const std::exception &error)
    {
      axis_armed_ = false;
      publish_axis(0);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", error.what());
    }
  }

  void joy(const sensor_msgs::msg::Joy &message)
  {
    const auto now = std::chrono::steady_clock::now();
    const bool first = !joy_received_ ||
        std::chrono::duration<double>(now - *joy_received_).count() > input_timeout_;
    joy_received_ = now;
    // Reconnection seeds button state, so a held button never becomes a new press.
    if (first)
    {
      previous_buttons_ = message.buttons;
      axis_armed_ = false;
    }

    handle_buttons(message, first);
    handle_axes(message);
  }

  void handle_buttons(const sensor_msgs::msg::Joy &message, bool first)
  {
    const auto pressed = [](const std::vector<int32_t> &buttons, int index)
    {
      return index >= 0 && static_cast<size_t>(index) < buttons.size() && buttons[index] != 0;
    };
    const std::array<uint8_t, 6> actions{Event::NEXT, Event::PREVIOUS, Event::POWER_ON,
        Event::POWER_OFF, Event::POSITION_PLUS, Event::POSITION_MINUS};
    // Power OFF takes priority over other simultaneous buttons.
    if (!first && pressed(message.buttons, buttons_[3]) && !pressed(previous_buttons_, buttons_[3]))
    {
      dispatch(Event::POWER_OFF);
    }
    else if (!first)
    {
      for (size_t i : {size_t(0), size_t(1), size_t(2), size_t(4), size_t(5)})
      {
        if (pressed(message.buttons, buttons_[i]) && !pressed(previous_buttons_, buttons_[i]))
        {
          dispatch(actions[i]);
          break;
        }
      }
    }

    previous_buttons_ = message.buttons;
  }

  void handle_axes(const sensor_msgs::msg::Joy &message)
  {
    double speed = 1.0;
    if (speed_axis_ >= 0)
    {
      if (static_cast<size_t>(speed_axis_) >= message.axes.size() ||
          !std::isfinite(message.axes[speed_axis_]) || std::abs(message.axes[speed_axis_]) > 1)
      {
        publish_axis(0);
        axis_armed_ = false;
        return;
      }
      speed = std::clamp(static_cast<double>(message.axes[speed_axis_]), 0.0, 1.0);
      if (speed_axis_inverted_)
      {
        speed = 1.0 - speed;
      }
    }
    if (minus_axis_ >= 0)
    {
      if (static_cast<size_t>(minus_axis_) >= message.axes.size() ||
          static_cast<size_t>(plus_axis_) >= message.axes.size() ||
          !std::isfinite(message.axes[minus_axis_]) || !std::isfinite(message.axes[plus_axis_]) ||
          std::abs(message.axes[minus_axis_]) > 1 || std::abs(message.axes[plus_axis_]) > 1)
      {
        publish_axis(0);
        axis_armed_ = false;
        return;
      }
      const double minus = trigger_axes_ ? (1.0 - message.axes[minus_axis_]) / 2.0
                                         : std::abs(message.axes[minus_axis_]);
      const double plus = trigger_axes_ ? (1.0 - message.axes[plus_axis_]) / 2.0
                                        : std::abs(message.axes[plus_axis_]);
      if (trigger_axes_ && !axis_armed_ && (minus > deadzone_ || plus > deadzone_))
      {
        publish_axis(0);
        return;
      }
      input_axis(plus - minus, speed);
    }
    else if (axis_index_ >= 0 && static_cast<size_t>(axis_index_) < message.axes.size())
    {
      input_axis(message.axes[axis_index_], speed);
    }
    else
    {
      publish_axis(0);
      axis_armed_ = false;
    }
  }

  std::array<int, 6> buttons_;
  int axis_index_;
  int minus_axis_;
  int plus_axis_;
  int speed_axis_;
  double deadzone_;
  double axis_scale_;
  double input_timeout_;
  bool speed_axis_inverted_;
  bool trigger_axes_;
  bool axis_armed_ = false;
  std::optional<std::chrono::steady_clock::time_point> joy_received_;
  std::vector<int32_t> previous_buttons_;
  rclcpp::Publisher<Event>::SharedPtr events_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr axis_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_;
};
} // namespace advrf_ros2_io

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::spin(std::make_shared<advrf_ros2_io::JoyControlNode>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("joystick_input_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
