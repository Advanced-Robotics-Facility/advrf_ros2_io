#include <rclcpp/rclcpp.hpp>
#include <advrf_ros2_interfaces/msg/joint_control_event.hpp>
#include <chrono>
#include <optional>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
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
    const std::array<std::string, 4> names{
        "next_button", "previous_button",
        "position_plus_button", "position_minus_button"};
    const std::array<int, 4> defaults{5, 4, -1, -1};
    std::set<int> used;

    for (size_t i = 0; i < names.size(); ++i)
    {
      buttons_[i] = declare_parameter(names[i], defaults[i]);
      if (buttons_[i] < -1 || (buttons_[i] >= 0 && !used.insert(buttons_[i]).second))
      {
        throw std::invalid_argument("Joy buttons must be distinct indices or -1 to disable");
      }
    }
    toggle_active_button_ = declare_parameter("toggle_active_button", -1);
    if (toggle_active_button_ < -1 ||
        (toggle_active_button_ >= 0 && !used.insert(toggle_active_button_).second))
      throw std::invalid_argument("toggle_active_button must be distinct or -1 to disable");
    if (toggle_active_button_ >= 0)
    {
      active_client_ = create_client<SetActive>("ecat/set_active_tx");
      active_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { check_toggle(); });
    }
    active_state_ = create_subscription<std_msgs::msg::Bool>(
        "ecat/active", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr state)
        {
          if (!bridge_active_ || *bridge_active_ != state->data)
          {
            axis_armed_ = false;
            publish_axis(0);
          }
          bridge_active_ = state->data;
        });
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
  using SetActive = std_srvs::srv::SetBool;
  using ActiveClient = rclcpp::Client<SetActive>;

  void toggle_active()
  {
    if (active_request_) return;
    if (!bridge_active_ || !active_client_->service_is_ready())
    {
      RCLCPP_WARN(get_logger(), "Bridge active state/service unavailable; toggle ignored");
      return;
    }
    auto request = std::make_shared<SetActive::Request>();
    request->data = !*bridge_active_;
    active_sent_ = std::chrono::steady_clock::now();
    active_request_.emplace(active_client_->async_send_request(request));
  }

  void check_toggle()
  {
    if (!active_request_) return;
    if (active_request_->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      try
      {
        const auto response = active_request_->get();
        if (response->success) RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        else RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      }
      catch (const std::exception &error) { RCLCPP_WARN(get_logger(), "%s", error.what()); }
      active_request_.reset();
    }
    else if (std::chrono::steady_clock::now() - active_sent_ >= std::chrono::seconds(1))
    {
      active_client_->remove_pending_request(*active_request_);
      active_request_.reset();
      bridge_active_.reset();
      axis_armed_ = false;
      publish_axis(0);
      RCLCPP_WARN(get_logger(), "Bridge toggle timeout; waiting for state, no automatic retry");
    }
  }

  void dispatch(uint8_t action)
  {
    if (action != Event::POSITION_PLUS && action != Event::POSITION_MINUS)
    {
      axis_armed_ = false;
    }

    if ((action == Event::POSITION_PLUS || action == Event::POSITION_MINUS) &&
        !bridge_active_.value_or(false)) return;

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
    const std::array<uint8_t, 4> actions{Event::NEXT, Event::PREVIOUS,
        Event::POSITION_PLUS, Event::POSITION_MINUS};
    if (!first && pressed(message.buttons, toggle_active_button_) &&
        !pressed(previous_buttons_, toggle_active_button_))
    {
      toggle_active();
    }
    else if (!first)
    {
      for (size_t i = 0; i < actions.size(); ++i)
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
    if (!bridge_active_.value_or(false))
    {
      axis_armed_ = false;
      publish_axis(0);
      return;
    }
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

  int toggle_active_button_ = -1;
  std::optional<bool> bridge_active_;
  ActiveClient::SharedPtr active_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_state_;
  rclcpp::TimerBase::SharedPtr active_timer_;
  std::optional<ActiveClient::FutureAndRequestId> active_request_;
  std::chrono::steady_clock::time_point active_sent_;
  std::array<int, 4> buttons_;
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
