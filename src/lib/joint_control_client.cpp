#include "advrf_ros2_io/joint_control_client.hpp"
#include <cmath>
#include <algorithm>
#include <set>

namespace advrf_ros2_io
{
JointControlClient::JointControlClient(rclcpp::Node &node) : node_(node)
{
  auto &config = config_;
  config.joints = node.declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  config.position_step = node.declare_parameter("position_step", 0.01);
  config.axis_speed = node.declare_parameter("axis_speed", 0.1);
  config.feedback_timeout = node.declare_parameter("feedback_timeout", 0.5);
  config.input_timeout = node.declare_parameter("input_timeout", 0.25);
  config.lower_positions = node.declare_parameter<std::vector<double>>("lower_positions", std::vector<double>{});
  config.upper_positions = node.declare_parameter<std::vector<double>>("upper_positions", std::vector<double>{});
  const double rate = node.declare_parameter("publish_rate", 50.0);
  if (!std::isfinite(rate) || rate <= 0)
    throw std::invalid_argument("publish_rate must be finite and positive");
  publisher_ = node.create_publisher<sensor_msgs::msg::JointState>("joint_commands", rclcpp::QoS(1));
  selection_ = node.create_publisher<sensor_msgs::msg::JointState>("selected_joint", rclcpp::QoS(1).transient_local());

  if (!config.joints.empty()) initialize();
  else
  {
    if (!config.lower_positions.empty() || !config.upper_positions.empty())
      throw std::invalid_argument("Explicit joints are required with indexed position limits");
    RCLCPP_INFO(node_.get_logger(), "Waiting for joint discovery on rx/joint_states");
  }
  feedback_ = node.create_subscription<sensor_msgs::msg::JointState>("rx/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message)
      {
        if (message->position.size() != message->name.size()) return;
        const std::set<std::string> names(message->name.begin(), message->name.end());
        if (names.empty() || names.count("") || names.size() != message->name.size()) return;
        if (!control_)
        {
          if (!std::all_of(message->position.begin(), message->position.end(),
                           [](double value) { return std::isfinite(value); })) return;
          config_.joints = message->name;
          initialize();
          publish_selection();
        }
        const auto now = JointControl::Clock::now();
        for (size_t i = 0; i < message->name.size(); ++i)
          control_->feedback(message->name[i], message->position[i], now);
      });
  last_tick_ = JointControl::Clock::now();
  timer_ = node.create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { tick(); });
  if (control_) publish_selection();
}

void JointControlClient::initialize()
{
  control_ = std::make_unique<JointControl>(config_,
      [this](const std::string &joint, double position)
      {
        if (publisher_->get_subscription_count() == 0)
          throw std::runtime_error("No joint_commands subscriber");
        sensor_msgs::msg::JointState command;
        command.header.stamp = node_.now();
        command.name = {joint};
        command.position = {position};
        publisher_->publish(command);
      });
}

const std::string &JointControlClient::selected_joint() const
{
  static const std::string empty;
  return control_ ? control_->selected_joint() : empty;
}

JointControlClient::~JointControlClient()
{
  timer_->cancel();
}

void JointControlClient::event(JointEvent event)
{
  if (!control_) throw std::runtime_error("Waiting for joints on rx/joint_states; event ignored");
  control_->event(event);
  if (event == JointEvent::Next || event == JointEvent::Previous) publish_selection();
}

void JointControlClient::axis(double value) { if (control_) control_->axis(value); }

void JointControlClient::publish_selection()
{
  sensor_msgs::msg::JointState selected;
  selected.header.stamp = node_.now();
  selected.name = {control_->selected_joint()};
  selection_->publish(selected);
  RCLCPP_INFO(node_.get_logger(), "Selected joint: %s", control_->selected_joint().c_str());
}

void JointControlClient::tick()
{
  const auto now = JointControl::Clock::now();
  const double dt = std::chrono::duration<double>(now - last_tick_).count();
  last_tick_ = now;
  if (!control_) return;
  try { control_->tick(dt, now); }
  catch (const std::exception &error)
  {
    control_->reset_motion();
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000, "%s", error.what());
  }
}
} // namespace advrf_ros2_io
