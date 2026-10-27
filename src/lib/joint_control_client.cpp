#include "advrf_ros2_io/joint_control_client.hpp"
#include <cmath>
#include <algorithm>
#include <future>
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
  power_timeout_ = node.declare_parameter("power_timeout", 4.0);
  const double rate = node.declare_parameter("publish_rate", 50.0);
  if (!std::isfinite(rate) || rate <= 0 || !std::isfinite(power_timeout_) || power_timeout_ <= 0)
    throw std::invalid_argument("publish_rate and power_timeout must be finite and positive");
  publisher_ = node.create_publisher<sensor_msgs::msg::JointState>("joint_commands", rclcpp::QoS(1));
  selection_ = node.create_publisher<sensor_msgs::msg::JointState>("selected_joint", rclcpp::QoS(1).transient_local());
  power_ = node.create_client<Power>("motor_power_request");
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
        if (pending_) throw std::runtime_error("Motor power request in progress");
        if (publisher_->get_subscription_count() == 0)
          throw std::runtime_error("No joint_commands subscriber");
        sensor_msgs::msg::JointState command;
        command.header.stamp = node_.now();
        command.name = {joint};
        command.position = {position};
        publisher_->publish(command);
      }, [this](const std::string &joint, bool on) { power(joint, on); });
}

const std::string &JointControlClient::selected_joint() const
{
  static const std::string empty;
  return control_ ? control_->selected_joint() : empty;
}

JointControlClient::~JointControlClient()
{
  timer_->cancel();
  if (pending_) power_->remove_pending_request(*pending_);
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

void JointControlClient::power(const std::string &joint, bool on)
{
  if (pending_) throw std::runtime_error("Motor power request in progress");
  if (!power_->service_is_ready()) throw std::runtime_error("motor_power_request unavailable");
  auto request = std::make_shared<Power::Request>();
  request->joint = joint;
  request->action = on ? Power::Request::ACTION_POWER_ON : Power::Request::ACTION_POWER_OFF;
  pending_.emplace(power_->async_send_request(request));
  pending_joint_ = joint;
  power_sent_ = JointControl::Clock::now();
}

void JointControlClient::tick()
{
  const auto now = JointControl::Clock::now();
  const double dt = std::chrono::duration<double>(now - last_tick_).count();
  last_tick_ = now;
  if (!control_) return;
  if (pending_)
  {
    if (pending_->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      try
      {
        const auto response = pending_->get();
        if (response && response->success)
          RCLCPP_INFO(node_.get_logger(), "%s: %s", pending_joint_.c_str(), response->message.c_str());
        else
          RCLCPP_ERROR(node_.get_logger(), "%s: %s (outcome_unknown=%d)", pending_joint_.c_str(),
                       response ? response->message.c_str() : "Empty power response",
                       response ? response->outcome_unknown : true);
      }
      catch (const std::exception &error) { RCLCPP_ERROR(node_.get_logger(), "Power response: %s", error.what()); }
      pending_.reset();
      control_->reset_motion();
    }
    else if (std::chrono::duration<double>(now - power_sent_).count() >= power_timeout_)
    {
      power_->remove_pending_request(*pending_);
      pending_.reset();
      control_->reset_motion();
      RCLCPP_ERROR(node_.get_logger(), "%s: power timeout, outcome unknown; no retry", pending_joint_.c_str());
    }
    return;
  }
  try { control_->tick(dt, now); }
  catch (const std::exception &error)
  {
    control_->reset_motion();
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000, "%s", error.what());
  }
}
} // namespace advrf_ros2_io
