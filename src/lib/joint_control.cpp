#include "advrf_ros2_io/joint_control.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace advrf_ros2_io
{
JointControl::JointControl(Config config, PositionSink position)
    : config_(std::move(config)), position_(std::move(position)),
      feedback_(config_.joints.size())
{
  const std::set<std::string> names(config_.joints.begin(), config_.joints.end());
  if (names.empty() || names.count("") || names.size() != config_.joints.size() || !position_)
    throw std::invalid_argument("Unique, nonempty joints and command sinks are required");
  for (double value : {config_.position_step, config_.axis_speed, config_.feedback_timeout, config_.input_timeout})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Rates, steps and timeouts must be positive and finite");
  if (config_.lower_positions.empty() != config_.upper_positions.empty())
    throw std::invalid_argument("Both position limit arrays are required");
  if (!config_.lower_positions.empty())
  {
    if (config_.lower_positions.size() != names.size() || config_.upper_positions.size() != names.size())
      throw std::invalid_argument("Position limits must match joints");
    for (size_t i = 0; i < names.size(); ++i)
      if (!std::isfinite(config_.lower_positions[i]) || !std::isfinite(config_.upper_positions[i]) ||
          config_.lower_positions[i] > config_.upper_positions[i])
        throw std::invalid_argument("Invalid position limits");
  }
}

const std::string &JointControl::selected_joint() const { return config_.joints[selected_]; }

void JointControl::feedback(const std::string &joint, double position, Clock::time_point now)
{
  if (!std::isfinite(position)) return;
  const auto it = std::find(config_.joints.begin(), config_.joints.end(), joint);
  if (it != config_.joints.end()) feedback_[it - config_.joints.begin()] = Feedback{position, now};
}

void JointControl::reset_motion() { axis_ = 0; target_.reset(); }

void JointControl::event(JointEvent event, Clock::time_point now)
{
  switch (event)
  {
  case JointEvent::Next:
    reset_motion();
    selected_ = (selected_ + 1) % config_.joints.size();
    break;
  case JointEvent::Previous:
    reset_motion();
    selected_ = (selected_ + config_.joints.size() - 1) % config_.joints.size();
    break;
  case JointEvent::PositionPlus: move(config_.position_step, now); break;
  case JointEvent::PositionMinus: move(-config_.position_step, now); break;
  case JointEvent::Sync: reset_motion(); break;
  }
}

void JointControl::axis(double value, Clock::time_point now)
{
  if (!std::isfinite(value) || std::abs(value) > 1)
    throw std::invalid_argument("Axis must be finite and in [-1, 1]");
  axis_ = value;
  axis_received_ = now;
  if (value == 0) target_.reset();
}

void JointControl::tick(double dt, Clock::time_point now)
{
  if (!std::isfinite(dt) || dt <= 0 || dt > config_.input_timeout ||
      std::chrono::duration<double>(now - axis_received_).count() > config_.input_timeout)
  {
    reset_motion();
    return;
  }
  if (axis_ != 0) move(axis_ * config_.axis_speed * dt, now);
}

void JointControl::move(double delta, Clock::time_point now)
{
  const auto &feedback = feedback_[selected_];
  if (!feedback || std::chrono::duration<double>(now - feedback->received).count() > config_.feedback_timeout)
  {
    reset_motion();
    throw std::runtime_error("No fresh feedback for " + selected_joint());
  }
  double target = target_.value_or(feedback->position) + delta;
  if (!std::isfinite(target)) throw std::runtime_error("Non-finite position target");
  if (!config_.lower_positions.empty())
    target = std::clamp(target, config_.lower_positions[selected_], config_.upper_positions[selected_]);
  position_(selected_joint(), target);
  target_ = target;
}
} // namespace advrf_ros2_io
