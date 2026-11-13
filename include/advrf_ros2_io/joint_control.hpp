#pragma once

#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace advrf_ros2_io
{
enum class JointEvent { Next, Previous, PositionPlus, PositionMinus, Sync };

// Input-device and ROS independent. Call from one thread.
class JointControl
{
public:
  using Clock = std::chrono::steady_clock;
  struct Config
  {
    std::vector<std::string> joints;
    double position_step = 0.01;
    double axis_speed = 0.1;
    double feedback_timeout = 0.5;
    double input_timeout = 0.25;
    std::vector<double> lower_positions;
    std::vector<double> upper_positions;
  };
  using PositionSink = std::function<void(const std::string &, double)>;

  JointControl(Config config, PositionSink position);
  const std::string &selected_joint() const;
  void feedback(const std::string &joint, double position, Clock::time_point now = Clock::now());
  void event(JointEvent event, Clock::time_point now = Clock::now());
  void axis(double value, Clock::time_point now = Clock::now());
  void tick(double dt, Clock::time_point now = Clock::now());
  void reset_motion();

private:
  struct Feedback { double position; Clock::time_point received; };
  Config config_;
  PositionSink position_;
  std::vector<std::optional<Feedback>> feedback_;
  size_t selected_ = 0;
  std::optional<double> target_;
  double axis_ = 0;
  Clock::time_point axis_received_;
  void move(double delta, Clock::time_point now);
};
} // namespace advrf_ros2_io
