#include "advrf_ros2_io/joint_control.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

using namespace advrf_ros2_io;
using namespace std::chrono_literals;

TEST(JointControl, SelectionStepsAndPowerCaptureSelectedJoint)
{
  std::string joint, powered;
  double target = 0;
  bool on = false;
  JointControl control({{"a", "b"}}, [&](auto name, double value) { joint = name; target = value; },
                       [&](auto name, bool value) { powered = name; on = value; });
  const auto now = JointControl::Clock::now();
  control.feedback("a", 1, now);
  control.feedback("b", 2, now);
  control.event(JointEvent::PositionPlus, now);
  EXPECT_EQ(joint, "a");
  EXPECT_DOUBLE_EQ(target, 1.01);
  control.event(JointEvent::PositionPlus, now);
  EXPECT_DOUBLE_EQ(target, 1.02);
  control.event(JointEvent::Previous, now);
  EXPECT_EQ(control.selected_joint(), "b");
  control.event(JointEvent::PositionMinus, now);
  EXPECT_DOUBLE_EQ(target, 1.99);
  control.event(JointEvent::PowerOn, now);
  control.event(JointEvent::Next, now);
  EXPECT_EQ(powered, "b");
  EXPECT_TRUE(on);
  EXPECT_EQ(control.selected_joint(), "a");
  control.event(JointEvent::PowerOff, now);
  EXPECT_EQ(powered, "a");
  EXPECT_FALSE(on);
}

TEST(JointControl, FeedbackWatchdogAndSelectionCancelAxis)
{
  int commands = 0;
  double target = 0;
  JointControl control({{"a", "b"}}, [&](auto, double value) { ++commands; target = value; }, [](auto, bool) {});
  const auto now = JointControl::Clock::now();
  EXPECT_THROW(control.event(JointEvent::PositionPlus, now), std::runtime_error);
  control.feedback("a", 1, now);
  control.axis(1, now);
  control.tick(0.02, now + 20ms);
  EXPECT_DOUBLE_EQ(target, 1.002);
  control.tick(0.02, now + 300ms); // Input expired.
  EXPECT_EQ(commands, 1);
  control.axis(1, now + 300ms);
  control.event(JointEvent::Next, now + 300ms);
  control.tick(0.02, now + 320ms);
  EXPECT_EQ(commands, 1);
  EXPECT_THROW(control.event(JointEvent::PositionPlus, now + 320ms), std::runtime_error);
  control.event(JointEvent::Previous, now + 600ms);
  EXPECT_THROW(control.event(JointEvent::PositionPlus, now + 600ms), std::runtime_error);
  control.feedback("a", std::numeric_limits<double>::quiet_NaN(), now + 600ms);
  EXPECT_THROW(control.event(JointEvent::PositionPlus, now + 600ms), std::runtime_error);
  EXPECT_THROW(control.axis(2), std::invalid_argument);
}

TEST(JointControl, LimitsAndDelayedTicks)
{
  JointControl::Config config;
  config.joints = {"a"};
  config.lower_positions = {-1};
  config.upper_positions = {1};
  int commands = 0;
  double target = 0;
  JointControl control(config, [&](auto, double value) { ++commands; target = value; }, [](auto, bool) {});
  const auto now = JointControl::Clock::now();
  control.feedback("a", 0.999, now);
  control.event(JointEvent::PositionPlus, now);
  EXPECT_DOUBLE_EQ(target, 1);
  control.axis(-1, now);
  control.tick(1, now + 10ms);
  EXPECT_EQ(commands, 1);
  control.tick(0.02, now + 20ms);
  EXPECT_EQ(commands, 1);
  config.joints = {"a", "a"};
  EXPECT_THROW(JointControl(config, [](auto, double) {}, [](auto, bool) {}), std::invalid_argument);
}
