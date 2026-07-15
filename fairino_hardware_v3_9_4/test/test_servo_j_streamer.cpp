#include "fairino_hardware/servo_j_streamer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

class FakeClock : public fairino_hardware::MonotonicClock
{
public:
  int64_t now_nanoseconds() override {return now;}

  int sleep_until(int64_t absolute_nanoseconds) override
  {
    deadlines.push_back(absolute_nanoseconds);
    if (sleep_error != 0) {
      return sleep_error;
    }
    now = absolute_nanoseconds + lateness;
    return 0;
  }

  int64_t now{1000000000LL};
  int64_t lateness{0};
  int sleep_error{0};
  std::vector<int64_t> deadlines;
};

class MockRobot : public fairino_hardware::ServoJRobot
{
public:
  int servo_move_start(int communication_type) override
  {
    calls.push_back("start:" + std::to_string(communication_type));
    return start_error;
  }

  int servo_j(
    const std::array<double, 6> & joints_deg, float period_sec, int command_id,
    int communication_type) override
  {
    calls.push_back("servo:" + std::to_string(command_id));
    targets.push_back(joints_deg);
    periods.push_back(period_sec);
    communication_types.push_back(communication_type);
    if (clock != nullptr) {
      clock->now += servo_duration_nanoseconds;
    }
    return servo_error;
  }

  int stop_motion() override
  {
    calls.push_back("stop");
    return stop_error;
  }

  int servo_move_end(int communication_type) override
  {
    calls.push_back("end:" + std::to_string(communication_type));
    return end_error;
  }

  int start_error{0};
  int servo_error{0};
  int stop_error{0};
  int end_error{0};
  FakeClock * clock{nullptr};
  int64_t servo_duration_nanoseconds{0};
  std::vector<std::string> calls;
  std::vector<std::array<double, 6>> targets;
  std::vector<float> periods;
  std::vector<int> communication_types;
};

fairino_hardware::ServoJStreamRequest valid_request(size_t samples = 2)
{
  fairino_hardware::ServoJStreamRequest request;
  request.period_sec = 0.004;
  request.communication_type = 1;
  for (size_t sample = 0; sample < samples; ++sample) {
    for (size_t joint = 0; joint < 6; ++joint) {
      request.joints_deg.push_back(static_cast<double>(sample) * 0.5 + joint);
    }
  }
  return request;
}

TEST(ServoJStreamerValidation, RejectsMalformedAndUnsafeGoals)
{
  MockRobot robot;
  FakeClock clock;
  fairino_hardware::ServoJStreamer streamer(robot, clock);
  std::string reason;

  auto request = valid_request();
  request.joints_deg.pop_back();
  EXPECT_FALSE(streamer.validate(request, reason));

  request = valid_request();
  request.joints_deg[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(streamer.validate(request, reason));

  request = valid_request();
  request.joints_deg[6] = request.joints_deg[0] + 1.01;
  EXPECT_FALSE(streamer.validate(request, reason));

  request = valid_request();
  request.period_sec = 0.0;
  EXPECT_FALSE(streamer.validate(request, reason));

  request = valid_request();
  request.period_sec = 0.020;
  EXPECT_FALSE(streamer.validate(request, reason));

  request = valid_request();
  request.communication_type = 0;
  EXPECT_FALSE(streamer.validate(request, reason));
}

TEST(ServoJStreamerExecution, UsesAbsoluteDeadlinesUdpAndIncreasingIds)
{
  MockRobot robot;
  FakeClock clock;
  fairino_hardware::ServoJStreamer streamer(robot, clock);
  const auto request = valid_request(3);
  std::string reason;
  ASSERT_TRUE(streamer.reserve(request, reason));

  const auto result = streamer.run_reserved(request);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.samples_requested, 3U);
  EXPECT_EQ(result.samples_sent, 3U);
  EXPECT_EQ(result.last_command_id, 3U);
  ASSERT_EQ(clock.deadlines.size(), 3U);
  EXPECT_EQ(clock.deadlines[1] - clock.deadlines[0], 4000000LL);
  EXPECT_EQ(clock.deadlines[2] - clock.deadlines[1], 4000000LL);
  EXPECT_EQ(
    robot.calls,
    (std::vector<std::string>{"start:1", "servo:1", "servo:2", "servo:3", "end:1"}));
  EXPECT_EQ(robot.communication_types, (std::vector<int>{1, 1, 1}));
  EXPECT_NEAR(robot.periods.front(), 0.004, 1.0e-7);
  EXPECT_DOUBLE_EQ(result.maximum_interval_sec, 0.004);

  const auto second_request = valid_request(1);
  ASSERT_TRUE(streamer.reserve(second_request, reason));
  const auto second_result = streamer.run_reserved(second_request);
  EXPECT_EQ(second_result.last_command_id, 4U);
}

TEST(ServoJStreamerExecution, RejectsASecondActiveStream)
{
  MockRobot robot;
  FakeClock clock;
  fairino_hardware::ServoJStreamer streamer(robot, clock);
  const auto request = valid_request();
  std::string reason;
  ASSERT_TRUE(streamer.reserve(request, reason));
  EXPECT_FALSE(streamer.reserve(request, reason));
  EXPECT_NE(reason.find("already active"), std::string::npos);
  streamer.cancel_and_stop();
  const auto result = streamer.run_reserved(request);
  EXPECT_TRUE(result.cancelled);
}

TEST(ServoJStreamerExecution, CancellationStopsThenEndsAndReportsProgress)
{
  MockRobot robot;
  FakeClock clock;
  fairino_hardware::ServoJStreamer streamer(robot, clock);
  const auto request = valid_request(3);
  std::string reason;
  ASSERT_TRUE(streamer.reserve(request, reason));

  const auto result = streamer.run_reserved(
    request, [&streamer](uint64_t samples_sent, uint64_t) {
      if (samples_sent == 1) {
        streamer.cancel_and_stop();
      }
    });

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.cancelled);
  EXPECT_EQ(result.samples_requested, 3U);
  EXPECT_EQ(result.samples_sent, 1U);
  EXPECT_EQ(robot.calls, (std::vector<std::string>{"start:1", "servo:1", "stop", "end:1"}));
}

TEST(ServoJStreamerExecution, SdkFailureStopsThenEnds)
{
  MockRobot robot;
  robot.servo_error = 42;
  FakeClock clock;
  fairino_hardware::ServoJStreamer streamer(robot, clock);
  const auto request = valid_request();
  std::string reason;
  ASSERT_TRUE(streamer.reserve(request, reason));

  const auto result = streamer.run_reserved(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.controller_error, 42);
  EXPECT_EQ(result.samples_sent, 0U);
  EXPECT_EQ(robot.calls, (std::vector<std::string>{"start:1", "servo:1", "stop", "end:1"}));
}

TEST(ServoJStreamerExecution, TimingFailureStopsBeforeSendingLateCommand)
{
  MockRobot robot;
  FakeClock clock;
  clock.lateness = 5000000LL;
  fairino_hardware::ServoJStreamer streamer(robot, clock);
  const auto request = valid_request();
  std::string reason;
  ASSERT_TRUE(streamer.reserve(request, reason));

  const auto result = streamer.run_reserved(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.missed_deadlines, 1U);
  EXPECT_EQ(result.samples_sent, 0U);
  EXPECT_EQ(robot.calls, (std::vector<std::string>{"start:1", "stop", "end:1"}));
}

TEST(ServoJStreamerExecution, TimingMetricsIncludeSdkAndMutexDelay)
{
  MockRobot robot;
  FakeClock clock;
  robot.clock = &clock;
  robot.servo_duration_nanoseconds = 5000000LL;
  fairino_hardware::ServoJStreamer streamer(robot, clock);
  const auto request = valid_request();
  std::string reason;
  ASSERT_TRUE(streamer.reserve(request, reason));

  const auto result = streamer.run_reserved(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.samples_sent, 1U);
  EXPECT_EQ(result.missed_deadlines, 1U);
  EXPECT_EQ(robot.calls, (std::vector<std::string>{"start:1", "servo:1", "stop", "end:1"}));
}

TEST(ServoJStreamerExecution, StartFailureStillStopsThenEnds)
{
  MockRobot robot;
  robot.start_error = 7;
  FakeClock clock;
  fairino_hardware::ServoJStreamer streamer(robot, clock);
  const auto request = valid_request();
  std::string reason;
  ASSERT_TRUE(streamer.reserve(request, reason));

  const auto result = streamer.run_reserved(request);

  EXPECT_EQ(result.controller_error, 7);
  EXPECT_EQ(robot.calls, (std::vector<std::string>{"start:1", "stop", "end:1"}));
}

TEST(ServoJStreamerExecution, EndFailureStopsThenRetriesEnd)
{
  MockRobot robot;
  robot.end_error = 8;
  FakeClock clock;
  fairino_hardware::ServoJStreamer streamer(robot, clock);
  const auto request = valid_request(1);
  std::string reason;
  ASSERT_TRUE(streamer.reserve(request, reason));

  const auto result = streamer.run_reserved(request);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.controller_error, 8);
  EXPECT_EQ(
    robot.calls,
    (std::vector<std::string>{"start:1", "servo:1", "end:1", "stop", "end:1"}));
}

}  // namespace
