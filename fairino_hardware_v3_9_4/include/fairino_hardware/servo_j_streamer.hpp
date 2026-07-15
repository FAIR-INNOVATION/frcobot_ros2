#ifndef FAIRINO_HARDWARE__SERVO_J_STREAMER_HPP_
#define FAIRINO_HARDWARE__SERVO_J_STREAMER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace fairino_hardware
{

struct ServoJStreamRequest
{
  std::vector<double> joints_deg;
  double period_sec{0.004};
  uint8_t communication_type{1};
};

struct ServoJStreamMetrics
{
  bool success{false};
  std::string message;
  int controller_error{0};
  uint64_t samples_requested{0};
  uint64_t samples_sent{0};
  uint64_t last_command_id{0};
  double maximum_interval_sec{0.0};
  uint64_t missed_deadlines{0};
  bool cancelled{false};
};

struct ServoJStreamerConfig
{
  double minimum_period_sec{0.001};
  double maximum_period_sec{0.016};
  double maximum_joint_step_deg{1.0};
  double deadline_tolerance_sec{0.00025};
  // A non-positive value uses one command period as the fatal lateness threshold.
  double maximum_lateness_sec{0.0};
  double feedback_period_sec{0.05};
};

class ServoJRobot
{
public:
  virtual ~ServoJRobot() = default;
  virtual int servo_move_start(int communication_type) = 0;
  virtual int servo_j(
    const std::array<double, 6> & joints_deg, float period_sec, int command_id,
    int communication_type) = 0;
  virtual int stop_motion() = 0;
  virtual int servo_move_end(int communication_type) = 0;
};

class MonotonicClock
{
public:
  virtual ~MonotonicClock() = default;
  virtual int64_t now_nanoseconds() = 0;
  virtual int sleep_until(int64_t absolute_nanoseconds) = 0;
};

class SystemMonotonicClock : public MonotonicClock
{
public:
  int64_t now_nanoseconds() override;
  int sleep_until(int64_t absolute_nanoseconds) override;
};

class ServoJStreamer
{
public:
  using FeedbackCallback = std::function<void (uint64_t, uint64_t)>;

  ServoJStreamer(
    ServoJRobot & robot, MonotonicClock & clock,
    ServoJStreamerConfig config = ServoJStreamerConfig{});

  bool validate(const ServoJStreamRequest & request, std::string & reason) const;
  bool reserve(const ServoJStreamRequest & request, std::string & reason);
  ServoJStreamMetrics run_reserved(
    const ServoJStreamRequest & request, const FeedbackCallback & feedback = {});

  // Sets cancellation before taking the SDK lock, so no ServoJ can be issued after StopMotion.
  int cancel_and_stop();
  void request_cancel();
  bool active() const;
  int allocate_command_id();

private:
  int stop_once();
  void finish_with_stop_and_end(ServoJStreamMetrics & metrics, int communication_type);
  static int64_t seconds_to_nanoseconds(double seconds);

  ServoJRobot & robot_;
  MonotonicClock & clock_;
  ServoJStreamerConfig config_;
  std::atomic_bool active_{false};
  std::atomic_bool cancel_requested_{false};
  std::mutex stop_mutex_;
  bool stop_called_{false};
  int stop_error_{0};
  std::atomic<uint64_t> next_command_id_{1};
};

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__SERVO_J_STREAMER_HPP_
