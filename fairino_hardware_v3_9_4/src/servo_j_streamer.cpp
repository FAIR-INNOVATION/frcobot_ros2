#include "fairino_hardware/servo_j_streamer.hpp"

#include <cerrno>
#include <algorithm>
#include <cmath>
#include <climits>
#include <ctime>
#include <sstream>

namespace fairino_hardware
{

int64_t SystemMonotonicClock::now_nanoseconds()
{
  timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
    return -1;
  }
  return static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
}

int SystemMonotonicClock::sleep_until(int64_t absolute_nanoseconds)
{
  timespec deadline{};
  deadline.tv_sec = absolute_nanoseconds / 1000000000LL;
  deadline.tv_nsec = absolute_nanoseconds % 1000000000LL;
  int result;
  do {
    result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
  } while (result == EINTR);
  return result;
}

ServoJStreamer::ServoJStreamer(
  ServoJRobot & robot, MonotonicClock & clock, ServoJStreamerConfig config)
: robot_(robot), clock_(clock), config_(config)
{
}

bool ServoJStreamer::validate(const ServoJStreamRequest & request, std::string & reason) const
{
  if (!std::isfinite(config_.minimum_period_sec) ||
    !std::isfinite(config_.maximum_period_sec) || config_.minimum_period_sec <= 0.0 ||
    config_.maximum_period_sec < config_.minimum_period_sec ||
    !std::isfinite(config_.maximum_joint_step_deg) || config_.maximum_joint_step_deg <= 0.0 ||
    !std::isfinite(config_.deadline_tolerance_sec) || config_.deadline_tolerance_sec < 0.0 ||
    !std::isfinite(config_.maximum_lateness_sec) ||
    !std::isfinite(config_.feedback_period_sec) || config_.feedback_period_sec < 0.0)
  {
    reason = "invalid ServoJ driver configuration";
    return false;
  }
  if (request.joints_deg.empty() || request.joints_deg.size() % 6 != 0) {
    reason = "joints_deg must contain one or more complete six-joint samples";
    return false;
  }
  if (!std::isfinite(request.period_sec) ||
    request.period_sec < config_.minimum_period_sec ||
    request.period_sec > config_.maximum_period_sec)
  {
    std::ostringstream message;
    message << "period_sec must be finite and in [" << config_.minimum_period_sec << ", " <<
      config_.maximum_period_sec << "]";
    reason = message.str();
    return false;
  }
  if (request.communication_type != 1) {
    reason = "communication_type must be 1 (FAIRINO UDP port 20007)";
    return false;
  }
  for (size_t index = 0; index < request.joints_deg.size(); ++index) {
    if (!std::isfinite(request.joints_deg[index])) {
      reason = "every joint target must be finite";
      return false;
    }
    if (index >= 6 &&
      std::abs(request.joints_deg[index] - request.joints_deg[index - 6]) >
      config_.maximum_joint_step_deg)
    {
      std::ostringstream message;
      message << "adjacent samples exceed the " << config_.maximum_joint_step_deg <<
        " degree ServoJ command limit at sample " << (index / 6) << ", joint " <<
        (index % 6);
      reason = message.str();
      return false;
    }
  }
  return true;
}

bool ServoJStreamer::reserve(const ServoJStreamRequest & request, std::string & reason)
{
  if (!validate(request, reason)) {
    return false;
  }
  bool expected = false;
  if (!active_.compare_exchange_strong(expected, true)) {
    reason = "another ServoJ stream is already active";
    return false;
  }
  cancel_requested_.store(false);
  {
    std::lock_guard<std::mutex> stop_lock(stop_mutex_);
    stop_called_ = false;
    stop_error_ = 0;
  }
  return true;
}

int ServoJStreamer::stop_once()
{
  // Keep later cleanup callers behind the first StopMotion call so ServoMoveEnd
  // can never overtake a cancellation thread that is waiting for the SDK lock.
  std::lock_guard<std::mutex> stop_lock(stop_mutex_);
  if (stop_called_) {
    return stop_error_;
  }
  stop_called_ = true;
  stop_error_ = robot_.stop_motion();
  return stop_error_;
}

void ServoJStreamer::finish_with_stop_and_end(
  ServoJStreamMetrics & metrics, int communication_type)
{
  const int stop_error = stop_once();
  const int end_error = robot_.servo_move_end(communication_type);
  if (metrics.controller_error == 0) {
    metrics.controller_error = stop_error != 0 ? stop_error : end_error;
  }
}

ServoJStreamMetrics ServoJStreamer::run_reserved(
  const ServoJStreamRequest & request, const FeedbackCallback & feedback)
{
  ServoJStreamMetrics metrics;
  metrics.samples_requested = request.joints_deg.size() / 6;
  const int communication_type = request.communication_type;
  const int64_t period_ns = seconds_to_nanoseconds(request.period_sec);
  const int64_t tolerance_ns = seconds_to_nanoseconds(config_.deadline_tolerance_sec);
  const double fatal_lateness = config_.maximum_lateness_sec > 0.0 ?
    config_.maximum_lateness_sec : request.period_sec;
  const int64_t fatal_lateness_ns = seconds_to_nanoseconds(fatal_lateness);
  const int64_t feedback_period_ns = seconds_to_nanoseconds(config_.feedback_period_sec);

  auto release = [this]() {active_.store(false);};
  struct ScopeExit
  {
    std::function<void()> function;
    ~ScopeExit() {function();}
  } scope_exit{release};

  if (cancel_requested_.load()) {
    metrics.cancelled = true;
    metrics.message = "ServoJ stream cancelled";
    return metrics;
  }

  int error = robot_.servo_move_start(communication_type);
  if (error != 0) {
    metrics.controller_error = error;
    metrics.cancelled = cancel_requested_.load();
    metrics.message = "ServoMoveStart failed";
    finish_with_stop_and_end(metrics, communication_type);
    return metrics;
  }

  const int64_t start_ns = clock_.now_nanoseconds();
  if (start_ns < 0) {
    metrics.message = "clock_gettime(CLOCK_MONOTONIC) failed";
    finish_with_stop_and_end(metrics, communication_type);
    return metrics;
  }

  int64_t previous_send_ns = -1;
  int64_t last_feedback_ns = start_ns - feedback_period_ns;
  for (uint64_t sample = 0; sample < metrics.samples_requested; ++sample) {
    if (cancel_requested_.load()) {
      metrics.cancelled = true;
      metrics.message = "ServoJ stream cancelled";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }

    const int64_t deadline_ns = start_ns + static_cast<int64_t>(sample) * period_ns;
    const int sleep_error = clock_.sleep_until(deadline_ns);
    if (sleep_error != 0) {
      metrics.message = "clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME) failed";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    const int64_t wake_ns = clock_.now_nanoseconds();
    if (wake_ns < 0) {
      metrics.message = "clock_gettime(CLOCK_MONOTONIC) failed";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    const int64_t wake_lateness_ns = wake_ns - deadline_ns;
    const bool deadline_missed_before_call = wake_lateness_ns > tolerance_ns;
    if (deadline_missed_before_call) {
      ++metrics.missed_deadlines;
    }
    if (wake_lateness_ns > fatal_lateness_ns) {
      metrics.message = "ServoJ timing failure: command deadline exceeded maximum lateness";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    if (cancel_requested_.load()) {
      metrics.cancelled = true;
      metrics.message = "ServoJ stream cancelled";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }

    const int command_id = allocate_command_id();
    if (command_id < 0) {
      metrics.message = "ServoJ command ID exhausted the SDK integer range";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    std::array<double, 6> joints{};
    for (size_t joint = 0; joint < joints.size(); ++joint) {
      joints[joint] = request.joints_deg[sample * 6 + joint];
    }
    error = robot_.servo_j(
      joints, static_cast<float>(request.period_sec), command_id, communication_type);
    if (error != 0) {
      metrics.controller_error = error;
      metrics.cancelled = cancel_requested_.load();
      metrics.message = "ServoJ failed";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    ++metrics.samples_sent;
    metrics.last_command_id = static_cast<uint64_t>(command_id);

    // Measure after the SDK call so mutex contention and SDK/UDP call time are reflected.
    const int64_t sent_ns = clock_.now_nanoseconds();
    if (sent_ns < 0) {
      metrics.message = "clock_gettime(CLOCK_MONOTONIC) failed";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    const int64_t sent_lateness_ns = sent_ns - deadline_ns;
    if (!deadline_missed_before_call && sent_lateness_ns > tolerance_ns) {
      ++metrics.missed_deadlines;
    }
    if (previous_send_ns >= 0) {
      metrics.maximum_interval_sec = std::max(
        metrics.maximum_interval_sec,
        static_cast<double>(sent_ns - previous_send_ns) / 1.0e9);
    }
    previous_send_ns = sent_ns;
    if (sent_lateness_ns > fatal_lateness_ns) {
      metrics.message = "ServoJ timing failure: SDK command completed after maximum lateness";
      finish_with_stop_and_end(metrics, communication_type);
      return metrics;
    }
    if (feedback &&
      (sent_ns - last_feedback_ns >= feedback_period_ns ||
      metrics.samples_sent == metrics.samples_requested))
    {
      feedback(metrics.samples_sent, metrics.last_command_id);
      last_feedback_ns = sent_ns;
    }
  }

  if (cancel_requested_.load()) {
    metrics.cancelled = true;
    metrics.message = "ServoJ stream cancelled";
    finish_with_stop_and_end(metrics, communication_type);
    return metrics;
  }

  error = robot_.servo_move_end(communication_type);
  if (error != 0) {
    metrics.controller_error = error;
    metrics.cancelled = cancel_requested_.load();
    metrics.message = "ServoMoveEnd failed";
    finish_with_stop_and_end(metrics, communication_type);
    return metrics;
  }
  metrics.success = true;
  metrics.message = "ServoJ stream completed";
  return metrics;
}

int ServoJStreamer::cancel_and_stop()
{
  cancel_requested_.store(true);
  return active_.load() ? stop_once() : 0;
}

void ServoJStreamer::request_cancel()
{
  cancel_requested_.store(true);
}

bool ServoJStreamer::active() const
{
  return active_.load();
}

int ServoJStreamer::allocate_command_id()
{
  const uint64_t command_id = next_command_id_.fetch_add(1);
  return command_id <= static_cast<uint64_t>(INT_MAX) ? static_cast<int>(command_id) : -1;
}

int64_t ServoJStreamer::seconds_to_nanoseconds(double seconds)
{
  return static_cast<int64_t>(std::llround(seconds * 1.0e9));
}

}  // namespace fairino_hardware
