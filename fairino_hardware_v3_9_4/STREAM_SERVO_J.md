# Buffered ServoJ streaming

Use the `stream_servo_j` action for trajectories that have already been resampled to a
fixed joint-space period. The complete flattened trajectory crosses ROS once; the driver
then sends it from a local worker using `ServoMoveStart(1)`, repeated UDP
`ServoJ(..., period_sec, ..., command_id, 1)`, and `ServoMoveEnd(1)`.

The legacy `fairino_remote_command_service` forms `ServoMoveStart(...)`, `ServoJ(...)`,
and `ServoMoveEnd(...)` remain available for compatibility. They are not recommended for
sample-by-sample trajectory streaming.

Example goal with two six-joint samples at 4 ms:

```bash
ros2 action send_goal /stream_servo_j fairino_msgs/action/StreamServoJ \
  "{joints_deg: [0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0], period_sec: 0.004, communication_type: 1}" \
  --feedback
```

Goals are rejected unless they contain a non-empty multiple of six finite joint values,
use UDP communication type `1`, use a supported period, and change no joint by more than
the configured per-command limit between adjacent samples. Only one buffered stream can
be active. Other motion commands are rejected until it finishes; action cancellation or
the legacy `StopMotion()` command preempts the stream.

The result reports requested and sent sample counts, the controller error, final command
ID, maximum measured SDK-command interval, missed deadlines, and cancellation state.
Feedback is throttled independently of the command loop (50 ms by default).

Configuration parameters and defaults:

- `servo_j.minimum_period_sec`: `0.001`
- `servo_j.maximum_period_sec`: `0.016`
- `servo_j.maximum_joint_step_deg`: `1.0`
- `servo_j.deadline_tolerance_sec`: `0.00025`
- `servo_j.maximum_lateness_sec`: `0.0` (one requested period)
- `servo_j.feedback_period_sec`: `0.05`

The existing `nonrt_state_data` timer runs at 50 ms (20 Hz) outside a stream. Its SDK poll
is suppressed while a buffered stream is active because `GetRobotRealTimeState()` and
`ServoJ()` share the same SDK mutex; allowing the poll during a stream can block a command
past its deadline. State publication resumes on the first timer callback after the stream.
Service and action callbacks use separate callback groups, while access to the shared
FAIRINO SDK object is serialized. The blocking stream itself does not run on an executor
thread.

Clients that enforce a final joint tolerance should include a short run of repeated final
samples in the submitted buffer, then wait for a `nonrt_state_data` message published
after the action completes before checking the endpoint. Checking the cached state
immediately can report the controller's normal following lag (or the last pre-stream
state) as a goal-tolerance failure.
