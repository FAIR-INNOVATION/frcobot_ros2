# FreedriveModeController Implementation Plan

> DELETE THIS FILE when raising PRs.

---

## Overview

Add `FreedriveModeController` as a proper ros2_control controller plugin that drives freedrive
via three command interfaces on the hardware interface. Remove the existing service-based
approach from HWI. Orchestration (controller switching) is handled by a dedicated action server
in metalman2, keeping each layer responsible for its own concern.

Repos:
- frcobot_ros2 — branch `feature/freedrive-and-cleanup`
- metalman2 — branch `kirtan/freedrive` (off `dev`)

---

## Architecture

```
User / Application
       │
       ▼
/freedrive action (metalman_interfaces/action/Freedrive)
       │
       ▼
FreedriveModeActionServer  (metalman_core/freedrive.py)
  - switch_controller: deactivate arm_controller, activate FDC  (on enable)
  - publishes Bool to /freedrive_mode_controller/enable_freedrive
  - sleep_async 0.5s, then switch_controller back                (on disable)
       │
       ▼
FreedriveModeController  (fairino_controllers — ros2_control plugin)
  - topic → command interface bridge (like UR's FDC)
  - claims: freedrive/enable, freedrive/abort, freedrive/async_success
       │
       ▼
FairinoHardwareInterface  write()
  - enable:  Mode(1) + DragTeachSwitch(1) → _async_success
  - abort:   DragTeachSwitch(0) + Mode(0) + sync cmd←actual + ServoMoveStart()
             → _waiting_for_jtc_hold until |cmd-state| < 0.05 rad
  - freedrive active: skip ServoJ entirely
```

---

## Constants

```
ASYNC_WAITING  = 2.0   // sentinel: operation in-flight, poll again
```

---

## Step 1 — frcobot_ros2: HWI Header ✓

File: `fairino_hardware/include/fairino_hardware/fairino_hardware_interface.hpp`

Removed: all atomic flag / service / executor / thread members, `_post_freedrive_cycles`.

Added:
```cpp
double _enable_cmd{0.0};
double _abort_cmd{0.0};
double _async_success{0.0};
bool _in_freedrive{false};
bool _waiting_for_jtc_hold{false};
static constexpr double ASYNC_WAITING{2.0};
static constexpr double HOLD_TOLERANCE{0.05};
```

---

## Step 2 — frcobot_ros2: HWI Source ✓

File: `fairino_hardware/src/fairino_hardware_interface.cpp`

### export_command_interfaces()
Added after joint interfaces:
```cpp
command_interfaces.emplace_back("freedrive", "enable",        &_enable_cmd);
command_interfaces.emplace_back("freedrive", "abort",         &_abort_cmd);
command_interfaces.emplace_back("freedrive", "async_success", &_async_success);
```

### on_activate()
Removed all service/executor/thread construction. Initialises new members to 0/false.

### on_deactivate()
Removed executor cancel/join, service resets. Kept freedrive-exit guard, StopMotion, CloseRPC.

### read()
Removed all atomic flag exchange blocks. Kept joint state reads only.

### write() — priority-ordered logic:
```
1. _enable_cmd == 1.0  → Mode(1) + DragTeachSwitch(1) → _async_success=1.0/0.0
                          _in_freedrive=true, return OK

2. _abort_cmd == 1.0   → DragTeachSwitch(0) + Mode(0) + sync cmd→actual + ServoMoveStart()
                          _async_success=1.0/0.0, _in_freedrive=false, return OK

3. _in_freedrive        → return OK  (pure freedrive, no joint writes)

4. Normal ServoJ path
```

---

## Step 3 — frcobot_ros2: HWI CMakeLists.txt ✓

Removed `std_srvs` from find_package, ament_target_dependencies, ament_export_dependencies.

---

## Step 4 — frcobot_ros2: HWI package.xml ✓

Removed `<depend>std_srvs</depend>`.

---

## Step 5 — frcobot_ros2: URDF xacro

No change needed. The three `freedrive/*` command interfaces are exported programmatically
from `export_command_interfaces()` — not declared in URDF.

---

## Step 6 — frcobot_ros2: Create `fairino_controllers` package ✓

### Files created:
- `fairino_controllers/package.xml`
- `fairino_controllers/CMakeLists.txt`
- `fairino_controllers/fairino_controllers_plugin.xml`
- `fairino_controllers/include/fairino_controllers/freedrive_mode_controller.hpp`
- `fairino_controllers/src/freedrive_mode_controller.cpp`
- `fairino_controllers/launch/freedrive.launch.py`

### Key design (FDC — mirrors UR's FreedriveModeController):
- Claims only: `freedrive/enable`, `freedrive/abort`, `freedrive/async_success`
- No joint interfaces claimed (JTC retains those)
- Subscribes to `~/enable_freedrive` (std_msgs/Bool)
- `update()` state machine: detect desired≠current → write enable/abort → poll async_success
- No switch_controller logic — orchestration is the action server's responsibility

### freedrive.launch.py:
- Spawns `freedrive_mode_controller` as inactive
- Imported via `IncludeLaunchDescription` in moveit.launch.py

---

## Step 7 — frcobot_ros2: fairino20 ros2_controllers.yaml ✓

File: `fairino20_v6_moveit2_config/config/ros2_controllers.yaml`

Added `freedrive_mode_controller` entry.

---

## Step 8 — metalman2: ros2_controllers.yaml ✓

File: `metalman_moveit_config/config/ros2_controllers.yaml`

Added `freedrive_mode_controller` entry.

---

## Step 9 — metalman2: Freedrive action definition ✓

File: `metalman_interfaces/action/Freedrive.action`

```
bool enable        # Goal
---
bool success       # Result
string message
---
string status      # Feedback
```

---

## Step 10 — metalman2: FreedriveModeActionServer ✓

File: `metalman_core/metalman_core/freedrive.py`

Entry point: `freedrive_action_server` (registered in setup.py).

Parameters:
- `arm_controller` (default: `arm_controller`)
- `freedrive_controller` (default: `freedrive_mode_controller`)
- `controller_manager_ns` (default: `""`)

Enable flow:
1. switch_controller: activate FDC, deactivate arm_controller
2. Publish Bool(true) → FDC → HWI drag mode on

Disable flow:
1. Publish Bool(false) → FDC → HWI drag mode off
2. sleep_async 0.5s (non-blocking, lets HWI process DragTeachSwitch + ServoMoveStart)
3. switch_controller: activate arm_controller, deactivate FDC → JTC on_activate() re-seeds

Uses `metalman_core.async_ros.util.sleep_async` and `metalman_core.service_client.call_service`.

---

## Step 11 — metalman2: moveit.launch.py ✓

- `IncludeLaunchDescription` → `fairino_controllers/launch/freedrive.launch.py` (spawns FDC)
- `Node` → `metalman_core/freedrive_action_server`

---

## TODO

### TODO 1 — Resend Robot Program ✓

`resend_robot_program` as a `std_srvs/Trigger` service exposed directly from the HWI via `get_node()`.

Sequence (second `Mode(0)` from ROS1 dropped — redundant):
```
ProgramStop() → RobotEnable(1) → Mode(0) → ResetAllError() → ServoMoveStart() → sync cmd←actual
```

Implementation:
- 3 atomics in HWI (`_resend_pending`, `_resend_done`, `_resend_success`)
- Service callback: set `_resend_pending`, spin-wait ≤8ms per cycle, return result
- `write()`: `_resend_pending.exchange(false)` → run sequence → set `_resend_done`
- No controller, no command interfaces, no extra threads

---

### TODO 2 — Unlock Protective Stop ✓

`protective_stop_recovery` as an **Action** in `fr_arm_server.py` (metalman_core).

Flow:
1. Call HWI's `resend_robot_program` service (clears errors, re-enables servo)
2. Enable freedrive via internal `_freedrive_enable()` call (no action round-trip)

No new frcobot_ros2 code needed beyond TODO 1.

---

### TODO 1+2 — fr_arm_server (metalman_core) ✓

Replaces `freedrive.py`. Single node per arm, exposes all arm recovery operations:

```
/freedrive                    ← Action  (existing logic, moved here)
/protective_stop_recovery     ← Action  (resend + freedrive enable)
```

`resend_robot_program` is NOT re-exposed from Python — it's available directly from the HWI node.

Multi-arm: launch one `fr_arm_server` per arm with different parameters/namespace.

Parameters:
- `arm_controller` (default: `arm_controller`)
- `freedrive_controller` (default: `freedrive_mode_controller`)
- `controller_manager_ns` (default: `""`)
- `hw_ns` (default: `""`) — namespace of HWI node for resend service

---

### TODO 3 — FR Status node on port 20004 ✓

The HWI (`FRRobot::RPC()`) and any other node that instantiates `FRRobot` and calls `RPC()`
compete on port 2003, causing "port occupied" crashes on restart.

FR controller ports:
- Port 2003 — command/motion RPC (HWI owns this exclusively)
- Port 20004 — push-based state feed; controller sends `ROBOT_STATE_PKG` at **8 ms** intervals
  to every connected TCP client (no SDK instantiation required)

Implementation: standalone `fr_status_node` — raw TCP client on port 20004, no SDK, no RPC.
Receives `ROBOT_STATE_PKG` (pack(1) struct, frame_head = 0x5A5A), publishes `RobotNonrtState`.
Auto-reconnects on socket disconnect.

### `/fr_arm` namespace ✓

All user-facing FR arm interfaces are grouped under `/fr_arm`:

| Interface | Type | Where |
|-----------|------|--------|
| `/fr_arm/nonrt_state_data` | topic | `fr_status_node` (namespace="/fr_arm") |
| `/fr_arm/freedrive` | action | `fr_arm_server` (namespace="/fr_arm") |
| `/fr_arm/protective_stop_recovery` | action | `fr_arm_server` (namespace="/fr_arm") |
| `/fr_arm/resend_robot_program` | service | HWI via absolute name in `create_service()` |

`resend_robot_program` uses an absolute service name (`"/fr_arm/resend_robot_program"`) in the
HWI because the HWI node's namespace is inherited from `ros2_control_node` (root), which also
hosts the base drive controllers and cannot be re-namespaced per-arm.

> TODO: if additional arm types are added, extract arm-specific nodes into per-type launch files
> (e.g. `fr_arm.launch.py`) and dispatch from `moveit.launch.py` based on `arm_type` parameter.

Files created/modified:
- `fairino_hardware/src/fr_status_node.cpp` ✓
- `fairino_hardware/CMakeLists.txt` — added `fr_status_node` executable ✓
- `fairino_hardware/src/fairino_hardware_interface.cpp` — absolute service name `/fr_arm/resend_robot_program` ✓
- `metalman_moveit_config/launch/moveit.launch.py` — `fr_status_node` + `fr_arm_server` with `namespace="/fr_arm"`, `hw_ns="/fr_arm"` ✓

---

## Usage

```bash
# Enable freedrive
ros2 action send_goal /fr_arm/freedrive metalman_interfaces/action/Freedrive "{enable: true}"

# Disable freedrive (JTC re-seeds automatically)
ros2 action send_goal /fr_arm/freedrive metalman_interfaces/action/Freedrive "{enable: false}"

# Resend robot program (reset after error)
ros2 service call /fr_arm/resend_robot_program std_srvs/srv/Trigger

# Unlock protective stop (reset + enter freedrive so operator can move arm)
ros2 action send_goal /fr_arm/protective_stop_recovery metalman_interfaces/action/ProtectiveStopRecovery
```

---

## Build verification

```bash
# frcobot_ros2
colcon build --packages-select fairino_hardware fairino_controllers fairino_msgs

# metalman2
colcon build --packages-select metalman_interfaces metalman_core metalman_moveit_config
source install/setup.bash
```

---

## File Index

| # | File | Repo | Status |
|---|------|------|--------|
| 1 | `fairino_hardware/include/fairino_hardware/fairino_hardware_interface.hpp` | frcobot_ros2 | ✓ |
| 2 | `fairino_hardware/src/fairino_hardware_interface.cpp` | frcobot_ros2 | ✓ |
| 3 | `fairino_hardware/CMakeLists.txt` | frcobot_ros2 | ✓ |
| 4 | `fairino_hardware/package.xml` | frcobot_ros2 | ✓ |
| 5 | `fairino_controllers/CMakeLists.txt` | frcobot_ros2 | ✓ |
| 6 | `fairino_controllers/package.xml` | frcobot_ros2 | ✓ |
| 7 | `fairino_controllers/fairino_controllers_plugin.xml` | frcobot_ros2 | ✓ |
| 8 | `fairino_controllers/include/fairino_controllers/freedrive_mode_controller.hpp` | frcobot_ros2 | ✓ |
| 9 | `fairino_controllers/src/freedrive_mode_controller.cpp` | frcobot_ros2 | ✓ |
| 10 | `fairino_controllers/launch/freedrive.launch.py` | frcobot_ros2 | ✓ |
| 11 | `fairino20_v6_moveit2_config/config/ros2_controllers.yaml` | frcobot_ros2 | ✓ |
| 12 | `metalman_interfaces/action/Freedrive.action` | metalman2 | ✓ |
| 13 | `metalman_interfaces/action/ProtectiveStopRecovery.action` | metalman2 | ✓ |
| 14 | `metalman_interfaces/CMakeLists.txt` | metalman2 | ✓ |
| 15 | `metalman_core/metalman_core/fr_arm_server.py` | metalman2 | ✓ |
| 16 | `metalman_core/metalman_core/freedrive.py` | metalman2 | deleted |
| 17 | `metalman_core/setup.py` | metalman2 | ✓ |
| 18 | `metalman_core/package.xml` | metalman2 | ✓ |
| 19 | `metalman_moveit_config/config/ros2_controllers.yaml` | metalman2 | ✓ |
| 20 | `metalman_moveit_config/launch/moveit.launch.py` | metalman2 | ✓ |
