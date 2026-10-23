# frcobot_ros2

## PickNik fork

This fork carries PickNik changes to `fairino_hardware` for use with MoveIt Pro:

- **Controller software V3.9.8 or newer is required.** The bundled fairino SDK is
  libfairino 2.3.8, which matches V3.9.8. Older controller software is not supported.
- libfairino 2.3.8 is bundled for both `x86_64` and `aarch64`; CMake selects the build
  for the target architecture.
- ServoJ streaming is bracketed by `ServoMoveStart()`/`ServoMoveEnd()` and uses
  `cmdT = 0.0016` s, so the `controller_manager` `update_rate` must be 625 Hz.
- The driver halts the ServoJ stream after sustained rejections (error-14 storm guard).
- Joints may export an optional velocity state interface.

## Upstream
This is the ROS2 API project of Fairino robot(sofeware version must greater than V3.7.1), a serial of functions which based on Fair SDK API but were simplified are created, user can call them through service message.
API_description.md list all API functions.
Tutorial of installing and uasage of ROS2 API, please refer to the Fair document platform:https://fair-documentation.readthedocs.io/en/latest/ROSGuide/index.html#frcobot-ros2.

Version histroy:
2023.7.18 V1.0
