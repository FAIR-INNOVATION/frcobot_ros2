# frcobot_ros2
This is the ROS2 API project of Fairino robot(sofeware version must greater than V3.7.1), a serial of functions which based on Fair SDK API but were simplified are created, user can call them through service message.
API_description.md list all API functions.
Tutorial of installing and uasage of ROS2 API, please refer to the Fair document platform:https://fair-documentation.readthedocs.io/en/latest/ROSGuide/index.html#frcobot-ros2.

Version histroy:
2023.7.18 V1.0

For timing-sensitive ServoJ trajectories on the 3.9.4 driver, use the buffered
`fairino_msgs/action/StreamServoJ` interface described in
[`fairino_hardware_v3_9_4/STREAM_SERVO_J.md`](fairino_hardware_v3_9_4/STREAM_SERVO_J.md).
The string-based remote command service remains available for compatibility.
