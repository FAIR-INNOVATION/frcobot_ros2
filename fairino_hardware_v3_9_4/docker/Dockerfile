ARG BASE_IMAGE=grip-ros-zenoh:jazzy
FROM ${BASE_IMAGE}

ARG PACKAGE_NAME=fairino_hardware_v3_9_4

# Grip compose supplies the command; entrypoint only sources the ROS workspace.
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Package source and same-repo build dependency.
COPY src/frcobot_ros2/fairino_hardware_v3_9_4 /workspace/src/frcobot_ros2/fairino_hardware_v3_9_4
COPY src/frcobot_ros2/fairino_msgs /workspace/src/frcobot_ros2/fairino_msgs

WORKDIR /workspace

RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    PATHS="$(colcon list --packages-up-to ${PACKAGE_NAME} --paths-only | tr '\n' ' ')" && \
    rosdep install --from-paths ${PATHS} --ignore-src -r -y && \
    colcon build --packages-up-to ${PACKAGE_NAME} --symlink-install

ENTRYPOINT ["/ros_entrypoint.sh"]
