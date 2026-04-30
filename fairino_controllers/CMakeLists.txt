cmake_minimum_required(VERSION 3.8)
project(fairino_controllers)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_library(fairino_controllers SHARED
  src/freedrive_mode_controller.cpp
)

target_include_directories(fairino_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fairino_controllers
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  std_msgs
)

pluginlib_export_plugin_description_file(controller_interface fairino_controllers_plugin.xml)

install(TARGETS fairino_controllers
  EXPORT fairino_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(fairino_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp controller_interface hardware_interface pluginlib
  rclcpp_lifecycle std_msgs
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
