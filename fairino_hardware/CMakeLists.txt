cmake_minimum_required(VERSION 3.8)
project(fairino_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(fairino_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(SRC_DIR EXP_SRC_DIR)
file(GLOB SRC_DIR ./src/command_server.cpp ./src/command_server_node.cpp)
file(GLOB EXP_SRC_DIR ./examples/src/*.cpp)

#compile
add_executable(ros2_cmd_server ${SRC_DIR})
add_executable(srv_test_client ${EXP_SRC_DIR})
add_library(
  fairino_hardware 
  SHARED 
  ./src/fairino_robot.cpp 
  ./src/fairino_hardware_interface.cpp
)

#add headfile source
target_include_directories(ros2_cmd_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(srv_test_client PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/examples/include)
target_include_directories(fairino_hardware PUBLIC 
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/fairino_hardware>
)
target_link_libraries(fairino_hardware ${CMAKE_CURRENT_SOURCE_DIR}/include/xmlrpc/libXmlRpc.a)

#add dependencies
ament_target_dependencies(ros2_cmd_server rclcpp fairino_msgs)
ament_target_dependencies(srv_test_client rclcpp fairino_msgs)
ament_target_dependencies(fairino_hardware 
  hardware_interface
  pluginlib
  rclcpp
  fairino_msgs
  rclcpp_lifecycle 
)


#export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface fairino_hardware.xml)

#install all files
#command server
install(TARGETS ros2_cmd_server
  DESTINATION lib/${PROJECT_NAME}
)

#test example
install(TARGETS srv_test_client
  DESTINATION lib/${PROJECT_NAME}
)

#ros2_control hardware
install(TARGETS fairino_hardware
  EXPORT fairino_hardware
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#export
ament_export_targets(fairino_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp hardware_interface pluginlib rclcpp_lifecycle)

ament_package()
