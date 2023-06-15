cmake_minimum_required(VERSION 3.8)
project(fr_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(frhal_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(HEADFILE_DIR ./include/fr_ros2/state_feedback.h ./include/fr_ros2/global_val.h 
                 ./include/fr_ros2/data_type_def.h ./include/fr_ros2/ROS_API.h)
set(SRC_DIR ./src/main.cpp ./src/state_feedback.cpp ./src/global_val.cpp ./src/ROS_API.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/fr_ros2)

add_executable(ros2_cmd_server  ${HEADFILE_DIR} ${SRC_DIR})

ament_target_dependencies(ros2_cmd_server rclcpp std_msgs frhal_msgs)

install(TARGETS
  ros2_cmd_server
  DESTINATION lib/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
