cmake_minimum_required(VERSION 3.8)
project(fairino_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 生成可执行文件：名字就是 ros2_cmd_server
add_executable(ros2_cmd_server
  src/fairino_version_node.cpp
  src/fairino_version.cpp
  
)

ament_target_dependencies(ros2_cmd_server
  rclcpp
  ament_index_cpp
)

target_include_directories(ros2_cmd_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ros2_cmd_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
