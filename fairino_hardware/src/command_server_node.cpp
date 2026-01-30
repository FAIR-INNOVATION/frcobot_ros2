#include "fairino_hardware/command_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "libfairino/include/robot.h"

int main(int argc, char *argv[]){
//该main函数用于创建简化指令客户端的app
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  //创建用户指令节点和反馈节点
  auto node = std::make_shared<robot_command_thread>("fr_command_server");
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}