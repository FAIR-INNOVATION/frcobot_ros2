#include "rclcpp/rclcpp.hpp"
#include "fairino_version_pkg/fairino_version.hpp"

int main(int argc, char *argv[]){
//该main函数用于创建简化指令客户端的app
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  //创建用户指令节点和反馈节点
  auto node = std::make_shared<robot_version_thread>("robot_version_thread");
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}