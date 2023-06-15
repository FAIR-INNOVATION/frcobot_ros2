#include "rclcpp/rclcpp.hpp"
#include "state_feedback.h"
#include "global_val.h"
#include "ROS_API.h"

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor mulexecutor;
    auto state_recv_node = std::make_shared<state_recv_thread>("FR_recv_data_thread");//Socket thread node, used to receive status socket information
    mulexecutor.add_node(state_recv_node);
    auto ros_api_node = std::make_shared<ROS_API>("FR_ROS_API_node");
    mulexecutor.add_node(ros_api_node);//Create a ROS interface node and add it to the executor
    mulexecutor.spin();
    rclcpp::shutdown();
    return 0;
}
