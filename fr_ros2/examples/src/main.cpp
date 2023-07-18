#include "rclcpp/rclcpp.hpp"
#include "srv_test_client.hpp"

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::SingleThreadedExecutor singexecutor;
    auto test_node = std::make_shared<srv_test_node>("srv_test_node");
    singexecutor.add_node(test_node);
    singexecutor.spin();
    rclcpp::shutdown();
    return 0;
}