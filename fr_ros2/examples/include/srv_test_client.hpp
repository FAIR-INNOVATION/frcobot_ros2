#ifndef _SRV_TEST_CLIENT_
#define _SRV_TEST_CLIENT_

#define data_count 100
#define time_interval 8ms

#include "rclcpp/rclcpp.hpp"
#include "frhal_msgs/srv/ros_cmd_interface.hpp"

class srv_test_node:public rclcpp::Node{
public:
    explicit srv_test_node(const std::string node_name);
    ~srv_test_node();
    int write_command(const std::string cmd_str);
private:
    void _get_response_callback(rclcpp::Client<frhal_msgs::srv::ROSCmdInterface>::SharedFutureWithRequest future);
    rclcpp::Client<frhal_msgs::srv::ROSCmdInterface>::SharedPtr _client_ptr;
    rclcpp::TimerBase::SharedPtr _locktimer;
    rclcpp::TimerBase::SharedPtr _watchdog;
    double _j1_tor[data_count];
    double _j2_tor[data_count];
    double _j3_tor[data_count];
    double _j4_tor[data_count];
    double _j5_tor[data_count];
    double _j6_tor[data_count];
};



#endif