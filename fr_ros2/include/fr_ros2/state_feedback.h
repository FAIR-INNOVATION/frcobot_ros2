#ifndef STATE_FEEDBACK_H
#define STATE_FEEDBACK_H

#include "string"
#include "sys/socket.h"
#include "sys/types.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "fcntl.h"
#include "rclcpp/rclcpp.hpp"
#include "global_val.h"
#include "std_msgs/msg/string.hpp"
#include "data_type_def.h"
#include "frhal_msgs/msg/fr_state.hpp"

class state_recv_thread:public rclcpp::Node{//Nodes that accept non-real-time and real-time feedback data
public:
    explicit state_recv_thread(const std::string node_name);
    ~state_recv_thread();
private:
    std::string _controller_ip;
    void _state_recv_callback();
    void _rt_state_recv_callback();
    rclcpp::Publisher<frhal_msgs::msg::FRState>::SharedPtr _state_publisher;//In-process communication for sending status data strings
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _rt_state_publisher;//In-process communication for sending status data strings
    rclcpp::TimerBase::SharedPtr _locktimer;
    rclcpp::TimerBase::SharedPtr _rt_locktimer;
    int port1 = 8083;//Non-real-time state data acquisition port
    int port2 = 30004;//Real-time status data acquisition port
    int _socketfd1, _socketfd2;
};

#endif // STATE_FEEDBACK_H
