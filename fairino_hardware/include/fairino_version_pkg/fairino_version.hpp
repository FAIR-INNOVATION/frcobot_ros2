#ifndef _FAIRINO_VERSION_
#define _FAIRINO_VERSION_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "fairino_msgs/srv/remote_cmd_interface.hpp"
#include "fairino_msgs/msg/robot_nonrt_state.hpp"
#include "mutex"
#include "sys/socket.h"
#include "sys/types.h"
#include "sys/mman.h"
#include "netinet/in.h"
#include "netinet/tcp.h"
#include "arpa/inet.h"
#include "fcntl.h"
#include <regex>
#include <queue>
#include <atomic>
#include "semaphore.h"
#include <thread>
#include <mutex>


/**
 * @class robot_version_thread
 * @brief 通过控制8080端口，读取机械臂软件版本
 */
class robot_version_thread:public rclcpp::Node{
    public:
        explicit robot_version_thread(const std::string node_name);
        ~robot_version_thread();
    private:
        std::string _controller_ip;
        void _try_to_reconnect();
        int setKeepAlive(int fd, int idle_time, int interval_time, int probe_times);
        void _state_recv_callback();
        void _parseROSCommandData_callback(
            const std::shared_ptr<fairino_msgs::srv::RemoteCmdInterface::Request> req,
            std::shared_ptr<fairino_msgs::srv::RemoteCmdInterface::Response> res);
        bool package_exists(const std::string& pkg_name);
        int _socketfd1;
        int port1 = 8080;//获取8080

        rclcpp::TimerBase::SharedPtr _locktimer;
        std::atomic_bool _robot_recv_exit{false};

        // 重连期间对外提供的占位服务端与话题发布者，供对端一次性连接；
        // 重连成功拉起对应版本硬件节点前关闭，避免与版本化节点同名冲突
        rclcpp::Service<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr _recv_ros_command_server;
        rclcpp::Publisher<fairino_msgs::msg::RobotNonrtState>::SharedPtr _state_publisher;
        std::thread _reconnect_thread;
        std::mutex sock_mtx_;
        std::string target_lib;
        std::string libdir;
        const std::vector<std::string> stringList = {
            "fairino_hardware_v3_9_9",
            "fairino_hardware_v4_0_0",
            "fairino_hardware_v4_0_1"};//版本的名称


    };
#define CHN_VERSION
//#define ENG_VERSION
/*用于ros2_control框架使用的宏定义*/
#define POSITION_CONTROL_MODE 1
#define TORQUE_CONTROL_MODE 2
#define RT_PACKAGE_SIZE sizeof(FR_rt_state)
/******************************/

/*用于指令系统使用的宏定义*/
#define NONRT_PACKAGE_SIZE sizeof(FR_nonrt_state)
/******************************/

/*用于控制器IP地址的宏定义*/
#define CONTROLLER_IP "192.168.58.2"
/******************************/

/*用于接受缓存大小的宏定义*/
#define RECV_BUFF 512
/******************************/


#endif
