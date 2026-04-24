#ifndef _FAIRINO_VERSION_
#define _FAIRINO_VERSION_

#include <string>
#include "rclcpp/rclcpp.hpp"
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
        bool package_exists(const std::string& pkg_name);
        int _socketfd1;
        int port1 = 8080;//获取8080

        rclcpp::TimerBase::SharedPtr _locktimer;
        std::atomic_bool _robot_recv_exit{false};
        std::thread _reconnect_thread;
        std::mutex sock_mtx_;
        std::string target_lib;
        std::string libdir;
        const std::vector<std::string> stringList = {
            "fairino_hardware_v3_8_2_11",
            "fairino_hardware_v3_8_2_12",
            "fairino_hardware_v3_9_0",
            "fairino_hardware_v3_9_0_1",
            "fairino_hardware_v3_9_4",
            "fairino_hardware_v3_9_5"};//版本的名称


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
