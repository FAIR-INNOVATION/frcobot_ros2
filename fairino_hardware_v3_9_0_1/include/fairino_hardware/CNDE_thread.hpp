#ifndef _CNDE_THREAD_
#define _CNDE_THREAD_

#include "stdint.h"
#include "rclcpp/rclcpp.hpp"
#include "sys/socket.h"
#include "sys/types.h"
#include "sys/mman.h"
#include "fcntl.h"
#include "data_type_def.h"
#include "semaphore.h"
#include "netinet/in.h"
#include "netinet/tcp.h"
#include "arpa/inet.h"


#pragma pack(1)
typedef struct _CNDEState{
    uint16_t head{0};
    uint8_t count{0};
    uint8_t type{0};
    uint16_t len{0};
    double jnt_pos[6];
    double tcp_pos[6];
    uint16_t end{0};
}CNDEState;


typedef struct _OutConfig{
    uint16_t head{0x5A5A};
    uint8_t count{0};
    uint8_t type{0x01};
    uint16_t len{0};
    uint16_t cycle_time{0x05};
    char data[31]{};
    uint16_t end{0xA5A5};
}OutConfig;
#pragma pack()

/**
 * @class CNDE_recv_thread
 * @brief 接收控制器20006端口发送的UDP状态数据结构体并写入共享内存
 */
class CNDE_recv_thread:public rclcpp::Node{
    public:
        explicit CNDE_recv_thread(const std::string node_name);
        ~CNDE_recv_thread();
    private:
        int _socketfd1;
        struct sockaddr_in* udp_client1;

        std::string _controller_ip;
        void _state_recv_callback();
        rclcpp::TimerBase::SharedPtr _locktimer;
        int port1 = 20006;//实时状态数据获取端口
        sem_t* _sem;
        int _shm_fd;
        uint8_t* _shm_state_data;
    };
#endif
