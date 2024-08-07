#ifndef _FR_ROBOT_
#define _FR_ROBOT_

#include "thread"
#include "data_type_def.h"
#include "xmlrpc/XmlRpc.h"
#include "sys/socket.h"
#include "sys/types.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "fcntl.h"
#include <iostream>
#include <queue>
#include "rclcpp/rclcpp.hpp"

namespace fairino_hardware{
class fairino_robot{
public:
    explicit fairino_robot();
    ~fairino_robot();
    int inittorquecontrol();
    int initpositioncontrol();
    int stoprobot();
    FR_rt_state& read();
    void write(double cmd[6]);//servoJ and servoJT
    friend class FairinoHardwareInterface;
protected:
    std::string _controller_ip;
    std::unique_ptr<std::thread> _control_thread;//输入位置或者扭矩信息
    std::unique_ptr<XmlRpc::XmlRpcClient> _xml_client_ptr;//xmlrcp客户端
    int _port_xmlrpc = 20003;
    int _port_state = 20004;
    int _socket_state;
    int _control_mode;//0-none,1-位置控制,2-扭矩控制
};

}//end namespace
#endif