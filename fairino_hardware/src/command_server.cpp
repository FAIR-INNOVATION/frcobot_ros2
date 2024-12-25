#include "fairino_hardware/command_server.hpp"
#include <sys/types.h>
#include <sys/socket.h>

// #include "/usr/include/c++/11/bits/regex.h"
#include <regex>
#include <stdlib.h>
// #include "std/string.hpp"

std::mutex _reconnect_mutex;

#define LOSE_TCP_CONNECT_TIME_MAX 10

std::string  _start_return_data ="1";//Start指令异常时返回值，默认为1,无异常
int  _program_state;//程序运行状态：1-停止、2-运行、3-暂停、4-拖动


FRAPI_base::FRAPI_base(){
    _cmd_head = "/f/b";
    _cmd_tail = "/b/f";
    _cmd_interval = "III";
}

FRAPI_base::~FRAPI_base(){
    
}

std::string FRAPI_base::command_factry(std::string name, uint16_t counter, std::string data){
    std::string cmd_send;
    std::string cmd_id;
    std::string cmd_data;
    auto iter_cmd =  _fr_command_id.find(name);//在命令列表里面找
    auto iter_script = _fr_script_id.find(name);//在脚本指令列表里面找 
    // auto iter_get = _fr_get_id.find(name);//在get指令列表里找

    if(iter_cmd == _fr_command_id.end() && iter_script != _fr_script_id.end())
    {//在命令列表和get列表里没找到在脚本指令中找到了
        cmd_id = std::to_string(iter_script->second);//枚举类本质是int型数据,因此需要转换成字符串
        cmd_data = data;//脚本命令内容赋值
    }
    else if(iter_cmd != _fr_command_id.end()&& iter_script == _fr_script_id.end())
    {//在命令列表里面找到了在脚本指令和get列表中没找到
        cmd_id = std::to_string(iter_cmd->second);//枚举类本质是int型数据,因此需要转换成字符串
        cmd_data = name + "(" + data + ")";//指令数据转成string  
    }
    // else if(iter_cmd == _fr_command_id.end() && iter_script == _fr_script_id.end())
    // {//两个里面都没找到，说明是运动学接口，不合法的错误函数名称在前面的函数已经被过滤了
    //     if(name == "GetInverseKin"){
    //         cmd_id = "377";
    //     }else if(name == "GetForwardKin"){
    //         cmd_id = "377";
    //     }
    //     cmd_data = name + "(" + data + ")";//指令数据转成string
    // }
    std::string cmd_len = std::to_string(cmd_data.size());//size返回字符串长度,类型是uint,需要转换成字符串
    std::string cmd_counter = std::to_string(counter);
    cmd_send = _cmd_head + _cmd_interval + cmd_counter + _cmd_interval + cmd_id 
               + _cmd_interval + cmd_len + _cmd_interval + cmd_data + _cmd_interval + _cmd_tail;
    // std::cout << "factory: send_str is: " << cmd_send << std::endl;
    return cmd_send;    
}



robot_command_thread::robot_command_thread(const std::string node_name):FRAPI_base(),rclcpp::Node(node_name,
           rclcpp::NodeOptions().use_intra_process_comms(true))
{
    using namespace std::chrono_literals;
    _cmd_counter = 0;
    _script_counter = 0;
    _recv_data_cmdcount = 0;
    this->declare_parameter<uint8_t>("toolcoord_install",0);//默认工具安装在机器人末端
    this->declare_parameter<uint8_t>("toolcoord_type",0);//默认是工具坐标系
    this->declare_parameter<uint8_t>("collision_mode",0);//碰撞等级模式,默认是等级
    this->declare_parameter<uint8_t>("collision_config",1);//碰撞配置文件设置,默认不更新配置文件
    this->declare_parameter<uint8_t>("gripper_vel",50);
    this->declare_parameter<uint8_t>("gripper_force",50);
    this->declare_parameter<uint8_t>("gripper_maxtime",30000);
    this->declare_parameter<uint8_t>("gripper_block",1);//默认夹爪控制非阻塞
    this->declare_parameter<uint8_t>("DO_smooth",0);//默认DO不平滑
    this->declare_parameter<uint8_t>("DO_block",1);//默认DO非阻塞
    this->declare_parameter<uint8_t>("AO_block",1);//默认AO非阻塞
    this->declare_parameter<uint8_t>("JOG_acc",40);//JOG默认加速度40
    this->declare_parameter<int>("JOG_maxdis",5);//JOG默认单步5mm
    this->declare_parameter<int>("MoveJLC_tool",0);
    this->declare_parameter<int>("MoveJLC_user",0);
    this->declare_parameter<float>("MoveJLC_acc",0);
    this->declare_parameter<float>("MoveJLC_ovl",100);
    this->declare_parameter<double>("MoveJLC_eaxis1",0);
    this->declare_parameter<double>("MoveJLC_eaxis2",0);
    this->declare_parameter<double>("MoveJLC_eaxis3",0);
    this->declare_parameter<double>("MoveJLC_eaxis4",0);
    this->declare_parameter<float>("MoveJ_blendT",500);
    this->declare_parameter<float>("MoveL_blendR",500);
    this->declare_parameter<float>("MoveC_blendR",500);
    this->declare_parameter<float>("MoveL_search",0);
    this->declare_parameter<uint8_t>("MoveJLC_offset_flag",0);
    this->declare_parameter<double>("MoveJLC_offset_pos_x",0);
    this->declare_parameter<double>("MoveJLC_offset_pos_y",0);
    this->declare_parameter<double>("MoveJLC_offset_pos_z",0);
    this->declare_parameter<double>("MoveJLC_offset_pos_rx",0);
    this->declare_parameter<double>("MoveJLC_offset_pos_ry",0);
    this->declare_parameter<double>("MoveJLC_offset_pos_rz",0);
    this->declare_parameter<int>("Spline_tool",1);
    this->declare_parameter<int>("Spline_user",0);
    this->declare_parameter<float>("Spline_acc",0);
    this->declare_parameter<float>("Spline_ovl",100);
    this->declare_parameter<float>("NewSpline_blendR",10);

    _recv_ros_command_server = this->create_service<remote_cmd_server_srv_msg>(
        REMOTE_CMD_SERVER_NAME,
        std::bind(&robot_command_thread::_ParseROSCommandData_callback,this,std::placeholders::_1,std::placeholders::_2)
    );

    _recv_ros_script_server = this->create_service<remote_script_srv_msg>(
        REMOTE_SCRIPT_SERVER_NAME,
        std::bind(&robot_command_thread::_ParseROSScript_callback,this,std::placeholders::_1,std::placeholders::_2)
    );

    _controller_ip = CONTROLLER_IP;//控制器默认ip地址
    RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Start to create state feedback TCP socket.");
    // std::cout << "开始创建TCP socket  version:2024-05-24" << std::endl;
    _socketfd1 = socket(AF_INET,SOCK_STREAM,0);
    _socketfd2 = socket(AF_INET,SOCK_STREAM,0);
    if(_socketfd1 == -1 || _socketfd2 == -1){
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1001:Failed to create state feedback TCP socket");
        // std::cout << "错误: 创建socket失败!" << std::endl;
        exit(0);//创建套字失败,丢出错误
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"The socket command was created successfully. Now connect to the controller...");
        // std::cout << "创建指令socket成功,现在开始连接控制器..." << std::endl;
        struct sockaddr_in tcp_client1,tcp_client2;
        tcp_client1.sin_family = AF_INET;
        tcp_client2.sin_family = AF_INET;
        tcp_client1.sin_port = htons(port1);//8080端口
        tcp_client2.sin_port = htons(port2);//8082端口
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());
        tcp_client2.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

        //尝试连接控制器
        int res1 = connect(_socketfd1,(struct sockaddr *)&tcp_client1,sizeof(tcp_client1));
        int res2 = connect(_socketfd2,(struct sockaddr *)&tcp_client2,sizeof(tcp_client2));
        if(res1 || res2){
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1002:Can not connect to robot controller, program exit!");
            // std::cout << "错误:无法连接控制器数据端口,程序退出!" << std::endl;
            close(_socketfd1);
            close(_socketfd2);
            exit(0);//连接失败,丢出错误并返回
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Connected to robot controller.");
            // std::cout << "控制器指令端口连接成功" << std::endl;
            
            //设置TCP接收超时
            struct timeval timeout_val;
            timeout_val.tv_sec = 1;//1s超时
            timeout_val.tv_usec = 0;
            setsockopt(_socketfd1,SOL_SOCKET,SO_RCVTIMEO,&timeout_val,sizeof(timeout_val));
            setsockopt(_socketfd2,SOL_SOCKET,SO_RCVTIMEO,&timeout_val,sizeof(timeout_val));

            //开启并设置keepalive
            if(0 != setKeepAlive(_socketfd1, 5, 3, 3))
            {
                RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"Failed to enable TCP keepalive probes.");
                // RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"开启tcp保活探针失败");
            }
            if(0 != setKeepAlive(_socketfd2, 5, 3, 3))
            {
                RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"Failed to enable TCP keepalive probes.");
                // RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"开启tcp保活探针失败");
            }
        }
    }
}

/**
 * @brief 关闭旧套接字，发起重连 
 * @brief fd 旧的套接字
 * @param port 重连端口
 * @return -1 失败 / fd 成功
*/
int robot_command_thread::reConnect_tcp(int fd,int port)
{
    //关闭旧连接
    shutdown(fd, SHUT_RDWR);
    close(fd);
    sleep(100);

    int sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (-1 == sock_fd)
    {
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect: Failed to create socket.");
        // std::cout << "重连：创建套接字失败" << std::endl;
        return -1;
    }
    else
    {
        struct sockaddr_in tcp_client1;
        tcp_client1.sin_family = AF_INET;
        tcp_client1.sin_port = htons(port);
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

        // 尝试连接控制器
        int res1 = connect(sock_fd, (struct sockaddr *)&tcp_client1, sizeof(tcp_client1));
        if (res1)
        {
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect: Failed to initiate reconnection, program exiting!");
            // std::cout << "重连：发起重新连接失败,程序退出!" << std::endl;
            shutdown(sock_fd, SHUT_RDWR);
            close(sock_fd);
            return -1; 
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect: Reconnection initiated successfully.");
            // std::cout << "重连：发起重新连接连接成功" << std::endl;
            // 设置TCP接收超时
            struct timeval timeout_val;
            timeout_val.tv_sec = 1; // 1s超时
            timeout_val.tv_usec = 0;
            setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout_val, sizeof(timeout_val));

            //开启并设置keepalive
            if(0 != setKeepAlive(sock_fd, 5, 3, 3))
            {
                RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"Failed to enable TCP keepalive probes.");
                // RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"开启tcp保活探针失败");
            }
            return sock_fd;
        }
    }
}

robot_command_thread::~robot_command_thread()
{
    if (-1 != _socketfd1)
    {
        shutdown(_socketfd1, SHUT_RDWR);
        close(_socketfd1);
    }
    if(-1 != _socketfd2)
    {
        shutdown(_socketfd2, SHUT_RDWR);
        close(_socketfd2);
    }
}

std::string robot_command_thread::_send_data_factory_callback(std::string data)
{
    static char recv_buff[RECV_BUFF];
    RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Send command information...%s",data.c_str());
    // std::cout << "发送指令信息..." << data << std::endl;
    send(_socketfd1, data.c_str(), data.size(), 0); // 发送指令信息
    // std::cout << "发送指令成功!" << std::endl;
    memset(recv_buff, 0, sizeof(recv_buff));
    // rclcpp::sleep_for(30ms);
    int recv_bytes = recv(_socketfd1, recv_buff, sizeof(recv_buff), 0);
    if (recv_bytes > 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Received command response information...%s",recv_buff);
        // std::cout << "收到指令回复信息..." << std::string(recv_buff) << std::endl;
        std::cout << _ParseRecvData(std::string(recv_buff)) << std::endl;
        if (_ParseRecvData(std::string(recv_buff))!="0")
        {
            if (_recv_data_cmdid == 377)
            {
                // std::cout << "运动学接口返回参数" << std::endl;
                return std::to_string(-2001);
            }
            else if (_recv_data_cmdcount == _cmd_counter) // id和帧计数器能对上,说明对应回复信息是正确的
            {
                // std::cout << "设置函数接口返回参数" << std::endl;
                return _recv_get_data_res;
            }
            else if(_recv_data_cmdcount != _cmd_counter)//id和帧计数器没对上,再取一次
            {
                recv_bytes = recv(_socketfd1, recv_buff, sizeof(recv_buff), 0);
                if(recv_bytes > 0)
                {
                    RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Frame count does not match, retrieving again. received command response information...%s",recv_buff);
                    // std::cout << "帧计数对不上，再取一次，收到指令回复信息..." << std::string(recv_buff) << std::endl;
                    if (_ParseRecvData(std::string(recv_buff))!="0")
                    {
                        if (_recv_data_cmdid == 377)
                        {
                            // std::cout << "运动学接口返回参数" << std::endl;
                            return std::to_string(-2001);
                        }
                        else if (_recv_data_cmdcount == _cmd_counter) // id和帧计数器能对上,说明对应回复信息是正确的
                        {
                            // std::cout << "设置函数接口返回参数" << std::endl;
                            return _recv_get_data_res;
                        }
                        else
                        {
                            // std::cout << "未处理的情况,默认返回0" << std::endl;
                            return std::to_string(0);
                        }
                    }

                }

            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"An unhandled case.Default returns 0.");
                // std::cout << "未处理的情况,默认返回0" << std::endl;
                return std::to_string(0);
            }
        }
    }
    else if (recv_bytes == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Received 0 bytes, please check whether the network has been disconnected.");
        // std::cout << "接收到0字节，请检查网络是否已经断开" << std::endl;
        return std::to_string(0);
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Failure to receive.%s(error code:%d)",strerror(errno),errno);
        // std::cout << "接收失败" << " " << strerror(errno) << errno << std::endl;
        lose_connect_times++;
        if((lose_connect_times >= LOSE_TCP_CONNECT_TIME_MAX) || (_socketfd1 == -1))
        {
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Try to reconnect.");
            // std::cout << "try to reconnect." << std::endl;
            int fd = reConnect_tcp(_socketfd1, 8080);
            if(fd == -1)
            {
                RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect failed.");
                // std::cout << "重连失败" << std::endl;
                _socketfd1 = -1;
            }
            else{
                _socketfd1 = fd;
                RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect successfully.%d",_socketfd1);
                // std::cout << "重连成功 " << _socketfd1 << std::endl;
                fd = -1;
            }
            lose_connect_times = 0;
        }
        return std::to_string(0);
    }
}

std::string robot_command_thread::_send_script_data_callback(std::string data){
    static char recv_buff[RECV_BUFF];
    RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Send command information...%s",data.c_str());
    // std::cout << "发送指令信息..." << data << std::endl;
    send(_socketfd2,data.c_str(),data.size(),0);//发送指令信息
    //std::cout << "发送指令成功!" << std::endl;
    memset(recv_buff,0,sizeof(recv_buff));

    int recv_bytes = recv(_socketfd2,recv_buff,sizeof(recv_buff),0);
    if(recv_bytes > 0){
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Received command response information...%s",recv_buff);
        // std::cout << "收到指令回复信息..." << std::string(recv_buff) << std::endl;
        if(_ParseRecvData(std::string(recv_buff))!="0"){
            if(_recv_data_cmdcount == _script_counter){//id和帧计数器能对上,说明对应回复信息是正确的
                    //std::cout << "设置函数接口返回参数" << std::endl;
                return _recv_get_data_res;
            }else{
                //std::cout << "未处理的情况,默认返回0" << std::endl;
                return std::to_string(0);
            }
        }
    }else if(recv_bytes == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Received 0 bytes, please check whether the network has been disconnected.");
        // std::cout << "接收到0字节，请检查网络是否已经断开" << std::endl;
        return std::to_string(0);
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Failure to receive.%s(error code:%d)",strerror(errno),errno);
        // std::cout << "接收失败" << " " << strerror(errno) << errno << std::endl;
        lose_connect_times++;
        if((lose_connect_times >= LOSE_TCP_CONNECT_TIME_MAX) || (_socketfd2 == -1))
        {
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Try to reconnect.");
            // std::cout << "try to reconnect." << std::endl;
            int fd = reConnect_tcp(_socketfd2, 8082);
            if(fd == -1)
            {
                RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect failed.");
                // std::cout << "重连失败" << std::endl;
                _socketfd2 = -1;
            }
            else{
                _socketfd2 = fd;
                RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect successfully.%d",_socketfd2);
                // std::cout << "重连成功 " << _socketfd2 << std::endl;
                fd = -1;
            }
            lose_connect_times = 0;
        }
        return std::to_string(0);
    }
}
std::string robot_command_thread::getbuff(std::string para){
    std::cout << "getbuff" << std::endl;
    static char recv_buff[RECV_BUFF];
    memset(recv_buff, 0, sizeof(recv_buff));
    int recv_bytes = recv(_socketfd1, recv_buff, sizeof(recv_buff), 0);
    std::cout << "getbuff" << std::string(recv_buff) << std::endl;
    if(recv_bytes > 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Getbuff, throws an exception, fetches the extra error in the buff, fetches it again, and receives the instruction reply information...%s",recv_buff);
        // std::cout << "getbuff,抛出异常，取buff中多余错误，再取一次，收到指令回复信息..." << std::string(recv_buff) << std::endl;
        if (_ParseRecvData(std::string(recv_buff))!="0")
        {
            if (_recv_data_cmdid == 377)
            {
                // std::cout << "运动学接口返回参数" << std::endl;
                return std::to_string(-2001);
            }
            else if (_recv_data_cmdcount == _cmd_counter) // id和帧计数器能对上,说明对应回复信息是正确的
            {
                // std::cout << "设置函数接口返回参数" << std::endl;
                return _recv_get_data_res;
            }
            else
            {
                // std::cout << "未处理的情况,默认返回0" << std::endl;
                return std::to_string(0);
            }
        }

    }

}

// std::string robot_command_thread::_send_get_data_factory_callback(std::string data){
//     static char recv_buff[RECV_BUFF];
//     std::cout << "发送指令信息..." << data << std::endl;
//     send(_socketfd1,data.c_str(),data.size(),0);//发送指令信息
//     //std::cout << "发送指令成功!" << std::endl;
//     memset(recv_buff,0,sizeof(recv_buff));
//         //rclcpp::sleep_for(30ms);
//     sleep(1);
    
//     int recv_bytes = recv(_socketfd1,recv_buff,sizeof(recv_buff),0);
//     if(recv_bytes > 0)
//     {
//         std::cout << "收到指令回复信息..." << std::string(recv_buff) << std::endl;
//         if(_ParseGetRecvData(std::string(recv_buff))!="0")
//         {
//           if(_recv_data_cmdcount == _cmd_counter)//id和帧计数器能对上,说明对应回复信息是正确的
//             {
//                 std::cout << "设置函数接口返回参数" << std::endl;
//                 std::cout << _recv_get_data_res<< std::endl;
//                 return _recv_get_data_res;

//             }else
//             {
//                 std::cout << "未处理的情况,默认返回0" << std::endl;
//                 return "0";
//             }
//         }
//     }
//     else if(recv_bytes == 0)
//     {
//         std::cout << "接收到0字节，请检查网络是否已经断开" << std::endl;
//         return "0";
//     }else{
//         std::cout << "接收失败" << " " << strerror(errno) << errno << std::endl;
//         lose_connect_times++;
//         if((lose_connect_times >= LOSE_TCP_CONNECT_TIME_MAX) || (_socketfd1 == -1))
//         {
//             std::cout << "try to reconnect." << std::endl;
//             int fd = reConnect_tcp(_socketfd1, 8080);
//             if(fd == -1)
//             {
//                 std::cout << "重连失败" << std::endl;
//                 _socketfd1 = -1;
//             }
//             else{
//                 _socketfd1 = fd;
//                 std::cout << "重连成功 " << _socketfd1 << std::endl;
//                 fd = -1;
//             }
//             lose_connect_times = 0;
//         }
//         return "0";
//     }
// }

std::string robot_command_thread::_ParseRecvData(std::string str){
    std::regex pattern("/f/bIII(\\d*?)III(\\d*?)III(\\d*)III(.*?)III/b/f");
    std::smatch data_match;
    if(std::regex_match(str,data_match,pattern)){//判断帧数据是否符合模式
        //_mtx.lock();
        _recv_data_cmdcount = std::atol(data_match[1].str().c_str());
        _recv_data_cmdid = std::atol(data_match[2].str().c_str());
        if(_recv_data_cmdid == 377)
        {//运动学接口反馈回来的信息
            std::string kin_data = data_match[4];
            std::regex kin_pattern("(.*?),(.*?),(.*?),(.*?),(.*?),(.*?)");
            std::smatch kin_match;
            if(std::regex_match(kin_data,kin_match,kin_pattern)){
                for(int i=0;i<6;i++){
                    _kin_res[i] = atof(kin_match[i+1].str().c_str());
                }
            }else{
                RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Parsing error: The kinematic interface returned incorrect information.");
                // std::cout << "解析错误:运动学接口返回信息错误" << std::endl;
                return "0";
            }
            std::cout<<kin_data<<std::endl;
            _recv_get_data_res  = data_match[4];
            return _recv_get_data_res;
        }
        else if(_recv_data_cmdid == 842){//获取DH参数
            _recv_get_data_res  = data_match[4];
            return _recv_get_data_res;
        }
        else if(_recv_data_cmdid == 500){
            _recv_get_data_res  = data_match[4];
            return _recv_get_data_res;
        }
        else{//一般控制指令或者脚本指令的反馈信息,单一int型
            
            _recv_get_data_res = data_match[4].str().c_str();
            if((_recv_get_data_res=="1")&&(_recv_get_data_res=="0"))
            {
                
                std::cout << "res is:" << _recv_get_data_res << std::endl;
                return "1";
            }
            else
            {
                _start_recv_res = data_match[4].str().c_str();
                return _start_recv_res;
            }

        }
        //_mtx.unlock();
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Parse error: Incomplete communication information received, discard the frame content.");
        // std::cout << "解析错误：收到通讯信息不完整,丢弃该帧内容" << std::endl;
        return "0";
    }
}

// std::string robot_command_thread::_ParseGetRecvData(std::string str)
// {
//     std::regex pattern("/f/bIII(\\d*?)III(\\d*?)III(\\d*)III(.*?)III/b/f");
//     std::smatch data_match;
//     if(std::regex_match(str,data_match,pattern))
//     {//判断帧数据是否符合模式
//         //_mtx.lock();
//         _recv_data_cmdcount = std::atol(data_match[1].str().c_str());
//         _recv_data_cmdid = std::atol(data_match[2].str().c_str());
//         if(_recv_data_cmdid == 377){//运动学接口反馈回来的信息
//             _recv_get_data_res  = data_match[4];
//             return _recv_get_data_res;
//         }
//         else if(_recv_data_cmdid == 842){//获取DH参数
//             _recv_get_data_res  = data_match[4];
//             return _recv_get_data_res;
//         }
//         else if(_recv_data_cmdid == 500){
//             _recv_get_data_res  = data_match[4];
//             return _recv_get_data_res;
//         }else{
//             std::cout << "解析错误:返回指令编号错误" << std::endl;
//             return "0";
//         }
//     }
//     else{
//         std::cout << "解析错误：收到通讯信息不完整,丢弃该帧内容" << std::endl;
//         return "0";
//     }    
// }


void robot_command_thread::_ParseROSCommandData_callback(const std::shared_ptr<remote_cmd_server_srv_msg::Request> req,\ 
                                            std::shared_ptr<remote_cmd_server_srv_msg::Response> res)//用于解析用户发送的ROS接口指令
{//指令格式为movj(1,10)
    std::regex func_reg("([A-Z|a-z|_]+)[(](.*)[)]");//函数名的输入模式应该是字母函数名后跟(),圆括号中有所有输入参数
    std::smatch func_match;
    if(std::regex_match(req->cmd_str,func_match,func_reg))
    {
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Receive ROS instruction:%s",req->cmd_str.c_str());
        // std::cout << "收到ROS指令: " <<  req->cmd_str.data() <<std::endl;
        std::string func_name = func_match[1];
        std::string para_list = func_match[2];
        std::regex para_pattern("[A-Z|a-z|\\.\\d|\\d|,|-]*");//校验参数的内容,参数部分必须是字母,数字和逗号,负号组成,出现其他字符包括空格都会导致校验失败,
        // std::regex para_pattern("[A-Za-z\\d.\\-,]*");//校验参数的内容,参数部分必须是字母,数字和逗号,负号组成,出现其他字符包括空格都会导致校验失败,
        if(std::regex_match(para_list,para_pattern)){//检查参数输入是否合法
            if(func_name == "GET"){
                res->cmd_res = _get_variable(para_list);
            }else{
                auto find_idx = _fr_function_list.find(func_name);
                if(find_idx == _fr_function_list.end()){
                    RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction Error: The function for this instruction could not be found.");
                    // std::cout << "指令错误: 找不到该指令对应的函数" << std::endl;
                    res->cmd_res = std::string("0");
                }else if(find_idx != _fr_function_list.end()){
                    // res->cmd_res = std::to_string((this->*(find_idx->second))(para_list));
                    res->cmd_res = (this->*(find_idx->second))(para_list);
                }
                // auto find_idx_get = _fr_get_list.find(func_name);
                // if(find_idx == _fr_function_list.end()&& find_idx_get  == _fr_get_list.end()){
                //     std::cout << "指令错误: 找不到该指令对应的函数" << std::endl;
                //     res->cmd_res = std::string("0");
                // }else if(find_idx != _fr_function_list.end()&& find_idx_get  == _fr_get_list.end()){
                //     // res->cmd_res = std::to_string((this->*(find_idx->second))(para_list));
                //     res->cmd_res = (this->*(find_idx->second))(para_list);
                // }
                // else if(find_idx == _fr_function_list.end()&& find_idx_get  != _fr_get_list.end()){
                //     res->cmd_res = (this->*(find_idx_get->second))(para_list);
                // }
            }
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: The function parameter input is not valid. The parameter list is composed of letters, numbers and commas, and no Spaces can appear.");
            // std::cout << "指令错误:函数参数输入不合法,参数列表由字母,数字和逗号组成,不能有空格出现" <<std::endl;
            res->cmd_res = std::string("0");
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: The form of the function input is wrong, the function input must be [function name]() this form of input, please re-enter.");
        // std::cout << "指令错误:函数输入形式错误,函数输入必须是 [函数名]() 这种输入形式,请重新输入" << std::endl;
        res->cmd_res = std::string("0");
    }
}

void robot_command_thread::_ParseROSScript_callback(const std::shared_ptr<remote_script_srv_msg::Request> req,
                                        std::shared_ptr<remote_script_srv_msg::Response> res)
{
    static std::string script_content = "";
    std::cout << "row recv: "<< req->line_str << std::endl;
    if(std::regex_search(req->line_str,std::regex("fr_script_start:"))){//找到开始的字符，并取出程序名开启脚本传输
        script_content.clear();
        std::regex spt_reg("(.*)[:](.*)");
        std::smatch spt_match;
        if(std::regex_match(req->line_str,spt_match,spt_reg)){
            //spt_match[1];//"fr_script_start"
            std::string script_name = spt_match[2];//script name
            res->res = _send_script_data_callback(FRAPI_base::command_factry("ScriptName",++_script_counter,script_name));
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: Script name not found.");
            // std::cout << "指令错误:未找到脚本名称" << std::endl;
        }

    }else if(req->line_str == "fr_script_end"){//脚本传输结束
        res->res = _send_script_data_callback(FRAPI_base::command_factry("ScriptContent",++_script_counter,script_content));
    }else{//脚本正文部分
        script_content += req->line_str;
        res->res = "1";
    }


}




std::string robot_command_thread::_def_jnt_position(std::string pos){
    //std::regex pattern("[-|\\.|,|\\d]*"); //参数模式应该是所有参数都是数字和逗号
    std::regex pattern("[\\d.\\-,]*"); //参数模式应该是所有参数都是数字和逗号
    std::smatch para_match;
    if(std::regex_match(pos,para_match,pattern)){//进行参数正确性判断
        std::smatch data_match;
        std::regex search_para(",");//分隔符
        std::regex_token_iterator<std::string::iterator> iter_data(pos.begin(),pos.end(),search_para,-1);
        decltype(iter_data) end;
        int count = 0;
        for(;iter_data != end;iter_data++){
            count++;
        }
        if(count != 7){
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: joint position parameters are 6, please confirm the number of parameters input.");
            // std::cout << "指令错误:关节位置参数为6个,参数输入个数请确认" << std::endl;
            return std::to_string(0);
        }std::cout<<_recv_data_res<<std::endl;
        iter_data = std::regex_token_iterator<std::string::iterator>(pos.begin(),pos.end(),search_para,-1);
        int idx = atol(iter_data->str().c_str());//指令序号
        iter_data++;
        if(idx > _cmd_jnt_pos_list.size()+1 || idx <= 0){//如果大于当前容器最大值+1,那么要报错,因为序列容器中间无法留空
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: Container sequence number is out of limit.");
            // std::cout << "指令错误:容器序号超限" << std::endl;
            return std::to_string(0);
        }else if(idx <= _cmd_jnt_pos_list.size()){//如果是小于等于当前的容量,那么就是点位信息覆盖
            int i=0;
            for(;iter_data != end;iter_data++,i++){
                _cmd_jnt_pos_list.at(idx-1).jPos[i] = atof(iter_data->str().c_str());
            }
        }else if(idx == _cmd_jnt_pos_list.size()+1){//添加元素
            JointPos pos;
            int i=0;
            for(;iter_data != end;iter_data++,i++){
                pos.jPos[i] = atof(iter_data->str().c_str());
            }
            _cmd_jnt_pos_list.push_back(pos);
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: The first one is the storage sequence number, and the subsequent one is the joint position information, separated by commas, and no space can appear.");
        // std::cout << "指令错误：关节点位输入参数规则为第一个为存储序号,后续为关节位置信息,以逗号隔开,不能出现空格" << std::endl;
        return std::to_string(0);
    }
    return std::to_string(1);
}

std::string robot_command_thread::_def_cart_position(std::string pos){
    //std::regex pattern("[\\.\\d|\\d|,|-]*"); //参数模式应该是所有参数都是数字
    std::regex pattern("[\\d.\\-,]*"); //参数模式应该是所有参数都是数字
    std::smatch para_match;
    if(std::regex_match(pos,para_match,pattern)){
        std::smatch data_match;
        std::regex search_para(",");//分隔符
        std::regex_token_iterator<std::string::iterator> iter_data(pos.begin(),pos.end(),search_para,-1);
        decltype(iter_data) end;
        int count = 0;
        for(;iter_data != end;iter_data++){
            count++;
        }
        if(count != 7){
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: 6 Cartesian position parameters, please confirm the number of parameters input.");
            // std::cout << "指令错误:笛卡尔位置参数为6个,参数输入个数请确认" << std::endl;
            return std::to_string(0);
        }
        iter_data = std::regex_token_iterator<std::string::iterator>(pos.begin(),pos.end(),search_para,-1);
        int idx = atol(iter_data->str().c_str());//指令序号
        iter_data++;
        if(idx > _cmd_cart_pos_list.size()+1 || idx <= 0){//如果大于当前容器最大值+1,那么要报错,因为序列容器中间无法留空
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: Container sequence number is out of limit.");
            // std::cout << "指令错误:容器序号超限" << std::endl;
            return std::to_string(0);
        }else if(idx <= _cmd_cart_pos_list.size()){//如果是小于等于当前的容量,那么就是点位信息覆盖
            _cmd_cart_pos_list.at(idx-1).tran.x = atof(iter_data->str().c_str());iter_data++;
            _cmd_cart_pos_list.at(idx-1).tran.y = atof(iter_data->str().c_str());iter_data++;
            _cmd_cart_pos_list.at(idx-1).tran.z = atof(iter_data->str().c_str());iter_data++;
            _cmd_cart_pos_list.at(idx-1).rpy.rx = atof(iter_data->str().c_str());iter_data++;
            _cmd_cart_pos_list.at(idx-1).rpy.ry = atof(iter_data->str().c_str());iter_data++;
            _cmd_cart_pos_list.at(idx-1).rpy.rz = atof(iter_data->str().c_str());
        }else if(idx == _cmd_cart_pos_list.size()+1){//添加元素
            DescPose add_pos;
            add_pos.tran.x = atof(iter_data->str().c_str());iter_data++;
            add_pos.tran.y = atof(iter_data->str().c_str());iter_data++;
            add_pos.tran.z = atof(iter_data->str().c_str());iter_data++;
            add_pos.rpy.rx = atof(iter_data->str().c_str());iter_data++;
            add_pos.rpy.ry = atof(iter_data->str().c_str());iter_data++;
            add_pos.rpy.rz = atof(iter_data->str().c_str());
            _cmd_cart_pos_list.push_back(add_pos);
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: The Cartesian point input parameter rule is that the first one is the storage sequence number, and the subsequent one is the Cartesian position information, separated by commas, and no Spaces can appear.");
        // std::cout << "指令错误：笛卡尔点位输入参数规则为第一个为存储序号,后续为笛卡尔位置信息,以逗号隔开,不能出现空格" << std::endl;
        return std::to_string(0);
    }
    return std::to_string(1);
}

std::string robot_command_thread::_get_variable(std::string para_list){
    std::regex pattern("([A-Z]*),([0-9]*)");//参数模式应该是 JNT,数字 或 CART,数字 的形式
    std::smatch para_match;
    if(std::regex_match(para_list,para_match,pattern)){
        if(para_match[1] == "JNT"){
            int idx = atol(para_match[2].str().c_str());
            if(idx <= _cmd_jnt_pos_list.size() && idx > 0){
                JointPos pos = _cmd_jnt_pos_list.at(idx-1);
                std::string res = std::to_string(pos.jPos[0]) + "," +\
                                  std::to_string(pos.jPos[1]) + "," +\
                                  std::to_string(pos.jPos[2]) + "," +\
                                  std::to_string(pos.jPos[3]) + "," +\
                                  std::to_string(pos.jPos[4]) + "," +\
                                  std::to_string(pos.jPos[5]);
                return res;
            }else{
                RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: The sequence number of the input point is out of range.");
                // std::cout << "指令错误:输入点的序号超出范围" << std::endl;
            }
        }else if(para_match[1] == "CART"){
                int idx = atol(para_match[2].str().c_str());
                if(idx <= _cmd_cart_pos_list.size() && idx > 0){
                    DescPose pos = _cmd_cart_pos_list.at(idx-1);
                    std::string res = std::to_string(pos.tran.x) + "," +\
                                          std::to_string(pos.tran.y) + "," +\
                                          std::to_string(pos.tran.z) + "," +\
                                          std::to_string(pos.rpy.rx) + "," +\
                                          std::to_string(pos.rpy.ry) + "," +\
                                          std::to_string(pos.rpy.rz);
                    return res;
                }else{
                    RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: The sequence number of the input point is out of range.");
                    // std::cout << "指令错误:输入点的序号超出范围" << std::endl;
                }
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: Invalid GET instruction parameter.");
            // std::cout << "指令错误: 无效的GET指令参数" << std::endl;
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: GET instruction parameter is invalid, parameter form is [JNT|CART],[serial number].");
        // std::cout << "指令错误: GET指令参数非法,参数形式为[JNT|CART],[序号]" << std::endl;
    }
}

std::string  robot_command_thread::DragTeachSwitch(std::string para){//拖动示教模式切换
    return _send_data_factory_callback(FRAPI_base::command_factry("DragTeachSwitch",++_cmd_counter,para));
}

std::string  robot_command_thread::RobotEnable(std::string para){//机械臂使能
    return _send_data_factory_callback(FRAPI_base::command_factry("RobotEnable",++_cmd_counter,para));
}

std::string robot_command_thread::Mode(std::string para){//手动模式,自动模式切换
    return _send_data_factory_callback(FRAPI_base::command_factry("Mode",++_cmd_counter,para));
}

std::string robot_command_thread::SetSpeed(std::string para){
    return _send_data_factory_callback(FRAPI_base::command_factry("SetSpeed",++_cmd_counter,para));
}

std::string robot_command_thread::SetToolCoord(std::string para){
    //int id, DescPose *coord, int type, int install
    std::string install,type;
    install = this->get_parameter("toolcoord_install").value_to_string();
    type = this->get_parameter("toolcoord_type").value_to_string();
    para = para + "," + type + "," + install;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetToolCoord",++_cmd_counter,para));
}

std::string robot_command_thread::SetToolList(std::string para){
    //int id, DescPose *coord, int type, int install
    std::string install,type;
    install = this->get_parameter("toolcoord_install").value_to_string();
    type = this->get_parameter("toolcoord_type").value_to_string();
    para = para + "," + type  + "," + install;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetToolList",++_cmd_counter,para));
}

std::string robot_command_thread::SetExToolCoord(std::string para){
    //int id, DescPose *etcp, DescPose *etool
    return _send_data_factory_callback(FRAPI_base::command_factry("SetExToolCoord",++_cmd_counter,para));
}

std::string robot_command_thread::SetExToolList(std::string para){
    //int id, DescPose *etcp, DescPose *etool
    return _send_data_factory_callback(FRAPI_base::command_factry("SetExToolList",++_cmd_counter,para));
}

std::string robot_command_thread::SetWObjCoord(std::string para){
    //int id, DescPose *coord
    return _send_data_factory_callback(FRAPI_base::command_factry("SetWObjCoord",++_cmd_counter,para));
}

std::string robot_command_thread::SetWObjList(std::string para){
    //int id, DescPose *coord
    return _send_data_factory_callback(FRAPI_base::command_factry("SetWObjList",++_cmd_counter,para));
}

std::string robot_command_thread::SetLoadWeight(std::string para){
    //float weight
    return _send_data_factory_callback(FRAPI_base::command_factry("SetLoadWeight",++_cmd_counter,para));
}
    
std::string robot_command_thread::SetLoadCoord(std::string para){
    //DescTran *coord
    return _send_data_factory_callback(FRAPI_base::command_factry("SetLoadCoord",++_cmd_counter,para));
}

std::string robot_command_thread::SetRobotInstallPos(std::string para){
    //uint8_t install
    return _send_data_factory_callback(FRAPI_base::command_factry("SetRobotInstallPos",++_cmd_counter,para));
}

std::string robot_command_thread::SetRobotInstallAngle(std::string para){
    //double yangle, double zangle
    return _send_data_factory_callback(FRAPI_base::command_factry("SetRobotInstallAngle",++_cmd_counter,para));
}

//安全配置
std::string robot_command_thread::SetAnticollision(std::string para){
    //int mode, float level[6], int config
    std::string mode,config;
    mode = this->get_parameter("collision_mode").value_to_string();
    config = this->get_parameter("collision_config").value_to_string();
    para = mode + "," + "{" + para + "}" + "," + config;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetAnticollision",++_cmd_counter,para));
}

std::string robot_command_thread::SetCollisionStrategy(std::string para){
    //int strategy
    // 使用字符串流分割字符串
    std::stringstream ss(para);
    std::string item;
    std::vector<std::string> params;

    // 分割字符串并存储到向量中
    while (std::getline(ss, item, ',')) {
        params.push_back(item);
    }

    // 创建新的字符串
    std::string nested_part = "{" + params[3] + "," + params[4] + "," + params[5] + "," +
                               params[6] + "," + params[7] + "," + params[8] + "}";
    
    // 组合最终字符串
    std::string new_para = params[0] + "," + params[1] + "," + params[2] + "," + nested_part;

    // 将结果赋值回 para
    para = new_para;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetCollisionStrategy",++_cmd_counter,para));
}

std::string robot_command_thread::SetLimitPositive(std::string para){
    //float limit[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetLimitPositive",++_cmd_counter,para));
}

std::string robot_command_thread::SetLimitNegative(std::string para){
    //float limit[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetLimitNegative",++_cmd_counter,para));
}

std::string robot_command_thread::ResetAllError(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("ResetAllError",++_cmd_counter,para));
}

std::string robot_command_thread::FrictionCompensationOnOff(std::string para){
    //uint8_t state
    return _send_data_factory_callback(FRAPI_base::command_factry("FrictionCompensationOnOff",++_cmd_counter,para));
}

std::string robot_command_thread::SetFrictionValue_level(std::string para){
    //float coeff[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetFrictionValue_level",++_cmd_counter,para));
}

std::string robot_command_thread::SetFrictionValue_wall(std::string para){
    //float coeff[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetFrictionValue_wall",++_cmd_counter,para));
}

std::string robot_command_thread::SetFrictionValue_ceiling(std::string para){
    //float coeff[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetFrictionValue_ceiling",++_cmd_counter,para));
}

std::string robot_command_thread::SetFrictionValue_freedom(std::string para){
    //float coeff[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetFrictionValue_freedom",++_cmd_counter,para));
}

//外设控制
std::string robot_command_thread::ActGripper(std::string para){
    //int index, uint8_t act
    return _send_data_factory_callback(FRAPI_base::command_factry("ActGripper",++_cmd_counter,para));
}

std::string robot_command_thread::MoveGripper(std::string para){
    //int index, int pos, int vel, int force, int max_time, uint8_t block
    std::string vel,force,mtime,block;
    vel = this->get_parameter("gripper_vel").value_to_string();
    force = this->get_parameter("gripper_force").value_to_string();
    mtime  = this->get_parameter("gripper_maxtime").value_to_string();
    block = this->get_parameter("gripper_block").value_to_string();
    para = para + "," + vel + "," + force + "," + mtime + "," + block; 
    return _send_data_factory_callback(FRAPI_base::command_factry("MoveGripper",++_cmd_counter,para));
}
    
//IO控制
std::string robot_command_thread::SetDO(std::string para){
    //int id, uint8_t status, uint8_t smooth, uint8_t block
    std::string smooth,block;
    smooth = this->get_parameter("DO_smooth").value_to_string();
    block = this->get_parameter("DO_block").value_to_string();
    para = para + "," + smooth + "," + block;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetDO",++_cmd_counter,para));
}

std::string robot_command_thread::SetToolDO(std::string para){
    //int id, uint8_t status, uint8_t smooth, uint8_t block
    std::string smooth,block;
    smooth = this->get_parameter("DO_smooth").value_to_string();
    block = this->get_parameter("DO_block").value_to_string();
    para = para + "," + smooth + "," + block;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetToolDO",++_cmd_counter,para));
}

std::string robot_command_thread::SetAO(std::string para){
    //int id, float value, uint8_t block
    std::string block;
    block = this->get_parameter("AO_block").value_to_string();
    std::regex pattern("(\\d),([.|\\d]*)");
    std::smatch val_match;
    std::regex_match(para,val_match,pattern);
    std::string num = std::to_string(atof(val_match[2].str().c_str())/100*4095);
    para = val_match[1].str() + "," + num + "," + block;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetAO",++_cmd_counter,para));
}

std::string robot_command_thread::SetToolAO(std::string para){
    //int id, float value, uint8_t block
    std::string block;
    block = this->get_parameter("AO_block").value_to_string();
    std::regex pattern("(\\d),([.|\\d]*)");
    std::smatch val_match;
    std::regex_match(para,val_match,pattern);
    std::string num = std::to_string(atof(val_match[2].str().c_str())/100*4095);
    para = val_match[1].str() + "," + num + "," + block;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetToolAO",++_cmd_counter,para));
}

std::string robot_command_thread::SetAuxDO(std::string para){
    //int number, int switch
    para = para + "," + "1";
    return _send_data_factory_callback(FRAPI_base::command_factry("SetAuxDO",++_cmd_counter,para));
}

std::string robot_command_thread::SetAuxAO(std::string para){
    //int number int percentage
    std::string block;
    block = this->get_parameter("AO_block").value_to_string();
    std::regex pattern("(\\d),([.|\\d]*)");
    std::smatch val_match;
    std::regex_match(para,val_match,pattern);
    std::string num = std::to_string(atof(val_match[2].str().c_str())/100*4095);
    para = val_match[1].str() + "," + num + "," + block;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetAuxAO",++_cmd_counter,para));
}

//外部轴功能
std::string robot_command_thread::ExtAxisLoadModbusTCPDriver(std::string para){//加载外部轴
    //??
    return _send_data_factory_callback(FRAPI_base::command_factry("ExtAxisLoadModbusTCPDriver",++_cmd_counter,para));
}

std::string robot_command_thread::ExtAxisServoOn(std::string para){//外部轴使能
    //int number, int switch
    return _send_data_factory_callback(FRAPI_base::command_factry("ExtAxisServoOn",++_cmd_counter,para));
}

std::string robot_command_thread::ExtAxisStartJog(std::string para){//外部轴点动
    //6, int number, int direction, int speed, int acc_speed, double max_distance
    return _send_data_factory_callback(FRAPI_base::command_factry("ExtAxisStartJog",++_cmd_counter,para));
}

std::string robot_command_thread::ExtAxisSetHoming(std::string para){//外部轴回零
    //int number, int zero_mode, int search_speed, int latch_speed
    return _send_data_factory_callback(FRAPI_base::command_factry("ExtAxisSetHoming", ++_cmd_counter,para));
}

std::string robot_command_thread::StopExtAxisJog(std::string para){
    //empty
    return _send_data_factory_callback(FRAPI_base::command_factry("StopExtAxisJog", ++_cmd_counter,para));
}


//运动指令
std::string robot_command_thread::StartJOG(std::string para){
    //uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis
    std::string acc,max_dis;
    //acc = this->get_parameter("JOG_acc").value_to_string();
    //max_dis = this->get_parameter("JOG_maxdis").value_to_string();
    //para = para + "," + acc + "," + max_dis;
    return _send_data_factory_callback(FRAPI_base::command_factry("StartJOG",++_cmd_counter,para));
}

std::string robot_command_thread::StopJOG(std::string para){
    //uint8_t ref
    return _send_data_factory_callback(FRAPI_base::command_factry("StopJOG",++_cmd_counter,para));
}

std::string robot_command_thread::StopLine(std::string para){
    //empty
    return _send_data_factory_callback(FRAPI_base::command_factry("STOPLINE",++_cmd_counter,para));
}

std::string robot_command_thread::StopTool(std::string para){
    //empty
    return _send_data_factory_callback(FRAPI_base::command_factry("STOPTOOL",++_cmd_counter,para));
}

std::string robot_command_thread::ImmStopJOG(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("ImmStopJOG",++_cmd_counter,para));
}

std::string robot_command_thread::MoveJ(std::string para){
    //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos
    std::string speed,tool,user,acc,ovl,eaxis1,eaxis2,eaxis3,eaxis4,blendT,offset_flag,offset_pos_x,\
                offset_pos_y,offset_pos_z,offset_pos_rx,offset_pos_ry,offset_pos_rz;

    acc = this->get_parameter("MoveJLC_acc").value_to_string();
    ovl = this->get_parameter("MoveJLC_ovl").value_to_string();
    eaxis1 = this->get_parameter("MoveJLC_eaxis1").value_to_string();
    eaxis2 = this->get_parameter("MoveJLC_eaxis2").value_to_string();
    eaxis3 = this->get_parameter("MoveJLC_eaxis3").value_to_string();
    eaxis4 = this->get_parameter("MoveJLC_eaxis4").value_to_string();
    blendT = this->get_parameter("MoveJ_blendT").value_to_string();
    offset_flag = this->get_parameter("MoveJLC_offset_flag").value_to_string();
    offset_pos_x = this->get_parameter("MoveJLC_offset_pos_x").value_to_string();
    offset_pos_y = this->get_parameter("MoveJLC_offset_pos_y").value_to_string();
    offset_pos_z = this->get_parameter("MoveJLC_offset_pos_z").value_to_string();
    offset_pos_rx = this->get_parameter("MoveJLC_offset_pos_rx").value_to_string();
    offset_pos_ry = this->get_parameter("MoveJLC_offset_pos_ry").value_to_string();
    offset_pos_rz = this->get_parameter("MoveJLC_offset_pos_rz").value_to_string();
    std::cout<<para<<std::endl;

    std::regex search_para(",");
    std::regex_token_iterator<std::string::iterator> iter_data(para.begin(),para.end(),search_para,-1);
    std::regex_token_iterator<std::string::iterator> end;
    std::smatch num_match;
    std::string head_str = iter_data->str();
    ++iter_data;
    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)"))){//第一个元素是否满足JNT1这种模式
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error :MoveJ input position point number exceeded the limit.");
            // std::cout << "指令错误:MoveJ输入位置点序号超限" << std::endl;
            return 0;
        }
        JointPos tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        if (iter_data != end)
        { 
            speed = iter_data->str();
            ++iter_data;

            // Check if there are more elements in the iterator
            if (iter_data != end) 
            {
                tool = iter_data->str();
                ++iter_data;

                // Check if there are more elements in the iterator
                if (iter_data != end) {
                    user = iter_data->str();
                }
            }
        }
        std::string para2 = "";
        for(int i=0;i<6;i++){
            para2 += std::to_string(tmp_jnt_pos.jPos[i]);
            if(i<5){
                para2 += ",";
            }
        }
        //std::cout << "send kin req data: " << para2; 
        // std::string res = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//求解正向运动学
        std::string res = GetForwardKin(para2);//求解正向运动学
        if(res == "-2001"){
            para.clear();
            for(int i=0;i<6;i++){
                para += std::to_string(tmp_jnt_pos.jPos[i]) + ","; 
            }
            for(int i=0;i<6;i++){
                para += std::to_string(_kin_res[i]) + ",";
            }
            para = para + tool + "," + user + "," + speed + "," + acc  + "," + ovl + "," + eaxis1 + ","\
                   + eaxis2 + "," + eaxis3 + "," + eaxis4 + "," + blendT + "," + offset_flag + ","\
                   + offset_pos_x + "," + offset_pos_y + "," + offset_pos_z + "," + offset_pos_rx + ","\
                   + offset_pos_ry + "," + offset_pos_rz;
            // std::string tmp_para = FRAPI_base::command_factry("MoveJ",1,para);
            // std::cout << "MoveJ发送数据: " << tmp_para << std::endl;
            return _send_data_factory_callback(FRAPI_base::command_factry("MoveJ",++_cmd_counter,para));
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: An error occurred in the forward kinematics of the MoveJ instruction call.");
            // std::cout << "指令错误:MoveJ指令调用正向运动学发生错误" << std::endl;
            return std::to_string(0);
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error :MoveJ input position point number exceeded the limit.");
            // std::cout << "指令错误:MoveJ输入位置点序号超限" << std::endl;
            return 0;
        }
        DescPose tmp_cart_pos = _cmd_cart_pos_list.at(index-1);
        // iter_data++;
        if (iter_data != end)
        { 
            speed = iter_data->str();
            ++iter_data;

            // Check if there are more elements in the iterator
            if (iter_data != end) 
            {
                tool = iter_data->str();
                ++iter_data;

                // Check if there are more elements in the iterator
                if (iter_data != end) {
                    user = iter_data->str();
                }
            }
        }
        // std::string speed = iter_data->str(); 
        std::string para2 = "0,";
        para2 = para2 + std::to_string(tmp_cart_pos.tran.x) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.tran.y) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.tran.z) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.rpy.rx) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.rpy.ry) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.rpy.rz) + ",";
        para2 += "-1";
        //std::cout << "send kin req data: " << para2; 
        // std::string res = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//求解逆向运动学
        std::string res = GetInverseKin(para2);//求解逆向运动学
        if(res == "-2001"){
            para.clear();
            for(int i=0;i<6;i++){
                para += std::to_string(_kin_res[i]) + ","; 
            }
            para = para + std::to_string(tmp_cart_pos.tran.x) + ",";
            para = para + std::to_string(tmp_cart_pos.tran.y) + ",";
            para = para + std::to_string(tmp_cart_pos.tran.z) + ",";
            para = para + std::to_string(tmp_cart_pos.rpy.rx) + ",";
            para = para + std::to_string(tmp_cart_pos.rpy.ry) + ",";
            para = para + std::to_string(tmp_cart_pos.rpy.rz) + ",";
            para = para + tool + "," + user + "," + speed + "," + acc  + "," + ovl + "," + eaxis1 + ","\
                   + eaxis2 + "," + eaxis3 + "," + eaxis4 + "," + blendT + "," + offset_flag + ","\
                   + offset_pos_x + "," + offset_pos_y + "," + offset_pos_z + "," + offset_pos_rx + ","\
                   + offset_pos_ry + "," + offset_pos_rz;
            //std::cout << "send movej req data: " << para;   
            return _send_data_factory_callback(FRAPI_base::command_factry("MoveJ",++_cmd_counter,para));
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: An error occurred in the reverse kinematics of the MoveJ instruction call.");
            // std::cout << "指令错误:MoveJ指令调用逆向运动学发生错误" << std::endl;
            return std::to_string(0);
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: Invalid MoveJ parameter input, no point position information found.");
        // std::cout << "指令错误:MoveJ参数输入非法,没有找到点位信息" << std::endl;
        return std::to_string(0);
    }

}
    
std::string robot_command_thread::MoveL(std::string para){
    //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos
  std::string tool,user,acc,ovl,eaxis1,eaxis2,eaxis3,eaxis4,blendR,search,offset_flag,offset_pos_x,\
                offset_pos_y,offset_pos_z,offset_pos_rx,offset_pos_ry,offset_pos_rz;
    tool = this->get_parameter("MoveJLC_tool").value_to_string();
    user = this->get_parameter("MoveJLC_user").value_to_string();
    acc = this->get_parameter("MoveJLC_acc").value_to_string();
    ovl = this->get_parameter("MoveJLC_ovl").value_to_string();
    eaxis1 = this->get_parameter("MoveJLC_eaxis1").value_to_string();
    eaxis2 = this->get_parameter("MoveJLC_eaxis2").value_to_string();
    eaxis3 = this->get_parameter("MoveJLC_eaxis3").value_to_string();
    eaxis4 = this->get_parameter("MoveJLC_eaxis4").value_to_string();
    blendR = this->get_parameter("MoveL_blendR").value_to_string();
    search = this->get_parameter("MoveL_search").value_to_string();
    offset_flag = this->get_parameter("MoveJLC_offset_flag").value_to_string();
    offset_pos_x = this->get_parameter("MoveJLC_offset_pos_x").value_to_string();
    offset_pos_y = this->get_parameter("MoveJLC_offset_pos_y").value_to_string();
    offset_pos_z = this->get_parameter("MoveJLC_offset_pos_z").value_to_string();
    offset_pos_rx = this->get_parameter("MoveJLC_offset_pos_rx").value_to_string();
    offset_pos_ry = this->get_parameter("MoveJLC_offset_pos_ry").value_to_string();
    offset_pos_rz = this->get_parameter("MoveJLC_offset_pos_rz").value_to_string();

    std::regex search_para(",");
    std::regex_token_iterator<std::string::iterator> iter_data(para.begin(),para.end(),search_para,-1);
    std::smatch num_match;
    std::string head_str = iter_data->str();
    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)"))){   
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error :MoveL input position number exceeded the limit.");
            // std::cout << "指令错误:MoveL输入位置点序号超限" << std::endl;
            return 0;
        }
        JointPos tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        iter_data++;
        std::string speed = iter_data->str();
        std::string para2 = "";
        for(int i=0;i<6;i++){
                para2 += std::to_string(tmp_jnt_pos.jPos[i]);
            if(i<5){
                para2 += ",";
            }
        }
        // std::string res = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//求解正向运动学
        std::string res = GetForwardKin(para2);//求解正向运动学
        if(res == "-2001"){
            para.clear();
            for(int i=0;i<6;i++){
                para += std::to_string(tmp_jnt_pos.jPos[i]) + ","; 
            }
            for(int i=0;i<6;i++){
                para += std::to_string(_kin_res[i]) + ",";
            }
            para = para + tool + "," + user + "," + speed + "," + acc  + "," + ovl + "," + blendR + "," \
                   + eaxis1 + "," + eaxis2 + "," + eaxis3 + "," + eaxis4 + "," + search + "," + offset_flag + ","\
                   + offset_pos_x + "," + offset_pos_y + "," + offset_pos_z + "," + offset_pos_rx + ","\
                   + offset_pos_ry + "," + offset_pos_rz;
            return _send_data_factory_callback(FRAPI_base::command_factry("MoveL",++_cmd_counter,para));
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: An error occurred in the MoveL instruction call forward kinematics.");
            // std::cout << "指令错误:MoveL指令调用正向运动学发生错误" << std::endl;
            return std::to_string(0);
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error :MoveL input position number exceeded the limit.");
            // std::cout << "指令错误:MoveL输入位置点序号超限" << std::endl;
            return std::to_string(0);
        }
        DescPose tmp_cart_pos = _cmd_cart_pos_list.at(index-1);
        iter_data++;
        std::string speed = iter_data->str();
        std::string para2 = "0,";
        para2 = para2 + std::to_string(tmp_cart_pos.tran.x) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.tran.y) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.tran.z) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.rpy.rx) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.rpy.ry) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.rpy.rz) + ",";
        para2 += "-1";
        // std::string res = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//求解逆向运动学
        std::string res = GetInverseKin(para2);//求解逆向运动学
        if(res == "-2001"){
            para.clear();
            for(int i=0;i<6;i++){
                para += std::to_string(_kin_res[i]) + ","; 
            }
            para = para + std::to_string(tmp_cart_pos.tran.x) + ",";
            para = para + std::to_string(tmp_cart_pos.tran.y) + ",";
            para = para + std::to_string(tmp_cart_pos.tran.z) + ",";
            para = para + std::to_string(tmp_cart_pos.rpy.rx) + ",";
            para = para + std::to_string(tmp_cart_pos.rpy.ry) + ",";
            para = para + std::to_string(tmp_cart_pos.rpy.rz) + ",";
            para = para + tool + "," + user + "," + speed + "," + acc  + "," + ovl + "," + blendR + ","\
                   + eaxis1 + "," + eaxis2 + "," + eaxis3 + "," + eaxis4 + "," + search + "," + offset_flag + ","\
                   + offset_pos_x + "," + offset_pos_y + "," + offset_pos_z + "," + offset_pos_rx + ","\
                   + offset_pos_ry + "," + offset_pos_rz;
            return _send_data_factory_callback(FRAPI_base::command_factry("MoveL",++_cmd_counter,para));
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: An error occurred when the MoveL instruction called reverse kinematics.");
            // std::cout << "指令错误:MoveL指令调用逆向运动学发生错误" << std::endl;
            return std::to_string(0);
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: Invalid MoveL parameter input.");
        // std::cout << "指令错误:MoveL参数输入非法" << std::endl;
        return std::to_string(0);
    }

}

std::string robot_command_thread::MoveC(std::string para){
    //JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p,JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t,float ovl, float blendR
    std::string tool,user,acc,ovl,eaxis1,eaxis2,eaxis3,eaxis4,blendR,offset_flag,offset_pos_x,\
                offset_pos_y,offset_pos_z,offset_pos_rx,offset_pos_ry,offset_pos_rz;
    tool = this->get_parameter("MoveJLC_tool").value_to_string();
    user = this->get_parameter("MoveJLC_user").value_to_string();
    acc = this->get_parameter("MoveJLC_acc").value_to_string();
    ovl = this->get_parameter("MoveJLC_ovl").value_to_string();
    eaxis1 = this->get_parameter("MoveJLC_eaxis1").value_to_string();
    eaxis2 = this->get_parameter("MoveJLC_eaxis2").value_to_string();
    eaxis3 = this->get_parameter("MoveJLC_eaxis3").value_to_string();
    eaxis4 = this->get_parameter("MoveJLC_eaxis4").value_to_string();
    blendR = this->get_parameter("MoveC_blendR").value_to_string();
    offset_flag = this->get_parameter("MoveJLC_offset_flag").value_to_string();
    offset_pos_x = this->get_parameter("MoveJLC_offset_pos_x").value_to_string();
    offset_pos_y = this->get_parameter("MoveJLC_offset_pos_y").value_to_string();
    offset_pos_z = this->get_parameter("MoveJLC_offset_pos_z").value_to_string();
    offset_pos_rx = this->get_parameter("MoveJLC_offset_pos_rx").value_to_string();
    offset_pos_ry = this->get_parameter("MoveJLC_offset_pos_ry").value_to_string();
    offset_pos_rz = this->get_parameter("MoveJLC_offset_pos_rz").value_to_string();

    std::regex search_para(",");
    std::regex_token_iterator<std::string::iterator> iter_data(para.begin(),para.end(),search_para,-1);
    std::smatch num_match,num_match2;
    std::string head_str = iter_data->str();//第一个点位
    iter_data++;
    std::string second_str = iter_data->str();//第二个点位
    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)")) && std::regex_match(second_str,num_match2,std::regex("(JNT)([0-9]*)"))){   
        int index = atol(num_match[2].str().c_str());
        int index2 = atol(num_match2[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size() || index2 > _cmd_jnt_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error :MoveC input position point sequence number exceeded limit.");
            // std::cout << "指令错误:MoveC输入位置点序号超限" << std::endl;
            return 0;
        }
        JointPos tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        JointPos tmp_jnt_pos2 = _cmd_jnt_pos_list.at(index2-1);
        iter_data++;
        std::string speed = iter_data->str(); 
        std::string para2 = "";
        for(int i=0;i<6;i++){
            para2 += std::to_string(tmp_jnt_pos.jPos[i]);
            if(i<5){
                para2 += ",";
            }
        }
        // std::string res1 = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//求解正向运动学
        std::string res1 = GetForwardKin(para2);//求解正向运动学
        float _kin_res_1[6];
        for(int i=0;i<6;i++){
            _kin_res_1[i] = _kin_res[i];
        }
        para2.clear();
        for(int i=0;i<6;i++){
            para2 += std::to_string(tmp_jnt_pos2.jPos[i]);
            if(i<5){
                para2 += ",";
            }
        }
        // std::string res2 = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//求解正向运动学
        std::string res2 = GetForwardKin(para2);//求解正向运动学
        if(res1 == "-2001" && res2 == "-2001"){
            para.clear();
            para2.clear();
            for(int i=0;i<6;i++){
                para += std::to_string(tmp_jnt_pos.jPos[i]) + ","; 
                para2 += std::to_string(tmp_jnt_pos2.jPos[i]) + ",";
            }
            for(int i=0;i<6;i++){
                para += std::to_string(_kin_res_1[i]) + ",";
                para2 += std::to_string(_kin_res[i]) + ",";
            }
            para = para + tool + "," + user + "," + speed + "," + acc  + "," + eaxis1 + "," \
                   + eaxis2 + "," + eaxis3 + "," + eaxis4 + "," + offset_flag + "," + offset_pos_x + "," \
                   + offset_pos_y + "," + offset_pos_z + "," + offset_pos_rx + "," + offset_pos_ry + "," + offset_pos_rz + ","\
                   + para2 + tool + "," + user + "," + speed + "," + acc  + "," + eaxis1 + "," \
                   + eaxis2 + "," + eaxis3 + "," + eaxis4 + "," + offset_flag + "," + offset_pos_x + "," \
                   + offset_pos_y + "," + offset_pos_z + "," + offset_pos_rx + "," + offset_pos_ry + "," + offset_pos_rz + ","\
                   + ovl + "," + blendR ;
            return _send_data_factory_callback(FRAPI_base::command_factry("MoveC",++_cmd_counter,para));
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: An error occurred in the MoveC instruction call forward kinematics.");
            // std::cout << "指令错误:MoveC指令调用正向运动学发生错误" << std::endl;
            return std::to_string(0);
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)")) && std::regex_match(second_str,num_match2,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        int index2 = atol(num_match2[2].str().c_str());
        if(index > _cmd_cart_pos_list.size() || index2 > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error :MoveC input position point sequence number exceeded limit.");
            // std::cout << "指令错误:MoveC输入位置点序号超限" << std::endl;
            return std::to_string(0);
        }
        DescPose tmp_cart_pos = _cmd_cart_pos_list.at(index-1);
        DescPose tmp_cart_pos2 = _cmd_cart_pos_list.at(index2-1);
        iter_data++;
        std::string speed = iter_data->str();
        std::string para2 = "0,";
        para2 += std::to_string(tmp_cart_pos.tran.x) + ",";
        para2 += std::to_string(tmp_cart_pos.tran.y) + ",";
        para2 += std::to_string(tmp_cart_pos.tran.z) + ",";
        para2 += std::to_string(tmp_cart_pos.rpy.rx) + ",";
        para2 += std::to_string(tmp_cart_pos.rpy.ry) + ",";
        para2 += std::to_string(tmp_cart_pos.rpy.rz) + ",";
        para2 += "-1";
        // std::string res1 = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//求解逆向运动学
        std::string res1 = GetInverseKin(para2);//求解逆向运动学
        float _kin_res_1[6];
        for(int i=0;i<6;i++){
            _kin_res_1[i] = _kin_res[i];
        }
        para2.clear();
        para2 = "0,";
        para2 += std::to_string(tmp_cart_pos2.tran.x) + ",";
        para2 += std::to_string(tmp_cart_pos2.tran.y) + ",";
        para2 += std::to_string(tmp_cart_pos2.tran.z) + ",";
        para2 += std::to_string(tmp_cart_pos2.rpy.rx) + ",";
        para2 += std::to_string(tmp_cart_pos2.rpy.ry) + ",";
        para2 += std::to_string(tmp_cart_pos2.rpy.rz) + ",";
        para2 += "-1";
        // std::string res2 = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//求解逆向运动学
        std::string res2 = GetInverseKin(para2);//求解逆向运动学
        if(res1 == "-2001" && res2 == "-2001"){
            para.clear();
            para2.clear();
            for(int i=0;i<6;i++){
                para += std::to_string(_kin_res_1[i]) + ","; 
                para2 += std::to_string(_kin_res[i]) + ",";
            }
            para += std::to_string(tmp_cart_pos.tran.x) + ",";
            para += std::to_string(tmp_cart_pos.tran.y) + ",";
            para += std::to_string(tmp_cart_pos.tran.z) + ",";
            para += std::to_string(tmp_cart_pos.rpy.rx) + ",";
            para += std::to_string(tmp_cart_pos.rpy.ry) + ",";
            para += std::to_string(tmp_cart_pos.rpy.rz) + ",";
            para2 += std::to_string(tmp_cart_pos2.tran.x) + ",";
            para2 += std::to_string(tmp_cart_pos2.tran.y) + ",";
            para2 += std::to_string(tmp_cart_pos2.tran.z) + ",";
            para2 += std::to_string(tmp_cart_pos2.rpy.rx) + ",";
            para2 += std::to_string(tmp_cart_pos2.rpy.ry) + ",";
            para2 += std::to_string(tmp_cart_pos2.rpy.rz) + ",";
            para = para + tool + "," + user + "," + speed + "," + acc  + "," + eaxis1 + "," \
                   + eaxis2 + "," + eaxis3 + "," + eaxis4 + "," + offset_flag + "," + offset_pos_x + "," \
                   + offset_pos_y + "," + offset_pos_z + "," + offset_pos_rx + "," + offset_pos_ry + "," + offset_pos_rz + ","\
                   + para2 + tool + "," + user + "," + speed + "," + acc  + "," + eaxis1 + "," \
                   + eaxis2 + "," + eaxis3 + "," + eaxis4 + "," + offset_flag + "," + offset_pos_x + "," \
                   + offset_pos_y + "," + offset_pos_z + "," + offset_pos_rx + "," + offset_pos_ry + "," + offset_pos_rz + ","\
                   + ovl + "," + blendR ;
            return _send_data_factory_callback(FRAPI_base::command_factry("MoveC",++_cmd_counter,para));
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: An error occurred when the MoveC instruction called the inverse kinematics.");
            // std::cout << "指令错误:MoveC指令调用逆向运动学发生错误" << std::endl;
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: Invalid MoveC parameter input.");
        // std::cout << "指令错误:MoveC参数输入非法" << std::endl;
        return std::to_string(0);
    }
}

std::string robot_command_thread::Circle(std::string para){
    //JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, float ovl, uint8_t offset_flag, DescPose *offset_pos
    return std::to_string(-1);
}

// int robot_command_thread::NewSpiral(std::string para){
//     //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, ExaxisPos *epos, float ovl, uint8_t offset_flag, DescPose *offset_pos, SpiralParam spiral_param
//     return -1;
// }


std::string robot_command_thread::SplineStart(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("SplineStart",++_cmd_counter,para));
}

std::string robot_command_thread::SplinePTP(std::string para){
    //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl
    //默认进来的都是JNT数据
    std::regex search_para(",");
    std::regex_token_iterator<std::string::iterator> iter_data(para.begin(),para.end(),search_para,-1);
    std::smatch num_match;
    std::string head_str = iter_data->str();
    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)"))){ 
        std::string tool,user,acc,ovl;
        tool = this->get_parameter("Spline_tool").value_to_string();
        user = this->get_parameter("Spline_user").value_to_string();
        acc = this->get_parameter("Spline_acc").value_to_string();
        ovl = this->get_parameter("Spline_ovl").value_to_string();
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error :SplinePTP input position number exceeded the limit.");
            // std::cout << "指令错误:SplinePTP输入位置点序号超限" << std::endl;
            return 0;
        }
        JointPos tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        iter_data++;
        std::string speed = iter_data->str();
        std::string para2 = "";
        for(int i=0;i<6;i++){
            para2 += std::to_string(tmp_jnt_pos.jPos[i]);
            if(i<5){
                para2 += ",";
            }
        }
        //std::cout << "SplinePTP数据: " << FRAPI_base::command_factry("GetForwardKin",1,para2) << std::endl;
        // std::string res = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//求解正向运动学
        std::string res = GetForwardKin(para2);//求解正向运动学
        if(res == "-2001"){
            para.clear();
            for(int i=0;i<6;i++){
                para += std::to_string(tmp_jnt_pos.jPos[i]) + ","; 
            }
            for(int i=0;i<6;i++){
                para += std::to_string(_kin_res[i]) + ",";
            }
            para = para + tool + "," + user + "," + speed + "," + acc  + "," + ovl;
            //std::cout << FRAPI_base::command_factry("SplinePTP",1,para) << std::endl;
            return _send_data_factory_callback(FRAPI_base::command_factry("SplinePTP",++_cmd_counter,para));
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: An error occurred in the Spline instruction calling forward kinematics.");
            // std::cout << "指令错误:Spline指令调用正向运动学发生错误" << std::endl;
            return 0;
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error :Spline parameter input is invalid.");
        // std::cout << "指令错误:Spline参数输入非法" << std::endl;
        return 0;
    }
}

std::string robot_command_thread::SplineEnd(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("SplineEnd",++_cmd_counter,para));
}

std::string robot_command_thread::NewSplineStart(std::string para){
    //uint8_t ctlPoint
    return _send_data_factory_callback(FRAPI_base::command_factry("NewSplineStart",++_cmd_counter,para));
}

std::string robot_command_thread::NewSplinePoint(std::string para){
    //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl float blendR uint8_t lastFlag
    //输入参数:pos,speed,lastflag
    std::regex search_para(",");
    std::regex_token_iterator<std::string::iterator> iter_data(para.begin(),para.end(),search_para,-1);
    std::smatch num_match;
    std::string head_str = iter_data->str();
    if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        std::string tool,user,acc,ovl,blendR;
        tool = this->get_parameter("Spline_tool").value_to_string();
        user = this->get_parameter("Spline_user").value_to_string();
        acc = this->get_parameter("Spline_acc").value_to_string();
        ovl = this->get_parameter("Spline_ovl").value_to_string();
        blendR = this->get_parameter("NewSpline_blendR").value_to_string();
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error :NewSplinePoint input position point number exceeded the limit.");
            // std::cout << "指令错误:NewSplinePoint输入位置点序号超限" << std::endl;
            return 0;
        }
        //std::cout << "cart index is: " << index_str << std::endl;
        DescPose tmp_cart_pos = _cmd_cart_pos_list.at(index-1);
        iter_data++;
        std::string speed = iter_data->str();
        //std::cout << "cart speed is: " << speed << std::endl;
        iter_data++;
        std::string lastflag = iter_data->str();
        //std::cout << "cart last flag is: " << lastflag << std::endl;
        std::string para2 = "0,";
        para2 = para2 + std::to_string(tmp_cart_pos.tran.x) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.tran.y) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.tran.z) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.rpy.rx) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.rpy.ry) + ",";
        para2 = para2 + std::to_string(tmp_cart_pos.rpy.rz) + ",";
        para2 += "-1";
        // std::string res = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//求解逆向运动学
        std::string res = GetInverseKin(para2);//求解逆向运动学
        if(res == "-2001"){
            para.clear();
            for(int i=0;i<6;i++){
                para += std::to_string(_kin_res[i]) + ","; 
            }
            para = para + std::to_string(tmp_cart_pos.tran.x) + ",";
            para = para + std::to_string(tmp_cart_pos.tran.y) + ",";
            para = para + std::to_string(tmp_cart_pos.tran.z) + ",";
            para = para + std::to_string(tmp_cart_pos.rpy.rx) + ",";
            para = para + std::to_string(tmp_cart_pos.rpy.ry) + ",";
            para = para + std::to_string(tmp_cart_pos.rpy.rz) + ",";
            para = para + tool + "," + user + "," + speed + "," + acc + "," + ovl + "," + blendR + "," + lastflag;
            return _send_data_factory_callback(FRAPI_base::command_factry("NewSplinePoint",++_cmd_counter,para));
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: An error occurred when the NewSplinePoint instruction invoked the reverse kinematics.");
            // std::cout << "指令错误:NewSplinePoint指令调用逆向运动学发生错误" << std::endl;
            return std::to_string(0);
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction error: Invalid NewSplinePoint parameter input.");
        // std::cout << "指令错误:NewSplinePoint参数输入非法" << std::endl;
        return std::to_string(0);
    }
}

std::string robot_command_thread::NewSplineEnd(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("NewSplineEnd",++_cmd_counter,para));
}

std::string robot_command_thread::StopMotion(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("StopMotion",++_cmd_counter,para));
}

std::string robot_command_thread::PointsOffsetEnable(std::string para){
    //int flag, DescPose *offset_pos
    return _send_data_factory_callback(FRAPI_base::command_factry("PointsOffsetEnable",++_cmd_counter,para));
}

std::string robot_command_thread::PointsOffsetDisable(std::string para){
    //empty para
    return _send_data_factory_callback(FRAPI_base::command_factry("PointsOffsetDisable",++_cmd_counter,para));
}

std::string robot_command_thread::AuxServoSetParam(std::string para)
{
    return _send_data_factory_callback(FRAPI_base::command_factry("AuxServoSetParam",++_cmd_counter,para));
}
// 扩展轴使能 0=去使能 1-使能
std::string robot_command_thread::AuxServoEnable(std::string para)
{
    return _send_data_factory_callback(FRAPI_base::command_factry("AuxServoEnable",++_cmd_counter,para));
}
//初始设置，控制模式，默认0（位置模式）
std::string robot_command_thread::AuxServoSetControlMode(std::string para)
{
    return _send_data_factory_callback(FRAPI_base::command_factry("AuxServoSetControlMode",++_cmd_counter,para));
}
//使能前第二步：设置目标位置和速度
std::string robot_command_thread::AuxServoSetTargetPos(std::string para)
{
    return _send_data_factory_callback(FRAPI_base::command_factry("AuxServoSetTargetPos",++_cmd_counter,para));
}
std::string robot_command_thread::AuxServoSetTargetSpeed(std::string para)
{
    return _send_data_factory_callback(FRAPI_base::command_factry("AuxServoSetTargetSpeed",++_cmd_counter,para));
}
std::string robot_command_thread::AuxServoSetTargetTorque(std::string para)
{
    return _send_data_factory_callback(FRAPI_base::command_factry("AuxServoSetTargetTorque",++_cmd_counter,para));
}
//使能前第一步：设置回零方，速度。默认：0（当前位置回零）- 5 - 1
std::string robot_command_thread::AuxServoHoming(std::string para)
{
    return _send_data_factory_callback(FRAPI_base::command_factry("AuxServoHoming",++_cmd_counter,para));
}
std::string robot_command_thread::AuxServoClearError(std::string para)
{
    return _send_data_factory_callback(FRAPI_base::command_factry("AuxServoClearError",++_cmd_counter,para));
}
//初始设置，驱动器编号
std::string robot_command_thread::AuxServoSetStatusID(std::string para)
{
    return _send_data_factory_callback(FRAPI_base::command_factry("AuxServoSetStatusID",++_cmd_counter,para));
}

std::string robot_command_thread::ScriptStart(std::string para)
{
    return _send_data_factory_callback(FRAPI_base::command_factry("ScriptStart",++_cmd_counter,para));
}

std::string robot_command_thread::ScriptStart_return(std::string para)
{
    static char recv_buf[RECV_BUFF];
    memset(recv_buf,0,sizeof(recv_buf));
    if(_program_state == 2)
    {
        if(recv(_socketfd1,recv_buf,sizeof(recv_buf),0)>-1)//脚本运行指令如果执行成功只会返回一次信息即“1”，再recv只要recv到说明出现问题
        {
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Received instruction reply message...",std::string(recv_buf));
            // std::cout << "收到指令回复信息..." << std::string(recv_buf) << std::endl;
            _ParseRecvData(std::string(recv_buf));
            
            if(_ParseRecvData(std::string(recv_buf))!="0")
            {
                RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Instruction parsed successfully.");
                // std::cout<<"指令解析成功"<<std::endl;
               if(_recv_data_cmdcount == _cmd_counter)//帧计数器能对上,说明对应回复信息对应指令Start
                {
                    _start_return_data = _start_recv_res;

                    std::cout<<_start_return_data<<std::endl;
                } 
                else
                {
                    _start_return_data = "0";
                }
            }
        }
        else
        {
            _start_return_data = "1";
        }
        sleep(100);
    }

    if(recv(_socketfd1,recv_buf,sizeof(recv_buf),0)>-1)//脚本运行指令如果执行成功只会返回一次信息即“1”，再recv只要recv到说明出现问题，此处多recv一次
    {
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Received instruction reply message...%s",recv_buf);
        // std::cout << "收到指令回复信息..." << std::string(recv_buf) << std::endl;
        if(_ParseRecvData(std::string(recv_buf))!="0")
        {
            if(_recv_data_cmdcount == _cmd_counter)//帧计数器能对上,说明对应回复信息对应指令Start
            {
                _start_return_data = _start_recv_res;
                std::cout<<_start_return_data<<std::endl;
            } 
            else
            {
                _start_return_data = "0";
            }
        }
    }
    else
    {
        _start_return_data = "1";
    }
    return std::to_string(0);
}


std::string robot_command_thread::ScriptStop(std::string para){
    //empty
    return _send_data_factory_callback(FRAPI_base::command_factry("ScriptStop",++_cmd_counter,para));
}

std::string robot_command_thread::ScriptPause(std::string para){
    //empty
    return _send_data_factory_callback(FRAPI_base::command_factry("ScriptPause",++_cmd_counter,para));
}

std::string robot_command_thread::ScriptResume(std::string para){
    //empty
    return _send_data_factory_callback(FRAPI_base::command_factry("ScriptResume",++_cmd_counter,para));
}

//正逆向运动学
std::string robot_command_thread::GetForwardKin(std::string para){
    return _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para));
}

std::string robot_command_thread::GetInverseKin(std::string para){
    return _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para));
}

std::string robot_command_thread::GetTCPOffset(std::string para)
{
    // return _send_get_data_factory_callback(FRAPI_base::command_factry("GetTCPOffset",++_cmd_counter,para));
    std::cout << "GetTCPOffset" << std::endl;
    return _send_data_factory_callback(FRAPI_base::command_factry("GetTCPOffset",++_cmd_counter,para));
   
}

std::string robot_command_thread::GetDHCompensation(std::string para)
{
    // return _send_get_data_factory_callback(FRAPI_base::command_factry("GetDHCompensation",++_cmd_counter,para));
    std::cout << "GetDHCompensation" << std::endl;
    return _send_data_factory_callback(FRAPI_base::command_factry("GetDHCompensation",++_cmd_counter,para));
   
}

std::string robot_command_thread::GetTCPOffseta(std::string para)
{
    // return _send_get_data_factory_callback(FRAPI_base::command_factry("GetTCPOffseta",++_cmd_counter,para));
    std::cout << "GetTCPOffseta" << std::endl;
    return _send_data_factory_callback(FRAPI_base::command_factry("GetTCPOffseta",++_cmd_counter,para));
   
}

/**
 * @brief 
 * @param idle_time 空闲idle_time后，开始发射探针
 * @param interval_time 发射首个探针后，如果interval_time内没有响应，再次发射探针
 * @param probe_times 一共会发射probe_times次探针
 * @return -1-开启失败；0-成功
*/
int robot_command_thread::setKeepAlive(int fd, int idle_time, int interval_time, int probe_times)
{
    int val = 1;
	//开启keepalive机制
    if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val)) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"setsockopt SO_KEEPALIVE: %s", strerror(errno));
        return -1;
    }
 
    val = idle_time;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &val, sizeof(val)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"setsockopt TCP_KEEPIDLE: %s\n", strerror(errno));
        return -1;
    }
 
    val = interval_time;
    if (val == 0) val = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &val, sizeof(val)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"setsockopt TCP_KEEPINTVL: %s\n", strerror(errno));
        return -1;
    }
 
    val = probe_times;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &val, sizeof(val)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"setsockopt TCP_KEEPCNT: %s\n", strerror(errno));
        return -1;
    }
 
    return 0;
}


/******状态信息获取节点******/
/*************************/
robot_recv_thread::robot_recv_thread(const std::string node_name):rclcpp::Node(node_name)
{
    using namespace std::chrono_literals;
    _controller_ip = CONTROLLER_IP;//控制器默认ip地址
    RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Start creating a status feedback TCP socket.");
    // std::cout << "开始创建状态反馈TCP socket" << std::endl;

    //只保留8081端口的连接，8083连接传输的数据已经不用
    // _socketfd1 = socket(AF_INET,SOCK_STREAM,0);//状态获取端口只有TCP
    _socketfd2 = socket(AF_INET,SOCK_STREAM,0);//状态获取端口只有TCP

    if(_socketfd2 == -1)
    {
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Error: socket creation failed!");
        // std::cout << "错误: 创建socket失败!" << std::endl;
        exit(0);//创建套字失败,丢出错误
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Create status feedback socket successfully, start connecting controller...");
        // std::cout << "创建状态反馈socket成功,开始连接控制器..." << std::endl;
        struct sockaddr_in tcp_client1,tcp_client2;
        tcp_client1.sin_family = AF_INET;
        tcp_client1.sin_port = htons(port1);//8083端口
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());
        tcp_client2.sin_family = AF_INET;
        tcp_client2.sin_port = htons(port2);//8081端口
        tcp_client2.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

        //尝试连接控制器
        int res1 = 0;
        // int res1 = connect(_socketfd1,(struct sockaddr *)&tcp_client1,sizeof(tcp_client1));
        int res2 = connect(_socketfd2,(struct sockaddr *)&tcp_client2,sizeof(tcp_client2));
        if(0 != res2)
        {
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Error: Unable to connect to controller data port, program exit!");
            // std::cout << "错误:无法连接控制器数据端口,程序退出!" << std::endl;
            exit(0);//连接失败,丢出错误并返回
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"The controller status port is connected successfully.");
            // std::cout << "控制器状态端口连接成功" << std::endl;
            //将socket设置成非阻塞模式
            // int flags1 = fcntl(_socketfd1,F_GETFL,0);
            // fcntl(_socketfd1,F_SETFL,flags1|SOCK_NONBLOCK);
            //std::cout << "数据反馈端口配置成功" << std::endl;
            int flags2 = fcntl(_socketfd2,F_GETFL,0);
            fcntl(_socketfd2,F_SETFL,flags2|SOCK_NONBLOCK);
            
            //开启keepalive
            if(0 != setKeepAlive(_socketfd2, 5, 3, 3))
            {
                RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"Failed to enable tcp keepalive probe.");
                // RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"开启tcp保活探针失败");
            }

            _state_publisher = this->create_publisher<robot_feedback_msg>(
                "nonrt_state_data",
                10
            );
            _locktimer = this->create_wall_timer(100ms,std::bind(&robot_recv_thread::_state_recv_callback,this));//创建一个定时器任务用于获取非实时状态数据,触发间隔为100ms
        }

        //连接成功，创建守护线程,如果该连接端掉，则自动发起重连接;生命周期随该节点
        _try_to_reconnect();

    }
}
void robot_recv_thread::_try_to_reconnect()
{
    std::cout << "check connect " << std::endl;
    auto _reconnect_func = [this]()
    {
        while ((1 != _robot_recv_exit))
        {
            do{
                std::lock_guard<std::mutex> _lock(_reconnect_mutex);
                /* try to re-connect 58.2 8081*/
                if (1 == _is_reconnect)
                {
                    // 关闭旧连接
                    shutdown(_socketfd2, SHUT_RDWR);
                    close(_socketfd2);
                    _socketfd2 = -1;
                    // std::this_thread::sleep_for(std::chrono::seconds(1));

                    int sock_fd = socket(AF_INET, SOCK_STREAM, 0);
                    if (-1 == sock_fd)
                    {
                        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect: Failed to create the socket, try again after 3s.");
                        // std::cout << "重连：创建套接字失败, 3s 后再次尝试" << std::endl;
                        break;
                    }
                    else
                    {
                        struct sockaddr_in tcp_client1;
                        tcp_client1.sin_family = AF_INET;
                        tcp_client1.sin_port = htons(port2);
                        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

                        // 尝试连接控制器
                        int res1 = connect(sock_fd, (struct sockaddr *)&tcp_client1, sizeof(tcp_client1));
                        if (res1)
                        {
                            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect: Attempt to reconnect failed, program exit! Try again after 3s.");
                            // std::cout << "重连：发起重新连接失败,程序退出! 3s 后再次尝试" << std::endl;
                            shutdown(sock_fd, SHUT_RDWR);
                            close(sock_fd);
                            break;
                        }
                        else
                        {
                            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect: Initiated reconnect successfully.");
                            // std::cout << "重连：发起重新连接连接成功" << std::endl;
                            // 设置TCP接收超时
                            int flags2 = fcntl(sock_fd, F_GETFL, 0);
                            fcntl(sock_fd, F_SETFL, flags2 | SOCK_NONBLOCK);

                            // 开启并设置keepalive
                            if (0 != setKeepAlive(sock_fd, 5, 3, 3))
                            {
                                RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"Failed to enable tcp keepalive probe.");
                                // RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"), "开启tcp保活探针失败");
                            }
                            // return sock_fd;
                            _socketfd2 = sock_fd;
                            _is_reconnect = 0;
                            break;
                        }
                    }
                }
            }while(0);
            /* 以3s的频率检查 */
            std::this_thread::sleep_for(std::chrono::seconds(3));
            // std::cout << "thread " << std::this_thread::get_id() << " check connect end" << std::endl;
        }
            
    };

    _reconnect_thread = std::thread(_reconnect_func);
}
robot_recv_thread::~robot_recv_thread(){
    //关闭并销毁socket
    if(_socketfd2 != -1){
        shutdown(_socketfd2,SHUT_RDWR);
        close(_socketfd2);
    }
    // if(_socketfd1 != -1){
    //     shutdown(_socketfd1,SHUT_RDWR);
    //     close(_socketfd1);
    // }

    _robot_recv_exit = 1;
    if(_reconnect_thread.joinable())
    {
        _reconnect_thread.join();
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"The reconnect thread exits.");
        // std::cout << "重连线程退出" <<std::endl;
    }
}

/**
 * @brief 
 * @param idle_time 空闲idle_time后，开始发射探针
 * @param interval_time 发射首个探针后，如果interval_time内没有响应，再次发射探针
 * @param probe_times 一共会发射probe_times次探针
 * @return -1-开启失败；0-成功
*/
int robot_recv_thread::setKeepAlive(int fd, int idle_time, int interval_time, int probe_times)
{
    int val = 1;
	//开启keepalive机制
    if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val)) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"setsockopt SO_KEEPALIVE: %s", strerror(errno));
        return -1;
    }
 
    val = idle_time;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &val, sizeof(val)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"setsockopt TCP_KEEPIDLE: %s\n", strerror(errno));
        return -1;
    }
 
    val = interval_time;
    if (val == 0) val = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &val, sizeof(val)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"setsockopt TCP_KEEPINTVL: %s\n", strerror(errno));
        return -1;
    }
 
    val = probe_times;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &val, sizeof(val)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("FrHardwareInterface"),"setsockopt TCP_KEEPCNT: %s\n", strerror(errno));
        return -1;
    }
 
    return 0;
}

void robot_recv_thread::_state_recv_callback(){
    static FR_nonrt_state state_data;

    static std::queue<FR_nonrt_state> store_buff;//用于存储缓存区多余的数据，需要限制长度
    static char temp_buff[NONRT_PACKAGE_SIZE] = {0};//用于临时存储不完整帧的数据，之后用于数据拼接
    static int future_recv = NONRT_PACKAGE_SIZE;//接受到不完整帧数据后用于存储下一帧头部需要接收的剩余数据长度
    int datalen = NONRT_PACKAGE_SIZE;
    uint16_t* head_ptr;
    uint8_t* checksum_ptr;
    uint16_t checksum = 0;
    /* 
    //取8083端口数据
    while(datalen > 0){//如果缓存区还有数据，那么就一直循环取出
        char recv_buff[future_recv] = {0};
        datalen = recv(_socketfd1,recv_buff,sizeof(recv_buff),0);//取出固定长度的数据.
        datalen = -1;
        head_ptr = (uint16_t*)(recv_buff);//通过指针访问的方法取出frame_head
        checksum_ptr = (uint8_t*)(recv_buff);//和校验指针
        
        if(*head_ptr == 0x5A5A){//检测包头
        
            if(datalen == NONRT_PACKAGE_SIZE){//有时候缓冲区尾部数据不是一个完整的帧，因此需要保存不完整的信息用于下一帧数据拼接
                for(int i=0;i< datalen-2;i++){
			        checksum += *checksum_ptr; 
			        checksum_ptr++;           
                }
                if(checksum == *((uint16_t*)checksum_ptr)){//进行和校验
                    memcpy(&state_data,recv_buff,sizeof(recv_buff));
                    if(store_buff.size() < 10){
                        store_buff.emplace(state_data);//将数据放入队列中
                    }else{//如果队列中数据满10个，那么删掉队列头部数据然后队尾再插入数据
                        store_buff.pop();//弹出队首的元素
                        store_buff.emplace(state_data);
                    }
                    }else{//和校验不通过，丢出错误信息，不向队列中添加信息
                        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:状态数据和校验失败!");
                        state_data.check_sum = 0;    
                    }
                    checksum = 0;
            }else{//需要拼接的情况
                memset(temp_buff,0,sizeof(temp_buff));//清空临时存放变量
                memcpy(temp_buff,recv_buff,sizeof(recv_buff));//临时存放不完整数据
                for(int i=0;i< datalen;i++){
		            checksum += *checksum_ptr; 
		            checksum_ptr++;           
                }
                future_recv = NONRT_PACKAGE_SIZE - datalen;
                //std::cout << "获得不完整头部数据 " << "size: " << datalen  << std::endl;
            }
        }else{
            //std::cout << "未获得包头 " << "size: " << datalen  << std::endl;
            if(datalen == -1){
                int error_code = 0;
                socklen_t len1 = sizeof(error_code);
                getsockopt(_socketfd1,SOL_SOCKET,SO_ERROR,&error_code,&len1);
                //break;//已经没有数据了，退出循环
            }else{//进行数据拼接
                for(int i=0;i< datalen-2;i++){//进行和校验计算
			        checksum += *checksum_ptr; 
			        checksum_ptr++;           
                }
                if(checksum == *((uint16_t*)checksum_ptr)){//进行和校验
                    memcpy(&temp_buff[NONRT_PACKAGE_SIZE-future_recv],recv_buff,sizeof(recv_buff));
                    memcpy(&state_data,temp_buff,sizeof(temp_buff));
                    //需要检测check_sum来判断这段数据是否是上一帧拼接数据，通讯有干扰的情况下会导致数据通讯异常错过包头
                    //std::cout << "获得拼接数据 " <<  "size: " <<  datalen << std::endl;
                    if(store_buff.size() < 10){
                        store_buff.emplace(state_data);//将数据放入队列中
                    }else{//队列长度等于10的时候，弹出队首的元素，在队尾加入元素
                        store_buff.pop();
                        store_buff.emplace(state_data);
                    }
                }else{//和校验不通过
                    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:拼接状态数据和校验失败!");
                    state_data.check_sum = 0;    
                }    
                // std::cout << "包头信息: " << state_data.head << "," << state_data.check_sum << "," <<  (int)state_data.count <<std::endl;
                future_recv = NONRT_PACKAGE_SIZE;
            }
            checksum = 0;
        }
    }//end while*/

    static _CTRL_STATE ctrl_state;
    _CTRL_STATE state_data_store;
    static std::queue<_CTRL_STATE> ctrl_state_store_buff;//用于存储缓存区多余的数据，需要限制长度
    static char ctrl_state_temp_buff[_CTRL_STATE_SIZE] = {0};//用于临时存储不完整帧的数据，之后用于数据拼接
    static int ctrl_state_future_recv = _CTRL_STATE_SIZE ;//接受到不完整帧数据后用于存储下一帧头部需要接收的剩余数据长度
    int ctrl_state_datalen = _CTRL_STATE_SIZE ;
    uint32_t* ctrl_state_head_ptr;

    while(ctrl_state_datalen > 0){//如果缓存区还有数据，那么就一直循环取出     
        char recv_buff[ctrl_state_future_recv] = {0};
        ctrl_state_datalen = recv(_socketfd2,recv_buff,sizeof(recv_buff),0);//取出固定长度的数据   
         //std::cout<<"ctrl_state_datalen "<<ctrl_state_datalen<<std::endl;
         //std::cout<<"data_len_expected: "<<_CTRL_STATE_SIZE << std::endl;

        /* 如果处于重连流程，不需要再读取，直接返回 */
        {
            std::lock_guard<std::mutex> _lock(_reconnect_mutex);
            if(_is_reconnect == 1)
            {
                RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Reconnect, please wait.......");
                // std::cout << "重连中，请等待......" << std::endl;
                return ;
            }
        }
        // 这里是非阻塞读，开启探针后，当recv失败时，通过errno查看结果
        if ((ctrl_state_datalen == 0) || ((ctrl_state_datalen == -1) && (errno != EINTR )&& (errno != EWOULDBLOCK) && (errno != EAGAIN)))
        {
            {
                std::lock_guard<std::mutex> _lock(_reconnect_mutex);
                _is_reconnect = 1;
            }
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"The network may have been disconnected. Please check the network connection.......%s(error code: %d)", strerror(errno), errno);
            // std::cout << "网络可能已经断开，请检查网络连接 ......" << strerror(errno) << "  " << errno << std::endl;
            return ;
        }

        // std::cout << ctrl_state_datalen << " 开始取数据  " << _is_reconnect << " " << strerror(errno) << std::endl;
        ctrl_state_head_ptr = (uint32_t*)(recv_buff);//通过指针访问的方法取出frame_head
        if(*ctrl_state_head_ptr == 0x622F662F){//检测包头
            if(ctrl_state_datalen == _CTRL_STATE_SIZE)
            {//有时候缓冲区尾部数据不是一个完整的帧，因此需要保存不完整的信息用于下一帧数据拼接
                 uint8_t* data_len_ptr = (uint8_t*)(recv_buff);
                 data_len_ptr += 7;
                  //std::cout << "信息完整无需拼接,8081数据校验长度: " << *((int*)(data_len_ptr)) << std::endl;
                    memcpy(&ctrl_state, recv_buff, sizeof(ctrl_state));
                    if(ctrl_state_store_buff.size() < 10)
                    {
                        ctrl_state_store_buff.emplace(ctrl_state);//将数据放入队列中
                    }
                    else
                    {//如果队列中数据满10个，那么删掉队列头部数据然后队尾再插入数据
                        ctrl_state_store_buff.pop();//弹出队首的元素
                        ctrl_state_store_buff.emplace(ctrl_state);
                    }
                    checksum = 0;
            }
            // 需要拼接的情况
            else {
                uint8_t* data_len_ptr = (uint8_t*)(recv_buff);
                data_len_ptr += 7;
                //std::cout << "有包头但是数据长度不正确,8081数据校验长度: " << *((int*)(data_len_ptr))  << std::endl;
                memset(ctrl_state_temp_buff, 0, sizeof(ctrl_state_temp_buff));//清空临时存放变量
                memcpy(ctrl_state_temp_buff, recv_buff, ctrl_state_datalen);//只复制接收到的数据部分
                ctrl_state_future_recv = _CTRL_STATE_SIZE - ctrl_state_datalen;
            }
        }else{
            if(ctrl_state_datalen == -1){
                int error_code = 0;
                socklen_t len1 = sizeof(error_code);
                getsockopt(_socketfd2,SOL_SOCKET,SO_ERROR,&error_code,&len1);
                //break;//已经没有数据了，退出循环
                // std::cout << "请检查网络连接 " << strerror(errno) << "  " << errno << std::endl;                
            }            
            else{//进行数据拼接
                memset(ctrl_state_temp_buff, 0, sizeof(ctrl_state_temp_buff));//清空临时存放变量
                memcpy(ctrl_state_temp_buff, recv_buff, ctrl_state_datalen);//只复制接收到的数据部分
                ctrl_state_future_recv = _CTRL_STATE_SIZE - ctrl_state_datalen;
                 //std::cout << "获得不完整头部数据 " << ctrl_state_datalen  << std::endl;
                if(ctrl_state_store_buff.size() < 10){
                    ctrl_state_store_buff.emplace(ctrl_state);//将数据放入队列中
                }else{//队列长度等于10的时候，弹出队首的元素，在队尾加入元素
                    ctrl_state_store_buff.pop();
                    ctrl_state_store_buff.emplace(ctrl_state);
                }
                ctrl_state_future_recv = _CTRL_STATE_SIZE;
            }
        }
    }//end while

    if(!ctrl_state_store_buff.empty())
        {
            ctrl_state = ctrl_state_store_buff.front();
            ctrl_state_store_buff.pop();

            auto msg = robot_feedback_msg();
            msg.prg_state = ctrl_state.program_state;
            msg.error_code = 0;
            msg.robot_mode = ctrl_state.robot_mode;
            msg.j1_cur_pos = ctrl_state.jt_cur_pos[0];
            msg.j2_cur_pos = ctrl_state.jt_cur_pos[1];
            msg.j3_cur_pos = ctrl_state.jt_cur_pos[2];
            msg.j4_cur_pos = ctrl_state.jt_cur_pos[3];
            msg.j5_cur_pos = ctrl_state.jt_cur_pos[4];
            msg.j6_cur_pos = ctrl_state.jt_cur_pos[5];
            msg.cart_x_cur_pos = ctrl_state.tl_cur_pos_base[0];
            msg.cart_y_cur_pos = ctrl_state.tl_cur_pos_base[1];
            msg.cart_z_cur_pos = ctrl_state.tl_cur_pos_base[2];
            msg.cart_a_cur_pos = ctrl_state.tl_cur_pos_base[3];
            msg.cart_b_cur_pos = ctrl_state.tl_cur_pos_base[4];
            msg.cart_c_cur_pos = ctrl_state.tl_cur_pos_base[5];

            msg.flange_x_cur_pos = ctrl_state.flange_cur_pos[0];
            msg.flange_y_cur_pos = ctrl_state.flange_cur_pos[1];
            msg.flange_z_cur_pos = ctrl_state.flange_cur_pos[2];
            msg.flange_a_cur_pos = ctrl_state.flange_cur_pos[3];
            msg.flange_b_cur_pos = ctrl_state.flange_cur_pos[4];
            msg.flange_c_cur_pos = ctrl_state.flange_cur_pos[5];

            msg.work_num = ctrl_state.workPieceNum;
            msg.tool_num = ctrl_state.toolNum;

            // msg.exaxispos1 = ctrl_state.exaxis_status[0].exAxisPos;=
            msg.exaxispos1 = ctrl_state.auxservo_state.servo_actual_pos;
            
            msg.exaxispos2 = ctrl_state.exaxis_status[1].exAxisPos;
            msg.exaxispos3 = ctrl_state.exaxis_status[2].exAxisPos;
            msg.exaxispos4 = ctrl_state.exaxis_status[3].exAxisPos;
            msg.robot_motion_done = ctrl_state.motion_done;
            msg.j1_cur_tor = ctrl_state.jt_cur_tor[0];
            msg.j2_cur_tor = ctrl_state.jt_cur_tor[1];
            msg.j3_cur_tor = ctrl_state.jt_cur_tor[2];
            msg.j4_cur_tor = ctrl_state.jt_cur_tor[3];
            msg.j5_cur_tor = ctrl_state.jt_cur_tor[4];
            msg.j6_cur_tor = ctrl_state.jt_cur_tor[5];
            msg.prg_name = std::string(ctrl_state.curLuaFileName);
            msg.prg_total_line = 0;
            msg.prg_cur_line = 0;
            msg.dgt_output_h = ctrl_state.cl_dgt_output_h;
            msg.dgt_output_l = ctrl_state.cl_dgt_output_l;
            msg.tl_dgt_output_l = ctrl_state.tl_dgt_output_l;
            msg.dgt_input_h = ctrl_state.cl_dgt_input_h;
            msg.dgt_input_l = ctrl_state.cl_dgt_input_l;
            msg.tl_dgt_input_l = ctrl_state.tl_dgt_input_l;
            msg.ft_fx_data = ctrl_state.FT_data[0];
            msg.ft_fy_data = ctrl_state.FT_data[1];
            msg.ft_fz_data = ctrl_state.FT_data[2];
            msg.ft_tx_data = ctrl_state.FT_data[3];
            msg.ft_ty_data = ctrl_state.FT_data[4];
            msg.ft_tz_data = ctrl_state.FT_data[5];
            msg.ft_actstatus = ctrl_state.FT_ActStatus;
            msg.emg = ctrl_state.btn_box_stop_signal;
            msg.grip_motion_done = ctrl_state.gripperMotionDone;
            msg.ctrlboxerror=ctrl_state.ctrlBoxError;
            // int i = 0;
            // for(auto itme = msg.safetyboxsig.begin();itme < msg.safetyboxsig.end();itme++){
            //     *itme = ctrl_state.safetyBoxSignal[i];
            //     i++;
            // }

            msg.check_sum = state_data.check_sum;
            // msg.start_return = _start_return_data;
            _state_publisher->publish(msg);

            // std::cout << int(msg.dgt_input_h) <<" "<< int(msg.dgt_input_l) <<std::endl;
        }
    //下面是从队列中读取数据
    if(!store_buff.empty() && 0){//如果队列不为空，那就读取数据，否则auxservo_state就跳过本次callback
        state_data = store_buff.front();
        store_buff.pop();
        auto msg = robot_feedback_msg();
        // msg.prg_state = state_data.prg_state;
        // _program_state = msg.prg_state;
        // msg.error_code = state_data.error_code;
        // msg.robot_mode = state_data.robot_mode;
        // msg.j1_cur_pos = state_data.j1_cur_pos;
        // msg.j2_cur_pos = state_data.j2_cur_pos;
        // msg.j3_cur_pos = state_data.j3_cur_pos;
        // msg.j4_cur_pos = state_data.j4_cur_pos;
        // msg.j5_cur_pos = state_data.j5_cur_pos;
        // msg.j6_cur_pos = state_data.j6_cur_pos;
        // msg.cart_x_cur_pos = state_data.cart_x_cur_pos;
        // msg.cart_y_cur_pos = state_data.cart_y_cur_pos;
        // msg.cart_z_cur_pos = state_data.cart_z_cur_pos;
        // msg.cart_a_cur_pos = state_data.cart_a_cur_pos;
        // msg.cart_b_cur_pos = state_data.cart_b_cur_pos;
        // msg.cart_c_cur_pos = state_data.cart_c_cur_pos;

        // msg.j1_cur_tor = state_data.j1_cur_tor;
        // msg.j2_cur_tor = state_data.j2_cur_tor;
        // msg.j3_cur_tor = state_data.j3_cur_tor;
        // msg.j4_cur_tor = state_data.j4_cur_tor;
        // msg.j5_cur_tor = state_data.j5_cur_tor;
        // msg.j6_cur_tor = state_data.j6_cur_tor;
        // msg.prg_name = std::string(state_data.prg_name);
        // msg.prg_total_line = state_data.prg_total_line;
        // msg.prg_cur_line = state_data.prg_cur_line;
        // msg.dgt_output_h = state_data.dgt_output_h;
        // msg.dgt_output_l = state_data.dgt_output_l;
        // msg.tl_dgt_output_l = state_data.dgt_output_l;
        // msg.dgt_input_h = state_data.dgt_input_h;
        // msg.dgt_input_l = state_data.dgt_input_l;
        // msg.tl_dgt_input_l = state_data.tl_dgt_input_l;
        // msg.ft_fx_data = state_data.FT_Fx_data;
        // msg.ft_fy_data = state_data.FT_Fy_data;
        // msg.ft_fz_data = state_data.FT_Fz_data;
        // msg.ft_tx_data = state_data.FT_Tx_data;
        // msg.ft_ty_data = state_data.FT_Ty_data;
        // msg.ft_tz_data = state_data.FT_Tz_data;
        // msg.ft_actstatus = state_data.FT_ActStatus;
        // msg.emg = state_data.EMG;
        // // msg.robot_motion_done = state_data.robot_motion_done;
        // msg.grip_motion_done = state_data.grip_motion_done;
        msg.check_sum = state_data.check_sum;

        // msg.start_return = _start_return_data;
        _state_publisher->publish(msg);
        //std::cout <<_start_return_data<<std::endl;
        

    }
}
