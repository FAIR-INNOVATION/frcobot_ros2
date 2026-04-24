#include "fairino_version_pkg/fairino_version.hpp"
#include <sys/types.h>
#include <sys/socket.h>
#include "sys/mman.h"
#include <ament_index_cpp/get_package_share_directory.hpp>  
#include <regex>                                            
#include <thread>
#include <cstdlib>
#include <ament_index_cpp/get_package_prefix.hpp>


#define LOGGER_NAME "fairino_ros2_version_thread"
std::atomic_bool _reconnect_flag;

#ifdef CHN_VERSION
char* msgout[] = {
    "开始创建状态反馈TCP socket",
    "错误: 创建socket失败!",
    "创建状态反馈socket成功,开始连接控制器...",
    "错误:无法连接控制器反馈数据端口,程序即将退出!",
    "控制器状态端口连接成功",
    "开启tcp心跳检测失败",
    "守护线程:创建套接字失败, 3s后再次尝试",
    "守护线程:发起重新连接失败, 3s后再次尝试",
    "守护线程:重新连接成功",
    "守护线程:重连线程退出",
    "网络断开，请检查网络",
    "正在获取当前软件版本号",
    "当前软件版本为：",
    "未匹配到软件版本号，原始回复：",
    "正在启动fairino_hardware...",
    "错误：未找到与当前机械臂软件版本号匹配的 fairino_hardware"
};
#endif

#ifdef ENG_VERSION
char* msgout[] = {
    "Ready to create state feedback client socket",
    "Error:socket create failed",
    "Socket created,ready to connect robot...",
    "Error:failed to connect robot state feedback port,program about to exit!",
    "Connected to robot state feedback port",
    "Failed to set socket keep alive",
    "Keep alive:recreate socket failed, try again after 3sec",
    "Keep alive:reconnect robot failed, try again after 3sec",
    "Keep alive:reconnect success!",
    "Keep alive:thread exit!",
    "State feedback socket disconnected, please check your network",
    "Current robotic arm software version number:",
    "The current software version is:",
    "No software version number was matched. Original reply:",
    "Starting fairino_hardware...",
    "Error: No fairino_hardware found that matches the current robot arm software version number"
};
#endif

typedef enum _msg_id{
    create_state_feedback,
    socket_create_failed,
    socket_create_success,
    socket_connect_failed,
    socket_connect_success,
    keep_alive_failed,
    keep_alive_recreate_socket_failed,
    keep_alive_reconnect_failed,
    keep_alive_reconnect_success,
    keep_alive_exit,
    network_diconnect,
    current_ver_robot,
    ver_robot,
    not_ver_robot,
    connect_package,
    not_package,
}msg_id;


/**
 * @brief 状态监控节点构造函数
 * @param [in] node_name-节点名称
 */
robot_version_thread::robot_version_thread(const std::string node_name):rclcpp::Node(node_name){
    using namespace std::chrono_literals;
    _controller_ip = CONTROLLER_IP;//控制器默认ip地址
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(create_state_feedback)]);

    //8080端口获取数据
    _socketfd1 = socket(AF_INET,SOCK_STREAM,0);//状态获取端口只有TCP

    if(_socketfd1 == -1){
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(socket_create_failed)]);
        exit(0);//创建套字失败,丢出错误
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(socket_create_success)]);
        struct sockaddr_in tcp_client1;
        tcp_client1.sin_family = AF_INET;
        tcp_client1.sin_port = htons(port1);//8080端口
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());


        //尝试连接控制器
        int res1 = connect(_socketfd1,(struct sockaddr *)&tcp_client1,sizeof(tcp_client1));
        if(0 != res1){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(socket_connect_failed)]);
            exit(0);//连接失败,丢出错误并返回
        }else{
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(socket_connect_success)]);

            //设置超时接受
            struct timeval tv;
            tv.tv_sec  = 1;     // 1 秒
            tv.tv_usec = 0;
            setsockopt(_socketfd1, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            
            //开启keepalive
            if(0 != setKeepAlive(_socketfd1, 5, 3, 3)){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(keep_alive_failed)]);
            }
        }
        
          // 启动定时器：100ms 查询一次，成功后会 cancel + 退出
        _locktimer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&robot_version_thread::_state_recv_callback, this));
        //连接成功，创建守护线程,如果该连接端掉，则自动发起重连接;生命周期随该节点
        _try_to_reconnect();
    }
}

robot_version_thread::~robot_version_thread(){
    //关闭并销毁socket
     _robot_recv_exit.store(true);

 // 删除软链接
   
    std::string lib_2_2 = libdir + "/libfairino.so.2.2";
    std::string lib_2 = libdir + "/libfairino.so.2";

    std::remove(lib_2_2.c_str());
    std::remove(lib_2.c_str());

     
  if(_locktimer) _locktimer->cancel();

  if(_reconnect_thread.joinable())
    _reconnect_thread.join();

  std::lock_guard<std::mutex> lk(sock_mtx_);
  if(_socketfd1 != -1){
    shutdown(_socketfd1, SHUT_RDWR);
    close(_socketfd1);
    _socketfd1 = -1;
  }
}

/**
 * @brief TCP断开重连函数
 */
void robot_version_thread::_try_to_reconnect(){
    auto _reconnect_func = [this](){
        while ((1 != _robot_recv_exit)){
            /* try to re-connect 58.2 8081*/
            if (_reconnect_flag){
                // 关闭旧连接
                shutdown(_socketfd1, SHUT_RDWR);
                close(_socketfd1);
                _socketfd1 = -1;
                // std::this_thread::sleep_for(std::chrono::seconds(1));

                int sock_fd = socket(AF_INET, SOCK_STREAM, 0);
                if (-1 == sock_fd){
                    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(\
                        keep_alive_recreate_socket_failed)]);
                }
                else
                {
                    struct sockaddr_in tcp_client1;
                    tcp_client1.sin_family = AF_INET;
                    tcp_client1.sin_port = htons(port1);
                    tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

                    // 尝试连接控制器
                    int res1 = connect(sock_fd, (struct sockaddr *)&tcp_client1, sizeof(tcp_client1));
                    if (res1)
                    {
                        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(\
                            keep_alive_reconnect_failed)]);
                        shutdown(sock_fd, SHUT_RDWR);
                        close(sock_fd);
                    }
                    else
                    {
                        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(\
                            keep_alive_reconnect_success)]);
                        // 设置TCP接收超时
                        int flags2 = fcntl(sock_fd, F_GETFL, 0);
                        fcntl(sock_fd, F_SETFL, flags2 | SOCK_NONBLOCK);

                        // 开启并设置keepalive
                        if (0 != setKeepAlive(sock_fd, 5, 3, 3)){
                            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(\
                                keep_alive_failed)]);
                        }
                        // return sock_fd;
                        _socketfd1 = sock_fd;
                        _reconnect_flag.store(false);
                    }
                }
            }
            /* 以3s的频率检查 */
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
            
    };

    _reconnect_thread = std::thread(_reconnect_func);
    _reconnect_thread.detach();
}

/**
 * @brief 
 * @param idle_time 空闲idle_time后，开始发射探针
 * @param interval_time 发射首个探针后，如果interval_time内没有响应，再次发射探针
 * @param probe_times 一共会发射probe_times次探针
 * @return -1-开启失败；0-成功
*/
int robot_version_thread::setKeepAlive(int fd, int idle_time, int interval_time, int probe_times){
    int val = 1;
	//开启keepalive机制
    if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val)) == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"setsockopt SO_KEEPALIVE: %s", strerror(errno));
        return -1;
    }
 
    val = idle_time;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &val, sizeof(val)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"setsockopt TCP_KEEPIDLE: %s\n", strerror(errno));
        return -1;
    }
 
    val = interval_time;
    if (val == 0) val = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &val, sizeof(val)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"setsockopt TCP_KEEPINTVL: %s\n", strerror(errno));
        return -1;
    }
 
    val = probe_times;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &val, sizeof(val)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"setsockopt TCP_KEEPCNT: %s\n", strerror(errno));
        return -1;
    }
 
    return 0;
}





void robot_version_thread::_state_recv_callback(){
    /* 如果处于重连流程，不需要再读取，直接返回 */
    if(_reconnect_flag){
        //RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"重连中，请等待......");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(current_ver_robot)]);

    static char recv_buff[RECV_BUFF];
    std::string data = "/f/bIII4III905III20IIIGetSoftwareVersion()III/b/f";
    send(_socketfd1, data.c_str(), data.size(), 0); // 发送指令信息
    memset(recv_buff, 0, sizeof(recv_buff));
    int recv_bytes = recv(_socketfd1, recv_buff, sizeof(recv_buff), 0);
    if (recv_bytes > 0)
    {
        // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(receive_command)]);
        std::string resp(recv_buff, recv_bytes);        // 用有效字节构造字符串

        
        // 1) 停止定时器
        if (_locktimer) _locktimer->cancel();

        // 2) 关闭 8080 socket
        shutdown(_socketfd1, SHUT_RDWR);
        close(_socketfd1);
        _socketfd1 = -1;

        // static const std::regex re(R"(\),([^,]+),V)");
        static const std::regex re(R"(v([^,]+),V)");
        std::smatch m;
        std::string ver;

        if (std::regex_search(resp, m, re) && m.size() >= 2) {
            ver = "v"+m[1].str();
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), 
                    "%s %s", 
                    msgout[msg_id(ver_robot)], 
                    ver.c_str());   
        } else {
            // std::cout << "未匹配到版本字段，原始回复: " << resp << std::endl;
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
                    "%s %s", 
                    msgout[msg_id(not_ver_robot)], 
                    resp.c_str());
        }

        for (auto &c : ver) if (c == '.') c = '_';    // v3_8_2
        std::string pkg = "fairino_hardware_" + ver;  // fairino_hardware_v3_8_2
        if (package_exists(pkg)) {
            std::cout << "包存在: " << pkg << "，准备启动节点..." << std::endl;
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(connect_package)]);

            // 0）先删除掉原来的软连接
            for(int i = 0; i < stringList.size(); i++)
            {
                try {
                    std::string so_prefix = ament_index_cpp::get_package_prefix(stringList[i]);
                    
                    std::string so_libdir = so_prefix + "/lib";
                    std::string so_lib_2_2 = so_libdir + "/libfairino.so.2.2";
                    std::string so_lib_2 = so_libdir + "/libfairino.so.2";
                    std::remove(so_lib_2_2.c_str());
                    std::remove(so_lib_2.c_str());
                }catch (const ament_index_cpp::PackageNotFoundError& e) {
                    // std::cout << "Package not found: " << stringList[i] << ". Skipping..." << std::endl;
                    continue;
                }
            }

             // 1) 先定义 local_setup（用 share 目录拼）
            std::string local_setup =
                ament_index_cpp::get_package_share_directory(pkg) + "/local_setup.bash";

            

            // 2) 再拿 prefix/lib（用于固定 LD_LIBRARY_PATH，避免串库）
            std::string prefix = ament_index_cpp::get_package_prefix(pkg);
            libdir = prefix + "/lib";
            

            // 原始库文件
            // 修改第二位数字，减去6
            int second_digit = ver[3] - '0';  // 将字符转换为数字
            second_digit -= 6;  // 第二位数字减去6

            // 获取第三位数字
            int third_digit = ver[5] - '0';  // 将字符转换为数字

            // 构建新的库文件名
            std::string original_lib = "libfairino.so.2." + std::to_string(second_digit )+ "." + std::to_string(third_digit);

            // 输出新的库文件名
            // std::cout << "新库文件名: " << original_lib << std::endl;
            // std::string original_lib = "libfairino.so.2.3.0";
            target_lib = libdir + "/" + original_lib;


            // 创建 libfairino 的软链接
            std::string lib_2_2 = libdir + "/libfairino.so.2.2";
            std::string lib_2 = libdir + "/libfairino.so.2";

            // 创建软链接的命令
            std::string symlink_cmd = 
                "ln -sf " + target_lib + " " + lib_2_2 + " && "   // 将 libfairino.so.2.3.0 链接到 libfairino.so.2.2
                "ln -sf " + lib_2_2 + " " + lib_2;  // 将 libfairino.so.2.2 链接到 libfairino.so.2
            
            // 执行软链接创建命令
            std::system(symlink_cmd.c_str());


             std::string cmd =
                    "source " + local_setup + " ; "
                    "export LD_LIBRARY_PATH=" + libdir + ":$LD_LIBRARY_PATH ; "
                    "trap \"kill $(jobs -p)\" SIGINT; "  // 捕获 SIGINT 并关闭后台进程
                    "ros2 run " + pkg + " ros2_cmd_server &";  // 后台运行 ros2_cmd_server

            // 阻塞运行
            std::thread([cmd]() {
                std::system(cmd.c_str());
            }).join();  // 等待线程完成

        } else {
            // std::cout << "包不存在: " << std::endl;
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), 
                    "%s %s %s", 
                    msgout[msg_id(not_package)], 
                    pkg.c_str(),resp.c_str());
            return;
        }


    }
    else if (recv_bytes == 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(network_diconnect)]);
    }
    else { // recv_bytes < 0
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            _reconnect_flag.store(true);
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(network_diconnect)]);
            return;
        }
    }
}


bool robot_version_thread::package_exists(const std::string& pkg_name)
{
  try {
    (void)ament_index_cpp::get_package_share_directory(pkg_name);
    return true;   // 能找到 share 目录 → 包存在（已 install）
  } catch (...) {
    return false;  // 找不到 → 包不存在
  }
}