#include "fairino_hardware/CNDE_thread.hpp"
#include <iostream>
#include "fairino_hardware/global_val_def.hpp"

#define LOGGER_NAME "CNDE_state_thread"

/**
 * @brief 状态监控节点构造函数
 * @param [in] node_name-节点名称
 */
CNDE_recv_thread::CNDE_recv_thread(const std::string node_name):rclcpp::Node(node_name){
    using namespace std::chrono_literals;
    _controller_ip = CONTROLLER_IP;//控制器默认ip地址

    //创建套字
    _socketfd1 = socket(AF_INET,SOCK_DGRAM,0);//使用UDP通讯

    if(_socketfd1 == -1){
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"端口连接失败，线程退出.");
        exit(0);//创建套字失败,丢出错误
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"CNDE socket创建成功.");
    
        udp_client1 = new struct sockaddr_in;
        udp_client1->sin_family = AF_INET;
        udp_client1->sin_port = htons(port1);//20006端口
        udp_client1->sin_addr.s_addr = inet_addr(_controller_ip.c_str());

        //将socket设置成非阻塞模式
        int flags1 = fcntl(_socketfd1,F_GETFL,0);
        fcntl(_socketfd1,F_SETFL,flags1|SOCK_NONBLOCK);


        /*首先往端口发送停止帧，设置功能只有在CNDE功能停止后才能生效*/
        uint8_t stop_buf[]{0x5A,0x5A,0x00,0x03,0x00,0x00,0xA5,0xA5};
        sendto(_socketfd1,stop_buf,8,0,(struct sockaddr*)udp_client1,sizeof(struct sockaddr));

        /*往端口发送设置帧*/
        std::string output_config = "actual_joint_pos,actual_TCP_pos";
        OutConfig o_config;
        o_config.len = output_config.size() + 2;
        memcpy(o_config.data,output_config.c_str(),31);
        uint8_t buf[256];
        memcpy(buf,&o_config,sizeof(o_config));
        //std::cout << "o_config尺寸:" << sizeof(o_config) << std::endl;
        // for(int i=0;i<sizeof(o_config);i++){
        //     printf("buf index %d:%x\r\n",i,buf[i]);
        // }
        //发送配置帧
        sendto(_socketfd1,buf,sizeof(o_config),0,(struct sockaddr*)udp_client1,sizeof(struct sockaddr));
        sleep(1);
        socklen_t addr_len = sizeof(udp_client1);
        auto recv_size = recvfrom(_socketfd1,buf,256,0,(struct sockaddr*)udp_client1,&addr_len);
        //std::cout << "config反馈数据大小:" << recv_size << std::endl;
        sleep(1);

        /*发送CNDE开始帧*/
        uint8_t start_buf[]{0x5A,0x5A,0x00,0x02,0x00,0x00,0xA5,0xA5};
        sendto(_socketfd1,start_buf,8,0,(struct sockaddr*)udp_client1,sizeof(struct sockaddr));

        /*创建共享内存空间，用于发布topic消息的同时，把数据结构也塞入共享内存中*/
        _shm_fd = shm_open("fairino_nonrt_state_data",O_CREAT|O_RDWR,0777);
        _sem = sem_open("/sem_nonrt_state_data",O_CREAT,0666,1);//创建信号量
        if(_sem == SEM_FAILED){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"sem create failed");
        }
        if(_shm_fd < 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"shared memory create failed");
            exit(0);//丢出错误并返回
        }
        ftruncate(_shm_fd,SHM_SHARED_DATA_SIZE);//预留10000byte空间容量
        _shm_state_data = (uint8_t*)mmap(NULL,SHM_SHARED_DATA_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,_shm_fd,0);
        /***********************************************************/

        _locktimer = this->create_wall_timer(1ms,std::bind(&CNDE_recv_thread::_state_recv_callback,this));//创建一个定时器任务用于获取非实时状态数据,触发间隔为100ms

    }
}

CNDE_recv_thread::~CNDE_recv_thread(){
    munmap(_shm_state_data,SHM_SHARED_DATA_SIZE);
    close(_shm_fd);
}


void CNDE_recv_thread::_state_recv_callback(){
    char buf[256];
    static CNDEState rt_state;
    static auto cur_clock = rclcpp::Clock();//使用系统时钟作为时间戳

    socklen_t addr_len = sizeof(udp_client1);
    auto recv_size = recvfrom(_socketfd1,buf,104,0,(struct sockaddr*)udp_client1,&addr_len);
    //std::cout << "size:" << recv_size << std::endl;

    if(recv_size == 104){
        memcpy(&rt_state,buf,104);
        //std::cout << "head:" << std::hex << rt_state.head <<std::endl;
        //std::cout << "jnt_pos:" << rt_state.jnt_pos[0] << "," << rt_state.jnt_pos[1] << std::endl;
        //std::cout << "tcp_pos:" << rt_state.tcp_pos[0] << "," << rt_state.tcp_pos[1] << "," << rt_state.tcp_pos[2] << std::endl;

        ShmData shm_shared_data;
        for(int i=0;i<6;i++){
            shm_shared_data.jnt_cur_pos[i] = rt_state.jnt_pos[i];
            shm_shared_data.tcp_cur_pos[i] = rt_state.tcp_pos[i];
            shm_shared_data.flange_cur_pos[i] = rt_state.tcp_pos[i];
        }

        shm_shared_data.timestamp = cur_clock.now().nanoseconds();

        for(int i=0;i<4;i++){
            shm_shared_data.exaxis_cur_pos[i] = global_exaxis_pos()[i];
        }

        //sem_wait(_sem);
        if(sem_trywait(_sem) == 0){
            memcpy(_shm_state_data,&shm_shared_data,sizeof(shm_shared_data));
            sem_post(_sem);
        }
        // uint8_t* pt_head = (uint8_t*)_shm_state_data;
        // std::cout << *(double*)pt_head << "," << shm_shared_data.jnt_cur_pos[0] << std::endl;
    }
}