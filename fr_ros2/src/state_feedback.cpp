#include "state_feedback.h"


/******状态信息获取节点******/
/*************************/
state_recv_thread::state_recv_thread(const std::string node_name):rclcpp::Node(node_name,
                   rclcpp::NodeOptions().use_intra_process_comms(true))
{
    using namespace std::chrono_literals;
    _controller_ip = "192.168.58.2";//控制器默认ip地址
    std::cout << "开始创建状态反馈TCP socket" << std::endl;
    _socketfd1 = socket(AF_INET,SOCK_STREAM,0);//状态获取端口只有TCP
    _socketfd2 = socket(AF_INET,SOCK_STREAM,0);
    if(_socketfd1 == -1 || _socketfd2 == -1){
        std::cout << "错误: 创建socket失败！" << std::endl;
        exit(0);//创建套字失败，丢出错误
    }else{
        std::cout << "创建状态反馈socket成功，开始连接控制器..." << std::endl;
        struct sockaddr_in tcp_client1,tcp_client2;
        tcp_client1.sin_family = AF_INET;
        tcp_client2.sin_family = AF_INET;
        tcp_client1.sin_port = htons(port1);//8083端口
        tcp_client2.sin_port = htons(port2);//30004端口
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());
        tcp_client2.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

        //尝试连接控制器
        int res1 = connect(_socketfd1,(struct sockaddr *)&tcp_client1,sizeof(tcp_client1));
        int res2 = connect(_socketfd2,(struct sockaddr *)&tcp_client2,sizeof(tcp_client2));
        if(res1 || res2){
            std::cout << "错误:无法连接控制器数据端口，程序退出!" << std::endl;
            exit(0);//连接失败，丢出错误并返回
        }else{
            std::cout << "控制器状态端口连接成功" << std::endl;
            /*
            //下面开始RTDE端口配置及启动信息发送，确认启动成功后即可开始后续程序运行
            std::string RTDE_config_data,RTDE_start_data;
            std::string config_data = std::string("actual_p,actual_dp,actual_current,\
                               actual_TCP_pose,actual_TCP_speed,actual_TCP_force");
            std::string config_data_head = std::string("0x5A5A01");
            std::string config_data_len = std::to_string(config_data.size());
            std::string config_data_checksum = std::to_string(config_data_head.size() + config_data.size() + config_data_len.size());
            RTDE_config_data = config_data_head + config_data_len + config_data + config_data_checksum;
            send(_socketfd2,RTDE_config_data.c_str(),RTDE_config_data.size(),0);//30004 RTDE端口发送配置信息
            config_data_head = std::string("0x5A5A120");
            RTDE_start_data = config_data_head + std::to_string(config_data_head.size());
            send(_socketfd2,RTDE_start_data.c_str(),RTDE_start_data.size(),0);//30004 RTDE端口发送启动信息
            */

            // char recv_buff[128];
            // int cnt = 0;
            // while(recv(_socketfd2,recv_buff,sizeof(recv_buff),0) == -1){ 
            //     cnt++;
            //     if(cnt == 50){
            //         std::cout << "警告:实时数据端口未配置成功,程序退出" << std::endl;
            //         exit(0);
            //     }
            // }

            //将两个socket设置成非阻塞模式
            int flags1 = fcntl(_socketfd1,F_GETFL,0);
            int flags2 = fcntl(_socketfd2,F_GETFL,0);
            fcntl(_socketfd1,F_SETFL,flags1|SOCK_NONBLOCK);
            fcntl(_socketfd2,F_SETFL,flags2|SOCK_NONBLOCK);
            //std::cout << "数据反馈端口配置成功" << std::endl;
            _state_publisher = this->create_publisher<frhal_msgs::msg::FRState>(
                "nonrt_state_data",
                10
            );
            _rt_state_publisher = this->create_publisher<std_msgs::msg::String>(
                "rt_state_data",
                10
            );
            _locktimer = this->create_wall_timer(100ms,std::bind(&state_recv_thread::_state_recv_callback,this));//创建一个定时器任务用于获取非实时状态数据，触发间隔为100ms
            //_rt_locktimer = this->create_wall_timer(8ms,std::bind(&state_recv_thread::_rt_state_recv_callback,this));//创建一个定时器人物用于获取实时状态数据，触发间隔8ms
        }
    }
}

state_recv_thread::~state_recv_thread(){
    //关闭并销毁socket
    if(_socketfd1 != -1){
        close(_socketfd1);
    }
    if(_socketfd2 != -1){
        close(_socketfd2);
    }
}

void state_recv_thread::_state_recv_callback(){
    static char recv_buff[241];
    static FR_nonrt_state state_data;
    memset(recv_buff,0,sizeof(recv_buff));
    if(recv(_socketfd1,recv_buff,sizeof(recv_buff),0) > -1){
        memcpy(&state_data,recv_buff,sizeof(recv_buff));
        auto msg = frhal_msgs::msg::FRState();
        msg.prg_state = state_data.prg_state;
        msg.error_code = state_data.error_code;
        msg.robot_mode = state_data.robot_mode;
        msg.j1_cur_pos = state_data.j1_cur_pos;
        msg.j2_cur_pos = state_data.j2_cur_pos;
        msg.j3_cur_pos = state_data.j3_cur_pos;
        msg.j4_cur_pos = state_data.j4_cur_pos;
        msg.j5_cur_pos = state_data.j5_cur_pos;
        msg.j6_cur_pos = state_data.j6_cur_pos;
        msg.cart_x_cur_pos = state_data.cart_x_cur_pos;
        msg.cart_y_cur_pos = state_data.cart_y_cur_pos;
        msg.cart_z_cur_pos = state_data.cart_z_cur_pos;
        msg.cart_a_cur_pos = state_data.cart_a_cur_pos;
        msg.cart_b_cur_pos = state_data.cart_b_cur_pos;
        msg.cart_c_cur_pos = state_data.cart_c_cur_pos;
        msg.tool_num = state_data.tool_num;
        msg.j1_cur_tor = state_data.j1_cur_tor;
        msg.j2_cur_tor = state_data.j2_cur_tor;
        msg.j3_cur_tor = state_data.j3_cur_tor;
        msg.j4_cur_tor = state_data.j4_cur_tor;
        msg.j5_cur_tor = state_data.j5_cur_tor;
        msg.j6_cur_tor = state_data.j6_cur_tor;
        msg.prg_name = std::string(state_data.prg_name);
        msg.prg_total_line = state_data.prg_total_line;
        msg.prg_cur_line = state_data.prg_cur_line;
        msg.dgt_output_h = state_data.dgt_output_h;
        msg.dgt_output_l = state_data.dgt_output_l;
        msg.tl_dgt_output_l = state_data.dgt_output_l;
        msg.dgt_input_h = state_data.dgt_input_h;
        msg.dgt_input_l = state_data.dgt_input_l;
        msg.tl_dgt_input_l = state_data.tl_dgt_input_l;
        msg.ft_fx_data = state_data.FT_Fx_data;
        msg.ft_fy_data = state_data.FT_Fy_data;
        msg.ft_fz_data = state_data.FT_Fz_data;
        msg.ft_tx_data = state_data.FT_Tx_data;
        msg.ft_ty_data = state_data.FT_Ty_data;
        msg.ft_tz_data = state_data.FT_Tz_data;
        msg.ft_actstatus = state_data.FT_ActStatus;
        msg.emg = state_data.EMG;
        msg.robot_motion_done = state_data.robot_motion_done;
        msg.grip_motion_done = state_data.grip_motion_done;
        _state_publisher->publish(msg);
    }
}

void state_recv_thread::_rt_state_recv_callback(){
    static char recv_buff[300];
    memset(recv_buff,0,sizeof(recv_buff));
    if(recv(_socketfd2,recv_buff,sizeof(recv_buff),0) > -1){
        //_rt_state_publisher->publish(msg);

    }
}
