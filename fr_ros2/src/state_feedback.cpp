#include "state_feedback.h"


/******状态信息获取节点******/
/*************************/
state_recv_thread::state_recv_thread(const std::string node_name):rclcpp::Node(node_name,
                   rclcpp::NodeOptions().use_intra_process_comms(true))
{
    using namespace std::chrono_literals;
    _controller_ip = "192.168.58.2";//控制器默认ip地址
    RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Start to create state feedback TCP socket.");
    //std::cout << "开始创建状态反馈TCP socket" << std::endl;
    _socketfd1 = socket(AF_INET,SOCK_STREAM,0);//状态获取端口只有TCP
    if(_socketfd1 == -1){
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1001:Failed to create state feedback TCP socket");
        //std::cout << "错误: 创建socket失败！" << std::endl;
        exit(0);//创建套字失败，丢出错误
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"State feedback TCP socket created.");
        //std::cout << "创建状态反馈socket成功，开始连接控制器..." << std::endl;
        struct sockaddr_in tcp_client1;
        tcp_client1.sin_family = AF_INET;
        tcp_client1.sin_port = htons(port1);//8083端口
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

        //尝试连接控制器
        int res1 = connect(_socketfd1,(struct sockaddr *)&tcp_client1,sizeof(tcp_client1));
        if(res1){
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1002:Can not connect to robot controller, program exit!");
            //std::cout << "错误:无法连接控制器数据端口，程序退出!" << std::endl;
            exit(0);//连接失败，丢出错误并返回
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Connected to robot controller.");
            //std::cout << "控制器状态端口连接成功" << std::endl;
            //将socket设置成非阻塞模式
            int flags1 = fcntl(_socketfd1,F_GETFL,0);
            fcntl(_socketfd1,F_SETFL,flags1|SOCK_NONBLOCK);
            //std::cout << "数据反馈端口配置成功" << std::endl;
            _state_publisher = this->create_publisher<frhal_msgs::msg::FRState>(
                "nonrt_state_data",
                10
            );
            _locktimer = this->create_wall_timer(100ms,std::bind(&state_recv_thread::_state_recv_callback,this));//创建一个定时器任务用于获取非实时状态数据，触发间隔为100ms
        }
    }
}

state_recv_thread::~state_recv_thread(){
    //关闭并销毁socket
    if(_socketfd1 != -1){
        close(_socketfd1);
    }
}

void state_recv_thread::_state_recv_callback(){
    static char recv_buff[sizeof(FR_nonrt_state)];
    static FR_nonrt_state state_data;
    memset(recv_buff,0,sizeof(recv_buff));
    if(recv(_socketfd1,recv_buff,sizeof(recv_buff),0) > -1){
        //std::cout << "got state data" << std::endl;
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
