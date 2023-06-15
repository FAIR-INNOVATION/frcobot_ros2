#include "state_feedback.h"


/******Status information acquisition node******/
/*************************/
state_recv_thread::state_recv_thread(const std::string node_name):rclcpp::Node(node_name,
                   rclcpp::NodeOptions().use_intra_process_comms(true))
{
    using namespace std::chrono_literals;
    _controller_ip = "192.168.58.2";//Controller default ip address
    std::cout << "Start to create status feedback TCP socket" << std::endl;
    _socketfd1 = socket(AF_INET,SOCK_STREAM,0);//The status acquisition port is only TCP
    _socketfd2 = socket(AF_INET,SOCK_STREAM,0);
    if(_socketfd1 == -1 || _socketfd2 == -1){
        std::cout << "Error: Failed to create socket!" << std::endl;
        exit(0);//Failed to create socket, throwing error
    }else{
        std::cout << "The status feedback socket is created successfully, and the connection to the controller starts..." << std::endl;
        struct sockaddr_in tcp_client1,tcp_client2;
        tcp_client1.sin_family = AF_INET;
        tcp_client2.sin_family = AF_INET;
        tcp_client1.sin_port = htons(port1);//Port 8083
        tcp_client2.sin_port = htons(port2);//Port 30004
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());
        tcp_client2.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

        //Try to connect to the controller
        int res1 = connect(_socketfd1,(struct sockaddr *)&tcp_client1,sizeof(tcp_client1));
        int res2 = connect(_socketfd2,(struct sockaddr *)&tcp_client2,sizeof(tcp_client2));
        if(res1 || res2){
            std::cout << "ERROR: Failed to connect to controller data port, program exited!" << std::endl;
            exit(0);//Connection fails, throws an error and returns
        }else{
            std::cout << "Controller status port connection successful" << std::endl;
            /*
            //Next, start the RTDE port configuration and send the startup information. After confirming that the startup is successful, you can start the subsequent program operation.
            std::string RTDE_config_data,RTDE_start_data;
            std::string config_data = std::string("actual_p,actual_dp,actual_current,\
                               actual_TCP_pose,actual_TCP_speed,actual_TCP_force");
            std::string config_data_head = std::string("0x5A5A01");
            std::string config_data_len = std::to_string(config_data.size());
            std::string config_data_checksum = std::to_string(config_data_head.size() + config_data.size() + config_data_len.size());
            RTDE_config_data = config_data_head + config_data_len + config_data + config_data_checksum;
            send(_socketfd2,RTDE_config_data.c_str(),RTDE_config_data.size(),0);//30004 RTDE Port sending configuration information
            config_data_head = std::string("0x5A5A120");
            RTDE_start_data = config_data_head + std::to_string(config_data_head.size());
            send(_socketfd2,RTDE_start_data.c_str(),RTDE_start_data.size(),0);//30004 RTDE Port sends start information
            */

            // char recv_buff[128];
            // int cnt = 0;
            // while(recv(_socketfd2,recv_buff,sizeof(recv_buff),0) == -1){ 
            //     cnt++;
            //     if(cnt == 50){
            //         std::cout << "Warning: The real-time data port is not configured successfully, and the program exits" << std::endl;
            //         exit(0);
            //     }
            // }

            //Set the two sockets to non-blocking mode
            int flags1 = fcntl(_socketfd1,F_GETFL,0);
            int flags2 = fcntl(_socketfd2,F_GETFL,0);
            fcntl(_socketfd1,F_SETFL,flags1|SOCK_NONBLOCK);
            fcntl(_socketfd2,F_SETFL,flags2|SOCK_NONBLOCK);
            //std::cout << "The data feedback port configuration is successful" << std::endl;
            _state_publisher = this->create_publisher<frhal_msgs::msg::FRState>(
                "nonrt_state_data",
                10
            );
            _rt_state_publisher = this->create_publisher<std_msgs::msg::String>(
                "rt_state_data",
                10
            );
            _locktimer = this->create_wall_timer(100ms,std::bind(&state_recv_thread::_state_recv_callback,this));//Create a timer task to obtain non-real-time status data, with a trigger interval of 100ms
            //_rt_locktimer = this->create_wall_timer(8ms,std::bind(&state_recv_thread::_rt_state_recv_callback,this));//Create a timer character to obtain real-time status data, and the trigger interval is 8ms
        }
    }
}

state_recv_thread::~state_recv_thread(){
    //close and destroy socket
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
