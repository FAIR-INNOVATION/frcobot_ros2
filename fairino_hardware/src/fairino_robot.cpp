#include "fairino_hardware/fairino_robot.hpp"

namespace fairino_hardware{

fairino_robot::fairino_robot(){
//配置状态反馈端口，引入xmlrpc库
    _control_mode = 0;
    _controller_ip = CONTROLLER_IP;//控制器默认ip地址
    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:开始创建状态反馈socket");    
    _socket_state = socket(AF_INET,SOCK_STREAM,0);//状态获取端口只有TCP
    if(_socket_state == -1){
        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot错误: 创建状态反馈socket失败!");    
        exit(0);//创建套字失败,丢出错误
    }else{
        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:创建状态反馈socket成功,开始连接控制器...");    
        struct sockaddr_in tcp_client1;
        tcp_client1.sin_family = AF_INET;
        tcp_client1.sin_port = htons(_port_state);//rt反馈端口
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

        //尝试连接控制器
        int res1 = connect(_socket_state,(struct sockaddr *)&tcp_client1,sizeof(tcp_client1));
        if(res1){
            RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot错误:无法连接控制器数据端口,程序退出!");    
            exit(0);//连接失败,丢出错误并返回
        }else{
            RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:控制器状态端口连接成功"); 
            //设置tcp通道未非阻塞，设置超时的话会导致连接断开问题
            int flags1 = fcntl(_socket_state,F_GETFL,0);
            fcntl(_socket_state,F_SETFL,flags1|SOCK_NONBLOCK);   
            // //设置TCP通道的发送超时和接受超时
            // struct timeval timeout_val;
            // timeval.tv_sec = 1;//1s
            // timeval.tv_msec = 0;
            // setsockopt(_socket_state,SOL_SOCKET,SO_RCVTIMEO,&timeout_val,sizeof(timeout_val));
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:开始创建xmlrpc连接...");    
    _xml_client_ptr = std::make_unique<XmlRpc::XmlRpcClient>(_controller_ip.c_str(), _port_xmlrpc);//创建xmlrpc连接
    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:xmlrpc连接成功!");    
    _xml_client_ptr->setKeepOpen(1);//设置保持链接
}


fairino_robot::~fairino_robot(){
    if(_socket_state != -1){//关闭TCP端口
        shutdown(_socket_state,SHUT_RDWR);
    }
    _xml_client_ptr->close();//关闭xmlrpc端口
}


int fairino_robot::stoprobot(){
    XmlRpc::XmlRpcValue noArg, result;
    _xml_client_ptr->execute("StopMotion",noArg,result);
    return int(result);
    return 0;
}


int fairino_robot::inittorquecontrol(){
    if(!stoprobot()){
        _control_mode = TORQUE_CONTROL_MODE;
        return 0;
    }else{
        return -1;
    }
}


int fairino_robot::initpositioncontrol(){
//做一些调用servoj之前的准备工作
    if(!stoprobot()){
        _control_mode = POSITION_CONTROL_MODE;
        return 0;
    }else{
        return -1;
    }
}

FR_rt_state& fairino_robot::read(){
//调用TCP通讯，直接用memcpy函数进行拷贝，不需要展开数据
    static FR_rt_state state_data;
    static std::queue<FR_rt_state> store_buff;//用于存储缓存区多余的数据，需要限制长度
    static char temp_buff[RT_PACKAGE_SIZE] = {0};
    static int future_recv = RT_PACKAGE_SIZE;
    int datalen = RT_PACKAGE_SIZE;
    uint16_t* head_ptr;
    uint8_t* checksum_ptr;
    uint16_t checksum = 0;
    while(datalen > 0){
        char recv_buff[future_recv] = {0};
        datalen = recv(_socket_state,recv_buff,sizeof(recv_buff),0);//从缓存区获取数据
        head_ptr = (uint16_t*)(recv_buff);//取得帧头
        checksum_ptr = (uint8_t*)(recv_buff);//和校验指针
        if(*head_ptr == 0x5A5A){//检测包头
            if(datalen == RT_PACKAGE_SIZE){//有时候缓冲区尾部数据不是一个完整的帧，因此需要保存不完整的信息用于下一帧数据拼接
                for(int i=0;i< datalen-2;i++){
			        checksum += *checksum_ptr; 
			        checksum_ptr++;           
                }
                if(checksum == *((uint16_t*)checksum_ptr)){//进行和校验
                    memcpy(&state_data,recv_buff,sizeof(recv_buff));
                    if(store_buff.size() < 10){
                        store_buff.emplace(state_data);//将数据放入队列中
                        //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:获得完整数据,size:%d",datalen);    
                    	}else{//如果队列中数据满10个，那么删掉队列头部数据然后队尾再插入数据
                        	store_buff.pop();//弹出队首的元素
                        	store_buff.emplace(state_data);
                    	}
                    }else{//和校验不通过，丢出错误信息，不向队列中添加信息
                        //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:期望数据size:%d",datalen);
                        RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:状态数据和校验失败!当前数据包size:%d,和校验计算数据:%d,和校验读取数据:%d",datalen,checksum,*((uint16_t*)checksum_ptr));
                        state_data.check_sum = 0;    
                    }
                    checksum = 0;
            }else{//需要拼接的情况，或者数据结构不对导致长度小于期望值
                //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:获得不完整头部数据,size:%d",datalen);
                memset(temp_buff,0,sizeof(temp_buff));//清空临时存放变量
                memcpy(temp_buff,recv_buff,sizeof(recv_buff));//临时存放不完整数据
                for(int i=0;i< datalen;i++){
		            checksum += *checksum_ptr; 
		            checksum_ptr++;           
                }
                future_recv = RT_PACKAGE_SIZE - datalen;
            }
        }else{
            //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:未获得head,size:%d",datalen);
            if(datalen == -1){
                int error_code = 0;
                socklen_t len1 = sizeof(error_code);
                getsockopt(_socket_state,SOL_SOCKET,SO_ERROR,&error_code,&len1);//获取错误码
                //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:状态数据异常!");    
                //break;//已经没有数据了，退出循环
            }else{//进行数据拼接
                 for(int i=0;i< datalen-2;i++){//进行和校验计算
			        checksum += *checksum_ptr; 
			        checksum_ptr++;           
                 }
                 if(checksum == *((uint16_t*)checksum_ptr)){//进行和校验
                    memcpy(&temp_buff[RT_PACKAGE_SIZE-future_recv],recv_buff,sizeof(recv_buff));
                    memcpy(&state_data,temp_buff,sizeof(temp_buff));
                    //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:获得拼接数据，size:%d",datalen);
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
                future_recv = RT_PACKAGE_SIZE;
            }
            checksum = 0;
        }
    }//end while
    //下面是从队列中读取数据
    if(!store_buff.empty()){//如果队列不为空，那就读取数据，否则就跳过本次callback
        state_data = store_buff.front();
        store_buff.pop();
        //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:状态数据: %d,%d",state_data.frame_head,(int)state_data.frame_cnt);    
    }
    return state_data;
}


void fairino_robot::write(double cmd[6]){
//通过xmlrpc库写入servoj指令
    if(_control_mode == POSITION_CONTROL_MODE){//位置控制模式
        XmlRpc::XmlRpcValue Args, result;
        for(int i=0;i<6;i++){
            Args[0][i] = cmd[i];//赋值位置指令
        }
        for(int i=0;i<4;i++){
            Args[1][i] = 0.;//赋值位置指令
        }
        Args[2] = 0.;
        Args[3] = 0.;
        Args[4] = 0.008;//8ms
        Args[5] = 0.;
        Args[6] = 0.;
        try{
            //RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"发送servoJ: %f,%f,%f,%f,%f,%f",cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5]);    
            if (_xml_client_ptr->execute("ServoJ", Args, result)){
                if(int(result) != 0){
                    RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:收到指令反馈错误代码%d",int(result));    
                }
            }else{
                RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:指令未发送成功");    
            }
        }catch(XmlRpc::XmlRpcException e){
            RCLCPP_INFO(rclcpp::get_logger("FrHardwareInterface"),"fairino_robot:指令发送出现异常!信息:%s",e.getMessage().c_str());    
        }
    }else if(_control_mode == TORQUE_CONTROL_MODE){

    }
}


}//end namespace
