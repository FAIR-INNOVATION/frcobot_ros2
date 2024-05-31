#include "ROS_API.h"

FRAPI_base::FRAPI_base(){
    _cmd_head = "/f/b";
    _cmd_tail = "/b/f";
    _cmd_interval = "III";
}

FRAPI_base::~FRAPI_base(){
    
}

std::string FRAPI_base::command_factry(std::string name, uint16_t counter, std::string data){
    std::string cmd_send;
    auto iter = glob_FR_cmd_id().find(name);
    std::string cmd_id;
    if(iter->second == 0){
        if(name == "GetInverseKin"){
            cmd_id = "377";
        }else if(name == "GetForwardKin"){
            cmd_id = "377";
        }
    }else{
        cmd_id = std::to_string(iter->second);//枚举类本质是int型数据，因此需要转换成字符串
    }
    std::string cmd_data = name + "(" + data + ")";//指令数据转成string
    std::string cmd_len = std::to_string(cmd_data.size());//size返回字符串长度，类型是uint，需要转换成字符串
    std::string cmd_counter = std::to_string(counter);
    cmd_send = _cmd_head + _cmd_interval + cmd_counter + _cmd_interval + cmd_id 
               + _cmd_interval + cmd_len + _cmd_interval + cmd_data + _cmd_interval + _cmd_tail;
    //std::cout << "factory: send_str is: " << cmd_send << std::endl;
    return cmd_send;    
}





ROS_API::ROS_API(const std::string node_name):FRAPI_base(),rclcpp::Node(node_name,
           rclcpp::NodeOptions().use_intra_process_comms(true))
{
    using namespace std::chrono_literals;
    _cmd_counter = 0;
    _recv_data_cmdcount = 0;
    _skip_answer_flag = 0;
    this->declare_parameter<uint8_t>("toolcoord_install",0);//默认工具安装在机器人末端
    this->declare_parameter<uint8_t>("toolcoord_type",0);//默认是工具坐标系
    this->declare_parameter<uint8_t>("collision_mode",0);//碰撞等级模式，默认是等级
    this->declare_parameter<uint8_t>("collision_config",1);//碰撞配置文件设置，默认不更新配置文件
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
    this->declare_parameter<float>("ServoJT_timeinterval",0.008);

    _recv_ros_command_server = this->create_service<frhal_msgs::srv::ROSCmdInterface>(
        "FR_ROS_API_service",
        std::bind(&ROS_API::_ParseROSCommandData_callback,this,std::placeholders::_1,std::placeholders::_2)
    );

    _controller_ip = "192.168.58.2";//controller ip address
    RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Start to create command TCP socket");
    _socketfd1 = socket(AF_INET,SOCK_STREAM,0);
    _socketfd2 = socket(AF_INET,SOCK_STREAM,0);
    if(_socketfd1 == -1 || _socketfd2 == -1){
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1001:Failed to create TCP socket");
        exit(0);//创建套字失败，丢出错误
    }else{
        RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"TCP socket created! Start to connect robot controller");
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
            exit(0);//连接失败，丢出错误并返回
        }else{
            RCLCPP_INFO(rclcpp::get_logger("fairino_hardware"),"Connected to robot controller!");
            int flags1 = fcntl(_socketfd1,F_GETFL,0);
            int flags2 = fcntl(_socketfd2,F_GETFL,0);
            fcntl(_socketfd1,F_SETFL,flags1|SOCK_NONBLOCK);
            fcntl(_socketfd2,F_SETFL,flags2|SOCK_NONBLOCK);//将两个socket设置成非阻塞模式
        }
    }
}

ROS_API::~ROS_API(){

}

void ROS_API::_selectfunc(std::string func_name){
    if(func_name == "JNTPoint"){
        funcP = &ROS_API::_def_jnt_position;
    }else if(func_name == "CARTPoint"){
        funcP = &ROS_API::_def_cart_position;
    }else if(func_name == "DragTeachSwitch"){
        funcP = &ROS_API::DragTeachSwitch;
    }else if(func_name == "RobotEnable"){
        funcP = &ROS_API::RobotEnable;
    }else if(func_name == "SetSpeed"){
        funcP = &ROS_API::SetSpeed;
    }else if(func_name == "Mode"){
        funcP = &ROS_API::Mode;
    }else if(func_name == "SetToolCoord"){
        funcP = &ROS_API::SetToolCoord;
    }else if(func_name == "SetToolList"){
        funcP = &ROS_API::SetToolList;
    }else if(func_name == "SetExToolCoord"){
        funcP = &ROS_API::SetExToolCoord;
    }else if(func_name == "SetExToolList"){
        funcP = &ROS_API::SetExToolList;
    }else if(func_name == "SetWObjCoord"){
        funcP = &ROS_API::SetWObjCoord;
    }else if(func_name == "SetWObjList"){
        funcP = &ROS_API::SetWObjList;
    }else if(func_name == "SetLoadWeight"){
        funcP = &ROS_API::SetLoadWeight;
    }else if(func_name == "SetLoadCoord"){
        funcP = &ROS_API::SetLoadCoord;
    }else if(func_name == "SetRobotInstallPos"){
        funcP = &ROS_API::SetRobotInstallPos;
    }else if(func_name == "SetRobotInstallAngle"){
        funcP = &ROS_API::SetRobotInstallAngle;
    }else if(func_name == "SetAnticollision"){
        funcP = &ROS_API::SetAnticollision;
    }else if(func_name == "SetCollisionStrategy"){
        funcP = &ROS_API::SetCollisionStrategy;
    }else if(func_name == "SetLimitPositive"){
        funcP = &ROS_API::SetLimitPositive;
    }else if(func_name == "SetLimitNegative"){
        funcP = &ROS_API::SetLimitNegative;
    }else if(func_name == "ResetAllError"){
        funcP = &ROS_API::ResetAllError;
    }else if(func_name == "FrictionCompensationOnOff"){
        funcP = &ROS_API::FrictionCompensationOnOff;
    }else if(func_name == "SetFrictionValue_level"){
        funcP = &ROS_API::SetFrictionValue_level;
    }else if(func_name == "SetFrictionValue_wall"){
        funcP = &ROS_API::SetFrictionValue_wall;
    }else if(func_name == "SetFrictionValue_ceiling"){
        funcP = &ROS_API::SetFrictionValue_ceiling;
    }else if(func_name == "SetFrictionValue_freedom"){
        funcP = &ROS_API::SetFrictionValue_freedom;
    }else if(func_name == "ActGripper"){
        funcP = &ROS_API::ActGripper;
    }else if(func_name == "MoveGripper"){
        funcP = &ROS_API::MoveGripper;
    }else if(func_name == "SetDO"){
        funcP = &ROS_API::SetDO;
    }else if(func_name == "SetToolDO"){
        funcP = &ROS_API::SetToolDO;
    }else if(func_name == "SetAO"){
        funcP = &ROS_API::SetAO;
    }else if(func_name == "SetToolAO"){
        funcP = &ROS_API::SetToolAO;
    }else if(func_name == "StartJOG"){
        funcP = &ROS_API::StartJOG;
    }else if(func_name == "StopJOG"){
        funcP = &ROS_API::StopJOG;
    }else if(func_name == "ImmStopJOG"){
        funcP = &ROS_API::ImmStopJOG;
    }else if(func_name == "MoveJ"){
        funcP = &ROS_API::MoveJ;
    }else if(func_name == "MoveL"){
        funcP = &ROS_API::MoveL;
    }else if(func_name == "MoveC"){
        funcP = &ROS_API::MoveC;
    }else if(func_name == "ServoJTStart"){
        funcP = &ROS_API::ServoJTStart;
    }else if(func_name == "ServoJT"){
        funcP = &ROS_API::ServoJT;
    }else if(func_name == "ServoJTEnd"){
        funcP = &ROS_API::ServoJTEnd;
    }else if(func_name == "Circle"){
        funcP = &ROS_API::Circle;
    }else if(func_name == "NewSpiral"){
        //funcP = &ROS_API::NewSpiral;
    }else if(func_name == "SplineStart"){
        funcP = &ROS_API::SplineStart;
    }else if(func_name == "SplinePTP"){
        funcP = &ROS_API::SplinePTP;
    }else if(func_name == "SplineEnd"){
        funcP = &ROS_API::SplineEnd;
    }else if(func_name == "NewSplineStart"){
        funcP = &ROS_API::NewSplineStart;
    }else if(func_name == "NewSplinePoint"){
        funcP = &ROS_API::NewSplinePoint;
    }else if(func_name == "NewSplineEnd"){
        funcP = &ROS_API::NewSplineEnd;
    }else if(func_name == "StopMotion"){
        funcP = &ROS_API::StopMotion;
    }else if(func_name == "PointsOffsetEnable"){
        funcP = &ROS_API::PointsOffsetEnable;
    }else if(func_name == "PointsOffsetDisable"){
        funcP = &ROS_API::PointsOffsetDisable;
    }else if(func_name == "ProgramRun"){
        funcP = &ROS_API::ProgramRun;
    }else{
        funcP = NULL;
    }
}

int ROS_API::_send_data_factory_callback(std::string data){
    using namespace std::chrono_literals;
    static char recv_buff[128];
    send(_socketfd1,data.c_str(),data.size(),0);//发送指令信息
    memset(recv_buff,0,sizeof(recv_buff));
    if(!_skip_answer_flag) {//针对servoJT指令，不需要看反馈值直接发送
        rclcpp::sleep_for(30ms);
        if(recv(_socketfd1,recv_buff,sizeof(recv_buff),0) > -1){
            if(_ParseRecvData(std::string(recv_buff))){
                if(_recv_data_cmdid == 377){
                    return -2001;
                }else if(_recv_data_cmdcount == _cmd_counter){//id和帧计数器能对上，说明对应回复信息是正确的
                    return _recv_data_res;
                }else{
                    return 0;
                }
            }
        }
    }else{
        _skip_answer_flag = 0;
        return 0;
    }
}



int ROS_API::_ParseRecvData(std::string str){
    std::regex pattern("/f/bIII(\\d*?)III(\\d*?)III(\\d*)III(.*?)III/b/f");
    std::smatch data_match;
    if(std::regex_match(str,data_match,pattern)){//判断帧数据是否符合模式
        //_mtx.lock();
        _recv_data_cmdcount = std::atol(data_match[1].str().c_str());
        _recv_data_cmdid = std::atol(data_match[2].str().c_str());
        if(_recv_data_cmdid == 377){//运动学接口反馈回来的信息
            std::string kin_data = data_match[4];
            std::regex kin_pattern("(.*?),(.*?),(.*?),(.*?),(.*?),(.*?)");
            std::smatch kin_match;
            if(std::regex_match(kin_data,kin_match,kin_pattern)){
                for(int i=0;i<6;i++){
                    _kin_res[i] = atof(kin_match[i+1].str().c_str());
                }
            }else{
                RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1011: Kinematic computation failed, please check position data in motion command.");
                return 0;
            }
            return 1;
        }else{//一般控制指令的反馈信息,单一int型
            _recv_data_res = atoi(data_match[4].str().c_str());
            return 1;
        }
        //_mtx.unlock();
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1012: The frame is not completed, give up this frame.");
        return 0;
    }
}


void ROS_API::_ParseROSCommandData_callback(const std::shared_ptr<frhal_msgs::srv::ROSCmdInterface::Request> req,\ 
                                            std::shared_ptr<frhal_msgs::srv::ROSCmdInterface::Response> res)//用于解析用户发送的ROS接口指令
{//指令格式为movj(1,10)
    std::regex func_reg("([A-Z|a-z|_]+)[(](.*)[)]");//函数名的输入模式应该是字母函数名后跟()，圆括号中有所有输入参数
    std::smatch func_match;
    if(std::regex_match(req->cmd_str,func_match,func_reg)){
        //std::cout << "收到ROS指令: " <<  req->cmd_str.data() <<std::endl;
        std::string func_name = func_match[1];
        std::string para_list = func_match[2];
        //std::regex para_pattern("[A-Z|a-z|\\.\\d|\\d|,|-]*");//校验参数的内容，参数部分必须是字母，数字和逗号，负号组成，出现其他字符包括空格都会导致校验失败,
        std::regex para_pattern("[A-Za-z\\d.\\-,]*");//校验参数的内容，参数部分必须是字母，数字和逗号，负号组成，出现其他字符包括空格都会导致校验失败,
        if(std::regex_match(para_list,para_pattern)){//检查参数输入是否合法
            if(func_name == "GET"){
                res->cmd_res = _get_variable(para_list);
            }else{
                _selectfunc(func_name);
                if(funcP == NULL){
                    RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1021: Can not find corresponding command.");
                    res->cmd_res = std::string("0");
                }else{
                    res->cmd_res = std::to_string((this->*(ROS_API::funcP))(para_list));
                }
            }
        }else{
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1022: Illegal command parameters, parameters must consist of alphabet, number and dot,space is not allowed.");
            res->cmd_res = std::string("0");
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1023: Illegal command function format, the function name must fllow fomart [function name](parameters).");
        res->cmd_res = std::string("0");
    }
}

int ROS_API::_def_jnt_position(std::string pos){
    //std::regex pattern("[-|\\.|,|\\d]*"); //参数模式应该是所有参数都是数字和逗号
    std::regex pattern("[\\d.\\-,]*"); //参数模式应该是所有参数都是数字和逗号
    std::smatch para_match;
    if(std::regex_match(pos,para_match,pattern)){//进行参数正确性判断
        std::smatch data_match;
        std::regex search_para(",");//分隔符
        std::regex_token_iterator iter_data(pos.begin(),pos.end(),search_para,-1);
        decltype(iter_data) end;
        int count = 0;
        for(;iter_data != end;iter_data++){
            count++;
        }
        if(count != 7){
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1031: JNTPoint command only take 7 parameters, please check number of parameters.");
            return 0;
        }
        iter_data = std::regex_token_iterator(pos.begin(),pos.end(),search_para,-1);
        int idx = atol(iter_data->str().c_str());//指令序号
        iter_data++;
        if(idx > _cmd_jnt_pos_list.size()+1 || idx <= 0){//如果大于当前容器最大值+1,那么要报错，因为序列容器中间无法留空
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1032: Point index is out of range.");
            return 0;
        }else if(idx <= _cmd_jnt_pos_list.size()){//如果是小于等于当前的容量，那么就是点位信息覆盖
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
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1033: The role of parameter: the first para. is point index, the others is joint position values, split by comma, space is not allowed.");
        return 0;
    }
    return 1;
}

int ROS_API::_def_cart_position(std::string pos){
    //std::regex pattern("[\\.\\d|\\d|,|-]*"); //参数模式应该是所有参数都是数字
    std::regex pattern("[\\d.\\-,]*"); //参数模式应该是所有参数都是数字
    std::smatch para_match;
    if(std::regex_match(pos,para_match,pattern)){
        std::smatch data_match;
        std::regex search_para(",");//分隔符
        std::regex_token_iterator iter_data(pos.begin(),pos.end(),search_para,-1);
        decltype(iter_data) end;
        int count = 0;
        for(;iter_data != end;iter_data++){
            count++;
        }
        if(count != 7){
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1041: Point index is out of range.");
            return 0;
        }
        iter_data = std::regex_token_iterator(pos.begin(),pos.end(),search_para,-1);
        int idx = atol(iter_data->str().c_str());//指令序号
        iter_data++;
        if(idx > _cmd_cart_pos_list.size()+1 || idx <= 0){//如果大于当前容器最大值+1,那么要报错，因为序列容器中间无法留空
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1042: Point index is out of range.");
            return 0;
        }else if(idx <= _cmd_cart_pos_list.size()){//如果是小于等于当前的容量，那么就是点位信息覆盖
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
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1043: The role of parameter: the first para. is point index, the others is cartesian position values, split by comma, space is not allowed.");
        return 0;
    }
    return 1;
}

std::string ROS_API::_get_variable(std::string para_list){
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
                RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1051: Point index is out of range.");
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
                    RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1052: Point index is out of range.");
                }
        }else{
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1053: invalid parameter.");
            //std::cout << "指令错误: 无效的GET指令参数" << std::endl;
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1054: Illegal command function format, the function name must fllow fomart [function name](parameters).");
        //std::cout << "指令错误: GET指令参数非法,参数形式为[JNT|CART],[序号]" << std::endl;
    }
}

int  ROS_API::DragTeachSwitch(std::string para){//拖动示教模式切换
    return _send_data_factory_callback(FRAPI_base::command_factry("DragTeachSwitch",++_cmd_counter,para));
}

int  ROS_API::RobotEnable(std::string para){//机械臂使能
    return _send_data_factory_callback(FRAPI_base::command_factry("RobotEnable",++_cmd_counter,para));
}

int ROS_API::Mode(std::string para){//手动模式，自动模式切换
    return _send_data_factory_callback(FRAPI_base::command_factry("Mode",++_cmd_counter,para));
}

int ROS_API::SetSpeed(std::string para){
    return _send_data_factory_callback(FRAPI_base::command_factry("SetSpeed",++_cmd_counter,para));
}

int ROS_API::SetToolCoord(std::string para){
    //int id, DescPose *coord, int type, int install
    std::string install,type;
    install = this->get_parameter("toolcoord_install").value_to_string();
    type = this->get_parameter("toolcoord_type").value_to_string();
    para = para + "," + type + "," + install;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetToolCoord",++_cmd_counter,para));
}

int ROS_API::SetToolList(std::string para){
    //int id, DescPose *coord, int type, int install
    std::string install,type;
    install = this->get_parameter("toolcoord_install").value_to_string();
    type = this->get_parameter("toolcoord_type").value_to_string();
    para = para + "," + type  + "," + install;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetToolList",++_cmd_counter,para));
}

int ROS_API::SetExToolCoord(std::string para){
    //int id, DescPose *etcp, DescPose *etool
    return _send_data_factory_callback(FRAPI_base::command_factry("SetExToolCoord",++_cmd_counter,para));
}

int ROS_API::SetExToolList(std::string para){
    //int id, DescPose *etcp, DescPose *etool
    return _send_data_factory_callback(FRAPI_base::command_factry("SetExToolList",++_cmd_counter,para));
}

int ROS_API::SetWObjCoord(std::string para){
    //int id, DescPose *coord
    return _send_data_factory_callback(FRAPI_base::command_factry("SetWObjCoord",++_cmd_counter,para));
}

int ROS_API::SetWObjList(std::string para){
    //int id, DescPose *coord
    return _send_data_factory_callback(FRAPI_base::command_factry("SetWObjList",++_cmd_counter,para));
}

int ROS_API::SetLoadWeight(std::string para){
    //float weight
    return _send_data_factory_callback(FRAPI_base::command_factry("SetLoadWeight",++_cmd_counter,para));
}
    
int ROS_API::SetLoadCoord(std::string para){
    //DescTran *coord
    return _send_data_factory_callback(FRAPI_base::command_factry("SetLoadCoord",++_cmd_counter,para));
}

int ROS_API::SetRobotInstallPos(std::string para){
    //uint8_t install
    return _send_data_factory_callback(FRAPI_base::command_factry("SetRobotInstallPos",++_cmd_counter,para));
}

int ROS_API::SetRobotInstallAngle(std::string para){
    //double yangle, double zangle
    return _send_data_factory_callback(FRAPI_base::command_factry("SetRobotInstallAngle",++_cmd_counter,para));
}

//安全配置
int ROS_API::SetAnticollision(std::string para){
    //int mode, float level[6], int config
    std::string mode,config;
    mode = this->get_parameter("collision_mode").value_to_string();
    config = this->get_parameter("collision_config").value_to_string();
    para = mode + "," + "{" + para + "}" + "," + config;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetAnticollision",++_cmd_counter,para));
}

int ROS_API::SetCollisionStrategy(std::string para){
    //int strategy
    return _send_data_factory_callback(FRAPI_base::command_factry("SetCollisionStrategy",++_cmd_counter,para));
}

int ROS_API::SetLimitPositive(std::string para){
    //float limit[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetLimitPositive",++_cmd_counter,para));
}

int ROS_API::SetLimitNegative(std::string para){
    //float limit[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetLimitNegative",++_cmd_counter,para));
}

int ROS_API::ResetAllError(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("ResetAllError",++_cmd_counter,para));
}

int ROS_API::FrictionCompensationOnOff(std::string para){
    //uint8_t state
    return _send_data_factory_callback(FRAPI_base::command_factry("FrictionCompensationOnOff",++_cmd_counter,para));
}

int ROS_API::SetFrictionValue_level(std::string para){
    //float coeff[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetFrictionValue_level",++_cmd_counter,para));
}

int ROS_API::SetFrictionValue_wall(std::string para){
    //float coeff[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetFrictionValue_wall",++_cmd_counter,para));
}

int ROS_API::SetFrictionValue_ceiling(std::string para){
    //float coeff[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetFrictionValue_ceiling",++_cmd_counter,para));
}

int ROS_API::SetFrictionValue_freedom(std::string para){
    //float coeff[6]
    return _send_data_factory_callback(FRAPI_base::command_factry("SetFrictionValue_freedom",++_cmd_counter,para));
}

//外设控制
int ROS_API::ActGripper(std::string para){
    //int index, uint8_t act
    return _send_data_factory_callback(FRAPI_base::command_factry("ActGripper",++_cmd_counter,para));
}

int ROS_API::MoveGripper(std::string para){
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
int ROS_API::SetDO(std::string para){
    //int id, uint8_t status, uint8_t smooth, uint8_t block
    std::string smooth,block;
    smooth = this->get_parameter("DO_smooth").value_to_string();
    block = this->get_parameter("DO_block").value_to_string();
    para = para + "," + smooth + "," + block;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetDO",++_cmd_counter,para));
}

int ROS_API::SetToolDO(std::string para){
    //int id, uint8_t status, uint8_t smooth, uint8_t block
    std::string smooth,block;
    smooth = this->get_parameter("DO_smooth").value_to_string();
    block = this->get_parameter("DO_block").value_to_string();
    para = para + "," + smooth + "," + block;
    return _send_data_factory_callback(FRAPI_base::command_factry("SetToolDO",++_cmd_counter,para));
}

int ROS_API::SetAO(std::string para){
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

int ROS_API::SetToolAO(std::string para){
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

//运动指令
int ROS_API::StartJOG(std::string para){
    //uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis
    std::string acc,max_dis;
    acc = this->get_parameter("JOG_acc").value_to_string();
    max_dis = this->get_parameter("JOG_maxdis").value_to_string();
    para = para + "," + acc + "," + max_dis;
    return _send_data_factory_callback(FRAPI_base::command_factry("StartJOG",++_cmd_counter,para));
}

int ROS_API::StopJOG(std::string para){
    //uint8_t ref
    return _send_data_factory_callback(FRAPI_base::command_factry("StopJOG",++_cmd_counter,para));
}

int ROS_API::ImmStopJOG(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("ImmStopJOG",++_cmd_counter,para));
}

int ROS_API::MoveJ(std::string para){
    //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos
    std::string tool,user,acc,ovl,eaxis1,eaxis2,eaxis3,eaxis4,blendT,offset_flag,offset_pos_x,\
                offset_pos_y,offset_pos_z,offset_pos_rx,offset_pos_ry,offset_pos_rz;
    tool = this->get_parameter("MoveJLC_tool").value_to_string();
    user = this->get_parameter("MoveJLC_user").value_to_string();
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

    std::regex search_para(",");
    std::regex_token_iterator iter_data(para.begin(),para.end(),search_para,-1);
    std::smatch num_match;
    std::string head_str = iter_data->str();
    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)"))){//第一个元素是否满足JNT1这种模式
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size()){
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1061: MoveJ command point index out of range.");
            //std::cout << "指令错误:MoveJ输入位置点序号超限" << std::endl;
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
        //std::cout << "send kin req data: " << para2; 
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//求解正向运动学
        if(res == -2001){
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
            if(blendT == "-1"){
                
            }
            return _send_data_factory_callback(FRAPI_base::command_factry("MoveJ",++_cmd_counter,para));
        }else{
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1062: Forward kinematic computation error.");
            //std::cout << "指令错误:MoveJ指令调用正向运动学发生错误" << std::endl;
            return 0;
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1063: MoveJ command point index out of range.");
            //std::cout << "指令错误:MoveJ输入位置点序号超限" << std::endl;
            return 0;
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
        //std::cout << "send kin req data: " << para2; 
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//求解逆向运动学
        if(res == -2001){
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
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1064: Inverse kinematic computation error.");
            //std::cout << "指令错误:MoveJ指令调用逆向运动学发生错误" << std::endl;
            return 0;
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1065: Illegal MoveJ command parameters,valid point index");
        //std::cout << "指令错误:MoveJ参数输入非法,没有找到点位信息" << std::endl;
        return 0;
    }

}
    
int ROS_API::MoveL(std::string para){
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
    std::regex_token_iterator iter_data(para.begin(),para.end(),search_para,-1);
    std::smatch num_match;
    std::string head_str = iter_data->str();
    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)"))){   
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size()){
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1071: MoveL command point index out of range.");
            //std::cout << "指令错误:MoveL输入位置点序号超限" << std::endl;
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
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//求解正向运动学
        if(res == -2001){
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
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1072: Forward kinematic computation error.");
            //std::cout << "指令错误:MoveL指令调用正向运动学发生错误" << std::endl;
            return 0;
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1073: Point index out of range.");
            //std::cout << "指令错误:MoveL输入位置点序号超限" << std::endl;
            return 0;
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
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//求解逆向运动学
        if(res == -2001){
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
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1074: Inverse kinematic computation error.");
            //std::cout << "指令错误:MoveL指令调用逆向运动学发生错误" << std::endl;
            return 0;
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1075: Illegal MoveL command parameters,valid point index");
        //std::cout << "指令错误:MoveL参数输入非法" << std::endl;
        return 0;
    }

}

int ROS_API::MoveC(std::string para){
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
    std::regex_token_iterator iter_data(para.begin(),para.end(),search_para,-1);
    std::smatch num_match,num_match2;
    std::string head_str = iter_data->str();//第一个点位
    iter_data++;
    std::string second_str = iter_data->str();//第二个点位
    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)")) && std::regex_match(second_str,num_match2,std::regex("(JNT)([0-9]*)"))){   
        int index = atol(num_match[2].str().c_str());
        int index2 = atol(num_match2[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size() || index2 > _cmd_jnt_pos_list.size()){
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1081: MoveC command point index out of range.");
            //std::cout << "指令错误:MoveC输入位置点序号超限" << std::endl;
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
        int res1 = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//求解正向运动学
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
        int res2 = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//求解正向运动学
        if(res1 == -2001 && res2 == -2001){
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
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1082: Forward kinematic computation error.");
            //std::cout << "指令错误:MoveC指令调用正向运动学发生错误" << std::endl;
            return 0;
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)")) && std::regex_match(second_str,num_match2,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        int index2 = atol(num_match2[2].str().c_str());
        if(index > _cmd_cart_pos_list.size() || index2 > _cmd_cart_pos_list.size()){
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1083: Point index out of range.");
            //std::cout << "指令错误:MoveC输入位置点序号超限" << std::endl;
            return 0;
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
        int res1 = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//求解逆向运动学
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
        int res2 = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//求解逆向运动学
        if(res1 == -2001 && res2 == -2001){
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
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1084: Inverse kinematic computation error.");
            //std::cout << "指令错误:MoveC指令调用逆向运动学发生错误" << std::endl;
            return 0;
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1085: Illegal MoveL command parameters,valid point index");
        //std::cout << "指令错误:MoveC参数输入非法" << std::endl;
        return 0;
    }
}

int ROS_API::Circle(std::string para){
    //JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, float ovl, uint8_t offset_flag, DescPose *offset_pos
    return -1;
}

// int ROS_API::NewSpiral(std::string para){
//     //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, ExaxisPos *epos, float ovl, uint8_t offset_flag, DescPose *offset_pos, SpiralParam spiral_param
//     return -1;
// }

int ROS_API::ServoJTStart(std::string para){
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("ServoJTStart",++_cmd_counter,para));
}


int ROS_API::ServoJT(std::string para){
    //{double tor1-tor6}, double interval
    std::string time_interval = this->get_parameter("ServoJT_timeinterval").value_to_string();
    para = "{" + para + "}," + "0.008"; 
    _skip_answer_flag = 1;
    return _send_data_factory_callback(FRAPI_base::command_factry("ServoJT",++_cmd_counter,para));
}


int ROS_API::ServoJTEnd(std::string para){
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("ServoJTEnd",++_cmd_counter,para));
}



int ROS_API::SplineStart(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("SplineStart",++_cmd_counter,para));
}

int ROS_API::SplinePTP(std::string para){
    //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl
    //默认进来的都是JNT数据
    std::regex search_para(",");
    std::regex_token_iterator iter_data(para.begin(),para.end(),search_para,-1);
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
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1091: Point out of index.");
            //std::cout << "指令错误:SplinePTP输入位置点序号超限" << std::endl;
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
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//求解正向运动学
        if(res == -2001){
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
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1092: Forward kinematic computation error.");
            //std::cout << "指令错误:Spline指令调用正向运动学发生错误" << std::endl;
            return 0;
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1093: Illegal parameters.");
        //std::cout << "指令错误:Spline参数输入非法" << std::endl;
        return 0;
    }
}

int ROS_API::SplineEnd(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("SplineEnd",++_cmd_counter,para));
}

int ROS_API::NewSplineStart(std::string para){
    //uint8_t ctlPoint
    return _send_data_factory_callback(FRAPI_base::command_factry("NewSplineStart",++_cmd_counter,para));
}

int ROS_API::NewSplinePoint(std::string para){
    //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl float blendR uint8_t lastFlag
    //输入参数:pos,speed,lastflag
    std::regex search_para(",");
    std::regex_token_iterator iter_data(para.begin(),para.end(),search_para,-1);
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
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1101: Point out of index.");
            //std::cout << "指令错误:NewSplinePoint输入位置点序号超限" << std::endl;
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
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//求解逆向运动学
        if(res == -2001){
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
            RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1102: Inverse kinematic computation error.");
            //std::cout << "指令错误:NewSplinePoint指令调用逆向运动学发生错误" << std::endl;
            return 0;
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("fairino_hardware"),"-1103: Illegal parameters.");
        //std::cout << "指令错误:NewSplinePoint参数输入非法" << std::endl;
        return 0;
    }
}

int ROS_API::NewSplineEnd(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("NewSplineEnd",++_cmd_counter,para));
}

int ROS_API::StopMotion(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("StopMotion",++_cmd_counter,para));
}

int ROS_API::PointsOffsetEnable(std::string para){
    //int flag, DescPose *offset_pos
    return _send_data_factory_callback(FRAPI_base::command_factry("PointsOffsetEnable",++_cmd_counter,para));
}

int ROS_API::PointsOffsetDisable(std::string para){
    //empty para
    return _send_data_factory_callback(FRAPI_base::command_factry("PointsOffsetDisable",++_cmd_counter,para));
}


int ROS_API::ProgramRun(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("ProgramRun",++_cmd_counter,para));
}