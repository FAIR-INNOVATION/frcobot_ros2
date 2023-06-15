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
        cmd_id = std::to_string(iter->second);//The enumeration class is essentially int type data, so it needs to be converted into a string
    }
    std::string cmd_data = name + "(" + data + ")";//Command data into string
    std::string cmd_len = std::to_string(cmd_data.size());//Size returns the length of the string, the type is uint, which needs to be converted into a string
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
    _retry_count = 10;//Initialization command feedback receiving retry times
    _recv_data_cmdcount = 0;
    this->declare_parameter<uint8_t>("toolcoord_install",0);//The default tool is installed at the end of the robot
    this->declare_parameter<uint8_t>("toolcoord_type",0);//The default is the tool coordinate system
    this->declare_parameter<uint8_t>("collision_mode",0);//Collision level mode, default is level
    this->declare_parameter<uint8_t>("collision_config",1);//Collision configuration file settings, the configuration file is not updated by default
    this->declare_parameter<uint8_t>("gripper_vel",50);
    this->declare_parameter<uint8_t>("gripper_force",50);
    this->declare_parameter<uint8_t>("gripper_maxtime",30000);
    this->declare_parameter<uint8_t>("gripper_block",1);//Default gripper control non-blocking
    this->declare_parameter<uint8_t>("DO_smooth",0);//Default DO is not smooth
    this->declare_parameter<uint8_t>("DO_block",1);//By default DO is non-blocking
    this->declare_parameter<uint8_t>("AO_block",1);//By default AO is non-blocking
    this->declare_parameter<uint8_t>("JOG_acc",40);//JOG default acceleration 40
    this->declare_parameter<int>("JOG_maxdis",5);//JOG default single step 5mm
    this->declare_parameter<int>("MoveJLC_tool",1);
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

    _recv_ros_command_server = this->create_service<frhal_msgs::srv::ROSCmdInterface>(
        "FR_ROS_API_service",
        std::bind(&ROS_API::_ParseROSCommandData_callback,this,std::placeholders::_1,std::placeholders::_2)
    );

    _controller_ip = "192.168.58.2";//Controller default ip address
    std::cout << "Start creating instruction TCP socket" << std::endl;
    _socketfd1 = socket(AF_INET,SOCK_STREAM,0);
    _socketfd2 = socket(AF_INET,SOCK_STREAM,0);
    if(_socketfd1 == -1 || _socketfd2 == -1){
        std::cout << "Error: Failed to create socket!" << std::endl;
        exit(0);//Failed to create socket, throwing error
    }else{
        std::cout << "The instruction socket is created successfully, and the connection to the controller begins..." << std::endl;
        struct sockaddr_in tcp_client1,tcp_client2;
        tcp_client1.sin_family = AF_INET;
        tcp_client2.sin_family = AF_INET;
        tcp_client1.sin_port = htons(port1);//Port 8080
        tcp_client2.sin_port = htons(port2);//Port 8082
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());
        tcp_client2.sin_addr.s_addr = inet_addr(_controller_ip.c_str());

        //Try to connect to the controller
        int res1 = connect(_socketfd1,(struct sockaddr *)&tcp_client1,sizeof(tcp_client1));
        int res2 = connect(_socketfd2,(struct sockaddr *)&tcp_client2,sizeof(tcp_client2));
        if(res1 || res2){
            std::cout << "Error: Unable to connect to controller data port, program exited!" << std::endl;
            exit(0);//Connection fails, throws an error and returns
        }else{
            std::cout << "The controller command port connection is successful" << std::endl;
            int flags1 = fcntl(_socketfd1,F_GETFL,0);
            int flags2 = fcntl(_socketfd2,F_GETFL,0);
            fcntl(_socketfd1,F_SETFL,flags1|SOCK_NONBLOCK);
            fcntl(_socketfd2,F_SETFL,flags2|SOCK_NONBLOCK);//Set the two sockets to non-blocking mode
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
    }else{
        funcP = NULL;
    }
}

int ROS_API::_send_data_factory_callback(std::string data){
    using namespace std::chrono_literals;
    static char recv_buff[128];
    int cnt = 0;
    //std::cout << "Send command information..." << data << std::endl;
    send(_socketfd1,data.c_str(),data.size(),0);//Send command information
    //std::cout << "Send command successfully!" << std::endl;
    memset(recv_buff,0,sizeof(recv_buff));
    //while(cnt < _retry_count){
        rclcpp::sleep_for(30ms);
        if(recv(_socketfd1,recv_buff,sizeof(recv_buff),0) > -1){
            //std::cout << "Reply message received..." << std::string(recv_buff) << std::endl;
            if(_ParseRecvData(std::string(recv_buff))){
                if(_recv_data_cmdid == 377){
                    //std::cout << "Kinematics Interface Return Parameters" << std::endl;
                    return -2001;
                }else if(_recv_data_cmdcount == _cmd_counter){//The id and the frame counter can be matched, indicating that the corresponding reply information is correct
                    //std::cout << "Set function interface return parameters" << std::endl;
                    return _recv_data_res;
                }else{
                    //std::cout << "unhandled case, return by default 0" << std::endl;
                    return 0;
                }
            }
        }
        cnt++;
    //}
    //std::cout << "Accept command feedback timeout" << std::endl;
}



int ROS_API::_ParseRecvData(std::string str){
    std::regex pattern("/f/bIII(\\d*?)III(\\d*?)III(\\d*)III(.*?)III/b/f");
    std::smatch data_match;
    if(std::regex_match(str,data_match,pattern)){//Determine whether the frame data conforms to the pattern
        //_mtx.lock();
        _recv_data_cmdcount = std::atol(data_match[1].str().c_str());
        _recv_data_cmdid = std::atol(data_match[2].str().c_str());
        if(_recv_data_cmdid == 377){//The information fed back by the kinematics interface
            std::string kin_data = data_match[4];
            std::regex kin_pattern("(.*?),(.*?),(.*?),(.*?),(.*?),(.*?)");
            std::smatch kin_match;
            if(std::regex_match(kin_data,kin_match,kin_pattern)){
                for(int i=0;i<6;i++){
                    _kin_res[i] = atof(kin_match[i+1].str().c_str());
                }
            }else{
                std::cout << "Parsing error: The kinematics interface returned incorrect information" << std::endl;
                return 0;
            }
            return 1;
        }else{//Feedback information of general control commands, single int type
            _recv_data_res = atol(data_match[4].str().c_str());
            return 1;
        }
        //_mtx.unlock();
    }else{
        std::cout << "Parsing error: the received communication information is incomplete, discard the frame content" << std::endl;
        return 0;
    }
}


void ROS_API::_ParseROSCommandData_callback(const std::shared_ptr<frhal_msgs::srv::ROSCmdInterface::Request> req,\ 
                                            std::shared_ptr<frhal_msgs::srv::ROSCmdInterface::Response> res)//Used to parse the ROS interface instructions sent by the user
{//The command format is movj(1,10)
    std::regex func_reg("([A-Z|a-z|_]+)[(](.*)[)]");//The input pattern for a function name should be an alphabetic function name followed by (), with all input parameters in parentheses
    std::smatch func_match;
    if(std::regex_match(req->cmd_str,func_match,func_reg)){
        //std::cout << "Receive ROS command: " <<  req->cmd_str.data() <<std::endl;
        std::string func_name = func_match[1];
        std::string para_list = func_match[2];
        //std::regex para_pattern("[A-Z|a-z|\\.\\d|\\d|,|-]*");//Verify the content of the parameter. The parameter part must be composed of letters, numbers, commas, and negative signs. Other characters including spaces will cause the verification to fail.
        std::regex para_pattern("[A-Za-z\\d.\\-,]*");//Verify the content of the parameter. The parameter part must be composed of letters, numbers, commas, and negative signs. Other characters including spaces will cause the verification to fail.
        if(std::regex_match(para_list,para_pattern)){//Check whether the parameter input is legal
            if(func_name == "GET"){
                res->cmd_res = _get_variable(para_list);
            }else{
                _selectfunc(func_name);
                if(funcP == NULL){
                    std::cout << "Instruction error: The function corresponding to the instruction cannot be found" << std::endl;
                    res->cmd_res = std::string("0");
                }else{
                    res->cmd_res = std::to_string((this->*(ROS_API::funcP))(para_list));
                }
            }
        }else{
            std::cout << "Instruction error: function parameter input is illegal, the parameter list consists of letters, numbers and commas, no spaces can appear" <<std::endl;
            res->cmd_res = std::string("0");
        }
    }else{
        std::cout << "Instruction error: the function input form is wrong, the function input must be the input form of [function name](), please re-enter" << std::endl;
        res->cmd_res = std::string("0");
    }
}

int ROS_API::_def_jnt_position(std::string pos){
    //std::regex pattern("[-|\\.|,|\\d]*"); //The parameter pattern should be all parameters are numbers and commas
    std::regex pattern("[\\d.\\-,]*"); //The parameter pattern should be all parameters are numbers and commas
    std::smatch para_match;
    if(std::regex_match(pos,para_match,pattern)){//To judge the correctness of parameters
        std::smatch data_match;
        std::regex search_para(",");//Delimiter
        std::regex_token_iterator iter_data(pos.begin(),pos.end(),search_para,-1);
        decltype(iter_data) end;
        int count = 0;
        for(;iter_data != end;iter_data++){
            count++;
        }
        if(count != 7){
            std::cout << "Command error: there are 6 joint position parameters, please confirm the number of parameter input" << std::endl;
            return 0;
        }
        iter_data = std::regex_token_iterator(pos.begin(),pos.end(),search_para,-1);
        int idx = atol(iter_data->str().c_str());//Command number
        iter_data++;
        if(idx > _cmd_jnt_pos_list.size()+1 || idx <= 0){//If it is greater than the maximum value of the current container + 1, then an error will be reported, because the middle of the sequence container cannot be left blank
            std::cout << "Instruction error: container serial number exceeded" << std::endl;
            return 0;
        }else if(idx <= _cmd_jnt_pos_list.size()){//If it is less than or equal to the current capacity, then it is point information coverage
            int i=0;
            for(;iter_data != end;iter_data++,i++){
                _cmd_jnt_pos_list.at(idx-1).jPos[i] = atof(iter_data->str().c_str());
            }
        }else if(idx == _cmd_jnt_pos_list.size()+1){//Add element
            JointPos pos;
            int i=0;
            for(;iter_data != end;iter_data++,i++){
                pos.jPos[i] = atof(iter_data->str().c_str());
            }
            _cmd_jnt_pos_list.push_back(pos);
        }
    }else{
        std::cout << "Instruction error: The rule of joint point input parameters is that the first one is the storage number, followed by the joint position information, separated by commas, and no spaces can appear" << std::endl;
        return 0;
    }
    return 1;
}

int ROS_API::_def_cart_position(std::string pos){
    //std::regex pattern("[\\.\\d|\\d|,|-]*"); //The parameter pattern should be all parameters are numbers
    std::regex pattern("[\\d.\\-,]*"); //The parameter pattern should be all parameters are numbers
    std::smatch para_match;
    if(std::regex_match(pos,para_match,pattern)){
        std::smatch data_match;
        std::regex search_para(",");//Delimiter
        std::regex_token_iterator iter_data(pos.begin(),pos.end(),search_para,-1);
        decltype(iter_data) end;
        int count = 0;
        for(;iter_data != end;iter_data++){
            count++;
        }
        if(count != 7){
            std::cout << "Instruction error: there are 6 Cartesian positional parameters, please confirm the number of parameter input" << std::endl;
            return 0;
        }
        iter_data = std::regex_token_iterator(pos.begin(),pos.end(),search_para,-1);
        int idx = atol(iter_data->str().c_str());//Command number
        iter_data++;
        if(idx > _cmd_cart_pos_list.size()+1 || idx <= 0){//If it is greater than the maximum value of the current container + 1, then an error will be reported, because the middle of the sequence container cannot be left blank
            std::cout << "Instruction error: container serial number exceeded" << std::endl;
            return 0;
        }else if(idx <= _cmd_cart_pos_list.size()){//If it is less than or equal to the current capacity, then it is point information coverage
            _cmd_cart_pos_list.at(idx-1).tran.x = atof(iter_data->str().c_str());iter_data++;
            _cmd_cart_pos_list.at(idx-1).tran.y = atof(iter_data->str().c_str());iter_data++;
            _cmd_cart_pos_list.at(idx-1).tran.z = atof(iter_data->str().c_str());iter_data++;
            _cmd_cart_pos_list.at(idx-1).rpy.rx = atof(iter_data->str().c_str());iter_data++;
            _cmd_cart_pos_list.at(idx-1).rpy.ry = atof(iter_data->str().c_str());iter_data++;
            _cmd_cart_pos_list.at(idx-1).rpy.rz = atof(iter_data->str().c_str());
        }else if(idx == _cmd_cart_pos_list.size()+1){//Add element
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
        std::cout << "Instruction error: Cartesian point input parameter rule is that the first is the storage number, followed by Cartesian position information, separated by commas, no spaces can appear" << std::endl;
        return 0;
    }
    return 1;
}

std::string ROS_API::_get_variable(std::string para_list){
    std::regex pattern("([A-Z]*),([0-9]*)");//The parameter mode should be in the form of JNT,number or CART,number
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
                std::cout << "Command error: The sequence number of the input point is out of range" << std::endl;
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
                    std::cout << "Command error: The sequence number of the input point is out of range" << std::endl;
                }
        }else{
            std::cout << "Command Error: Invalid GET command parameter" << std::endl;
        }
    }else{
        std::cout << "Command error: GET command parameter is illegal, the parameter format is [JNT|CART], [serial number]" << std::endl;
    }
}

int  ROS_API::DragTeachSwitch(std::string para){//Drag to switch teaching mode
    return _send_data_factory_callback(FRAPI_base::command_factry("DragTeachSwitch",++_cmd_counter,para));
}

int  ROS_API::RobotEnable(std::string para){//Arm enable
    return _send_data_factory_callback(FRAPI_base::command_factry("RobotEnable",++_cmd_counter,para));
}

int ROS_API::Mode(std::string para){//Manual mode, automatic mode switching
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

//Security configuration
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

//Peripheral control
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
    
//IO Control
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

//Motion command
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
    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)"))){//Whether the first element satisfies the JNT1 pattern
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size()){
            std::cout << "Instruction error: MoveJ input position number exceeds the limit" << std::endl;
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
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//Solving Forward Kinematics
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
            // std::cout << "MoveJ send data: " << tmp_para << std::endl;
            return _send_data_factory_callback(FRAPI_base::command_factry("MoveJ",++_cmd_counter,para));
        }else{
            std::cout << "Instruction error: An error occurred when the MoveJ instruction invoked forward kinematics" << std::endl;
            return 0;
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            std::cout << "Instruction error: MoveJ input position number exceeds the limit" << std::endl;
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
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//Solving Inverse Kinematics
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
            std::cout << "Instruction error: An error occurred when the MoveJ instruction invoked inverse kinematics" << std::endl;
            return 0;
        }
    }else{
        std::cout << "Instruction error: MoveJ parameter input is illegal, no point information found" << std::endl;
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
            std::cout << "Instruction error: MoveL input position number exceeds the limit" << std::endl;
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
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//Solving Forward Kinematics
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
            std::cout << "Instruction error: An error occurred when the MoveL instruction invoked forward kinematics" << std::endl;
            return 0;
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            std::cout << "Instruction error: MoveL input position number exceeds the limit" << std::endl;
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
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//Solving Inverse Kinematics
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
            std::cout << "Instruction error: An error occurred when the MoveL instruction invoked inverse kinematics" << std::endl;
            return 0;
        }
    }else{
        std::cout << "Instruction error: MoveL parameter input is illegal" << std::endl;
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
    std::string head_str = iter_data->str();//First point
    iter_data++;
    std::string second_str = iter_data->str();//Second point
    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)")) && std::regex_match(second_str,num_match2,std::regex("(JNT)([0-9]*)"))){   
        int index = atol(num_match[2].str().c_str());
        int index2 = atol(num_match2[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size() || index2 > _cmd_jnt_pos_list.size()){
            std::cout << "Command error: MoveC input position point number exceeds the limit" << std::endl;
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
        int res1 = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//Solving Forward Kinematics
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
        int res2 = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//Solving Forward Kinematics
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
            std::cout << "Instruction error: An error occurred when the MoveC instruction invoked forward kinematics" << std::endl;
            return 0;
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)")) && std::regex_match(second_str,num_match2,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        int index2 = atol(num_match2[2].str().c_str());
        if(index > _cmd_cart_pos_list.size() || index2 > _cmd_cart_pos_list.size()){
            std::cout << "Command error: MoveC input position point number exceeds the limit" << std::endl;
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
        int res1 = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//Solving Inverse Kinematics
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
        int res2 = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//Solving Inverse Kinematics
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
            std::cout << "Instruction error: An error occurred when the MoveC instruction invoked inverse kinematics" << std::endl;
            return 0;
        }
    }else{
        std::cout << "Instruction error: MoveC parameter input is illegal" << std::endl;
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


int ROS_API::SplineStart(std::string para){
    //empty para
    para.clear();
    return _send_data_factory_callback(FRAPI_base::command_factry("SplineStart",++_cmd_counter,para));
}

int ROS_API::SplinePTP(std::string para){
    //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl
    //By default, JNT data comes in
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
            std::cout << "Command error: SplinePTP input location point number exceeds limit" << std::endl;
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
        //std::cout << "SplinePTP data: " << FRAPI_base::command_factry("GetForwardKin",1,para2) << std::endl;
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetForwardKin",++_cmd_counter,para2));//Solving Forward Kinematics
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
            std::cout << "Command error:An error occurred when the Spline instruction called forward kinematics" << std::endl;
            return 0;
        }
    }else{
        std::cout << "Command error:Spline parameter input is illegal" << std::endl;
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
    //Input parameters:pos,speed,lastflag
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
            std::cout << "Command error:NewSplinePoint输入位置点序号超限" << std::endl;
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
        int res = _send_data_factory_callback(FRAPI_base::command_factry("GetInverseKin",++_cmd_counter,para2));//Solving Inverse Kinematics
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
            std::cout << "Command error:An error occurred when the NewSplinePoint instruction invoked inverse kinematics" << std::endl;
            return 0;
        }
    }else{
        std::cout << "Command error:NewSplinePoint parameter input is illegal" << std::endl;
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