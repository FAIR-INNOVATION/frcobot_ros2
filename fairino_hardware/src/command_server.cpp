#include "fairino_hardware/command_server.hpp"
#include <sys/types.h>
#include <sys/socket.h>
#include "fairino_hardware/version_control.h"

std::mutex _reconnect_mutex;

#define LOSE_TCP_CONNECT_TIME_MAX 10
#define LOGGER_NAME "fairino_ros2_command_server"

int  _program_state;//程序运行状态：1-停止、2-运行、3-暂停、4-拖动


/**
 * @brief 构造函数，用于初始化参数服务器中的变量，加载SDK库并连接机械臂
 * @param[in] node_name-节点名称，构建ros2 node必须的参数
 */
robot_command_thread::robot_command_thread(const std::string node_name):rclcpp::Node(node_name,
           rclcpp::NodeOptions().use_intra_process_comms(true)){
    using namespace std::chrono_literals;

    /*******************************************初始化私有变量**************************************/

    /*********************************************************************************************/

    /*************************************初始化参数服务器中的变量************************************/
    this->declare_parameter<uint8_t>("toolcoord_install",0);//默认工具安装在机器人末端
    this->declare_parameter<uint8_t>("toolcoord_type",0);//默认是工具坐标系
    this->declare_parameter<uint8_t>("collision_mode",0);//碰撞等级模式,默认是等级
    this->declare_parameter<uint8_t>("collision_config",1);//碰撞配置文件设置,默认不更新配置文件
    this->declare_parameter<uint8_t>("gripper_vel",80);
    this->declare_parameter<uint8_t>("gripper_force",50);
    this->declare_parameter<int>("gripper_maxtime",30000);
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
    this->declare_parameter<uint8_t>("MoveL_search",0);
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
    /*********************************************************************************************/

    /***********************************创建字符串指令服务器*****************************************/
    _recv_ros_command_server = this->create_service<remote_cmd_server_srv_msg>(
        REMOTE_CMD_SERVER_NAME,
        std::bind(&robot_command_thread::_parseROSCommandData_callback,\
            this,\
            std::placeholders::_1,\
            std::placeholders::_2)
        );
    /*********************************************************************************************/

    /********************************尝试使用SDK库连接机械臂******************************************/
    _controller_ip = CONTROLLER_IP;//控制器默认ip地址
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"ROS2指令服务器创建成功，准备连接机械臂  版本号:V%d.%f",\
        VERSION_MAJOR,\
        VERSION_MINOR);
    _ptr_robot = std::make_unique<FRRobot>();
    error_t returncode = _ptr_robot->RPC(_controller_ip.c_str());
    if(returncode !=0 ){
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"连接机械臂失败，程序即将退出！");
        exit(0);
    }
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"连接机械臂成功！");
    /*********************************************************************************************/
}


/**
 * @brief 析构函数，关闭xmlrpc连接并销毁SDK实例对象
 */
robot_command_thread::~robot_command_thread()
{
    _ptr_robot->CloseRPC();
    _ptr_robot->~FRRobot();
}




/**
 * @brief 私有函数，service的回调函数，用于解析字符串指令，跳转对应的处理函数
 */
void robot_command_thread::_parseROSCommandData_callback(
        const std::shared_ptr<remote_cmd_server_srv_msg::Request> req,\ 
        std::shared_ptr<remote_cmd_server_srv_msg::Response> res){
    //指令格式为movj(1,10)
    std::regex func_reg("([A-Z|a-z|_]+)[(](.*)[)]");//函数名的输入模式应该是字母函数名后跟(),圆括号中有所有输入参数
    std::smatch func_match;
    if(std::regex_match(req->cmd_str,func_match,func_reg)){
        std::string func_name = func_match[1];
        std::string para_list = func_match[2];
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"收到ROS指令，名称:%s,参数:%s",\
            func_name.c_str(),para_list.c_str());

        //校验参数的内容,参数部分必须是字母,数字和逗号,负号组成,出现其他字符包括空格都会导致校验失败
        std::regex para_pattern("[A-Z|a-z|\\.\\d|\\d|,|-]*");
        if(std::regex_match(para_list,para_pattern)){//检查参数输入是否合法
            auto find_idx = _fr_function_list.find(func_name);
            if(find_idx == _fr_function_list.end()){
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误: 找不到该指令对应的函数");
                res->cmd_res = std::string("-1");
            }else if(find_idx != _fr_function_list.end()){
                try{
                    res->cmd_res = (this->*(find_idx->second))(para_list);
                }catch(const std::invalid_argument& e){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"字符串转换过程出现无效参数");
                    res->cmd_res = "-1";
                }catch(const std::out_of_range& e){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"字符串转换过程出现超范围数据");
                    res->cmd_res = "-1";
                }catch(const std::logic_error& e){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"取参数过程发生异常，请检查参数个数是否正确");
                    res->cmd_res = "-1";
                }
            }
        }else{
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:函数参数输入不合法,参数列表由字母,数字和逗号组成,不能有空格出现");
            res->cmd_res = std::string("-1");
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:函数输入形式错误,函数输入必须是 [函数名]() 这种输入形式,请重新输入");
        res->cmd_res = std::string("-1");
    }
}

// template<typename T,typename ... Ts>
// void _recurseVar(T& first_arg,Ts&... args){

// }

/**
 * @brief 私有函数，用于按逗号分割字符串并存储入std::list容器中
 * @param [in] str-需要进行风格的字符串
 * @param [out] list_data-输出的按逗号风格出来的字符串列表 
 */
void robot_command_thread::_splitString2List(std::string str,std::list<std::string> &list_data){
    list_data.clear();
    std::regex search_para(",");//分隔符
    std::regex_token_iterator iter_data(str.begin(),str.end(),search_para,-1);
    std::regex_token_iterator<std::string::iterator> end; 
    while(iter_data != end){
        list_data.push_back(iter_data->str());
        iter_data++;
    }
}

/**
 * @brief 私有函数，用于按逗号分割字符串并存储入std::vector容器中
 * @param [in] str-需要进行风格的字符串
 * @param [out] list_data-输出的按逗号风格出来的字符串列表 
 */
void robot_command_thread::_splitString2Vec(std::string str,std::vector<std::string> &vector_data){
    vector_data.clear();
    std::regex search_para(",");//分隔符
    std::regex_token_iterator iter_data(str.begin(),str.end(),search_para,-1);
    std::regex_token_iterator<std::string::iterator> end; 
    while(iter_data != end){
        vector_data.push_back(iter_data->str());
        iter_data++;
    }
}

/**
 * @brief 私有函数，用于将list中开头6个数据填装到DescPose对象中，注意，此函数会弹出list对象中头6个数据
 * @param [in] data-list数据，用于存储字符串列表
 * @param [out] pose-输出的DescPose对象
 */
void robot_command_thread::_fillDescPose(std::list<std::string>& data,DescPose& pose){
    pose.tran.x = std::stod(data.front().c_str());data.pop_front();
    pose.tran.y = std::stod(data.front().c_str());data.pop_front();
    pose.tran.z = std::stod(data.front().c_str());data.pop_front();
    pose.rpy.rx = std::stod(data.front().c_str());data.pop_front();
    pose.rpy.ry = std::stod(data.front().c_str());data.pop_front();
    pose.rpy.rz = std::stod(data.front().c_str());data.pop_front();
}

/**
 * @brief 私有函数，用于将list中开头3个数据填装到DescTran对象中，注意，此函数会弹出list对象中头3个数据
 * @param [in] data-list数据，用于存储字符串列表
 * @param [out] pose-输出的DescTran对象
 */
void robot_command_thread::_fillDescTran(std::list<std::string>& data,DescTran& trans){
    trans.x = std::stod(data.front().c_str());data.pop_front();
    trans.y = std::stod(data.front().c_str());data.pop_front();
    trans.z = std::stod(data.front().c_str());data.pop_front();
}


/**
 * @brief 私有函数，用于处理JNTPoint()指令
 * @param [in] pos-点位序号及关节数据，以逗号间隔，共7个，第一个为序号，后续为关节值
 * @return 指令执行是否成功
 * @retval 0-成功，-1-失败
 */
std::string robot_command_thread::defJntPosition(std::string pos){
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
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"指令错误:关节位置参数为6个,参数输入个数请确认");
            return "-1";
        }
        iter_data = std::regex_token_iterator(pos.begin(),pos.end(),search_para,-1);
        int idx = atol(iter_data->str().c_str());//指令序号
        iter_data++;
        if(idx > _cmd_jnt_pos_list.size()+1 || idx <= 0){//如果大于当前容器最大值+1,那么要报错,因为序列容器中间无法留空
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"指令错误:容器序号超限");
            return "-1";
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
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),\
            "指令错误：关节点位输入参数规则为第一个为存储序号,后续为关节位置信息,以逗号隔开,不能出现空格");
        return "-1";
    }
    return "0";
}


/**
 * @brief 私有函数，用于处理CARTPoint()指令
 * @param [in] pos-笛卡尔数据，以逗号间隔，共7个,第一个参数为点位序号，后续为点位信息
 * @return 指令执行是否成功
 * @retval 0-成功，-1-失败
 */
std::string robot_command_thread::defCartPosition(std::string pos){
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
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"指令错误:笛卡尔位置参数为6个,参数输入个数请确认");
            return "-1";
        }
        iter_data = std::regex_token_iterator(pos.begin(),pos.end(),search_para,-1);
        int idx = atol(iter_data->str().c_str());//指令序号
        iter_data++;
        if(idx > _cmd_cart_pos_list.size()+1 || idx <= 0){//如果大于当前容器最大值+1,那么要报错,因为序列容器中间无法留空
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"指令错误:容器序号超限");
            return "-1";
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
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),\
            "指令错误：笛卡尔点位输入参数规则为第一个为存储序号,后续为笛卡尔位置信息,以逗号隔开,不能出现空格");
        return "-1";
    }
    return "0";
}


/**
 * @brief 私有函数，用于处理GET()指令，获取关节或者笛卡尔点位信息
 * @param [in] para_list 参数列表，形式为JNT/CART+数字组成
 * @return 指令执行是否成功
 * @retval 0-成功，-1-失败
 */
std::string robot_command_thread::getVariable(std::string para_list){
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
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:输入点的序号超出范围");
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
                    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:输入点的序号超出范围");
                }
        }else{
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误: 无效的GET指令参数");
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误: GET指令参数非法,参数形式为[JNT|CART],[序号]");
    }
}

/**
 * @brief 拖动示教模式切换函数
 * @param [in] para-模式选择
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码
 */
std::string  robot_command_thread::DragTeachSwitch(std::string para){
    return std::to_string(_ptr_robot->DragTeachSwitch(std::stoi(para)));
}

/**
 * @brief 机械臂使能
 * @param [in] para-使能开关,0-disable,1-enable
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string  robot_command_thread::RobotEnable(std::string para){
    return std::to_string(_ptr_robot->RobotEnable(std::stoi(para)));
}

/**
 * @brief 机械臂手动/自动模式切换
 * @param [in] para-模式设置
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::Mode(std::string para){
    return std::to_string(_ptr_robot->Mode(std::stoi(para)));
}

/**
 * @brief 设置速度
 * @param [in] para-速度值，单位为百分比
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetSpeed(std::string para){
    return std::to_string(_ptr_robot->SetSpeed(std::stoi(para)));
}

/**
 * @brief 设置工具坐标系标定值
 * @param [in] para-按照顺序依次包含id,x,y,z,rx,ry,rz
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetToolCoord(std::string para){
    //int id, DescPose *coord, int type, int install
    std::string install,type;
    install = this->get_parameter("toolcoord_install").value_to_string();
    type = this->get_parameter("toolcoord_type").value_to_string();
    para = para + "," + type + "," + install;

    std::list<std::string> datalist;
    _splitString2List(para,datalist);
    int id = std::stoi(datalist.front().c_str());datalist.pop_front();
    DescPose trans;
    _fillDescPose(datalist,trans);

    int typei = std::stoi(datalist.front().c_str());datalist.pop_front();
    int installi = std::stoi(datalist.front().c_str());
    int toolid = 0;
    double loadNo = 0.;
    //RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"settoolcoord:%d,%f,%f,%f,%f,%f,%f,%d,%d",\
    id,trans.tran.x, trans.tran.y, trans.tran.z,trans.rpy.rx,trans.rpy.ry,trans.rpy.rz,typei,installi);
    return std::to_string(_ptr_robot->SetToolCoord(id,&trans,typei,installi,toolid,loadNo));
}

/**
 * @brief 设置工具坐标系标定值列表
 * @param [in] para-按照顺序依次包含id,x,y,z,rx,ry,rz
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetToolList(std::string para){
    //int id, DescPose *coord, int type, int install
    std::string install,type;
    install = this->get_parameter("toolcoord_install").value_to_string();
    type = this->get_parameter("toolcoord_type").value_to_string();
    para = para + "," + type  + "," + install;

    std::list<std::string> list;
    _splitString2List(para,list);
    
    int id = std::stoi(list.front().c_str());list.pop_front();
    DescPose trans;
    _fillDescPose(list,trans);
    int typei = std::stod(list.front().c_str());list.pop_front();
    int installi = std::stod(list.front().c_str());
    uint8_t loadNo = 0;

    return std::to_string(_ptr_robot->SetToolList(id,&trans,typei,installi,loadNo));
}

/**
 * @brief 设置外部工具坐标系标定值
 * @param [in] para-按照顺序依次包含id,x,y,z,rx,ry,rz
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetExToolCoord(std::string para){
    //int id, DescPose *etcp, DescPose *etool
    DescPose etcp,etool;

    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    
    _fillDescPose(list,etcp);
    _fillDescPose(list,etool);

    return std::to_string(_ptr_robot->SetExToolCoord(id,&etcp,&etool));
}


/**
 * @brief 设置外部工具坐标系列表
 * @param [in] para-按照顺序依次包含id,x,y,z,rx,ry,rz
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetExToolList(std::string para){
    //int id, DescPose *etcp, DescPose *etool
    DescPose etcp,etool;

    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    
    _fillDescPose(list,etcp);
    _fillDescPose(list,etool);

    return std::to_string(_ptr_robot->SetExToolList(id,&etcp,&etool));
}

/**
 * @brief 设置工件坐标系标定值
 * @param [in] para-按照顺序依次包含id,x,y,z,rx,ry,rz
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetWObjCoord(std::string para){
    //int id, DescPose *coord
    DescPose coord;
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    
    _fillDescPose(list,coord);
    int ref_frame = 0;
    return std::to_string(_ptr_robot->SetWObjCoord(id,&coord,ref_frame));
}

/**
 * @brief 设置工件坐标系列表
 * @param [in] para-按照顺序依次包含id,x,y,z,rx,ry,rz
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetWObjList(std::string para){
    //int id, DescPose *coord
        DescPose coord;
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    
    _fillDescPose(list,coord);
    int ref_frame = 0;
    return std::to_string(_ptr_robot->SetWObjList(id,&coord,ref_frame));
}

/**
 * @brief 设置末端负载重量
 * @param [in] para-重量值，单位kg
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetLoadWeight(std::string para){
    //float weight
    return std::to_string(_ptr_robot->SetLoadWeight(std::stof(para)));
}

/**
 * @brief 设置末端负载质心偏移
 * @param [in] para-x,y,z,rx,ry,rz
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetLoadCoord(std::string para){
    //DescTran *coord
    DescTran coord;
    std::list<std::string> list;
    _splitString2List(para,list);
    _fillDescTran(list,coord);
    return std::to_string(_ptr_robot->SetLoadCoord(&coord));
}


/**
 * @brief 设置机械臂安装方式
 * @param [in] para-安装方式
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetRobotInstallPos(std::string para){
    //uint8_t install
    return std::to_string(_ptr_robot->SetRobotInstallPos(std::stoi(para)));
}

/**
 * @brief 设置机械臂安装角度
 * @param [in] para-顺序为yangle,zangle
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetRobotInstallAngle(std::string para){
    //double yangle, double zangle
    std::list<std::string> list;
    _splitString2List(para,list);
    double yangle = std::stod(list.front().c_str());
    double zangle = std::stod(list.front().c_str());
    return std::to_string(_ptr_robot->SetRobotInstallAngle(yangle,zangle));
}

/**
 * @brief 设置防碰撞策略
 * @param [in] para-顺序为mode,level[6],config
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetAnticollision(std::string para){
    //int mode, float level[6], int config
    std::string mode,config;
    mode = this->get_parameter("collision_mode").value_to_string();
    config = this->get_parameter("collision_config").value_to_string();
    para = mode + "," + para + "," + config;

    std::list<std::string> list;
    _splitString2List(para,list);
    
    int modei = std::stoi(list.front());list.pop_front();
    float level[6];
    for(int i=0;i<6;i++){
        level[i] = std::stof(list.front());
        list.pop_front();
    }
    int configi = std::stoi(list.front());
    return std::to_string(_ptr_robot->SetAnticollision(modei,level,configi));
}

/**
 * @brief 设置防碰撞等级
 * @param [in] para-strategy碰撞等级
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetCollisionStrategy(std::string para){
    //int strategy
    int margin[6]{1,1,1,1,1,1};
    return std::to_string(_ptr_robot->SetCollisionStrategy(std::stoi(para),1000,150,margin));
}

/**
 * @brief 设置关节正限位的值
 * @param [in] para-limit[6]每个关节限位角度
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetLimitPositive(std::string para){
    //float limit[6]
    std::list<std::string> list;
    _splitString2List(para,list);
    float limit[6];
    for(int i=0;i<6;i++){
        limit[i] = std::stof(list.front());
        list.pop_front();
    }
    return std::to_string(_ptr_robot->SetLimitPositive(limit));
}

/**
 * @brief 设置关节负限位的值
 * @param [in] para-limit[6]每个关节限位角度
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetLimitNegtive(std::string para){
    //float limit[6]
    std::list<std::string> list;
    _splitString2List(para,list);
    float limit[6];
    for(int i=0;i<6;i++){
        limit[i] = std::stof(list.front());
        list.pop_front();
    }
    return std::to_string(_ptr_robot->SetLimitNegative(limit));}

/**
 * @brief 尝试清除错误
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ResetAllError(std::string para){
    //empty para
    para.clear();
    return std::to_string(_ptr_robot->ResetAllError());
}

/**
 * @brief 摩擦力补偿开关
 * @param [in] para- 1-打开，0-关闭
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FrictionCompensationOnOff(std::string para){
    //uint8_t state
    return std::to_string(_ptr_robot->FrictionCompensationOnOff(std::stoi(para)));
}


/**
 * @brief 设置关节摩擦力等级
 * @param [in] para- coeff[6]
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetFrictionValue_level(std::string para){
    //float coeff[6]
    std::list<std::string> list;
    _splitString2List(para,list);
    float coeff[6];
    for(int i=0;i<6;i++){
        coeff[i] = std::stof(list.front());
        list.pop_front();
    }
    return std::to_string(_ptr_robot->SetFrictionValue_level(coeff));
}


/**
 * @brief 侧装状态设置关节摩擦力
 * @param [in] para- coeff[6]
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetFrictionValue_wall(std::string para){
    //float coeff[6]
    std::list<std::string> list;
    _splitString2List(para,list);
    float coeff[6];
    for(int i=0;i<6;i++){
        coeff[i] = std::stof(list.front());
        list.pop_front();
    }
    return std::to_string(_ptr_robot->SetFrictionValue_wall(coeff));
}

/**
 * @brief 倒装状态设置关节摩擦力
 * @param [in] para- coeff[6]
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetFrictionValue_ceiling(std::string para){
    //float coeff[6]
    std::list<std::string> list;
    _splitString2List(para,list);
    float coeff[6];
    for(int i=0;i<6;i++){
        coeff[i] = std::stof(list.front());
        list.pop_front();
    }
    return std::to_string(_ptr_robot->SetFrictionValue_ceiling(coeff));
}

/**
 * @brief 自由安装状态设置关节摩擦力
 * @param [in] para- coeff[6]
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetFrictionValue_freedom(std::string para){
    //float coeff[6]
    std::list<std::string> list;
    _splitString2List(para,list);
    float coeff[6];
    for(int i=0;i<6;i++){
        coeff[i] = std::stof(list.front());
        list.pop_front();
    }
    return std::to_string(_ptr_robot->SetFrictionValue_freedom(coeff));
}

/**
 * @brief 激活夹爪
 * @param [in] para-顺序依次为index,act
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ActGripper(std::string para){
    //int index, uint8_t act
    std::list<std::string> list;
    _splitString2List(para,list);
    int index = std::stoi(list.front());list.pop_front();
    uint8_t act = std::stoi(list.front());
    return std::to_string(_ptr_robot->ActGripper(index,act));
}

/**
 * @brief 控制夹爪动作
 * @param [in] para-顺序依次为index,pos
 * @attention 默认使用的线性导轨夹爪，如果需要改旋转夹爪需要修改后面的4个参数
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveGripper(std::string para){
    //int index, int pos, int vel, int force, int max_time, uint8_t block
    std::string vel,force,mtime,block;
    vel = this->get_parameter("gripper_vel").value_to_string();
    force = this->get_parameter("gripper_force").value_to_string();
    mtime  = this->get_parameter("gripper_maxtime").value_to_string();//数据读取异常
    block = this->get_parameter("gripper_block").value_to_string();
    para = para + "," + vel + "," + force + "," + mtime + "," + block; 
    
    std::list<std::string> list;
    _splitString2List(para,list);
    
    int index = std::stoi(list.front());list.pop_front();
    int pos = std::stoi(list.front());list.pop_front();
    int veli = std::stoi(list.front());list.pop_front();
    int forcei = std::stoi(list.front());list.pop_front();
    int max_timei = 30000;
    uint8_t blocki = 1;
    return std::to_string(_ptr_robot->MoveGripper(index,pos,veli,forcei,max_timei,blocki,0,0,0,0));
}
    
/**
 * @brief 设置数字IO输出
 * @param [in] para-顺序依次为id,status
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetDO(std::string para){
    //int id, uint8_t status, uint8_t smooth, uint8_t block
    std::string smooth,block;
    smooth = this->get_parameter("DO_smooth").value_to_string();
    block = this->get_parameter("DO_block").value_to_string();
    para = para + "," + smooth + "," + block;

    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    uint8_t status = std::stoi(list.front());list.pop_front();
    uint8_t smoothi = std::stoi(list.front());list.pop_front();
    uint8_t blocki = std::stoi(list.front());
    
    return std::to_string(_ptr_robot->SetDO(id,status,smoothi,blocki));
}

/**
 * @brief 设置工具数字IO输出
 * @param [in] para-顺序依次为id,status
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetToolDO(std::string para){
    //int id, uint8_t status, uint8_t smooth, uint8_t block
    std::string smooth,block;
    smooth = this->get_parameter("DO_smooth").value_to_string();
    block = this->get_parameter("DO_block").value_to_string();
    para = para + "," + smooth + "," + block;

    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    uint8_t status = std::stoi(list.front());list.pop_front();
    uint8_t smoothi = std::stoi(list.front());list.pop_front();
    uint8_t blocki = std::stoi(list.front());
    
    return std::to_string(_ptr_robot->SetToolDO(id,status,smoothi,blocki));
}

/**
 * @brief 设置模拟IO输出
 * @param [in] para-顺序依次为id,value
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetAO(std::string para){
    //int id, float value, uint8_t block
    std::string block;
    block = this->get_parameter("AO_block").value_to_string();
    para = para + "," + block;

    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    float value = std::stof(list.front())/100*4095;list.pop_front();
    uint8_t blocki = std::stoi(list.front());

    return std::to_string(_ptr_robot->SetAO(id,value,blocki));
}

/**
 * @brief 设置工具模拟IO输出
 * @param [in] para-顺序依次为id,value
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetToolAO(std::string para){
    //int id, float value, uint8_t block
    std::string block;
    block = this->get_parameter("AO_block").value_to_string();
    para = para + "," + block;

    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    float value = std::stof(list.front())/100*4095;list.pop_front();
    uint8_t blocki = std::stoi(list.front());

    return std::to_string(_ptr_robot->SetToolAO(id,value,blocki));
}

/**
 * @brief 设置辅助数字IO输出
 * @param [in] para-顺序依次为number,open
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetAuxDO(std::string para){
    //int DOnumber,bool open, bool smooth, bool block
    std::string smooth,block;
    smooth = this->get_parameter("DO_smooth").value_to_string();
    block = this->get_parameter("DO_block").value_to_string();
    para = para + "," + smooth + "," + block;

    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    uint8_t open = std::stoi(list.front());list.pop_front();
    uint8_t smoothi = std::stoi(list.front());list.pop_front();
    uint8_t blocki = std::stoi(list.front());
    return std::to_string(_ptr_robot->SetAuxDO(id,open,smoothi,blocki));
}

/**
 * @brief 设置辅助模拟IO输出
 * @param [in] para-顺序依次为number,percentage
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetAuxAO(std::string para){
    //int number int percentage
    std::string block;
    block = this->get_parameter("AO_block").value_to_string();
    para = para + "," + block;

    std::list<std::string> list;
    _splitString2List(para,list);
    
    int number = std::stoi(list.front());list.pop_front();
    double value = std::stod(list.front())/100*4095;list.pop_front();
    uint8_t blocki = std::stoi(list.front());

    return std::to_string(_ptr_robot->SetAuxAO(number,value,blocki));
}

/**
 * @brief 外部轴使能
 * @param [in] para-顺序依次为number,switch
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtAxisServoOn(std::string para){
    //int number, int switch
    std::list<std::string> list;
    _splitString2List(para,list);
    
    int number = std::stoi(list.front());list.pop_front();
    uint8_t sw = std::stoi(list.front());
    return std::to_string(_ptr_robot->ExtAxisServoOn(number,sw));
}

/**
 * @brief 外部轴点动开始
 * @param [in] para-顺序依次为number,direction,speed,acc,max_distance
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtAxisStartJog(std::string para){
    //6, int number, int direction, int speed, int acc_speed, double max_distance
    std::list<std::string> list;
    _splitString2List(para,list);
    
    int number = std::stoi(list.front());list.pop_front();
    int direction = std::stoi(list.front());list.pop_front();
    int speed = std::stoi(list.front());list.pop_front();
    int acc = std::stoi(list.front());list.pop_front();
    double max_dis = std::stod(list.front());

    return std::to_string(_ptr_robot->ExtAxisStartJog(number,direction,speed,acc,max_dis));
}

/**
 * @brief 外部轴回零
 * @param [in] para-顺序依次为number,zero_mode,search_speed,latch_speed
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtAxisSetHoming(std::string para){
    //int number, int zero_mode, int search_speed, int latch_speed
    std::list<std::string> list;
    _splitString2List(para,list);
    int number = std::stoi(list.front());list.pop_front();
    int mode = std::stoi(list.front());list.pop_front();
    int search_speed = std::stoi(list.front());list.pop_front();
    int latch_speed = std::stoi(list.front());

    return std::to_string(_ptr_robot->ExtAxisSetHoming(number,mode,search_speed,latch_speed));
}

/**
 * @brief 外部轴停止点动
 * @param [in] para- index
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::StopExtAxisJog(std::string para){
    //int indexid
    return std::to_string(_ptr_robot->ExtAxisStopJog(std::stoi(para)));
}


/**
 * @brief 机械臂关节点动
 * @param [in] para- 参数顺序依次为ref,nb,dir,vel,acc,max_dis
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::StartJOG(std::string para){
    //uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis
    std::list<std::string> list;
    _splitString2List(para,list);
    
    uint8_t ref = std::stoi(list.front());list.pop_front();
    uint8_t nb = std::stoi(list.front());list.pop_front();
    uint8_t dir = std::stoi(list.front());list.pop_front();
    float vel = std::stof(list.front());list.pop_front();
    float acc = std::stof(list.front());list.pop_front();
    float dis = std::stof(list.front());

    return std::to_string(_ptr_robot->StartJOG(ref,nb,dir,vel,acc,dis));
}

/**
 * @brief 机械臂减速停止点动
 * @param [in] para- ref
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::StopJOG(std::string para){
    //uint8_t ref
    return std::to_string(_ptr_robot->StopJOG(std::stoi(para)));
}

/**
 * @brief 机械臂立即停止点动
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ImmStopJOG(std::string para){
    //empty para
    para.clear();
    return std::to_string(_ptr_robot->ImmStopJOG());
}

/**
 * @brief 机械臂关节空间运动
 * @param [in] para - 顺序依次为JNT点位/CART点位,speed,tool,user
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveJ(std::string para){
    //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos
    int speed,tool,user;

    float acc = this->get_parameter("MoveJLC_acc").as_double();
    float ovl = this->get_parameter("MoveJLC_ovl").as_double();
    double eaxis1 = this->get_parameter("MoveJLC_eaxis1").as_double();
    double eaxis2 = this->get_parameter("MoveJLC_eaxis2").as_double();
    double eaxis3 = this->get_parameter("MoveJLC_eaxis3").as_double();
    double eaxis4 = this->get_parameter("MoveJLC_eaxis4").as_double();
    float blendT = this->get_parameter("MoveJ_blendT").as_double();
    uint8_t offset_flag = this->get_parameter("MoveJLC_offset_flag").as_int();
    double offset_pos_x = this->get_parameter("MoveJLC_offset_pos_x").as_double();
    double offset_pos_y = this->get_parameter("MoveJLC_offset_pos_y").as_double();
    double offset_pos_z = this->get_parameter("MoveJLC_offset_pos_z").as_double();
    double offset_pos_rx = this->get_parameter("MoveJLC_offset_pos_rx").as_double();
    double offset_pos_ry = this->get_parameter("MoveJLC_offset_pos_ry").as_double();
    double offset_pos_rz = this->get_parameter("MoveJLC_offset_pos_rz").as_double();


    std::regex search_para(",");
    std::regex_token_iterator iter_data(para.begin(),para.end(),search_para,-1);
    std::regex_token_iterator<std::string::iterator> end;
    std::smatch num_match;
    std::string head_str = iter_data->str();

    JointPos tmp_jnt_pos;
    DescPose cartpos;

    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)"))){//第一个元素是否满足JNT1这种模式
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveJ输入位置点序号超限");
            return "-1";
        }
        tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        if(_ptr_robot->GetForwardKin(&tmp_jnt_pos,&cartpos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveJ指令调用正向运动学发生错误");
            return "-1";
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveJ输入位置点序号超限");
            return "-1";
        }
        cartpos = _cmd_cart_pos_list.at(index-1);
        if(_ptr_robot->GetInverseKin(0,&cartpos,-1,&tmp_jnt_pos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveJ指令调用逆向运动学发生错误");
            return "-1";
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveJ参数输入非法,没有找到点位信息");
        return "-1";
    }

    if (iter_data != end){
        iter_data++;

        speed = std::stoi(iter_data->str());
        iter_data++;

        // Check if there are more elements in the iterator
        tool = std::stoi(iter_data->str());
        iter_data++;

        user = std::stoi(iter_data->str());
    }

    ExaxisPos extpos{eaxis1,eaxis2,eaxis3,eaxis4};
    DescPose offsetpos{offset_pos_x,offset_pos_y,offset_pos_z,offset_pos_rx,offset_pos_ry,offset_pos_rz};

        // std::string tmp_para = FRAPI_base::command_factry("MoveJ",1,para);
        // std::cout << "MoveJ发送数据: " << tmp_para << std::endl;
    return std::to_string(_ptr_robot->MoveJ(&tmp_jnt_pos,&cartpos,tool,user,speed,acc,\
        ovl,&extpos,blendT,offset_flag,&offsetpos));
}


/**
 * @brief 机械臂笛卡尔空间直线运动
 * @param [in] para - 顺序依次为JNT点位/CART点位,speed,tool,user
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveL(std::string para){
    //JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos
    int speed,tool,user;

    float acc = this->get_parameter("MoveJLC_acc").as_double();
    float ovl = this->get_parameter("MoveJLC_ovl").as_double();
    double eaxis1 = this->get_parameter("MoveJLC_eaxis1").as_double();
    double eaxis2 = this->get_parameter("MoveJLC_eaxis2").as_double();
    double eaxis3 = this->get_parameter("MoveJLC_eaxis3").as_double();
    double eaxis4 = this->get_parameter("MoveJLC_eaxis4").as_double();
    float blendR = this->get_parameter("MoveL_blendR").as_double();
    uint8_t search = this->get_parameter("MoveL_search").as_int();
    uint8_t offset_flag = this->get_parameter("MoveJLC_offset_flag").as_int();
    double offset_pos_x = this->get_parameter("MoveJLC_offset_pos_x").as_double();
    double offset_pos_y = this->get_parameter("MoveJLC_offset_pos_y").as_double();
    double offset_pos_z = this->get_parameter("MoveJLC_offset_pos_z").as_double();
    double offset_pos_rx = this->get_parameter("MoveJLC_offset_pos_rx").as_double();
    double offset_pos_ry = this->get_parameter("MoveJLC_offset_pos_ry").as_double();
    double offset_pos_rz = this->get_parameter("MoveJLC_offset_pos_rz").as_double();

    std::regex search_para(",");
    std::regex_token_iterator iter_data(para.begin(),para.end(),search_para,-1);
    std::regex_token_iterator<std::string::iterator> end;
    std::smatch num_match;
    std::string head_str = iter_data->str();

    JointPos tmp_jnt_pos;
    DescPose cartpos;

     if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)"))){//第一个元素是否满足JNT1这种模式
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveL输入位置点序号超限");
            return "-1";
        }
        tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        if(_ptr_robot->GetForwardKin(&tmp_jnt_pos,&cartpos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveL指令调用正向运动学发生错误");
            return "-1";
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveL输入位置点序号超限");
            return "-1";
        }
        cartpos = _cmd_cart_pos_list.at(index-1);
        if(_ptr_robot->GetInverseKin(0,&cartpos,-1,&tmp_jnt_pos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveL指令调用逆向运动学发生错误");
            return "-1";
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveL参数输入非法,没有找到点位信息");
        return "-1";
    }

    if (iter_data != end){
        iter_data++;

        speed = std::stoi(iter_data->str());
        iter_data++;

        // Check if there are more elements in the iterator
        tool = std::stoi(iter_data->str());
        iter_data++;

        user = std::stoi(iter_data->str());
    }

    ExaxisPos extpos{eaxis1,eaxis2,eaxis3,eaxis4};
    DescPose offsetpos{offset_pos_x,offset_pos_y,offset_pos_z,offset_pos_rx,offset_pos_ry,offset_pos_rz};

    return std::to_string(_ptr_robot->MoveL(&tmp_jnt_pos,&cartpos,tool,user,speed,acc,ovl,blendR,\
        &extpos,search,offset_flag,&offsetpos));
}


/**
 * @brief 机械臂笛卡尔空间圆弧运动
 * @param [in] para - 顺序依次为两个JNT点位/CART点位,speed,tool,user
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveC(std::string para){
    /*JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p,
    JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t,float ovl, float blendR*/
    int speed,tool,user;

    float acc = this->get_parameter("MoveJLC_acc").as_double();
    float ovl = this->get_parameter("MoveJLC_ovl").as_double();
    double eaxis1 = this->get_parameter("MoveJLC_eaxis1").as_double();
    double eaxis2 = this->get_parameter("MoveJLC_eaxis2").as_double();
    double eaxis3 = this->get_parameter("MoveJLC_eaxis3").as_double();
    double eaxis4 = this->get_parameter("MoveJLC_eaxis4").as_double();
    float blendR = this->get_parameter("MoveL_blendR").as_double();
    uint8_t search = this->get_parameter("MoveL_search").as_int();
    uint8_t offset_flag = this->get_parameter("MoveJLC_offset_flag").as_int();
    double offset_pos_x = this->get_parameter("MoveJLC_offset_pos_x").as_double();
    double offset_pos_y = this->get_parameter("MoveJLC_offset_pos_y").as_double();
    double offset_pos_z = this->get_parameter("MoveJLC_offset_pos_z").as_double();
    double offset_pos_rx = this->get_parameter("MoveJLC_offset_pos_rx").as_double();
    double offset_pos_ry = this->get_parameter("MoveJLC_offset_pos_ry").as_double();
    double offset_pos_rz = this->get_parameter("MoveJLC_offset_pos_rz").as_double();

    std::regex search_para(",");
    std::regex_token_iterator iter_data(para.begin(),para.end(),search_para,-1);
    std::regex_token_iterator<std::string::iterator> end;

    std::smatch num_match,num_match2;
    std::string head_str = iter_data->str();//第一个点位
    iter_data++;
    std::string second_str = iter_data->str();//第二个点位

    JointPos tmp_jnt_pos,tmp_jnt_pos2;
    DescPose cartpos,cartpos2;

    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)")) && \
        std::regex_match(second_str,num_match2,std::regex("(JNT)([0-9]*)"))){   
        int index = atol(num_match[2].str().c_str());
        int index2 = atol(num_match2[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size() || index2 > _cmd_jnt_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveC输入位置点序号超限");
            return "-1";
        }
        JointPos tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        JointPos tmp_jnt_pos2 = _cmd_jnt_pos_list.at(index2-1);

        if(_ptr_robot->GetForwardKin(&tmp_jnt_pos,&cartpos) != 0 ||
            _ptr_robot->GetForwardKin(&tmp_jnt_pos2,&cartpos2) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveC指令调用正向运动学发生错误");
            return "-1";
        } 
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)")) && \
              std::regex_match(second_str,num_match2,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        int index2 = atol(num_match2[2].str().c_str());
        if(index > _cmd_cart_pos_list.size() || index2 > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveC输入位置点序号超限");
            return "-1";
        }
        cartpos = _cmd_cart_pos_list.at(index-1);
        cartpos2 = _cmd_cart_pos_list.at(index2-1);
        if(_ptr_robot->GetInverseKin(0,&cartpos,-1,&tmp_jnt_pos) != 0 ||
            _ptr_robot->GetInverseKin(0,&cartpos2,-1,&tmp_jnt_pos2) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveC指令调用逆向运动学发生错误");
            return "-1";
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:MoveC参数输入非法");
        return "-1";
    }

    if (iter_data != end){
        iter_data++;

        speed = std::stoi(iter_data->str());
        iter_data++;

        // Check if there are more elements in the iterator
        tool = std::stoi(iter_data->str());
        iter_data++;

        user = std::stoi(iter_data->str());
    }

    ExaxisPos extpos{eaxis1,eaxis2,eaxis3,eaxis4};
    DescPose offsetpos{offset_pos_x,offset_pos_y,offset_pos_z,offset_pos_rx,\
        offset_pos_ry,offset_pos_rz};

    return std::to_string(_ptr_robot->MoveC(&tmp_jnt_pos,&cartpos,tool,user,speed,acc,\
        &extpos,offset_flag,&offsetpos,&tmp_jnt_pos2,&cartpos2,tool,user,speed,acc,&extpos,\
        offset_flag,&offsetpos,ovl,blendR));      
}

std::string robot_command_thread::Circle(std::string para){
    /*JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, 
    int puser, float pvel, float pacc, ExaxisPos *epos_p, JointPos *joint_pos_t, 
    DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, 
    ExaxisPos *epos_t, float ovl, uint8_t offset_flag, DescPose *offset_pos*/
    return "-1";
}

/**
 * @brief 机械臂关节空间样条运动开始
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SplineStart(std::string para){
    //empty para
    para.clear();
    return std::to_string(_ptr_robot->SplineStart());
}

/**
 * @brief 机械臂关节空间样条运动
 * @param [in] para-顺序依次是JNT点位，tool,user,vel
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SplinePTP(std::string para){
    /*JointPos *joint_pos, DescPose *desc_pos, int tool, 
    int user, float vel, float acc, float ovl*/
    //默认进来的都是JNT数据
    int speed,tool,user;

    float acc = this->get_parameter("Spline_acc").as_double();
    float ovl = this->get_parameter("Spline_ovl").as_double();

    std::regex search_para(",");
    std::regex_token_iterator iter_data(para.begin(),para.end(),search_para,-1);
    std::regex_token_iterator<std::string::iterator> end;

    std::smatch num_match;
    std::string head_str = iter_data->str();
    if(std::regex_match(head_str,num_match,std::regex("(JNT)([0-9]*)"))){ 
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_jnt_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),\
                "指令错误:SplinePTP输入位置点序号超限");
            return "-1";
        }

        JointPos tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        DescPose cartpos;

        if(_ptr_robot->GetForwardKin(&tmp_jnt_pos,&cartpos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),\
                "指令错误:Spline指令调用正向运动学发生错误");
            return "-1";
        }

        if (iter_data != end){
            iter_data++;

            speed = std::stoi(iter_data->str());
            iter_data++;

            // Check if there are more elements in the iterator
            tool = std::stoi(iter_data->str());
            iter_data++;

            user = std::stoi(iter_data->str());
        }

        return std::to_string(_ptr_robot->SplinePTP(&tmp_jnt_pos,&cartpos,\
                                tool,user,speed,acc,ovl));
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:Spline参数输入非法");
        return "-1";
    }
}


/**
 * @brief 机械臂关节空间样条运动结束
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SplineEnd(std::string para){
    //empty para
    para.clear();
    return std::to_string(_ptr_robot->SplineEnd());
}


/**
 * @brief 机械臂笛卡尔空间样条运动开始
 * @param [in] para-type 0-圆弧过渡，1-给定点位为路径点
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::NewSplineStart(std::string para){
    //uint8_t ctlPoint
    return std::to_string(_ptr_robot->NewSplineStart(std::stoi(para)));
}

/**
 * @brief 机械臂笛卡尔空间样条运动
 * @param [in] para-顺序依次是CART点位，speed,tool,user,lastflag
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::NewSplinePoint(std::string para){
    /*JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, 
      float acc, float ovl float blendR uint8_t lastFlag*/
    //输入参数:pos,speed,lastflag

    int speed,tool,user;
    uint8_t lastflag;

    float acc = this->get_parameter("Spline_acc").as_double();
    float ovl = this->get_parameter("Spline_ovl").as_double();
    double blendR = this->get_parameter("NewSpline_blendR").as_double();


    std::regex search_para(",");
    std::regex_token_iterator iter_data(para.begin(),para.end(),search_para,-1);
    std::regex_token_iterator<std::string::iterator> end;

    std::smatch num_match;
    std::string head_str = iter_data->str();

    if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:NewSplinePoint输入位置点序号超限");
            return "-1";
        }
        DescPose cartpos = _cmd_cart_pos_list.at(index-1);
        JointPos jntpos;

        if(_ptr_robot->GetInverseKin(0,&cartpos,-1,&jntpos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:NewSplinePoint指令调用逆向运动学发生错误");
            return "-1";
        }

        if (iter_data != end){
            iter_data++;

            speed = std::stoi(iter_data->str());
            iter_data++;

            // Check if there are more elements in the iterator
            tool = std::stoi(iter_data->str());
            iter_data++;

            user = std::stoi(iter_data->str());
            iter_data++;

            lastflag = std::stoi(iter_data->str());
        }
        
        return std::to_string(_ptr_robot->NewSplinePoint(&jntpos,&cartpos,tool,user,speed,\
            acc,ovl,blendR,lastflag));
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"指令错误:NewSplinePoint参数输入非法");
        return "-1";
    }
}

/**
 * @brief 机械臂笛卡尔空间样条运动结束
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::NewSplineEnd(std::string para){
    //empty para
    para.clear();
    return std::to_string(_ptr_robot->NewSplineEnd());
}

/**
 * @brief 机械臂停止运动
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::StopMotion(std::string para){
    //empty para
    para.clear();
    return std::to_string(_ptr_robot->StopMotion());
}

/**
 * @brief 点位偏移使能
 * @param [in] para-顺序依次是flag, offset_pos
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::PointsOffsetEnable(std::string para){
    //int flag, DescPose *offset_pos
    std::list<std::string> list;
    _splitString2List(para,list);

    int flag = std::stoi(list.front());list.pop_front();
    DescPose cartpos;
    _fillDescPose(list,cartpos);
    
    return std::to_string(_ptr_robot->PointsOffsetEnable(flag,&cartpos));
}

/**
 * @brief 点位偏移关闭
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::PointsOffsetDisable(std::string para){
    //empty para
    return std::to_string(_ptr_robot->PointsOffsetDisable());
}

/**
 * @brief 设置扩展轴配置参数
 * @param [in] para-servoid,servocomany,model,softversion,reoslution,ratio
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoSetParam(std::string para){ 
    int servoid,servocomany,model,version,resolution;
    double ratio;

    std::list<std::string> list;
    _splitString2List(para,list);
    
    servoid = std::stoi(list.front());list.pop_front();
    servocomany = std::stoi(list.front());list.pop_front();
    model = std::stoi(list.front());list.pop_front();
    version = std::stoi(list.front());list.pop_front();
    resolution = std::stoi(list.front());list.pop_front();
    ratio = std::stod(list.front());
    return std::to_string(_ptr_robot->AuxServoSetParam(servoid,servocomany,model,\
                            version,resolution,ratio));
}

/**
 * @brief 设置扩展轴使能
 * @param [in] para-servoid, status
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoEnable(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    
    int servoid;
    uint8_t status;
    servoid = std::stoi(list.front());list.pop_front();
    status = std::stoi(list.front());

    return std::to_string(_ptr_robot->AuxServoEnable(servoid,status));
}


/**
 * @brief 设置扩展轴控制模式
 * @param [in] para-servoid, mode
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoSetControlMode(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int servoid,mode;
    servoid = std::stoi(list.front());list.pop_front();
    mode = std::stoi(list.front());
    
    return std::to_string(_ptr_robot->AuxServoSetControlMode(servoid,mode));
}

/**
 * @brief 设置扩展轴目标位置
 * @param [in] para-servoid,pos,speed
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoSetTargetPos(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int servoid;
    double pos,speed;
    servoid = std::stoi(list.front());list.pop_front();
    pos = std::stod(list.front());list.pop_front();
    speed = std::stod(list.front());
    
    return std::to_string(_ptr_robot->AuxServoSetTargetPos(servoid,pos,speed));
}

/**
 * @brief 设置扩展轴目标速度
 * @param [in] para-servoid,speed
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoSetTargetSpeed(std::string para){

    std::list<std::string> list;
    _splitString2List(para,list);

    int servoid;
    double speed;

    servoid = std::stoi(list.front());list.pop_front();
    speed = std::stod(list.front());
    return std::to_string(_ptr_robot->AuxServoSetTargetSpeed(servoid,speed));
}

/**
 * @brief 设置扩展轴目标扭矩
 * @param [in] para-servoid,torque
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoSetTargetTorque(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int servoid;
    double torque;
    servoid = std::stoi(list.front());list.pop_front();
    torque = std::stod(list.front());

    return std::to_string(_ptr_robot->AuxServoSetTargetTorque(servoid,torque));
}

/**
 * @brief 扩展轴归零
 * @param [in] para-servoid,mode,searchvel,latchvel
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoHoming(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int servoid,mode;
    double searchvel,latchvel;
    servoid = std::stoi(list.front());list.pop_front();
    mode = std::stoi(list.front());list.pop_front();
    searchvel = std::stod(list.front());list.pop_front();
    latchvel = std::stod(list.front());

    return std::to_string(_ptr_robot->AuxServoHoming(servoid,mode,searchvel,latchvel));
}

/**
 * @brief 扩展轴清除错误
 * @param [in] para-servoid
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoClearError(std::string para){
    return std::to_string(_ptr_robot->AuxServoClearError(std::stoi(para)));
}

/**
 * @brief 设置扩展轴ID
 * @param [in] para-servoid
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoSetStatusID(std::string para){
    return std::to_string(_ptr_robot->AuxServosetStatusID(std::stoi(para)));
}


/**
 * @brief 开始执行脚本
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ScriptStart(std::string para){
    return std::to_string(_ptr_robot->ProgramRun());
}

/**
 * @brief 停止脚本
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ScriptStop(std::string para){
    //empty
    return std::to_string(_ptr_robot->ProgramStop());
}

/**
 * @brief 暂停脚本执行
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ScriptPause(std::string para){
    //empty
    return std::to_string(_ptr_robot->ProgramPause());
}

/**
 * @brief 继续执行脚本
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ScriptResume(std::string para){
    //empty
    return std::to_string(_ptr_robot->ProgramResume());
}

/**
 * @brief 获取工具标定值
 * @return TCP标定值
 * @retval x,y,z,rx,ry,rz 
 */
std::string robot_command_thread::GetTCPOffset(std::string para){
    uint8_t index = std::stoi(para);
    DescPose cartpos;
    if(_ptr_robot->GetTCPOffset(index,&cartpos) == 0){
        return std::to_string(cartpos.tran.x) + "," + \
               std::to_string(cartpos.tran.y) + "," + \
               std::to_string(cartpos.tran.z) + "," + \
               std::to_string(cartpos.rpy.rx) + "," + \
               std::to_string(cartpos.rpy.ry) + "," + \
               std::to_string(cartpos.rpy.rz);
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"获取工具标定值失败");
        return "0,0,0,0,0,0";
    }
}

/**
 * @brief 获取DH补偿值
 * @return DH补偿值
 * @retval a1,a2,a3,a4,a5,a6
 */
std::string robot_command_thread::GetDHCompensation(std::string para){
    double dhcomp[6];
    if(_ptr_robot->GetDHCompensation(dhcomp) == 0){
        return std::to_string(dhcomp[0]) + "," + \
               std::to_string(dhcomp[1]) + "," + \
               std::to_string(dhcomp[2]) + "," + \
               std::to_string(dhcomp[3]) + "," + \
               std::to_string(dhcomp[4]) + "," + \
               std::to_string(dhcomp[5]);
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"获取DH补偿值失败");
        return "0,0,0,0,0,0";
    }
}

/**
 * @brief 可移动设备使能
 * @param [in] para-使能状态，0-去使能 1-使能
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::TractorEnable(std::string para){
    //bool enable
    int sw = std::stoi(para);
    return std::to_string(_ptr_robot->TractorEnable(sw));
}

/**
 * @brief 可移动设备回零
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::TractorHoming(std::string para){
    //empty para
    return std::to_string(_ptr_robot->TractorHoming());
}

/**
 * @brief 可移动设备直线运动
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::TractorMoveL(std::string para){
    //double distance, double vel
    std::list<std::string> list;
    _splitString2List(para,list);
    
    double distance = std::stod(list.front());list.pop_front();
    double vel = std::stod(list.front());
    return std::to_string(_ptr_robot->TractorMoveL(distance,vel));
}

/**
 * @brief 可移动设备圆弧运动
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::TractorMoveC(std::string para){
    //double ratio, double angle, double vel
    std::list<std::string> list;
    _splitString2List(para,list);
    
    double ratio = std::stod(list.front());list.pop_front();
    double angle = std::stod(list.front());list.pop_front();
    double vel = std::stod(list.front());
    return std::to_string(_ptr_robot->TractorMoveC(ratio,angle,vel));
}

/**
 * @brief 可移动设备运动停止
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::TractorStop(std::string para){
    //empty para
    return std::to_string(_ptr_robot->TractorStop());
}






/******状态信息获取节点******/
/*************************/
/**
 * @brief 状态监控节点构造函数
 * @param [in] node_name-节点名称
 */
robot_recv_thread::robot_recv_thread(const std::string node_name):rclcpp::Node(node_name){
    using namespace std::chrono_literals;
    _controller_ip = CONTROLLER_IP;//控制器默认ip地址
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"开始创建状态反馈TCP socket");

    //只保留8081端口的连接，8083连接传输的数据已经不用
    _socketfd1 = socket(AF_INET,SOCK_STREAM,0);//状态获取端口只有TCP

    if(_socketfd1 == -1){
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"错误: 创建socket失败!");
        exit(0);//创建套字失败,丢出错误
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"创建状态反馈socket成功,开始连接控制器...");
        struct sockaddr_in tcp_client1;
        tcp_client1.sin_family = AF_INET;
        tcp_client1.sin_port = htons(port1);//8083端口
        tcp_client1.sin_addr.s_addr = inet_addr(_controller_ip.c_str());


        //尝试连接控制器
        int res1 = connect(_socketfd1,(struct sockaddr *)&tcp_client1,sizeof(tcp_client1));
        if(0 != res1){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"错误:无法连接控制器数据端口,程序退出!");
            exit(0);//连接失败,丢出错误并返回
        }else{
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"控制器状态端口连接成功");
            //将socket设置成非阻塞模式
            int flags1 = fcntl(_socketfd1,F_GETFL,0);
            fcntl(_socketfd1,F_SETFL,flags1|SOCK_NONBLOCK);
            
            //开启keepalive
            if(0 != setKeepAlive(_socketfd1, 5, 3, 3)){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"开启tcp心跳检测失败");
            }

            _state_publisher = this->create_publisher<robot_feedback_msg>(
                "nonrt_state_data",
                1
            );
            _locktimer = this->create_wall_timer(10ms,std::bind(&robot_recv_thread::_state_recv_callback,this));//创建一个定时器任务用于获取非实时状态数据,触发间隔为100ms
        }

        //连接成功，创建守护线程,如果该连接端掉，则自动发起重连接;生命周期随该节点
        _try_to_reconnect();
    }
}

/**
 * @brief TCP断开重连函数
 */
void robot_recv_thread::_try_to_reconnect(){
    auto _reconnect_func = [this](){
        while ((1 != _robot_recv_exit)){
            do{
                std::lock_guard<std::mutex> _lock(_reconnect_mutex);
                /* try to re-connect 58.2 8081*/
                if (_reconnect_flag){
                    // 关闭旧连接
                    shutdown(_socketfd1, SHUT_RDWR);
                    close(_socketfd1);
                    _socketfd1 = -1;
                    // std::this_thread::sleep_for(std::chrono::seconds(1));

                    int sock_fd = socket(AF_INET, SOCK_STREAM, 0);
                    if (-1 == sock_fd){
                        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"创建套接字失败, 3s 后再次尝试");
                        break;
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
                            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),\
                                    "发起重新连接失败,程序退出! 3s 后再次尝试");
                            shutdown(sock_fd, SHUT_RDWR);
                            close(sock_fd);
                            break;
                        }
                        else
                        {
                            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"TCP重新连接成功");
                            // 设置TCP接收超时
                            int flags2 = fcntl(sock_fd, F_GETFL, 0);
                            fcntl(sock_fd, F_SETFL, flags2 | SOCK_NONBLOCK);

                            // 开启并设置keepalive
                            if (0 != setKeepAlive(sock_fd, 5, 3, 3)){
                                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"开启tcp心跳检测失败");
                            }
                            // return sock_fd;
                            _socketfd1 = sock_fd;
                            _reconnect_flag.store(false);
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

/**
 * @brief 状态监控节点类的析构函数
 */
robot_recv_thread::~robot_recv_thread(){
    //关闭并销毁socket
    if(_socketfd1 != -1){
        shutdown(_socketfd1,SHUT_RDWR);
        close(_socketfd1);
    }

    _robot_recv_exit = 1;
    if(_reconnect_thread.joinable()){
        _reconnect_thread.join();
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"重连线程退出");
    }
}

/**
 * @brief 
 * @param idle_time 空闲idle_time后，开始发射探针
 * @param interval_time 发射首个探针后，如果interval_time内没有响应，再次发射探针
 * @param probe_times 一共会发射probe_times次探针
 * @return -1-开启失败；0-成功
*/
int robot_recv_thread::setKeepAlive(int fd, int idle_time, int interval_time, int probe_times){
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


/**
 * @brief 8081反馈数据端口topic监听回调函数
*/
void robot_recv_thread::_state_recv_callback(){
    static bool concatante_flag = 0;//帧数据拼接flag
    static std::once_flag oneflag;
    static _CTRL_STATE ctrl_state;
    static uint32_t delta_length = 0;
    static int length_left = 0;
    static uint32_t begin_index = 0;
    static std::queue<_CTRL_STATE> ctrl_state_store_buff;//用于存储缓存区多余的数据，需要限制长度
    static char ctrl_state_temp_buff[_CTRL_STATE_SIZE] = {0};//用于临时存储不完整帧的数据，之后用于数据拼接

   /* 如果处于重连流程，不需要再读取，直接返回 */
    if(_reconnect_flag){
        //RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"重连中，请等待......");
        return;
    }

    if(!concatante_flag){//不需要数据拼接的情况
        char recv_buff[_CTRL_STATE_SIZE] = {0};
        int length_read = 0;

        /* 如果处于重连流程，不需要再读取，直接返回 */
        do{
            length_read = recv(_socketfd1,recv_buff,1,0);
            // 这里是非阻塞读，开启探针后，当recv失败时，通过errno查看结果
            if ((length_read == 0) || ((length_read == -1) && (errno != EINTR )&&\
            (errno != EWOULDBLOCK) && (errno != EAGAIN))){
                _reconnect_flag.store(true);
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"网络可能已经断开，请检查网络连接 ......");
                break;
            }
            if(length_read == 1 && recv_buff[0] == '/'){
                length_read = recv(_socketfd1,&recv_buff[1],3,0);
                std::string head_str(recv_buff,4); 
                if(length_read == 3 && head_str == "/f/b"){
                    break;
                }
            }
        }while(1);

        if(_reconnect_flag){//等待重连，while中无法使用return
            return;
        }

        int len = recv(_socketfd1,&recv_buff[4],7,0);
        if(len == -1){
            return;
        }
        int32_t* ptr_frame_length = (int32_t*)(&recv_buff[7]);
        if(*ptr_frame_length < (_CTRL_STATE_SIZE-14)){//帧长度小于等于预期，直接填装
            std::call_once(oneflag,[&](){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),\
                    "反馈状态数据帧长度小于预期,差距为%i,请对齐fairino_hardware功能包与机械臂软件版本。",\
                    _CTRL_STATE_SIZE-14-*ptr_frame_length
                );
            });
            delta_length = 0;
        }else if(*ptr_frame_length > (_CTRL_STATE_SIZE-14)){//帧长度大于预期，需要削去多余数据，默认削去尾部数据
            std::call_once(oneflag,[&](){
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),\
                    "反馈状态数据帧长度大于预期,差距为%i,请对齐fairino_hardware功能包与机械臂软件版本。",\
                    *ptr_frame_length - _CTRL_STATE_SIZE+14
                );
            });
            delta_length = *ptr_frame_length - _CTRL_STATE_SIZE+14;
        }else{
            delta_length = 0;
        }
        // std::cout << ctrl_state_datalen << " 开始取数据  " << _is_reconnect << " " << strerror(errno) << std::endl;
        int future_len = *ptr_frame_length + 3;//注意别漏掉帧尾的数据，需要考虑到已经读了frame_length
        char oneshot_buff[future_len] = {0};
        int recv_length = recv(_socketfd1,oneshot_buff,future_len,0);
        if(recv_length < future_len){//获取长度小于预期，说明帧数据需要进行拼接
            if(recv_length != -1){
                concatante_flag = 1;
                memset(ctrl_state_temp_buff,0,_CTRL_STATE_SIZE);
                memcpy(&recv_buff[11],oneshot_buff,recv_length);//如果是数据长度差太多，执行这句可能出现数组越界的情况
                memcpy(ctrl_state_temp_buff,recv_buff,_CTRL_STATE_SIZE);
                length_left = future_len - recv_length;
                begin_index = recv_length + 11;
                //std::cout << "有包头但是数据长度不正确,8081数据校验长度: "<< _CTRL_STATE_SIZE << "," << *((int*)(data_len_ptr))  << std::endl;
            }
            return;
        }else{
            memcpy(&recv_buff[11],oneshot_buff,future_len-delta_length);
            memcpy(&ctrl_state, recv_buff, sizeof(ctrl_state));
            if(ctrl_state_store_buff.size() < 3){
                ctrl_state_store_buff.emplace(ctrl_state);//将数据放入队列中
            }else{//如果队列中数据满10个，那么删掉队列头部数据然后队尾再插入数据
                ctrl_state_store_buff.pop();//弹出队首的元素
                ctrl_state_store_buff.emplace(ctrl_state);
            }
        }
    }else{//需要数据拼接的情况
        //RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"进入拼接状态,%i",length_left);
        char concatante_buff[length_left] = {0};
        int length_concatante = recv(_socketfd1,concatante_buff,length_left,0);
        if ((length_concatante == 0) || ((length_concatante == -1) && (errno != EINTR )&&\
        (errno != EWOULDBLOCK) && (errno != EAGAIN))){
            _reconnect_flag.store(true);
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"网络可能已经断开，请检查网络连接 ......");
            return;
        }
        if(length_concatante < length_left){
            if(length_concatante != -1){
                memcpy(&ctrl_state_temp_buff[begin_index],concatante_buff,length_concatante);
                begin_index += length_concatante;
                length_left -= length_concatante;
                //break;//已经没有数据了，退出循环
            }
            return;
        }else{//length_concatante == length_left
            concatante_flag = 0;
            //RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"获得最后一段帧数据：%i",length_concatante);
            std::string tail(&concatante_buff[length_left-7],7);
            if(tail == "III/b/f"){
                memcpy(&ctrl_state_temp_buff[begin_index],concatante_buff,length_concatante);
                memcpy(&ctrl_state,ctrl_state_temp_buff,_CTRL_STATE_SIZE);
                if(ctrl_state_store_buff.size() < 3){
                    ctrl_state_store_buff.emplace(ctrl_state);//将数据放入队列中
                }else{//队列长度等于10的时候，弹出队首的元素，在队尾加入元素
                    ctrl_state_store_buff.pop();
                    ctrl_state_store_buff.emplace(ctrl_state);
                }
            }else{
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"帧数据拼接失败，帧尾数据校验失败，重新寻找帧头,%i,%i",length_concatante,length_left);
                return;
            }
        }
    }//end while

    //下面是从队列中读取数据
    if(!ctrl_state_store_buff.empty()){
        ctrl_state = ctrl_state_store_buff.front();
        ctrl_state_store_buff.pop();

        auto msg = robot_feedback_msg();
        auto cur_clock = rclcpp::Clock();

            msg.error_code = 0;
            msg.robot_motion_done = ctrl_state.motion_done;
            msg.robot_mode = ctrl_state.robot_mode;
            msg.emg = ctrl_state.btn_box_stop_signal;
            msg.grip_motion_done = ctrl_state.gripperMotionDone;

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

            msg.j1_cur_tor = ctrl_state.jt_cur_tor[0];
            msg.j2_cur_tor = ctrl_state.jt_cur_tor[1];
            msg.j3_cur_tor = ctrl_state.jt_cur_tor[2];
            msg.j4_cur_tor = ctrl_state.jt_cur_tor[3];
            msg.j5_cur_tor = ctrl_state.jt_cur_tor[4];
            msg.j6_cur_tor = ctrl_state.jt_cur_tor[5];

            msg.prg_state = ctrl_state.program_state;
            msg.abnormal_stop = ctrl_state.abnormal_stop;
            msg.prg_name = std::string(ctrl_state.curLuaFileName);
            msg.prg_total_line = 0;
            msg.prg_cur_line = ctrl_state.line_number;

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
            
            msg.timestamp = RCL_NS_TO_S(cur_clock.now().nanoseconds());
            for(int i=0;i<6;i++){
                msg.safetyboxsig[i] = ctrl_state.safetyBoxSignal[i];
            }
            _state_publisher->publish(msg);
        }
    //下面是从队列中读取数据
    // if(!store_buff.empty() && 0){//如果队列不为空，那就读取数据，否则auxservo_state就跳过本次callback
    //     state_data = store_buff.front();
    //     store_buff.pop();
    //     auto msg = robot_feedback_msg();

    //     _state_publisher->publish(msg);
    //     //std::cout <<_start_return_data<<std::endl;
        

    // }
}
