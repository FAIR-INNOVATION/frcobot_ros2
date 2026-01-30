#include "fairino_hardware/command_server.hpp"
#include <sys/types.h>
#include <sys/socket.h>
#include "fairino_hardware/version_control.h"

std::atomic_bool _reconnect_flag;
std::atomic<int> mainerrcode;
std::atomic<int> suberrcode;

#define LOGGER_NAME "fairino_ros2_command_server"

#ifdef CHN_VERSION
char* msgout[] = {
    "ROS2指令服务器创建成功,准备连接机械臂",
    "fairino_hardware版本号:",
    "适配机械臂软件版本号:",
    "构建时间:",
    "连接机械臂失败，准备重连...",
    "连接机械臂失败，程序即将退出！",
    "连接机械臂成功！",
    "收到ROS指令,名称&参数:",
    "指令错误: 找不到该指令对应的函数",
    "出现无效参数，请检查参数数据类型",
    "参数出现超范围数据",
    "取参数过程发生异常，请检查参数个数是否正确",
    "指令错误:函数参数输入不合法,参数列表由字母,数字和逗号组成,不能有空格出现",
    "指令错误:函数输入形式错误,函数输入必须是 [函数名]() 这种输入形式,请重新输入",
    "指令错误:关节位置参数为6个,参数输入个数请确认",
    "指令错误:关节/笛卡尔容器序号超限",
    "指令错误：关节点位输入参数规则为第一个为存储序号,后续为关节位置信息,以逗号隔开,不能出现空格",
    "指令错误:笛卡尔位置参数为6个,参数输入个数请确认",
    "指令错误：笛卡尔点位输入参数规则为第一个为存储序号,后续为笛卡尔位置信息,以逗号隔开,不能出现空格",
    "指令错误: 无效的GET指令参数",
    "指令错误: GET指令参数非法,参数形式为[JNT|CART],[序号]",
    "指令错误:指令调用正向运动学发生错误",
    "指令错误:指令调用逆向运动学发生错误",
    "指令错误:点位参数输入非法,没有找到点位信息",
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
    "反馈状态数据帧长度小于预期,请对齐fairino_hardware功能包与机械臂软件版本。",
    "反馈状态数据帧长度大于预期,请对齐fairino_hardware功能包与机械臂软件版本。",
    "帧数据拼接失败，帧尾数据校验失败，重新寻找帧头"
};
#endif

#ifdef ENG_VERSION
char* msgout[] = {
    "ROS2 command server created,ready to connect robot",
    "fairino_hardware:",
    "Adapt to software version of robot:",
    "Package build time:",
    "Robot connect failed! try to reconnect...",
    "Robot connect failed! program about to exit",
    "Robot connected!",
    "Receive ROS command,command name&parameters:",
    "Command error:invalid fucntion name",
    "Invalid parameter,please check data types of input parameters",
    "Parameter out of range,please check value of input parameters",
    "Incorrect parameter number,please check numbers of input parameters",
    "Illegal parameters,parameters consist of number,dot and letters,space in not allowed",
    "Illegal command format,you must follow the format:'function name(parameters)'",
    "Number of joint position is 6, please check parameter number",
    "Joint pos container index out of range",
    "Parameter fomart:index,j1pos...j6pos,please input correct format",
    "Number of cartesean position is 6, please check parameter number",
    "Parameter fomart:index,x,y,z,r,p,y,please input correct format",
    "Invalid command GET parameter",
    "Illegal command GET parameter,format:[JNT|CART],[index]",
    "Joint/Cartesean position container index out of range",
    "Forward kinematic error occur,please check input point",
    "Inverse kinematic error occur,please check input point",
    "Invalid container index,can't find the point",
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
    "The volumn of state feedback data is smaller than expected, please check robot sofeware version",
    "The volumn of state feedback data is larger than expected, please check robot sofeware version",
    "The sumcheck of state feedback data is failed, drop the data and search frame head again"
};
#endif

typedef enum _msg_id{
    hello,
    ver_package,
    ver_robot,
    build_time,
    try_reconnect,
    connect_failed,
    connect_success,
    receive_cmd,
    invalid_cmd,
    invalid_datatype,
    invalid_range,
    invalid_paranum,
    illegal_para,
    illegal_cmdfomart,
    invalid_jntposnum,
    out_container_range,
    invalid_jntcontainer_format,
    invalid_cartposnum,
    invalid_cartcontainer_format,
    invalid_get_para,
    illegal_get_para,
    fwd_kin_error,
    inv_kin_error,
    invalid_container_index,
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
    feedback_data_small,
    feedback_data_large,
    search_head_again
}msg_id;






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
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Robot ip:%s",CONTROLLER_IP);

    //打印输出版本信息及其他前置信息
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(hello)]);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),(std::string(msgout[msg_id(ver_package)])+std::string("V%i.%i.%i")).c_str(),\
        VERSION_MAJOR,VERSION_MINOR,VERSION_MINOR2);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),(std::string(msgout[msg_id(ver_robot)])+std::string("V%i.%i.%i,")).c_str(),\
        VERSION_ROBOT_MARJOR,VERSION_ROBOT_MINOR,VERSION_ROBOT_MINOR2);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),(std::string(msgout[msg_id(build_time)])+std::string("%s,%s")).c_str(),\
        __TIME__,__DATE__);

    //开始初始化
    int connect_count = 0;
    _ptr_robot = std::make_unique<FRRobot>();
    _ptr_robot->SetReConnectParam(true,30000,500);
    //_ptr_robot->LoggerInit(0,"/home/fa/ros2_ws/fairino_SDK_log/cppsdk.log",5);
    while(connect_count <_connect_retry_SDK){
        error_t returncode = _ptr_robot->RPC(_controller_ip.c_str());
        if(returncode !=0){
            connect_count++;
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(try_reconnect)]);
        }else{//正常情况
            break;
        }
        if(connect_count == _connect_retry_SDK){
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(connect_failed)]);
            exit(0);  
        }
    }

    _locktimer = this->create_wall_timer(10ms,std::bind(&robot_command_thread::_getRobotRTState,this));
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(connect_success)]);
    /*********************************************************************************************/
     _state_publisher = this->create_publisher<robot_feedback_msg>("nonrt_state_data",1);
    _locktimer1 = this->create_wall_timer(10ms,std::bind(&robot_command_thread::_state_recv_callback,this));//创建一个定时器任务用于获取非实时状态数据,触发间隔为100ms

}


/**
 * @brief 析构函数，关闭xmlrpc连接并销毁SDK实例对象
 */
robot_command_thread::~robot_command_thread()
{
    //_ptr_robot->CloseRPC();
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
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),(std::string(msgout[msg_id(receive_cmd)])+\
            std::string("%s,%s")).c_str(),func_name.c_str(),para_list.c_str());

        //校验参数的内容,参数部分必须是字母,数字和逗号,负号组成,出现其他字符包括空格都会导致校验失败
        std::regex para_pattern(".*");
        if(std::regex_match(para_list,para_pattern)){//检查参数输入是否合法
            auto find_idx = _fr_function_list.find(func_name);
            if(find_idx == _fr_function_list.end()){
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_cmd)]);
                res->cmd_res = std::string("-1");
            }else if(find_idx != _fr_function_list.end()){
                try{
                    res->cmd_res = (this->*(find_idx->second))(para_list);
                }catch(const std::invalid_argument& e){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_datatype)]);
                    res->cmd_res = "-1";
                }catch(const std::out_of_range& e){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_range)]);
                    res->cmd_res = "-1";
                }catch(const std::logic_error& e){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_paranum)]);
                    res->cmd_res = "-1";
                }
            }
        }else{
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(illegal_para)]);
            res->cmd_res = std::string("-1");
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(illegal_cmdfomart)]);
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
 * @brief 私有函数，用于将list中开头6个数据填充到JointPos对象中
 * @param [in] data-list数据，用于存储字符串列表
 * @param [out] pos-输出的JointPos对象
 */
void robot_command_thread::_fillJointPose(std::list<std::string>& data,JointPos& pos){
    for(int i=0;i<6;i++){
        pos.jPos[i] = std::stod(data.front().c_str());
        data.pop_front();
    }
}

void robot_command_thread::_getRobotRTState(){
    static ROBOT_STATE_PKG tmp;
    _ptr_robot->GetRobotRealTimeState(&tmp);
    mainerrcode = tmp.main_code;
    suberrcode = tmp.sub_code;
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
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_jntposnum)]);
            return "-1";
        }
        iter_data = std::regex_token_iterator(pos.begin(),pos.end(),search_para,-1);
        int idx = atol(iter_data->str().c_str());//指令序号
        iter_data++;
        if(idx > _cmd_jnt_pos_list.size()+1 || idx <= 0){//如果大于当前容器最大值+1,那么要报错,因为序列容器中间无法留空
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
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
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_jntcontainer_format)]);
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
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_cartposnum)]);
            return "-1";
        }
        iter_data = std::regex_token_iterator(pos.begin(),pos.end(),search_para,-1);
        int idx = atol(iter_data->str().c_str());//指令序号
        iter_data++;
        if(idx > _cmd_cart_pos_list.size()+1 || idx <= 0){//如果大于当前容器最大值+1,那么要报错,因为序列容器中间无法留空
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
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
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_cartcontainer_format)]);
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
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
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
                    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
                }
        }else{
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_get_para)]);
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(illegal_get_para)]);
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
 * @param [in] para-loadNum,weight
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetLoadWeight(std::string para){
    //int loadNum,float weight
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    float weight = std::stof(list.front().c_str());
    //return std::to_string(_ptr_robot->SetLoadWeight(id));
    return std::to_string(_ptr_robot->SetLoadWeight(id,weight));//V378
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
    double yangle = std::stod(list.front().c_str());list.pop_front();
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
 * @param [in] para-strategy,safedisntance,safevel
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetCollisionStrategy(std::string para){
    //int strategy int safetime,int safedistance,int safevel,int* safemargin
    std::list<std::string> list;
    _splitString2List(para,list);
    int strategy = std::stoi(list.front().c_str());list.pop_front();
    int safedistance = std::stoi(list.front().c_str());list.pop_front();
    int safevel = std::stoi(list.front().c_str());
    int margin[6]{1,1,1,1,1,1};
    //return std::to_string(_ptr_robot->SetCollisionStrategy(std::stoi(para),1000,safedistance,margin));
    return std::to_string(_ptr_robot->SetCollisionStrategy(std::stoi(para),1000,safedistance,safevel,margin));//V378
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
 * @brief 获取UDP扩展轴坐标系
 * @return 指令执行是否成功及坐标系数值
 * @retval flag,x,y,z,rx,ry,rz 
 */
std::string robot_command_thread::ExtAxisGetCoord(std::string para){
    DescPose cartpos;
    int res = _ptr_robot->ExtAxisGetCoord(cartpos);
    return std::to_string(res) + "," + \
        std::to_string(cartpos.tran.x) + "," + \
        std::to_string(cartpos.tran.y) + "," + \
        std::to_string(cartpos.tran.z) + "," + \
        std::to_string(cartpos.rpy.rx) + "," + \
        std::to_string(cartpos.rpy.ry) + "," + \
        std::to_string(cartpos.rpy.rz);
}

/**
 * @brief UDP扩展轴移动
 * @return 指令执行是否成功及坐标系数值
 * @retval flag,x,y,z,rx,ry,rz 
 */
std::string robot_command_thread::ExtAxisMove(std::string para){
    ExaxisPos tarpos;
    std::list<std::string> list;
    _splitString2List(para,list);
    tarpos.ePos[0] = std::stof(list.front());list.pop_front();
    tarpos.ePos[1] = std::stof(list.front());list.pop_front();
    tarpos.ePos[2] = std::stof(list.front());list.pop_front();
    tarpos.ePos[3] = std::stof(list.front());list.pop_front();
    int vel = std::stoi(list.front());
    return std::to_string(_ptr_robot->ExtAxisMove(tarpos,vel));
}

/**
 * @brief UDP扩展轴移动
 * @return 指令执行是否成功及坐标系数值
 * @retval flag,x,y,z,rx,ry,rz 
 */
std::string robot_command_thread::ExtAxisSyncMoveJ(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    JointPos jntpos;
    DescPose cartpos,offsetpos;
    ExaxisPos epos;
    for(int i=0;i<6;i++){
        jntpos.jPos[i] = std::stof(list.front());
        list.pop_front();
    }
    cartpos.tran.x = std::stof(list.front());list.pop_front();
    cartpos.tran.y = std::stof(list.front());list.pop_front();
    cartpos.tran.z = std::stof(list.front());list.pop_front();
    cartpos.rpy.rx = std::stof(list.front());list.pop_front();
    cartpos.rpy.ry = std::stof(list.front());list.pop_front();
    cartpos.rpy.rz = std::stof(list.front());list.pop_front();

    int tool = std::stoi(list.front());list.pop_front();
    int user = std::stoi(list.front());list.pop_front();
    int vel = std::stoi(list.front());list.pop_front();
    int acc = std::stoi(list.front());list.pop_front();
    int ovl = std::stoi(list.front());list.pop_front();

    for(int i=0;i<4;i++){
        epos.ePos[i] = std::stof(list.front());
        list.pop_front();
    }
    float blend = std::stof(list.front());list.pop_front();
    uint8_t offset_flag = std::stoi(list.front());list.pop_front();
    offsetpos.tran.x = std::stof(list.front());list.pop_front();
    offsetpos.tran.y = std::stof(list.front());list.pop_front();
    offsetpos.tran.z = std::stof(list.front());list.pop_front();
    offsetpos.rpy.rx = std::stof(list.front());list.pop_front();
    offsetpos.rpy.ry = std::stof(list.front());list.pop_front();
    offsetpos.rpy.rz = std::stof(list.front());

    return std::to_string(_ptr_robot->ExtAxisSyncMoveJ(jntpos,cartpos,tool,user,vel,acc,ovl,epos,\
        blend,offset_flag,offsetpos));
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
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
            return "-1";
        }
        tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        if(_ptr_robot->GetForwardKin(&tmp_jnt_pos,&cartpos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(fwd_kin_error)]);
            return "-1";
        }
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"返回正向运动学结果:%f,%f,%f,%f,%f,%f",cartpos.tran.x,\
            cartpos.tran.y,cartpos.tran.z,cartpos.rpy.rx,cartpos.rpy.ry,cartpos.rpy.rz);
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
            return "-1";
        }
        cartpos = _cmd_cart_pos_list.at(index-1);
        if(_ptr_robot->GetInverseKin(0,&cartpos,-1,&tmp_jnt_pos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(inv_kin_error)]);
            return "-1";
        }
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"返回逆向运动学结果:%f,%f,%f,%f,%f,%f",tmp_jnt_pos.jPos[0],\
            tmp_jnt_pos.jPos[1],tmp_jnt_pos.jPos[2],tmp_jnt_pos.jPos[3],tmp_jnt_pos.jPos[4],tmp_jnt_pos.jPos[5]);
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_container_index)]);
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
    }
    if (iter_data != end){
        eaxis1 = std::stod(iter_data->str());iter_data++;
        eaxis2 = std::stod(iter_data->str());iter_data++;
        eaxis3 = std::stod(iter_data->str());iter_data++;
        eaxis4 = std::stod(iter_data->str());iter_data++;
    }
    ExaxisPos extpos{eaxis1,eaxis2,eaxis3,eaxis4};
    DescPose offsetpos{offset_pos_x,offset_pos_y,offset_pos_z,offset_pos_rx,offset_pos_ry,offset_pos_rz};

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
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
            return "-1";
        }
        tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        if(_ptr_robot->GetForwardKin(&tmp_jnt_pos,&cartpos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(fwd_kin_error)]);
            return "-1";
        }
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        if(index > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
            return "-1";
        }
        cartpos = _cmd_cart_pos_list.at(index-1);
        if(_ptr_robot->GetInverseKin(0,&cartpos,-1,&tmp_jnt_pos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(inv_kin_error)]);
            return "-1";
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_container_index)]);
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
    }
    if (iter_data != end){
        eaxis1 = std::stod(iter_data->str());iter_data++;
        eaxis2 = std::stod(iter_data->str());iter_data++;
        eaxis3 = std::stod(iter_data->str());iter_data++;
        eaxis4 = std::stod(iter_data->str());iter_data++;
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
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
            return "-1";
        }
        JointPos tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        JointPos tmp_jnt_pos2 = _cmd_jnt_pos_list.at(index2-1);

        if(_ptr_robot->GetForwardKin(&tmp_jnt_pos,&cartpos) != 0 ||
            _ptr_robot->GetForwardKin(&tmp_jnt_pos2,&cartpos2) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(fwd_kin_error)]);
            return "-1";
        } 
    }else if(std::regex_match(head_str,num_match,std::regex("(CART)([0-9]*)")) && \
              std::regex_match(second_str,num_match2,std::regex("(CART)([0-9]*)"))){
        int index = atol(num_match[2].str().c_str());
        int index2 = atol(num_match2[2].str().c_str());
        if(index > _cmd_cart_pos_list.size() || index2 > _cmd_cart_pos_list.size()){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
            return "-1";
        }
        cartpos = _cmd_cart_pos_list.at(index-1);
        cartpos2 = _cmd_cart_pos_list.at(index2-1);
        if(_ptr_robot->GetInverseKin(0,&cartpos,-1,&tmp_jnt_pos) != 0 ||
            _ptr_robot->GetInverseKin(0,&cartpos2,-1,&tmp_jnt_pos2) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(inv_kin_error)]);
            return "-1";
        }
    }else{
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_container_index)]);
        return "-1";
    }

    ExaxisPos extpos1{eaxis1,eaxis2,eaxis3,eaxis4},extpos2{eaxis1,eaxis2,eaxis3,eaxis4};

    if (iter_data != end){
        iter_data++;

        speed = std::stoi(iter_data->str());
        iter_data++;

        // Check if there are more elements in the iterator
        tool = std::stoi(iter_data->str());
        iter_data++;

        user = std::stoi(iter_data->str());
        iter_data++;
    }

    if (iter_data != end){
        eaxis1 = std::stod(iter_data->str());iter_data++;
        eaxis2 = std::stod(iter_data->str());iter_data++;
        eaxis3 = std::stod(iter_data->str());iter_data++;
        eaxis4 = std::stod(iter_data->str());iter_data++;

        extpos1.ePos[0] = eaxis1;
        extpos1.ePos[1] = eaxis2;
        extpos1.ePos[2] = eaxis3;
        extpos1.ePos[3] = eaxis4;

        eaxis1 = std::stod(iter_data->str());iter_data++;
        eaxis2 = std::stod(iter_data->str());iter_data++;
        eaxis3 = std::stod(iter_data->str());iter_data++;
        eaxis4 = std::stod(iter_data->str());iter_data++;

        extpos2.ePos[0] = eaxis1;
        extpos2.ePos[1] = eaxis2;
        extpos2.ePos[2] = eaxis3;
        extpos2.ePos[3] = eaxis4;
    }

    DescPose offsetpos{offset_pos_x,offset_pos_y,offset_pos_z,offset_pos_rx,\
        offset_pos_ry,offset_pos_rz};

    return std::to_string(_ptr_robot->MoveC(&tmp_jnt_pos,&cartpos,tool,user,speed,acc,\
        &extpos1,offset_flag,&offsetpos,&tmp_jnt_pos2,&cartpos2,tool,user,speed,acc,&extpos2,\
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
 * @brief 机械臂关节伺服指令，该指令对于实时性要求较高
 * @param [in] jntpos-六个关节的位置指令，单位为度，eaxispos-4个外部轴位置指令，单位为度，deltaT-指令时间间隔，范围0.001~0.0016
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ServoJ(std::string para){
    std::list<std::string> datalist;
    _splitString2List(para,datalist);

    JointPos jpos;
    _fillJointPose(datalist,jpos);

    ExaxisPos eaxispos;
    eaxispos.ePos[0] = std::stod(datalist.front().c_str());datalist.pop_front();
    eaxispos.ePos[1] = std::stod(datalist.front().c_str());datalist.pop_front();
    eaxispos.ePos[2] = std::stod(datalist.front().c_str());datalist.pop_front();
    eaxispos.ePos[3] = std::stod(datalist.front().c_str());datalist.pop_front();

    int deltaT = std::stod(datalist.front().c_str());
    return std::to_string(_ptr_robot->ServoJ(&jpos,&eaxispos,0,0,deltaT,0,0));
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
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
            return "-1";
        }

        JointPos tmp_jnt_pos = _cmd_jnt_pos_list.at(index-1);
        DescPose cartpos;

        if(_ptr_robot->GetForwardKin(&tmp_jnt_pos,&cartpos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(fwd_kin_error)]);
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
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_container_index)]);
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
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(out_container_range)]);
            return "-1";
        }
        DescPose cartpos = _cmd_cart_pos_list.at(index-1);
        JointPos jntpos;

        if(_ptr_robot->GetInverseKin(0,&cartpos,-1,&jntpos) != 0){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(inv_kin_error)]);
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
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(invalid_container_index)]);
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
 * @brief 加载脚本
 * @param [in] para-program_name[64]
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ScriptLoad(std::string para){
    //program_name[64]
    char name[64];
    if(para.length() < 64){
        para.copy(name,para.length());
        name[para.length()] = '\0';
    }
    return std::to_string(_ptr_robot->ProgramLoad(name));
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
 * @brief 获取fairino_hardware版本号
 * @return 错误码及版本号
 * @retval res,version
 */
std::string robot_command_thread::GetVersion(std::string para){
    std::string ver = "fairino_hardware:V" + std::to_string(VERSION_MAJOR) + "." + \
        std::to_string(VERSION_MINOR) + "." + std::to_string(VERSION_MINOR2);
    return std::string("0," + ver);
}

/**
 * @brief 获取fairino_msg版本号
 * @return 错误码及版本号
 * @retval res,version
 */
std::string robot_command_thread::GetMsgVersion(std::string para){
    std::string ver = "fairino_msgs:V" + std::to_string(VERSION_MSG_MARJOR) + "." + \
        std::to_string(VERSION_MSG_MINOR) + "." + std::to_string(VERSION_MSG_MINOR2);
    return std::string("0," + ver);
}
/**
 * @brief 获取机械臂版本号
 * @return 错误码及版本号
 * @retval res,softwareversion
 */
std::string robot_command_thread::GetRobotVersion(std::string para){
    char robotmodel[64],softversion[64],ctrversion[64];
    int res = _ptr_robot->GetSoftwareVersion(robotmodel,softversion,ctrversion);
    return std::string(std::to_string(res) + ",robot:" + std::string(softversion));
}

/**
 * @brief 获取机械臂控制器版本号
 * @return 错误码及版本号
 * @retval res,sofewareversion
 */
std::string robot_command_thread::GetControllerVersion(std::string para){
    char robotmodel[64],softversion[64],ctrversion[64];
    int res = _ptr_robot->GetSoftwareVersion(robotmodel,softversion,ctrversion);
    return std::string(std::to_string(res) + ",robot_controller:" + std::string(ctrversion));
}

/**
 * @brief 获取工具标定值
 * @return TCP标定值
 * @retval res,x,y,z,rx,ry,rz 
 */
std::string robot_command_thread::GetTCPOffset(std::string para){
    uint8_t index = std::stoi(para);
    DescPose cartpos;
    int res = _ptr_robot->GetTCPOffset(index,&cartpos);
    return std::to_string(res) + "," + \
            std::to_string(cartpos.tran.x) + "," + \
            std::to_string(cartpos.tran.y) + "," + \
            std::to_string(cartpos.tran.z) + "," + \
            std::to_string(cartpos.rpy.rx) + "," + \
            std::to_string(cartpos.rpy.ry) + "," + \
            std::to_string(cartpos.rpy.rz);
}

/**
 * @brief 获取DH补偿值
 * @return DH补偿值
 * @retval a1,a2,a3,a4,a5,a6
 */
std::string robot_command_thread::GetDHCompensation(std::string para){
    double dhcomp[6];
    int res = _ptr_robot->GetDHCompensation(dhcomp);
    return std::to_string(res) + "," + \
            std::to_string(dhcomp[0]) + "," + \
            std::to_string(dhcomp[1]) + "," + \
            std::to_string(dhcomp[2]) + "," + \
            std::to_string(dhcomp[3]) + "," + \
            std::to_string(dhcomp[4]) + "," + \
            std::to_string(dhcomp[5]);
}

/**
 * @brief 获取焊接中断状态
 * @return 焊接中断状态结构体信息
 * @retval breakOffState,weldArcState
 */
std::string robot_command_thread::GetWeldingBreakOffState(std::string para){
    // static WELDING_BREAKOFF_STATE state;
    // int res = _ptr_robot->GetRobotRealTimeState(&_robot_realtime_state);
    // state = _robot_realtime_state.weldingBreakOffState;
    // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"焊接状态参数:%i,%i",\
    //     _robot_realtime_state.weldingBreakOffState.breakOffState,
    //     _robot_realtime_state.weldingBreakOffState.weldArcState);
    // return std::string(std::to_string(res) + "," + std::to_string(state.breakOffState) + \
    //         "," + std::to_string(state.weldArcState));
}

/**
 * @brief 获取错误码
 * @return 错误码
 * @retval maincode,subcode
 */
std::string robot_command_thread::GetErrorCode(std::string para){
    int maincode,subcode;
    _ptr_robot->GetRobotErrorCode(&maincode,&subcode);
    return std::string(std::to_string(maincode) + "," + std::to_string(subcode));
}

/**
 * @brief 获取逆向运动学
 * @return 逆向运动学解
 * @retval maincode,subcode
 */
std::string robot_command_thread::GetInverseKin(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front());list.pop_front();
    DescPose cartpose;
    _fillDescPose(list,cartpose);
    int config = std::stoi(list.front());list.pop_front();
    JointPos jntpose;
    int res = _ptr_robot->GetInverseKin(type,&cartpose,config,&jntpose);
    return std::string(std::to_string(res) + "," + std::to_string(jntpose.jPos[0]) + "," +\
            std::to_string(jntpose.jPos[1]) + "," + std::to_string(jntpose.jPos[2]) + "," +\
            std::to_string(jntpose.jPos[3]) + "," + std::to_string(jntpose.jPos[4]) + "," +\
            std::to_string(jntpose.jPos[5]));
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
 * @param [in] para-distance,vel
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
 * @param [in] para-ratio,angle,vel
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


/**
 * @brief 上传轨迹J文件
 * @param [in] para-filepath
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::TrajectoryJUpLoad(std::string para){
    //string filepath
    return std::to_string(_ptr_robot->TrajectoryJUpLoad(para));
}

/**
 * @brief 删除轨迹J文件
 * @param [in] para-filename
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::TrajectoryJDelete(std::string para){
    //const string filename
    return std::to_string(_ptr_robot->TrajectoryJDelete(para));
}

/**
 * @brief 加载轨迹J文件
 * @param [in] para-name[30],ovl
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LoadTrajectoryJ(std::string para){
    //char name[30], float ovl
    std::list<std::string> list;
    _splitString2List(para,list);
    char name[30];
    list.front().copy(name,list.front().size());list.pop_front();
    float ovl = std::stof(list.front());
    return std::to_string(_ptr_robot->LoadTrajectoryJ(name,ovl,1));
}

/**
 * @brief 运行轨迹J文件
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveTrajectoryJ(std::string para){
    //empty para
    return std::to_string(_ptr_robot->MoveTrajectoryJ());
}

/**
 * @brief 获取轨迹J文件第一个点的初始位姿
 * @param [in] para-name[30]
 * @return 初始点位姿 
 * @retval x,y,z,rx,ry,rz
 */
std::string robot_command_thread::GetTrajectoryStartPose(std::string para){
    //char name[30]
    char name[30];
    if(para.length() <= 29){
        para.copy(name,para.length());
        name[para.length()] = '\0';
    }
    DescPose pos;
    if(_ptr_robot->GetTrajectoryStartPose(name,&pos) == 0){
        return std::to_string(pos.tran.x) + "," +\
                std::to_string(pos.tran.y) + "," +\
                std::to_string(pos.tran.z) + "," +\
                std::to_string(pos.rpy.rx) + "," +\
                std::to_string(pos.rpy.ry) + "," +\   
                std::to_string(pos.rpy.rz);
    }else{
        return std::string("0,0,0,0,0,0");
    }
}

/**
 * @brief 获取轨迹J文件点数量
 * @return 点数
 * @retval num
 */
std::string robot_command_thread::GetTrajectoryPointNum(std::string para){
    //empty para
    int num;
    if(_ptr_robot->GetTrajectoryPointNum(&num) == 0){
        return std::to_string(num);
    }else{
        return "-1";
    }
}

/**
 * @brief 设置轨迹J运行速度
 * @param [in] para-vel
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTrajectoryJSpeed(std::string para){
    //double vel
    return std::to_string(_ptr_robot->SetTrajectoryJSpeed(std::stod(para)));
}


/**
 * @brief 下载LUA脚本
 * @param [in] para-filename,filepath
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LuaDownLoad(std::string para){
    //string filename,string filepath
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string name = list.front();list.pop_front();
    std::string path = list.front();
    return std::to_string(_ptr_robot->LuaDownLoad(name,path));
}

/**
 * @brief 上传LUA脚本
 * @param [in] para-filepath
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LuaUpload(std::string para){
    //string filepath
    int res = _ptr_robot->LuaUpload(para);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"上传LUA脚本名称为:%s",para.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"上传LUA脚本调用SDK的结果为:%i",res);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"LUA upload result:%i",res);
    return std::to_string(res);
}

/**
 * @brief 删除LUA脚本
 * @param [in] para-filename
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LuaDelete(std::string para){
    //string filename
    return std::to_string(_ptr_robot->LuaDelete(para));
}

/**
 * @brief 获取LUA脚本列表
 * @param [in] para-filename
 * @return LUA脚本名称列表
 * @retval name1,name2,name3...
 */
std::string robot_command_thread::GetLuaList(std::string para){
    std::list<std::string> list;
    if(_ptr_robot->GetLuaList(&list) == 0){
        std::string str = "";
        for(auto item : list){
            str += item;
            str += ",";
        }
        return str;
    }else{
        return "-1";
    }
}

/**
 * @brief 根据点位信息计算工具坐标系
 * @param [in] method,pos
 * @return 工具坐标系结果及错误码
 * @retval errorcode,x,y,z,rx,ry,rz
 */
std::string robot_command_thread::ComputeToolCoordWithPoints(std::string para){
    //int method,Jointpos pos
    std::list<std::string> list;
    _splitString2List(para,list);

    int method = std::stoi(list.front().c_str());list.pop_front();
    JointPos pos;
    _fillJointPose(list,pos);
    DescPose cartpos;
    int res = _ptr_robot->ComputeToolCoordWithPoints(method,&pos,cartpos);
    return std::string(std::to_string(res) + "," + std::to_string(cartpos.tran.x) + "," + \
            std::to_string(cartpos.tran.y) + "," + std::to_string(cartpos.tran.z) + "," + \
            std::to_string(cartpos.rpy.rx) + "," + std::to_string(cartpos.rpy.ry) + "," + \
            std::to_string(cartpos.rpy.rz));
}

/**
 * @brief 根据点位信息计算工件坐标系
 * @param [in] method,pos,reframe
 * @return 工件坐标系结果及错误码
 * @retval errorcode,x,y,z,rx,ry,rz
 */
std::string robot_command_thread::ComputeWObjCoordWithPoints(std::string para){
    //int method,descpos pos,int refframe
    std::list<std::string> list;
    _splitString2List(para,list);

    int method = std::stoi(list.front().c_str());list.pop_front();
    DescPose posin,posout;
    _fillDescPose(list,posin);
    int reframe = std::stoi(list.front().c_str());
    int res = _ptr_robot->ComputeWObjCoordWithPoints(method,&posin,reframe,posout);
    return std::string(std::to_string(res) + "," + std::to_string(posout.tran.x) + "," + \
            std::to_string(posout.tran.y) + "," + std::to_string(posout.tran.z) + "," + \
            std::to_string(posout.rpy.rx) + "," + std::to_string(posout.rpy.ry) + "," + \
            std::to_string(posout.rpy.rz));
}

/**
 * @brief 设置机器人焊接电弧意外中断检测参数
 * @param [in] checkEnable 是否使能检测；0-不使能；1-使能
 * @param [in] arcInterruptTimeLength 电弧中断确认时长(ms)
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeldingSetCheckArcInterruptionParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int checkenable = std::stoi(list.front().c_str());list.pop_front();
    int arcInterruptTimeLength  = std::stoi(list.front().c_str());
    return std::to_string(_ptr_robot->WeldingSetCheckArcInterruptionParam(\
            checkenable,arcInterruptTimeLength));
}

/**
 * @brief 获取机器人焊接电弧意外中断检测参数
 * @return 错误码及返回参数
 * @retval errorcode,checkenable,arcInterruptTimeLength
 */
std::string robot_command_thread::WeldingGetCheckArcInterruptionParam(std::string para){
    int checkenable,arcInterruptTimeLength;
    int res = _ptr_robot->WeldingGetCheckArcInterruptionParam(&checkenable,&arcInterruptTimeLength);
    return std::string(std::to_string(res) + "," + std::to_string(checkenable) + "," \
            + std::to_string(arcInterruptTimeLength));
}

/**
 * @brief 设置机器人焊接中断恢复参数
 * @param [in] enable 是否使能焊接中断恢复
 * @param [in] length 焊缝重叠距离(mm)
 * @param [in] velocity 机器人回到再起弧点速度百分比(0-100)
 * @param [in] moveType 机器人运动到再起弧点方式；0-LIN；1-PTP
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码
 */
std::string robot_command_thread::WeldingSetReWeldAfterBreakOffParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int enable = std::stoi(list.front().c_str());list.pop_front();
    double length = std::stod(list.front().c_str());list.pop_front();
    double velocity = std::stod(list.front().c_str());list.pop_front();
    int moveType = std::stoi(list.front().c_str());
    return std::to_string(_ptr_robot->WeldingSetReWeldAfterBreakOffParam(enable,length,velocity,moveType));
}

/**
 * @brief 获取机器人焊接中断恢复参数
 * @return 错误码及返回参数
 * @retval errorcode,enable,length,velocity,movetype
 */
std::string robot_command_thread::WeldingGetReWeldAfterBreakOffParam(std::string para){
    int enable,moveType;
    double length,velocity;
    int res = _ptr_robot->WeldingGetReWeldAfterBreakOffParam(&enable,&length,&velocity,&moveType);
    return std::string(std::to_string(res) + "," + std::to_string(enable) + "," + \
            std::to_string(length) + "," + std::to_string(velocity) + "," + \
            std::to_string(moveType));
}

/**
 * @brief 开始机器人焊接电弧意外中断
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeldingStartReWeldAfterBreakOff(std::string para){
    return std::to_string(_ptr_robot->WeldingStartReWeldAfterBreakOff());
}

/**
 * @brief 停止机器人焊接电弧意外中断
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeldingAbortWeldAfterBreakOff(std::string para){
    return std::to_string(_ptr_robot->WeldingAbortWeldAfterBreakOff());
}

/**
 * @brief 查询SDK版本号
 * @return SDK版本号
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetSDKVersion(std::string para){
    char version;
    int res = _ptr_robot->GetSDKVersion(&version);
    return std::string(std::to_string(res) + ",SDKVersion:" + std::to_string(version));
}

/**
 * @brief 查询控制器IP
 * @return 控制器IP
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetControllerIP(std::string para){
    char ip;
    int res = _ptr_robot->GetControllerIP(&ip);
    return std::string(std::to_string(res) + ",ControllerIP:" + std::to_string(ip));
}

/**
 * @brief 查询机器人是否处于拖动示教模式
 * @return 0-非拖动示教模式，1-拖动示教模式
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::IsInDragTeach(std::string para){
    uint8_t state;
    int res = _ptr_robot->IsInDragTeach(&state);
    return std::string(std::to_string(res) + "," + std::to_string(state));
}

/**
 * @brief 笛卡尔空间螺旋线运动
 * @param  [in] joint_pos  目标关节位置,单位deg
 * @param  [in] desc_pos   目标笛卡尔位姿
 * @param  [in] tool  工具坐标号，范围[0~14]
 * @param  [in] user  工件坐标号，范围[0~14]
 * @param  [in] vel  速度百分比，范围[0~100]
 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] epos  扩展轴位置，单位mm
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos  位姿偏移量
 * @param  [in] spiral_param  螺旋参数
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::NewSpiral(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    JointPos pos;
    _fillJointPose(list,pos);
    DescPose cartpose;
    _fillDescPose(list,cartpose);
    int tool = std::stoi(list.front().c_str());list.pop_front();
    int user = std::stoi(list.front().c_str());list.pop_front();
    float vel = std::stod(list.front().c_str());list.pop_front();
    float acc = std::stod(list.front().c_str());list.pop_front();
    ExaxisPos eaxispos;
    eaxispos.ePos[0] = std::stod(list.front().c_str());list.pop_front();
    eaxispos.ePos[1] = std::stod(list.front().c_str());list.pop_front();
    eaxispos.ePos[2] = std::stod(list.front().c_str());list.pop_front();
    eaxispos.ePos[3] = std::stod(list.front().c_str());list.pop_front();
    float ovl = std::stod(list.front().c_str());list.pop_front();
    uint8_t offset_flag = std::stod(list.front().c_str());list.pop_front();
    DescPose offsetpos;
    offsetpos.tran.x = std::stof(list.front());list.pop_front();
    offsetpos.tran.y = std::stof(list.front());list.pop_front();
    offsetpos.tran.z = std::stof(list.front());list.pop_front();
    offsetpos.rpy.rx = std::stof(list.front());list.pop_front();
    offsetpos.rpy.ry = std::stof(list.front());list.pop_front();
    offsetpos.rpy.rz = std::stof(list.front());
    SpiralParam spiral_param;
    spiral_param.circle_num = std::stoi(list.front().c_str());list.pop_front();
	spiral_param.circle_angle = std::stod(list.front().c_str());list.pop_front();
	spiral_param.rad_init = std::stod(list.front().c_str());list.pop_front();
	spiral_param.rad_add = std::stod(list.front().c_str());list.pop_front();
	spiral_param.rotaxis_add = std::stod(list.front().c_str());list.pop_front();
	spiral_param.rot_direction = std::stoi(list.front().c_str());list.pop_front();
	spiral_param.velAccMode = std::stoi(list.front().c_str());list.pop_front();
    
    return std::to_string(_ptr_robot->NewSpiral(&pos,&cartpose,tool,user,vel,acc,\
        &eaxispos,ovl,offset_flag,&offsetpos,spiral_param));
}

/**
 * @brief 伺服运动开始，配合ServoJ、ServoCart指令使用
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ServoMoveStart(std::string para){
    int res = _ptr_robot->ServoMoveStart();
    return std::string(std::to_string(res));
}

/**
 * @brief 伺服运动结束，配合ServoJ、ServoCart指令使用
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ServoMoveEnd(std::string para){
    int res = _ptr_robot->ServoMoveEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief  笛卡尔空间伺服模式运动
 * @param  [in]  mode  0-绝对运动(基坐标系)，1-增量运动(基坐标系)，2-增量运动(工具坐标系)
 * @param  [in]  desc_pos  目标笛卡尔位姿或位姿增量
 * @param  [in]  pos_gain  位姿增量比例系数，仅在增量运动下生效，范围[0~1]
 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放，默认为0
 * @param  [in] vel  速度百分比，范围[0~100]，暂不开放，默认为0
 * @param  [in] cmdT  指令下发周期，单位s，建议范围[0.001~0.0016]
 * @param  [in] filterT 滤波时间，单位s，暂不开放，默认为0
 * @param  [in] gain  目标位置的比例放大器，暂不开放，默认为0
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ServoCart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int mode = std::stoi(list.front().c_str());list.pop_front();
    DescPose desc_pose;
    _fillDescPose(list,desc_pose);
    float pos_gain[6];
    pos_gain[0] = std::stod(list.front().c_str());list.pop_front();
    pos_gain[1] = std::stod(list.front().c_str());list.pop_front();
    pos_gain[2] = std::stod(list.front().c_str());list.pop_front();
    pos_gain[3] = std::stod(list.front().c_str());list.pop_front();
    pos_gain[4] = std::stod(list.front().c_str());list.pop_front();
    pos_gain[5] = std::stod(list.front().c_str());list.pop_front();
    float acc = std::stod(list.front().c_str());list.pop_front();
    float vel = std::stod(list.front().c_str());list.pop_front();
    float cmdT = std::stod(list.front().c_str());list.pop_front();
    float filterT = std::stod(list.front().c_str());list.pop_front();
    float gain = std::stod(list.front().c_str());list.pop_front();
    
    return std::to_string(_ptr_robot->ServoCart(mode,&desc_pose,pos_gain,acc,vel,cmdT,\
        filterT,gain));
}

/**
 * @brief  笛卡尔空间点到点运动
 * @param  [in]  desc_pos  目标笛卡尔位姿或位姿增量
 * @param  [in] tool  工具坐标号，范围[0~14]
 * @param  [in] user  工件坐标号，范围[0~14]
 * @param  [in] vel  速度百分比，范围[0~100]
 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] blendT [-1.0]-运动到位(阻塞)，[0~500.0]-平滑时间(非阻塞)，单位ms
 * @param  [in] config  关节空间配置，[-1]-参考当前关节位置解算，[0~7]-参考特定关节空间配置解算，默认为-1
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveCart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    DescPose desc_pose;
    _fillDescPose(list,desc_pose);
    int tool = std::stoi(list.front().c_str());list.pop_front();
    int user = std::stoi(list.front().c_str());list.pop_front();
    
    float vel = std::stod(list.front().c_str());list.pop_front();
    float acc = std::stod(list.front().c_str());list.pop_front();
    float ovl = std::stod(list.front().c_str());list.pop_front();
    float blendT = std::stod(list.front().c_str());list.pop_front();
    int config = std::stoi(list.front().c_str());list.pop_front();
    
    return std::to_string(_ptr_robot->MoveCart(&desc_pose,tool,user,vel,acc,\
        ovl,blendT,config));
}

/**
 * @brief 暂停运动
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::PauseMotion(std::string para){
    int res = _ptr_robot->PauseMotion();
    return std::string(std::to_string(res));
}

/**
 * @brief 恢复运动
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ResumeMotion(std::string para){
    int res = _ptr_robot->ResumeMotion();
    return std::string(std::to_string(res));
}

/**
 * @brief 获取控制箱数字量输入
 * @param  [in] id  io编号，范围[0~15]
 * @param  [in] block  0-阻塞，1-非阻塞
 * @param  [out] result  0-低电平，1-高电平
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetDI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    uint8_t block = std::stoi(list.front().c_str());list.pop_front();
    uint8_t result;
    int res = _ptr_robot->GetDI(id,block,&result);
    return std::string(std::to_string(res) + "," + std::to_string(result));
}

/**
 * @brief 获取工具数字量输入
 * @param  [in] id  io编号，范围[0~1]
 * @param  [in] block  0-阻塞，1-非阻塞
 * @param  [out] result  0-低电平，1-高电平
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetToolDI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    uint8_t block = std::stoi(list.front().c_str());list.pop_front();
    uint8_t result;
    int res = _ptr_robot->GetToolDI(id,block,&result);
    return std::string(std::to_string(res) + "," + std::to_string(result));
}

/**
 * @brief 等待控制箱数字量输入
 * @param  [in] id  io编号，范围[0~15]
 * @param  [in]  status 0-关，1-开
 * @param  [in]  max_time  最大等待时间，单位ms
 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WaitDI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    uint8_t status = std::stoi(list.front().c_str());list.pop_front();
    int max_time = std::stoi(list.front().c_str());list.pop_front();
    int opt = std::stoi(list.front().c_str());list.pop_front();
    int res = _ptr_robot->WaitDI(id,status,max_time,opt);
    return std::string(std::to_string(res));
}

/**
 * @brief 等待控制箱多路数字量输入
 * @param  [in] mode 0-多路与，1-多路或
 * @param  [in] id  io编号，bit0~bit7对应DI0~DI7，bit8~bit15对应CI0~CI7
 * @param  [in]  status 0-关，1-开
 * @param  [in]  max_time  最大等待时间，单位ms
 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WaitMultiDI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int mode = std::stoi(list.front().c_str());list.pop_front();
    int id = std::stoi(list.front().c_str());list.pop_front();
    uint8_t status = std::stoi(list.front().c_str());list.pop_front();
    int max_time = std::stoi(list.front().c_str());list.pop_front();
    int opt = std::stoi(list.front().c_str());list.pop_front();
    int res = _ptr_robot->WaitMultiDI(mode,id,status,max_time,opt);
    return std::string(std::to_string(res));
}

/**
 * @brief 等待工具数字量输入
 * @param  [in] id  io编号，范围[0~1]
 * @param  [in]  status 0-关，1-开
 * @param  [in]  max_time  最大等待时间，单位ms
 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WaitToolDI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    uint8_t status = std::stoi(list.front().c_str());list.pop_front();
    int max_time = std::stoi(list.front().c_str());list.pop_front();
    int opt = std::stoi(list.front().c_str());list.pop_front();

    int res = _ptr_robot->WaitToolDI(id,status,max_time,opt);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取控制箱模拟量输入
 * @param  [in] id  io编号，范围[0~1]
 * @param  [in] block  0-阻塞，1-非阻塞
 * @param  [out] result  输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetAI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    uint8_t block = std::stoi(list.front().c_str());list.pop_front();
    float result;

    int res = _ptr_robot->GetAI(id,block,&result);
    return std::string(std::to_string(res) + "," + std::to_string(result));
}

/**
 * @brief  获取工具模拟量输入
 * @param  [in] id  io编号，范围[0]
 * @param  [in] block  0-阻塞，1-非阻塞
 * @param  [out] result  输入电流或电压值百分比，范围[0~100]对应电压[0~10V]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetToolAI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    uint8_t block = std::stoi(list.front().c_str());list.pop_front();
    float result;

    int res = _ptr_robot->GetToolAI(id,block,&result);
    return std::string(std::to_string(res) + "," + std::to_string(result));
}

/**
 * @brief 获取机器人末端点记录按钮状态
 * @param [out] state 按钮状态，0-按下，1-松开
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetAxlePointRecordBtnState(std::string para){
    uint8_t state;

    int res = _ptr_robot->GetAxlePointRecordBtnState(&state);
    return std::string(std::to_string(res) + "," + std::to_string(state));
}

/**
 * @brief 获取机器人末端DO输出状态
 * @param [out] do_state DO输出状态，do0~do1对应bit1~bit2,从bit0开始
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetToolDO(std::string para){
    uint8_t do_state;

    int res = _ptr_robot->GetToolDO(&do_state);
    return std::string(std::to_string(res) + "," + std::to_string(do_state));
}

/**
 * @brief 获取机器人控制器DO输出状态
 * @param [out] do_state_h DO输出状态，co0~co7对应bit0~bit7
 * @param [out] do_state_l DO输出状态，do0~do7对应bit0~bit7
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetDO(std::string para){
    uint8_t do_state_h;
    uint8_t do_state_l;

    int res = _ptr_robot->GetDO(&do_state_h,&do_state_l);
    return std::string(std::to_string(res) + "," + std::to_string(do_state_h) + "," + std::to_string(do_state_l));
}

/**
 * @brief 等待控制箱模拟量输入
 * @param  [in] id  io编号，范围[0~1]
 * @param  [in]  sign 0-大于，1-小于
 * @param  [in]  value 输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
 * @param  [in]  max_time  最大等待时间，单位ms
 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WaitAI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    int sign = std::stoi(list.front().c_str());list.pop_front();
    float value = std::stof(list.front().c_str());list.pop_front();
    int max_time = std::stoi(list.front().c_str());list.pop_front();
    int opt = std::stoi(list.front().c_str());list.pop_front();

    int res = _ptr_robot->WaitAI(id,sign,value,max_time,opt);
    return std::string(std::to_string(res));
}

/**
 * @brief 等待工具模拟量输入
 * @param  [in] id  io编号，范围[0]
 * @param  [in]  sign 0-大于，1-小于
 * @param  [in]  value 输入电流或电压值百分比，范围[0~100]对应电压[0~10V]
 * @param  [in]  max_time  最大等待时间，单位ms
 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WaitToolAI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    int sign = std::stoi(list.front().c_str());list.pop_front();
    float value = std::stof(list.front().c_str());list.pop_front();
    int max_time = std::stoi(list.front().c_str());list.pop_front();
    int opt = std::stoi(list.front().c_str());list.pop_front();

    int res = _ptr_robot->WaitToolAI(id,sign,value,max_time,opt);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置系统变量值
 * @param  [in]  id  变量编号，范围[1~20]
 * @param  [in]  value 变量值
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetSysVarValue(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();
    float value = std::stof(list.front().c_str());list.pop_front();

    int res = _ptr_robot->SetSysVarValue(id,value);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置工具参考点-六点法
 * @param [in] point_num 点编号,范围[1~6]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetToolPoint(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int point_num = std::stoi(list.front().c_str());list.pop_front();

    int res = _ptr_robot->SetToolPoint(point_num);
    return std::string(std::to_string(res));
}

/**
 * @brief  计算工具坐标系
 * @param [out] tcp_pose 工具坐标系
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ComputeTool(std::string para){
    DescPose tcp_pose;
    int res = _ptr_robot->ComputeTool(&tcp_pose);
    return std::string(std::to_string(res) + "," + std::to_string(tcp_pose.tran.x) + "," + \
            std::to_string(tcp_pose.tran.y) + "," + std::to_string(tcp_pose.tran.z) + "," + \
            std::to_string(tcp_pose.rpy.rx) + "," + std::to_string(tcp_pose.rpy.ry) + "," + \
            std::to_string(tcp_pose.rpy.rz));
}

/**
 * @brief 设置工具参考点-四点法
 * @param [in] point_num 点编号,范围[1~4]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTcp4RefPoint(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int point_num = std::stoi(list.front().c_str());list.pop_front();

    int res = _ptr_robot->SetTcp4RefPoint(point_num);
    return std::string(std::to_string(res));
}

/**
 * @brief  计算工具坐标系
 * @param [out] tcp_pose 工具坐标系
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ComputeTcp4(std::string para){
    DescPose tcp_pose;
    int res = _ptr_robot->ComputeTcp4(&tcp_pose);
    return std::string(std::to_string(res) + "," + std::to_string(tcp_pose.tran.x) + "," + \
            std::to_string(tcp_pose.tran.y) + "," + std::to_string(tcp_pose.tran.z) + "," + \
            std::to_string(tcp_pose.rpy.rx) + "," + std::to_string(tcp_pose.rpy.ry) + "," + \
            std::to_string(tcp_pose.rpy.rz));
}

/**
 * @brief 设置外部工具参考点-六点法
 * @param [in] point_num 点编号,范围[1~4]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetExTCPPoint(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int point_num = std::stoi(list.front().c_str());list.pop_front();

    int res = _ptr_robot->SetExTCPPoint(point_num);
    return std::string(std::to_string(res));
}

/**
 * @brief  计算外部工具坐标系
 * @param [out] tcp_pose 外部工具坐标系
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ComputeExTCF(std::string para){
    DescPose tcp_pose;
    int res = _ptr_robot->ComputeExTCF(&tcp_pose);
    return std::string(std::to_string(res) + "," + std::to_string(tcp_pose.tran.x) + "," + \
            std::to_string(tcp_pose.tran.y) + "," + std::to_string(tcp_pose.tran.z) + "," + \
            std::to_string(tcp_pose.rpy.rx) + "," + std::to_string(tcp_pose.rpy.ry) + "," + \
            std::to_string(tcp_pose.rpy.rz));
}

/**
 * @brief 设置工件参考点-三点法
 * @param [in] point_num 点编号,范围[1~3]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetWObjCoordPoint(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int point_num = std::stoi(list.front().c_str());list.pop_front();

    int res = _ptr_robot->SetWObjCoordPoint(point_num);
    return std::string(std::to_string(res));
}

/**
 * @brief  计算工件坐标系
 * @param [in] method 计算方法 0：原点-x轴-z轴  1：原点-x轴-xy平面
 * @param [in] refFrame 参考坐标系
 * @param [out] wobj_pose 工件坐标系
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ComputeWObjCoord(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int method = std::stoi(list.front().c_str());list.pop_front();
    int refFrame = std::stoi(list.front().c_str());list.pop_front();

    DescPose wobj_pose;
    int res = _ptr_robot->ComputeWObjCoord(method,refFrame,&wobj_pose);
    return std::string(std::to_string(res) + "," + std::to_string(wobj_pose.tran.x) + "," + \
            std::to_string(wobj_pose.tran.y) + "," + std::to_string(wobj_pose.tran.z) + "," + \
            std::to_string(wobj_pose.rpy.rx) + "," + std::to_string(wobj_pose.rpy.ry) + "," + \
            std::to_string(wobj_pose.rpy.rz));
}

/**
 * @brief  等待指定时间
 * @param  [in]  t_ms  单位ms
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WaitMs(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int t_ms = std::stoi(list.front().c_str());list.pop_front();

    int res = _ptr_robot->WaitMs(t_ms);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置负限位
 * @param  [in] limit 六个关节位置，单位deg
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetLimitNegative(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    float limit[6];
    limit[0] = std::stod(list.front().c_str());list.pop_front();
    limit[1] = std::stod(list.front().c_str());list.pop_front();
    limit[2] = std::stod(list.front().c_str());list.pop_front();
    limit[3] = std::stod(list.front().c_str());list.pop_front();
    limit[4] = std::stod(list.front().c_str());list.pop_front();
    limit[5] = std::stod(list.front().c_str());list.pop_front();


    int res = _ptr_robot->SetLimitNegative(limit);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取机器人安装角度
 * @param  [out] yangle 倾斜角
 * @param  [out] zangle 旋转角
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetRobotInstallAngle(std::string para){
    float yangle;
    float zangle;

    int res = _ptr_robot->GetRobotInstallAngle(&yangle,&zangle);
    return std::string(std::to_string(res) + "," + std::to_string(yangle) + "," + std::to_string(zangle));
}

/**
 * @brief  获取系统变量值
 * @param  [in] id 系统变量编号，范围[1~20]
 * @param  [out] value  系统变量值
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetSysVarValue(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front().c_str());list.pop_front();

    float value;

    int res = _ptr_robot->GetSysVarValue(id,&value);
    return std::string(std::to_string(res) + "," + std::to_string(value));
}

/**
 * @brief  获取当前关节位置(角度)
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] jPos 六个关节位置，单位deg
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetActualJointPosDegree(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    JointPos jPos;

    int res = _ptr_robot->GetActualJointPosDegree(flag,&jPos);
    return std::string(std::to_string(res) + "," + std::to_string(jPos.jPos[0]) + "," +\
            std::to_string(jPos.jPos[1]) + "," + std::to_string(jPos.jPos[2]) + "," +\
            std::to_string(jPos.jPos[3]) + "," + std::to_string(jPos.jPos[4]) + "," +\
            std::to_string(jPos.jPos[5]));
}

/**
 * @brief  获取关节反馈速度-deg/s
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] speed [x,y,z,rx,ry,rz]速度
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetActualJointSpeedsDegree(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    float speed[6];

    int res = _ptr_robot->GetActualJointSpeedsDegree(flag,speed);
    return std::string(std::to_string(res) + "," + std::to_string(speed[0]) + "," +\
            std::to_string(speed[1]) + "," + std::to_string(speed[2]) + "," +\
            std::to_string(speed[3]) + "," + std::to_string(speed[4]) + "," +\
            std::to_string(speed[5]));
}

/**
 * @brief  获取关节反馈加速度-deg/s^2
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] acc 六个关节加速度
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetActualJointAccDegree(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    float acc[6];

    int res = _ptr_robot->GetActualJointAccDegree(flag,acc);
    return std::string(std::to_string(res) + "," + std::to_string(acc[0]) + "," +\
            std::to_string(acc[1]) + "," + std::to_string(acc[2]) + "," +\
            std::to_string(acc[3]) + "," + std::to_string(acc[4]) + "," +\
            std::to_string(acc[5]));
}

/**
 * @brief  获取TCP指令速度
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] tcp_speed 线性速度
 * @param  [out] ori_speed 姿态速度
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetTargetTCPCompositeSpeed(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    float tcp_speed;
    float ori_speed;

    int res = _ptr_robot->GetTargetTCPCompositeSpeed(flag,&tcp_speed,&ori_speed);
    return std::string(std::to_string(res) + "," + std::to_string(tcp_speed) + "," +\
            std::to_string(ori_speed));
}

/**
 * @brief  获取TCP反馈速度
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] tcp_speed 线性速度
 * @param  [out] ori_speed 姿态速度
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetActualTCPCompositeSpeed(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    float tcp_speed;
    float ori_speed;

    int res = _ptr_robot->GetActualTCPCompositeSpeed(flag,&tcp_speed,&ori_speed);
    return std::string(std::to_string(res) + "," + std::to_string(tcp_speed) + "," +\
            std::to_string(ori_speed));
}

/**
 * @brief  获取TCP指令速度
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] speed [x,y,z,rx,ry,rz]速度
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetTargetTCPSpeed(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    float speed[6];

    int res = _ptr_robot->GetTargetTCPSpeed(flag,speed);
    return std::string(std::to_string(res) + "," + std::to_string(speed[0]) + "," +\
            std::to_string(speed[1]) + "," + std::to_string(speed[2]) + "," +\
            std::to_string(speed[3]) + "," + std::to_string(speed[4]) + "," +\
            std::to_string(speed[5]));
}

/**
 * @brief  获取TCP反馈速度
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] speed [x,y,z,rx,ry,rz]速度
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetActualTCPSpeed(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    float speed[6];

    int res = _ptr_robot->GetActualTCPSpeed(flag,speed);
    return std::string(std::to_string(res) + "," + std::to_string(speed[0]) + "," +\
            std::to_string(speed[1]) + "," + std::to_string(speed[2]) + "," +\
            std::to_string(speed[3]) + "," + std::to_string(speed[4]) + "," +\
            std::to_string(speed[5]));
}

/**
 * @brief  获取当前工具位姿
 * @param  [in] flag  0-阻塞，1-非阻塞
 * @param  [out] desc_pos  工具位姿
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetActualTCPPose(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    DescPose desc_pos;
    int res = _ptr_robot->GetActualTCPPose(flag,&desc_pos);
    return std::string(std::to_string(res) + "," + std::to_string(desc_pos.tran.x) + "," + \
            std::to_string(desc_pos.tran.y) + "," + std::to_string(desc_pos.tran.z) + "," + \
            std::to_string(desc_pos.rpy.rx) + "," + std::to_string(desc_pos.rpy.ry) + "," + \
            std::to_string(desc_pos.rpy.rz));
}

/**
 * @brief  获取当前工具坐标系编号
 * @param  [in] flag  0-阻塞，1-非阻塞
 * @param  [out] id  工具坐标系编号
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetActualTCPNum(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    int id;
    int res = _ptr_robot->GetActualTCPNum(flag,&id);
    return std::string(std::to_string(res) + "," + std::to_string(id));
}

/**
 * @brief  获取当前工件坐标系编号
 * @param  [in] flag  0-阻塞，1-非阻塞
 * @param  [out] id  工件坐标系编号
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetActualWObjNum(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    int id;
    int res = _ptr_robot->GetActualWObjNum(flag,&id);
    return std::string(std::to_string(res) + "," + std::to_string(id));
}

/**
 * @brief  获取当前末端法兰位姿
 * @param  [in] flag  0-阻塞，1-非阻塞
 * @param  [out] desc_pos  法兰位姿
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetActualToolFlangePose(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front().c_str());list.pop_front();

    DescPose desc_pos;
    int res = _ptr_robot->GetActualToolFlangePose(flag,&desc_pos);
    return std::string(std::to_string(res) + "," + std::to_string(desc_pos.tran.x) + "," + \
            std::to_string(desc_pos.tran.y) + "," + std::to_string(desc_pos.tran.z) + "," + \
            std::to_string(desc_pos.rpy.rx) + "," + std::to_string(desc_pos.rpy.ry) + "," + \
            std::to_string(desc_pos.rpy.rz));
}

/**
 * @brief  逆运动学求解，参考指定关节位置求解
 * @param  [in] type 0-绝对位姿(基坐标系)，1-增量位姿(基坐标系)，2-增量位姿(工具坐标系)
 * @param  [in] desc_pos 笛卡尔位姿
 * @param  [in] joint_pos_ref 参考关节位置
 * @param  [out] joint_pos 关节位置
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetInverseKinRef(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front());list.pop_front();
    DescPose desc_pos;
    _fillDescPose(list,desc_pos);
    JointPos joint_pos_ref;
    _fillJointPose(list,joint_pos_ref);

    JointPos jntpose;
    int res = _ptr_robot->GetInverseKinRef(type,&desc_pos,&joint_pos_ref,&jntpose);
    return std::string(std::to_string(res) + "," + std::to_string(jntpose.jPos[0]) + "," +\
            std::to_string(jntpose.jPos[1]) + "," + std::to_string(jntpose.jPos[2]) + "," +\
            std::to_string(jntpose.jPos[3]) + "," + std::to_string(jntpose.jPos[4]) + "," +\
            std::to_string(jntpose.jPos[5]));
}

/**
 * @brief  逆运动学求解，参考指定关节位置判断是否有解
 * @param  [in] posMode 0 绝对位姿，1 相对位姿-基坐标系，2 相对位姿-工具坐标系
 * @param  [in] desc_pos 笛卡尔位姿
 * @param  [in] joint_pos_ref 参考关节位置
 * @param  [out] result 0-无解，1-有解
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetInverseKinHasSolution(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front());list.pop_front();
    DescPose desc_pos;
    _fillDescPose(list,desc_pos);
    JointPos joint_pos_ref;
    _fillJointPose(list,joint_pos_ref);

    uint8_t result;
    int res = _ptr_robot->GetInverseKinHasSolution(type,&desc_pos,&joint_pos_ref,&result);
    return std::string(std::to_string(res) + "," + std::to_string(result));
}

/**
 * @brief  正运动学求解
 * @param  [in] joint_pos 关节位置
 * @param  [out] desc_pos 笛卡尔位姿
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetForwardKin(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    JointPos joint_pos;
    _fillJointPose(list,joint_pos);

    DescPose desc_pos;
    int res = _ptr_robot->GetForwardKin(&joint_pos,&desc_pos);
    return std::string(std::to_string(res) + "," + std::to_string(desc_pos.tran.x) + "," + \
            std::to_string(desc_pos.tran.y) + "," + std::to_string(desc_pos.tran.z) + "," + \
            std::to_string(desc_pos.rpy.rx) + "," + std::to_string(desc_pos.rpy.ry) + "," + \
            std::to_string(desc_pos.rpy.rz));
}

/**
 * @brief 获取当前关节转矩
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] torques 关节转矩
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetJointTorques(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front());list.pop_front();

    float torques[6];
    int res = _ptr_robot->GetJointTorques(flag,torques);
    return std::string(std::to_string(res) + "," + std::to_string(torques[0]) + "," +\
            std::to_string(torques[1]) + "," + std::to_string(torques[2]) + "," +\
            std::to_string(torques[3]) + "," + std::to_string(torques[4]) + "," +\
            std::to_string(torques[5]));
}

/**
 * @brief  获取当前负载的重量
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] weight 负载重量，单位kg
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetTargetPayload(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front());list.pop_front();

    float weight;
    int res = _ptr_robot->GetTargetPayload(flag,&weight);
    return std::string(std::to_string(res) + "," + std::to_string(weight));
}

/**
 * @brief  获取当前负载的质心
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] cog 负载质心，单位mm
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetTargetPayloadCog(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front());list.pop_front();

    DescTran cog;
    int res = _ptr_robot->GetTargetPayloadCog(flag,&cog);
    return std::string(std::to_string(res) + "," + std::to_string(cog.x) + "," +\
            std::to_string(cog.y) + "," + std::to_string(cog.z));
}

/**
 * @brief  获取当前工件坐标系
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] desc_pos 工件坐标系位姿
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetWObjOffset(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front());list.pop_front();

    DescPose desc_pos;
    int res = _ptr_robot->GetWObjOffset(flag,&desc_pos);
    return std::string(std::to_string(res) + "," + std::to_string(desc_pos.tran.x) + "," + \
            std::to_string(desc_pos.tran.y) + "," + std::to_string(desc_pos.tran.z) + "," + \
            std::to_string(desc_pos.rpy.rx) + "," + std::to_string(desc_pos.rpy.ry) + "," + \
            std::to_string(desc_pos.rpy.rz));
}

/**
 * @brief  获取关节软限位角度
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] negative  负限位角度，单位deg
 * @param  [out] positive  正限位角度，单位deg
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetJointSoftLimitDeg(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front());list.pop_front();

    float negative[6];
    float positive[6];
    int res = _ptr_robot->GetJointSoftLimitDeg(flag,negative,positive);
    return std::string(std::to_string(res) + "," + std::to_string(negative[0]) + "," +\
            std::to_string(negative[1]) + "," + std::to_string(negative[2]) + "," +\
            std::to_string(negative[3]) + "," + std::to_string(negative[4]) + "," +\
            std::to_string(negative[5]) + "," + std::to_string(positive[0]) + "," +\
            std::to_string(positive[1]) + "," + std::to_string(positive[2]) + "," +\
            std::to_string(positive[3]) + "," + std::to_string(positive[4]) + "," +\
            std::to_string(positive[5]));
}

/**
 * @brief  获取系统时间
 * @param  [out] t_ms 单位ms
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetSystemClock(std::string para){
    float t_ms;
    int res = _ptr_robot->GetSystemClock(&t_ms);
    return std::string(std::to_string(res) + "," + std::to_string(t_ms));
}

/**
 * @brief  获取机器人当前关节位置
 * @param  [out]  config  关节空间配置，范围[0~7]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetRobotCurJointsConfig(std::string para){
    int config;
    int res = _ptr_robot->GetRobotCurJointsConfig(&config);
    return std::string(std::to_string(res) + "," + std::to_string(config));
}

/**
 * @brief  获取机器人当前速度
 * @param  [out]  vel  速度，单位mm/s
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetDefaultTransVel(std::string para){
    float vel;
    int res = _ptr_robot->GetDefaultTransVel(&vel);
    return std::string(std::to_string(res) + "," + std::to_string(vel));
}

/**
 * @brief  查询机器人运动是否完成
 * @param  [out]  state  0-未完成，1-完成
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetRobotMotionDone(std::string para){
    uint8_t state;
    int res = _ptr_robot->GetRobotMotionDone(&state);
    return std::string(std::to_string(res) + "," + std::to_string(state));
}

/**
 * @brief  查询机器人错误码
 * @param  [out]  maincode  主错误码
 * @param  [out]  subcode   子错误码
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetRobotErrorCode(std::string para){
    int maincode;
    int subcode;
    int res = _ptr_robot->GetRobotErrorCode(&maincode,&subcode);
    return std::string(std::to_string(res) + "," + std::to_string(maincode) + "," + std::to_string(subcode));
}

/**
 * @brief  查询机器人示教管理点位数据
 * @param  [in]  name  点位名
 * @param  [out]  data   点位数据
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetRobotTeachingPoint(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    char name[64];
    list.front().copy(name,list.front().size());list.pop_front();

    float data[20];
    int res = _ptr_robot->GetRobotTeachingPoint(name,data);
    return std::string(std::to_string(res) + "," + std::to_string(data[0]) + "," +\
            std::to_string(data[1]) + "," + std::to_string(data[2]) + "," +\
            std::to_string(data[3]) + "," + std::to_string(data[4]) + "," +\
            std::to_string(data[5]) + "," + std::to_string(data[6]) + "," +\
            std::to_string(data[7]) + "," + std::to_string(data[8]) + "," +\
            std::to_string(data[9]) + "," + std::to_string(data[10]) + "," +\
            std::to_string(data[11]) + "," + std::to_string(data[12]) + "," +\
            std::to_string(data[13]) + "," + std::to_string(data[14]) + "," +\
            std::to_string(data[15]) + "," + std::to_string(data[16]) + "," +\
            std::to_string(data[17]) + "," + std::to_string(data[18]) + "," +\
            std::to_string(data[19]));
}

/**
 * @brief  查询机器人运动队列缓存长度
 * @param  [out]  len  缓存长度
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetMotionQueueLength(std::string para){
    int len;
    int res = _ptr_robot->GetMotionQueueLength(&len);
    return std::string(std::to_string(res) + "," + std::to_string(len));
}

/**
 * @brief  设置轨迹记录参数
 * @param  [in] type  记录数据类型，1-关节位置
 * @param  [in] name  轨迹文件名
 * @param  [in] period_ms  数据采样周期，固定值2ms或4ms或8ms
 * @param  [in] di_choose  DI选择,bit0~bit7对应控制箱DI0~DI7，bit8~bit9对应末端DI0~DI1，0-不选择，1-选择
 * @param  [in] do_choose  DO选择,bit0~bit7对应控制箱DO0~DO7，bit8~bit9对应末端DO0~DO1，0-不选择，1-选择
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTPDParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front());list.pop_front();
    char name[30];
    list.front().copy(name,list.front().size());list.pop_front();
    int period_ms = std::stoi(list.front());list.pop_front();
    uint16_t di_choose = std::stoi(list.front());list.pop_front();
    uint16_t do_choose = std::stoi(list.front());list.pop_front();

    float data[20];
    int res = _ptr_robot->SetTPDParam(type,name,period_ms,di_choose,do_choose);
    return std::string(std::to_string(res));
}

/**
 * @brief  开始轨迹记录
 * @param  [in] type  记录数据类型，1-关节位置
 * @param  [in] name  轨迹文件名
 * @param  [in] period_ms  数据采样周期，固定值2ms或4ms或8ms
 * @param  [in] di_choose  DI选择,bit0~bit7对应控制箱DI0~DI7，bit8~bit9对应末端DI0~DI1，0-不选择，1-选择
 * @param  [in] do_choose  DO选择,bit0~bit7对应控制箱DO0~DO7，bit8~bit9对应末端DO0~DO1，0-不选择，1-选择
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTPDStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front());list.pop_front();
    char name[30];
    list.front().copy(name,list.front().size());list.pop_front();
    int period_ms = std::stoi(list.front());list.pop_front();
    uint16_t di_choose = std::stoi(list.front());list.pop_front();
    uint16_t do_choose = std::stoi(list.front());list.pop_front();

    float data[20];
    int res = _ptr_robot->SetTPDStart(type,name,period_ms,di_choose,do_choose);
    return std::string(std::to_string(res));
}

/**
 * @brief  停止轨迹记录
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetWebTPDStop(std::string para){
    int res = _ptr_robot->SetWebTPDStop();
    return std::string(std::to_string(res));
}

/**
 * @brief  删除轨迹记录
 * @param  [in] name  轨迹文件名
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTPDDelete(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    
    char name[30];
    list.front().copy(name,list.front().size());list.pop_front();

    int res = _ptr_robot->SetTPDDelete(name);
    return std::string(std::to_string(res));
}

/**
 * @brief  轨迹预加载
 * @param  [in] name  轨迹文件名
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LoadTPD(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    char name[30];
    list.front().copy(name,list.front().size());list.pop_front();
    
    int res = _ptr_robot->LoadTPD(name);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取轨迹起始位姿
 * @param  [in] name 轨迹文件名
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetTPDStartPose(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    char name[30];
    list.front().copy(name,list.front().size());list.pop_front();
    
    DescPose desc_pose;
    int res = _ptr_robot->GetTPDStartPose(name,&desc_pose);
    return std::string(std::to_string(res) + "," + std::to_string(desc_pose.tran.x) + "," + \
            std::to_string(desc_pose.tran.y) + "," + std::to_string(desc_pose.tran.z) + "," + \
            std::to_string(desc_pose.rpy.rx) + "," + std::to_string(desc_pose.rpy.ry) + "," + \
            std::to_string(desc_pose.rpy.rz));
}

/**
 * @brief  轨迹复现
 * @param  [in] name  轨迹文件名
 * @param  [in] blend 0-不平滑，1-平滑
 * @param  [in] ovl  速度缩放百分比，范围[0~100]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveTPD(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    char name[30];
    list.front().copy(name,list.front().size());list.pop_front();
    uint8_t blend = std::stoi(list.front());list.pop_front();
    float ovl = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->MoveTPD(name,blend,ovl);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置轨迹运行中的力和扭矩
 * @param  [in] ft 三个方向的力和扭矩，单位N和Nm
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTrajectoryJForceTorque(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    ForceTorque ft;
    ft.fx = std::stod(list.front());list.pop_front();
    ft.fy = std::stod(list.front());list.pop_front();
    ft.fz = std::stod(list.front());list.pop_front();
    ft.tx = std::stod(list.front());list.pop_front();
    ft.ty = std::stod(list.front());list.pop_front();
    ft.tz = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->SetTrajectoryJForceTorque(&ft);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置轨迹运行中的沿x方向的力
 * @param  [in] fx 沿x方向的力，单位N
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTrajectoryJForceFx(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    double fx = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->SetTrajectoryJForceFx(fx);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置轨迹运行中的沿y方向的力
 * @param  [in] fy 沿y方向的力，单位N
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTrajectoryJForceFy(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    double fy = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->SetTrajectoryJForceFy(fy);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置轨迹运行中的沿z方向的力
 * @param  [in] fz 沿x方向的力，单位N
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTrajectoryJForceFz(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    double fz = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->SetTrajectoryJForceFz(fz);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置轨迹运行中的绕x轴的扭矩
 * @param  [in] tx 绕x轴的扭矩，单位Nm
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTrajectoryJTorqueTx(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    double tx = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->SetTrajectoryJTorqueTx(tx);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置轨迹运行中的绕y轴的扭矩
 * @param  [in] ty 绕y轴的扭矩，单位Nm
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTrajectoryJTorqueTy(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    double ty = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->SetTrajectoryJTorqueTy(ty);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置轨迹运行中的绕z轴的扭矩
 * @param  [in] tz 绕z轴的扭矩，单位Nm
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetTrajectoryJTorqueTz(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    double tz = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->SetTrajectoryJTorqueTz(tz);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置开机自动加载默认的作业程序
 * @param  [in] flag  0-开机不自动加载默认程序，1-开机自动加载默认程序
 * @param  [in] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LoadDefaultProgConfig(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front());list.pop_front();
    char program_name[64];
    list.front().copy(program_name,list.front().size());list.pop_front();
    
    int res = _ptr_robot->LoadDefaultProgConfig(flag,program_name);
    return std::string(std::to_string(res));
}

/**
 * @brief  加载指定的作业程序
 * @param  [in] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ProgramLoad(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    char program_name[64];
    list.front().copy(program_name,list.front().size());list.pop_front();
    
    int res = _ptr_robot->ProgramLoad(program_name);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取已加载的作业程序名
 * @param  [out] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetLoadedProgram(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    char program_name[64];
    list.front().copy(program_name,list.front().size());list.pop_front();
    
    int res = _ptr_robot->GetLoadedProgram(program_name);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取当前机器人作业程序执行的行号
 * @param  [out] line  行号
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetCurrentLine(std::string para){
    int line;     
    int res = _ptr_robot->GetCurrentLine(&line);
    return std::string(std::to_string(res) + "," + std::to_string(line));
}

/**
 * @brief  运行当前加载的作业程序
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ProgramRun(std::string para){
    int res = _ptr_robot->ProgramRun();
    return std::string(std::to_string(res));
}

/**
 * @brief  暂停当前运行的作业程序
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ProgramPause(std::string para){
    int res = _ptr_robot->ProgramPause();
    return std::string(std::to_string(res));
}

/**
 * @brief  恢复当前暂停的作业程序
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ProgramResume(std::string para){
    int res = _ptr_robot->ProgramResume();
    return std::string(std::to_string(res));
}

/**
 * @brief  终止当前运行的作业程序
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ProgramStop(std::string para){
    int res = _ptr_robot->ProgramStop();
    return std::string(std::to_string(res));
}

/**
 * @brief  获取机器人作业程序执行状态
 * @param  [out]  state 1-程序停止或无程序运行，2-程序运行中，3-程序暂停
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetProgramState(std::string para){
    uint8_t state;
    int res = _ptr_robot->GetProgramState(&state);
    return std::string(std::to_string(res) + "," + std::to_string(state));
}

/**
 * @brief  配置夹爪
 * @param  [in] company  夹爪厂商，1-Robotiq，2-慧灵，3-天机，4-大寰，5-知行
 * @param  [in] device  设备号，Robotiq(0-2F-85系列)，慧灵(0-NK系列,1-Z-EFG-100)，天机(0-TEG-110)，大寰(0-PGI-140)，知行(0-CTPM2F20)
 * @param  [in] softvesion  软件版本号，暂不使用，默认为0
 * @param  [in] bus 设备挂在末端总线位置，暂不使用，默认为0
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetGripperConfig(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int company = std::stoi(list.front());list.pop_front();
    int device = std::stoi(list.front());list.pop_front();
    int softvesion = std::stoi(list.front());list.pop_front();
    int bus = std::stoi(list.front());list.pop_front();


    int res = _ptr_robot->SetGripperConfig(company,device,softvesion,bus);
    return std::string(std::to_string(res));
}

/**
 *@brief  获取夹爪配置
 *@param  [out] company  夹爪厂商，1-Robotiq，2-慧灵，3-天机，4-大寰，5-知行
 *@param  [out] device  设备号，Robotiq(0-2F-85系列)，慧灵(0-NK系列,1-Z-EFG-100)，天机(0-TEG-110)，大寰(0-PGI-140)，知行(0-CTPM2F20)
 *@param  [out] softvesion  软件版本号，暂不使用，默认为0
 *@param  [out] bus 设备挂在末端总线位置，暂不使用，默认为0
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperConfig(std::string para){
    int company;
    int device;
    int softvesion;
    int bus;

    int res = _ptr_robot->GetGripperConfig(&company,&device,&softvesion,&bus);
    return std::string(std::to_string(res) + "," + std::to_string(company) + "," + \
            std::to_string(device) + "," + std::to_string(softvesion) + "," + \
            std::to_string(bus));
}

/**
 * @brief  获取夹爪运动状态
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] staus  0-运动未完成，1-运动完成
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperMotionDone(std::string para){
    uint16_t fault;
    uint8_t staus;

    int res = _ptr_robot->GetGripperMotionDone(&fault,&staus);
    return std::string(std::to_string(res) + "," + std::to_string(fault) + "," + \
            std::to_string(staus));
}

/**
 * @brief  获取夹爪激活状态
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] status  bit0~bit15对应夹爪编号0~15，bit=0为未激活，bit=1为激活
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperActivateStatus(std::string para){
    uint16_t fault;
    uint16_t status;

    int res = _ptr_robot->GetGripperActivateStatus(&fault,&status);
    return std::string(std::to_string(res) + "," + std::to_string(fault) + "," + \
            std::to_string(status));
}

/**
 * @brief  获取夹爪位置
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] position  位置百分比，范围0~100%
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperCurPosition(std::string para){
    uint16_t fault;
    uint8_t position;

    int res = _ptr_robot->GetGripperCurPosition(&fault,&position);
    return std::string(std::to_string(res) + "," + std::to_string(fault) + "," + \
            std::to_string(position));
}

/**
 * @brief  获取夹爪速度
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] speed  速度百分比，范围0~100%
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperCurSpeed(std::string para){
    uint16_t fault;
    int8_t speed;

    int res = _ptr_robot->GetGripperCurSpeed(&fault,&speed);
    return std::string(std::to_string(res) + "," + std::to_string(fault) + "," + \
            std::to_string(speed));
}

/**
 * @brief  获取夹爪电流
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] current  电流百分比，范围0~100%
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperCurCurrent(std::string para){
    uint16_t fault;
    int8_t current;

    int res = _ptr_robot->GetGripperCurCurrent(&fault,&current);
    return std::string(std::to_string(res) + "," + std::to_string(fault) + "," + \
            std::to_string(current));
}

/**
 * @brief  获取夹爪电压
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] voltage  电压,单位0.1V
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperVoltage(std::string para){
    uint16_t fault;
    int voltage;

    int res = _ptr_robot->GetGripperVoltage(&fault,&voltage);
    return std::string(std::to_string(res) + "," + std::to_string(fault) + "," + \
            std::to_string(voltage));
}

/**
 * @brief  获取夹爪温度
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] temp  温度，单位℃
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperTemp(std::string para){
    uint16_t fault;
    int temp;

    int res = _ptr_robot->GetGripperTemp(&fault,&temp);
    return std::string(std::to_string(res) + "," + std::to_string(fault) + "," + \
            std::to_string(temp));
}

/**
 * @brief  获取旋转夹爪的旋转圈数
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] num  旋转圈数
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperRotNum(std::string para){
    uint16_t fault;
    double num;

    int res = _ptr_robot->GetGripperRotNum(&fault,&num);
    return std::string(std::to_string(res) + "," + std::to_string(fault) + "," + \
            std::to_string(num));
}

/**
 * @brief  获取旋转夹爪的旋转速度
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] speed  旋转速度百分比
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperRotSpeed(std::string para){
    uint16_t fault;
    int speed;

    int res = _ptr_robot->GetGripperRotSpeed(&fault,&speed);
    return std::string(std::to_string(res) + "," + std::to_string(fault) + "," + \
            std::to_string(speed));
}

/**
 * @brief  获取旋转夹爪的旋转力矩
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] torque  旋转力矩百分比
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetGripperRotTorque(std::string para){
    uint16_t fault;
    int torque;

    int res = _ptr_robot->GetGripperRotTorque(&fault,&torque);
    return std::string(std::to_string(res) + "," + std::to_string(fault) + "," + \
            std::to_string(torque));
}

/**
 * @brief  计算预抓取点-视觉
 * @param  [in] desc_pos  抓取点笛卡尔位姿
 * @param  [in] zlength   z轴偏移量
 * @param  [in] zangle    绕z轴旋转偏移量
 * @param  [out] pre_pos  预抓取点
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ComputePrePick(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    DescPose desc_pos;
    _fillDescPose(list,desc_pos);
    double zlength = std::stod(list.front());list.pop_front();
    double zangle = std::stod(list.front());list.pop_front();

    DescPose pre_pos;

    int res = _ptr_robot->ComputePrePick(&desc_pos,zlength,zangle,&pre_pos);
    return std::string(std::to_string(res) + "," + std::to_string(pre_pos.tran.x) + "," + \
            std::to_string(pre_pos.tran.y) + "," + std::to_string(pre_pos.tran.z) + "," + \
            std::to_string(pre_pos.rpy.rx) + "," + std::to_string(pre_pos.rpy.ry) + "," + \
            std::to_string(pre_pos.rpy.rz));
}

/**
 * @brief  计算撤退点-视觉
 * @param  [in] desc_pos  抓取点笛卡尔位姿
 * @param  [in] zlength   z轴偏移量
 * @param  [in] zangle    绕z轴旋转偏移量
 * @param  [out] post_pos 撤退点
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ComputePostPick(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    DescPose desc_pos;
    _fillDescPose(list,desc_pos);
    double zlength = std::stod(list.front());list.pop_front();
    double zangle = std::stod(list.front());list.pop_front();

    DescPose post_pos;

    int res = _ptr_robot->ComputePostPick(&desc_pos,zlength,zangle,&post_pos);
    return std::string(std::to_string(res) + "," + std::to_string(post_pos.tran.x) + "," + \
            std::to_string(post_pos.tran.y) + "," + std::to_string(post_pos.tran.z) + "," + \
            std::to_string(post_pos.rpy.rx) + "," + std::to_string(post_pos.rpy.ry) + "," + \
            std::to_string(post_pos.rpy.rz));
}

/**
 * @brief  配置力传感器
 * @param  [in] company  力传感器厂商，17-坤维科技，19-航天十一院，20-ATI传感器，21-中科米点，22-伟航敏芯，23-NBIT，24-鑫精诚(XJC)，26-NSR
 * @param  [in] device  设备号，坤维(0-KWR75B)，航天十一院(0-MCS6A-200-4)，ATI(0-AXIA80-M8)，中科米点(0-MST2010)，伟航敏芯(0-WHC6L-YB-10A)，NBIT(0-XLH93003ACS)，鑫精诚XJC(0-XJC-6F-D82)，NSR(0-NSR-FTSensorA)
 * @param  [in] softvesion  软件版本号，暂不使用，默认为0
 * @param  [in] bus 设备挂在末端总线位置，暂不使用，默认为0
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_SetConfig(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int company = std::stoi(list.front());list.pop_front();
    int device = std::stoi(list.front());list.pop_front();
    int softvesion = std::stoi(list.front());list.pop_front();
    int bus = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FT_SetConfig(company,device,softvesion,bus);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取力传感器配置
 * @param  [out] company  力传感器厂商，待定
 * @param  [out] device  设备号，暂不使用，默认为0
 * @param  [out] softvesion  软件版本号，暂不使用，默认为0
 * @param  [out] bus 设备挂在末端总线位置，暂不使用，默认为0
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_GetConfig(std::string para){
    int company;
    int device;
    int softvesion;
    int bus;

    int res = _ptr_robot->FT_GetConfig(&company,&device,&softvesion,&bus);
    return std::string(std::to_string(res) + "," + std::to_string(company) + "," + \
            std::to_string(device) + "," + std::to_string(softvesion) + "," + \
            std::to_string(bus));
}

/**
 * @brief  力传感器激活
 * @param  [in] act  0-复位，1-激活
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_Activate(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t act = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FT_Activate(act);
    return std::string(std::to_string(res));
}

/**
 * @brief  力传感器校零
 * @param  [in] act  0-去除零点，1-零点矫正
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_SetZero(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t act = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FT_SetZero(act);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置力传感器参考坐标系
 * @param  [in] ref  0-工具坐标系，1-基坐标系,2-自定义坐标系
 * @param  [in] coord  自定义坐标系值
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_SetRCS(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t ref = std::stoi(list.front());list.pop_front();
    DescPose coord;
    _fillDescPose(list,coord);

    int res = _ptr_robot->FT_SetRCS(ref,coord);
    return std::string(std::to_string(res));
}

/**
 * @brief  负载重量辨识记录
 * @param  [in] id  传感器坐标系编号，范围[1~14]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_PdIdenRecord(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t id = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FT_PdIdenRecord(id);
    return std::string(std::to_string(res));
}

/**
 * @brief  负载重量辨识计算
 * @param  [out] weight  负载重量，单位kg
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_PdIdenCompute(std::string para){
    float weight;

    int res = _ptr_robot->FT_PdIdenCompute(&weight);
    return std::string(std::to_string(res) + "," + std::to_string(weight));
}

/**
 * @brief  负载质心辨识记录
 * @param  [in] id  传感器坐标系编号，范围[1~14]
 * @param  [in] index 点编号，范围[1~3]
 * @return  错误码
 */
std::string robot_command_thread::FT_PdCogIdenRecord(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    int index = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FT_PdCogIdenRecord(id,index);
    return std::string(std::to_string(res));
}

/**
 * @brief  负载质心辨识计算
 * @param  [out] cog  负载质心，单位mm
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_PdCogIdenCompute(std::string para){
    DescTran cog;

    int res = _ptr_robot->FT_PdCogIdenCompute(&cog);
    return std::string(std::to_string(res) + "," + std::to_string(cog.x) + "," +\
            std::to_string(cog.y) + "," + std::to_string(cog.z));
}

/**
 * @brief  获取参考坐标系下力/扭矩数据
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] ft  力/扭矩，fx,fy,fz,tx,ty,tz
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_GetForceTorqueRCS(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front());list.pop_front();

    ForceTorque ft;

    int res = _ptr_robot->FT_GetForceTorqueRCS(flag,&ft);
    return std::string(std::to_string(res) + "," + std::to_string(ft.fx) + "," +\
            std::to_string(ft.fy) + "," + std::to_string(ft.fz) + "," + \
            std::to_string(ft.tx) + "," + std::to_string(ft.ty) + "," + \
            std::to_string(ft.tz));
}

/**
 * @brief  获取力传感器原始力/扭矩数据
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] ft  力/扭矩，fx,fy,fz,tx,ty,tz
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_GetForceTorqueOrigin(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front());list.pop_front();

    ForceTorque ft;

    int res = _ptr_robot->FT_GetForceTorqueOrigin(flag,&ft);
    return std::string(std::to_string(res) + "," + std::to_string(ft.fx) + "," +\
            std::to_string(ft.fy) + "," + std::to_string(ft.fz) + "," + \
            std::to_string(ft.tx) + "," + std::to_string(ft.ty) + "," + \
            std::to_string(ft.tz));
}

/**
 * @brief  碰撞守护
 * @param  [in] flag 0-关闭碰撞守护，1-开启碰撞守护
 * @param  [in] sensor_id 力传感器编号
 * @param  [in] select  选择六个自由度是否检测碰撞，0-不检测，1-检
 * @param  [in] ft  碰撞力/扭矩，fx,fy,fz,tx,ty,tz
 * @param  [in] max_threshold 最大阈值
 * @param  [in] min_threshold 最小阈值
 * @note   力/扭矩检测范围：(ft-min_threshold, ft+max_threshold)
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_Guard(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front());list.pop_front();
    int sensor_id = std::stoi(list.front());list.pop_front();
    uint8_t select[6];
    select[0] = std::stoi(list.front());list.pop_front();
    select[1] = std::stoi(list.front());list.pop_front();
    select[2] = std::stoi(list.front());list.pop_front();
    select[3] = std::stoi(list.front());list.pop_front();
    select[4] = std::stoi(list.front());list.pop_front();
    select[5] = std::stoi(list.front());list.pop_front();
    ForceTorque ft;
    ft.fx = std::stod(list.front());list.pop_front();
    ft.fy = std::stod(list.front());list.pop_front();
    ft.fz = std::stod(list.front());list.pop_front();
    ft.tx = std::stod(list.front());list.pop_front();
    ft.ty = std::stod(list.front());list.pop_front();
    ft.tz = std::stod(list.front());list.pop_front();
    float max_threshold[6];
    max_threshold[0] = std::stod(list.front());list.pop_front();
    max_threshold[1] = std::stod(list.front());list.pop_front();
    max_threshold[2] = std::stod(list.front());list.pop_front();
    max_threshold[3] = std::stod(list.front());list.pop_front();
    max_threshold[4] = std::stod(list.front());list.pop_front();
    max_threshold[5] = std::stod(list.front());list.pop_front();
    float min_threshold[6];
    min_threshold[0] = std::stod(list.front());list.pop_front();
    min_threshold[1] = std::stod(list.front());list.pop_front();
    min_threshold[2] = std::stod(list.front());list.pop_front();
    min_threshold[3] = std::stod(list.front());list.pop_front();
    min_threshold[4] = std::stod(list.front());list.pop_front();
    min_threshold[5] = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->FT_Guard(flag,sensor_id,select,&ft,max_threshold,min_threshold);
    return std::string(std::to_string(res));
}

/**
 * @brief  恒力控制
 * @param  [in] flag 0-关闭恒力控制，1-开启恒力控制
 * @param  [in] sensor_id 力传感器编号
 * @param  [in] select  选择六个自由度是否检测碰撞，0-不检测，1-检测
 * @param  [in] ft  碰撞力/扭矩，fx,fy,fz,tx,ty,tz
 * @param  [in] ft_pid 力pid参数，力矩pid参数
 * @param  [in] adj_sign 自适应启停控制，0-关闭，1-开启
 * @param  [in] ILC_sign ILC启停控制， 0-停止，1-训练，2-实操
 * @param  [in] max_dis 最大调整距离，单位mm
 * @param  [in] max_ang 最大调整角度，单位deg
 * @param  [in] M 质量参数 
 * @param  [in] B 阻尼参数
 * @param  [in] polishRadio 打磨半径，单位mm
 * @param  [in] filter_Sign 滤波开启标志 0-关；1-开，默认关闭
 * @param  [in] posAdapt_sign 姿态顺应开启标志 0-关；1-开，默认关闭
 * @param  [in] isNoBlock 阻塞标志，0-阻塞；1-非阻塞
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_Control(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    int parameters_count = list.size();
    if(parameters_count == 36)
    {
        uint8_t flag = std::stoi(list.front());list.pop_front();
        int sensor_id = std::stoi(list.front());list.pop_front();
        uint8_t select[6];
        select[0] = std::stoi(list.front());list.pop_front();
        select[1] = std::stoi(list.front());list.pop_front();
        select[2] = std::stoi(list.front());list.pop_front();
        select[3] = std::stoi(list.front());list.pop_front();
        select[4] = std::stoi(list.front());list.pop_front();
        select[5] = std::stoi(list.front());list.pop_front();
        ForceTorque ft;
        ft.fx = std::stod(list.front());list.pop_front();
        ft.fy = std::stod(list.front());list.pop_front();
        ft.fz = std::stod(list.front());list.pop_front();
        ft.tx = std::stod(list.front());list.pop_front();
        ft.ty = std::stod(list.front());list.pop_front();
        ft.tz = std::stod(list.front());list.pop_front();
        float ft_pid[6];
        ft_pid[0] = std::stod(list.front());list.pop_front();
        ft_pid[1] = std::stod(list.front());list.pop_front();
        ft_pid[2] = std::stod(list.front());list.pop_front();
        ft_pid[3] = std::stod(list.front());list.pop_front();
        ft_pid[4] = std::stod(list.front());list.pop_front();
        ft_pid[5] = std::stod(list.front());list.pop_front();
        uint8_t adj_sign = std::stoi(list.front());list.pop_front();
        uint8_t ILC_sign = std::stoi(list.front());list.pop_front();
        float max_dis = std::stod(list.front());list.pop_front();
        float max_ang = std::stod(list.front());list.pop_front();
        double M[2];
        M[0] = std::stod(list.front());list.pop_front();
        M[1] = std::stod(list.front());list.pop_front();
        double B[2];
        B[0] = std::stod(list.front());list.pop_front();
        B[1] = std::stod(list.front());list.pop_front();
        double threshold[2];
        threshold[0] = std::stod(list.front());list.pop_front();
        threshold[1] = std::stod(list.front());list.pop_front();
        double adjustCoeff[2];
        adjustCoeff[0] = std::stod(list.front());list.pop_front();
        adjustCoeff[1] = std::stod(list.front());list.pop_front();
        double polishRadio = std::stod(list.front());list.pop_front();
        int filter_Sign = std::stoi(list.front());list.pop_front();
        int posAdapt_sign = std::stoi(list.front());list.pop_front();
        int isNoBlock = std::stoi(list.front());list.pop_front();

        int res = _ptr_robot->FT_Control(flag,sensor_id,select,&ft,ft_pid,adj_sign,ILC_sign,max_dis,max_ang,M,B,threshold,adjustCoeff,polishRadio,filter_Sign,posAdapt_sign,isNoBlock);

        return std::string(std::to_string(res));
    }
    else
    {
        uint8_t flag = std::stoi(list.front());list.pop_front();
        int sensor_id = std::stoi(list.front());list.pop_front();
        uint8_t select[6];
        select[0] = std::stoi(list.front());list.pop_front();
        select[1] = std::stoi(list.front());list.pop_front();
        select[2] = std::stoi(list.front());list.pop_front();
        select[3] = std::stoi(list.front());list.pop_front();
        select[4] = std::stoi(list.front());list.pop_front();
        select[5] = std::stoi(list.front());list.pop_front();
        ForceTorque ft;
        ft.fx = std::stod(list.front());list.pop_front();
        ft.fy = std::stod(list.front());list.pop_front();
        ft.fz = std::stod(list.front());list.pop_front();
        ft.tx = std::stod(list.front());list.pop_front();
        ft.ty = std::stod(list.front());list.pop_front();
        ft.tz = std::stod(list.front());list.pop_front();
        float ft_pid[6];
        ft_pid[0] = std::stod(list.front());list.pop_front();
        ft_pid[1] = std::stod(list.front());list.pop_front();
        ft_pid[2] = std::stod(list.front());list.pop_front();
        ft_pid[3] = std::stod(list.front());list.pop_front();
        ft_pid[4] = std::stod(list.front());list.pop_front();
        ft_pid[5] = std::stod(list.front());list.pop_front();
        uint8_t adj_sign = std::stoi(list.front());list.pop_front();
        uint8_t ILC_sign = std::stoi(list.front());list.pop_front();
        float max_dis = std::stod(list.front());list.pop_front();
        float max_ang = std::stod(list.front());list.pop_front();
        double M[2];
        M[0] = std::stod(list.front());list.pop_front();
        M[1] = std::stod(list.front());list.pop_front();
        double B[2];
        B[0] = std::stod(list.front());list.pop_front();
        B[1] = std::stod(list.front());list.pop_front();
        double polishRadio = std::stod(list.front());list.pop_front();
        int filter_Sign = std::stoi(list.front());list.pop_front();
        int posAdapt_sign = std::stoi(list.front());list.pop_front();
        int isNoBlock = std::stoi(list.front());list.pop_front();

        int res = _ptr_robot->FT_Control(flag,sensor_id,select,&ft,ft_pid,adj_sign,ILC_sign,max_dis,max_ang,M,B,polishRadio,filter_Sign,posAdapt_sign,isNoBlock);
        return std::string(std::to_string(res));
    }
}

/**
 * @brief  螺旋线探索
 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
 * @param  [in] dr 每圈半径进给量
 * @param  [in] ft 插入动作触发力，单位N
 * @param  [in] max_t_ms 最大探索时间，单位ms
 * @param  [in] max_vel 最大线速度，单位mm/s
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_SpiralSearch(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int rcs = std::stoi(list.front());list.pop_front();
    float dr = std::stod(list.front());list.pop_front();
    float ft = std::stod(list.front());list.pop_front();
    float max_t_ms = std::stod(list.front());list.pop_front();
    float max_vel = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->FT_SpiralSearch(rcs,dr,ft,max_t_ms,max_vel);
    return std::string(std::to_string(res));
}

/**
 * @brief  旋转插入
 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
 * @param  [in] angVelRot 旋转角速度，单位deg/s
 * @param  [in] ft  力/扭矩阈值，fx,fy,fz,tx,ty,tz，范围[0~100]
 * @param  [in] max_angle 最大旋转角度，单位deg
 * @param  [in] orn 力/扭矩方向，1-沿z轴方向，2-绕z轴方向
 * @param  [in] max_angAcc 最大旋转加速度，单位deg/s^2，暂不使用，默认为0
 * @param  [in] rotorn  旋转方向，1-顺时针，2-逆时针
 * @param  [in] strategy 未检测到力/力矩的处理策略，0-报错；1-警告，继续运动
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_RotInsertion(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int rcs = std::stoi(list.front());list.pop_front();
    float angVelRot = std::stod(list.front());list.pop_front();
    float ft = std::stod(list.front());list.pop_front();
    float max_angle = std::stod(list.front());list.pop_front();
    uint8_t orn = std::stoi(list.front());list.pop_front();
    float max_angAcc = std::stod(list.front());list.pop_front();
    uint8_t rotorn = std::stoi(list.front());list.pop_front();
    int strategy = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FT_RotInsertion(rcs,angVelRot,ft,max_angle,orn,max_angAcc,rotorn,strategy);
    return std::string(std::to_string(res));
}

/**
 * @brief  直线插入
 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
 * @param  [in] ft  动作终止力阈值，单位N，范围[0~100]
 * @param  [in] lin_v 直线速度，单位mm/s
 * @param  [in] lin_a 直线加速度，单位mm/s^2，暂不使用
 * @param  [in] max_dis 最大插入距离，单位mm
 * @param  [in] linorn  插入方向，0-负方向，1-正方向
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_LinInsertion(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int rcs = std::stoi(list.front());list.pop_front();
    float ft = std::stod(list.front());list.pop_front();
    float lin_v = std::stod(list.front());list.pop_front();
    float lin_a = std::stod(list.front());list.pop_front();
    float max_dis = std::stod(list.front());list.pop_front();
    uint8_t linorn = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FT_LinInsertion(rcs,ft,lin_v,lin_a,max_dis,linorn);
    return std::string(std::to_string(res));
}

/**
 * @brief  表面定位
 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
 * @param  [in] dir  移动方向，1-正方向，2-负方向
 * @param  [in] axis 移动轴，1-x轴，2-y轴，3-z轴
 * @param  [in] lin_v 探索直线速度，单位mm/s
 * @param  [in] lin_a 探索直线加速度，单位mm/s^2，暂不使用，默认为0
 * @param  [in] max_dis 最大探索距离，单位mm
 * @param  [in] ft  动作终止力阈值，单位N
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_FindSurface(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int rcs = std::stoi(list.front());list.pop_front();
    uint8_t dir = std::stoi(list.front());list.pop_front();
    uint8_t axis = std::stoi(list.front());list.pop_front();
    float lin_v = std::stod(list.front());list.pop_front();
    float lin_a = std::stod(list.front());list.pop_front();
    float max_dis = std::stod(list.front());list.pop_front();
    float ft = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->FT_FindSurface(rcs,dir,axis,lin_v,lin_a,max_dis,ft);
    return std::string(std::to_string(res));
}

/**
 * @brief  计算中间平面位置开始
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_CalCenterStart(std::string para){
    int res = _ptr_robot->FT_CalCenterStart();
    return std::string(std::to_string(res));
}

/**
 * @brief  计算中间平面位置结束
 * @param  [out] pos 中间平面位姿
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_CalCenterEnd(std::string para){
    DescPose pos;

    int res = _ptr_robot->FT_CalCenterEnd(&pos);
    return std::string(std::to_string(res) + "," + std::to_string(pos.tran.x) + "," + \
            std::to_string(pos.tran.y) + "," + std::to_string(pos.tran.z) + "," + \
            std::to_string(pos.rpy.rx) + "," + std::to_string(pos.rpy.ry) + "," + \
            std::to_string(pos.rpy.rz));
}

/**
 * @brief  柔顺控制开启
 * @param  [in] p 位置调节系数或柔顺系数
 * @param  [in] force 柔顺开启力阈值，单位N
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_ComplianceStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    float p = std::stod(list.front());list.pop_front();
    float force = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->FT_ComplianceStart(p,force);
    return std::string(std::to_string(res));
}

/**
 * @brief  柔顺控制关闭
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::FT_ComplianceStop(std::string para){
    int res = _ptr_robot->FT_ComplianceStop();
    return std::string(std::to_string(res));
}

/**
 * @brief  负载辨识初始化
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LoadIdentifyDynFilterInit(std::string para){
    int res = _ptr_robot->LoadIdentifyDynFilterInit();
    return std::string(std::to_string(res));
}

/**
 * @brief  负载辨识初始化
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LoadIdentifyDynVarInit(std::string para){
    int res = _ptr_robot->LoadIdentifyDynVarInit();
    return std::string(std::to_string(res));
}

/**
 * @brief 负载辨识主程序
 * @param [in] joint_torque 关节扭矩
 * @param [in] joint_pos 关节位置
 * @param [in] t 采样周期
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LoadIdentifyMain(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double joint_torque[6];
    joint_torque[0] = std::stod(list.front());list.pop_front();
    joint_torque[1] = std::stod(list.front());list.pop_front();
    joint_torque[2] = std::stod(list.front());list.pop_front();
    joint_torque[3] = std::stod(list.front());list.pop_front();
    joint_torque[4] = std::stod(list.front());list.pop_front();
    joint_torque[5] = std::stod(list.front());list.pop_front();
    double joint_pos[6];
    joint_pos[0] = std::stod(list.front());list.pop_front();
    joint_pos[1] = std::stod(list.front());list.pop_front();
    joint_pos[2] = std::stod(list.front());list.pop_front();
    joint_pos[3] = std::stod(list.front());list.pop_front();
    joint_pos[4] = std::stod(list.front());list.pop_front();
    joint_pos[5] = std::stod(list.front());list.pop_front();
    double t = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->LoadIdentifyMain(joint_torque,joint_pos,t);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取负载辨识结果
 * @param [in] gain
 * @param [out] weight 负载重量
 * @param [out] cog 负载质心
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LoadIdentifyGetResult(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double gain[12];
    for(int i = 0;i < 12; i++){
        gain[i] = std::stod(list.front());list.pop_front();
    }
    double weight;
    DescTran cog;

    int res = _ptr_robot->LoadIdentifyGetResult(gain,&weight,&cog);
    return std::string(std::to_string(res) + "," + std::to_string(weight) + "," +\
            std::to_string(cog.x) + "," + std::to_string(cog.y) + "," +\
            std::to_string(cog.z));
}

/**
 * @brief 传动带启动、停止
 * @param [in] status 状态，1-启动，0-停止
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorStartEnd(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t status = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ConveyorStartEnd(status);
    return std::string(std::to_string(res));
}

/**
 * @brief 记录IO检测点
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorPointIORecord(std::string para){
    int res = _ptr_robot->ConveyorPointIORecord();
    return std::string(std::to_string(res));
}

/**
 * @brief 记录A点
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorPointARecord(std::string para){
    int res = _ptr_robot->ConveyorPointARecord();
    return std::string(std::to_string(res));
}

/**
 * @brief 记录参考点
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorRefPointRecord(std::string para){
    int res = _ptr_robot->ConveyorRefPointRecord();
    return std::string(std::to_string(res));
}

/**
 * @brief 记录B点
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorPointBRecord(std::string para){
    int res = _ptr_robot->ConveyorPointBRecord();
    return std::string(std::to_string(res));
}

/**
 * @brief 传送带工件IO检测
 * @param [in] max_t 最大检测时间，单位ms
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorIODetect(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int max_t = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ConveyorIODetect(max_t);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取物体当前位置
 * @param [in] mode
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorGetTrackData(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int mode = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ConveyorGetTrackData(mode);
    return std::string(std::to_string(res));
}

/**
 * @brief 传动带跟踪开始
 * @param [in] status 状态，1-启动，0-停止
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorTrackStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t status = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ConveyorTrackStart(status);
    return std::string(std::to_string(res));
}

/**
 * @brief 传动带跟踪停止
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorTrackEnd(std::string para){
    int res = _ptr_robot->ConveyorTrackEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief 传动带参数配置
 * @param [in] para[0] 编码器通道 1~2
 * @param [in] para[1] 编码器转一圈的脉冲数
 * @param [in] para[2] 编码器转一圈传送带行走距离
 * @param [in] para[3] 工件坐标系编号 针对跟踪运动功能选择工件坐标系编号，跟踪抓取、TPD跟踪设为0
 * @param [in] para[4] 是否配视觉  0 不配  1 配
 * @param [in] para[5] 速度比  针对传送带跟踪抓取选项（1-100）  其他选项默认为1 
 * @param [in] followType 跟踪运动类型，0-跟踪运动；1-追检运动
 * @param [in] startDis 追检抓取需要设置， 跟踪起始距离， -1：自动计算(工件到达机器人下方后自动追检)，单位mm， 默认值0
 * @param [in] endDis 追检抓取需要设置，跟踪终止距离， 单位mm， 默认值100
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorSetParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    float param[6];
    param[0] = std::stod(list.front());list.pop_front();
    param[1] = std::stod(list.front());list.pop_front();
    param[2] = std::stod(list.front());list.pop_front();
    param[3] = std::stod(list.front());list.pop_front();
    param[4] = std::stod(list.front());list.pop_front();
    param[5] = std::stod(list.front());list.pop_front();
    int followType = std::stoi(list.front());list.pop_front();
    int startDis = std::stoi(list.front());list.pop_front();
    int endDis = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ConveyorSetParam(param,followType,startDis,endDis);
    return std::string(std::to_string(res));
}

/**
 * @brief 传动带抓取点补偿
 * @param [in] cmp 补偿位置 double[3]{x, y, z}
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ConveyorCatchPointComp(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double cmp[3];
    cmp[0] = std::stod(list.front());list.pop_front();
    cmp[1] = std::stod(list.front());list.pop_front();
    cmp[2] = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->ConveyorCatchPointComp(cmp);
    return std::string(std::to_string(res));
}

/**
 * @brief 直线运动
 * @param [in] status 状态，1-启动，0-停止
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::TrackMoveL(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    char name[32];
    list.front().copy(name,list.front().size());list.pop_front();
    int tool = std::stoi(list.front());list.pop_front();
    int wobj = std::stoi(list.front());list.pop_front();
    float vel = std::stod(list.front());list.pop_front();
    float acc = std::stod(list.front());list.pop_front();
    float ovl = std::stod(list.front());list.pop_front();
    float blendR = std::stod(list.front());list.pop_front();
    uint8_t flag = std::stoi(list.front());list.pop_front();
    uint8_t type = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->TrackMoveL(name,tool,wobj,vel,acc,ovl,blendR,flag,type);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取SSH公钥
 * @param [out] keygen 公钥
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetSSHKeygen(std::string para){
    char keygen[1024];
    int res = _ptr_robot->GetSSHKeygen(keygen);
    return std::string(std::to_string(res) + "," + std::string(keygen));
}

/**
 * @brief 下发SCP指令
 * @param [in] mode 0-上传（上位机->控制器），1-下载（控制器->上位机）
 * @param [in] sshname 上位机用户名
 * @param [in] sship 上位机ip地址
 * @param [in] usr_file_url 上位机文件路径
 * @param [in] robot_file_url 机器人控制器文件路径
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetSSHScpCmd(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int mode = std::stoi(list.front());list.pop_front();
    char sshname[32];
    list.front().copy(sshname,list.front().size());list.pop_front();
    char sship[32];
    list.front().copy(sship,list.front().size());list.pop_front();
    char usr_file_url[128];
    list.front().copy(usr_file_url,list.front().size());list.pop_front();
    char robot_file_url[128];
    list.front().copy(robot_file_url,list.front().size());list.pop_front();

    int res = _ptr_robot->SetSSHScpCmd(mode,sshname,sship,usr_file_url,robot_file_url);
    return std::string(std::to_string(res));
}

/**
 * @brief 计算指定路径下文件的MD5值
 * @param [in] file_path 文件路径包含文件名，默认Traj文件夹路径为:"/fruser/traj/",如"/fruser/traj/trajHelix_aima_1.txt"
 * @param [out] md5 文件MD5值
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ComputeFileMD5(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    char file_path[256];
    list.front().copy(file_path,list.front().size());list.pop_front();

    char md5[256];

    int res = _ptr_robot->ComputeFileMD5(file_path,md5);
    return std::string(std::to_string(res) + "," + std::string(md5));
}

/**
 * @brief 获取机器人急停状态
 * @param [out] state 急停状态，0-非急停，1-急停
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetRobotEmergencyStopState(std::string para){
    uint8_t state;

    int res = _ptr_robot->GetRobotEmergencyStopState(&state);
    return std::string(std::to_string(res) + "," + std::to_string(state));
}

/**
 * @brief 获取SDK与机器人的通讯状态
 * @param [out]  state 通讯状态，0-通讯正常，1-通讯异常
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetSDKComState(std::string para){
    int state;

    int res = _ptr_robot->GetSDKComState(&state);
    return std::string(std::to_string(res) + "," + std::to_string(state));
}

/**
 * @brief 获取安全停止信号
 * @param [out]  si0_state 安全停止信号SI0
 * @param [out]  si1_state 安全停止信号SI1
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetSafetyStopState(std::string para){
    uint8_t si0_state;
    uint8_t si1_state;

    int res = _ptr_robot->GetSafetyStopState(&si0_state,&si1_state);
    return std::string(std::to_string(res) + "," + std::to_string(si0_state) + "," +\
            std::to_string(si1_state));
}

/**
 * @brief 获取机器人硬件版本
 * @param[out] ctrlBoxBoardversion 控制箱载板硬件版本
 * @param[out] driver1version 驱动器1硬件版本
 * @param[out] driver2version 驱动器2硬件版本
 * @param[out] driver3version 驱动器3硬件版本
 * @param[out] driver4version 驱动器4硬件版本
 * @param[out] driver5version 驱动器5硬件版本
 * @param[out] driver6version 驱动器6硬件版本
 * @param[out] endBoardversion 未端版硬件版本
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetHardwareVersion(std::string para){
    char ctrlBoxBoardversion[128];
    char driver1version[128];
    char driver2version[128];
    char driver3version[128];
    char driver4version[128];
    char driver5version[128];
    char driver6version[128];
    char endBoardversion[128];

    int res = _ptr_robot->GetHardwareVersion(ctrlBoxBoardversion,driver1version,driver2version,driver3version,driver4version,driver5version,driver6version,endBoardversion);
    return std::string(std::to_string(res) + "," + std::string(ctrlBoxBoardversion) + "," +\
            std::string(driver1version) + "," + std::string(driver2version) + "," +\
            std::string(driver3version) + "," + std::string(driver4version) + "," +\
            std::string(driver5version) + "," + std::string(driver6version) + "," +\
            std::string(endBoardversion));
}

/**
 * @brief 获取机器人固件版本
 * @param[out] ctrlBoxBoardversion 控制箱载板固件版本
 * @param[out] driver1version 驱动器1固件版本
 * @param[out] driver2version 驱动器2固件版本
 * @param[out] driver3version 驱动器3固件版本
 * @param[out] driver4version 驱动器4固件版本
 * @param[out] driver5version 驱动器5固件版本
 * @param[out] driver6version 驱动器6固件版本
 * @param[out] endBoardversion 未端版固件版本
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetFirmwareVersion(std::string para){
    char ctrlBoxBoardversion[128];
    char driver1version[128];
    char driver2version[128];
    char driver3version[128];
    char driver4version[128];
    char driver5version[128];
    char driver6version[128];
    char endBoardversion[128];

    int res = _ptr_robot->GetFirmwareVersion(ctrlBoxBoardversion,driver1version,driver2version,driver3version,driver4version,driver5version,driver6version,endBoardversion);
    return std::string(std::to_string(res) + "," + std::string(ctrlBoxBoardversion) + "," +\
            std::string(driver1version) + "," + std::string(driver2version) + "," +\
            std::string(driver3version) + "," + std::string(driver4version) + "," +\
            std::string(driver5version) + "," + std::string(driver6version) + "," +\
            std::string(endBoardversion));
}


/**
 * @brief 点位表切换
 * @param [in] pointTableName 要切换的点位表名称    pointTable1.db
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::PointTableSwitch(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string pointTableName = list.front();list.pop_front();

    int res = _ptr_robot->PointTableSwitch(pointTableName);
    return std::string(std::to_string(res));
}

/**
 * @brief 下载点位表数据库
 * @param [in] pointTableName 要下载的点位表名称    pointTable1.db
 * @param [in] saveFilePath 下载点位表的存储路径   C://test/
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::PointTableDownLoad(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string pointTableName = list.front();list.pop_front();
    std::string saveFilePath = list.front();list.pop_front();

    int res = _ptr_robot->PointTableDownLoad(pointTableName,saveFilePath);
    return std::string(std::to_string(res));
}

/**
 * @brief 上传点位表数据库
 * @param [in] pointTableFilePath 上传点位表的全路径名   C://test/pointTable1.db
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::PointTableUpLoad(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string pointTableFilePath = list.front();list.pop_front();

    int res = _ptr_robot->PointTableUpLoad(pointTableFilePath);
    return std::string(std::to_string(res));
}

/**
 * @brief 点位表更新lua文件
 * @param [in] pointTableName 要切换的点位表名称   "pointTable1.db",当点位表为空，即""时，表示将lua程序更新为未应用点位表的初始程
 * @param [in] luaFileName 要更新的lua文件名称   "testPointTable.lua"
 * @param [out] errorStr 切换点位表错误信息
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::PointTableUpdateLua(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string pointTableName = list.front();list.pop_front();
    std::string luaFileName = list.front();list.pop_front();

    int res = _ptr_robot->PointTableUpdateLua(pointTableName,luaFileName);
    return std::string(std::to_string(res));
}

/**
 * @brief 焊接开始
 * @param [in] ioType io类型 0-控制器IO； 1-扩展IO
 * @param [in] arcNum 焊机配置文件编号
 * @param [in] timeout 起弧超时时间
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ARCStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int ioType = std::stoi(list.front());list.pop_front();
    int arcNum = std::stoi(list.front());list.pop_front();
    int timeout = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ARCStart(ioType,arcNum,timeout);
    return std::string(std::to_string(res));
}

/**
 * @brief 焊接结束
 * @param [in] ioType io类型 0-控制器IO； 1-扩展IO
 * @param [in] arcNum 焊机配置文件编号
 * @param [in] timeout 起弧超时时间
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ARCEnd(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int ioType = std::stoi(list.front());list.pop_front();
    int arcNum = std::stoi(list.front());list.pop_front();
    int timeout = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ARCEnd(ioType,arcNum,timeout);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焊接电流与输出模拟量对应关系
 * @param [in] currentMin 焊接电流-模拟量输出线性关系左侧点电流值(A)
 * @param [in] currentMax 焊接电流-模拟量输出线性关系右侧点电流值(A)
 * @param [in] outputVoltageMin 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
 * @param [in] outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
 * @param [in] AOIndex 焊接电流模拟量输出端口
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeldingSetCurrentRelation(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double currentMin = std::stod(list.front());list.pop_front();
    double currentMax = std::stod(list.front());list.pop_front();
    double outputVoltageMin = std::stod(list.front());list.pop_front();
    double outputVoltageMax = std::stod(list.front());list.pop_front();
    int AOIndex = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeldingSetCurrentRelation(currentMin,currentMax,outputVoltageMin,outputVoltageMax,AOIndex);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焊接电压与输出模拟量对应关系
 * @param [in] weldVoltageMin 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
 * @param [in] weldVoltageMax 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
 * @param [in] outputVoltageMin 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
 * @param [in] outputVoltageMax 焊接电压-模拟量输出线性关系右侧点模拟量输出电压值(V)
 * @param [in] AOIndex 焊接电压模拟量输出端口
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeldingSetVoltageRelation(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double weldVoltageMin = std::stod(list.front());list.pop_front();
    double weldVoltageMax = std::stod(list.front());list.pop_front();
    double outputVoltageMin = std::stod(list.front());list.pop_front();
    double outputVoltageMax = std::stod(list.front());list.pop_front();
    int AOIndex = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeldingSetVoltageRelation(weldVoltageMin,weldVoltageMax,outputVoltageMin,outputVoltageMax,AOIndex);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取焊接电流与输出模拟量对应关系
 * @param [out] currentMin 焊接电流-模拟量输出线性关系左侧点电流值(A)
 * @param [out] currentMax 焊接电流-模拟量输出线性关系右侧点电流值(A)
 * @param [out] outputVoltageMin 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
 * @param [out] outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
 * @param [out] AOIndex 焊接电流模拟量输出端口
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeldingGetCurrentRelation(std::string para){
    double currentMin;
    double currentMax;
    double outputVoltageMin;
    double outputVoltageMax;
    int AOIndex;

    int res = _ptr_robot->WeldingGetCurrentRelation(&currentMin,&currentMax,&outputVoltageMin,&outputVoltageMax,&AOIndex);
    return std::string(std::to_string(res) + "," + std::to_string(currentMin) + "," +\
            std::to_string(currentMax) + "," + std::to_string(outputVoltageMin) + "," +\
            std::to_string(outputVoltageMax) + "," + std::to_string(AOIndex));
}

/**
 * @brief 获取焊接电压与输出模拟量对应关系
 * @param [out] weldVoltageMin 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
 * @param [out] weldVoltageMax 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
 * @param [out] outputVoltageMin 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
 * @param [out] outputVoltageMax 焊接电压-模拟量输出线性关系右侧点模拟量输出电压值(V)
 * @param [out] AOIndex 焊接电压模拟量输出端口
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeldingGetVoltageRelation(std::string para){
    double weldVoltageMin;
    double weldVoltageMax;
    double outputVoltageMin;
    double outputVoltageMax;
    int AOIndex;

    int res = _ptr_robot->WeldingGetVoltageRelation(&weldVoltageMin,&weldVoltageMax,&outputVoltageMin,&outputVoltageMax,&AOIndex);
    return std::string(std::to_string(res) + "," + std::to_string(weldVoltageMin) + "," +\
            std::to_string(weldVoltageMax) + "," + std::to_string(outputVoltageMin) + "," +\
            std::to_string(outputVoltageMax) + "," + std::to_string(AOIndex));
}

/**
 * @brief 设置焊接电流
 * @param [in] ioType 控制IO类型 0-控制箱IO；1-扩展IO
 * @param [in] current 焊接电流值(A)
 * @param [in] AOIndex 焊接电流控制箱模拟量输出端口(0-1)
 * @param [in] blend 是否平滑 0-不平滑；1-平滑
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeldingSetCurrent(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int ioType = std::stoi(list.front());list.pop_front();
    double current = std::stod(list.front());list.pop_front();
    int AOIndex = std::stoi(list.front());list.pop_front();
    int blend = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeldingSetCurrent(ioType,current,AOIndex,blend);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焊接电压
 * @param [in] ioType 控制IO类型 0-控制箱IO；1-扩展IO
 * @param [in] voltage 焊接电压值(A)
 * @param [in] AOIndex 焊接电压控制箱模拟量输出端口(0-1)
 * @param [in] blend 是否平滑 0-不平滑；1-平滑
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeldingSetVoltage(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int ioType = std::stoi(list.front());list.pop_front();
    double voltage = std::stod(list.front());list.pop_front();
    int AOIndex = std::stoi(list.front());list.pop_front();
    int blend = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeldingSetVoltage(ioType,voltage,AOIndex,blend);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置摆动参数
 * @param [in] weaveNum 摆焊参数配置编号
 * @param [in] weaveType 摆动类型 0-三角波摆动(LIN/ARC/Circle)；1-垂直L型三角波摆动(LIN/ARC/Circle)；2-圆形摆动-顺时针(LIN)；3-圆形摆动-逆时针(LIN)；4-正弦波摆动(LIN/ARC/Circle)；5-垂直L型正弦波摆动(LIN/ARC/Circle)；6-立焊三角摆动
 * @param [in] weaveFrequency 摆动频率(Hz)
 * @param [in] weaveIncStayTime 摆动等待时间 0-周期不包含等待时间；1-周期包含等待时间
 * @param [in] weaveRange 摆动幅度(mm)
 * @param [in] weaveLeftRange 垂直三角摆动左侧边长度(mm)
 * @param [in] weaveRightRange 垂直三角摆动右侧边长度(mm)
 * @param [in] additionalStayTime 垂直三角摆动零点停留时间(mm)
 * @param [in] weaveLeftStayTime 摆动左停留时间(ms)
 * @param [in] weaveRightStayTime 摆动右停留时间(ms)
 * @param [in] weaveCircleRadio 圆形摆动-回调比例(0-100%)
 * @param [in] weaveStationary 摆动位置等待，0-等待时间内位置继续移动；1-等待时间内位置静止
 * @param [in] weaveYawAngle 摆动方向方位角(绕摆动Z轴旋转)，单位°
 * @param [in] weaveRotAngle 摆动方向侧倾角(绕摆动X轴偏转)，单位°
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeaveSetPara(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int weaveNum = std::stoi(list.front());list.pop_front();
    int weaveType = std::stoi(list.front());list.pop_front();
    double weaveFrequency = std::stod(list.front());list.pop_front();
    int weaveIncStayTime = std::stoi(list.front());list.pop_front();
    double weaveRange = std::stod(list.front());list.pop_front();
    double weaveLeftRange = std::stod(list.front());list.pop_front();
    double weaveRightRange = std::stod(list.front());list.pop_front();
    int additionalStayTime = std::stoi(list.front());list.pop_front();
    int weaveLeftStayTime = std::stoi(list.front());list.pop_front();
    int weaveRightStayTime = std::stoi(list.front());list.pop_front();
    int weaveCircleRadio = std::stoi(list.front());list.pop_front();
    int weaveStationary = std::stoi(list.front());list.pop_front();
    double weaveYawAngle = std::stod(list.front());list.pop_front();
    double weaveRotAngle = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->WeaveSetPara(weaveNum,weaveType,weaveFrequency,weaveIncStayTime,
                                        weaveRange,weaveLeftRange,weaveRightRange,additionalStayTime,weaveLeftStayTime,
                                        weaveRightStayTime,weaveCircleRadio,weaveStationary,weaveYawAngle,weaveRotAngle);
    return std::string(std::to_string(res));
}

/**
 * @brief 即时设置摆动参数
 * @param [in] weaveNum 摆焊参数配置编号
 * @param [in] weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-垂直正弦波摆动
 * @param [in] weaveFrequency 摆动频率(Hz)
 * @param [in] weaveIncStayTime 等待模式 0-周期不包含等待时间；1-周期包含等待时间
 * @param [in] weaveRange 摆动幅度(mm)
 * @param [in] weaveLeftStayTime 摆动左停留时间(ms)
 * @param [in] weaveRightStayTime 摆动右停留时间(ms)
 * @param [in] weaveCircleRadio 圆形摆动-回调比率(0-100%)
 * @param [in] weaveStationary 摆动位置等待，0-等待时间内位置继续移动；1-等待时间内位置静止
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeaveOnlineSetPara(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int weaveNum = std::stoi(list.front());list.pop_front();
    int weaveType = std::stoi(list.front());list.pop_front();
    double weaveFrequency = std::stod(list.front());list.pop_front();
    int weaveIncStayTime = std::stoi(list.front());list.pop_front();
    double weaveRange = std::stod(list.front());list.pop_front();
    int weaveLeftStayTime = std::stoi(list.front());list.pop_front();
    int weaveRightStayTime = std::stoi(list.front());list.pop_front();
    int weaveCircleRadio = std::stoi(list.front());list.pop_front();
    int weaveStationary = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeaveOnlineSetPara(weaveNum,weaveType,weaveFrequency,weaveIncStayTime,
                                        weaveRange,weaveLeftStayTime,weaveRightStayTime,weaveCircleRadio,weaveStationary);
    return std::string(std::to_string(res));
}

/**
 * @brief 摆动开始
 * @param [in] weaveNum 摆焊参数配置编号
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeaveStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int weaveNum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeaveStart(weaveNum);
    return std::string(std::to_string(res));
}

/**
 * @brief 摆动结束
 * @param [in] weaveNum 摆焊参数配置编号
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WeaveEnd(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int weaveNum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeaveEnd(weaveNum);
    return std::string(std::to_string(res));
}

/**
 * @brief 正向送丝
 * @param [in] ioType io类型  0-控制器IO；1-扩展IO
 * @param [in] wireFeed 送丝控制  0-停止送丝；1-送丝
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetForwardWireFeed(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int ioType = std::stoi(list.front());list.pop_front();
    int wireFeed = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetForwardWireFeed(ioType,wireFeed);
    return std::string(std::to_string(res));
}

/**
 * @brief 反向送丝
 * @param [in] ioType io类型  0-控制器IO；1-扩展IO
 * @param [in] wireFeed 送丝控制  0-停止送丝；1-送丝
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetReverseWireFeed(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int ioType = std::stoi(list.front());list.pop_front();
    int wireFeed = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetReverseWireFeed(ioType,wireFeed);
    return std::string(std::to_string(res));
}

/**
 * @brief 送气
 * @param [in] ioType io类型  0-控制器IO；1-扩展IO
 * @param [in] airControl 送气控制  0-停止送气；1-送气
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetAspirated(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int ioType = std::stoi(list.front());list.pop_front();
    int airControl = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetAspirated(ioType,airControl);
    return std::string(std::to_string(res));
}

/**
 *@brief段焊开始
 *@param[in]startDesePos 起始点笛卡尔位置
 *@param[in]endDesePos 结束点笛卡尔位姿
 *@param[in]startJPos 起始点关节位姿
 *@param[in]endJPos 结束点关节位姿
 *@param[in]weldLength 焊接段长度(mm)
 *@param[in]noWeldLength 非焊接段长度(mm)
 *@param[in]weldIOType 焊接IO类型(0-控制箱IO；1-扩展IO)
 *@param[in]arcNum 焊机配置文件编号
 *@param[in]weldTimeout 起/收弧超时时间
 *@param[in]isWeave 是否摆动
 *@param[in]weaveNum 摆焊参数配置编号
 *@param[in]tool 工具号
 *@param[in]user 工件号
 *@param[in]vel 速度百分比，范围[0~100]
 *@param[in]acc 加速度百分比，范围[0~100],暂不开放
 *@param[in]ovl 速度缩放因子，范围[0~100]
 *@param[in]blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
 *@param[in]epos 扩展轴位置，单位mm
 *@param[in]search 0-不焊丝寻位，1-焊丝寻位
 *@param[in]offset_flag 0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 *@param[in]offset_pos 位姿偏移量
 */
std::string robot_command_thread::SegmentWeldStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    DescPose startDesePos;
    _fillDescPose(list,startDesePos);
    DescPose endDesePos;
    _fillDescPose(list,endDesePos);
    JointPos startJPos;
    _fillJointPose(list,startJPos);
    JointPos endJPos;
    _fillJointPose(list,endJPos);
    double weldLength = std::stod(list.front());list.pop_front();
    double noWeldLength = std::stod(list.front());list.pop_front();
    int weldIOType = std::stoi(list.front());list.pop_front();
    int arcNum = std::stoi(list.front());list.pop_front();
    int weldTimeout = std::stoi(list.front());list.pop_front();
    int isWeave = std::stoi(list.front());list.pop_front();
    int weaveNum = std::stoi(list.front());list.pop_front();
    int tool = std::stoi(list.front());list.pop_front();
    int user = std::stoi(list.front());list.pop_front();
    float vel = std::stod(list.front());list.pop_front();
    float acc = std::stod(list.front());list.pop_front();
    float ovl = std::stod(list.front());list.pop_front();
    float blendR = std::stod(list.front());list.pop_front();
    ExaxisPos epos;
    epos.ePos[0] = std::stod(list.front().c_str());list.pop_front();
    epos.ePos[1] = std::stod(list.front().c_str());list.pop_front();
    epos.ePos[2] = std::stod(list.front().c_str());list.pop_front();
    epos.ePos[3] = std::stod(list.front().c_str());list.pop_front();
    uint8_t search = std::stoi(list.front());list.pop_front();
    uint8_t offset_flag = std::stoi(list.front());list.pop_front();
    DescPose offset_pos;
    _fillDescPose(list,offset_pos);

    int res = _ptr_robot->SegmentWeldStart(&startDesePos,&endDesePos,&startJPos,&endJPos,weldLength,noWeldLength,
                                            weldIOType,arcNum,weldTimeout,isWeave,weaveNum,tool,user,vel,acc,ovl,blendR,
                                            &epos,search,offset_flag,&offset_pos);
    return std::string(std::to_string(res));
}

/**
 * @brief 初始化日志参数;
 * @param output_model：输出模式，0-直接输出；1-缓冲输出；2-异步输出;
 * @param file_path: 文件保存路径+名称，,长度上限256，名称必须是xxx.log的形式，比如/home/fr/linux/fairino.log;
 * @param file_num：滚动存储的文件数量，1~20个.单个文件上限50M;
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::LoggerInit(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int output_model = std::stoi(list.front());list.pop_front();
    std::string file_path = list.front();list.pop_front();
    int file_num = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->LoggerInit(output_model,file_path,file_num);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置日志过滤等级;
 * @param lvl: 过滤等级值，值越小输出日志越少，默认值是1. 1-error, 2-warnning, 3-inform, 4-debug;
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetLoggerLevel(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int lvl = std::stoi(list.front());list.pop_front();

    _ptr_robot->SetLoggerLevel(lvl);
    return "0";
}

/**
 * @brief 获取485扩展轴配置参数
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @param [out] servoCompany 伺服驱动器厂商，1-戴纳泰克
 * @param [out] servoModel 伺服驱动器型号，1-FD100-750C
 * @param [out] servoSoftVersion 伺服驱动器软件版本，1-V1.0
 * @param [out] servoResolution 编码器分辨率
 * @param [out] axisMechTransRatio 机械传动比
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoGetParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int servoId = std::stoi(list.front());list.pop_front();

    int servoCompany;
    int servoModel;
    int servoSoftVersion;
    int servoResolution;
    double axisMechTransRatio;

    int res = _ptr_robot->AuxServoGetParam(servoId,&servoCompany,&servoModel,&servoSoftVersion,&servoResolution,&axisMechTransRatio);
    return std::string(std::to_string(res) + "," + std::to_string(servoCompany) + "," +\
            std::to_string(servoModel) + "," + std::to_string(servoSoftVersion) + "," +\
            std::to_string(servoResolution) + "," + std::to_string(axisMechTransRatio));
}

/**
 * @brief 获取485扩展轴伺服状态
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @param [out] servoErrCode 伺服驱动器故障码
 * @param [out] servoState 伺服驱动器状态[十进制数转为二进制，bit0-bit5：伺服使能-伺服运行-正限位触发-负限位触发-定位完成-回零完成]
 * @param [out] servoPos 伺服当前位置 mm或°
 * @param [out] servoSpeed 伺服当前速度 mm/s或°/s
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::AuxServoGetStatus(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int servoId = std::stoi(list.front());list.pop_front();

    int servoErrCode;
    int servoState;
    double servoPos;
    double servoSpeed;
    double servoTorque;

    int res = _ptr_robot->AuxServoGetStatus(servoId,&servoErrCode,&servoState,&servoPos,&servoSpeed,&servoTorque);
    return std::string(std::to_string(res) + "," + std::to_string(servoErrCode) + "," +\
            std::to_string(servoState) + "," + std::to_string(servoPos) + "," +\
            std::to_string(servoSpeed) + "," + std::to_string(servoTorque));
}

/**
 * @brief 设置机器人外设协议
 * @param [out] protocol 机器人外设协议号 4096-扩展轴控制卡；4097-ModbusSlave；4098-ModbusMaster
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetExDevProtocol(std::string para){
    int protocol;

    int res = _ptr_robot->GetExDevProtocol(&protocol);
    return std::string(std::to_string(res) + "," + std::to_string(protocol));
}

/**
 * @brief 获取机器人外设协议
 * @param [in] protocol 机器人外设协议号 4096-扩展轴控制卡；4097-ModbusSlave；4098-ModbusMaster
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetExDevProtocol(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int protocol = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetExDevProtocol(protocol);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置机器人加速度
 * @param [in] acc 机器人加速度百分比
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetOaccScale(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double acc = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->SetOaccScale(acc);
    return std::string(std::to_string(res));
}

/**
 * @brief 控制箱AO飞拍开始
 * @param [in] AONum 控制箱AO编号
 * @param [in] maxTCPSpeed 最大TCP速度值[1-5000mm/s]，默认1000
 * @param [in] maxAOPercent 最大TCP速度值对应的AO百分比，默认100%
 * @param [in] zeroZoneCmp 死区补偿值AO百分比，整形，默认为20%，范围[0-100]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveAOStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int AONum = std::stoi(list.front());list.pop_front();
    int maxTCPSpeed = std::stoi(list.front());list.pop_front();
    int maxAOPercent = std::stoi(list.front());list.pop_front();
    int zeroZoneCmp = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->MoveAOStart(AONum,maxTCPSpeed,maxAOPercent,zeroZoneCmp);
    return std::string(std::to_string(res));
}

/**
 * @brief 控制箱AO飞拍停止
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveAOStop(std::string para){
    int res = _ptr_robot->MoveAOStop();
    return std::string(std::to_string(res));
}

/**
 * @brief 末端AO飞拍开始
 * @param [in] AONum 末端AO编号
 * @param [in] maxTCPSpeed 最大TCP速度值[1-5000mm/s]，默认1000
 * @param [in] maxAOPercent 最大TCP速度值对应的AO百分比，默认100%
 * @param [in] zeroZoneCmp 死区补偿值AO百分比，整形，默认为20%，范围[0-100]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveToolAOStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int AONum = std::stoi(list.front());list.pop_front();
    int maxTCPSpeed = std::stoi(list.front());list.pop_front();
    int maxAOPercent = std::stoi(list.front());list.pop_front();
    int zeroZoneCmp = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->MoveToolAOStart(AONum,maxTCPSpeed,maxAOPercent,zeroZoneCmp);
    return std::string(std::to_string(res));
}

/**
 * @brief 末端AO飞拍停止
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MoveToolAOStop(std::string para){
    int res = _ptr_robot->MoveToolAOStop();
    return std::string(std::to_string(res));
}

/**
 * @brief UDP扩展轴通讯参数配置
 * @param [in] ip PLC IP地址
 * @param [in] port	端口号
 * @param [in] period	通讯周期(ms，默认为2，请勿修改此参数)
 * @param [in] lossPkgTime	丢包检测时间(ms)
 * @param [in] lossPkgNum	丢包次数
 * @param [in] disconnectTime	通讯断开确认时长
 * @param [in] reconnectEnable	通讯断开自动重连使能 0-不使能 1-使能
 * @param [in] reconnectPeriod	重连周期间隔(ms)
 * @param [in] reconnectNum	重连次数
 * @param [in] selfConnect 断电重启是否自动建立连接；0-不建立连接；1-建立连接
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtDevSetUDPComParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string ip = list.front();list.pop_front();
    int port = std::stoi(list.front());list.pop_front();
    int period = std::stoi(list.front());list.pop_front();
    int lossPkgTime = std::stoi(list.front());list.pop_front();
    int lossPkgNum = std::stoi(list.front());list.pop_front();
    int disconnectTime = std::stoi(list.front());list.pop_front();
    int reconnectEnable = std::stoi(list.front());list.pop_front();
    int reconnectPeriod = std::stoi(list.front());list.pop_front();
    int reconnectNum = std::stoi(list.front());list.pop_front();
    int selfConnect = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ExtDevSetUDPComParam(ip,port,period,lossPkgTime,lossPkgNum,disconnectTime,reconnectEnable,
                                                reconnectPeriod,reconnectNum,selfConnect);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取UDP扩展轴通讯参数
 * @param [out] ip PLC IP地址
 * @param [out] port	端口号
 * @param [out] period	通讯周期(ms，默认为2，请勿修改此参数)
 * @param [out] lossPkgTime	丢包检测时间(ms)
 * @param [out] lossPkgNum	丢包次数
 * @param [out] disconnectTime	通讯断开确认时长
 * @param [out] reconnectEnable	通讯断开自动重连使能 0-不使能 1-使能
 * @param [out] reconnectPeriod	重连周期间隔(ms)
 * @param [out] reconnectNum	重连次数
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtDevGetUDPComParam(std::string para){
    std::string ip;
    int port;
    int period;
    int lossPkgTime;
    int lossPkgNum;
    int disconnectTime;
    int reconnectEnable;
    int reconnectPeriod;
    int reconnectNum;

    int res = _ptr_robot->ExtDevGetUDPComParam(ip,port,period,lossPkgTime,lossPkgNum,disconnectTime,reconnectEnable,
                                                reconnectPeriod,reconnectNum);
    return std::string(std::to_string(res) + "," + std::string(ip) + "," +\
            std::to_string(port) + "," + std::to_string(period) + "," +\
            std::to_string(lossPkgTime) + "," + std::to_string(lossPkgNum) + "," +\
            std::to_string(disconnectTime) + "," + std::to_string(reconnectEnable) + "," +\
            std::to_string(reconnectPeriod) + "," + std::to_string(reconnectNum));
}

/**
 * @brief 加载UDP通信
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtDevLoadUDPDriver(std::string para){
    int res = _ptr_robot->ExtDevLoadUDPDriver();
    return std::string(std::to_string(res));
}

/**
 * @brief 卸载UDP通信
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtDevUnloadUDPDriver(std::string para){
    int res = _ptr_robot->ExtDevUnloadUDPDriver();
    return std::string(std::to_string(res));
}

/**
 * @brief 设置扩展DI输入滤波时间
 * @param [in] filterTime 滤波时间(ms)
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetAuxDIFilterTime(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int filterTime = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetAuxDIFilterTime(filterTime);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置扩展AI输入滤波时间
 * @param [in] AONum AO编号
 * @param [in] filterTime 滤波时间(ms)
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetAuxAIFilterTime(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int AONum = std::stoi(list.front());list.pop_front();
    int filterTime = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetAuxAIFilterTime(AONum,filterTime);
    return std::string(std::to_string(res));
}

/**
 * @brief 等待扩展DI输入
 * @param [in] DINum DI编号
 * @param [in] bOpen 开关 0-关；1-开
 * @param [in] time 最大等待时间(ms)
 * @param [in] errorAlarm 是否继续运动
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WaitAuxDI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int DINum = std::stoi(list.front());list.pop_front();
    int bOpen = std::stoi(list.front());list.pop_front();
    int time = std::stoi(list.front());list.pop_front();
    int errorAlarm = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WaitAuxDI(DINum,bOpen,time,errorAlarm);
    return std::string(std::to_string(res));
}

/**
 * @brief 等待扩展AI输入
 * @param [in] AINum AI编号
 * @param [in] sign 0-大于；1-小于
 * @param [in] value AI值
 * @param [in] time 最大等待时间(ms)
 * @param [in] errorAlarm 是否继续运动
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::WaitAuxAI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int AINum = std::stoi(list.front());list.pop_front();
    int sign = std::stoi(list.front());list.pop_front();
    int value = std::stoi(list.front());list.pop_front();
    int time = std::stoi(list.front());list.pop_front();
    int errorAlarm = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WaitAuxAI(AINum,sign,value,time,errorAlarm);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取扩展DI值
 * @param [in] DINum DI编号
 * @param [in] isNoBlock 是否阻塞
 * @param [out] isOpen 0-关；1-开
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetAuxDI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int DINum = std::stoi(list.front());list.pop_front();
    int isNoBlock = std::stoi(list.front());list.pop_front();

    bool isOpen;

    int res = _ptr_robot->GetAuxDI(DINum,isNoBlock,isOpen);
    int isOpen_tmp;
    if(isOpen == true) isOpen_tmp = 1;
    else isOpen_tmp = 0;
    return std::string(std::to_string(res) + "," + std::to_string(isOpen_tmp));
}

/**
 * @brief 获取扩展DI值
 * @param [in] DINum DI编号
 * @param [in] isNoBlock 是否阻塞
 * @param [out] isOpen 0-关；1-开
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::GetAuxAI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int AINum = std::stoi(list.front());list.pop_front();
    int isNoBlock = std::stoi(list.front());list.pop_front();

    int value;

    int res = _ptr_robot->GetAuxAI(AINum,isNoBlock,value);
    return std::string(std::to_string(res) + "," + std::to_string(value));
}

/**
 * @brief UDP扩展轴通信异常断开后恢复连接
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtDevUDPClientComReset(std::string para){
    int res = _ptr_robot->ExtDevUDPClientComReset();
    return std::string(std::to_string(res));
}

/**
 * @brief UDP扩展轴通信异常断开后关闭通讯
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtDevUDPClientComClose(std::string para){
    int res = _ptr_robot->ExtDevUDPClientComClose();
    return std::string(std::to_string(res));
}

/**
 * @brief UDP扩展轴参数配置
 * @param [in] axisID 扩展轴号[1-4]
 * @param [in] axisType 扩展轴类型 0-平移；1-旋转
 * @param [in] axisDirection 扩展轴方向 0-正向；1-方向
 * @param [in] axisMax 扩展轴最大位置 mm
 * @param [in] axisMin 扩展轴最小位置 mm
 * @param [in] axisVel 速度mm/s
 * @param [in] axisAcc 加速度mm/s2
 * @param [in] axisLead 导程mm
 * @param [in] encResolution 编码器分辨率
 * @param [in] axisOffect焊缝起始点扩展轴偏移量
 * @param [in] axisCompany 驱动器厂家 1-禾川；2-汇川；3-松下
 * @param [in] axisModel 驱动器型号 1-禾川-SV-XD3EA040L-E，2-禾川-SV-X2EA150A-A，1-汇川-SV620PT5R4I，1-松下-MADLN15SG，2-松下-MSDLN25SG，3-松下-MCDLN35SG
 * @param [in] axisEncType 编码器类型  0-增量；1-绝对值
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtAxisParamConfig(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int axisID = std::stoi(list.front());list.pop_front();
    int axisType = std::stoi(list.front());list.pop_front();
    int axisDirection = std::stoi(list.front());list.pop_front();
    double axisMax = std::stod(list.front());list.pop_front();
    double axisMin = std::stod(list.front());list.pop_front();
    double axisVel = std::stod(list.front());list.pop_front();
    double axisAcc = std::stod(list.front());list.pop_front();
    double axisLead = std::stod(list.front());list.pop_front();
    int encResolution = std::stoi(list.front());list.pop_front();
    double axisOffect = std::stod(list.front());list.pop_front();
    int axisCompany = std::stoi(list.front());list.pop_front();
    int axisModel = std::stoi(list.front());list.pop_front();
    int axisEncType = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ExtAxisParamConfig(axisID,axisType,axisDirection,axisMax,axisMin,axisVel,axisAcc,axisLead,
                                                encResolution,axisOffect,axisCompany,axisModel,axisEncType);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置扩展机器人相对扩展轴位置
 * @param [in] installType 0-机器人安装在外部轴上，1-机器人安装在外部轴外
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetRobotPosToAxis(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int installType = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetRobotPosToAxis(installType);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置扩展轴系统DH参数配置
 * @param [in]  axisConfig 外部轴构型，0-单自由度直线滑轨，1-两自由度L型变位机，2-三自由度，3-四自由度，4-单自由度变位机
 * @param [in]  axisDHd1 外部轴DH参数d1 mm
 * @param [in]  axisDHd2 外部轴DH参数d2 mm
 * @param [in]  axisDHd3 外部轴DH参数d3 mm
 * @param [in]  axisDHd4 外部轴DH参数d4 mm
 * @param [in]  axisDHa1 外部轴DH参数11 mm
 * @param [in]  axisDHa2 外部轴DH参数a2 mm
 * @param [in]  axisDHa3 外部轴DH参数a3 mm
 * @param [in]  axisDHa4 外部轴DH参数a4 mm
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetAxisDHParaConfig(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int axisConfig = std::stoi(list.front());list.pop_front();
    double axisDHd1 = std::stod(list.front());list.pop_front();
    double axisDHd2 = std::stod(list.front());list.pop_front();
    double axisDHd3 = std::stod(list.front());list.pop_front();
    double axisDHd4 = std::stod(list.front());list.pop_front();
    double axisDHa1 = std::stod(list.front());list.pop_front();
    double axisDHa2 = std::stod(list.front());list.pop_front();
    double axisDHa3 = std::stod(list.front());list.pop_front();
    double axisDHa4 = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->SetAxisDHParaConfig(axisConfig,axisDHd1,axisDHd2,axisDHd3,axisDHd4,axisDHa1,axisDHa2,axisDHa3,
                                                axisDHa4);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置扩展轴坐标系参考点-四点法
 * @param [in]  pointNum 点编号[1-4]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtAxisSetRefPoint(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int pointNum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ExtAxisSetRefPoint(pointNum);
    return std::string(std::to_string(res));
}

/**
 * @brief 计算扩展轴坐标系-四点法
 * @param [out]  coord 坐标系值
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtAxisComputeECoordSys(std::string para){
    DescPose coord;

    int res = _ptr_robot->ExtAxisComputeECoordSys(coord);
    return std::string(std::to_string(res) + "," + std::to_string(coord.tran.x) + "," + \
            std::to_string(coord.tran.y) + "," + std::to_string(coord.tran.z) + "," + \
            std::to_string(coord.rpy.rx) + "," + std::to_string(coord.rpy.ry) + "," + \
            std::to_string(coord.rpy.rz));
}

/**
 * @brief 应用扩展轴坐标系
 * @param [in]  axisCoordNum 坐标系编号
 * @param [in]  toolNum 工具号
 * @param [in]  coord 坐标系值
 * @param [in]  calibFlag 标定标志 0-否，1-是
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::ExtAxisActiveECoordSys(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int axisCoordNum = std::stoi(list.front());list.pop_front();
    int toolNum = std::stoi(list.front());list.pop_front();
    DescPose coord;
    _fillDescPose(list,coord);
    int calibFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ExtAxisActiveECoordSys(axisCoordNum,toolNum,coord,calibFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置标定参考点在变位机末端坐标系下位姿
 * @param [in] pos 位姿值
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::SetRefPointInExAxisEnd(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    DescPose pos;
    _fillDescPose(list,pos);

    int res = _ptr_robot->SetRefPointInExAxisEnd(pos);
    return std::string(std::to_string(res));
}

/**
 * @brief 变位机坐标系参考点设置
 * @param [in]  pointNum 点编号[1-4]
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::PositionorSetRefPoint(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int pointNum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->PositionorSetRefPoint(pointNum);
    return std::string(std::to_string(res));
}

/**
 * @brief 变位机坐标系计算-四点法
 * @param [out]  coord 坐标系值
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::PositionorComputeECoordSys(std::string para){
    DescPose coord;

    int res = _ptr_robot->PositionorComputeECoordSys(coord);
    return std::string(std::to_string(res) + "," + std::to_string(coord.tran.x) + "," + \
            std::to_string(coord.tran.y) + "," + std::to_string(coord.tran.z) + "," + \
            std::to_string(coord.rpy.rx) + "," + std::to_string(coord.rpy.ry) + "," + \
            std::to_string(coord.rpy.rz));
}

/**
 * @brief  UDP扩展轴与机器人直线运动同步运动
 * @param  [in] joint_pos  目标关节位置,单位deg
 * @param  [in] desc_pos   目标笛卡尔位姿
 * @param  [in] tool  工具坐标号，范围[0~14]
 * @param  [in] user  工件坐标号，范围[0~14]
 * @param  [in] vel  速度百分比，范围[0~100]
 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
 * @param  [in] epos  扩展轴位置，单位mm
 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos  位姿偏移量
 * @return  错误码
 */
std::string robot_command_thread::ExtAxisSyncMoveL(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);


    JointPos joint_pos;
    _fillJointPose(list,joint_pos);
    DescPose desc_pos;
    _fillDescPose(list,desc_pos);
    int tool = std::stoi(list.front());list.pop_front();
    int user = std::stoi(list.front());list.pop_front();
    float vel = std::stod(list.front());list.pop_front();
    float acc = std::stod(list.front());list.pop_front();
    float ovl = std::stod(list.front());list.pop_front();
    float blendR = std::stod(list.front());list.pop_front();
    ExaxisPos epos;
    epos.ePos[0] = std::stod(list.front().c_str());list.pop_front();
    epos.ePos[1] = std::stod(list.front().c_str());list.pop_front();
    epos.ePos[2] = std::stod(list.front().c_str());list.pop_front();
    epos.ePos[3] = std::stod(list.front().c_str());list.pop_front();
    uint8_t offset_flag = std::stoi(list.front());list.pop_front();
    DescPose offset_pos;
    _fillDescPose(list,offset_pos);

    int res = _ptr_robot->ExtAxisSyncMoveL(joint_pos, desc_pos, tool, user, vel, acc, ovl, blendR, epos, offset_flag, offset_pos);
    return std::string(std::to_string(res));
}

/**
 * @brief  UDP扩展轴与机器人圆弧运动同步运动
 * @param  [in] joint_pos_p  路径点关节位置,单位deg
 * @param  [in] desc_pos_p   路径点笛卡尔位姿
 * @param  [in] ptool  工具坐标号，范围[0~14]
 * @param  [in] puser  工件坐标号，范围[0~14]
 * @param  [in] pvel  速度百分比，范围[0~100]
 * @param  [in] pacc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] epos_p  扩展轴位置，单位mm
 * @param  [in] poffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos_p  位姿偏移量
 * @param  [in] joint_pos_t  目标点关节位置,单位deg
 * @param  [in] desc_pos_t   目标点笛卡尔位姿
 * @param  [in] ttool  工具坐标号，范围[0~14]
 * @param  [in] tuser  工件坐标号，范围[0~14]
 * @param  [in] tvel  速度百分比，范围[0~100]
 * @param  [in] tacc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] epos_t  扩展轴位置，单位mm
 * @param  [in] toffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos_t  位姿偏移量
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
 * @return  错误码
 */
std::string robot_command_thread::ExtAxisSyncMoveC(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);


    JointPos joint_pos_p;
    _fillJointPose(list,joint_pos_p);
    DescPose desc_pos_p;
    _fillDescPose(list,desc_pos_p);
    int ptool = std::stoi(list.front());list.pop_front();
    int puser = std::stoi(list.front());list.pop_front();
    float pvel = std::stod(list.front());list.pop_front();
    float pacc = std::stod(list.front());list.pop_front();
    ExaxisPos epos_p;
    epos_p.ePos[0] = std::stod(list.front().c_str());list.pop_front();
    epos_p.ePos[1] = std::stod(list.front().c_str());list.pop_front();
    epos_p.ePos[2] = std::stod(list.front().c_str());list.pop_front();
    epos_p.ePos[3] = std::stod(list.front().c_str());list.pop_front();
    uint8_t poffset_flag = std::stoi(list.front());list.pop_front();
    DescPose offset_pos_p;
    _fillDescPose(list,offset_pos_p);
    JointPos joint_pos_t;
    _fillJointPose(list,joint_pos_t);
    DescPose desc_pos_t;
    _fillDescPose(list,desc_pos_t);
    int ttool = std::stoi(list.front());list.pop_front();
    int tuser = std::stoi(list.front());list.pop_front();
    float tvel = std::stod(list.front());list.pop_front();
    float tacc = std::stod(list.front());list.pop_front();
    ExaxisPos epos_t;
    epos_t.ePos[0] = std::stod(list.front().c_str());list.pop_front();
    epos_t.ePos[1] = std::stod(list.front().c_str());list.pop_front();
    epos_t.ePos[2] = std::stod(list.front().c_str());list.pop_front();
    epos_t.ePos[3] = std::stod(list.front().c_str());list.pop_front();
    uint8_t toffset_flag = std::stoi(list.front());list.pop_front();
    DescPose offset_pos_t;
    _fillDescPose(list,offset_pos_t);
    float ovl = std::stod(list.front());list.pop_front();
    float blendR = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->ExtAxisSyncMoveC(joint_pos_p, desc_pos_p, ptool, puser, pvel, pacc, epos_p, poffset_flag, offset_pos_p, joint_pos_t, desc_pos_t, ttool, tuser, tvel, tacc, epos_t, toffset_flag, offset_pos_t, ovl, blendR);
    return std::string(std::to_string(res));
}

/**
 * @brief  焊丝寻位开始
 * @param  [in] refPos  1-基准点 2-接触点
 * @param  [in] searchVel   寻位速度 %
 * @param  [in] searchDis  寻位距离 mm
 * @param  [in] autoBackFlag 自动返回标志，0-不自动；-自动
 * @param  [in] autoBackVel  自动返回速度 %
 * @param  [in] autoBackDis  自动返回距离 mm
 * @param  [in] offectFlag  1-带偏移量寻位；2-示教点寻位
 * @return  错误码
 */
std::string robot_command_thread::WireSearchStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int refPos = std::stoi(list.front());list.pop_front();
    float searchVel = std::stod(list.front());list.pop_front();
    int searchDis = std::stoi(list.front());list.pop_front();
    int autoBackFlag = std::stoi(list.front());list.pop_front();
    float autoBackVel = std::stod(list.front());list.pop_front();
    int autoBackDis = std::stoi(list.front());list.pop_front();
    int offectFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WireSearchStart(refPos, searchVel, searchDis, autoBackFlag, autoBackVel, autoBackDis, offectFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief  焊丝寻位结束
 * @param  [in] refPos  1-基准点 2-接触点
 * @param  [in] searchVel   寻位速度 %
 * @param  [in] searchDis  寻位距离 mm
 * @param  [in] autoBackFlag 自动返回标志，0-不自动；-自动
 * @param  [in] autoBackVel  自动返回速度 %
 * @param  [in] autoBackDis  自动返回距离 mm
 * @param  [in] offectFlag  1-带偏移量寻位；2-示教点寻位
 * @return  错误码
 */
std::string robot_command_thread::WireSearchEnd(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int refPos = std::stoi(list.front());list.pop_front();
    float searchVel = std::stod(list.front());list.pop_front();
    int searchDis = std::stoi(list.front());list.pop_front();
    int autoBackFlag = std::stoi(list.front());list.pop_front();
    float autoBackVel = std::stod(list.front());list.pop_front();
    int autoBackDis = std::stoi(list.front());list.pop_front();
    int offectFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WireSearchEnd(refPos, searchVel, searchDis, autoBackFlag, autoBackVel, autoBackDis, offectFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief  计算焊丝寻位偏移量
 * @param  [in] seamType  焊缝类型
 * @param  [in] method   计算方法
 * @param  [in] varNameRef 基准点1-6，“#”表示无点变量
 * @param  [in] varNameRes 接触点1-6，“#”表示无点变量
 * @param  [out] offectFlag 0-偏移量直接叠加到指令点；1-偏移量需要对指令点进行坐标变换
 * @param  [out] offect 偏移位姿[x, y, z, a, b, c]
 * @return  错误码
 */
std::string robot_command_thread::GetWireSearchOffset(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int seamType = std::stoi(list.front());list.pop_front();
    int method = std::stoi(list.front());list.pop_front();
    std::vector<std::string> varNameRef;
    for(int i = 0;i < 6; i++){
        varNameRef.push_back(list.front());
        list.pop_front();
    }
    std::vector<std::string> varNameRes;
    for(int i = 0;i < 6; i++){
        varNameRes.push_back(list.front());
        list.pop_front();
    }

    int offectFlag;
    DescPose offect;

    int res = _ptr_robot->GetWireSearchOffset(seamType, method, varNameRef, varNameRes, offectFlag, offect);
    return std::string(std::to_string(res) + "," + std::to_string(offectFlag) + "," + std::to_string(offect.tran.x) + "," + \
            std::to_string(offect.tran.y) + "," + std::to_string(offect.tran.z) + "," + \
            std::to_string(offect.rpy.rx) + "," + std::to_string(offect.rpy.ry) + "," + \
            std::to_string(offect.rpy.rz));
}

/**
 * @brief  等待焊丝寻位完成
 * @return  错误码
 */
std::string robot_command_thread::WireSearchWait(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string varName = list.front();list.pop_front();

    int res = _ptr_robot->WireSearchWait(varName);
    return std::string(std::to_string(res));
}

/**
 * @brief  焊丝寻位接触点写入数据库
 * @param  [in] varName  接触点名称 “RES0” ~ “RES99”
 * @param  [in] pos  接触点数据[x, y, x, a, b, c]
 * @return  错误码
 */
std::string robot_command_thread::SetPointToDatabase(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string varName = list.front();list.pop_front();
    DescPose pos;
    _fillDescPose(list,pos);

    int res = _ptr_robot->SetPointToDatabase(varName,pos);
    return std::string(std::to_string(res));
}

/**
 * @brief  电弧跟踪控制
 * @param  [in] flag 开关，0-关；1-开
 * @param  [in] dalayTime 滞后时间，单位ms
 * @param  [in] isLeftRight 左右偏差补偿
 * @param  [in] klr 左右调节系数(灵敏度);
 * @param  [in] tStartLr 左右开始补偿时间cyc
 * @param  [in] stepMaxLr 左右每次最大补偿量 mm
 * @param  [in] sumMaxLr 左右总计最大补偿量 mm
 * @param  [in] isUpLow 上下偏差补偿
 * @param  [in] kud 上下调节系数(灵敏度);
 * @param  [in] tStartUd 上下开始补偿时间cyc
 * @param  [in] stepMaxUd 上下每次最大补偿量 mm
 * @param  [in] sumMaxUd 上下总计最大补偿量
 * @param  [in] axisSelect 上下坐标系选择，0-摆动；1-工具；2-基座
 * @param  [in] referenceType 上下基准电流设定方式，0-反馈；1-常数
 * @param  [in] referSampleStartUd 上下基准电流采样开始计数(反馈);，cyc
 * @param  [in] referSampleCountUd 上下基准电流采样循环计数(反馈);，cyc
 * @param  [in] referenceCurrent 上下基准电流mA
 * @param  [in] offsetType 偏置跟踪类型，0-不偏置；1-采样；2-百分比
 * @param  [in] offsetParameter 偏置参数；采样(偏置采样开始时间，默认采一周期)；百分比(偏置百分比(-100 ~ 100))
 * @return  错误码
 */
std::string robot_command_thread::ArcWeldTraceControl(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int flag = std::stoi(list.front());list.pop_front();
    double delaytime = std::stod(list.front());list.pop_front();
    int isLeftRight = std::stoi(list.front());list.pop_front();
    double klr = std::stod(list.front());list.pop_front();
    double tStartLr = std::stod(list.front());list.pop_front();
    double stepMaxLr = std::stod(list.front());list.pop_front();
    double sumMaxLr = std::stod(list.front());list.pop_front();
    int isUpLow = std::stoi(list.front());list.pop_front();
    double kud = std::stod(list.front());list.pop_front();
    double tStartUd = std::stod(list.front());list.pop_front();
    double stepMaxUd = std::stod(list.front());list.pop_front();
    double sumMaxUd = std::stod(list.front());list.pop_front();
    int axisSelect = std::stoi(list.front());list.pop_front();
    int referenceType = std::stoi(list.front());list.pop_front();
    double referSampleStartUd = std::stod(list.front());list.pop_front();
    double referSampleCountUd = std::stod(list.front());list.pop_front();
    double referenceCurrent = std::stod(list.front());list.pop_front();
    int offsetType = std::stoi(list.front());list.pop_front();
    int offsetParameter = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ArcWeldTraceControl(flag, delaytime, isLeftRight, klr, tStartLr, stepMaxLr, sumMaxLr, isUpLow, kud, tStartUd, stepMaxUd, sumMaxUd, axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent, offsetType = 0, offsetParameter);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置电弧跟踪输入信号端口
 * @param  [in] channel 电弧跟踪AI通带选择,[0-3]
 */
std::string robot_command_thread::ArcWeldTraceExtAIChannelConfig(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int channel = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ArcWeldTraceExtAIChannelConfig(channel);
    return std::string(std::to_string(res));
}

/**
 * @brief  力传感器辅助拖动
 * @param  [in] status 控制状态，0-关闭；1-开启
 * @param  [in] asaptiveFlag 自适应开启标志，0-关闭；1-开启
 * @param  [in] interfereDragFlag 干涉区拖动标志，0-关闭；1-开启
 * @param  [in] ingularityConstraintsFlag 奇异点策略，0-规避；1-穿越
 * @param  [in] M 惯性系数
 * @param  [in] B 阻尼系数
 * @param  [in] K 刚度系数
 * @param  [in] F 拖动六维力阈值
 * @param  [in] Fmax 最大拖动力限制
 * @param  [in] Vmax 最大关节速度限制
 * @return  错误码
 */
std::string robot_command_thread::EndForceDragControl(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int status = std::stoi(list.front());list.pop_front();
    int asaptiveFlag = std::stoi(list.front());list.pop_front();
    int interfereDragFlag = std::stoi(list.front());list.pop_front();
    int ingularityConstraintsFlag = std::stoi(list.front());list.pop_front();
    std::vector<double> M;
    for(int i = 0;i < 6; i++){
        M.push_back(std::stod(list.front()));
        list.pop_front();
    }
    std::vector<double> B;
    for(int i = 0;i < 6; i++){
        B.push_back(std::stod(list.front()));
        list.pop_front();
    }
    std::vector<double> K;
    for(int i = 0;i < 6; i++){
        K.push_back(std::stod(list.front()));
        list.pop_front();
    }
    std::vector<double> F;
    for(int i = 0;i < 6; i++){
        F.push_back(std::stod(list.front()));
        list.pop_front();
    }
    double Fmax = std::stod(list.front());list.pop_front();
    double Vmax = std::stod(list.front());list.pop_front();


    int res = _ptr_robot->EndForceDragControl(status,asaptiveFlag,interfereDragFlag,ingularityConstraintsFlag,M,B,K,F,Fmax,Vmax);
    return std::string(std::to_string(res));
}

/**
 * @brief  报错清除后力传感器自动开启
 * @param  [in] status 控制状态，0-关闭；1-开启
 * @return  错误码
 */
std::string robot_command_thread::SetForceSensorDragAutoFlag(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int status = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetForceSensorDragAutoFlag(status);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取力传感器拖动开关状态
 * @param  [out] dragState 力传感器辅助拖动控制状态，0-关闭；1-开启
 * @param  [out] sixDimensionalDragState 六维力辅助拖动控制状态，0-关闭；1-开启
 * @return  错误码
 */
std::string robot_command_thread::GetForceAndTorqueDragState(std::string para){
    int dragState;
    int sixDimensionalDragState;

    int res = _ptr_robot->GetForceAndTorqueDragState(dragState,sixDimensionalDragState);
    return std::string(std::to_string(res) + "," + std::to_string(dragState) + "," + \
            std::to_string(sixDimensionalDragState));
}

/**
 * @brief  设置力传感器下负载重量
 * @param  [in] weight 负载重量 kg
 * @return  错误码
 */
std::string robot_command_thread::SetForceSensorPayload(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double weight = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->SetForceSensorPayload(weight);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置力传感器下负载质心
 * @param  [in] x 负载质心x mm
 * @param  [in] y 负载质心y mm
 * @param  [in] z 负载质心z mm
 * @return  错误码
 */
std::string robot_command_thread::SetForceSensorPayloadCog(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double x = std::stod(list.front());list.pop_front();
    double y = std::stod(list.front());list.pop_front();
    double z = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->SetForceSensorPayloadCog(x,y,z);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取力传感器下负载重量
 * @param  [in] weight 负载重量 kg
 * @return  错误码
 */
std::string robot_command_thread::GetForceSensorPayload(std::string para){
    double weight;

    int res = _ptr_robot->GetForceSensorPayload(weight);
    return std::string(std::to_string(res) + "," + std::to_string(weight));
}

/**
 * @brief  获取力传感器下负载重量
 * @param  [in] weight 负载重量 kg
 * @return  错误码
 */
std::string robot_command_thread::GetForceSensorPayloadCog(std::string para){
    double x;
    double y;
    double z;

    int res = _ptr_robot->GetForceSensorPayloadCog(x,y,z);
    return std::string(std::to_string(res) + "," + std::to_string(x) + "," + \
            std::to_string(y) + "," + std::to_string(z));
}

/**
 * @brief  力传感器自动校零
 * @param  [out] weight 传感器质量 kg
 * @param  [out] pos 传感器质心 mm
 * @return  错误码
 */
std::string robot_command_thread::ForceSensorAutoComputeLoad(std::string para){
    double weight;
    DescTran pos;

    int res = _ptr_robot->ForceSensorAutoComputeLoad(weight,pos);
    return std::string(std::to_string(res) + "," + std::to_string(weight) + "," +\
            std::to_string(pos.x) + "," + std::to_string(pos.y) + "," +\
            std::to_string(pos.z));
}

/**
 * @brief  传感器自动校零数据记录
 * @param  [in] recordCount 记录数据个数 1-3
 * @return  错误码
 */
std::string robot_command_thread::ForceSensorSetSaveDataFlag(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int recordCount = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->ForceSensorSetSaveDataFlag(recordCount);
    return std::string(std::to_string(res));
}

/**
 * @brief  传感器自动校零计算
 * @param  [out] weight 传感器质量 kg
 * @param  [out] pos 传感器质心 [x, y, z]
 * @return  错误码
 */
std::string robot_command_thread::ForceSensorComputeLoad(std::string para){
    double weight;
    DescTran pos;

    int res = _ptr_robot->ForceSensorComputeLoad(weight,pos);
    return std::string(std::to_string(res) + "," + std::to_string(weight) + "," +\
            std::to_string(pos.x) + "," + std::to_string(pos.y) + "," +\
            std::to_string(pos.z));
}

/**
 * @brief  段焊获取位置和姿态
 * @param  [in] startPos 起始点坐标
 * @param  [in] endPos 终止点坐标
 * @param  [in] startDistance 焊接点至起点的长度
 * @param  [out] weldPointDesc 焊接点的笛卡尔坐标信息
 * @param  [out] weldPointJoint 焊接点的笛卡尔坐标信息
 * @param  [out] tool 工具号
 * @param  [out] user 工件号
 * @return  错误码
 */
std::string robot_command_thread::GetSegmentWeldPoint(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);
    DescPose startPos;
    _fillDescPose(list,startPos);
    DescPose endPos;
    _fillDescPose(list,endPos);
    double startDistance = std::stod(list.front());list.pop_front();

    DescPose weldPointDesc;
    JointPos weldPointJoint;
    int tool;
    int user;

    int res = _ptr_robot->GetSegmentWeldPoint(startPos,endPos,startDistance,weldPointDesc,weldPointJoint,tool,user);
    return std::string(std::to_string(res) + "," + std::to_string(weldPointDesc.tran.x) + "," + \
            std::to_string(weldPointDesc.tran.y) + "," + std::to_string(weldPointDesc.tran.z) + "," + \
            std::to_string(weldPointDesc.rpy.rx) + "," + std::to_string(weldPointDesc.rpy.ry) + "," + \
            std::to_string(weldPointDesc.rpy.rz) + "," + std::to_string(weldPointJoint.jPos[0]) + "," +\
            std::to_string(weldPointJoint.jPos[1]) + "," + std::to_string(weldPointJoint.jPos[2]) + "," +\
            std::to_string(weldPointJoint.jPos[3]) + "," + std::to_string(weldPointJoint.jPos[4]) + "," +\
            std::to_string(weldPointJoint.jPos[5]) + "," + std::to_string(tool) + "," +\
            std::to_string(user));
}

/**
 * @brief  设置焊接工艺曲线参数
 * @param  [in] id 焊接工艺编号(1-99)
 * @param  [in] startCurrent 起弧电流(A)
 * @param  [in] startVoltage 起弧电压(V)
 * @param  [in] startTime 起弧时间(ms)
 * @param  [in] weldCurrent 焊接电流(A)
 * @param  [in] weldVoltage 焊接电压(V)
 * @param  [in] endCurrent 收弧电流(A)
 * @param  [in] endVoltage 收弧电压(V)
 * @param  [in] endTime 收弧时间(ms)
 * @return  错误码
 */
std::string robot_command_thread::WeldingSetProcessParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    double startCurrent = std::stod(list.front());list.pop_front();
    double startVoltage = std::stod(list.front());list.pop_front();
    double startTime = std::stod(list.front());list.pop_front();
    double weldCurrent = std::stod(list.front());list.pop_front();
    double weldVoltage = std::stod(list.front());list.pop_front();
    double endCurrent = std::stod(list.front());list.pop_front();
    double endVoltage = std::stod(list.front());list.pop_front();
    double endTime = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->WeldingSetProcessParam(id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取焊接工艺曲线参数
 * @param  [in] id 焊接工艺编号(1-99)
 * @param  [out] startCurrent 起弧电流(A)
 * @param  [out] startVoltage 起弧电压(V)
 * @param  [out] startTime 起弧时间(ms)
 * @param  [out] weldCurrent 焊接电流(A)
 * @param  [out] weldVoltage 焊接电压(V)
 * @param  [out] endCurrent 收弧电流(A)
 * @param  [out] endVoltage 收弧电压(V)
 * @param  [out] endTime 收弧时间(ms)
 * @return  错误码
 */
std::string robot_command_thread::WeldingGetProcessParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    double startCurrent;
    double startVoltage;
    double startTime;
    double weldCurrent;
    double weldVoltage;
    double endCurrent;
    double endVoltage;
    double endTime;

    int res = _ptr_robot->WeldingGetProcessParam(id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
    return std::string(std::to_string(res) + "," + std::to_string(startCurrent) + "," + \
            std::to_string(startVoltage) + "," + std::to_string(startTime) + "," + \
            std::to_string(weldCurrent) + "," + std::to_string(weldVoltage) + "," + \
            std::to_string(endCurrent) + "," + std::to_string(endVoltage) + "," + \
            std::to_string(endTime));
}

/**
 * @brief  末端传感器配置
 * @param  [in] idCompany 厂商，18-JUNKONG；25-HUIDE
 * @param  [in] idDevice 类型，0-JUNKONG/RYR6T.V1.0
 * @param  [in] idSoftware 软件版本，0-J1.0/HuiDe1.0(暂未开放)
 * @param  [in] idBus 挂载位置，1-末端1号口；2-末端2号口...8-末端8号口(暂未开放)
 * @return  错误码
 */
std::string robot_command_thread::AxleSensorConfig(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int idCompany = std::stoi(list.front());list.pop_front();
    int idDevice = std::stoi(list.front());list.pop_front();
    int idSoftware = std::stoi(list.front());list.pop_front();
    int idBus = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->AxleSensorConfig(idCompany, idDevice, idSoftware, idBus);
    return std::string(std::to_string(res));
}

/**
 * @brief  末端传感器配置
 * @param  [in] idCompany 厂商，18-JUNKONG；25-HUIDE
 * @param  [in] idDevice 类型，0-JUNKONG/RYR6T.V1.0
 * @param  [in] idSoftware 软件版本，0-J1.0/HuiDe1.0(暂未开放)
 * @param  [in] idBus 挂载位置，1-末端1号口；2-末端2号口...8-末端8号口(暂未开放)
 * @return  错误码
 */
std::string robot_command_thread::AxleSensorConfigGet(std::string para){
    int idCompany;
    int idDevice;
    int res = _ptr_robot->AxleSensorConfigGet(idCompany, idDevice);
    return std::string(std::to_string(res) + "," + std::to_string(idCompany) + "," + \
            std::to_string(idDevice));
}

/**
 * @brief  末端传感器激活
 * @param  [in] actFlag 0-复位；1-激活
 * @return  错误码
 */
std::string robot_command_thread::AxleSensorActivate(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int actFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->AxleSensorActivate(actFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置控制箱DO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
std::string robot_command_thread::SetOutputResetCtlBoxDO(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int resetFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetOutputResetCtlBoxDO(resetFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置控制箱AO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
std::string robot_command_thread::SetOutputResetCtlBoxAO(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int resetFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetOutputResetCtlBoxAO(resetFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置末端工具DO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
std::string robot_command_thread::SetOutputResetAxleDO(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int resetFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetOutputResetAxleDO(resetFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置末端工具AO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
std::string robot_command_thread::SetOutputResetAxleAO(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int resetFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetOutputResetAxleAO(resetFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置扩展DO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
std::string robot_command_thread::SetOutputResetExtDO(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int resetFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetOutputResetExtDO(resetFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置扩展AO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
std::string robot_command_thread::SetOutputResetExtAO(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int resetFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetOutputResetExtAO(resetFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief  设置SmartTool停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
std::string robot_command_thread::SetOutputResetSmartToolDO(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int resetFlag = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetOutputResetSmartToolDO(resetFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief  仿真摆动开始
 * @param  [in] weaveNum  摆动参数编号
 * @return  错误码
 */
std::string robot_command_thread::WeaveStartSim(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int weaveNum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeaveStartSim(weaveNum);
    return std::string(std::to_string(res));
}

/**
 * @brief  仿真摆动结束
 * @param  [in] weaveNum  摆动参数编号
 * @return  错误码
 */
std::string robot_command_thread::WeaveEndSim(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int weaveNum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeaveEndSim(weaveNum);
    return std::string(std::to_string(res));
}

/**
 * @brief  开始轨迹检测预警(不运动)
 * @param  [in] weaveNum  摆动参数编号
 * @return  错误码
 */
std::string robot_command_thread::WeaveInspectStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int weaveNum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeaveInspectStart(weaveNum);
    return std::string(std::to_string(res));
}

/**
 * @brief  结束轨迹检测预警(不运动)
 * @param  [in] weaveNum  摆动参数编号
 * @return  错误码
 */
std::string robot_command_thread::WeaveInspectEnd(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int weaveNum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WeaveInspectEnd(weaveNum);
    return std::string(std::to_string(res));
}

/**
 * @brief 扩展IO-配置焊机气体检测信号
 * @param  [in] DONum  气体检测信号扩展DO编号
 * @return  错误码
 */
std::string robot_command_thread::SetAirControlExtDoNum(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int DONum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetAirControlExtDoNum(DONum);
    return std::string(std::to_string(res));
}

/**
 * @brief 扩展IO-配置焊机起弧信号
 * @param  [in] DONum  气体检测信号扩展DO编号
 * @return  错误码
 */
std::string robot_command_thread::SetArcStartExtDoNum(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int DONum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetArcStartExtDoNum(DONum);
    return std::string(std::to_string(res));
}

/**
 * @brief 扩展IO-配置焊机反向送丝信号
 * @param  [in] DONum  气体检测信号扩展DO编号
 * @return  错误码
 */
std::string robot_command_thread::SetWireReverseFeedExtDoNum(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int DONum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetWireReverseFeedExtDoNum(DONum);
    return std::string(std::to_string(res));
}

/**
 * @brief 扩展IO-配置焊机正向送丝信号
 * @param  [in] DONum  气体检测信号扩展DO编号
 * @return  错误码
 */
std::string robot_command_thread::SetWireForwardFeedExtDoNum(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int DONum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetWireForwardFeedExtDoNum(DONum);
    return std::string(std::to_string(res));
}

/**
 * @brief 扩展IO-配置焊机起弧成功信号
 * @param  [in] DINum  起弧成功信号扩展DI编号
 * @return  错误码
 */
std::string robot_command_thread::SetArcDoneExtDiNum(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int DINum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetArcDoneExtDiNum(DINum);
    return std::string(std::to_string(res));
}

/**
 * @brief 扩展IO-配置焊机准备信号
 * @param  [in] DINum  焊机准备信号扩展DI编号
 * @return  错误码
 */
std::string robot_command_thread::SetWeldReadyExtDiNum(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int DINum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetWeldReadyExtDiNum(DINum);
    return std::string(std::to_string(res));
}

/**
 * @brief 扩展IO-配置焊接中断恢复信号
 * @param  [in] reWeldDINum  焊接中断后恢复焊接信号扩展DI编号
 * @param  [in] abortWeldDINum  焊接中断后退出焊接信号扩展DI编号
 * @return  错误码
 */
std::string robot_command_thread::SetExtDIWeldBreakOffRecover(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int reWeldDINum = std::stoi(list.front());list.pop_front();
    int abortWeldDINum = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetExtDIWeldBreakOffRecover(reWeldDINum,abortWeldDINum);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置机器人碰撞检测方法
 * @param [in] method 碰撞检测方法：0-电流模式；1-双编码器；2-电流和双编码器同时开启
 * @param [in] thresholdMode 碰撞等级阈值方式；0-碰撞等级固定阈值方式；1-自定义碰撞检测阈值
 * @return  错误码
 */
std::string robot_command_thread::SetCollisionDetectionMethod(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int method = std::stoi(list.front());list.pop_front();
    int thresholdMode = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetCollisionDetectionMethod(method,thresholdMode);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置静态下碰撞检测开始关闭
 * @param  [in] status 0-关闭；1-开启
 * @return  错误码
 */
std::string robot_command_thread::SetStaticCollisionOnOff(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int status = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetStaticCollisionOnOff(status);
    return std::string(std::to_string(res));
}

/**
 * @brief 关节扭矩功率检测
 * @param  [in] status 0-关闭；1-开启
 * @param  [in] power 设定最大功率(W);
 * @return  错误码
 */
std::string robot_command_thread::SetPowerLimit(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int status = std::stoi(list.front());list.pop_front();
    double power = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->SetPowerLimit(status,power);
    return std::string(std::to_string(res));
}

/**
 * @brief 关节扭矩控制开始
 * @return  错误码
 */
std::string robot_command_thread::ServoJTStart(std::string para){
    int res = _ptr_robot->ServoJTStart();
    return std::string(std::to_string(res));
}

/**
 * @brief 关节扭矩控制
 * @param  [in] torque j1~j6关节扭矩，单位Nm
 * @param  [in] interval 指令周期，单位s，范围[0.001~0.008]
 * @return  错误码
 */
std::string robot_command_thread::ServoJT(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    float torque[6];
    torque[0] = std::stod(list.front());list.pop_front();
    torque[1] = std::stod(list.front());list.pop_front();
    torque[2] = std::stod(list.front());list.pop_front();
    torque[3] = std::stod(list.front());list.pop_front();
    torque[4] = std::stod(list.front());list.pop_front();
    torque[5] = std::stod(list.front());list.pop_front();
    double interval = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->ServoJT(torque,interval);
    return std::string(std::to_string(res));
}

/**
 * @brief 关节扭矩控制结束
 * @return  错误码
 */
std::string robot_command_thread::ServoJTEnd(std::string para){
    int res = _ptr_robot->ServoJTEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief 设置机器人 20004 端口反馈周期
 * @param [in] period 机器人 20004 端口反馈周期(ms)
 * @return  错误码
 */
std::string robot_command_thread::SetRobotRealtimeStateSamplePeriod(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int period = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetRobotRealtimeStateSamplePeriod(period);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取机器人 20004 端口反馈周期
 * @param [out] period 机器人 20004 端口反馈周期(ms)
 * @return  错误码
 */
std::string robot_command_thread::GetRobotRealtimeStateSamplePeriod(std::string para){
    int period;

    int res = _ptr_robot->GetRobotRealtimeStateSamplePeriod(period);
    return std::string(std::to_string(res) + "," + std::to_string(period));
}

/**
 * @brief 获取机器人关节驱动器温度(℃)
 * @return  错误码
 */
std::string robot_command_thread::GetJointDriverTemperature(std::string para){
    double temperature[6];

    int res = _ptr_robot->GetJointDriverTemperature(temperature);
    return std::string(std::to_string(res) + "," + std::to_string(temperature[0]) + "," +\
            std::to_string(temperature[1]) + "," + std::to_string(temperature[2]) + "," +\
            std::to_string(temperature[3]) + "," + std::to_string(temperature[4]) + "," +\
            std::to_string(temperature[5]));
}

/**
 * @brief 获取机器人关节驱动器扭矩(Nm)
 * @return  错误码
 */
std::string robot_command_thread::GetJointDriverTorque(std::string para){
    double torque[6];

    int res = _ptr_robot->GetJointDriverTorque(torque);
    return std::string(std::to_string(res) + "," + std::to_string(torque[0]) + "," +\
            std::to_string(torque[1]) + "," + std::to_string(torque[2]) + "," +\
            std::to_string(torque[3]) + "," + std::to_string(torque[4]) + "," +\
            std::to_string(torque[5]));
}

/**
 * @brief 电弧追踪 + 多层多道补偿开启
 * @return  错误码
 */
std::string robot_command_thread::ArcWeldTraceReplayStart(std::string para){

    int res = _ptr_robot->ArcWeldTraceReplayStart();
    return std::string(std::to_string(res));
}

/**
 * @brief 电弧追踪 + 多层多道补偿关闭
 * @return  错误码
 */
std::string robot_command_thread::ArcWeldTraceReplayEnd(std::string para){

    int res = _ptr_robot->ArcWeldTraceReplayEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief  偏移量坐标变化-多层多道焊
 * @param  [in] pointo 基准点笛卡尔位姿
 * @param  [in] pointX 基准点X向偏移方向点笛卡尔位姿
 * @param  [in] pointZ 基准点Z向偏移方向点笛卡尔位姿
 * @param  [in] dx x方向偏移量(mm)
 * @param  [in] dz z方向偏移量(mm)
 * @param  [in] dry 绕y轴偏移量(°)
 * @return 指令执行是否成功
 * @retval 0-成功，其他-错误码 
 */
std::string robot_command_thread::MultilayerOffsetTrsfToBase(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    DescTran pointO;
    pointO.x = std::stod(list.front().c_str());list.pop_front();
    pointO.y = std::stod(list.front().c_str());list.pop_front();
    pointO.z = std::stod(list.front().c_str());list.pop_front();
    DescTran pointX;
    pointX.x = std::stod(list.front().c_str());list.pop_front();
    pointX.y = std::stod(list.front().c_str());list.pop_front();
    pointX.z = std::stod(list.front().c_str());list.pop_front();
    DescTran pointZ;
    pointZ.x = std::stod(list.front().c_str());list.pop_front();
    pointZ.y = std::stod(list.front().c_str());list.pop_front();
    pointZ.z = std::stod(list.front().c_str());list.pop_front();
    double dx = std::stod(list.front().c_str());list.pop_front();
    double dy = std::stod(list.front().c_str());list.pop_front();
    double db = std::stod(list.front().c_str());list.pop_front();

    DescPose offset;
    
    int res = _ptr_robot->MultilayerOffsetTrsfToBase(pointO, pointX, pointZ, dx, dy, db, offset);
    return std::string(std::to_string(res) + "," + std::to_string(offset.tran.x) + "," + \
            std::to_string(offset.tran.y) + "," + std::to_string(offset.tran.z) + "," + \
            std::to_string(offset.rpy.rx) + "," + std::to_string(offset.rpy.ry) + "," + \
            std::to_string(offset.rpy.rz));
}

/**
 * @brief 指定姿态速度开启
 * @param [in] ratio 姿态速度百分比[0-300]
 * @return  错误码
 */
std::string robot_command_thread::AngularSpeedStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int ratio = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->AngularSpeedStart(ratio);
    return std::string(std::to_string(res));
}

/**
 * @brief 指定姿态速度关闭
 * @return  错误码
 */
std::string robot_command_thread::AngularSpeedEnd(std::string para){
    int res = _ptr_robot->AngularSpeedEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief 指定姿态速度开启
 * @param [in] ratio 姿态速度百分比[0-300]
 * @return  错误码
 */
std::string robot_command_thread::SoftwareUpgrade(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string filePath = list.front();list.pop_front();
    int block = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SoftwareUpgrade(filePath,block);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取机器人软件升级状态
 * @param [out] state 机器人软件包升级状态(0-空闲中或上传升级包中；1~100：升级完成百分比；-1:升级软件失败；-2：校验失败；-3：版本校验失败；-4：解压失败；-5：用户配置升级失败；-6：外设配置升级失败；-7：扩展轴配置升级失败；-8：机器人配置升级失败；-9：DH参数配置升级失败)
 * @return  错误码
 */
std::string robot_command_thread::GetSoftwareUpgradeState(std::string para){
    int state;

    int res = _ptr_robot->GetSoftwareUpgradeState(state);
    return std::string(std::to_string(res) + "," + std::to_string(state));
}

/**
 * @brief 设置485扩展轴运动加减速度
 * @param [in] acc 485扩展轴运动加速度
 * @param [in] dec 485扩展轴运动减速度
 * @return  错误码
 */
std::string robot_command_thread::AuxServoSetAcc(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double acc = std::stod(list.front());list.pop_front();
    double dec = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->AuxServoSetAcc(acc,dec);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置485扩展轴急停加减速度
 * @param [in] acc 485扩展轴急停加速度
 * @param [in] dec 485扩展轴急停减速度
 * @return  错误码
 */
std::string robot_command_thread::AuxServoSetEmergencyStopAcc(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double acc = std::stod(list.front());list.pop_front();
    double dec = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->AuxServoSetEmergencyStopAcc(acc,dec);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取485扩展轴运动加减速度
 * @param [out] acc 485扩展轴运动加速度
 * @param [out] dec 485扩展轴运动减速度
 * @return  错误码
 */
std::string robot_command_thread::AuxServoGetAcc(std::string para){
    double acc;
    double dec;

    int res = _ptr_robot->AuxServoGetAcc(acc,dec);
    return std::string(std::to_string(res) + "," + std::to_string(acc) + "," + \
            std::to_string(dec));
}

/**
 * @brief 获取485扩展轴急停加减速度
 * @param [out] acc 485扩展轴急停加速度
 * @param [out] dec 485扩展轴急停减速度
 * @return  错误码
 */
std::string robot_command_thread::AuxServoGetEmergencyStopAcc(std::string para){
    double acc;
    double dec;

    int res = _ptr_robot->AuxServoGetEmergencyStopAcc(acc,dec);
    return std::string(std::to_string(res) + "," + std::to_string(acc) + "," + \
            std::to_string(dec));
}

/**
 * @brief 获取末端通讯参数
 * @param param 末端通讯参数
 * @return  错误码
 */
std::string robot_command_thread::GetAxleCommunicationParam(std::string para){
    AxleComParam param;

    int res = _ptr_robot->GetAxleCommunicationParam(&param);
    return std::string(std::to_string(res) + "," + std::to_string(param.baudRate) + "," + \
            std::to_string(param.dataBit) + "," + std::to_string(param.stopBit) + "," + \
            std::to_string(param.verify) + "," + std::to_string(param.timeout) + "," + \
            std::to_string(param.timeoutTimes) + "," + std::to_string(param.period));
}

/**
 * @brief 设置末端通讯参数
 * @param param  末端通讯参数
 * @return  错误码
 */
std::string robot_command_thread::SetAxleCommunicationParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    AxleComParam param;
    param.baudRate = std::stoi(list.front());list.pop_front();
	param.dataBit = std::stoi(list.front());list.pop_front(); 
	param.stopBit = std::stoi(list.front());list.pop_front(); 
	param.verify = std::stoi(list.front());list.pop_front();  
	param.timeout = std::stoi(list.front());list.pop_front(); 
	param.timeoutTimes = std::stoi(list.front());list.pop_front();
	param.period = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetAxleCommunicationParam(param);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置末端文件传输类型
 * @param type 1-MCU升级文件；2-LUA文件
 * @return  错误码
 */
std::string robot_command_thread::SetAxleFileType(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetAxleFileType(type);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置启用末端LUA执行
 * @param enable 0-不启用；1-启用
 * @return  错误码
 */
std::string robot_command_thread::SetAxleLuaEnable(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int enable = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetAxleLuaEnable(enable);
    return std::string(std::to_string(res));
}

/**
 * @brief 末端LUA文件异常错误恢复
 * @param status 0-不恢复；1-恢复
 * @return  错误码
 */
std::string robot_command_thread::SetRecoverAxleLuaErr(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int status = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetRecoverAxleLuaErr(status);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取末端LUA执行使能状态
 * @param status status[0]: 0-未使能；1-已使能
 * @return  错误码
 */
std::string robot_command_thread::GetAxleLuaEnableStatus(std::string para){
    int status[1];

    int res = _ptr_robot->GetAxleLuaEnableStatus(status);
    return std::string(std::to_string(res) + "," + std::to_string(status[0]));
}

/**
 * @brief 设置末端LUA末端设备启用类型
 * @param forceSensorEnable 力传感器启用状态，0-不启用；1-启用
 * @param gripperEnable 夹爪启用状态，0-不启用；1-启用
 * @param IOEnable IO设备启用状态，0-不启用；1-启用
 * @return  错误码
 */
std::string robot_command_thread::SetAxleLuaEnableDeviceType(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int forceSensorEnable = std::stoi(list.front());list.pop_front();
    int gripperEnable = std::stoi(list.front());list.pop_front();
    int IOEnable = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetAxleLuaEnableDeviceType(forceSensorEnable,gripperEnable,IOEnable);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取末端LUA末端设备启用类型
 * @param enable enable[0]:forceSensorEnable 力传感器启用状态，0-不启用；1-启用
 * @param enable enable[1]:gripperEnable 夹爪启用状态，0-不启用；1-启用
 * @param enable enable[2]:IOEnable IO设备启用状态，0-不启用；1-启用
 * @return  错误码
 */
std::string robot_command_thread::GetAxleLuaEnableDeviceType(std::string para){
    int forceSensorEnable;
    int gripperEnable;
    int IOEnable;

    int res = _ptr_robot->GetAxleLuaEnableDeviceType(&forceSensorEnable,&gripperEnable,&IOEnable);
    return std::string(std::to_string(res) + "," + std::to_string(forceSensorEnable) + "," + \
            std::to_string(gripperEnable) + "," + std::to_string(IOEnable));
}

/**
 * @brief 获取当前配置的末端设备
 * @param forceSensorEnable 力传感器启用设备编号 0-未启用；1-启用
 * @param gripperEnable 夹爪启用设备编号，0-不启用；1-启用
 * @param IODeviceEnable IO设备启用设备编号，0-不启用；1-启用
 * @return  错误码
 */
std::string robot_command_thread::GetAxleLuaEnableDevice(std::string para){
    int forceSensorEnable[8];
    int gripperEnable[8];
    int IOEnable[8];

    int res = _ptr_robot->GetAxleLuaEnableDevice(forceSensorEnable,gripperEnable,IOEnable);
    return std::string(std::to_string(res) + "," + std::to_string(forceSensorEnable[0]) + "," + \
            std::to_string(forceSensorEnable[1]) + "," + std::to_string(forceSensorEnable[2]) + "," + \
            std::to_string(forceSensorEnable[3]) + "," + std::to_string(forceSensorEnable[4]) + "," + \
            std::to_string(forceSensorEnable[5]) + "," + std::to_string(forceSensorEnable[6]) + "," + \
            std::to_string(forceSensorEnable[7]) + "," + std::to_string(gripperEnable[0]) + "," + \
            std::to_string(gripperEnable[1]) + "," + std::to_string(gripperEnable[2]) + "," + \
            std::to_string(gripperEnable[3]) + "," + std::to_string(gripperEnable[4]) + "," + \
            std::to_string(gripperEnable[5]) + "," + std::to_string(gripperEnable[6]) + "," + \
            std::to_string(gripperEnable[7]) + "," + std::to_string(IOEnable[0]) + "," + \
            std::to_string(IOEnable[1]) + "," + std::to_string(IOEnable[2]) + "," + \
            std::to_string(IOEnable[3]) + "," + std::to_string(IOEnable[4]) + "," + \
            std::to_string(IOEnable[5]) + "," + std::to_string(IOEnable[6]) + "," + \
            std::to_string(IOEnable[7]));
}

/**
 * @brief 设置启用夹爪动作控制功能
 * @param id 夹爪设备编号
 * @param func func[0]-夹爪使能；func[1]-夹爪初始化；2-位置设置；3-速度设置；4-力矩设置；6-读夹爪状态；7-读初始化状态；8-读故障码；9-读位置；10-读速度；11-读力矩
 * @return  错误码
 */
std::string robot_command_thread::SetAxleLuaGripperFunc(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    int len = list.size();
    int func[len];
    for(int i = 0; i < len;i++){
        func[i] = std::stoi(list.front());list.pop_front();
    }
    
    int res = _ptr_robot->SetAxleLuaGripperFunc(id,func);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取启用夹爪动作控制功能
 * @param id 夹爪设备编号
 * @param func func[0]-夹爪使能；func[1]-夹爪初始化；2-位置设置；3-速度设置；4-力矩设置；6-读夹爪状态；7-读初始化状态；8-读故障码；9-读位置；10-读速度；11-读力矩
 * @return  错误码
 */
std::string robot_command_thread::GetAxleLuaGripperFunc(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    int func[16];
    
    int res = _ptr_robot->GetAxleLuaGripperFunc(id,func);
    return std::string(std::to_string(res) + "," + std::to_string(func[0]) + "," + \
            std::to_string(func[1]) + "," + std::to_string(func[2]) + "," + \
            std::to_string(func[3]) + "," + std::to_string(func[4]) + "," + \
            std::to_string(func[5]) + "," + std::to_string(func[6]) + "," + \
            std::to_string(func[7]) + "," + std::to_string(func[8]) + "," + \
            std::to_string(func[9]) + "," + std::to_string(func[10]) + "," + \
            std::to_string(func[11]) + "," + std::to_string(func[12]) + "," + \
            std::to_string(func[13]) + "," + std::to_string(func[14]) + "," + \
            std::to_string(func[15]));
}

/**
 * @brief 设置控制器外设协议LUA文件名
 * @param id 协议编号
 * @param name lua文件名称 “CTRL_LUA_test.lua”
 * @return  错误码
 */
std::string robot_command_thread::SetCtrlOpenLUAName(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    std::string name = list.front();list.pop_front();
    
    int res = _ptr_robot->SetCtrlOpenLUAName(id,name);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置控制器外设协议LUA文件名
 * @param id 协议编号
 * @param name lua文件名称 “CTRL_LUA_test.lua”
 * @return  错误码
 */
std::string robot_command_thread::GetCtrlOpenLUAName(std::string para){
    std::string name[4];
    
    int res = _ptr_robot->GetCtrlOpenLUAName(name);
    return std::string(std::to_string(res) + "," + std::string(name[0]) + "," + \
            std::string(name[1]) + "," + std::string(name[2]) + "," + \
            std::string(name[3]));
}

/**
 * @brief 加载控制器LUA协议
 * @param id 控制器LUA协议编号
 * @return  错误码
 */
std::string robot_command_thread::LoadCtrlOpenLUA(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LoadCtrlOpenLUA(id);
    return std::string(std::to_string(res));
}

/**
 * @brief 卸载控制器LUA协议
 * @param id 控制器LUA协议编号
 * @return  错误码
 */
std::string robot_command_thread::UnloadCtrlOpenLUA(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->UnloadCtrlOpenLUA(id);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置控制器LUA协议错误码
 * @param id 控制器LUA协议编号
 * @return  错误码
 */
std::string robot_command_thread::SetCtrlOpenLuaErrCode(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    int code = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->SetCtrlOpenLuaErrCode(id,code);
    return std::string(std::to_string(res));
}

/**
 * @brief 机器人Ethercat从站文件写入
 * @param type 从站文件类型，1-升级从站文件；2-升级从站配置文件
 * @param slaveID 从站号
 * @param fileName 上传文件名
 * @return  错误码
 */
std::string robot_command_thread::SlaveFileWrite(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front());list.pop_front();
    int slaveID = std::stoi(list.front());list.pop_front();
    std::string fileName = list.front();list.pop_front();
    
    int res = _ptr_robot->SlaveFileWrite(type,slaveID,fileName);
    return std::string(std::to_string(res));
}

/**
 * @brief 上传末端Lua开放协议文件
 * @param filePath 本地lua文件路径名 ".../AXLE_LUA_End_DaHuan.lua"
 * @return  错误码
 */
std::string robot_command_thread::AxleLuaUpload(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string filePath = list.front();list.pop_front();
    
    int res = _ptr_robot->AxleLuaUpload(filePath);
    return std::string(std::to_string(res));
}

/**
 * @brief 机器人Ethercat从站进入boot模式
 * @return  错误码
 */
std::string robot_command_thread::SetSysServoBootMode(std::string para){
    int res = _ptr_robot->SetSysServoBootMode();
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焊丝寻位扩展IO端口
 * @param searchDoneDINum 焊丝寻位成功DO端口(0-127)
 * @param searchStartDONum 焊丝寻位启停控制DO端口(0-127)
 * @return  错误码
 */
std::string robot_command_thread::SetWireSearchExtDIONum(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int searchDoneDINum = std::stoi(list.front());list.pop_front();
    int searchStartDONum = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->SetWireSearchExtDIONum(searchDoneDINum,searchStartDONum);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焊机控制模式扩展DO端口
 * @param DONum 焊机控制模式DO端口(0-127)
 * @return  错误码
 */
std::string robot_command_thread::SetWeldMachineCtrlModeExtDoNum(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int DONum = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->SetWeldMachineCtrlModeExtDoNum(DONum);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焊机控制模式
 * @param mode 焊机控制模式;0-一元化
 * @return  错误码
 */
std::string robot_command_thread::SetWeldMachineCtrlMode(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int mode = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->SetWeldMachineCtrlMode(mode);
    return std::string(std::to_string(res));
}

/**
 * @brief 开始奇异位姿保护
 * @param [in] protectMode 奇异保护模式，0：关节模式；1-笛卡尔模式
 * @param [in] minShoulderPos 肩奇异调整范围(mm), 默认100
 * @param [in] minElbowPos 肘奇异调整范围(mm), 默认50
 * @param [in] minWristPos 腕奇异调整范围(°), 默认10
 */
std::string robot_command_thread::SingularAvoidStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int protectMode = std::stoi(list.front());list.pop_front();
    int minShoulderPos = std::stoi(list.front());list.pop_front();
    int minElbowPos = std::stoi(list.front());list.pop_front();
    int minWristPos = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->SingularAvoidStart(protectMode,minShoulderPos,minElbowPos,minWristPos);
    return std::string(std::to_string(res));
}

/**
 * @brief 停止奇异位姿保护
 * @return 错误码
 */
std::string robot_command_thread::SingularAvoidEnd(std::string para){
    int res = _ptr_robot->SingularAvoidEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief 开始Ptp运动FIR滤波
 * @param [in] maxAcc 最大加速度极值(deg/s2)
 * @param [in] maxJek 统一关节急动度极值(deg/s3)
 * @return 错误码
 */
std::string robot_command_thread::PtpFIRPlanningStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double maxAcc = std::stod(list.front());list.pop_front();
    double maxJek = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->PtpFIRPlanningStart(maxAcc,maxJek);
    return std::string(std::to_string(res));
}

/**
 * @brief 关闭Ptp运动FIR滤波
 * @return 错误码
 */
std::string robot_command_thread::PtpFIRPlanningEnd(std::string para){
    int res = _ptr_robot->PtpFIRPlanningEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief 开始LIN、ARC运动FIR滤波
 * @param [in] maxAccLin 线加速度极值(mm/s2)
 * @param [in] maxAccDeg 角加速度极值(deg/s2)
 * @param [in] maxJerkLin 线加加速度极值(mm/s3)
 * @param [in] maxJerkDeg 角加加速度极值(deg/s3)
 * @return 错误码
 */
std::string robot_command_thread::LinArcFIRPlanningStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double maxAccLin = std::stod(list.front());list.pop_front();
    double maxAccDeg = std::stod(list.front());list.pop_front();
    double maxJerkLin = std::stod(list.front());list.pop_front();
    double maxJerkDeg = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->LinArcFIRPlanningStart(maxAccLin,maxAccDeg,maxJerkLin,maxJerkDeg);
    return std::string(std::to_string(res));
}

/**
 * @brief 关闭LIN、ARC运动FIR滤波
 * @return 错误码
 */
std::string robot_command_thread::LinArcFIRPlanningEnd(std::string para){
    int res = _ptr_robot->LinArcFIRPlanningEnd();
    return std::string(std::to_string(res));
}


/**
 * @brief 激光轨迹记录
 * @param [in] status 0-停止记录；1-实时跟踪；2-开始记录；3-轨迹复现；4-边记录边复现
 * @param [in] delayMode 数据处理方式。0-延时时间；1-延时距离
 * @param [in] delayTime 激光传感器起始点运动到机器人焊枪处所需要的时间(ms)
 * @param [in] delayDisExAxisNum 延时距离对应外部轴号，bit0-3对应轴1-4
 * @param [in] delayDis 激光传感器起始点运动到机器人焊枪处所需要的距离(mm/°)
 * @param [in] sensitivePara 补偿灵敏度系数(0~1)
 * @param [in] trackMode 定点跟踪类型。0-扩展轴异步运动；1-机器人
 * @param [in] triggerMode 定点跟踪触发方式。0-跟踪时长；1-IO
 * @param [in] runTime 机器人定点跟踪时长(s)
 * @param [in] speed 机器人运动速度百分比
 * @return 错误码
 */
std::string robot_command_thread::LaserSensorRecord(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int status = std::stoi(list.front());list.pop_front();
    int delayMode = std::stoi(list.front());list.pop_front();
    int delayTime = std::stoi(list.front());list.pop_front();
    int delayDisExAxisNum = std::stoi(list.front());list.pop_front();
    double delayDis = std::stod(list.front());list.pop_front();
    double sensitivePara = std::stod(list.front());list.pop_front();
    int trackMode = std::stoi(list.front());list.pop_front();
    int triggerMode = std::stoi(list.front());list.pop_front();
    int runTime = std::stoi(list.front());list.pop_front();
    double speed = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserSensorRecord(status, delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, trackMode, triggerMode, runTime, speed);
    return std::string(std::to_string(res));
}

std::string robot_command_thread::LaserTrackingLaserOn(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int weldId = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserTrackingLaserOn(weldId);
    return std::string(std::to_string(res));
}

std::string robot_command_thread::LaserTrackingLaserOff(std::string para){
    int res = _ptr_robot->LaserTrackingLaserOff();
    return std::string(std::to_string(res));
}

std::string robot_command_thread::LaserTrackingTrackOn(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int coordId = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserTrackingTrackOn(coordId);
    return std::string(std::to_string(res));
}

std::string robot_command_thread::LaserTrackingTrackOff(std::string para){
    int res = _ptr_robot->LaserTrackingTrackOff();
    return std::string(std::to_string(res));
}

std::string robot_command_thread::LaserTrackingSearchStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int direction = std::stoi(list.front());list.pop_front();
    DescTran directionPoint;
    directionPoint.x = std::stod(list.front().c_str());list.pop_front();
    directionPoint.y = std::stod(list.front().c_str());list.pop_front();
    directionPoint.z = std::stod(list.front().c_str());list.pop_front();
    int vel = std::stoi(list.front());list.pop_front();
    int distance = std::stoi(list.front());list.pop_front();
    int timeout = std::stoi(list.front());list.pop_front();
    int posSensorNum = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserTrackingSearchStart(direction, directionPoint, vel, distance, timeout, posSensorNum);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光外设打开关闭函数
 * @param [in] OnOff 0-关闭 1-打开
 * @param [in] weldId 焊缝ID 默认为0
 * @return 错误码
 */
std::string robot_command_thread::LaserTrackingLaserOnOff(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int OnOff = std::stoi(list.front());list.pop_front();
    int weldId = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserTrackingLaserOnOff(OnOff,weldId);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光跟踪开始结束函数
 * @param [in] OnOff 0-结束 1-开始
 * @param [in] coordId 激光外设工具坐标系编号
 * @return 错误码
 */
std::string robot_command_thread::LaserTrackingTrackOnOff(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int OnOff = std::stoi(list.front());list.pop_front();
    int coordId = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserTrackingTrackOnOff(OnOff,coordId);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光寻位-固定方向
 * @param [in] direction 0-x+ 1-x- 2-y+ 3-y- 4-z+ 5-z-
 * @param [in] vel 速度 单位%
 * @param [in] distance 最大寻位距离 单位mm
 * @param [in] distance 寻位超时时间 单位ms
 * @param [in] posSensorNum 激光标定的工具坐标编号
 * @return 错误码
 */
std::string robot_command_thread::LaserTrackingSearchStart_xyz(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int direction = std::stoi(list.front());list.pop_front();
    int vel = std::stoi(list.front());list.pop_front();
    int distance = std::stoi(list.front());list.pop_front();
    int timeout = std::stoi(list.front());list.pop_front();
    int posSensorNum = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserTrackingSearchStart_xyz(direction,vel,distance,timeout,posSensorNum);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光寻位-任意方向
 * @param [in] directionPoint 寻位输入的点的xyz左边
 * @param [in] vel 速度 单位%
 * @param [in] distance 最大寻位距离 单位mm
 * @param [in] timeout 寻位超时时间 单位ms
 * @param [in] posSensorNum 激光标定的工具坐标编号
 * @return 错误码
 */
std::string robot_command_thread::LaserTrackingSearchStart_point(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    DescTran direction;
    direction.x = std::stod(list.front().c_str());list.pop_front();
    direction.y = std::stod(list.front().c_str());list.pop_front();
    direction.z = std::stod(list.front().c_str());list.pop_front();
    int vel = std::stoi(list.front());list.pop_front();
    int distance = std::stoi(list.front());list.pop_front();
    int timeout = std::stoi(list.front());list.pop_front();
    int posSensorNum = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserTrackingSearchStart_point(direction,vel,distance,timeout,posSensorNum);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光寻位结束
 * @return 错误码
 */
std::string robot_command_thread::LaserTrackingSearchStop(std::string para){
    int res = _ptr_robot->LaserTrackingSearchStop();
    return std::string(std::to_string(res));
}

/**
 * @brief 激光寻位-任意方向
 * @param [in] directionPoint 寻位输入的点的xyz左边
 * @param [in] vel 速度 单位%
 * @param [in] distance 最大寻位距离 单位mm
 * @param [in] timeout 寻位超时时间 单位ms
 * @param [in] posSensorNum 激光标定的工具坐标编号
 * @return 错误码
 */
std::string robot_command_thread::LaserTrackingSensorConfig(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string ip = list.front();list.pop_front();
    int port = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserTrackingSensorConfig(ip,port);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光外设采样周期配置
 * @param [in] period 激光外设采样周期 单位ms
 * @return 错误码
 */
std::string robot_command_thread::LaserTrackingSensorSamplePeriod(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int period = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserTrackingSensorSamplePeriod(period);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光外设驱动加载
 * @param [in] type 激光外设驱动的协议类型 101-睿牛 102-创想 103-全视 104-同舟 105-奥太
 * @return 错误码
 */
std::string robot_command_thread::LoadPosSensorDriver(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LoadPosSensorDriver(type);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光外设驱动卸载
 * @return 错误码
 */
std::string robot_command_thread::UnLoadPosSensorDriver(std::string para){
    int res = _ptr_robot->UnLoadPosSensorDriver();
    return std::string(std::to_string(res));
}

/**
 * @brief 激光焊缝轨迹记录
 * @param [in] status 0-停止记录 1-实时跟踪  2-开始记录
 * @param [in] delayTime 延时时间 单位ms
 * @return 错误码
 */
std::string robot_command_thread::LaserSensorRecord1(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int status = std::stoi(list.front());list.pop_front();
    int delayTime = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserSensorRecord1(status,delayTime);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光焊缝轨迹复现
 * @param [in] delayTime 延时时间 单位ms
 * @param [in] speed 速度 单位%
 * @return 错误码
 */
std::string robot_command_thread::LaserSensorReplay(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int delayTime = std::stoi(list.front());list.pop_front();
    double speed = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserSensorReplay(delayTime,speed);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光跟踪复现
 * @return 错误码
 */
std::string robot_command_thread::MoveLTR(std::string para){
    int res = _ptr_robot->MoveLTR();
    return std::string(std::to_string(res));
}

/**
 * @brief 激光焊缝轨迹复现
 * @param [in] delayMode 模式 0-延时时间 1-延时距离
 * @param [in] delayTime 延时时间 单位ms
 * @param [in] delayDisExAxisNum 扩展轴编号
 * @param [in] delayDis 延时距离 单位mm
 * @param [in] sensitivePara 补偿灵敏系数
 * @param [in] trackMode 定点跟踪类型。0-扩展轴异步运动；1-机器人
 * @param [in] triggerMode 定点跟踪触发方式。0-跟踪时长；1-IO
 * @param [in] runTime 机器人定点跟踪时长(s)
 * @param [in] speed 速度 单位%
 * @return 错误码
 */
std::string robot_command_thread::LaserSensorRecordandReplay(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int delayMode = std::stoi(list.front());list.pop_front();
    int delayTime = std::stoi(list.front());list.pop_front();
    int delayDisExAxisNum = std::stoi(list.front());list.pop_front();
    double delayDis = std::stod(list.front());list.pop_front();
    double sensitivePara = std::stod(list.front());list.pop_front();
    int trackMode = std::stoi(list.front());list.pop_front();
    int triggerMode = std::stoi(list.front());list.pop_front();
    double runTime = std::stod(list.front());list.pop_front();
    double speed = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->LaserSensorRecordandReplay(delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, trackMode, triggerMode, runTime, speed);
    return std::string(std::to_string(res));
}

/**
 * @brief 运动到焊缝记录的起点
 * @param [in] moveType 0-moveJ 1-moveL
 * @param [in] ovl 速度 单位%
 * @return 错误码
 */
std::string robot_command_thread::MoveToLaserRecordStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int moveType = std::stoi(list.front());list.pop_front();
    double ovl = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->MoveToLaserRecordStart(moveType,ovl);
    return std::string(std::to_string(res));
}

/**
 * @brief 运动到焊缝记录的终点
 * @param [in] moveType 0-moveJ 1-moveL
 * @param [in] ovl 速度 单位%
 * @return 错误码
 */
std::string robot_command_thread::MoveToLaserRecordEnd(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int moveType = std::stoi(list.front());list.pop_front();
    double ovl = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->MoveToLaserRecordEnd(moveType,ovl);
    return std::string(std::to_string(res));
}

/**
 * @brief 运动到激光传感器寻位点
 * @param [in] moveFlag 运动类型：0-PTP；1-LIN
 * @param [in] ovl 速度缩放因子，0-100
 * @param [in] dataFlag 焊缝缓存数据选择：0-执行规划数据；1-执行记录数据
 * @param [in] plateType 板材类型：0-波纹板；1-瓦楞板；2-围栏板；3-油桶；4-波纹甲壳钢
 * @param [in] trackOffectType 激光传感器偏移类型：0-不偏移；1-基坐标系偏移；2-工具坐标系偏移；3-激光传感器原始数据偏移
 * @param [in] offset 偏移量
 * @return 错误码
 */
std::string robot_command_thread::MoveToLaserSeamPos(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int moveFlag = std::stoi(list.front());list.pop_front();
    double ovl = std::stod(list.front());list.pop_front();
    int dataFlag = std::stoi(list.front());list.pop_front();
    int plateType = std::stoi(list.front());list.pop_front();
    int trackOffectType = std::stoi(list.front());list.pop_front();
    DescPose offset;
    _fillDescPose(list,offset);
    
    int res = _ptr_robot->MoveToLaserSeamPos(moveFlag,ovl,dataFlag,plateType,trackOffectType,offset);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取激光传感器寻位点坐标信息
 * @param [in] trackOffectType 激光传感器偏移类型：0-不偏移；1-基坐标系偏移；2-工具坐标系偏移；3-激光传感器原始数据偏移
 * @param [in] offset 偏移量
 * @param [out] jPos 关节位置[°]
 * @param [out] descPos 笛卡尔位置[mm]
 * @param [out] tool 工具坐标系
 * @param [out] user 工件坐标系
 * @param [out] exaxis 扩展轴位置[mm]
 * @return 错误码
 */
std::string robot_command_thread::GetLaserSeamPos(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int trackOffectType = std::stoi(list.front());list.pop_front();
    DescPose offset;
    _fillDescPose(list,offset);

    JointPos jPos;
    DescPose descPos;
    int tool;
    int user;
    ExaxisPos exaxis;
    
    int res = _ptr_robot->GetLaserSeamPos(trackOffectType,offset,jPos,descPos,tool,user,exaxis);
    return std::string(std::to_string(res) + "," + std::to_string(jPos.jPos[0]) + "," +\
            std::to_string(jPos.jPos[1]) + "," + std::to_string(jPos.jPos[2]) + "," +\
            std::to_string(jPos.jPos[3]) + "," + std::to_string(jPos.jPos[4]) + "," +\
            std::to_string(jPos.jPos[5]) + "," + std::to_string(descPos.tran.x) + "," + \
            std::to_string(descPos.tran.y) + "," + std::to_string(descPos.tran.z) + "," + \
            std::to_string(descPos.rpy.rx) + "," + std::to_string(descPos.rpy.ry) + "," + \
            std::to_string(descPos.rpy.rz) + "," + std::to_string(tool) + "," +\
            std::to_string(user) + "," + std::to_string(exaxis.ePos[0]) + "," +\
            std::to_string(exaxis.ePos[1]) + "," + std::to_string(exaxis.ePos[2]) + "," +\
            std::to_string(exaxis.ePos[3]));
}

/**
 * @brief 摆动渐变开始
 * @param [in] weaveChangeFlag 1-变摆动参数；2-变摆动参数+焊接速度
 * @param [in] weaveNum 摆动编号 
 * @param [in] velStart 焊接开始速度，(cm/min)
 * @param [in] velEnd 焊接结束速度，(cm/min)
 * @return 错误码
 */
std::string robot_command_thread::WeaveChangeStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int weaveChangeFlag = std::stoi(list.front());list.pop_front();
    int weaveNum = std::stoi(list.front());list.pop_front();
    double velStart = std::stod(list.front());list.pop_front();
    double velEnd = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->WeaveChangeStart(weaveChangeFlag,weaveNum,velStart,velEnd);
    return std::string(std::to_string(res));
}

/**
 * @brief 摆动渐变结束
 * @return 错误码
 */
std::string robot_command_thread::WeaveChangeEnd(std::string para){
    int res = _ptr_robot->WeaveChangeEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief 轨迹预处理(轨迹前瞻)
 * @param [in] name  轨迹文件名
 * @param [in] mode 采样模式，0-不进行采样；1-等数据间隔采样；2-等误差限制采样
 * @param [in] errorLim 误差限制，使用直线拟合生效
 * @param [in] type 平滑方式，0-贝塞尔平滑
 * @param [in] precision 平滑精度，使用贝塞尔平滑时生效
 * @param [in] vamx 设定的最大速度，mm/s
 * @param [in] amax 设定的最大加速度，mm/s2
 * @param [in] jmax 设定的最大加加速度，mm/s3
 * @param [in] flag 匀速前瞻开启开关 0-不开启；1-开启
 * @return 错误码
 */
std::string robot_command_thread::LoadTrajectoryLA(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    char name[30];
    list.front().copy(name,list.front().size());list.pop_front();
    int mode = std::stoi(list.front());list.pop_front();
    double errorLim = std::stod(list.front());list.pop_front();
    int type = std::stoi(list.front());list.pop_front();
    double precision = std::stod(list.front());list.pop_front();
    double vamx = std::stod(list.front());list.pop_front();
    double amax = std::stod(list.front());list.pop_front();
    double jmax = std::stod(list.front());list.pop_front();
    int flag = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->LoadTrajectoryLA(name, mode, errorLim, type, precision, vamx, amax, jmax, flag);
    return std::string(std::to_string(res));
}

/**
 * @brief 轨迹复现(轨迹前瞻)
 * @return 错误码
 */
std::string robot_command_thread::MoveTrajectoryLA(std::string para){
    int res = _ptr_robot->MoveTrajectoryLA();
    return std::string(std::to_string(res));
}

/**
 * @brief 自定义碰撞检测阈值功能开始，设置关节端和TCP端的碰撞检测阈值
 * @param [in] flag 1-仅关节检测开启；2-仅TCP检测开启；3-关节和TCP检测同时开启
 * @param [in] jointDetectionThreshould 关节碰撞检测阈值 j1-j6
 * @param [in] tcpDetectionThreshould TCP碰撞检测阈值，xyzabc
 * @param [in] block 0-非阻塞；1-阻塞
 * @return 错误码
 */
std::string robot_command_thread::CustomCollisionDetectionStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int flag = std::stoi(list.front());list.pop_front();
    double jointDetectionThreshould[6];
    jointDetectionThreshould[0] = std::stod(list.front());list.pop_front();
    jointDetectionThreshould[1] = std::stod(list.front());list.pop_front();
    jointDetectionThreshould[2] = std::stod(list.front());list.pop_front();
    jointDetectionThreshould[3] = std::stod(list.front());list.pop_front();
    jointDetectionThreshould[4] = std::stod(list.front());list.pop_front();
    jointDetectionThreshould[5] = std::stod(list.front());list.pop_front();
    double tcpDetectionThreshould[6];
    tcpDetectionThreshould[0] = std::stod(list.front());list.pop_front();
    tcpDetectionThreshould[1] = std::stod(list.front());list.pop_front();
    tcpDetectionThreshould[2] = std::stod(list.front());list.pop_front();
    tcpDetectionThreshould[3] = std::stod(list.front());list.pop_front();
    tcpDetectionThreshould[4] = std::stod(list.front());list.pop_front();
    tcpDetectionThreshould[5] = std::stod(list.front());list.pop_front();
    int block = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->CustomCollisionDetectionStart(flag, jointDetectionThreshould, tcpDetectionThreshould, block);
    return std::string(std::to_string(res));
}

/**
 * @brief 自定义碰撞检测阈值功能关闭
 * @return 错误码
 */
std::string robot_command_thread::CustomCollisionDetectionEnd(std::string para){
    int res = _ptr_robot->CustomCollisionDetectionEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief 加速度平滑开启
 * @param [in] saveFlag 是否断电保存
 * @return 错误码
 */
std::string robot_command_thread::AccSmoothStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int saveFlag = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->AccSmoothStart(saveFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief 加速度平滑关闭
 * @param [in] saveFlag 是否断电保存
 * @return 错误码
 */
std::string robot_command_thread::AccSmoothEnd(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int saveFlag = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->AccSmoothEnd(saveFlag);
    return std::string(std::to_string(res));
}

/**
 * @brief 控制器日志下载
 * @param [in] savePath 保存文件路径"D://zDown/"
 * @return 错误码
 */
std::string robot_command_thread::RbLogDownload(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string savePath = list.front();list.pop_front();
    
    int res = _ptr_robot->RbLogDownload(savePath);
    return std::string(std::to_string(res));
}

/**
 * @brief 所有数据源下载
 * @param [in] savePath 保存文件路径"D://zDown/"
 * @return 错误码
 */
std::string robot_command_thread::AllDataSourceDownload(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string savePath = list.front();list.pop_front();
    
    int res = _ptr_robot->AllDataSourceDownload(savePath);
    return std::string(std::to_string(res));
}

/**
 * @brief 数据备份包下载
 * @param [in] savePath 保存文件路径"D://zDown/"
 * @return 错误码
 */
std::string robot_command_thread::DataPackageDownload(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string savePath = list.front();list.pop_front();
    
    int res = _ptr_robot->DataPackageDownload(savePath);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取控制箱SN码
 * @param [out] SNCode 控制箱SN码
 * @return 错误码
 */
std::string robot_command_thread::GetRobotSN(std::string para){
    std::string SNCode;
    
    int res = _ptr_robot->GetRobotSN(SNCode);
    return std::string(std::to_string(res) + "," + std::string(SNCode));
}

/**
 * @brief 关闭机器人操作系统
 * @return 错误码
 */
std::string robot_command_thread::ShutDownRobotOS(std::string para){
    int res = _ptr_robot->ShutDownRobotOS();
    return std::string(std::to_string(res));
}

/**
 * @brief 传送带通讯输入检测
 * @param [in] timeout 等待超时时间ms
 * @return 错误码
 */
std::string robot_command_thread::ConveyorComDetect(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int timeout = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->ConveyorComDetect(timeout);
    return std::string(std::to_string(res));
}

/**
 * @brief 传送带通讯输入检测触发
 * @return 错误码
 */
std::string robot_command_thread::ConveyorComDetectTrigger(std::string para){
    int res = _ptr_robot->ConveyorComDetectTrigger();
    return std::string(std::to_string(res));
}

/**
 * @brief 电弧跟踪焊机电流反馈AI通道选择
 * @param [in]  channel 通道；0-扩展AI0；1-扩展AI1；2-扩展AI2；3-扩展AI3；4-控制箱AI0；5-控制箱AI1
 * @return 错误码
 */
std::string robot_command_thread::ArcWeldTraceAIChannelCurrent(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int channel = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->ArcWeldTraceAIChannelCurrent(channel);
    return std::string(std::to_string(res));
}

/**
 * @brief 电弧跟踪焊机电压反馈AI通道选择
 * @param [in]  channel 通道；0-扩展AI0；1-扩展AI1；2-扩展AI2；3-扩展AI3；4-控制箱AI0；5-控制箱AI1
 * @return 错误码
 */
std::string robot_command_thread::ArcWeldTraceAIChannelVoltage(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int channel = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->ArcWeldTraceAIChannelVoltage(channel);
    return std::string(std::to_string(res));
}

/**
 * @brief 电弧跟踪焊机电流反馈转换参数
 * @param [in] AILow AI通道下限，默认值0V，范围[0-10V]
 * @param [in] AIHigh AI通道上限，默认值10V，范围[0-10V]
 * @param [in] currentLow AI通道下限对应焊机电流值，默认值0V，范围[0-200V]
 * @param [in] currentHigh AI通道上限对应焊机电流值，默认值100V，范围[0-200V]
 * @return 错误码
 */
std::string robot_command_thread::ArcWeldTraceCurrentPara(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    float AILow = std::stod(list.front());list.pop_front();
    float AIHigh = std::stod(list.front());list.pop_front();
    float currentLow = std::stod(list.front());list.pop_front();
    float currentHigh = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->ArcWeldTraceCurrentPara(AILow, AIHigh, currentLow, currentHigh);
    return std::string(std::to_string(res));
}

/**
 * @brief 电弧跟踪焊机电压反馈转换参数
 * @param [in] AILow AI通道下限，默认值0V，范围[0-10V]
 * @param [in] AIHigh AI通道上限，默认值10V，范围[0-10V]
 * @param [in] voltageLow AI通道下限对应焊机电压值，默认值0V，范围[0-200V]
 * @param [in] voltageHigh AI通道上限对应焊机电压值，默认值100V，范围[0-200V]
 * @return 错误码
 */
std::string robot_command_thread::ArcWeldTraceVoltagePara(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    float AILow = std::stod(list.front());list.pop_front();
    float AIHigh = std::stod(list.front());list.pop_front();
    float voltageLow = std::stod(list.front());list.pop_front();
    float voltageHigh = std::stod(list.front());list.pop_front();
    
    int res = _ptr_robot->ArcWeldTraceVoltagePara(AILow, AIHigh, voltageLow, voltageHigh);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焊接电压渐变开始
 * @param [in] IOType 控制类型；0-控制箱IO；1-数字通信协议(UDP);2-数字通信协议(ModbusTCP)
 * @param [in] voltageStart 起始焊接电压(V)
 * @param [in] voltageEnd 终止焊接电压(V)
 * @param [in] AOIndex 控制箱AO端口号(0-1)
 * @param [in] blend 是否平滑 0-不平滑；1-平滑
 * @return 错误码
 */
std::string robot_command_thread::WeldingSetVoltageGradualChangeStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int IOType = std::stoi(list.front());list.pop_front();
    double voltageStart = std::stod(list.front());list.pop_front();
    double voltageEnd = std::stod(list.front());list.pop_front();
    int AOIndex = std::stoi(list.front());list.pop_front();
    int blend = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->WeldingSetVoltageGradualChangeStart(IOType, voltageStart, voltageEnd, AOIndex,blend);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焊接电压渐变结束
 * @return 错误码
 */
std::string robot_command_thread::WeldingSetVoltageGradualChangeEnd(std::string para){
    int res = _ptr_robot->WeldingSetVoltageGradualChangeEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焊接电流渐变开始
 * @param [in] IOType 控制类型；0-控制箱IO；1-数字通信协议(UDP);2-数字通信协议(ModbusTCP)
 * @param [in] voltageStart 起始焊接电流(A)
 * @param [in] voltageEnd 终止焊接电流(A)
 * @param [in] AOIndex 控制箱AO端口号(0-1)
 * @param [in] blend 是否平滑 0-不平滑；1-平滑
 * @return 错误码
 */
std::string robot_command_thread::WeldingSetCurrentGradualChangeStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int IOType = std::stoi(list.front());list.pop_front();
    double currentStart = std::stod(list.front());list.pop_front();
    double currentEnd = std::stod(list.front());list.pop_front();
    int AOIndex = std::stoi(list.front());list.pop_front();
    int blend = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->WeldingSetCurrentGradualChangeStart(IOType, currentStart, currentEnd, AOIndex,blend);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焊接电流渐变结束
 * @return 错误码
 */
std::string robot_command_thread::WeldingSetCurrentGradualChangeEnd(std::string para){
    int res = _ptr_robot->WeldingSetCurrentGradualChangeEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief 获取SmartTool按钮状态
 * @param [out] state SmartTool手柄按钮状态;(bit0:0-通信正常；1-通信掉线；bit1-撤销操作；bit2-清空程序；
                      bit3-A键；bit4-B键；bit5-C键；bit6-D键；bit7-E键；bit8-IO键；bit9-手自动；bit10开始)
 * @return 错误码
 */
std::string robot_command_thread::GetSmarttoolBtnState(std::string para){
    int state;
    
    int res = _ptr_robot->GetSmarttoolBtnState(state);
    return std::string(std::to_string(res) + "," + std::to_string(state));
}

/**
 * @brief 设置宽电压控制箱温度及风扇电流监控参数
 * @param [in] enable 0-不使能监测；1-使能监测
 * @param [in] period 监测周期(s),范围1-100
 * @return 错误码
 */
std::string robot_command_thread::SetWideBoxTempFanMonitorParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int enable = std::stoi(list.front());list.pop_front();
    int period = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->SetWideBoxTempFanMonitorParam(enable,period);
    return std::string(std::to_string(res));
}

/**
 * @brief 获取宽电压控制箱温度及风扇电流监控参数
 * @param [out] enable 0-不使能监测；1-使能监测
 * @param [out] period 监测周期(s),范围1-100
 * @return 错误码
 */
std::string robot_command_thread::GetWideBoxTempFanMonitorParam(std::string para){
    int enable;
    int period;
    
    int res = _ptr_robot->GetWideBoxTempFanMonitorParam(enable,period);
    return std::string(std::to_string(res) + "," + std::to_string(enable) + "," +\
            std::to_string(period));
}

/**
 * @brief 设置焦点标定点
 * @param [in] pointNum 焦点标定点编号 1-8
 * @param [in] point 标定点坐标
 * @return 错误码
 */
std::string robot_command_thread::SetFocusCalibPoint(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int pointNum = std::stoi(list.front());list.pop_front();
    DescPose point;
    _fillDescPose(list,point);
    
    int res = _ptr_robot->SetFocusCalibPoint(pointNum,point);
    return std::string(std::to_string(res));
}

/**
 * @brief 计算焦点标定结果
 * @param [in] pointNum 标定点个数
 * @param [out] resultPos 标定结果XYZ
 * @param [out] accuracy 标定精度误差
 * @return 错误码
 */
std::string robot_command_thread::ComputeFocusCalib(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int pointNum = std::stoi(list.front());list.pop_front();

    DescTran resultPos;
    float accuracy ;

    int res = _ptr_robot->ComputeFocusCalib(pointNum,resultPos,accuracy);
    return std::string(std::to_string(res) + "," + std::to_string(resultPos.x) + "," +\
                std::to_string(resultPos.y) + "," + std::to_string(resultPos.z) + "," +\
                std::to_string(accuracy));
}

/**
 * @brief 开启焦点跟随
 * @param [in] kp 比例参数，默认50.0
 * @param [in] kpredict 前馈参数，默认19.0
 * @param [in] aMax 最大角加速度限制，默认1440°/s^2
 * @param [in] vMax 最大角速度限制，默认180°/s
 * @param [in] type 锁定X轴指向(0-参考输入矢量；1-水平；2-垂直)
 * @return 错误码
 */
std::string robot_command_thread::FocusStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double kp = std::stod(list.front());list.pop_front();
    double kpredict = std::stod(list.front());list.pop_front();
    double aMax = std::stod(list.front());list.pop_front();
    double vMax = std::stod(list.front());list.pop_front();
    int type = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FocusStart(kp,kpredict,aMax,vMax,type);
    return std::string(std::to_string(res));
}

/**
 * @brief 停止焦点跟随
 * @return 错误码
 */
std::string robot_command_thread::FocusEnd(std::string para){
    int res = _ptr_robot->FocusEnd();
    return std::string(std::to_string(res));
}

/**
 * @brief 设置焦点坐标
 * @param [in] pos 焦点坐标XYZ
 * @return 错误码
 */
std::string robot_command_thread::SetFocusPosition(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    DescTran pos;
    pos.x = std::stod(list.front().c_str());list.pop_front();
    pos.y = std::stod(list.front().c_str());list.pop_front();
    pos.z = std::stod(list.front().c_str());list.pop_front();

    int res = _ptr_robot->SetFocusPosition(pos);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置编码器升级(暂未开放)
 * @param [in] path 本地升级包全路径(D://zUP/XXXXX.bin)
 * @return 错误码
 */
std::string robot_command_thread::SetEncoderUpgrade(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string path = list.front();list.pop_front();

    int res = _ptr_robot->SetEncoderUpgrade(path);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置关节固件升级
 * @param [in] type 升级文件类型；1-升级固件(使用前需要使机器人进入boot模式)；2-升级从站配置文件(使用前需要去使能机器人)
 * @param [in] path 本地升级包全路径(D://zUP/XXXXX.bin)
 * @return 错误码
 */
std::string robot_command_thread::SetJointFirmwareUpgrade(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front().c_str());list.pop_front();
    std::string path = list.front();list.pop_front();

    int res = _ptr_robot->SetJointFirmwareUpgrade(type,path);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置控制箱固件升级
 * @param [in] type 升级文件类型；1-升级固件(使用前需要使机器人进入boot模式)；2-升级从站配置文件(使用前需要去使能机器人)
 * @param [in] path 本地升级包全路径(D://zUP/XXXXX.bin)
 * @return 错误码
 */
std::string robot_command_thread::SetCtrlFirmwareUpgrade(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front().c_str());list.pop_front();
    std::string path = list.front();list.pop_front();

    int res = _ptr_robot->SetCtrlFirmwareUpgrade(type,path);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置末端固件升级
 * @param [in] type 升级文件类型；1-升级固件(使用前需要使机器人进入boot模式)；2-升级从站配置文件(使用前需要去使能机器人)
 * @param [in] path 本地升级包全路径(D://zUP/XXXXX.bin)
 * @return 错误码
 */
std::string robot_command_thread::SetEndFirmwareUpgrade(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front().c_str());list.pop_front();
    std::string path = list.front();list.pop_front();

    int res = _ptr_robot->SetEndFirmwareUpgrade(type,path);
    return std::string(std::to_string(res));
}

/**
 * @brief 关节全参数配置文件升级(使用前需要去使能机器人)
 * @param [in] path 本地升级包全路径(D://zUP/XXXXX.bin)
 * @return 错误码
 */
std::string robot_command_thread::JointAllParamUpgrade(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string path = list.front();list.pop_front();

    int res = _ptr_robot->JointAllParamUpgrade(path);
    return std::string(std::to_string(res));
}

/**
 * @brief 设置机器人型号(使用前需要去使能机器人)
 * @param [in] type 机器人型号
 * @return 错误码
 */
std::string robot_command_thread::SetRobotType(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int type = std::stoi(list.front().c_str());list.pop_front();

    int res = _ptr_robot->SetRobotType(type);
    return std::string(std::to_string(res));
}

/**
 * @brief 激光传感器记录点
 * @param [in] coordID 激光传感器坐标系
 * @param [out] desc 激光传感器识别点笛卡尔位置
 * @param [out] joint 激光传感器识别点关节位置
 * @param [out] exaxis 激光传感器识别点扩展轴位置
 * @return 错误码
 */
std::string robot_command_thread::LaserRecordPoint(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int coordID = std::stoi(list.front());list.pop_front();

    DescPose desc;
    JointPos joint;
    ExaxisPos exaxis;

    int res = _ptr_robot->LaserRecordPoint(coordID,desc,joint,exaxis);
    return std::string(std::to_string(res) + ","  + std::to_string(desc.tran.x) + "," + \
            std::to_string(desc.tran.y) + "," + std::to_string(desc.tran.z) + "," + \
            std::to_string(desc.rpy.rx) + "," + std::to_string(desc.rpy.ry) + "," + \
            std::to_string(desc.rpy.rz) + "," + std::to_string(joint.jPos[0]) + "," +\
            std::to_string(joint.jPos[1]) + "," + std::to_string(joint.jPos[2]) + "," +\
            std::to_string(joint.jPos[3]) + "," + std::to_string(joint.jPos[4]) + "," +\
            std::to_string(joint.jPos[5]) + "," + std::to_string(exaxis.ePos[0]) + "," +\
            std::to_string(exaxis.ePos[1]) + "," + std::to_string(exaxis.ePos[2]) + "," +\
            std::to_string(exaxis.ePos[3]));            
}

/**
 * @brief 设置扩展轴与机器人同步运动策略
 * @param [in] strategy 策略；0-以机器人为主；1-扩展轴与机器人同步
 * @return 错误码
 */
std::string robot_command_thread::SetExAxisRobotPlan(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int strategy = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetExAxisRobotPlan(strategy);
    return std::string(std::to_string(res));
}

/**
* @brief  设置与机器人通讯重连参数
* @param  [in] enable  网络故障时使能重连 true-使能 false-不使能
* @param  [in] reconnectTime 重连时间，单位ms
* @param  [in] period 重连周期，单位ms
* @return  错误码
*/
std::string robot_command_thread::SetReConnectParam(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    bool enable = (std::stoi(list.front()) != 0);list.pop_front();
    int reconnectTime = std::stoi(list.front());list.pop_front();
    int period = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetReConnectParam(enable,reconnectTime,period);
    return std::string(std::to_string(res));
}

/**
 * @brief  获取从站板卡参数
 * @param  [out] type  0-Ethercat，1-CClink, 3-Ethercat, 4-EIP
 * @param  [out] version  协议版本
 * @param  [out] connState  0-未连接 1-已连接
 * @return  错误码
 */
std::string robot_command_thread::GetFieldBusConfig(std::string para){

    uint8_t type;
    uint8_t version;
    uint8_t connState;

    int res = _ptr_robot->GetFieldBusConfig(&type,&version,&connState);
    return std::string(std::to_string(res) + ","  + std::to_string(type) + "," + \
            std::to_string(version) + "," + std::to_string(connState));            
}

/**
 * @brief  写入从站DO
 * @param  [in] DOIndex  DO编号
 * @param  [in] wirteNum  写入的数量
 * @param  [in] status[8] 写入的数值，最多写8个
 * @return  错误码
 */
std::string robot_command_thread::FieldBusSlaveWriteDO(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t DOIndex = std::stoi(list.front());list.pop_front();
    uint8_t wirteNum = std::stoi(list.front());list.pop_front();

    uint8_t status[8]; 
    status[0] = std::stoi(list.front());list.pop_front();
    status[1] = std::stoi(list.front());list.pop_front();
    status[2] = std::stoi(list.front());list.pop_front();
    status[3] = std::stoi(list.front());list.pop_front();
    status[4] = std::stoi(list.front());list.pop_front();
    status[5] = std::stoi(list.front());list.pop_front();
    status[6] = std::stoi(list.front());list.pop_front();
    status[7] = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FieldBusSlaveWriteDO(DOIndex,wirteNum,status);
    return std::string(std::to_string(res));            
}

/**
 * @brief  写入从站AO
 * @param  [in] AOIndex  AO编号
 * @param  [in] wirteNum  写入的数量
 * @param  [in] status[8] 写入的数值，最多写8个
 * @return  错误码
 */
std::string robot_command_thread::FieldBusSlaveWriteAO(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t AOIndex = std::stoi(list.front());list.pop_front();
    uint8_t wirteNum = std::stoi(list.front());list.pop_front();

    int status[8]; 
    status[0] = std::stoi(list.front());list.pop_front();
    status[1] = std::stoi(list.front());list.pop_front();
    status[2] = std::stoi(list.front());list.pop_front();
    status[3] = std::stoi(list.front());list.pop_front();
    status[4] = std::stoi(list.front());list.pop_front();
    status[5] = std::stoi(list.front());list.pop_front();
    status[6] = std::stoi(list.front());list.pop_front();
    status[7] = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FieldBusSlaveWriteAO(AOIndex,wirteNum,status);
    return std::string(std::to_string(res));            
}

/**
 * @brief  读取从站DI
 * @param  [in] DOIndex  DI编号
 * @param  [in] readeNum  读取的数量
 * @param  [out] status[8] 读取到的数值，最多读8个
 * @return  错误码
 */
std::string robot_command_thread::FieldBusSlaveReadDI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t DOIndex = std::stoi(list.front());list.pop_front();
    uint8_t readNum = std::stoi(list.front());list.pop_front();

    uint8_t status[8]; 

    int res = _ptr_robot->FieldBusSlaveReadDI(DOIndex,readNum,status);
    return std::string(std::to_string(res) + ","  + std::to_string(status[0]) + "," +\
        std::to_string(status[1]) + "," + std::to_string(status[2]) + "," +\
        std::to_string(status[3]) + "," + std::to_string(status[4]) + "," +\
        std::to_string(status[5]) + "," + std::to_string(status[6]) + "," +\
        std::to_string(status[7]));             
}

/**
 * @brief  读取从站AI
 * @param  [in] AOIndex  AI编号
 * @param  [in] readeNum  读取的数量
 * @param  [out] status[8] 读取到的数值，最多读8个
 * @return  错误码
 */
std::string robot_command_thread::FieldBusSlaveReadAI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t AIIndex = std::stoi(list.front());list.pop_front();
    uint8_t readNum = std::stoi(list.front());list.pop_front();

    int status[8]; 

    int res = _ptr_robot->FieldBusSlaveReadAI(AIIndex,readNum,status);
    return std::string(std::to_string(res) + ","  + std::to_string(status[0]) + "," +\
        std::to_string(status[1]) + "," + std::to_string(status[2]) + "," +\
        std::to_string(status[3]) + "," + std::to_string(status[4]) + "," +\
        std::to_string(status[5]) + "," + std::to_string(status[6]) + "," +\
        std::to_string(status[7]));             
}

/**
 * @brief 等待扩展DI输入
 * @param [in] DIIndex DI编号
 * @param [in] status 0-低电平；1-高电平
 * @param [in] waitMs 最大等待时间(ms)
 * @return 错误码
 */
std::string robot_command_thread::FieldBusSlaveWaitDI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t DIIndex = std::stoi(list.front());list.pop_front();
    bool status = (std::stoi(list.front()) != 0);list.pop_front();
    int waitMs = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FieldBusSlaveWaitDI(DIIndex,status,waitMs);
    return std::string(std::to_string(res));            
}

/**
 * @brief 等待扩展AI输入
 * @param [in] AIIndex AI编号
 * @param [in] waitType 0-大于；1-小于
 * @param [in] value AI值
 * @param [in] waitMs 最大等待时间(ms)
 * @return 错误码
 */
std::string robot_command_thread::FieldBusSlaveWaitAI(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t AIIndex = std::stoi(list.front());list.pop_front();
    uint8_t waitType = std::stoi(list.front());list.pop_front();
    double value = std::stod(list.front());list.pop_front();
    int waitMs = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->FieldBusSlaveWaitAI(AIIndex,waitType,value,waitMs);
    return std::string(std::to_string(res));            
}

/**
 * @brief 控制阵列式吸盘
 * @param [in] slaveID 从站号
 * @param [in] len 长度
 * @param [in] ctrlValue 控制值
 * @return 错误码
 */
std::string robot_command_thread::SetSuckerCtrl(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t slaveID = std::stoi(list.front());list.pop_front();
    uint8_t len = std::stoi(list.front());list.pop_front();

    uint8_t ctrlValue[20];
    ctrlValue[0] = std::stoi(list.front());list.pop_front();
    ctrlValue[1] = std::stoi(list.front());list.pop_front();
    ctrlValue[2] = std::stoi(list.front());list.pop_front();
    ctrlValue[3] = std::stoi(list.front());list.pop_front();
    ctrlValue[4] = std::stoi(list.front());list.pop_front();
    ctrlValue[5] = std::stoi(list.front());list.pop_front();
    ctrlValue[6] = std::stoi(list.front());list.pop_front();
    ctrlValue[7] = std::stoi(list.front());list.pop_front();
    ctrlValue[8] = std::stoi(list.front());list.pop_front();
    ctrlValue[9] = std::stoi(list.front());list.pop_front();
    ctrlValue[10] = std::stoi(list.front());list.pop_front();
    ctrlValue[11] = std::stoi(list.front());list.pop_front();
    ctrlValue[12] = std::stoi(list.front());list.pop_front();
    ctrlValue[13] = std::stoi(list.front());list.pop_front();
    ctrlValue[14] = std::stoi(list.front());list.pop_front();
    ctrlValue[15] = std::stoi(list.front());list.pop_front();
    ctrlValue[16] = std::stoi(list.front());list.pop_front();
    ctrlValue[17] = std::stoi(list.front());list.pop_front();
    ctrlValue[18] = std::stoi(list.front());list.pop_front();
    ctrlValue[19] = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SetSuckerCtrl(slaveID,len,ctrlValue);
    return std::string(std::to_string(res));            
}

/**
 * @brief 获取阵列式吸盘状态
 * @param [in] slaveID 从站号
 * @param [out] state 吸附状态 0-释放物体 1-检测到工件吸附成功 2-没有吸附到物体 3-物体脱离
 * @param [out] pressValue 当前真空度 单位kpa 
 * @param [out] error 吸盘当前的错误码
 * @return 错误码
 */
std::string robot_command_thread::GetSuckerState(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t slaveID = std::stoi(list.front());list.pop_front();

    uint8_t state;
    int pressValue;
    int error;

    int res = _ptr_robot->GetSuckerState(slaveID,&state,&pressValue,&error);
    return std::string(std::to_string(res) + ","  + std::to_string(state) + "," +\
        std::to_string(pressValue) + "," + std::to_string(error));          
}

/**
 * @brief 等待吸盘状态
 * @param [in] slaveID 从站号
 * @param [in] state 吸附状态 0-释放物体 1-检测到工件吸附成功 2-没有吸附到物体 3-物体脱离
 * @param [in] ms 等待最大时间
 * @return 错误码
 */
std::string robot_command_thread::WaitSuckerState(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t slaveID = std::stoi(list.front());list.pop_front();
    uint8_t state = std::stoi(list.front());list.pop_front();
    int ms = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->WaitSuckerState(slaveID,state,ms);
    return std::string(std::to_string(res));            
}

/**
 * @brief 上传Lua文件
 * @param [in] filePath 本地lua文件路径名
 * @return 错误码
 */
std::string robot_command_thread::OpenLuaUpload(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string filePath = list.front();list.pop_front();

    int res = _ptr_robot->OpenLuaUpload(filePath);
    return std::string(std::to_string(res));            
}


/**
* @brief 阻抗启停控制
* @param [in] status 0：关闭；1-开启
* @param [in] workSpace 0-关节空间；1-迪卡尔空间
* @param [in] forceThreshold 触发力阈值(N)
* @param [in] m 质量参数
* @param [in] b 阻尼参数
* @param [in] k 刚度参数
* @param [in] maxV 最大线速度(mm/s)
* @param [in] maxVA 最大线加速度(mm/s2)
* @param [in] maxW 最大角速度(°/s)
* @param [in] maxWA 最大角加速度(°/s2)
* @return 错误码
*/
std::string robot_command_thread::ImpedanceControlStartStop(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int status = std::stoi(list.front());list.pop_front();
    int workSpace = std::stoi(list.front());list.pop_front();
    
    double forceThreshold[6];
    forceThreshold[0] = std::stod(list.front());list.pop_front();
    forceThreshold[1] = std::stod(list.front());list.pop_front();
    forceThreshold[2] = std::stod(list.front());list.pop_front();
    forceThreshold[3] = std::stod(list.front());list.pop_front();
    forceThreshold[4] = std::stod(list.front());list.pop_front();
    forceThreshold[5] = std::stod(list.front());list.pop_front();
    double m[6];
    m[0] = std::stod(list.front());list.pop_front();
    m[1] = std::stod(list.front());list.pop_front();
    m[2] = std::stod(list.front());list.pop_front();
    m[3] = std::stod(list.front());list.pop_front();
    m[4] = std::stod(list.front());list.pop_front();
    m[5] = std::stod(list.front());list.pop_front();
    double b[6];
    b[0] = std::stod(list.front());list.pop_front();
    b[1] = std::stod(list.front());list.pop_front();
    b[2] = std::stod(list.front());list.pop_front();
    b[3] = std::stod(list.front());list.pop_front();
    b[4] = std::stod(list.front());list.pop_front();
    b[5] = std::stod(list.front());list.pop_front();
    double k[6];
    k[0] = std::stod(list.front());list.pop_front();
    k[1] = std::stod(list.front());list.pop_front();
    k[2] = std::stod(list.front());list.pop_front();
    k[3] = std::stod(list.front());list.pop_front();
    k[4] = std::stod(list.front());list.pop_front();
    k[5] = std::stod(list.front());list.pop_front();
    
    double maxV = std::stod(list.front());list.pop_front();
    double maxVA = std::stod(list.front());list.pop_front();
    double maxW = std::stod(list.front());list.pop_front();
    double maxWA = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->ImpedanceControlStartStop(status,workSpace,forceThreshold,m,b,k,maxV,maxVA,maxW,maxWA);
    return std::string(std::to_string(res));            
}

/**
 * @brief 设置拖动开启前负载力检测
 * @param [in] flag 0-关闭；1-开启
 * @return 错误码
 */
std::string robot_command_thread::SetTorqueDetectionSwitch(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    uint8_t flag = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->SetTorqueDetectionSwitch(flag);
    return std::string(std::to_string(res));            
}

/**
* @brief 根据编号获取工具坐标系
* @param [in] id 工具坐标系编号
* @param [out] coord 坐标系数值
* @return 错误码
*/
std::string robot_command_thread::GetToolCoordWithID(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    DescPose coord;

    int res = _ptr_robot->GetToolCoordWithID(id,coord);
    return std::string(std::to_string(res) + ","  + std::to_string(coord.tran.x) + "," + \
            std::to_string(coord.tran.y) + "," + std::to_string(coord.tran.z) + "," + \
            std::to_string(coord.rpy.rx) + "," + std::to_string(coord.rpy.ry) + "," + \
            std::to_string(coord.rpy.rz));            
}  

/**
* @brief 根据编号获取工件坐标系
* @param [in] id 工件坐标系编号
* @param [out] coord 坐标系数值
* @return 错误码
*/
std::string robot_command_thread::GetWObjCoordWithID(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    DescPose coord;

    int res = _ptr_robot->GetWObjCoordWithID(id,coord);
    return std::string(std::to_string(res) + ","  + std::to_string(coord.tran.x) + "," + \
            std::to_string(coord.tran.y) + "," + std::to_string(coord.tran.z) + "," + \
            std::to_string(coord.rpy.rx) + "," + std::to_string(coord.rpy.ry) + "," + \
            std::to_string(coord.rpy.rz));            
}  

/**
* @brief 根据编号获取外部工具坐标系
* @param [in] id 外部工具坐标系编号
* @param [out] coord 坐标系数值
* @return 错误码
*/
std::string robot_command_thread::GetExToolCoordWithID(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    DescPose coord;

    int res = _ptr_robot->GetExToolCoordWithID(id,coord);
    return std::string(std::to_string(res) + ","  + std::to_string(coord.tran.x) + "," + \
            std::to_string(coord.tran.y) + "," + std::to_string(coord.tran.z) + "," + \
            std::to_string(coord.rpy.rx) + "," + std::to_string(coord.rpy.ry) + "," + \
            std::to_string(coord.rpy.rz));            
} 

/**
* @brief 根据编号获取扩展轴坐标系
* @param [in] id 外部工具坐标系编号
* @param [out] coord 坐标系数值
* @return 错误码
*/
std::string robot_command_thread::GetExAxisCoordWithID(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    DescPose coord;

    int res = _ptr_robot->GetExAxisCoordWithID(id,coord);
    return std::string(std::to_string(res) + ","  + std::to_string(coord.tran.x) + "," + \
            std::to_string(coord.tran.y) + "," + std::to_string(coord.tran.z) + "," + \
            std::to_string(coord.rpy.rx) + "," + std::to_string(coord.rpy.ry) + "," + \
            std::to_string(coord.rpy.rz));            
} 

/**
* @brief 根据编号获取负载质量及质心
* @param [in] id 负载编号
* @param [out] weight 负载质量
* @param [out] cog 负载质心
* @return 错误码
*/
std::string robot_command_thread::GetTargetPayloadWithID(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();

    double weight;
    DescTran cog;

    int res = _ptr_robot->GetTargetPayloadWithID(id,weight,cog);
    return std::string(std::to_string(res) + "," + std::to_string(weight) + "," + std::to_string(cog.x) + "," +\
                std::to_string(cog.y) + "," + std::to_string(cog.z));       
} 

/**
* @brief 获取当前工具坐标系
* @param [out] coord 坐标系数值
* @return 错误码
*/
std::string robot_command_thread::GetCurToolCoord(std::string para){
    DescPose coord;

    int res = _ptr_robot->GetCurToolCoord(coord);
    return std::string(std::to_string(res) + ","  + std::to_string(coord.tran.x) + "," + \
            std::to_string(coord.tran.y) + "," + std::to_string(coord.tran.z) + "," + \
            std::to_string(coord.rpy.rx) + "," + std::to_string(coord.rpy.ry) + "," + \
            std::to_string(coord.rpy.rz));            
}

/**
* @brief 获取当前工件坐标系
* @param [out] coord 坐标系数值
* @return 错误码
*/
std::string robot_command_thread::GetCurWObjCoord(std::string para){
    DescPose coord;

    int res = _ptr_robot->GetCurWObjCoord(coord);
    return std::string(std::to_string(res) + ","  + std::to_string(coord.tran.x) + "," + \
            std::to_string(coord.tran.y) + "," + std::to_string(coord.tran.z) + "," + \
            std::to_string(coord.rpy.rx) + "," + std::to_string(coord.rpy.ry) + "," + \
            std::to_string(coord.rpy.rz));            
}

/**
* @brief 获取当前外部工具坐标系
* @param [out] coord 坐标系数值
* @return 错误码
*/
std::string robot_command_thread::GetCurExToolCoord(std::string para){
    DescPose coord;

    int res = _ptr_robot->GetCurExToolCoord(coord);
    return std::string(std::to_string(res) + ","  + std::to_string(coord.tran.x) + "," + \
            std::to_string(coord.tran.y) + "," + std::to_string(coord.tran.z) + "," + \
            std::to_string(coord.rpy.rx) + "," + std::to_string(coord.rpy.ry) + "," + \
            std::to_string(coord.rpy.rz));            
}

/**
* @brief 获取当前扩展轴坐标系
* @param [out] coord 坐标系数值
* @return 错误码
*/
std::string robot_command_thread::GetCurExAxisCoord(std::string para){
    DescPose coord;

    int res = _ptr_robot->GetCurExAxisCoord(coord);
    return std::string(std::to_string(res) + ","  + std::to_string(coord.tran.x) + "," + \
            std::to_string(coord.tran.y) + "," + std::to_string(coord.tran.z) + "," + \
            std::to_string(coord.rpy.rx) + "," + std::to_string(coord.rpy.ry) + "," + \
            std::to_string(coord.rpy.rz));            
}

/**
 * @brief 机器人操作系统升级(LA控制箱)
 * @param [in] filePath 操作系统升级包全路径
 * @return  错误码
 */
std::string robot_command_thread::KernelUpgrade(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string filePath = list.front();list.pop_front();

    int res = _ptr_robot->KernelUpgrade(filePath);
    return std::string(std::to_string(res));            
}

/**
 * @brief 获取机器人操作系统升级结果(LA控制箱)
 * @param [out] result 升级结果：0:成功；-1:失败
 * @return  错误码
 */
std::string robot_command_thread::GetKernelUpgradeResult(std::string para){
    int result;

    int res = _ptr_robot->GetKernelUpgradeResult(result);
    return std::string(std::to_string(res) + ","  + std::to_string(result));            
}

/**
 * @brief 设置自定义摆动参数
 * @param [in] id 自定义摆动编号：0-2
 * @param [in] pointNum 摆动点位个数 0-10
 * @param [in] point 移动端点数据x,y,z
 * @param [in] stayTime 摆动停留时间ms
 * @param [in] frequency 摆动频率 Hz
 * @param [in] incStayType 等待模式：0-周期不包含等待时间；1-周期包含等待时间
 * @param [in] stationary 摆动位置等待：0-等待时间内继续运动；1-等待时间内位置静止
 * @return  错误码
 */
std::string robot_command_thread::CustomWeaveSetPara(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    int pointNum = std::stoi(list.front());list.pop_front();

    DescTran point[10];
    for(int i = 0; i < 10; i++)
    {
        point[i].x = std::stod(list.front().c_str());list.pop_front();
        point[i].y = std::stod(list.front().c_str());list.pop_front();
        point[i].z = std::stod(list.front().c_str());list.pop_front();
    }
    double stayTime[10];
    stayTime[0] = std::stod(list.front().c_str());list.pop_front();
    stayTime[1] = std::stod(list.front().c_str());list.pop_front();
    stayTime[2] = std::stod(list.front().c_str());list.pop_front();
    stayTime[3] = std::stod(list.front().c_str());list.pop_front();
    stayTime[4] = std::stod(list.front().c_str());list.pop_front();
    stayTime[5] = std::stod(list.front().c_str());list.pop_front();
    stayTime[6] = std::stod(list.front().c_str());list.pop_front();
    stayTime[7] = std::stod(list.front().c_str());list.pop_front();
    stayTime[8] = std::stod(list.front().c_str());list.pop_front();
    stayTime[9] = std::stod(list.front().c_str());list.pop_front();

    double frequency = std::stod(list.front().c_str());list.pop_front();
    int incStayType = std::stoi(list.front());list.pop_front();
    int stationary = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->CustomWeaveSetPara(id,pointNum,point,stayTime,frequency,incStayType,stationary);
    return std::string(std::to_string(res));          
} 

/**
 * @brief 获取自定义摆动参数
 * @param [in] id 自定义摆动编号：0-2
 * @param [out] pointNum 摆动点位个数 0-10
 * @param [out] point 移动端点数据x,y,z
 * @param [out] stayTime 摆动停留时间ms
 * @param [out] frequency 摆动频率 Hz
 * @param [out] incStayType 等待模式：0-周期不包含等待时间；1-周期包含等待时间
 * @param [out] stationary 摆动位置等待：0-等待时间内继续运动；1-等待时间内位置静止
 * @return  错误码
 */
std::string robot_command_thread::CustomWeaveGetPara(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int id = std::stoi(list.front());list.pop_front();
    int pointNum;
    DescTran point[10];
    double stayTime[10];
    double frequency;
    int incStayType;
    int stationary;

    int res = _ptr_robot->CustomWeaveGetPara(id,pointNum,point,stayTime,frequency,incStayType,stationary);
    return std::string(std::to_string(res) + "," + std::to_string(pointNum) + "," +\
                std::to_string(point[0].x) + "," + std::to_string(point[0].y) + "," +\
                std::to_string(point[0].z) + "," + std::to_string(point[1].x) + "," +\
                std::to_string(point[1].y) + "," + std::to_string(point[1].z) + "," +\
                std::to_string(point[2].x) + "," + std::to_string(point[2].y) + "," +\
                std::to_string(point[2].z) + "," + std::to_string(point[3].x) + "," +\
                std::to_string(point[3].y) + "," + std::to_string(point[3].z) + "," +\
                std::to_string(point[4].x) + "," + std::to_string(point[4].y) + "," +\
                std::to_string(point[4].z) + "," + std::to_string(point[5].x) + "," +\
                std::to_string(point[5].y) + "," + std::to_string(point[5].z) + "," +\
                std::to_string(point[6].x) + "," + std::to_string(point[6].y) + "," +\
                std::to_string(point[6].z) + "," + std::to_string(point[7].x) + "," +\
                std::to_string(point[7].y) + "," + std::to_string(point[7].z) + "," +\
                std::to_string(point[8].x) + "," + std::to_string(point[8].y) + "," +\
                std::to_string(point[8].z) + "," + std::to_string(point[9].x) + "," +\
                std::to_string(point[9].y) + "," + std::to_string(point[9].z) + "," +\
                std::to_string(stayTime[0]) + "," + std::to_string(stayTime[1]) + "," +\
                std::to_string(stayTime[2]) + "," + std::to_string(stayTime[3]) + "," +\
                std::to_string(stayTime[4]) + "," + std::to_string(stayTime[5]) + "," +\
                std::to_string(stayTime[6]) + "," + std::to_string(stayTime[7]) + "," +\
                std::to_string(stayTime[8]) + "," + std::to_string(stayTime[9]) + "," +\
                std::to_string(frequency) + "," + std::to_string(incStayType) + "," +\
                std::to_string(stationary));            
} 

/**
 * @brief 关节扭矩传感器灵敏度标定功能开启
 * @param [in] status 0-关闭；1-开启
 * @return  错误码
 */
std::string robot_command_thread::JointSensitivityEnable(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int status = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->JointSensitivityEnable(status);
    return std::string(std::to_string(res));          
} 

/**
 * @brief 获取关节扭矩传感器灵敏度标定结果
 * @param [out] calibResult j1~j6关节灵敏度[0-1]
 * @param [out] linearityn j1~j6关节线性度[0-1]
 * @return 错误码
 */
std::string robot_command_thread::JointSensitivityCalibration(std::string para){
    double calibResult[6];
    double linearity[6];

    int res = _ptr_robot->JointSensitivityCalibration(calibResult,linearity);
    return std::string(std::to_string(res)  + "," +\
                std::to_string(calibResult[0]) + "," + std::to_string(calibResult[1]) + "," +\
                std::to_string(calibResult[2]) + "," + std::to_string(calibResult[3]) + "," +\
                std::to_string(calibResult[4]) + "," + std::to_string(calibResult[5]) + "," +\
                std::to_string(linearity[0]) + "," + std::to_string(linearity[1]) + "," +\
                std::to_string(linearity[2]) + "," + std::to_string(linearity[3]) + "," +\
                std::to_string(linearity[4]) + "," + std::to_string(linearity[5]));            
} 

/**
 * @brief 关节扭矩传感器灵敏度数据采集
 * @return 错误码
 */
std::string robot_command_thread::JointSensitivityCollect(std::string para){

    int res = _ptr_robot->JointSensitivityCollect();
    return std::string(std::to_string(res));            
} 

std::string robot_command_thread::Sleep(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int ms = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->Sleep(ms);
    return std::string(std::to_string(res));            
} 

/**
 * @brief 清空运动指令队列
 * @return 错误码
 */
std::string robot_command_thread::MotionQueueClear(std::string para){

    int res = _ptr_robot->MotionQueueClear();
    return std::string(std::to_string(res));            
} 

/**
 * @brief 获取机器人8个从站端口错误帧数
 * @param [out] inRecvErr 输入接收错误帧数 
 * @param [out] inCRCErr 输入CRC错误帧数 
 * @param [out] inTransmitErr 输入转发错误帧数 
 * @param [out] inLinkErr 输入链接错误帧数 
 * @param [out] outRecvErr 输出接收错误帧数
 * @param [out] outCRCErr 输出CRC错误帧数
 * @param [out] outTransmitErr 输出转发错误帧数
 * @param [out] outLinkErr 输出链接错误帧数
 * @return 错误码
 */
std::string robot_command_thread::GetSlavePortErrCounter(std::string para){
    int inRecvErr[8];
    int inCRCErr[8];
    int inTransmitErr[8];
    int inLinkErr[8];
    int outRecvErr[8];
    int outCRCErr[8];
    int outTransmitErr[8];
    int outLinkErr[8];

    int res = _ptr_robot->GetSlavePortErrCounter(inRecvErr,inCRCErr,inTransmitErr,inLinkErr,outRecvErr,outCRCErr,outTransmitErr,outLinkErr);
    std::string out = std::to_string(res);
    for (int i = 0; i < 8; ++i) {
        out += "," + std::to_string(inRecvErr[i]);
    }
    for (int i = 0; i < 8; ++i) {
        out += "," + std::to_string(inCRCErr[i]);
    }
    for (int i = 0; i < 8; ++i) {
        out += "," + std::to_string(inTransmitErr[i]);
    }
    for (int i = 0; i < 8; ++i) {
        out += "," + std::to_string(inLinkErr[i]);
    }
    for (int i = 0; i < 8; ++i) {
        out += "," + std::to_string(outRecvErr[i]);
    }
    for (int i = 0; i < 8; ++i) {
        out += "," + std::to_string(outCRCErr[i]);
    }
    for (int i = 0; i < 8; ++i) {
        out += "," + std::to_string(outTransmitErr[i]);
    }
    for (int i = 0; i < 8; ++i) {
        out += "," + std::to_string(outLinkErr[i]);
    }
    return out;        
} 

/**
 * @brief 从站端口错误帧清零
 * @param [in] slaveID 从站编号0~7
 * @return 错误码
 */
std::string robot_command_thread::SlavePortErrCounterClear(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int slaveID = std::stoi(list.front());list.pop_front();

    int res = _ptr_robot->SlavePortErrCounterClear(slaveID);
    return std::string(std::to_string(res));            
} 

/**
 * @brief 设置各轴速度前馈系数
 * @param [in] radio 各轴速度前馈系数
 * @return 错误码
 */
std::string robot_command_thread::SetVelFeedForwardRatio(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double radio[6];
    radio[0] = std::stod(list.front());list.pop_front();
    radio[1] = std::stod(list.front());list.pop_front();
    radio[2] = std::stod(list.front());list.pop_front();
    radio[3] = std::stod(list.front());list.pop_front();
    radio[4] = std::stod(list.front());list.pop_front();
    radio[5] = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->SetVelFeedForwardRatio(radio);
    return std::string(std::to_string(res));            
} 

/**
 * @brief 获取各轴速度前馈系数
 * @param [out] radio 各轴速度前馈系数
 * @return 错误码
 */
std::string robot_command_thread::GetVelFeedForwardRatio(std::string para){
    double radio[6];

    int res = _ptr_robot->GetVelFeedForwardRatio(radio);
    return std::string(std::to_string(res)  + "," +\
                std::to_string(radio[0]) + "," + std::to_string(radio[1]) + "," +\
                std::to_string(radio[2]) + "," + std::to_string(radio[3]) + "," +\
                std::to_string(radio[4]) + "," + std::to_string(radio[5]));            
}

/**
 * @brief 机器人MCU日志生成
 * @return 错误码
 */
std::string robot_command_thread::RobotMCULogCollect(std::string para){

    int res = _ptr_robot->RobotMCULogCollect();
    return std::string(std::to_string(res));            
} 

/**
 * @brief 移动到相贯线起始点
 * @param [in] mainPoint 主管6个示教点的笛卡尔位姿
 * @param [in] piecePoint 辅管6个示教点的笛卡尔位姿
 * @param [in] tool 工具坐标系编号
 * @param [in] wobj 工件坐标系编号
 * @param [in] vel 速度百分比
 * @param [in] acc 加速度百分比
 * @param [in] ovl 速度缩放因子
 * @param [in] oacc 加速度缩放因子
 * @param [in] moveType 运动类型; 0-PTP；1-LIN
 * @return 错误码
 */
std::string robot_command_thread::MoveToIntersectLineStart(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int parameters_count = list.size();

    if(parameters_count == 139)
    {
        DescPose mainPoint[6];
        _fillDescPose(list,mainPoint[0]);
        _fillDescPose(list,mainPoint[1]);
        _fillDescPose(list,mainPoint[2]);
        _fillDescPose(list,mainPoint[3]);
        _fillDescPose(list,mainPoint[4]);
        _fillDescPose(list,mainPoint[5]);
        ExaxisPos mainExaxisPos[6];
        for(int i = 0;i < 6; i++)
        {
            mainExaxisPos[i].ePos[0] = std::stod(list.front().c_str());list.pop_front();
            mainExaxisPos[i].ePos[1] = std::stod(list.front().c_str());list.pop_front();
            mainExaxisPos[i].ePos[2] = std::stod(list.front().c_str());list.pop_front();
            mainExaxisPos[i].ePos[3] = std::stod(list.front().c_str());list.pop_front();
        }
        DescPose piecePoint[6];
        _fillDescPose(list,piecePoint[0]);
        _fillDescPose(list,piecePoint[1]);
        _fillDescPose(list,piecePoint[2]);
        _fillDescPose(list,piecePoint[3]);
        _fillDescPose(list,piecePoint[4]);
        _fillDescPose(list,piecePoint[5]);
        ExaxisPos pieceExaxisPos[6];
        for(int i = 0;i < 6; i++)
        {
            pieceExaxisPos[i].ePos[0] = std::stod(list.front().c_str());list.pop_front();
            pieceExaxisPos[i].ePos[1] = std::stod(list.front().c_str());list.pop_front();
            pieceExaxisPos[i].ePos[2] = std::stod(list.front().c_str());list.pop_front();
            pieceExaxisPos[i].ePos[3] = std::stod(list.front().c_str());list.pop_front();
        }
        int extAxisFlag = std::stoi(list.front());list.pop_front();
        ExaxisPos exaxisPos;
        exaxisPos.ePos[0] = std::stod(list.front().c_str());list.pop_front();
        exaxisPos.ePos[1] = std::stod(list.front().c_str());list.pop_front();
        exaxisPos.ePos[2] = std::stod(list.front().c_str());list.pop_front();
        exaxisPos.ePos[3] = std::stod(list.front().c_str());list.pop_front();
        int tool = std::stoi(list.front());list.pop_front();
        int wobj = std::stoi(list.front());list.pop_front();
        double vel = std::stod(list.front());list.pop_front();
        double acc = std::stod(list.front());list.pop_front();
        double ovl = std::stod(list.front());list.pop_front();
        double oacc = std::stod(list.front());list.pop_front();
        int moveType = std::stoi(list.front());list.pop_front();
        int moveDirection = std::stoi(list.front());list.pop_front();
        DescPose offset;
        _fillDescPose(list,offset);

        int res = _ptr_robot->MoveToIntersectLineStart(mainPoint,mainExaxisPos,piecePoint,pieceExaxisPos,extAxisFlag,exaxisPos,tool,wobj,vel,acc,ovl,oacc,moveType,moveDirection,offset);
        return std::string(std::to_string(res));
    }
    else
    {
        DescPose mainPoint[6];
        _fillDescPose(list,mainPoint[0]);
        _fillDescPose(list,mainPoint[1]);
        _fillDescPose(list,mainPoint[2]);
        _fillDescPose(list,mainPoint[3]);
        _fillDescPose(list,mainPoint[4]);
        _fillDescPose(list,mainPoint[5]);
        DescPose piecePoint[6];
        _fillDescPose(list,piecePoint[0]);
        _fillDescPose(list,piecePoint[1]);
        _fillDescPose(list,piecePoint[2]);
        _fillDescPose(list,piecePoint[3]);
        _fillDescPose(list,piecePoint[4]);
        _fillDescPose(list,piecePoint[5]);
        int tool = std::stoi(list.front());list.pop_front();
        int wobj = std::stoi(list.front());list.pop_front();
        double vel = std::stod(list.front());list.pop_front();
        double acc = std::stod(list.front());list.pop_front();
        double ovl = std::stod(list.front());list.pop_front();
        double oacc = std::stod(list.front());list.pop_front();
        int moveType = std::stoi(list.front());list.pop_front();

        int res = _ptr_robot->MoveToIntersectLineStart(mainPoint,piecePoint,tool,wobj,vel,acc,ovl,oacc,moveType);
        return std::string(std::to_string(res));   
    }         
} 

/**
 * @brief 相贯线运动
 * @param [in] mainPoint 主管6个示教点的笛卡尔位姿
 * @param [in] piecePoint 辅管6个示教点的笛卡尔位姿
 * @param [in] tool 工具坐标系编号
 * @param [in] wobj 工件坐标系编号
 * @param [in] vel 速度百分比
 * @param [in] acc 加速度百分比
 * @param [in] ovl 速度缩放因子
 * @param [in] oacc 加速度缩放因子
 * @param [in] moveDirection 运动方向; 0-顺时针；1-逆时针
 * @return 错误码
 */
std::string robot_command_thread::MoveIntersectLine(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int parameters_count = list.size();

    if(parameters_count == 150)
    {
        DescPose mainPoint[6];
        _fillDescPose(list,mainPoint[0]);
        _fillDescPose(list,mainPoint[1]);
        _fillDescPose(list,mainPoint[2]);
        _fillDescPose(list,mainPoint[3]);
        _fillDescPose(list,mainPoint[4]);
        _fillDescPose(list,mainPoint[5]);
        ExaxisPos mainExaxisPos[6];
        for(int i = 0;i < 6; i++)
        {
            mainExaxisPos[i].ePos[0] = std::stod(list.front().c_str());list.pop_front();
            mainExaxisPos[i].ePos[1] = std::stod(list.front().c_str());list.pop_front();
            mainExaxisPos[i].ePos[2] = std::stod(list.front().c_str());list.pop_front();
            mainExaxisPos[i].ePos[3] = std::stod(list.front().c_str());list.pop_front();
        }
        DescPose piecePoint[6];
        _fillDescPose(list,piecePoint[0]);
        _fillDescPose(list,piecePoint[1]);
        _fillDescPose(list,piecePoint[2]);
        _fillDescPose(list,piecePoint[3]);
        _fillDescPose(list,piecePoint[4]);
        _fillDescPose(list,piecePoint[5]);
        ExaxisPos pieceExaxisPos[6];
        for(int i = 0;i < 6; i++)
        {
            pieceExaxisPos[i].ePos[0] = std::stod(list.front().c_str());list.pop_front();
            pieceExaxisPos[i].ePos[1] = std::stod(list.front().c_str());list.pop_front();
            pieceExaxisPos[i].ePos[2] = std::stod(list.front().c_str());list.pop_front();
            pieceExaxisPos[i].ePos[3] = std::stod(list.front().c_str());list.pop_front();
        }
        int extAxisFlag = std::stoi(list.front());list.pop_front();
        ExaxisPos exaxisPos[4];
        for(int i = 0;i < 4; i++)
        {
            exaxisPos[i].ePos[0] = std::stod(list.front().c_str());list.pop_front();
            exaxisPos[i].ePos[1] = std::stod(list.front().c_str());list.pop_front();
            exaxisPos[i].ePos[2] = std::stod(list.front().c_str());list.pop_front();
            exaxisPos[i].ePos[3] = std::stod(list.front().c_str());list.pop_front();
        }
        int tool = std::stoi(list.front());list.pop_front();
        int wobj = std::stoi(list.front());list.pop_front();
        double vel = std::stod(list.front());list.pop_front();
        double acc = std::stod(list.front());list.pop_front();
        double ovl = std::stod(list.front());list.pop_front();
        double oacc = std::stod(list.front());list.pop_front();
        int moveDirection = std::stoi(list.front());list.pop_front();
        DescPose offset;
        _fillDescPose(list,offset);

        int res = _ptr_robot->MoveIntersectLine(mainPoint,mainExaxisPos,piecePoint,pieceExaxisPos,extAxisFlag,exaxisPos,tool,wobj,vel,acc,ovl,oacc,moveDirection,offset);
        return std::string(std::to_string(res));
    }
    else
    {
        DescPose mainPoint[6];
        _fillDescPose(list,mainPoint[0]);
        _fillDescPose(list,mainPoint[1]);
        _fillDescPose(list,mainPoint[2]);
        _fillDescPose(list,mainPoint[3]);
        _fillDescPose(list,mainPoint[4]);
        _fillDescPose(list,mainPoint[5]);
        DescPose piecePoint[6];
        _fillDescPose(list,piecePoint[0]);
        _fillDescPose(list,piecePoint[1]);
        _fillDescPose(list,piecePoint[2]);
        _fillDescPose(list,piecePoint[3]);
        _fillDescPose(list,piecePoint[4]);
        _fillDescPose(list,piecePoint[5]);
        int tool = std::stoi(list.front());list.pop_front();
        int wobj = std::stoi(list.front());list.pop_front();
        double vel = std::stod(list.front());list.pop_front();
        double acc = std::stod(list.front());list.pop_front();
        double ovl = std::stod(list.front());list.pop_front();
        double oacc = std::stod(list.front());list.pop_front();
        int moveDirection = std::stoi(list.front());list.pop_front();

        int res = _ptr_robot->MoveIntersectLine(mainPoint,piecePoint,tool,wobj,vel,acc,ovl,oacc,moveDirection);
        return std::string(std::to_string(res));   
    }         
}
/**
 * @brief 获取关节扭矩传感器迟滞误差
 * @param [out] hysteresisError j1~j6关节迟滞误差
 * @return 错误码
 */
std::string robot_command_thread::JointHysteresisError(std::string para){
    double hysteresisError[6];

    int res = _ptr_robot->JointHysteresisError(hysteresisError);
    return std::string(std::to_string(res)  + "," +\
                std::to_string(hysteresisError[0]) + "," + std::to_string(hysteresisError[1]) + "," +\
                std::to_string(hysteresisError[2]) + "," + std::to_string(hysteresisError[3]) + "," +\
                std::to_string(hysteresisError[4]) + "," + std::to_string(hysteresisError[5]));            
}

/**
 * @brief 获取关节扭矩传感器重复精度
 * @param [out] repeatability j1~j6关节扭矩传感器重复精度
 * @return 错误码
 */
std::string robot_command_thread::JointRepeatability(std::string para){
    double repeatability[6];

    int res = _ptr_robot->JointRepeatability(repeatability);
    return std::string(std::to_string(res)  + "," +\
                std::to_string(repeatability[0]) + "," + std::to_string(repeatability[1]) + "," +\
                std::to_string(repeatability[2]) + "," + std::to_string(repeatability[3]) + "," +\
                std::to_string(repeatability[4]) + "," + std::to_string(repeatability[5]));            
}

/**
 * @brief 设置关节力传感器参数
 * @param [in] M J1-J6质量系数[0.001 ~ 10]
 * @param [in] B J1-J6阻尼系数[0.001 ~ 10]
 * @param [in] K J1-J6刚度系数[0.001 ~ 10]
 * @param [in] threshold 力控制阈值，Nm
 * @param [in] sensitivity 灵敏度,Nm/V,[0 ~ 10]
 * @param [in] setZeroFlag 功能开启标志位；0-关闭；1-开启；2-位置1记录零点；3-位置2记录零点
 * @return 错误码
 */

std::string robot_command_thread::SetAdmittanceParams(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    double M[6];
    M[0] = std::stod(list.front());list.pop_front();
    M[1] = std::stod(list.front());list.pop_front();
    M[2] = std::stod(list.front());list.pop_front();
    M[3] = std::stod(list.front());list.pop_front();
    M[4] = std::stod(list.front());list.pop_front();
    M[5] = std::stod(list.front());list.pop_front();
    double B[6];
    B[0] = std::stod(list.front());list.pop_front();
    B[1] = std::stod(list.front());list.pop_front();
    B[2] = std::stod(list.front());list.pop_front();
    B[3] = std::stod(list.front());list.pop_front();
    B[4] = std::stod(list.front());list.pop_front();
    B[5] = std::stod(list.front());list.pop_front();
    double K[6];
    K[0] = std::stod(list.front());list.pop_front();
    K[1] = std::stod(list.front());list.pop_front();
    K[2] = std::stod(list.front());list.pop_front();
    K[3] = std::stod(list.front());list.pop_front();
    K[4] = std::stod(list.front());list.pop_front();
    K[5] = std::stod(list.front());list.pop_front();
    double threshold[6];
    threshold[0] = std::stod(list.front());list.pop_front();
    threshold[1] = std::stod(list.front());list.pop_front();
    threshold[2] = std::stod(list.front());list.pop_front();
    threshold[3] = std::stod(list.front());list.pop_front();
    threshold[4] = std::stod(list.front());list.pop_front();
    threshold[5] = std::stod(list.front());list.pop_front();
    double sensitivity[6];
    sensitivity[0] = std::stod(list.front());list.pop_front();
    sensitivity[1] = std::stod(list.front());list.pop_front();
    sensitivity[2] = std::stod(list.front());list.pop_front();
    sensitivity[3] = std::stod(list.front());list.pop_front();
    sensitivity[4] = std::stod(list.front());list.pop_front();
    sensitivity[5] = std::stod(list.front());list.pop_front();

    int setZeroFlag = std::stoi(list.front());list.pop_front();
    
    int res = _ptr_robot->SetAdmittanceParams(M,B,K,threshold,sensitivity,setZeroFlag);
    return std::string(std::to_string(res));            
}

/**
 * @brief 开启力矩补偿功能及补偿系数
 * @param [in] status 开关，0-关闭；1-开启
 * @param [in] torqueCoeff J1-J6力矩补偿系数[0-1]
 * @return 错误码
 */
std::string robot_command_thread::SerCoderCompenParams(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int status = std::stoi(list.front());list.pop_front();
    double torqueCoeff[6];
    torqueCoeff[0] = std::stod(list.front());list.pop_front();
    torqueCoeff[1] = std::stod(list.front());list.pop_front();
    torqueCoeff[2] = std::stod(list.front());list.pop_front();
    torqueCoeff[3] = std::stod(list.front());list.pop_front();
    torqueCoeff[4] = std::stod(list.front());list.pop_front();
    torqueCoeff[5] = std::stod(list.front());list.pop_front();

    int res = _ptr_robot->SerCoderCompenParams(status,torqueCoeff);
    return std::string(std::to_string(res));
}

/**
 * @brief 光电传感器TCP标定-计算工具RPY
 * @param [in] Btool 机器人笛卡尔位置
 * @param [in] Etool 当前工具坐标系数值
 * @param [in] senser 当前传感器坐标系数值(暂未开放)
 * @param [in] radius 圆周运动半径mm(暂未开放)
 * @param [in] dz 沿基座标系z轴负方向运动距离；当dz = 10000时，函数直接返回工具RPY
 * @param [out] TCPRPY 工具RPY数值
 * @return 错误码
 */
std::string robot_command_thread::TCPComputeRPY(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    DescPose Btool;
    _fillDescPose(list,Btool);
    DescPose Etool;
    _fillDescPose(list,Etool);
    DescPose sensor;
    _fillDescPose(list,sensor);
    double radius = std::stod(list.front());list.pop_front();
    double dz = std::stod(list.front());list.pop_front();
    Rpy TCPRPY;

    int res = _ptr_robot->TCPComputeRPY(Btool, Etool, sensor, radius, dz, TCPRPY);
    return std::string(std::to_string(res)  + "," + std::to_string(TCPRPY.rx) + "," +\
            std::to_string(TCPRPY.ry) + "," + std::to_string(TCPRPY.rz));         
}

/**
 * @brief 光电传感器TCP标定-计算工具XYZ
 * @param [in] select 0-计算工具TCP；1-计算传感器原点；2-计算传感器姿态；3-直接返回工具TCP；4-记录当前工件坐标系和工具坐标系
 * @param [in] originDirection 0-X方向；1-Y方向；2-Z方向
 * @param [in] pos1 机器人笛卡尔位置1
 * @param [in] pos2 机器人笛卡尔位置2
 * @param [in] pos3 机器人笛卡尔位置3
 * @param [in] pos4 机器人笛卡尔位置4
 * @param [out] TCP 工具XYZ数值
 * @return 错误码
 */
std::string robot_command_thread::TCPComputeXYZ(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    int select = std::stoi(list.front());list.pop_front();
    double originDirection = std::stod(list.front());list.pop_front();

    DescTran pos1;
    pos1.x = std::stod(list.front().c_str());list.pop_front();
    pos1.y = std::stod(list.front().c_str());list.pop_front();
    pos1.z = std::stod(list.front().c_str());list.pop_front();
    DescTran pos2;
    pos2.x = std::stod(list.front().c_str());list.pop_front();
    pos2.y = std::stod(list.front().c_str());list.pop_front();
    pos2.z = std::stod(list.front().c_str());list.pop_front();
    DescTran pos3;
    pos3.x = std::stod(list.front().c_str());list.pop_front();
    pos3.y = std::stod(list.front().c_str());list.pop_front();
    pos3.z = std::stod(list.front().c_str());list.pop_front();
    DescTran pos4;
    pos4.x = std::stod(list.front().c_str());list.pop_front();
    pos4.y = std::stod(list.front().c_str());list.pop_front();
    pos4.z = std::stod(list.front().c_str());list.pop_front();

    DescTran TCP;

    int res = _ptr_robot->TCPComputeXYZ(select, originDirection, pos1, pos2, pos3, pos4, TCP);
    return std::string(std::to_string(res)  + "," + std::to_string(TCP.x) + "," +\
            std::to_string(TCP.y) + "," + std::to_string(TCP.z));         
}

	/**
	 * @brief 光电传感器TCP标定-开始记录末端法兰中心位置
	 * @return 错误码
	 */
std::string robot_command_thread::TCPRecordFlangePosStart(std::string para){
    para.clear();

    int res = _ptr_robot->TCPRecordFlangePosStart();
    return std::string(std::to_string(res));
}


	/**
	 * @brief 光电传感器TCP标定-停止记录末端法兰中心位置
	 * @return 错误码
	 */
std::string robot_command_thread::TCPRecordFlangePosEnd(std::string para){
    para.clear();
    
    int res = _ptr_robot->TCPRecordFlangePosEnd();
    return std::string(std::to_string(res));
}

	/**
	 * @brief 光电传感器TCP标定-获取末端工具中心点位置
	 * @param [out] TCP 工具中心点位置(x,y,z)
	 * @return 错误码
	 */
std::string robot_command_thread::TCPGetRecordFlangePos(std::string para){
    DescTran TCP;
    
    int res = _ptr_robot->TCPGetRecordFlangePos(TCP);
    return std::string(std::to_string(res)  + "," + std::to_string(TCP.x) + "," +\
            std::to_string(TCP.y) + "," + std::to_string(TCP.z));   
}

/**
 * @brief 光电传感器TCP标定
 * @param [in] luaPath 自动标定lua程序路径：QX版本机器人-"/fruser/FR_CalibrateTheToolTcp.lua";LA版本机器人-"/usr/local/etc/controller/lua/FR_CalibrateTheToolTcp.lua"
 * @param [in] offsetX 示教点偏移(x,y,z)mm
 * @param [out] TCP 标定后的工具坐标系(x,y,z,rx,ry,rz)
 * @return 错误码
 */
std::string robot_command_thread::PhotoelectricSensorTCPCalibration(std::string para){
    std::list<std::string> list;
    _splitString2List(para,list);

    std::string luaPath = list.front();list.pop_front();
    DescTran offset;
    offset.x = std::stod(list.front().c_str());list.pop_front();
    offset.y = std::stod(list.front().c_str());list.pop_front();
    offset.z = std::stod(list.front().c_str());list.pop_front();

    DescPose TCP;

    int res = _ptr_robot->PhotoelectricSensorTCPCalibration(luaPath, offset, TCP);
    return std::string(std::to_string(res) + ","  + std::to_string(TCP.tran.x) + "," + \
        std::to_string(TCP.tran.y) + "," + std::to_string(TCP.tran.z) + "," + \
        std::to_string(TCP.rpy.rx) + "," + std::to_string(TCP.rpy.ry) + "," + \
        std::to_string(TCP.rpy.rz));            
}


/**
 * @brief 原地空运动
 * @return 错误码
 */
std::string robot_command_thread::MoveStationary(std::string para){
    para.clear();
    
    int res = _ptr_robot->MoveStationary();
    return std::string(std::to_string(res));
}




/**
 * @brief 数据端口topic监听回调函数
*/
void robot_command_thread::_state_recv_callback(){
    auto msg = robot_feedback_msg();
    static ROBOT_STATE_PKG ctrl_state;
    _ptr_robot->GetRobotRealTimeState(&ctrl_state);

    msg.prg_state = ctrl_state.program_state;   // 程序运行状态，1-停止；2-运行；3-暂停；
    msg.rbt_state = ctrl_state.robot_state;     // 机器人运动状态，1-停止；2-运行；3-暂停；4-拖动 
    msg.rbt_main_code = ctrl_state.main_code;   // 主故障码 
    msg.rbt_sub_code = ctrl_state.sub_code;     // 子故障码 
    msg.robot_mode = ctrl_state.robot_mode; // 机器人模式，1-手动模式；0-自动模式；

    // 6个轴当前关节位置，单位deg 
    msg.j1_cur_pos = ctrl_state.jt_cur_pos[0];
    msg.j2_cur_pos = ctrl_state.jt_cur_pos[1];
    msg.j3_cur_pos = ctrl_state.jt_cur_pos[2];
    msg.j4_cur_pos = ctrl_state.jt_cur_pos[3];
    msg.j5_cur_pos = ctrl_state.jt_cur_pos[4];
    msg.j6_cur_pos = ctrl_state.jt_cur_pos[5];

    // 工具当前位置
    msg.cart_x_cur_pos = ctrl_state.tl_cur_pos[0];
    msg.cart_y_cur_pos = ctrl_state.tl_cur_pos[1];
    msg.cart_z_cur_pos = ctrl_state.tl_cur_pos[2];
    msg.cart_a_cur_pos = ctrl_state.tl_cur_pos[3];
    msg.cart_b_cur_pos = ctrl_state.tl_cur_pos[4];
    msg.cart_c_cur_pos = ctrl_state.tl_cur_pos[5];

    // 末端法兰当前位置
    msg.flange_x_cur_pos = ctrl_state.flange_cur_pos[0];
    msg.flange_y_cur_pos = ctrl_state.flange_cur_pos[1];
    msg.flange_z_cur_pos = ctrl_state.flange_cur_pos[2];
    msg.flange_a_cur_pos = ctrl_state.flange_cur_pos[3];
    msg.flange_b_cur_pos = ctrl_state.flange_cur_pos[4];
    msg.flange_c_cur_pos = ctrl_state.flange_cur_pos[5];

    // 当前6个关节速度，单位deg/s 
    msg.j1_actual_qd = ctrl_state.actual_qd[0];
    msg.j2_actual_qd = ctrl_state.actual_qd[1];
    msg.j3_actual_qd = ctrl_state.actual_qd[2];
    msg.j4_actual_qd = ctrl_state.actual_qd[3];
    msg.j5_actual_qd = ctrl_state.actual_qd[4];
    msg.j6_actual_qd = ctrl_state.actual_qd[5];

    // 当前6个关节加速度，单位deg/s^2 
    msg.j1_actual_qdd = ctrl_state.actual_qdd[0];
    msg.j2_actual_qdd = ctrl_state.actual_qdd[1];
    msg.j3_actual_qdd = ctrl_state.actual_qdd[2];
    msg.j4_actual_qdd = ctrl_state.actual_qdd[3];
    msg.j5_actual_qdd = ctrl_state.actual_qdd[4];
    msg.j6_actual_qdd = ctrl_state.actual_qdd[5];


    msg.cart_lin_cmd_speed = ctrl_state.target_TCP_CmpSpeed[0];
    msg.cart_rot_cmd_speed = ctrl_state.target_TCP_CmpSpeed[1];

    // TCP指令速度
    msg.cart_x_cmd_speed = ctrl_state.target_TCP_Speed[0];
    msg.cart_y_cmd_speed = ctrl_state.target_TCP_Speed[1];
    msg.cart_z_cmd_speed = ctrl_state.target_TCP_Speed[2];
    msg.cart_a_cmd_speed = ctrl_state.target_TCP_Speed[3];
    msg.cart_b_cmd_speed = ctrl_state.target_TCP_Speed[4];
    msg.cart_c_cmd_speed = ctrl_state.target_TCP_Speed[5];

    msg.cart_lin_cur_speed = ctrl_state.actual_TCP_CmpSpeed[0];
    msg.cart_rot_cur_speed = ctrl_state.actual_TCP_CmpSpeed[1];

    // TCP实际速度
    msg.cart_x_cur_speed = ctrl_state.actual_TCP_Speed[0];
    msg.cart_y_cur_speed = ctrl_state.actual_TCP_Speed[1];
    msg.cart_z_cur_speed = ctrl_state.actual_TCP_Speed[2];
    msg.cart_a_cur_speed = ctrl_state.actual_TCP_Speed[3];
    msg.cart_b_cur_speed = ctrl_state.actual_TCP_Speed[4];
    msg.cart_c_cur_speed = ctrl_state.actual_TCP_Speed[5];

    // 6个轴当前扭矩，单位N·m 
    msg.j1_cur_tor = ctrl_state.jt_cur_tor[0];
    msg.j2_cur_tor = ctrl_state.jt_cur_tor[1];
    msg.j3_cur_tor = ctrl_state.jt_cur_tor[2];
    msg.j4_cur_tor = ctrl_state.jt_cur_tor[3];
    msg.j5_cur_tor = ctrl_state.jt_cur_tor[4];
    msg.j6_cur_tor = ctrl_state.jt_cur_tor[5];
    
    // 应用的工具坐标系编号 
    msg.tool_num = ctrl_state.tool;
    // 应用的工件坐标系编号 
    msg.work_num = ctrl_state.user;
    // 控制箱数字量IO输出15-8 
    msg.dgt_output_h = ctrl_state.cl_dgt_output_h;
    // 控制箱数字量IO输出7-0 
    msg.dgt_output_l = ctrl_state.cl_dgt_output_l;
    // 工具数字量IO输出7-0，仅bit0-bit1有效 
    msg.tl_dgt_output_l = ctrl_state.tl_dgt_output_l;
    // 控制箱数字量IO输入15-8 
    msg.dgt_input_h = ctrl_state.cl_dgt_input_h;
    // 控制箱数字量IO输入7-0 
    msg.dgt_input_l = ctrl_state.cl_dgt_input_l;
    // 工具数字量IO输入7-0，仅bit0-bit1有效 
    msg.tl_dgt_input_l = ctrl_state.tl_dgt_input_l;
    
    // 控制箱模拟量输入
    msg.cl_analog_input_1 = ctrl_state.cl_analog_input[0];
    msg.cl_analog_input_2 = ctrl_state.cl_analog_input[1];
    
    // 工具模拟量输入 
    msg.tl_anglog_input = ctrl_state.tl_anglog_input;

    // 力矩传感器原始数据
    msg.ft_fx_raw_data = ctrl_state.ft_sensor_raw_data[0];
    msg.ft_fy_raw_data = ctrl_state.ft_sensor_raw_data[1];
    msg.ft_fz_raw_data = ctrl_state.ft_sensor_raw_data[2];
    msg.ft_tx_raw_data = ctrl_state.ft_sensor_raw_data[3];
    msg.ft_ty_raw_data = ctrl_state.ft_sensor_raw_data[4];
    msg.ft_tz_raw_data = ctrl_state.ft_sensor_raw_data[5];

    // 力矩传感器数据
    msg.ft_fx_data = ctrl_state.ft_sensor_data[0];
    msg.ft_fy_data = ctrl_state.ft_sensor_data[1];
    msg.ft_fz_data = ctrl_state.ft_sensor_data[2];
    msg.ft_tx_data = ctrl_state.ft_sensor_data[3];
    msg.ft_ty_data = ctrl_state.ft_sensor_data[4];
    msg.ft_tz_data = ctrl_state.ft_sensor_data[5];

    //力矩传感器激活状态
    msg.ft_actstatus = ctrl_state.ft_sensor_active;
    
    // 急停标志，0-急停未按下，1-急停按下 
    msg.emg = ctrl_state.EmergencyStop;

    // 运动到位信号
    msg.motion_done = ctrl_state.motion_done;

    // 夹爪运动完成信号 
    msg.grip_motion_done = ctrl_state.gripper_motiondone;

    msg.mc_queue_len = ctrl_state.mc_queue_len;       // 运动指令队列长度 
    msg.collision_err = ctrl_state.collisionState;     // 碰撞检测，1-碰撞，0-无碰撞 
    msg.trajectory_pnum = ctrl_state.trajectory_pnum;    // 轨迹点编号 
    msg.safety_stop1_state = ctrl_state.safety_stop0_state; // 安全停止信号SI0 
    msg.safety_stop2_state = ctrl_state.safety_stop1_state; // 安全停止信号SI1 
    msg.gripper_fault_id = ctrl_state.gripper_fault_id;   // 错误夹爪号 
    msg.grippererro = ctrl_state.gripper_fault;      // 夹爪故障 
    msg.gripper_active = ctrl_state.gripper_active;     // 夹爪激活状态 
    msg.gripper_position = ctrl_state.gripper_position;   // 夹爪位置 
    msg.gripper_speed = ctrl_state.gripper_speed;      // 夹爪速度 
    msg.gripper_current = ctrl_state.gripper_current;    // 夹爪电流 
    msg.gripper_temp = ctrl_state.gripper_temp;       // 夹爪温度 
    msg.gripper_voltage = ctrl_state.gripper_voltage;    // 夹爪电压 

    msg.aux_servo_id = ctrl_state.aux_state.servoId;   // 485扩展轴状态
    msg.aux_servo_err = ctrl_state.aux_state.servoErrCode; 
    msg.aux_servo_state = ctrl_state.aux_state.servoState; 
    msg.aux_servo_pos = ctrl_state.aux_state.servoPos; 
    msg.aux_servo_vel = ctrl_state.aux_state.servoVel; 
    msg.aux_servo_torque = ctrl_state.aux_state.servoTorque; 
    // EXT_AXIS_STATUS extAxisStatus[4];  // UDP扩展轴状态 

    msg.ext_di_state_1 = ctrl_state.extDIState[0];        // 扩展DI输入
    msg.ext_di_state_2 = ctrl_state.extDIState[1];
    msg.ext_di_state_3 = ctrl_state.extDIState[2];    
    msg.ext_di_state_4 = ctrl_state.extDIState[3];  
    msg.ext_di_state_5 = ctrl_state.extDIState[4];  
    msg.ext_di_state_6 = ctrl_state.extDIState[5];  
    msg.ext_di_state_7 = ctrl_state.extDIState[6];  
    msg.ext_di_state_8 = ctrl_state.extDIState[7];  
    
    msg.ext_do_state_1 = ctrl_state.extDOState[0];        // 扩展DO输出
    msg.ext_do_state_2 = ctrl_state.extDOState[1];
    msg.ext_do_state_3 = ctrl_state.extDOState[2];    
    msg.ext_do_state_4 = ctrl_state.extDOState[3];  
    msg.ext_do_state_5 = ctrl_state.extDOState[4];  
    msg.ext_do_state_6 = ctrl_state.extDOState[5];  
    msg.ext_do_state_7 = ctrl_state.extDOState[6];  
    msg.ext_do_state_8 = ctrl_state.extDOState[7];  


    msg.ext_ai_state_1 = ctrl_state.extAIState[0];        // 扩展DO输出
    msg.ext_ai_state_2 = ctrl_state.extAIState[1];
    msg.ext_ai_state_3 = ctrl_state.extAIState[2];    
    msg.ext_ai_state_4 = ctrl_state.extAIState[3];  

    msg.ext_ao_state_1 = ctrl_state.extAOState[0];        // 扩展AO输出
    msg.ext_ao_state_2 = ctrl_state.extAOState[1];
    msg.ext_ao_state_3 = ctrl_state.extAOState[2];    
    msg.ext_ao_state_4 = ctrl_state.extAOState[3];  


    msg.rbt_enable_state = ctrl_state.rbtEnableState;            // 机器人使能状态
    // double   jointDriverTorque[6];        //机器人关节驱动器扭矩
    // double   jointDriverTemperature[6];   //机器人关节驱动器温度
    // msg.rbt_time = ctrl_state.robotTime;           // 机器人系统时间
    // int softwareUpgradeState;      // 机器人软件升级状态
    msg.end_lua_err_code = ctrl_state.endLuaErrCode;        // 末端LUA运行状态
    msg.cl_analog_output_1 = ctrl_state.cl_analog_output[0];  // 控制箱模拟量输出
    msg.cl_analog_output_2 = ctrl_state.cl_analog_output[0];
    msg.tl_analog_output = ctrl_state.tl_analog_output;     // 工具模拟量输出
    msg.gripper_rot_num = ctrl_state.gripperRotNum;           // 旋转夹爪当前旋转圈数
    msg.gripper_rot_speed = ctrl_state.gripperRotSpeed;       // 旋转夹爪当前旋转速度百分比
    msg.gripper_rot_torque = ctrl_state.gripperRotTorque;      // 旋转夹爪当前旋转力矩百分比

    msg.weldbreakoffstate = ctrl_state.weldingBreakOffState.breakOffState;//焊接中断状态
    msg.weldarcstate = ctrl_state.weldingBreakOffState.weldArcState;

    // 关节指令力矩
    msg.j1_tgt_tor = ctrl_state.jt_tgt_tor[0];
    msg.j2_tgt_tor = ctrl_state.jt_tgt_tor[1];
    msg.j3_tgt_tor = ctrl_state.jt_tgt_tor[2];
    msg.j4_tgt_tor = ctrl_state.jt_tgt_tor[3];
    msg.j5_tgt_tor = ctrl_state.jt_tgt_tor[4];
    msg.j6_tgt_tor = ctrl_state.jt_tgt_tor[5];


    // int smartToolState;                   // SmartTool手柄按钮状态
    msg.jwide_voltage_ctrl_box_temp = ctrl_state.wideVoltageCtrlBoxTemp;         //宽电压控制箱温度
    msg.wide_voltage_ctrl_box_fan_current = ctrl_state.wideVoltageCtrlBoxFanCurrent;//宽电压控制箱风扇电流(mA)
    
    //工具坐标系
    msg.tool_coord_x = ctrl_state.toolCoord[0];
    msg.tool_coord_y = ctrl_state.toolCoord[1];
    msg.tool_coord_z = ctrl_state.toolCoord[2];
    msg.tool_coord_a = ctrl_state.toolCoord[3];
    msg.tool_coord_b = ctrl_state.toolCoord[4];
    msg.tool_coord_c = ctrl_state.toolCoord[5];

        //工件坐标系
    msg.wobj_coord_x = ctrl_state.wobjCoord[0];
    msg.wobj_coord_y = ctrl_state.wobjCoord[1];
    msg.wobj_coord_z = ctrl_state.wobjCoord[2];
    msg.wobj_coord_a = ctrl_state.wobjCoord[3];
    msg.wobj_coord_b = ctrl_state.wobjCoord[4];
    msg.wobj_coord_c = ctrl_state.wobjCoord[5];

    //外部工具坐标系
    msg.ex_tool_coord_x = ctrl_state.extoolCoord[0];
    msg.ex_tool_coord_y = ctrl_state.extoolCoord[1];
    msg.ex_tool_coord_z = ctrl_state.extoolCoord[2];
    msg.ex_tool_coord_a = ctrl_state.extoolCoord[3];
    msg.ex_tool_coord_b = ctrl_state.extoolCoord[4];
    msg.ex_tool_coord_c = ctrl_state.extoolCoord[5];

    //扩展轴坐标系
    msg.ex_axis_coord_x = ctrl_state.exAxisCoord[0];
    msg.ex_axis_coord_y = ctrl_state.exAxisCoord[1];
    msg.ex_axis_coord_z = ctrl_state.exAxisCoord[2];
    msg.ex_axis_coord_a = ctrl_state.exAxisCoord[3];
    msg.ex_axis_coord_b = ctrl_state.exAxisCoord[4];
    msg.ex_axis_coord_c = ctrl_state.exAxisCoord[5];

    
    msg.load = ctrl_state.load;                //负载质量
    //负载质心
    msg.load_cog_x = ctrl_state.loadCog[0];      
    msg.load_cog_y = ctrl_state.loadCog[1];     
    msg.load_cog_z = ctrl_state.loadCog[2]; 

    //队列中最后一个ServoJ目标位置
    msg.j1_last_servoj_target = ctrl_state.lastServoTarget[0];
    msg.j2_last_servoj_target = ctrl_state.lastServoTarget[1];
    msg.j3_last_servoj_target = ctrl_state.lastServoTarget[2];
    msg.j4_last_servoj_target = ctrl_state.lastServoTarget[3];
    msg.j5_last_servoj_target = ctrl_state.lastServoTarget[4];
    msg.j6_last_servoj_target = ctrl_state.lastServoTarget[5]; 
    msg.servoj_cmd_num = ctrl_state.servoJCmdNum;           //servoJ指令计数


    _state_publisher->publish(msg);
}
