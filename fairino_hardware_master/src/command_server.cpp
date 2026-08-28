#include "fairino_hardware/command_server.hpp"
#include <sys/types.h>
#include <sys/socket.h>
#include "fairino_hardware/version_control.h"
#include "sys/mman.h"

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
    // _controller_ip = CONTROLLER_IP;//控制器默认ip地址
    // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Robot ip:%s",CONTROLLER_IP);

    // //打印输出版本信息及其他前置信息
    // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(hello)]);
    // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),(std::string(msgout[msg_id(ver_package)])+std::string("V%i.%i.%i")).c_str(),\
    //     VERSION_MAJOR,VERSION_MINOR,VERSION_MINOR2);
    // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),(std::string(msgout[msg_id(ver_robot)])+std::string("V%i.%i.%i,")).c_str(),\
    //     VERSION_ROBOT_MARJOR,VERSION_ROBOT_MINOR,VERSION_ROBOT_MINOR2);
    // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),(std::string(msgout[msg_id(build_time)])+std::string("%s,%s")).c_str(),\
    //     __TIME__,__DATE__);

    // //开始初始化
    // int connect_count = 0;
    // _ptr_robot = std::make_unique<FRRobot>();
    // // _ptr_robot->SetReConnectParam(true,86400000,500);
    // _ptr_robot->SetReConnectParam(true,86400000,500);
    // _ptr_robot->LoggerInit(0,"/home/fairino/ros2_ws/fairino_SDK_log/cppsdk.log",5);
    // _ptr_robot->SetLoggerLevel(3);

//     std::vector<RobotState> states = {
//         // RobotState::FrameHead,
//         // RobotState::FrameCnt,
//         // RobotState::DataLen,
//         RobotState::ProgramState,
//         RobotState::RobotState,
//         RobotState::MainCode,
//         RobotState::SubCode,
//         RobotState::RobotMode,
//         RobotState::JointCurPos,
//         RobotState::ToolCurPos,
//         RobotState::FlangeCurPos,
//         RobotState::ActualJointVel,
//         RobotState::ActualJointAcc,
//         RobotState::TargetTCPCmpSpeed,
//         RobotState::TargetTCPSpeed,
//         RobotState::ActualTCPCmpSpeed,
//         RobotState::ActualTCPSpeed,
//         RobotState::ActualJointTorque,
//         RobotState::Tool,
//         RobotState::User,
//         RobotState::ClDgtOutputH,
//         RobotState::ClDgtOutputL,
//         RobotState::TlDgtOutputL,
//         RobotState::ClDgtInputH,
//         RobotState::ClDgtInputL,
//         RobotState::TlDgtInputL,
//         RobotState::ClAnalogInput,
//         RobotState::TlAnalogInput,
//         RobotState::FtSensorRawData,
//         RobotState::FtSensorData,
//         RobotState::FtSensorActive,
//         RobotState::EmergencyStop,
//         RobotState::MotionDone,
//         RobotState::GripperMotiondone,
//         RobotState::McQueueLen,
//         RobotState::CollisionState,
//         RobotState::TrajectoryPnum,
//         RobotState::SafetyStop0State,
//         RobotState::SafetyStop1State,
//         RobotState::GripperFaultId,
//         RobotState::GripperFault,
//         RobotState::GripperActive,
//         RobotState::GripperPosition,
//         RobotState::GripperSpeed,
//         RobotState::GripperCurrent,
//         RobotState::GripperTemp,
//         RobotState::GripperVoltage,
//         RobotState::AuxState,
//         RobotState::ExtAxisStatus,
//         RobotState::ExtDIState,
//         RobotState::ExtDOState,
//         RobotState::ExtAIState,
//         RobotState::ExtAOState,
//         RobotState::RbtEnableState,
//         RobotState::JointDriverTorque,
//         RobotState::JointDriverTemperature,
//         RobotState::RobotTime,
//         RobotState::SoftwareUpgradeState,
//         RobotState::EndLuaErrCode,
//         RobotState::ClAnalogOutput,
//         RobotState::TlAnalogOutput,
//         RobotState::GripperRotNum,
//         RobotState::GripperRotSpeed,
//         RobotState::GripperRotTorque,
//         RobotState::WeldingBreakOffState,
//         RobotState::TargetJointTorque,
//         RobotState::SmartToolState,
//         RobotState::WideVoltageCtrlBoxTemp,
//         RobotState::WideVoltageCtrlBoxFanCurrent,
//         RobotState::ToolCoord,
//         RobotState::WobjCoord,
//         RobotState::ExtoolCoord,
//         RobotState::ExAxisCoord,
//         RobotState::Load,
//         RobotState::LoadCog,
//         RobotState::LastServoTarget,
//         RobotState::ServoJCmdNum,
//         RobotState::TargetJointPos,
//         RobotState::TargetJointVel,
//         RobotState::TargetJointAcc,
//         RobotState::TargetJointCurrent,
//         RobotState::ActualJointCurrent,
//         RobotState::ActualTCPForce,
//         RobotState::TargetTCPPos,
//         RobotState::CollisionLevel,
//         RobotState::SpeedScaleManual,
//         RobotState::SpeedScaleAuto,
//         RobotState::LuaLineNum,
//         RobotState::AbnomalStop,
//         RobotState::CurrentLuaFileName,
//         RobotState::ProgramTotalLine,
//         RobotState::SafetyBoxSingal,
//         RobotState::WeldVoltage,
//         RobotState::WeldCurrent,
//         RobotState::WeldTrackVel,
//         RobotState::TpdException,
//         RobotState::AlarmRebootRobot,
//         RobotState::ModbusMasterConnect,
//         RobotState::ModbusSlaveConnect,
//         RobotState::BtnBoxStopSignal,
//         RobotState::DragAlarm,
//         RobotState::SafetyDoorAlarm,
//         RobotState::SafetyPlaneAlarm,
//         RobotState::MotonAlarm,
//         RobotState::InterfaceAlarm,
//         RobotState::UdpCmdState,
//         RobotState::WeldReadyState,
//         RobotState::AlarmCheckEmergStopBtn,
//         RobotState::TsTmCmdComError,
//         RobotState::TsTmStateComError,
//         RobotState::CtrlBoxError,
//         RobotState::SafetyDataState,
//         RobotState::ForceSensorErrState,
//         RobotState::CtrlOpenLuaErrCode,
//         RobotState::StrangePosFlag,
//         RobotState::Alarm,
//         RobotState::DriverAlarm,
//         RobotState::AliveSlaveNumError,
//         RobotState::SlaveComError,
//         RobotState::CmdPointError,
//         RobotState::IOError,
//         RobotState::GripperError,
//         RobotState::FileError,
//         RobotState::ParaError,
//         RobotState::ExaxisOutLimitError,
//         RobotState::DriverComError,
//         RobotState::DriverError,
//         RobotState::OutSoftLimitError,
//         RobotState::AxleGenComData,
//         RobotState::SocketConnTimeout,
//         RobotState::SocketReadTimeout,
//         RobotState::TsWebStateComErr,
//         RobotState::ExaxisCoordID
//     };
    // int rnt = _ptr_robot->SetRobotRealtimeStateConfig(states, 8);
    // while(connect_count <_connect_retry_SDK){
    //     error_t returncode = _ptr_robot->RPC(_controller_ip.c_str());
    //     if(returncode !=0){
    //         connect_count++;
    //     }else{//正常情况
    //         break;
    //     }
    //     if(connect_count == _connect_retry_SDK){
    //         RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(connect_failed)]);
    //         exit(0);  
    //     }
    // }

    // _locktimer = this->create_wall_timer(100ms,std::bind(&robot_command_thread::_getRobotRTState,this));
    // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),msgout[msg_id(connect_success)]);
    /*********************************************************************************************/
    _state_publisher = this->create_publisher<robot_feedback_msg>("nonrt_state_data",1);
    _locktimer1 = this->create_wall_timer(10ms,std::bind(&robot_command_thread::_state_recv_callback,this));//创建一个定时器任务用于获取非实时状态数据,触发间隔为100ms
    // _locktimer2 = this->create_wall_timer(100ms,std::bind(&robot_command_thread::_ping_recv_callback,this));//创建一个定时器任务用于ping机器人,触发间隔为100ms

    //从站机器人控制
    current_domain_id_ = get_current_domain_id();
    if(current_domain_id_ == 10){
        master_client_ready_ = false;
        init_master_client_1();
        init_master_client_2();

        reconnect_timer_1_ = this->create_wall_timer(
            std::chrono::seconds(10),
            [this]() {
                if (!master_client_ready_1_ && master_client_1_) {
                    if (master_client_1_->wait_for_service(std::chrono::seconds(1))) {
                        master_client_ready_1_ = true;
                        RCLCPP_INFO(this->get_logger(), "域1服务已重新连接");
                    }
                }
            }
        );
        reconnect_timer_2_ = this->create_wall_timer(
            std::chrono::seconds(10),
            [this]() {
                if (!master_client_ready_2_ && master_client_2_) {
                    if (master_client_2_->wait_for_service(std::chrono::seconds(1))) {
                        master_client_ready_2_ = true;
                        RCLCPP_INFO(this->get_logger(), "域2服务已重新连接");
                    }
                }
            }
        );
    }
}


void robot_command_thread::init_master_client_1()
{
    // 保存当前环境变量
    char* old_domain = getenv("ROS_DOMAIN_ID");
    
    // 临时设置 Domain ID 为 0
    setenv("ROS_DOMAIN_ID", "1", 1);
    
    // 创建独立的上下文
    master_context_1_ = std::make_shared<rclcpp::Context>();
    master_context_1_->init(0, nullptr);
    
    // 恢复原来的环境变量
    if (old_domain) {
        setenv("ROS_DOMAIN_ID", old_domain, 1);
    } else {
        unsetenv("ROS_DOMAIN_ID");
    }
    
    // 创建主站Domain的节点
    rclcpp::NodeOptions master_options;
    master_options.context(master_context_1_);
    master_node_1_ = std::make_shared<rclcpp::Node>(
        "master_client_node_1", master_options);
    
    // 创建主站服务的客户端
    master_client_1_ = master_node_1_->create_client<remote_cmd_server_srv_msg>(
        REMOTE_CMD_SERVER_NAME);
    master_sub_1_ = master_node_1_->create_subscription<robot_feedback_msg>(
        "nonrt_state_data",
        10,
        [this](const robot_feedback_msg::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(msg_mutex_1_);
            msg_sub_1_ = msg;  // 互斥锁保护
        }
    );
    
    // 启动独立线程处理主站节点的事件循环
    spin_thread_1_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(master_node_1_);
        executor.spin();
    });
    
    // 等待主站服务可用
    RCLCPP_INFO(this->get_logger(), "主站正在等待域1中的主服务...");
    
    // 添加重试机制
    const int max_retries = 3;
    bool service_found = false;
    
    for (int retry = 0; retry < max_retries && !service_found; retry++) {
        if (master_client_1_->wait_for_service(std::chrono::seconds(3))) {
            service_found = true;
            master_client_ready_1_ = true;
            RCLCPP_INFO(this->get_logger(), "主站已连接到域1中的主服务");
        } else {
            RCLCPP_WARN(this->get_logger(), 
                       "主站等待域1主服务超时 (尝试 %d/%d)", 
                       retry + 1, max_retries);
        }
    }
    
    if (!master_client_ready_1_) {
        RCLCPP_ERROR(this->get_logger(), 
                    "主站无法连接到域1中的主服务");
    }
}
void robot_command_thread::init_master_client_2()
{
    // 保存当前环境变量
    char* old_domain = getenv("ROS_DOMAIN_ID");
    
    // 临时设置 Domain ID 为 0
    setenv("ROS_DOMAIN_ID", "2", 1);
    
    // 创建独立的上下文
    master_context_2_ = std::make_shared<rclcpp::Context>();
    master_context_2_->init(0, nullptr);
    
    // 恢复原来的环境变量
    if (old_domain) {
        setenv("ROS_DOMAIN_ID", old_domain, 1);
    } else {
        unsetenv("ROS_DOMAIN_ID");
    }
    
    // 创建主站Domain的节点
    rclcpp::NodeOptions master_options;
    master_options.context(master_context_2_);
    master_node_2_ = std::make_shared<rclcpp::Node>(
        "master_client_node_2", master_options);
    
    // 创建主站服务的客户端
    master_client_2_ = master_node_2_->create_client<remote_cmd_server_srv_msg>(
        REMOTE_CMD_SERVER_NAME);
    master_sub_2_ = master_node_2_->create_subscription<robot_feedback_msg>(
        "nonrt_state_data",
        10,
        [this](const robot_feedback_msg::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(msg_mutex_2_);
            msg_sub_2_ = msg;  // 互斥锁保护
        }
    );
    
    // 启动独立线程处理主站节点的事件循环
    spin_thread_2_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(master_node_2_);
        executor.spin();
    });
    
    // 等待主站服务可用
    RCLCPP_INFO(this->get_logger(), "主站正在等待域2中的主服务...");
    
    // 添加重试机制
    const int max_retries = 3;
    bool service_found = false;
    
    for (int retry = 0; retry < max_retries && !service_found; retry++) {
        if (master_client_2_->wait_for_service(std::chrono::seconds(3))) {
            service_found = true;
            master_client_ready_2_ = true;
            RCLCPP_INFO(this->get_logger(), "主站已连接到域2中的主服务");
        } else {
            RCLCPP_WARN(this->get_logger(), 
                       "主站等待域2主服务超时 (尝试 %d/%d)", 
                       retry + 1, max_retries);
        }
    }
    
    if (!master_client_ready_2_) {
        RCLCPP_ERROR(this->get_logger(), 
                    "主站无法连接到域2中的主服务");
    }
}

int robot_command_thread::get_current_domain_id()
{
    RCLCPP_INFO(this->get_logger(), 
                "===========111========");
    const char* domain_env = std::getenv("ROS_DOMAIN_ID");
    if (domain_env != nullptr) {
        RCLCPP_INFO(this->get_logger(), 
                    "ROS_DOMAIN_ID:%d",std::stoi(domain_env));
        return std::stoi(domain_env);
    }
    return 10;  // 默认 Domain ID 是 0
}

/**
 * @brief 析构函数，关闭xmlrpc连接并销毁SDK实例对象
 */
robot_command_thread::~robot_command_thread()
{
    //_ptr_robot->CloseRPC();
    // _ptr_robot->~FRRobot();
    if(current_domain_id_ == 10){
        master_node_1_.reset();
        if (master_context_1_) {
            master_context_1_->shutdown("Destructor called");
        }
        if (spin_thread_1_.joinable()) {
            spin_thread_1_.join();
        }
        master_node_2_.reset();
        if (master_context_2_) {
            master_context_2_->shutdown("Destructor called");
        }
        if (spin_thread_2_.joinable()) {
            spin_thread_2_.join();
        }
    }
}



/**
 * @brief 私有函数，service的回调函数，用于解析字符串指令，跳转对应的处理函数
 */
void robot_command_thread::_parseROSCommandData_callback(
        const std::shared_ptr<remote_cmd_server_srv_msg::Request> req,\ 
        std::shared_ptr<remote_cmd_server_srv_msg::Response> res){
    //指令格式为movj(1,10)
    std::regex func_reg("([A-Z|a-z|_|0-9]+)[(](.*)[)]");//函数名的输入模式应该是字母或者数字函数名后跟(),圆括号中有所有输入参数
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
 * @brief 获取fairino_hardware功能包版本号
 * @return 错误码及版本号
 * @retval 0,version
 */
std::string robot_command_thread::GetVersion(std::string para){
    std::string ver = "fairino_hardware:V0.0.0";
    return std::string("0," + ver);
}

/**
 * @brief 获取fairino_msgs版本号
 * @return 错误码及版本号
 * @retval 0,version
 */
std::string robot_command_thread::GetMsgVersion(std::string para){
    std::string ver = "fairino_msgs:V0.0.0";
    return std::string("0," + ver);
}
/**
 * @brief 获取机械臂版本号
 * @return 错误码及版本号
 * @retval res,softwareversion
 */
std::string robot_command_thread::GetRobotVersion(std::string para){
    return std::string("0,robot:v0.0.0");
}

/**
 * @brief 获取机械臂控制器版本号
 * @return 错误码及版本号
 * @retval res,sofewareversion
 */
std::string robot_command_thread::GetControllerVersion(std::string para){
    return std::string("0,robot_controller:v0.0.0");
}

/**
 * @brief 从站机器人控制
 * @param [in]
 * @return 错误码
 */
std::string robot_command_thread::Slave(std::string para){
    if(current_domain_id_ == 10){
        size_t pos = para.find(';');
        int DOMAIN_ID = 0;
        std::string cmd_str;
        if (pos != std::string::npos) {
            // DOMAIN_ID
            std::string DOMAIN_Str = para.substr(0, pos);
            DOMAIN_ID = std::stoi(DOMAIN_Str);
            cmd_str = para.substr(pos + 1);
        }
        // DOMAIN_ID输入异常数据处理    
        if (DOMAIN_ID > 2 || DOMAIN_ID < 1){
            RCLCPP_ERROR(this->get_logger(), "从站请求DOMAIN_ID错误");
            return std::to_string(-2004);
        }
        // 指令字符输入异常数据处理    
        if (cmd_str.empty()){
            RCLCPP_ERROR(this->get_logger(), "指令字符输入错误");
            return std::to_string(-2005);
        }

        if(DOMAIN_ID == 1){
            master_client_ready_ = master_client_ready_1_.load();
            master_client_ = master_client_1_;
        }else if(DOMAIN_ID == 2){
            master_client_ready_ = master_client_ready_2_.load();
            master_client_ = master_client_2_;
        }

        if (!master_client_ready_ || !master_client_->service_is_ready()) {
            RCLCPP_ERROR(this->get_logger(), "域%d的客户端未就绪",DOMAIN_ID);
            return std::to_string(-2007);
        }
        
        auto request = std::make_shared<remote_cmd_server_srv_msg::Request>();
        request->cmd_str = cmd_str;
        
        // 异步调用
        auto future = master_client_->async_send_request(request);
        
        // 等待响应（设置超时）
        auto status = future.wait_for(std::chrono::seconds(5));
        if (status == std::future_status::ready) {
            auto response = future.get();
            return response->cmd_res;
        } else {
            RCLCPP_ERROR(this->get_logger(), "域%d服务响应超时",DOMAIN_ID);
            return std::to_string(-2003);
        }
    }else{
        return std::to_string(-2010);
    }
}
/**
 * @brief 从站机器人控制
 * @param [in] 
 * @return 错误码
 */
std::string robot_command_thread::FeedBackIDSet(std::string para){
    if(current_domain_id_ == 10){
        std::list<std::string> list;
        _splitString2List(para,list);
        int DOMAIN_ID = std::stoi(list.front());list.pop_front();
        if(DOMAIN_ID < 0 || DOMAIN_ID > 2){
            return std::to_string(-2001);
        }
        if(DOMAIN_ID == 1 && !master_client_1_->service_is_ready()){
            return std::to_string(-2007);
        }
        if(DOMAIN_ID == 2 && !master_client_2_->service_is_ready()){
            return std::to_string(-2007);
        }
        sub_domain_id = DOMAIN_ID;

        return std::to_string(0);
    }else{
        return std::to_string(-2010);
    }
}







/**
 * @brief 数据端口topic监听回调函数
*/
void robot_command_thread::_state_recv_callback(){
    int current_domain = sub_domain_id.load();
    if(current_domain == 1){
        if(master_client_1_->service_is_ready()){
            std::lock_guard<std::mutex> lock(msg_mutex_1_);
            if (msg_sub_1_) {  // 检查是否有效
                msg_sub_1_->slave_status_1 = master_client_1_->service_is_ready();
                msg_sub_1_->slave_status_2 = master_client_2_->service_is_ready();
                msg_sub_1_->slave_domain_id = current_domain;
                if(master_client_1_->service_is_ready()){
                    if (msg_sub_1_) { 
                        msg_sub_1_->slave1_reconnect_flag = msg_sub_1_->reconnect_flag;
                        msg_sub_1_->slave1_exaxistatus_errorcode = msg_sub_1_->exaxistatus1[0];
                    }else{
                        msg_sub_1_->slave1_reconnect_flag = 1;
                        msg_sub_1_->slave1_exaxistatus_errorcode = 1;
                    }
                }else{
                    msg_sub_1_->slave1_reconnect_flag = 1;
                    msg_sub_1_->slave1_exaxistatus_errorcode = 1;
                }
                if(master_client_2_->service_is_ready()){
                    if (msg_sub_2_) { 
                        msg_sub_1_->slave2_reconnect_flag = msg_sub_2_->reconnect_flag;
                        msg_sub_1_->slave2_exaxistatus_errorcode = msg_sub_2_->exaxistatus1[0];
                    }else{
                        msg_sub_1_->slave2_reconnect_flag = 1;
                        msg_sub_1_->slave2_exaxistatus_errorcode = 1;
                    }
                }else{
                    msg_sub_1_->slave2_reconnect_flag = 1;
                    msg_sub_1_->slave2_exaxistatus_errorcode = 1;
                }
                _state_publisher->publish(*msg_sub_1_);
            }else{
                auto msg = robot_feedback_msg();
                msg.reconnect_flag = 1;
                msg.slave_status_1 = master_client_1_->service_is_ready();
                msg.slave_status_2 = master_client_2_->service_is_ready();
                msg.slave_domain_id = current_domain;
                if(master_client_1_->service_is_ready()){
                    if (msg_sub_1_) { 
                        msg.slave1_reconnect_flag = msg_sub_1_->reconnect_flag;
                        msg.slave1_exaxistatus_errorcode = msg_sub_1_->exaxistatus1[0];
                    }else{
                        msg.slave1_reconnect_flag = 1;
                        msg.slave1_exaxistatus_errorcode = 1;
                    }
                }else{
                    msg.slave1_reconnect_flag = 1;
                    msg.slave1_exaxistatus_errorcode = 1;
                }
                if(master_client_2_->service_is_ready()){
                    if (msg_sub_2_) { 
                        msg.slave2_reconnect_flag = msg_sub_2_->reconnect_flag;
                        msg.slave2_exaxistatus_errorcode = msg_sub_2_->exaxistatus1[0];
                    }else{
                        msg.slave2_reconnect_flag = 1;
                        msg.slave2_exaxistatus_errorcode = 1;
                    }
                }else{
                    msg.slave2_reconnect_flag = 1;
                    msg.slave2_exaxistatus_errorcode = 1;
                }
                _state_publisher->publish(msg);
            }
        }else{
            auto msg = robot_feedback_msg();
            msg.reconnect_flag = 1;
            msg.slave_status_1 = master_client_1_->service_is_ready();
            msg.slave_status_2 = master_client_2_->service_is_ready();
            msg.slave_domain_id = current_domain;
            if(master_client_1_->service_is_ready()){
                if (msg_sub_1_) { 
                    msg.slave1_reconnect_flag = msg_sub_1_->reconnect_flag;
                    msg.slave1_exaxistatus_errorcode = msg_sub_1_->exaxistatus1[0];
                }else{
                    msg.slave1_reconnect_flag = 1;
                    msg.slave1_exaxistatus_errorcode = 1;
                }
            }else{
                msg.slave1_reconnect_flag = 1;
                msg.slave1_exaxistatus_errorcode = 1;
            }
            if(master_client_2_->service_is_ready()){
                if (msg_sub_2_) { 
                    msg.slave2_reconnect_flag = msg_sub_2_->reconnect_flag;
                    msg.slave2_exaxistatus_errorcode = msg_sub_2_->exaxistatus1[0];
                }else{
                    msg.slave2_reconnect_flag = 1;
                    msg.slave2_exaxistatus_errorcode = 1;
                }
            }else{
                msg.slave2_reconnect_flag = 1;
                msg.slave2_exaxistatus_errorcode = 1;
            }
            _state_publisher->publish(msg);
        }
    }else if(current_domain == 2){
        if(master_client_2_->service_is_ready()){
            std::lock_guard<std::mutex> lock(msg_mutex_2_);
            if (msg_sub_2_) {  // 检查是否有效
                msg_sub_2_->slave_status_1 = master_client_1_->service_is_ready();
                msg_sub_2_->slave_status_2 = master_client_2_->service_is_ready();
                msg_sub_2_->slave_domain_id = current_domain;
                if(master_client_1_->service_is_ready()){
                    if (msg_sub_1_) { 
                        msg_sub_2_->slave1_reconnect_flag = msg_sub_1_->reconnect_flag;
                        msg_sub_2_->slave1_exaxistatus_errorcode = msg_sub_1_->exaxistatus1[0];
                    }else{
                        msg_sub_2_->slave1_reconnect_flag = 1;
                        msg_sub_2_->slave1_exaxistatus_errorcode = 1;
                    }
                }else{
                    msg_sub_2_->slave1_reconnect_flag = 1;
                    msg_sub_2_->slave1_exaxistatus_errorcode = 1;
                }
                if(master_client_2_->service_is_ready()){
                    if (msg_sub_2_) { 
                        msg_sub_2_->slave2_reconnect_flag = msg_sub_2_->reconnect_flag;
                        msg_sub_2_->slave2_exaxistatus_errorcode = msg_sub_2_->exaxistatus1[0];
                        }else{
                        msg_sub_2_->slave2_reconnect_flag = 1;
                        msg_sub_2_->slave2_exaxistatus_errorcode = 1;
                    }
                }else{
                    msg_sub_2_->slave2_reconnect_flag = 1;
                    msg_sub_2_->slave2_exaxistatus_errorcode = 1;
                }
                _state_publisher->publish(*msg_sub_2_);
            }else{
                auto msg = robot_feedback_msg();
                msg.reconnect_flag = 1;
                msg.slave_status_1 = master_client_1_->service_is_ready();
                msg.slave_status_2 = master_client_2_->service_is_ready();
                msg.slave_domain_id = current_domain;
                if(master_client_1_->service_is_ready()){
                    if (msg_sub_1_) { 
                        msg.slave1_reconnect_flag = msg_sub_1_->reconnect_flag;
                        msg.slave1_exaxistatus_errorcode = msg_sub_1_->exaxistatus1[0];
                    }else{
                        msg.slave1_reconnect_flag = 1;
                        msg.slave1_exaxistatus_errorcode = 1;
                    }
                }else{
                    msg.slave1_reconnect_flag = 1;
                    msg.slave1_exaxistatus_errorcode = 1;
                }
                if(master_client_2_->service_is_ready()){
                    if (msg_sub_2_) { 
                        msg.slave2_reconnect_flag = msg_sub_2_->reconnect_flag;
                        msg.slave2_exaxistatus_errorcode = msg_sub_2_->exaxistatus1[0];
                    }else{
                        msg.slave2_reconnect_flag = 1;
                        msg.slave2_exaxistatus_errorcode = 1;
                    }
                }else{
                    msg.slave2_reconnect_flag = 1;
                    msg.slave2_exaxistatus_errorcode = 1;
                }
                _state_publisher->publish(msg);
            }
        }else{
            auto msg = robot_feedback_msg();
            msg.reconnect_flag = 1;
            msg.slave_status_1 = master_client_1_->service_is_ready();
            msg.slave_status_2 = master_client_2_->service_is_ready();
            msg.slave_domain_id = current_domain;
            if(master_client_1_->service_is_ready()){
                if (msg_sub_1_) { 
                    msg.slave1_reconnect_flag = msg_sub_1_->reconnect_flag;
                    msg.slave1_exaxistatus_errorcode = msg_sub_1_->exaxistatus1[0];
                }else{
                    msg.slave1_reconnect_flag = 1;
                    msg.slave1_exaxistatus_errorcode = 1;
                }
            }else{
                msg.slave1_reconnect_flag = 1;
                msg.slave1_exaxistatus_errorcode = 1;
            }
            if(master_client_2_->service_is_ready()){
                if (msg_sub_2_) { 
                    msg.slave2_reconnect_flag = msg_sub_2_->reconnect_flag;
                    msg.slave2_exaxistatus_errorcode = msg_sub_2_->exaxistatus1[0];
                }else{
                    msg.slave2_reconnect_flag = 1;
                    msg.slave2_exaxistatus_errorcode = 1;
                }
            }else{
                msg.slave2_reconnect_flag = 1;
                msg.slave2_exaxistatus_errorcode = 1;
            }
            _state_publisher->publish(msg);
        }
    }else{
        auto msg = robot_feedback_msg();
        msg.reconnect_flag = 1;
        msg.slave_status_1 = master_client_1_->service_is_ready();
        msg.slave_status_2 = master_client_2_->service_is_ready();
        msg.slave_domain_id = current_domain;
        if(master_client_1_->service_is_ready()){
            if (msg_sub_1_) { 
                msg.slave1_reconnect_flag = msg_sub_1_->reconnect_flag;
                msg.slave1_exaxistatus_errorcode = msg_sub_1_->exaxistatus1[0];
            }else{
                msg.slave1_reconnect_flag = 1;
                msg.slave1_exaxistatus_errorcode = 1;
            }
        }else{
            msg.slave1_reconnect_flag = 1;
            msg.slave1_exaxistatus_errorcode = 1;
        }
        if(master_client_2_->service_is_ready()){
            if (msg_sub_2_) { 
                msg.slave2_reconnect_flag = msg_sub_2_->reconnect_flag;
                msg.slave2_exaxistatus_errorcode = msg_sub_2_->exaxistatus1[0];
            }else{
                msg.slave2_reconnect_flag = 1;
                msg.slave2_exaxistatus_errorcode = 1;
            }
        }else{
            msg.slave2_reconnect_flag = 1;
            msg.slave2_exaxistatus_errorcode = 1;
        }
        _state_publisher->publish(msg);
    }
}

/**
 * @brief 数据端口topic监听回调函数
*/
void robot_command_thread::_ping_recv_callback(){
    int current_domain = sub_domain_id.load();
    if(current_domain == 0){
        auto msg = robot_feedback_msg();
        bool current_ping_status = _check_ping("192.168.58.2");
        if(!current_ping_status){
            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"192.168.58.2 ping failed");
            msg.reconnect_flag = 1;
            _state_publisher->publish(msg);

        }
    }
}

bool robot_command_thread::_check_ping(const std::string& ip_address) {
    std::string cmd = "ping -c 1 -W 1 " + ip_address + " > /dev/null 2>&1";
    int result = system(cmd.c_str());
    return (result == 0);
}