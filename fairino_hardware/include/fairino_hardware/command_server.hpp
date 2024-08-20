#ifndef _COMMAND_SERVER_
#define _COMMAND_SERVER_

#include "stdlib.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "fairino_msgs/srv/remote_script_content.hpp"
#include "fairino_msgs/srv/remote_cmd_interface.hpp"
#include "fairino_msgs/msg/robot_nonrt_state.hpp"
#include "mutex"
#include "sys/socket.h"
#include "sys/types.h"
#include "netinet/in.h"
#include "netinet/tcp.h"
#include "arpa/inet.h"
#include "fcntl.h"
#include <regex>
#include "data_type_def.h"
#include <queue>

using remote_cmd_server_srv_msg = fairino_msgs::srv::RemoteCmdInterface;
using remote_script_srv_msg = fairino_msgs::srv::RemoteScriptContent;
using robot_feedback_msg = fairino_msgs::msg::RobotNonrtState;
#define REMOTE_CMD_SERVER_NAME  "fairino_remote_command_service"
#define REMOTE_SCRIPT_SERVER_NAME  "fairino_script_service"

class FRAPI_base{
public:
    FRAPI_base();
    virtual ~FRAPI_base();
    

protected:
    std::string command_factry(std::string name, uint16_t counter, std::string data);
    std::string _cmd_data;
    std::string _cmd_head;
    std::string _cmd_tail;
    std::string _cmd_interval;
private:
    const std::map<std::string,int> _fr_command_id{
    {"DragTeachSwitch",333},
    {"RobotEnable",632},
    {"Mode",303},
    {"SetSpeed",206},
    {"SetToolCoord",316},
    {"SetToolList",319},
    {"SetExToolCoord",330},
    {"SetExToolList",331},
    {"SetWObjCoord",251},
    {"SetWObjList",383},
    {"SetLoadWeight",306},
    {"SetLoadCoord",307},
    {"SetRobotInstallPos",337},
    {"SetRobotInstallAngle",631},//通用设置
    {"SetAnticollision",305},
    {"SetCollisionStrategy",569},
    {"SetLimitPositive",308},
    {"SetLimitNegative",309},
    {"ResetAllError",107},
    {"FrictionCompensationOnOff",338},
    {"SetFrictionValue_level",541},
    {"SetFrictionValue_wall",542},
    {"SetFrictionValue_ceiling",543},
    {"SetFrictionValue_freedom",637},//安全设置
    {"ActGripper",227},
    {"MoveGripper",228},//外设控制
    {"SetDO",204},
    {"SetToolDO",210},
    {"SetAO",209},
    {"SetToolAO",211},
    {"SetAuxDO",667},
    {"SetAuxAO",668},//IO控制
    {"ExtAxisLoadModbusTCPDriver",655},
    {"ExtAxisServoOn",296},
    {"ExtAxisStartJog",292},
    {"ExtAxisSetHoming",290},
    {"StopExtAxisJog",240},//外部轴控制
    {"StartJOG",232},
    {"StopJOG",233},
    {"StopLine",234},
    {"StopTool",235},
    {"ImmStopJOG",242},
    {"MoveJ",201},
    {"MoveL",203},
    {"MoveC",202},
    {"Circle",540},
    {"NewSpiral",577},
    {"SplineStart",346},
    {"SplinePTP",347},
    {"SplineEnd",350},
    {"NewSplineStart",553},
    {"NewSplinePoint",555},
    {"NewSplineEnd",554},
    {"StopMotion",102},
    {"PointsOffsetEnable",718},
    {"PointsOffsetDisable",719},//运动指令
    {"ScriptStart",101},
    {"ScriptStart_return",-1997},
    {"ScriptStop",102},
    {"ScriptPause",103},
    {"ScriptResume",104},//脚本控制
    {"AuxServoSetParam", 780},      /* 扩展轴控制 */
    {"AuxServoEnable", 782},        
    {"AuxServoSetControlMode", 783},
    {"AuxServoSetTargetPos", 784},
    {"AuxServoSetTargetSpeed", 786},
    {"AuxServoSetTargetTorque", 787},
    {"AuxServoHoming", 788},
    {"AuxServoClearError", 789},
    {"AuxServoSetStatusID", 790}
    };

    const std::map<std::string,int> _fr_script_id{
    {"ScriptName",105},
    {"ScriptContent",106}
    };

    const std::map<std::string,int> _fr_get_id{
    {"GetTCPOffset",377},
    {"GetDHCompensation",842},
    {"GetTCPOffseta",377}
    };
};


class robot_command_thread:public FRAPI_base,public rclcpp::Node{
public:
    robot_command_thread(const std::string node_name);
    ~robot_command_thread();
    

    //普通设置类
    int  DragTeachSwitch(std::string para);//拖动示教模式切换
    int  RobotEnable(std::string para);//机械臂使能
    int  Mode(std::string para);//手动模式，自动模式切换
    int  SetSpeed(std::string para);
    int  SetToolCoord(std::string para);
    int  SetToolList(std::string para);
    int  SetExToolCoord(std::string para);
    int  SetExToolList(std::string para);
    int  SetWObjCoord(std::string para);
    int  SetWObjList(std::string para);
    int  SetLoadWeight(std::string para);
    int  SetLoadCoord(std::string para);
    int  SetRobotInstallPos(std::string para);
    int  SetRobotInstallAngle(std::string para);

    //安全配置
    int  SetAnticollision(std::string para);
    int  SetCollisionStrategy(std::string para);
    int  SetLimitPositive(std::string para);
    int  SetLimitNegative(std::string para);
    int  ResetAllError(std::string para);
    int  FrictionCompensationOnOff(std::string para);
    int  SetFrictionValue_level(std::string para);
    int  SetFrictionValue_wall(std::string para);
    int  SetFrictionValue_ceiling(std::string para);
    int  SetFrictionValue_freedom(std::string para);

    //外设控制
    int  ActGripper(std::string para);
    int  MoveGripper(std::string para);
    
    //IO控制
    int  SetDO(std::string para);
    int  SetToolDO(std::string para);
    int  SetAO(std::string para);
    int  SetToolAO(std::string para);
    int  SetAuxDO(std::string para);
    int  SetAuxAO(std::string para);

    //外部轴控制
    int ExtAxisLoadModbusTCPDriver(std::string para);
    int ExtAxisServoOn(std::string para);
    int ExtAxisStartJog(std::string para);
    int ExtAxisSetHoming(std::string para);
    int StopExtAxisJog(std::string para);
    
    //运动指令
    int  StartJOG(std::string para);
    int  StopJOG(std::string para);
    int  StopLine(std::string para);
    int  StopTool(std::string para);
    int  ImmStopJOG(std::string para);
    int  MoveJ(std::string para);
    int  MoveL(std::string para);
    int  MoveC(std::string para);
    int  Circle(std::string para);
    //int  NewSpiral(std::string para);
    int  ServoJ(JointPos *joint_pos, float acc, float vel, float cmdT, float filterT, float gain);
    //int  ServoCart(int mode, DescPose *desc_pose, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain);
    int  SplineStart(std::string para);
    int  SplinePTP(std::string para);
    int  SplineEnd(std::string para);
    int  NewSplineStart(std::string para);
    int  NewSplinePoint(std::string para);
    int  NewSplineEnd(std::string para);
    int  StopMotion(std::string para);
    int  PointsOffsetEnable(std::string para);
    int  PointsOffsetDisable(std::string para);

    //扩展轴控制
    int AuxServoSetParam(std::string para);
    int AuxServoEnable(std::string para);
    int AuxServoSetControlMode(std::string para);
    int AuxServoSetTargetPos(std::string para);
    int AuxServoSetTargetSpeed(std::string para);
    int AuxServoSetTargetTorque(std::string para);
    int AuxServoHoming(std::string para);
    int AuxServoClearError(std::string para);
    int AuxServoSetStatusID(std::string para);

    //脚本控制指令
    int ScriptStart(std::string para);
    int ScriptStart_return(std::string para);
    int ScriptStop(std::string para);
    int ScriptPause(std::string para);
    int ScriptResume(std::string para);

    /**
	 *@brief  获取当前工件坐标系
	 *@param  [in] flag 0-阻塞，1-非阻塞
	 *@param  [out] desc_pos 工件坐标系位姿
	 *@return  错误码
	 */	
    std::string GetTCPOffset(std::string para);

    /**
     *@brief  获取DH参数补偿值
     *@param  [in] 
     *@param  [out] 补偿值[cmpstD1, cmpstA2, cmpstA3, cmpstD4, cmpstD5, cmpstD6]，单位: mm
     *@return  错误码
     */	
    std::string GetDHCompensation(std::string para);


    

    std::string GetTCPOffseta(std::string para);
    
    void write();//用于ros2_control调用servoj的接口，未来还需要支持扭矩指令的输入

private:
    int lose_connect_times = 0;
    int setKeepAlive(int fd, int idle_time, int interval_time, int probe_times);
    int reConnect_tcp(int fd,int port);

    int (robot_command_thread:: *funcP)(std::string para);//函数指针是有作用域的，所以全局函数的指针和类内成员函数的指针定义有很大不同，这里不能用typedef
    int _send_data_factory_callback(std::string data);//模板函数,用于指令字符串数据的发送和反馈接收确认
    int _send_script_data_callback(std::string data);//用于发送程序名称及程序内容
    std::string _send_get_data_factory_callback(std::string data);//模板函数,用于Get指令字符串数据的发送和反馈接收确认
    int _ParseRecvData(std::string str);//反馈值解析函数
    std::string _ParseGetRecvData(std::string str);//Get类指令反馈值解析函数

    void _ParseROSCommandData_callback(const std::shared_ptr<remote_cmd_server_srv_msg::Request> req,
                                             std::shared_ptr<remote_cmd_server_srv_msg::Response> res);//用于解析用户发送的ROS接口指令
    void _ParseROSScript_callback(const std::shared_ptr<remote_script_srv_msg::Request> req,
                                        std::shared_ptr<remote_script_srv_msg::Response> res);//用于解析用户发送的脚本
    int _def_jnt_position(std::string pos);
    int _def_cart_position(std::string pos);
    std::string _get_variable(std::string para_list);
    uint16_t _cmd_counter;//指令数据帧计数器
    uint16_t _script_counter;//脚本数据帧计数器
    std::string _cur_func_name;
    int _cur_id;
    int _recv_data_cmdcount;//接受到的回复信息中指令计数器的值
    int _recv_data_cmdid;//接受到的回复信息中指令id
    int _recv_data_res;//接受到的回复信息中指令反馈结果
    std::string _recv_get_data_res;//接受到的回复信息中指令反馈结果
    std::string _start_recv_res;//接受到的回复信息中指令反馈结果(start特殊情况，返回结果可能不是0和1会存在错误码)
    float _kin_res[6];
    //用于接受用户发送过来的字符串指令的service
    rclcpp::Service<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr _recv_ros_command_server;
    //用于接受用户发送过来的脚本service
    rclcpp::Service<fairino_msgs::srv::RemoteScriptContent>::SharedPtr _recv_ros_script_server;
    int _robot_install;//机械臂安装方式
    std::vector<JointPos> _cmd_jnt_pos_list;//存储关节数据点
    std::vector<DescPose> _cmd_cart_pos_list;//存储笛卡尔数据点
    std::string _controller_ip;
    int port1 = 8080;//TCP命令发送端口
    int port2 = 8082;//脚本传输和自定义数据流发送端口
    int _socketfd1 = -1;
    int _socketfd2 = -1;
    

    //TODO 把cpp文件中的if else结构搜索函数写成map函数，提高搜索效率
    const std::map<std::string,int(robot_command_thread::*)(std::string)> _fr_function_list{
    {"JNTPoint",&robot_command_thread::_def_jnt_position},
    {"CARTPoint",&robot_command_thread::_def_cart_position},
    {"DragTeachSwitch",&robot_command_thread::DragTeachSwitch},
    {"RobotEnable",&robot_command_thread::RobotEnable},
    {"SetSpeed",&robot_command_thread::SetSpeed},
    {"Mode",&robot_command_thread::Mode},
    {"SetToolCoord",&robot_command_thread::SetToolCoord},
    {"SetToolList",&robot_command_thread::SetToolList},
    {"SetExToolCoord",&robot_command_thread::SetExToolCoord},
    {"SetExToolList",&robot_command_thread::SetExToolList},
    {"SetWObjCoord",&robot_command_thread::SetWObjCoord},
    {"SetWObjList",&robot_command_thread::SetWObjList},
    {"SetLoadWeight",&robot_command_thread::SetLoadWeight},
    {"SetLoadCoord",&robot_command_thread::SetLoadCoord},
    {"SetRobotInstallPos",&robot_command_thread::SetRobotInstallPos},
    {"SetRobotInstallAngle",&robot_command_thread::SetRobotInstallAngle},
    {"SetAnticollision",&robot_command_thread::SetAnticollision},
    {"SetCollisionStrategy",&robot_command_thread::SetCollisionStrategy},
    {"SetLimitPositive",&robot_command_thread::SetLimitPositive},
    {"SetLimitNegative",&robot_command_thread::SetLimitNegative},
    {"ResetAllError",&robot_command_thread::ResetAllError},
    {"FrictionCompensationOnOff",&robot_command_thread::FrictionCompensationOnOff},
    {"SetFrictionValue_level",&robot_command_thread::SetFrictionValue_level},
    {"SetFrictionValue_wall",&robot_command_thread::SetFrictionValue_wall},
    {"SetFrictionValue_ceiling",&robot_command_thread::SetFrictionValue_ceiling},
    {"SetFrictionValue_freedom",&robot_command_thread::SetFrictionValue_freedom},
    {"ActGripper",&robot_command_thread::ActGripper},
    {"MoveGripper",&robot_command_thread::MoveGripper},
    {"SetDO",&robot_command_thread::SetDO},
    {"SetToolDO",&robot_command_thread::SetToolDO},
    {"SetAO",&robot_command_thread::SetAO},
    {"SetToolAO",&robot_command_thread::SetToolAO},
    {"SetAuxDO",&robot_command_thread::SetAuxDO},
    {"SetAuxAO",&robot_command_thread::SetAuxAO},
    {"ExtAxisLoadModbusTCPDriver",&robot_command_thread::ExtAxisLoadModbusTCPDriver},
    {"ExtAxisServoOn",&robot_command_thread::ExtAxisServoOn},
    {"ExtAxisStartJog",&robot_command_thread::ExtAxisStartJog},
    {"ExtAxisSetHoming",&robot_command_thread::ExtAxisSetHoming},
    {"StopExtAxisJog",&robot_command_thread::StopExtAxisJog},
    {"StartJOG",&robot_command_thread::StartJOG},
    {"StopJOG",&robot_command_thread::StopJOG},
    {"StopLine",&robot_command_thread::StopLine},
    {"StopTool",&robot_command_thread::StopTool},
    {"ImmStopJOG",&robot_command_thread::ImmStopJOG},
    {"MoveJ",&robot_command_thread::MoveJ},
    {"MoveL",&robot_command_thread::MoveL},
    {"MoveC",&robot_command_thread::MoveC},
    {"Circle",&robot_command_thread::Circle},
    //{"NewSpiral",&robot_command_thread::NewSpiral},
    {"SplineStart",&robot_command_thread::SplineStart},
    {"SplinePTP",&robot_command_thread::SplinePTP},
    {"SplineEnd",&robot_command_thread::SplineEnd},
    {"NewSplineStart",&robot_command_thread::NewSplineStart},
    {"NewSplinePoint",&robot_command_thread::NewSplinePoint},
    {"NewSplineEnd",&robot_command_thread::NewSplineEnd},
    {"StopMotion",&robot_command_thread::StopMotion},
    {"PointsOffsetEnable",&robot_command_thread::PointsOffsetEnable},
    {"PointsOffsetDisable",&robot_command_thread::PointsOffsetDisable},
    {"ScriptStart",&robot_command_thread::ScriptStart},
    {"ScriptStart_return",&robot_command_thread::ScriptStart_return},
    {"ScriptStop",&robot_command_thread::ScriptStop},
    {"ScriptPause",&robot_command_thread::ScriptPause},
    {"ScriptResume",&robot_command_thread::ScriptResume},

    {"AuxServoSetParam", &robot_command_thread::AuxServoSetParam},
    {"AuxServoEnable", &robot_command_thread::AuxServoEnable},
    {"AuxServoSetControlMode", &robot_command_thread::AuxServoSetControlMode},
    {"AuxServoSetTargetPos", &robot_command_thread::AuxServoSetTargetPos},
    {"AuxServoSetTargetSpeed", &robot_command_thread::AuxServoSetTargetSpeed},
    {"AuxServoSetTargetTorque", &robot_command_thread::AuxServoSetTargetTorque},
    {"AuxServoHoming", &robot_command_thread::AuxServoHoming},
    {"AuxServoClearError", &robot_command_thread::AuxServoClearError},
    {"AuxServoSetStatusID", &robot_command_thread::AuxServoSetStatusID}
    };

    const std::map<std::string,std::string(robot_command_thread::*)(std::string)> _fr_get_list{
    {"GetTCPOffset",&robot_command_thread::GetTCPOffset},
    {"GetDHCompensation",&robot_command_thread::GetDHCompensation},    
    {"GetTCPOffseta",&robot_command_thread::GetTCPOffseta}
    };
};


class robot_recv_thread:public rclcpp::Node{//接受非实时和实时反馈数据的节点
public:
    explicit robot_recv_thread(const std::string node_name);
    ~robot_recv_thread();
private:
    int setKeepAlive(int fd, int idle_time, int interval_time, int probe_times);

    int _is_reconnect = 0;              //1 尝试重连中；
    int _robot_recv_exit = 0;           //类即将析构，通知重连线程退出.
    std::thread _reconnect_thread;
    void _try_to_reconnect();

    std::string _controller_ip;
    void _state_recv_callback();
    void _rt_state_recv_callback();
    rclcpp::Publisher<robot_feedback_msg>::SharedPtr _state_publisher;//进程内通信，用于发送状态数据字符串
    rclcpp::TimerBase::SharedPtr _locktimer;
    int port1 = 8083;//非实时状态数据获取端口
    int _socketfd1;
    int port2 = 8081;//状态数据获取端口(数据全)
    int _socketfd2;
};




#endif
