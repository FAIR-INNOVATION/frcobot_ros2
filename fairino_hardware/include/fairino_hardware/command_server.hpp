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
#include "libfairino/include/robot.h"
#include <atomic>

using remote_cmd_server_srv_msg = fairino_msgs::srv::RemoteCmdInterface;
using remote_script_srv_msg = fairino_msgs::srv::RemoteScriptContent;
using robot_feedback_msg = fairino_msgs::msg::RobotNonrtState;
#define REMOTE_CMD_SERVER_NAME  "fairino_remote_command_service"
#define REMOTE_SCRIPT_SERVER_NAME  "fairino_script_service"


class robot_command_thread:public rclcpp::Node{
public:
    robot_command_thread(const std::string node_name);
    ~robot_command_thread();
    
    //点位信息设置类
    std::string defJntPosition(std::string pos);
    std::string defCartPosition(std::string pos);
    std::string getVariable(std::string para_list);

    //信息获取类
    std::string GetVersion(std::string para);
    std::string GetMsgVersion(std::string para);
    std::string GetRobotVersion(std::string para);
    std::string GetControllerVersion(std::string para);
    std::string GetTCPOffset(std::string para);
    std::string GetDHCompensation(std::string para);
    std::string GetWeldingBreakOffState(std::string para);
    std::string GetErrorCode(std::string para);
    std::string GetInverseKin(std::string para);

    //普通设置类
    std::string DragTeachSwitch(std::string para);//拖动示教模式切换
    std::string RobotEnable(std::string para);//机械臂使能
    std::string Mode(std::string para);//手动模式，自动模式切换
    std::string SetSpeed(std::string para);
    std::string SetToolCoord(std::string para);
    std::string SetToolList(std::string para);
    std::string SetExToolCoord(std::string para);
    std::string SetExToolList(std::string para);
    std::string SetWObjCoord(std::string para);
    std::string SetWObjList(std::string para);
    std::string SetLoadWeight(std::string para);
    std::string SetLoadCoord(std::string para);
    std::string SetRobotInstallPos(std::string para);
    std::string SetRobotInstallAngle(std::string para);

    //安全配置
    std::string SetAnticollision(std::string para);
    std::string SetCollisionStrategy(std::string para);
    std::string SetLimitPositive(std::string para);
    std::string SetLimitNegtive(std::string para);
    std::string ResetAllError(std::string para);
    std::string FrictionCompensationOnOff(std::string para);
    std::string SetFrictionValue_level(std::string para);
    std::string SetFrictionValue_wall(std::string para);
    std::string SetFrictionValue_ceiling(std::string para);
    std::string SetFrictionValue_freedom(std::string para);

    //外设控制
    std::string ActGripper(std::string para);
    std::string MoveGripper(std::string para);
    
    //IO控制
    std::string SetDO(std::string para);
    std::string SetToolDO(std::string para);
    std::string SetAO(std::string para);
    std::string SetToolAO(std::string para);
    std::string SetAuxDO(std::string para);
    std::string SetAuxAO(std::string para);

    //UDP扩展轴控制
    std::string ExtAxisServoOn(std::string para);
    std::string ExtAxisStartJog(std::string para);
    std::string ExtAxisSetHoming(std::string para);
    std::string StopExtAxisJog(std::string para);
    std::string ExtAxisGetCoord(std::string para);
    std::string ExtAxisMove(std::string para);
    std::string ExtAxisSyncMoveJ(std::string para);
    
    //运动指令
    std::string StartJOG(std::string para);
    std::string StopJOG(std::string para);
    std::string ImmStopJOG(std::string para);
    std::string MoveJ(std::string para);
    std::string MoveL(std::string para);
    std::string MoveC(std::string para);
    std::string Circle(std::string para);
    std::string ServoJ(std::string para);
    std::string SplineStart(std::string para);
    std::string SplinePTP(std::string para);
    std::string SplineEnd(std::string para);
    std::string NewSplineStart(std::string para);
    std::string NewSplinePoint(std::string para);
    std::string NewSplineEnd(std::string para);
    std::string StopMotion(std::string para);
    std::string PointsOffsetEnable(std::string para);
    std::string PointsOffsetDisable(std::string para);

    //485扩展轴控制
    std::string AuxServoSetParam(std::string para);
    std::string AuxServoEnable(std::string para);
    std::string AuxServoSetControlMode(std::string para);
    std::string AuxServoSetTargetPos(std::string para);
    std::string AuxServoSetTargetSpeed(std::string para);
    std::string AuxServoSetTargetTorque(std::string para);
    std::string AuxServoHoming(std::string para);
    std::string AuxServoClearError(std::string para);
    std::string AuxServoSetStatusID(std::string para);

    //脚本控制指令
    std::string ScriptLoad(std::string para);
    std::string ScriptStart(std::string para);
    std::string ScriptStop(std::string para);
    std::string ScriptPause(std::string para);
    std::string ScriptResume(std::string para);

    //可移动装置
    std::string TractorEnable(std::string para);
    std::string TractorHoming(std::string para);
    std::string TractorMoveL(std::string para);
    std::string TractorMoveC(std::string para);
    std::string TractorStop(std::string para);
    
    //轨迹J功能
    std::string TrajectoryJUpLoad(std::string para);
    std::string TrajectoryJDelete(std::string para);
    std::string LoadTrajectoryJ(std::string para);
    std::string MoveTrajectoryJ(std::string para);
    std::string GetTrajectoryStartPose(std::string para);
    std::string GetTrajectoryPointNum(std::string para);
    std::string SetTrajectoryJSpeed(std::string para);

    //LUA脚本传输功能
    std::string LuaDownLoad(std::string para);
    std::string LuaUpload(std::string para);
    std::string LuaDelete(std::string para);
    std::string GetLuaList(std::string para);

    //点位换算功能
    std::string ComputeToolCoordWithPoints(std::string para);
    std::string ComputeWObjCoordWithPoints(std::string para);

    //焊接中断恢复功能
    std::string WeldingSetCheckArcInterruptionParam(std::string para);
    std::string WeldingGetCheckArcInterruptionParam(std::string para);
    std::string WeldingSetReWeldAfterBreakOffParam(std::string para);
    std::string WeldingGetReWeldAfterBreakOffParam(std::string para);
    std::string WeldingStartReWeldAfterBreakOff(std::string para);
    std::string WeldingAbortWeldAfterBreakOff(std::string para);

private:
    std::unique_ptr<FRRobot> _ptr_robot;//机械臂SDK库指针
    ROBOT_STATE_PKG _robot_realtime_state;//从SDK获取的机械臂实时状态结构体
    rclcpp::TimerBase::SharedPtr _locktimer;

    int lose_connect_times = 0;
    int _connect_retry_SDK = 5;
    //函数指针是有作用域的，所以全局函数的指针和类内成员函数的指针定义有很大不同，这里不能用typedef
    int (robot_command_thread:: *funcP)(std::string para);

    //用于解析用户发送的ROS接口指令
    void _parseROSCommandData_callback(const std::shared_ptr<remote_cmd_server_srv_msg::Request> req,
                                    std::shared_ptr<remote_cmd_server_srv_msg::Response> res);
    void _timer_callback();
    void _splitString2List(std::string str,std::list<std::string> &list_data);
    void _splitString2Vec(std::string str,std::vector<std::string> &vector_data);
    void _fillDescPose(std::list<std::string>& data,DescPose& pose);
    void _fillDescTran(std::list<std::string>& data,DescTran& trans);
    void _fillJointPose(std::list<std::string>& data,JointPos& pos);
    void _getRobotRTState();
    //TODO 使用可变参数模板函数去填装SDK函数所需参数
    // template<typename T,typename ... Ts>
    // void _recurseVar(T& first_arg,Ts&... args);

    std::string _cur_func_name;
    int _cur_id;
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
    const std::map<std::string,std::string(robot_command_thread::*)(std::string)> _fr_function_list{
    {"JNTPoint",&robot_command_thread::defJntPosition},
    {"CARTPoint",&robot_command_thread::defCartPosition},
    {"GET",&robot_command_thread::getVariable},
    {"GetVersion",&robot_command_thread::GetVersion},
    {"GetMsgVersion",&robot_command_thread::GetMsgVersion},
    {"GetRobotVersion",&robot_command_thread::GetRobotVersion},
    {"GetControllerVersion",&robot_command_thread::GetControllerVersion},
    {"GetWeldingBreakOffState",&robot_command_thread::GetWeldingBreakOffState},
    {"GetErrorCode",&robot_command_thread::GetErrorCode},
    {"GetInverseKin",&robot_command_thread::GetInverseKin},
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
    {"SetLimitNegtive",&robot_command_thread::SetLimitNegtive},
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
    {"ExtAxisServoOn",&robot_command_thread::ExtAxisServoOn},
    {"ExtAxisStartJog",&robot_command_thread::ExtAxisStartJog},
    {"ExtAxisSetHoming",&robot_command_thread::ExtAxisSetHoming},
    {"ExtAxisSyncMoveJ",&robot_command_thread::ExtAxisSyncMoveJ},
    {"StopExtAxisJog",&robot_command_thread::StopExtAxisJog},
    {"ExtAxisGetCoord",&robot_command_thread::ExtAxisGetCoord},
    {"ExtAxisMove",&robot_command_thread::ExtAxisMove},
    {"StartJOG",&robot_command_thread::StartJOG},
    {"StopJOG",&robot_command_thread::StopJOG},
    {"ImmStopJOG",&robot_command_thread::ImmStopJOG},
    {"MoveJ",&robot_command_thread::MoveJ},
    {"MoveL",&robot_command_thread::MoveL},
    {"MoveC",&robot_command_thread::MoveC},
    {"Circle",&robot_command_thread::Circle},
    {"ServoJ",&robot_command_thread::ServoJ},
    {"SplineStart",&robot_command_thread::SplineStart},
    {"SplinePTP",&robot_command_thread::SplinePTP},
    {"SplineEnd",&robot_command_thread::SplineEnd},
    {"NewSplineStart",&robot_command_thread::NewSplineStart},
    {"NewSplinePoint",&robot_command_thread::NewSplinePoint},
    {"NewSplineEnd",&robot_command_thread::NewSplineEnd},
    {"StopMotion",&robot_command_thread::StopMotion},
    {"PointsOffsetEnable",&robot_command_thread::PointsOffsetEnable},
    {"PointsOffsetDisable",&robot_command_thread::PointsOffsetDisable},
    {"ScriptLoad",&robot_command_thread::ScriptLoad},
    {"ScriptStart",&robot_command_thread::ScriptStart},
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
    {"AuxServoSetStatusID", &robot_command_thread::AuxServoSetStatusID},
    {"GetTCPOffset",&robot_command_thread::GetTCPOffset},
    {"GetDHCompensation",&robot_command_thread::GetDHCompensation}, 
    {"TractorEnable",&robot_command_thread::TractorEnable},
    {"TractorHoming",&robot_command_thread::TractorHoming},
    {"TractorMoveL",&robot_command_thread::TractorMoveL},
    {"TractorMoveC",&robot_command_thread::TractorMoveC},
    {"TractorStop",&robot_command_thread::TractorStop},
    {"TrajectoryJUpLoad",&robot_command_thread::TrajectoryJUpLoad},
    {"TrajectoryJDelete",&robot_command_thread::TrajectoryJDelete},
    {"LoadTrajectoryJ",&robot_command_thread::LoadTrajectoryJ},
    {"MoveTrajectoryJ",&robot_command_thread::MoveTrajectoryJ},
    {"GetTrajectoryStartPose",&robot_command_thread::GetTrajectoryStartPose},
    {"GetTrajectoryPointNum",&robot_command_thread::GetTrajectoryPointNum},
    {"SetTrajectoryJSpeed",&robot_command_thread::SetTrajectoryJSpeed},
    {"LuaDownLoad",&robot_command_thread::LuaDownLoad},
    {"LuaUpload",&robot_command_thread::LuaUpload},
    {"LuaDelete",&robot_command_thread::LuaDelete},
    {"GetLuaList",&robot_command_thread::GetLuaList},
    {"ComputeToolCoordWithPoints",&robot_command_thread::ComputeToolCoordWithPoints},
    {"ComputeWObjCoordWithPoints",&robot_command_thread::ComputeWObjCoordWithPoints},
    {"WeldingSetCheckArcInterruptionParam",&robot_command_thread::WeldingSetCheckArcInterruptionParam},
    {"WeldingGetCheckArcInterruptionParam",&robot_command_thread::WeldingGetCheckArcInterruptionParam},
    {"WeldingSetReWeldAfterBreakOffParam",&robot_command_thread::WeldingSetReWeldAfterBreakOffParam},
    {"WeldingGetReWeldAfterBreakOffParam",&robot_command_thread::WeldingGetReWeldAfterBreakOffParam},
    {"WeldingStartReWeldAfterBreakOff",&robot_command_thread::WeldingStartReWeldAfterBreakOff},
    {"WeldingAbortWeldAfterBreakOff",&robot_command_thread::WeldingAbortWeldAfterBreakOff}
    };
};

/**
 * @class robot_recv_thread
 * @brief 接收控制器8081端口发送的TCP状态数据结构体并发布成topic消息
 */
class robot_recv_thread:public rclcpp::Node{
public:
    explicit robot_recv_thread(const std::string node_name);
    ~robot_recv_thread();
private:
    int setKeepAlive(int fd, int idle_time, int interval_time, int probe_times);
    int _socketfd1;
    std::atomic_bool _reconnect_flag;
    int _robot_recv_exit = 0;           //类即将析构，通知重连线程退出.
    std::thread _reconnect_thread;
    void _try_to_reconnect();

    std::string _controller_ip;
    void _state_recv_callback();
    rclcpp::Publisher<robot_feedback_msg>::SharedPtr _state_publisher;//进程内通信，用于发送状态数据字符串
    rclcpp::TimerBase::SharedPtr _locktimer;
    int port1 = 8081;//非实时状态数据获取端口

};




#endif
