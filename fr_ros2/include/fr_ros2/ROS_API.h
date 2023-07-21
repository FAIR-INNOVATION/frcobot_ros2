#ifndef ROS_API_H_
#define ROS_API_H_

#include "stdlib.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "frhal_msgs/srv/ros_cmd_interface.hpp"
#include "mutex"
#include "sys/socket.h"
#include "sys/types.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "fcntl.h"
#include <regex>
#include "data_type_def.h"
#include "global_val.h"

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
    template<typename T>
    T& _add_data_str(T str_data);
};


class ROS_API:public FRAPI_base,public rclcpp::Node{
public:
    ROS_API(const std::string node_name);
    ~ROS_API();
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

    //运动指令
    int  StartJOG(std::string para);
    int  StopJOG(std::string para);
    int  ImmStopJOG(std::string para);
    int  MoveJ(std::string para);
    int  MoveL(std::string para);
    int  MoveC(std::string para);
    int  Circle(std::string para);
    //int  NewSpiral(std::string para);
    //int  ServoJ(JointPos *joint_pos, float acc, float vel, float cmdT, float filterT, float gain);
    int  ServoJTStart(std::string para);
    int  ServoJT(std::string para);
    int  ServoJTEnd(std::string para);
    int  SplineStart(std::string para);
    int  SplinePTP(std::string para);
    int  SplineEnd(std::string para);
    int  NewSplineStart(std::string para);
    int  NewSplinePoint(std::string para);
    int  NewSplineEnd(std::string para);
    int  StopMotion(std::string para);
    int  PointsOffsetEnable(std::string para);
    int  PointsOffsetDisable(std::string para);

    //程序控制
    int ProgramRun(std::string para);

private:
    int (ROS_API:: *funcP)(std::string para);//函数指针是有作用域的，所以全局函数的指针和类内成员函数的指针定义有很大不同，这里不能用typedef
    int _send_data_factory_callback(std::string data);//模板函数,用于指令字符串数据的发送和反馈接收确认
    int _ParseRecvData(std::string str);//反馈值解析函数
    void _ParseROSCommandData_callback(const std::shared_ptr<frhal_msgs::srv::ROSCmdInterface::Request> req,
                                             std::shared_ptr<frhal_msgs::srv::ROSCmdInterface::Response> res);//用于解析用户发送的ROS接口指令
    void _selectfunc(std::string func_name);
    int _def_jnt_position(std::string pos);
    int _def_cart_position(std::string pos);
    std::string _get_variable(std::string para_list);
    uint16_t _cmd_counter;//指令数据帧计数器
    int _recv_data_cmdcount;//接受到的回复信息中指令计数器的值
    int _recv_data_cmdid;//接受到的回复信息中指令id
    int _recv_data_res;//接受到的回复信息中指令反馈结果
    float _kin_res[6];
    rclcpp::Service<frhal_msgs::srv::ROSCmdInterface>::SharedPtr _recv_ros_command_server;//用于接受用户发送过来的字符串指令
    int _robot_install;//机械臂安装方式
    std::vector<JointPos> _cmd_jnt_pos_list;//存储关节数据点
    std::vector<DescPose> _cmd_cart_pos_list;//存储笛卡尔数据点
    std::mutex _mtx;
    std::string _controller_ip;
    int port1 = 8080;//TCP命令发送端口
    int port2 = 8082;//脚本传输和自定义数据流发送端口
    int _socketfd1, _socketfd2;
    rclcpp::TimerBase::SharedPtr _locktimer;
    bool _skip_answer_flag;
};


#endif