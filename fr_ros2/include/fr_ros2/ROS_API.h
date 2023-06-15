#ifndef ROS_API_H_
#define ROS_API_H_

#include "stdlib.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "std_msgs/msg/string.hpp"
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
    //Common settings class
    int  DragTeachSwitch(std::string para);//Drag to switch teaching mode
    int  RobotEnable(std::string para);//Arm enable
    int  Mode(std::string para);//Manual mode, automatic mode switching
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

    //Security configuration
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

    //Peripheral control
    int  ActGripper(std::string para);
    int  MoveGripper(std::string para);
    
    //IO control
    int  SetDO(std::string para);
    int  SetToolDO(std::string para);
    int  SetAO(std::string para);
    int  SetToolAO(std::string para);

    //Motion command
    int  StartJOG(std::string para);
    int  StopJOG(std::string para);
    int  ImmStopJOG(std::string para);
    int  MoveJ(std::string para);
    int  MoveL(std::string para);
    int  MoveC(std::string para);
    int  Circle(std::string para);
    //int  NewSpiral(std::string para);
    //int  ServoJ(JointPos *joint_pos, float acc, float vel, float cmdT, float filterT, float gain);
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

private:
    int (ROS_API:: *funcP)(std::string para);//Function pointers have scope, so the definitions of pointers to global functions and pointers to member functions in a class are very different. Typedef cannot be used here
    int _send_data_factory_callback(std::string data);//Template function, used to send instruction string data and confirm receipt of feedback
    int _ParseRecvData(std::string str);//Feedback value analysis function
    void _ParseROSCommandData_callback(const std::shared_ptr<frhal_msgs::srv::ROSCmdInterface::Request> req,
                                             std::shared_ptr<frhal_msgs::srv::ROSCmdInterface::Response> res);//Used to parse the ROS interface instructions sent by the user
    void _selectfunc(std::string func_name);
    int _def_jnt_position(std::string pos);
    int _def_cart_position(std::string pos);
    std::string _get_variable(std::string para_list);
    int _retry_count;//Number of retries
    uint16_t _cmd_counter;//Command Data Frame Counter
    std::string _cur_func_name;
    int _cur_id;
    int _recv_data_cmdcount;//The value of the instruction counter in the received reply message
    int _recv_data_cmdid;//Command id in the received reply message
    int _recv_data_res;//The command feedback result in the received reply message
    float _kin_res[6];
    rclcpp::Service<frhal_msgs::srv::ROSCmdInterface>::SharedPtr _recv_ros_command_server;//Used to accept string commands sent by users
    int _robot_install;//Arm installation method
    std::vector<JointPos> _cmd_jnt_pos_list;//Store joint data points
    std::vector<DescPose> _cmd_cart_pos_list;//Store Cartesian data points
    std::mutex _mtx;
    std::string _controller_ip;
    int port1 = 8080;//TCP command sending port
    int port2 = 8082;//Script transmission and custom data stream sending port
    int _socketfd1, _socketfd2;
    rclcpp::TimerBase::SharedPtr _locktimer;

};


#endif