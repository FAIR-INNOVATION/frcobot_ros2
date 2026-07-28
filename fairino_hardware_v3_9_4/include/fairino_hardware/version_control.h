#ifndef VERSION_CONTROL_H_
#define VERSION_CONTROL_H_

//版本号规则：V(VERSION_MAJOR).(VESION_MINOR).(VERSION_MINOR2)
#define VERSION_MAJOR 3
#define VERSION_MINOR 0
#define VERSION_MINOR2 0

//用于记录对应机械臂软件版本号
#define VERSION_ROBOT_MARJOR 3
#define VERSION_ROBOT_MINOR 9
#define VERSION_ROBOT_MINOR2 4


//用于记录fairino_msgs版本号
/*
V3.0.3 将error_code拆分成main_error_code和sub_error_code并实现反馈
V3.0.4 新增扩展轴状态数组
V3.0.5 新增总线夹爪状态反馈
V3.0.6 新增ServoJ目标位置、指令计数、运动队列长度、关节速度/加速度反馈
*/
#define VERSION_MSG_MARJOR 3
#define VERSION_MSG_MINOR 0
#define VERSION_MSG_MINOR2 6

// #define CHN_VERSION
#define ENG_VERSION
#endif
