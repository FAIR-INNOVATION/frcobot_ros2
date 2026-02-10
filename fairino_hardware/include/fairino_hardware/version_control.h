#ifndef VERSION_CONTROL_H_
#define VERSION_CONTROL_H_

//版本号规则：V(VERSION_MAJOR).(VESION_MINOR).(VERSION_MINOR2)
#define VERSION_MAJOR 3
#define VERSION_MINOR 0
#define VERSION_MINOR2 1

//用于记录对应机械臂软件版本号
#define VERSION_ROBOT_MARJOR 3
#define VERSION_ROBOT_MINOR 9
#define VERSION_ROBOT_MINOR2 3


//用于记录fairino_msgs版本号
/*
V3.0.3 将error_code拆分成main_error_code和sub_error_code并实现反馈
V3.0.4 更新airino_msgs中反馈。删除了焊接相关参数、按钮盒控制参数等，增加了速度和加速度参数、力矩相关参数、夹爪相关参数等
*/
#define VERSION_MSG_MARJOR 3
#define VERSION_MSG_MINOR 0
#define VERSION_MSG_MINOR2 4

// #define CHN_VERSION
#define ENG_VERSION
#endif
