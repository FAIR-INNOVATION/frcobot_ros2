#ifndef _DATA_TYPE_DEF_H
#define _DATA_TYPE_DEF_H
#include "map"
#include "string"

/*用于ros2_control框架使用的宏定义*/
#define POSITION_CONTROL_MODE 1
#define TORQUE_CONTROL_MODE 2
#define RT_PACKAGE_SIZE sizeof(FR_rt_state)
/******************************/

/*用于指令系统使用的宏定义*/
#define NONRT_PACKAGE_SIZE sizeof(FR_nonrt_state)
/******************************/

/*用于控制器IP地址的宏定义*/
#define CONTROLLER_IP "192.168.58.2"
/******************************/

/*用于接受缓存大小的宏定义*/
#define RECV_BUFF 512
/******************************/

/*用于到为判断关节和笛卡尔位置指令与反馈差距的阈值*/
#define JNT_ERROR_THREASHOLD 0.2

/*8081端口状态反馈结构体宏定义*/
#define REG_VAR_NB_MAX_NUM 20	/** 数值型变量个数 */
#define REG_VAR_STR_MAX_NUM 10	/** 字符型变量个数 */
#define MAXSLAVES 8
#define MAXAUXJNTS 4
#define MAXSYSJNTS 6
#define TM_SYS_VAR_NUM 20
#define MAXPIOIN	1
#define MAXADCIN 	2
#define MAXAUXADCIN	1
#define _CTRL_STATE_SIZE sizeof(_CTRL_STATE)
/******************************/

#endif
