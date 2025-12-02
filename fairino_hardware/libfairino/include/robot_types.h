#ifndef    ROBOTTYPES_H_
#define    ROBOTTYPES_H_

#include <stdio.h>
#include <iostream>
#include <list>
#include <vector>
#include <string>
#include <string.h>
#include <sstream>

typedef  int errno_t;
typedef  unsigned char uint8_t;
typedef  signed char int8_t;

typedef  unsigned short int  uint16_t;

typedef  unsigned int uint32_t;

/**
 * @brief 关节位置数据类型
 */
typedef struct JointPos
{
	double jPos[6];   /* 六个关节位置，单位deg */
	JointPos(double J1, double J2, double J3, double J4, double J5, double J6)
	{
		jPos[0] = J1;
		jPos[1] = J2;
		jPos[2] = J3;
		jPos[3] = J4;
		jPos[4] = J5;
		jPos[5] = J6;
	}

	JointPos()
	{

	}
}JointPos;

/**
* @brief 笛卡尔空间位置数据类型
*/
typedef struct
{
	double x;    /* x轴坐标，单位mm  */
	double y;    /* y轴坐标，单位mm  */
	double z;    /* z轴坐标，单位mm  */
} DescTran;

/**
* @brief 欧拉角姿态数据类型
*/
typedef struct
{
	double rx;   /* 绕固定轴X旋转角度，单位：deg  */
	double ry;   /* 绕固定轴Y旋转角度，单位：deg  */
	double rz;   /* 绕固定轴Z旋转角度，单位：deg  */
} Rpy;

/**
 *@brief 笛卡尔空间位姿类型
 */
typedef struct DescPose
{
	DescTran tran;      /* 笛卡尔空间位置  */
	Rpy rpy;			/* 笛卡尔空间姿态  */

	DescPose(double x, double y, double z, double rx, double ry, double rz)
	{
		tran.x = x;
		tran.y = y;
		tran.z = z;

		rpy.rx = rx;
		rpy.ry = ry;
		rpy.rz = rz;
	}

	DescPose()
	{

	}
} DescPose;

/**
 * @brief 扩展轴位置数据类型
 */
typedef  struct ExaxisPos
{
	double ePos[4];   /* 四个扩展轴位置，单位mm */

	ExaxisPos(double axis1, double axis2, double axis3, double axis4)
	{
		ePos[0] = axis1;
		ePos[1] = axis2;
		ePos[2] = axis3;
		ePos[3] = axis4;
	}

	ExaxisPos()
	{

	}

}ExaxisPos;

/**
 * @brief 力传感器的受力分量和力矩分量
 */
typedef struct
{
	double fx;  /* 沿x轴受力分量，单位N  */
	double fy;  /* 沿y轴受力分量，单位N  */
	double fz;  /* 沿z轴受力分量，单位N  */
	double tx;  /* 绕x轴力矩分量，单位Nm */
	double ty;  /* 绕y轴力矩分量，单位Nm */
	double tz;  /* 绕z轴力矩分量，单位Nm */
} ForceTorque;


/**
 * @brief  螺旋参数数据类型
 */
typedef  struct
{
	int    circle_num;           /* 螺旋圈数  */
	float  circle_angle;         /* 螺旋倾角  */
	float  rad_init;             /* 螺旋初始半径，单位mm  */
	float  rad_add;              /* 半径增量  */
	float  rotaxis_add;          /* 转轴方向增量  */
	unsigned int rot_direction;  /* 旋转方向，0-顺时针，1-逆时针  */
}SpiralParam;

typedef struct AxleComParam
{
	int baudRate;   //波特率：支持 1-9600，2-14400，3-19200，4-38400，5-56000，6-67600，7-115200，8-128000；
	int dataBit;    //数据位：数据位支持（8,9），目前常用为 8
	int stopBit;    //停止位：1-1，2-0.5，3-2，4-1.5，目前常用为 1
	int verify;     //校验位：0-None，1-Odd，2-Even,目前常用为 0；
	int timeout;    //超时时间：1~1000ms，此值需要结合外设搭配设置合理的时间参数
	int timeoutTimes;  //超时次数：1~10，主要进行超时重发，减少偶发异常提高用户体验
	int period;     //周期性指令时间间隔：1~1000ms，主要用于周期性指令每次下发的时间间隔

	AxleComParam()
	{

	}

	AxleComParam(int _baudRate, int _dataBit, int _stopBit, int _verify, int _timeout, int _timeoutTimes, int _period)
	{
		baudRate = _baudRate;
		dataBit = _dataBit;
		stopBit = _stopBit;
		verify = _verify;
		timeout = _timeout;
		timeoutTimes = _timeoutTimes;
		period = _period;
	}
}AxleComParam;

/**
 * @brief 机器人时间
 */
#pragma pack(push)
#pragma pack(1)
typedef struct RobotTime
{
	uint16_t year = 0;
	uint8_t mouth = 0;
	uint8_t day = 0;
	uint8_t hour = 0;
	uint8_t minute = 0;
	uint8_t second = 0;
	uint16_t millisecond = 0;

	RobotTime()
	{

	}

	std::string ToString()
	{
		std::string rtn = std::to_string(year) + "-" + std::to_string(mouth) + "-" + std::to_string(day) + " " + std::to_string(hour) + ":" + std::to_string(minute) + ":" + std::to_string(second) + "." + std::to_string(millisecond);
		
		return rtn;
	}

}RobotTime;
#pragma pack(pop)

#pragma pack(push)
#pragma pack(1)
typedef struct ROBOT_AUX_STATE
{
	uint8_t servoId;
	int servoErrCode;
	int servoState;
	double servoPos;
	float servoVel;
	float servoTorque;
} robot_aux_state;
#pragma pack(pop)


#pragma pack(push)
#pragma pack(1)
typedef struct _EXT_AXIS_STATUS
{
	double pos;           //扩展轴位置
	double vel;           //扩展轴速度
	int errorCode;        //扩展轴故障码
	uint8_t ready;        //伺服准备好
	uint8_t inPos;        //伺服到位
	uint8_t alarm;        //伺服报警
	uint8_t flerr;        //跟随误差
	uint8_t nlimit;       //到负限位
	uint8_t pLimit;       //到正限位
	uint8_t mdbsOffLine;  //驱动器485总线掉线
	uint8_t mdbsTimeout;  //控制卡与控制箱485通信超时
	uint8_t homingStatus; //扩展轴回零状态
}EXT_AXIS_STATUS;
#pragma pack(pop)



#pragma pack(push)
#pragma pack(1)

typedef struct _ROBOT_STATE_PKG
{
	uint16_t frame_head;
	uint8_t  frame_cnt;
	uint16_t data_len;
	uint8_t  program_state;  //Program running status, 1- stop;2- Run; 3- Pause
	uint8_t  robot_state;//Robot motion state, 1- stop; 2- Run; 3- Pause; 4- Drag  
	int      main_code;
	int      sub_code;
	uint8_t  robot_mode;//Robot mode, 0-automatic mode; 1- Manual mode
	double   jt_cur_pos[6];//Current joint position
	double   tl_cur_pos[6];//Current tool position
	double   flange_cur_pos[6];//Current pose of end flange
	double   actual_qd[6];//Robot current joint speed
	double   actual_qdd[6];//Current joint acceleration of robot
	double   target_TCP_CmpSpeed[2];//Robot TCP synthesis command speed
	double   target_TCP_Speed[6]; //Robot TCP command speed
	double   actual_TCP_CmpSpeed[2]; //Robot TCP synthesizes actual speed
	double   actual_TCP_Speed[6];//Robot TCP actual speed
	double   jt_cur_tor[6];//Current torque 
	int      tool;//tool number
	int      user; //workpiece number
	uint8_t  cl_dgt_output_h;//Digital output 15-8
	uint8_t  cl_dgt_output_l;//Digital output 7-0
	uint8_t  tl_dgt_output_l; //tool digital output7-0
	uint8_t  cl_dgt_input_h;//Digital input 15-8
	uint8_t  cl_dgt_input_l;//Digital input 7-0
	uint8_t  tl_dgt_input_l;//tool Digital input 7-0
	uint16_t cl_analog_input[2];//Control box analog input
	uint16_t tl_anglog_input; //Tool analog input
	double   ft_sensor_raw_data[6];//Force/torque sensor raw data
	double   ft_sensor_data[6];//Force/torque sensor data
	uint8_t  ft_sensor_active;//Force/torque sensor active status, 0-reset, 1-activated
	uint8_t  EmergencyStop;//Emergency stop sign
	int      motion_done;//Position signal
	uint8_t  gripper_motiondone;//gripper movement complete signal
	int      mc_queue_len; //Motion queue length
	uint8_t  collisionState;//Collision detection, 1- collision; 0- No collision
	int      trajectory_pnum; //Track point number
    uint8_t  safety_stop0_state;  /* 安全停止信号SI0 *//* Safety stop signal SI0 */
    uint8_t  safety_stop1_state;  /* 安全停止信号SI1 *//* Safety stop signal SI1 */
    uint8_t  gripper_fault_id;    /* 错误夹爪号 */ /* gripper error number */
    uint16_t gripper_fault;       /* 夹爪故障 *//* Gripper fault */
    uint16_t gripper_active;      /* 夹爪激活状态 *//* Gripper active status */
    uint8_t  gripper_position;    /* 夹爪位置 */ /* Gripper position */
    int8_t   gripper_speed;       /* 夹爪速度 */ /* Gripper speed */
    int8_t   gripper_current;     /* 夹爪电流 *//* Gripper current */
    int      gripper_temp;        /* 夹爪温度 *//* Gripper temperature */
    int      gripper_voltage;     /* 夹爪电压 *//* Gripper voltage */
	robot_aux_state aux_state;/* 485Extended axis state */
	EXT_AXIS_STATUS extAxisStatus[4];  /* UDP扩展轴状态 */
	uint16_t extDIState[8];        //扩展DI输入
	uint16_t extDOState[8];        //扩展DO输出
	uint16_t extAIState[4];        //扩展AI输入
	uint16_t extAOState[4];        //扩展AO输出
	int rbtEnableState;            //机器人使能状态                robot enable state
	double   jointDriverTorque[6];        //机器人关节驱动器扭矩    Robot joint drive torque
	double   jointDriverTemperature[6];   //机器人关节驱动器温度    Robot joint drive temperature
	RobotTime robotTime;           //机器人系统时间                 Robot System time
	int softwareUpgradeState;  //机器人软件升级状态              Robot Software Upgrade State
	uint16_t endLuaErrCode;    //末端LUA运行状态 
	uint16_t check_sum;            /* 和校验 */
}ROBOT_STATE_PKG;

#pragma pack(pop)


#endif
