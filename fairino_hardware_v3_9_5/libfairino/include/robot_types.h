#ifndef ROBOTTYPES_H_
#define ROBOTTYPES_H_

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
		jPos[0] = 0.0;
		jPos[1] = 0.0;
		jPos[2] = 0.0;
		jPos[3] = 0.0;
		jPos[4] = 0.0;
		jPos[5] = 0.0;
	}
}JointPos;

/**
* @brief 笛卡尔空间位置数据类型
*/
typedef struct DescTran
{
	double x;    /* x轴坐标，单位mm  */
	double y;    /* y轴坐标，单位mm  */
	double z;    /* z轴坐标，单位mm  */

	DescTran(double _x, double _y, double _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	DescTran()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}
} DescTran;

/**
* @brief 欧拉角姿态数据类型
*/
typedef struct Rpy
{
	double rx;   /* 绕固定轴X旋转角度，单位：deg  */
	double ry;   /* 绕固定轴Y旋转角度，单位：deg  */
	double rz;   /* 绕固定轴Z旋转角度，单位：deg  */
	Rpy(double _rx, double _ry, double _rz)
	{
		rx = _rx;
		ry = _ry;
		rz = _rz;
	}

	Rpy()
	{
		rx = 0.0;
		ry = 0.0;
		rz = 0.0;
	}
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
		tran = { 0.0, 0.0, 0.0 };
		rpy = { 0.0, 0.0, 0.0 };
	}
} DescPose;

/**
 * @brief 扩展轴位置数据类型
 */
typedef struct ExaxisPos
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
	int velAccMode;              /* 速度加速度参数模式：0-角速度恒定；1-线速度恒定 */
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
	uint8_t ready;        //伺服准备好  0-使能；1-未使能
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
typedef struct _WELDING_BREAKOFF_STATE
{
	uint8_t breakOffState;  //焊接中断状态
	uint8_t weldArcState;   //焊接电弧中断状态
}WELDING_BREAKOFF_STATE;
#pragma pack(pop)



#pragma pack(push)
#pragma pack(1)

typedef struct _ROBOT_STATE_PKG
{
	uint16_t frame_head;      // 帧头，约定为0x5A5A 
	uint8_t  frame_cnt;       // 帧计数，循环计数0-255 
	uint16_t data_len;        // 数据内容的长度 
	uint8_t  program_state;   // 程序运行状态，1-停止；2-运行；3-暂停；
	uint8_t  robot_state;     // 机器人运动状态，1-停止；2-运行；3-暂停；4-拖动 
	int      main_code;       // 主故障码 
	int      sub_code;        // 子故障码 
	uint8_t  robot_mode;      // 机器人模式，1-手动模式；0-自动模式； 
	double   jt_cur_pos[6];   // 6个轴当前关节位置，单位deg 
	double   tl_cur_pos[6];   // 工具当前位置
				// tl_cur_pos[0]，沿x轴位置，单位mm，
				// tl_cur_pos[1]，沿y轴位置，单位mm，
				// tl_cur_pos[2]，沿z轴位置，单位mm，
				// tl_cur_pos[3]，绕固定轴X旋转角度，单位deg
				// tl_cur_pos[4]，绕固定轴y旋转角度，单位deg
				// tl_cur_pos[5]，绕固定轴z旋转角度，单位deg 
	double   flange_cur_pos[6]; // 末端法兰当前位置
				// flange_cur_pos[0]，沿x轴位置，单位mm，
				// flange_cur_pos[1]，沿y轴位置，单位mm，
				// flange_cur_pos[2]，沿z轴位置，单位mm，
				// flange_cur_pos[3]，绕固定轴X旋转角度，单位deg
				// flange_cur_pos[4]，绕固定轴y旋转角度，单位deg
				// flange_cur_pos[5]，绕固定轴z旋转角度，单位deg
	double   actual_qd[6];      // 当前6个关节速度，单位deg/s 
	double   actual_qdd[6];     // 当前6个关节加速度，单位deg/s^2 
	double   target_TCP_CmpSpeed[2]; 
				// target_TCP_CmpSpeed[0]，TCP合成指令速度(位置)，单位mm/s
				// target_TCP_CmpSpeed[1]，TCP合成指令速度(姿态)，单位deg/s  */
	double   target_TCP_Speed[6];    // TCP指令速度
					// target_TCP_Speed[0]，沿x轴速度，单位mm/s，
					// target_TCP_Speed[1]，沿y轴速度，单位mm/s，
					// target_TCP_Speed[2]，沿z轴速度，单位mm/s，
					// target_TCP_Speed[3]，绕固定轴X旋转角速度，单位deg/s
					// target_TCP_Speed[4]，绕固定轴y旋转角速度，单位deg/s
					// target_TCP_Speed[5]，绕固定轴z旋转角速度，单位deg/s */
	double   actual_TCP_CmpSpeed[2]; 
					// actual_TCP_CmpSpeed[0]，TCP合成实际速度(位置)，单位mm/s
				    // actual_TCP_CmpSpeed[1]，TCP合成实际速度(姿态)，单位deg/s 
	double   actual_TCP_Speed[6];    // TCP实际速度
					// actual_TCP_Speed[0]，沿x轴速度，单位mm/s，
					// actual_TCP_Speed[1]，沿y轴速度，单位mm/s，
					// actual_TCP_Speed[2]，沿z轴速度，单位mm/s，
					// actual_TCP_Speed[3]，绕固定轴X旋转角速度，单位deg/s
					// actual_TCP_Speed[4]，绕固定轴y旋转角速度，单位deg/s
					// actual_TCP_Speed[5]，绕固定轴z旋转角速度，单位deg/s 
	double   jt_cur_tor[6];      // 6个轴当前扭矩，单位N·m 
	int      tool;               // 应用的工具坐标系编号 
	int      user;               // 应用的工件坐标系编号 
	uint8_t  cl_dgt_output_h;    // 控制箱数字量IO输出15-8 
	uint8_t  cl_dgt_output_l;    // 控制箱数字量IO输出7-0 
	uint8_t  tl_dgt_output_l;    // 工具数字量IO输出7-0，仅bit0-bit1有效 
	uint8_t  cl_dgt_input_h;     // 控制箱数字量IO输入15-8 
	uint8_t  cl_dgt_input_l;     // 控制箱数字量IO输入7-0 
	uint8_t  tl_dgt_input_l;     // 工具数字量IO输入7-0，仅bit0-bit1有效 
	uint16_t cl_analog_input[2]; // cl_analog_input[0]，控制箱模拟量输入0
								 //cl_analog_input[1]，控制箱模拟量输入1 
	uint16_t tl_anglog_input;    // 工具模拟量输入 
	double   ft_sensor_raw_data[6]; // 力矩传感器原始数据
					// ft_sensor_raw_data[0]，沿x轴力，单位N
					// ft_sensor_raw_data[1]，沿y轴力，单位N
					// ft_sensor_raw_data[2]，沿z轴力，单位N
					// ft_sensor_raw_data[3]，沿x轴力矩，单位Nm
					// ft_sensor_raw_data[4]，沿y轴力矩，单位Nm
					// ft_sensor_raw_data[5]，沿z轴力矩，单位Nm 
	double   ft_sensor_data[6];     // 力矩传感器数据，
					// ft_sensor_data[0]，沿x轴力，单位N
					// ft_sensor_data[1]，沿y轴力，单位N
					// ft_sensor_data[2]，沿z轴力，单位N
					// ft_sensor_data[3]，沿x轴力矩，单位Nm
					// ft_sensor_data[4]，沿y轴力矩，单位Nm
					// ft_sensor_data[5]，沿z轴力矩，单位Nm 
	uint8_t  ft_sensor_active;   // 力矩传感器激活状态，0-复位，1-激活 
	uint8_t  EmergencyStop;      // 急停标志，0-急停未按下，1-急停按下 
	int      motion_done;        // 运动到位信号，1-到位，0-未到位 
	uint8_t  gripper_motiondone; // 夹爪运动完成信号，1-完成，0-未完成 
	int      mc_queue_len;       // 运动指令队列长度 
	uint8_t  collisionState;     // 碰撞检测，1-碰撞，0-无碰撞 
	int      trajectory_pnum;    // 轨迹点编号 
	uint8_t  safety_stop0_state; // 安全停止信号SI0 
	uint8_t  safety_stop1_state; // 安全停止信号SI1 
	uint8_t  gripper_fault_id;   // 错误夹爪号 
	uint16_t gripper_fault;      // 夹爪故障 
	uint16_t gripper_active;     // 夹爪激活状态 
	uint8_t  gripper_position;   // 夹爪位置 
	int8_t   gripper_speed;      // 夹爪速度 
	int8_t   gripper_current;    // 夹爪电流 
	int      gripper_temp;       // 夹爪温度 
	int      gripper_voltage;    // 夹爪电压 
	robot_aux_state aux_state;   // 485扩展轴状态
	EXT_AXIS_STATUS extAxisStatus[4];  // UDP扩展轴状态 
	uint16_t extDIState[8];        // 扩展DI输入
	uint16_t extDOState[8];        // 扩展DO输出
	uint16_t extAIState[4];        // 扩展AI输入
	uint16_t extAOState[4];        // 扩展AO输出
	int rbtEnableState;            // 机器人使能状态
	double   jointDriverTorque[6];        //机器人关节驱动器扭矩
	double   jointDriverTemperature[6];   //机器人关节驱动器温度
	RobotTime robotTime;           // 机器人系统时间
	int softwareUpgradeState;      // 机器人软件升级状态
	uint16_t endLuaErrCode;        // 末端LUA运行状态
	uint16_t cl_analog_output[2];  // 控制箱模拟量输出
	uint16_t tl_analog_output;     // 工具模拟量输出
	float gripperRotNum;           // 旋转夹爪当前旋转圈数
	uint8_t gripperRotSpeed;       // 旋转夹爪当前旋转速度百分比
	uint8_t gripperRotTorque;      // 旋转夹爪当前旋转力矩百分比
	WELDING_BREAKOFF_STATE weldingBreakOffState;  //焊接中断状态
	double jt_tgt_tor[6];                 // 关节指令力矩
	int smartToolState;                   // SmartTool手柄按钮状态
	float wideVoltageCtrlBoxTemp;         //宽电压控制箱温度
	uint16_t wideVoltageCtrlBoxFanCurrent;//宽电压控制箱风扇电流(mA)
	double toolCoord[6];        //当前工具坐标系数值；x,y,z,rx,ry,rz
	double wobjCoord[6];        //当前工件坐标系数值；x,y,z,rx,ry,rz
	double extoolCoord[6];      //当前外部工具坐标系数值；x,y,z,rx,ry,rz
	double exAxisCoord[6];      //当前扩展轴坐标系数值；x,y,z,rx,ry,rz
	double load;                //负载质量
	double loadCog[3];          //负载质心
	double lastServoTarget[6];  //队列中最后一个ServoJ目标位置
	int servoJCmdNum;           //servoJ指令计数
	double targetJointPos[6];   //6个关节指令位置，单位°
	double targetJointVel[6];   //6个关节指令速度，单位°/s
	double targetJointAcc[6];   //6个关节指令加速度，单位°/s2
	double targetJointCurrent[6]; //6个关节指令电流，单位A
	double actualJointCurrent[6]; //6个关节当前电流，单位A
	double actualTCPForce[6];   //机器人末端力矩Nm；x,y,z,rx,ry,rz
	double targetTCPPos[6];     //机器人TCP指令位置mm；x,y,z,rx,ry,rz
	uint8_t collisionLevel[6];  //机器人碰撞等级
	double speedScaleManual;    //手动模式全局速度百分比
	double speedScaleAuto;      //自动模式全局速度百分比
	int luaLineNum;             //当前lua程序运行行号
	uint8_t abnomalStop;        //0-无异常；1-有异常
	char currentLuaFileName[256]; //当前运行lua程序名称
	uint8_t programTotalLine;   //lua程序总行数
	uint8_t safetyBoxSingal[6]; //机器人按钮盒按钮状态
	double weldVoltage;         //焊接电压 V
	double weldCurrent;         //焊接电流
	double weldTrackVel;        //焊缝跟踪速度 mm/s
	uint8_t tpdException;       //TPD轨迹加载数量超限，0-未超限，1-超限 
	uint8_t alarmRebootRobot;   //警告，1-松开急停按钮请断电重启控制箱，2-关节通讯异常请断电重启控制箱
	uint8_t modbusMasterConnect;  //bit0-bit7位对应ModbusTCP的0-7主站连接状态  0-未连接   1-连接
	uint8_t modbusSlaveConnect;   //ModbusTCP从站连接状态 0-未连接；1-已连接
	uint8_t btnBoxStopSignal;     //按钮盒急停信号，0-松开急停；1-按下急停
	uint8_t dragAlarm;            //拖动警告，当前处于自动模式,0-不报警，1-报警 ，2-位置反馈异常不切换
	uint8_t safetyDoorAlarm;      //安全门警告；0-安全门关闭；1-安全门打开
	uint8_t safetyPlaneAlarm;     //进入安全墙警告；0-未进入安全墙；1-已进入安全墙
	uint8_t motonAlarm;           //运动警告
	uint8_t interfaceAlarm;       //进入干涉区警告
	int udpCmdState;              //20007端口UDP通讯连接状态
	uint8_t weldReadyState;       //焊机准备完成状态
	uint8_t alarmCheckEmergStopBtn;  //0-正常；1-通信异常，检查急停按钮是否松开
	uint8_t tsTmCmdComError;      //0-正常；1-扭矩指令通讯失败
	uint8_t tsTmStateComError;    //0-正常；1-扭矩状态通讯失败
	int ctrlBoxError;             //控制箱错误
	uint8_t safetyDataState;      //安全数据状态标志，0-正常，1-异常
	uint8_t forceSensorErrState;  //力传感器连接超时故障；bit0-bit1对应力传感器ID1-ID2
	uint8_t ctrlOpenLuaErrCode[4];  //4个控制器外设协议错误码(500错误码)
	uint8_t strangePosFlag;  //当前处于奇异位姿标志；0-正常；1-奇异位姿
	uint8_t alarm;           //警告
	uint8_t driverAlarm;     //驱动器报警轴号
	uint8_t aliveSlaveNumError; //活动从站数量错误，0：正常；1：数量错误
	uint8_t slaveComError[8];   //从站错误，0：正常；1：从站掉线；2：从站状态与设置值不一致；3：从站未配置；4：从站配置错误；5：从站初始化错误；6：从站邮箱通信初始化错误
	uint8_t cmdPointError;      //指令点错误
	uint8_t IOError;            //IO错误
	uint8_t gripperError;       //夹爪错误
	uint8_t fileError;          //文件错误
	uint8_t paraError;          //参数错误
	uint8_t exaxisOutLimitError;   //外部轴超出软限位错误
	uint8_t driverComError[6];     //与驱动器通信故障
	uint8_t driverError;           //驱动器通信故障轴号
	uint8_t outSoftLimitError;     //超出软限位故障
	char axleGenComData[130];      //机器人末端透传反馈数据
	uint8_t socketConnTimeout;     //socket连接超时，bit0-bit4:socketID 1-4
	uint8_t socketReadTimeout;     //socket读取超时，bit0-bit4:socketID 1-4
	uint8_t tsWebStateComErr;      //web-扭矩通讯失败；0-正常；1-失败
	uint16_t check_sum;         // 和校验
}ROBOT_STATE_PKG;

#pragma pack(pop)

enum class RobotState
{
	ProgramState = 3,           // 程序运行状态，1-停止；2-运行；3-暂停
	RobotState = 4,             // 机器人运动状态，1-停止；2-运行；3-暂停；4-拖动
	MainCode = 5,               // 主故障码
	SubCode = 6,                // 子故障码
	RobotMode = 7,              // 机器人模式，1-手动模式；0-自动模式
	JointCurPos = 8,            // 6个轴当前关节位置，单位deg
	ToolCurPos = 9,             // 工具当前位置：[0]沿x轴位置(mm)，[1]沿y轴(mm)，[2]沿z轴(mm)，[3]绕固定轴X旋转(deg)，[4]绕固定轴Y(deg)，[5]绕固定轴Z(deg)
	FlangeCurPos = 10,          // 末端法兰当前位置：[0]沿x轴(mm)，[1]沿y轴(mm)，[2]沿z轴(mm)，[3]绕固定轴X(deg)，[4]绕固定轴Y(deg)，[5]绕固定轴Z(deg)
	ActualJointVel = 11,        // 当前6个关节速度，单位deg/s
	ActualJointAcc = 12,        // 当前6个关节加速度，单位deg/s^2
	TargetTCPCmpSpeed = 13,     // TCP合成指令速度：[0]位置(mm/s)，[1]姿态(deg/s)
	TargetTCPSpeed = 14,        // TCP指令速度：[0]沿x轴(mm/s)，[1]沿y轴(mm/s)，[2]沿z轴(mm/s)，[3]绕X角速度(deg/s)，[4]绕Y(deg/s)，[5]绕Z(deg/s)
	ActualTCPCmpSpeed = 15,     // TCP合成实际速度：[0]位置(mm/s)，[1]姿态(deg/s)
	ActualTCPSpeed = 16,        // TCP实际速度：[0]沿x轴(mm/s)，[1]沿y轴(mm/s)，[2]沿z轴(mm/s)，[3]绕X角速度(deg/s)，[4]绕Y(deg/s)，[5]绕Z(deg/s)
	ActualJointTorque = 17,     // 6个轴当前扭矩，单位N·m
	Tool = 18,                  // 应用的工具坐标系编号
	User = 19,                  // 应用的工件坐标系编号
	ClDgtOutputH = 20,          // 控制箱数字量IO输出15-8
	ClDgtOutputL = 21,          // 控制箱数字量IO输出7-0
	TlDgtOutputL = 22,          // 工具数字量IO输出7-0，仅bit0-bit1有效
	ClDgtInputH = 23,           // 控制箱数字量IO输入15-8
	ClDgtInputL = 24,           // 控制箱数字量IO输入7-0
	TlDgtInputL = 25,           // 工具数字量IO输入7-0，仅bit0-bit1有效
	ClAnalogInput = 26,         // 控制箱模拟量输入：[0]通道0，[1]通道1
	TlAnalogInput = 27,         // 工具模拟量输入
	FtSensorRawData = 28,       // 力矩传感器原始数据：[0]沿x轴力(N)，[1]沿y轴力(N)，[2]沿z轴力(N)，[3]沿x轴力矩(Nm)，[4]沿y轴(Nm)，[5]沿z轴(Nm)
	FtSensorData = 29,          // 力矩传感器数据（经处理）：[0]沿x轴力(N)，[1]沿y轴力(N)，[2]沿z轴力(N)，[3]沿x轴力矩(Nm)，[4]沿y轴(Nm)，[5]沿z轴(Nm)
	FtSensorActive = 30,        // 力矩传感器激活状态，0-复位，1-激活
	EmergencyStop = 31,         // 急停标志，0-急停未按下，1-急停按下
	MotionDone = 32,            // 运动到位信号，1-到位，0-未到位
	GripperMotiondone = 33,     // 夹爪运动完成信号，1-完成，0-未完成
	McQueueLen = 34,            // 运动指令队列长度
	CollisionState = 35,        // 碰撞检测，1-碰撞，0-无碰撞
	TrajectoryPnum = 36,        // 轨迹点编号
	SafetyStop0State = 37,      // 安全停止信号SI0
	SafetyStop1State = 38,      // 安全停止信号SI1
	GripperFaultId = 39,        // 错误夹爪号
	GripperFault = 40,          // 夹爪故障
	GripperActive = 41,         // 夹爪激活状态
	GripperPosition = 42,       // 夹爪位置
	GripperSpeed = 43,          // 夹爪速度
	GripperCurrent = 44,        // 夹爪电流
	GripperTemp = 45,           // 夹爪温度
	GripperVoltage = 46,        // 夹爪电压
	AuxState = 47,              // 485扩展轴状态
	ExtAxisStatus = 48,         // UDP扩展轴状态（4个轴）
	ExtDIState = 49,            // 扩展DI输入（8个）
	ExtDOState = 50,            // 扩展DO输出（8个）
	ExtAIState = 51,            // 扩展AI输入（4个）
	ExtAOState = 52,            // 扩展AO输出（4个）
	RbtEnableState = 53,        // 机器人使能状态
	JointDriverTorque = 54,     // 机器人关节驱动器扭矩（6个关节）
	JointDriverTemperature = 55,// 机器人关节驱动器温度（6个关节）
	RobotTime = 56,             // 机器人系统时间
	SoftwareUpgradeState = 57,  // 机器人软件升级状态
	EndLuaErrCode = 58,         // 末端LUA运行状态
	ClAnalogOutput = 59,        // 控制箱模拟量输出（2个）
	TlAnalogOutput = 60,        // 工具模拟量输出
	GripperRotNum = 61,         // 旋转夹爪当前旋转圈数
	GripperRotSpeed = 62,       // 旋转夹爪当前旋转速度百分比
	GripperRotTorque = 63,      // 旋转夹爪当前旋转力矩百分比
	WeldingBreakOffState = 64,  // 焊接中断状态
	TargetJointTorque = 65,     // 关节指令力矩（6个关节）
	SmartToolState = 66,        // SmartTool手柄按钮状态
	WideVoltageCtrlBoxTemp = 67,// 宽电压控制箱温度
	WideVoltageCtrlBoxFanCurrent = 68, // 宽电压控制箱风扇电流(mA)
	ToolCoord = 69,             // 当前工具坐标系数值：x,y,z,rx,ry,rz
	WobjCoord = 70,             // 当前工件坐标系数值：x,y,z,rx,ry,rz
	ExtoolCoord = 71,           // 当前外部工具坐标系数值：x,y,z,rx,ry,rz
	ExAxisCoord = 72,           // 当前扩展轴坐标系数值：x,y,z,rx,ry,rz
	Load = 73,                  // 负载质量
	LoadCog = 74,               // 负载质心：x,y,z
	LastServoTarget = 75,       // 队列中最后一个ServoJ目标位置（6个关节）
	ServoJCmdNum = 76,          // servoJ指令计数
	TargetJointPos = 77,        // 6个关节指令位置，单位°
	TargetJointVel = 78,        // 6个关节指令速度，单位°/s
	TargetJointAcc = 79,        // 6个关节指令加速度，单位°/s²
	TargetJointCurrent = 80,    // 6个关节指令电流，单位A
	ActualJointCurrent = 81,    // 6个关节当前电流，单位A
	ActualTCPForce = 82,        // 机器人末端力矩：x,y,z,rx,ry,rz，单位Nm
	TargetTCPPos = 83,          // 机器人TCP指令位置：x,y,z,rx,ry,rz，单位mm
	CollisionLevel = 84,        // 机器人碰撞等级（6个）
	SpeedScaleManual = 85,      // 手动模式全局速度百分比
	SpeedScaleAuto = 86,        // 自动模式全局速度百分比
	LuaLineNum = 87,            // 当前lua程序运行行号
	AbnomalStop = 88,           // 0-无异常；1-有异常
	CurrentLuaFileName = 89,    // 当前运行lua程序名称
	ProgramTotalLine = 90,      // lua程序总行数
	SafetyBoxSingal = 91,       // 机器人按钮盒按钮状态（6个）
	WeldVoltage = 92,           // 焊接电压 V
	WeldCurrent = 93,           // 焊接电流
	WeldTrackVel = 94,          // 焊缝跟踪速度 mm/s
	TpdException = 95,          // TPD轨迹加载数量超限，0-未超限，1-超限
	AlarmRebootRobot = 96,      // 警告：1-松开急停后需断电重启，2-关节通讯异常需断电重启
	ModbusMasterConnect = 97,   // bit0-7对应ModbusTCP主站0-7连接状态，0-未连接，1-连接
	ModbusSlaveConnect = 98,    // ModbusTCP从站连接状态，0-未连接，1-已连接
	BtnBoxStopSignal = 99,      // 按钮盒急停信号，0-松开急停，1-按下急停
	DragAlarm = 100,            // 拖动警告：0-不报警，1-报警，2-位置反馈异常不切换
	SafetyDoorAlarm = 101,      // 安全门警告：0-关闭，1-打开
	SafetyPlaneAlarm = 102,     // 安全墙警告：0-未进入，1-已进入
	MotonAlarm = 103,           // 运动警告
	InterfaceAlarm = 104,       // 进入干涉区警告
	UdpCmdState = 105,          // 20007端口UDP通讯连接状态
	WeldReadyState = 106,       // 焊机准备完成状态
	AlarmCheckEmergStopBtn = 107, // 0-正常；1-通信异常，检查急停按钮
	TsTmCmdComError = 108,      // 0-正常；1-扭矩指令通讯失败
	TsTmStateComError = 109,    // 0-正常；1-扭矩状态通讯失败
	CtrlBoxError = 110,         // 控制箱错误
	SafetyDataState = 111,      // 安全数据状态，0-正常，1-异常
	ForceSensorErrState = 112,  // 力传感器连接超时，bit0-bit1对应ID1-ID2
	CtrlOpenLuaErrCode = 113,   // 4个控制器外设协议错误码(500错误码)
	StrangePosFlag = 114,       // 奇异位姿标志：0-正常，1-奇异位姿
	Alarm = 115,                // 警告
	DriverAlarm = 116,          // 驱动器报警轴号
	AliveSlaveNumError = 117,   // 活动从站数量错误：0-正常，1-数量错误
	SlaveComError = 118,        // 从站错误：0-正常，1-掉线，2-状态不一致，3-未配置，4-配置错误，5-初始化错误，6-邮箱通信初始化错误
	CmdPointError = 119,        // 指令点错误
	IOError = 120,              // IO错误
	GripperError = 121,         // 夹爪错误
	FileError = 122,            // 文件错误
	ParaError = 123,            // 参数错误
	ExaxisOutLimitError = 124,  // 外部轴超出软限位错误
	DriverComError = 125,       // 与驱动器通信故障（6个轴）
	DriverError = 126,          // 驱动器通信故障轴号
	OutSoftLimitError = 127,    // 超出软限位故障
	AxleGenComData = 128,       // 机器人末端透传反馈数据
	SocketConnTimeout = 129,    // socket连接超时，bit0-bit4对应socketID 1-4
	SocketReadTimeout = 130,    // socket读取超时，bit0-bit4对应socketID 1-4
	TsWebStateComErr = 131      // web-扭矩通讯失败：0-正常，1-失败
};


#endif
