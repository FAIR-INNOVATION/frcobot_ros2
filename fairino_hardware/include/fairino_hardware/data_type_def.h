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



/*********8081端口结构体数据部分********/
#pragma pack(1)
/** 外部轴状态结构体 */
typedef struct _REG_VAR
{
	uint8_t	num_full;			//数值型变量监控个数已满, 1:代表已满 0:代表未满
	uint8_t	str_full;			//字符型变量监控个数已满, 1:代表已满 0:代表未满
	uint8_t	num_cnt;			//数值型变量使用个数
	uint8_t	str_cnt;			//字符型变量使用个数
	double	num[REG_VAR_NB_MAX_NUM];			//数值型变量值
	char	str[REG_VAR_STR_MAX_NUM][100];		//字符型变量值
	char	num_name[REG_VAR_NB_MAX_NUM][20];	//数值型变量名
	char	str_name[REG_VAR_STR_MAX_NUM][20];	//字符型变量名
} REG_VAR;

typedef struct _EXTERNALAXIS_STATU
{
	double exAxisPos;			//外部轴位置
	double exAxisSpeedBack;		//外部轴速度
	int exAxisErrorCode;		//外部轴故障码
	unsigned char exAxisRDY;	//伺服准备好
	unsigned char exAxisINPOS;	//伺服到位
	unsigned char exAxisALM;	//伺服报警
	unsigned char exAxisFLERR;	//跟随误差
	unsigned char exAxisNLMT;	//到负限位
	unsigned char exAxisPLMT;	//到正限位
	unsigned char exAxisAbsOFLN;//驱动器485总线掉线
	unsigned char exAxisOFLIN;	//通信超时，控制卡与控制箱板485通信超时
	uint8_t	exAxisHomeStatus;	//外部轴回零状态
} EXTERNALAXIS_STATUS;

/** 扩展轴外部伺服状态结构体 */
typedef struct _AUXSERVO_STATE
{
	uint8_t servo_id;			// 伺服驱动器 ID 号
	int32_t servo_errcode; 		// 伺服驱动器故障码
	int32_t servo_state; 		// 伺服驱动器状态
	double servo_actual_pos; 	// 伺服当前位置
	float servo_actual_speed;	// 伺服当前速度
	float servo_actual_torque;  // 伺服当前转矩
}AUXSERVO_STATE;

/** 打磨头状态结构体 */
typedef struct _POLISHING_STATE
{
	uint16_t StatusWord;		//状态字
	int16_t	ActualVel;			//实际速度，单位 rpm
	int16_t	ActualTorque;		//实际接触力，单位 N
	int16_t	Temperature;		//实际内部温度， 单位 °C
	float	ActualPos;			//实际位置，单位 0.01mm
	uint16_t ErrorCode;			//错误码
	uint16_t OperationMode;		//当前模式 , 1-回零模式，2-位置模式，4-力矩模式 
	uint8_t WarningState;		//报警状态：0-无报警,1-报警(打磨头通讯异常)
	uint8_t Arrive;
	uint8_t Emergency;
} POLISHING_STATE;

/** 焊接中断状态结构体 */
typedef struct _WELDING_BREAK_OFF_STATE
{
	uint8_t breakOffState;		//焊接中断状态信息：0-未中断,1-已中断
	uint8_t weldArcState;		//焊接电弧状态信息：0-电弧未中断,1-电弧中断
} WELDING_BREAK_OFF_STATE;

/** 远程控制接口结构体 */
typedef struct _REMOTE_CTRL_INF_STATE
{
	uint8_t robot_ctrl_mode;		/* 机器人控制模式，0：本地控制模式(手/自动模式)，1：远程控制模式 */
	uint8_t errState;				/* 错误状态，0-运行正常，1-运行异常 */
	uint16_t errCode;				/* 错误码 */
}REMOTE_CTRL_INF_STATE;

/** 8081端口运动控制器状态结构体 */
typedef struct _CTRL_STATE
{
	char       head[7];
   	int        frame_len;               /* 消息帧长度                                     */
	double     runtime;                 /* 控制器启动时间,断电清零    */
	double 	   jt_tgt_pos[6];           /* 关节1-6目标位置                */
	double     jt_tgt_vel[6];           /* 关节1-6目标速度                */
	double     jt_tgt_acc[6];			/* 关节1-6目标加速度              */
	double     jt_tgt_cur[6];			/* 关节1-6目标电流		        */
	double 	   jt_tgt_tor[6];           /* 关节1-6目标扭矩                */
	double     jt_cur_pos[6];           /* 关节1-6当前位置                */
	double     jt_cur_vel[6]; 			/* 关节1-6当前速度                */
	double     jt_cur_cur[6];			/* 关节1-6当前电流 		        */
	double     jt_cur_tor[6];			/* 关节1-6当前扭矩                */
	double     tl_cur_pos[6];			/* 工具当前位置DKR		        */
	double	   tl_cur_vel[6];			/* 工具当前速度DKR                */
	double     tl_jtforce[6];			/**< 工具合力DKR			        */
	double     tl_tgt_pos[6];  			/**< 工具目标位置DKR	            */
	double     tl_tgt_vel[6];			/**< 工具目标速度DKR                */
	double     tl_cur_pos_base[6];      /**< 工具当前位置Base         */
	double     tl_tgt_pos_base[6];      /**< 工具目标位置Base          */
	uint8_t	   cl_dgt_input_h;          /**< 控制箱数字输入15-8             */
	uint8_t    cl_dgt_input_l;	        /**< 控制箱数字输入7-0              */
	uint8_t    tl_dgt_input_h;          /**< 工具数字输入15-8               */
	uint8_t    tl_dgt_input_l;          /**< 工具数字输入7-0                */
	uint8_t    tmp_dgt_input1;			/**< 空闲                           */
	uint8_t    tmp_dgt_input2;			/**< 空闲                           */
	uint8_t    tmp_dgt_input3;			/**< 空闲                           */
	uint8_t    tmp_dgt_input4;			/**< 空闲                           */
	uint16_t   analog_input[6];         /**< 模拟输入,四个控制箱,1个工具(当前只引出一路)  */
	double     ctrl_time;               /**< 控制器实时线程执行时间         */
	double     test_data;               /**< 测试数据ur机器人软件使用       */
	uint8_t    robot_mode;              /**< 机器人模式                     */
	double     joint_mode[6];  			/**< 关节模式					    */
	double     safe_mode;               /**< 安全模式                       */
	int        collision_level;         /**< 碰撞等级,一级敏感,三级不敏感   */
	double     drag_enable;   			/**< 拖动使能                       */
	double     tl_acc[7];				/**< 工具加速度     			    */
	uint8_t    dr_com_err[6]; 			/**< 与驱动器通信故障               */
	double	   dr_err;                  /**< 驱动器哪个轴故障               */
	double     dr_alarm;                /**< 驱动器哪个轴警告               */
	double     out_sflimit_err;         /**< 超出软限位故障                 */
	double     collision_err;           /**< 碰撞故障                       */
	double     dr_err_code;  			/**< 驱动器故障代码 			    */
	double     dr_alarm_code;           /**< 驱动器警告代码                */
	double     vel_ratio;               /**< 机器人运行速度比例             */
	double     linear_m_bmak;           /**< 线性动量基准 				    */
	uint8_t    flag_zero_set;           /**< 机器人零位设定完成标志         */
	double     test_data1;  			/**< 测试数据ur机器人软件使用       */
	double     cb_vol;					/**< 控制板电压   				    */
	double     cb_robot_vol;            /**< 控制板机器人电压               */
	double     cb_robot_cur;            /**< 控制板机器人电流               */
	double     jt_cur_vol[6];           /**< 关节当前电压                   */
	uint8_t    cl_dgt_output_h;         /**< 数字输出15-8     */
	uint8_t    cl_dgt_output_l;         /**< 数字输出7-0      */
	uint8_t    tl_dgt_output_h;         /**< 工具数字输出15-8(暂未使用)                       */
	uint8_t    tl_dgt_output_l;	        /**< 工具数字输出7-0(bit0-bit1有效)                   */
	uint8_t    tmp_dgt_output3;		    /**< 空闲                                */
	uint8_t    tmp_dgt_output4;		    /**< 空闲                                */
	uint8_t    tmp_dgt_output5;			/**< 空闲                                */
	uint8_t    tmp_dgt_output6;			/**< 空闲                                */
	uint16_t   analog_output[6];        /**< 模拟输出,四个控制箱,1个工具(当前只引出一路)  */
	uint8_t    program_state;           /**< 程序状态 ,1:停止,2:运行,3：暂停,4：拖动 */
	int        line_number;             /**< 自动运行程序执行到某一指令行号 */
	double     elbow_vel[5];            /**< 肘速度                         */
	uint8_t    strangePosFlag;          /**< 当前处于奇异位姿标志 */
	int        configStatus;            /**< 机器人关节配置状态  */
	uint8_t    aliveSlaveNumError;      /** 活动从站数量错误，1：数量错误；0：正常 */
	uint8_t    slaveComError[MAXSLAVES]; /** 从站错误，0：正常；1：从站掉线；2：从站状态与设置值不一致；3：从站未配置；4：从站配置错误；5：从站初始化错误；6：从站邮箱通信初始化错误 */
	uint8_t    cmdPointError;           /** 指令点关节位置与末端位姿不符错误，0：正常；1：直线指令点错误；2：圆弧指令点错误；3：TPD指令工具与当前工具不符；4：TPD当前指令与下一指令起始点偏差过大 ；5：TPD关节指令超限；6：PTP关节指令超限*/
	uint8_t    ioError;                 /** IO错误 */
	uint8_t    gripperError;            /** 夹爪错误 */
	uint8_t    fileError;               /** 文件错误 */
	uint8_t    paraError;               /** 参数错误 */
	uint8_t    exaxis_out_slimit_error; /** 外部轴超出软限位错误 */
	EXTERNALAXIS_STATUS exaxis_status[MAXAUXJNTS];  /** 外部轴状态 */
	uint8_t    exAxisActiveFlag;        /** 外部轴激活标志 */
	uint8_t    exAxisMotionStatus;      /** 外部轴运动状态，0：完成，1：运动中，2：暂停中，3：暂停完成 */
	uint8_t    alarm;                   /** 警告 */
	uint8_t    safetydoor_alarm;        /** 安全门警告 */
	int        toolNum;                 /** 工具号 */
	int        workPieceNum;            /** 工件号 */
	int        exAxisNum;               /** 外部轴号 */
	uint8_t    gripperFaultId;          /** 错误夹爪号 */
	uint16_t   gripperFaultNum;         /** 夹爪错误编号 */
	uint16_t   gripperActStatus;        /** 夹爪激活状态 */
	uint8_t    gripperMotionDone;       /** 夹爪运动完成 */
	int        aliveSlaveNumFeedback;   /** 活动从站数量反馈 */
	uint8_t    ctrl_query_state;        /** 控制器查询状态 */
	uint8_t    weld_readystate;         /** 焊接准备状态 */
	double     weldTrackSpeed;          /** 焊缝跟踪速度   mm/s */
	uint8_t    drag_alarm;              /** 拖动警告，当前处于自动模式,0-不报警，1-报警 ，2-位置反馈异常不切换 */
	double     LoadIdentifyData[4];     /** 负载辨识结果（weight,x,y,z） */
	int        conveyor_encoder_pos;    /** 传送带编码器位置 */
	double     conveyor_speed;          /** 传送带速度 mm/s */
	double     conveyorWorkPiecePos;    /** 传送带工件当前位置，单位mm */
	uint8_t    btn_box_stop_signal;     /** 按钮盒急停信号 ，1-按下急停*/
	uint8_t    motionAlarm;             /** 运动警告 */
	uint8_t    interfereAlarm;          /** 进入干涉区警告 */
    uint8_t    safetyPlaneAlarm;        /** 进入安全墙警告 */
	REG_VAR    reg_var;                 /** 注册变量 */
	uint8_t    encoder_type_flag;       /** 编码器类型切换完成标志，0：未完成，1：完成，2：超时 */
	uint8_t    curEncoderType[MAXSYSJNTS];   /** 当前各轴编码器类型，0：光编，1：磁编 */
    uint8_t    alarm_check_emerg_stop_btn;     /**1- 通信异常，检查急停按钮是否松开 */
    uint8_t    alarm_reboot_robot;             /** 警告，1-松开急停按钮请断电重启控制箱，2-关节通讯异常请断电重启控制箱*/
    uint8_t    ts_web_state_com_error;   /* web-扭矩通讯失败 */
    uint8_t    ts_tm_cmd_com_error;      /* tm-扭矩指令通讯失败 */
    uint8_t    ts_tm_state_com_error;    /* tm-扭矩状态通讯失败 */
    uint8_t    pause_param;              /* 暂停接口参数 */
	float      sys_var[TM_SYS_VAR_NUM];  /* 系统变量 */
	uint8_t    tpd_record_state;         /* TPD记录状态，1-记录中，0-不记录 */
	uint8_t    tpd_record_scale;         /* TPD记录进度百分比，0~100 */
	uint8_t    tpd_exception;            /* TPD轨迹加载数量超限，0-未超限，1-超限 */
	double     FT_data[6];               /* 力/扭矩传感器数据,Fx,Fy,Fz,Tx,Ty,Tz */
	uint8_t    FT_ActStatus;             /* 力/扭矩传感器激活状态，0-复位，1-激活 */
	int        motion_done;              /* 运动完成信号，0-未完成，1-完成 */
	uint8_t    abnormal_stop;            /* 非正常停止 0-正常，1-非正常*/
	uint8_t    socket_conn_timeout;      /* socket连接超时，1-4 */
	uint8_t    socket_read_timeout;      /* socket读取超时，1-4 */
	uint16_t   virtual_cl_dgt_input[MAXPIOIN]; /* 控制箱模拟DI输入 */
	uint16_t   virtual_tl_dgt_input[MAXPIOIN]; /* 末端模拟DI输入 */
	float      virtual_cl_analog_input[MAXADCIN]; /* 控制箱模拟AI输入 */
	float      Virtual_tl_analog_input[MAXAUXADCIN];    /* 末端模拟AI输入 */
	uint8_t    pushBtnBoxState;          /* 按钮盒状态,1-使用，0-禁用 */
	int        rbtEnableState;           /* 机器人使能状态，0-不使能，1-使能 */
	uint16_t   ctrlBoxError;             /* 控制箱错误 */
	uint8_t    BackToHomeState;          /* 倒带信号 */
	uint8_t    SlaveError[16];                        /* 从站故障,0：无故障，1：[0]-末端电源短路，[1]-控制箱电源短路 */
	char       curLuaFileName[256];                    /* 当前解析的Lua文件名 */
	uint8_t    ndf_layer;                             /* NewDofile 所在文件层 */
	int        ndf_row;                               /* NewDofile 所在文件行 */
	uint16_t   ndf_id;                                /* NewDofile所在文件层编号 */
	uint16_t   extPioInput[8];                        /* 扩展DI  */
	uint16_t   extPioOutput[8];						  /* 扩展DO  */
	uint16_t   extAdcInput[4];                        /* 扩展AI  */
	uint16_t   extAdcOutput[4];                       /* 扩展AO  */
	uint8_t    SlavePortINErrCounter[8][4];           /* 8个从站IN端口错误帧数 */
	uint8_t    SlavePortOUTErrCounter[8][4];          /* 8个从站OUT端口错误帧数 */
    POLISHING_STATE polish_state;                     /** 打磨设备状态 */
    float      welding_voltage;                       /** 焊接电压 */
    float      welding_current;                       /** 焊接电流 */
    AUXSERVO_STATE auxservo_state;                    /** 外部伺服状态 */
    WELDING_BREAK_OFF_STATE welding_state;            /* 焊接状态 */
	REMOTE_CTRL_INF_STATE remote_ctrl_state;          /* 远程控制状态 */
	uint8_t    safety_data_state;                     /* 安全数据状态标志，0-正常，1-异常 */
	char       curPointTableName[128];				  /* 当前应用的点位表信息 */
	int        jog_status;                            /* 点动运动状态 */
	uint16_t modbusSlaveDI[8];      				  /** 从站DI0 - DI128 */
	uint16_t modbusSlaveDO[8];      				  /** 从站DO0 - DI128 */
	int modbusSlaveAI[64];          				  /** 从站AI uint16 AI0-AI15  int16 AI16-AI31  float AI32-AI63 */
	int modbusSlaveAO[64];          				  /** 从站AO uint16 AO0-AO15  int16 AO16-AO31  float AO32-AO63 */
	uint8_t modbusMasterConnectState;     			  /** 0-7位对应0-7主站连接状态  0-未连接   1-连接 */
	float modbusMasterValue[8][128]; 	   			  /** modbusMaster的寄存器当前数值，每128个值为1个ModbusMaster寄存器的所有值 */
	uint8_t shoulderConfig;                              /* 肩关节配置，0：左肩配置，1：右肩配置*/
	uint8_t elbowConfig;                                 /* 肘关节配置，0：肘向下配置，1：肘向上配置*/
	uint8_t wristConfig;                                 /* 腕关节配置，0：腕向下配置，1：腕向上配置*/
	uint32_t motionCount;                                /* 运动计数 */
	double     flange_cur_pos[6];                     /** 末端法兰当前位姿 */
	uint16_t endLuaErrCode;                           /** 末端Lua文件异常状态 0-正常；1-异常 */
	uint8_t mdbsSlaveConnect; 						  /** Modbus从站连接状态 0-未连接；1-已连接 */
	uint16_t mdbsSlaveFuncDIState[6]; 				  /** Modbus从站功能DI输入状态；bit0-bit10分别对应“暂停”  ~  “清除所有故障” */
	int mdbsSlaveDOCtrlDIState; 					  /** Modbus从站控制DO输出功能的DI输入状态，bit0-bit7为DO0-DO7；bit8-bit15为CO0-CO7；bit16-bit17为工具DO0-DO1 */
	int safetyBoardComSendCount;                      /* 安全板通信发送数据包计数 */
	int safetyBoardComRecvCount;                      /* 安全板通信接收数据包计数 */
	int toolNoSRL;                                    /* 工具编号，[-1~19] */
	int frameNOSRL;                                   /* 工件编号，[-1~19] */
	double robposCartTF[6];                           /* 对应工具工件下的笛卡尔位姿 */
	double robposJointTF[6];                          /* 对应工具工件下的关节位置 */
	uint8_t turn_num[4];                              /* 关节圈数 */
	uint8_t robot_config;                             /* 关节配置 */
	uint8_t  forceSensorErrState;                     /** 力传感器连接超时故障；bit0-bit1对应力传感器ID1-ID2 */
	uint8_t ctrlOpenLuaRunningState;  				  /** 控制器开放协议运行状态，bit0-bit3对应协议编号0-3的运行状态，0-未运行，1-运行中 */
	uint8_t ctrlOpenLuaErrCode[4];   				  /** 4个控制器外设协议错误码(500错误码)；*/
	uint8_t safetyBoxSignal[6];                       /** 按钮盒按钮信号 */
	float jointIdentifyData;  						  /** 单关节模型辨识时实时曲线显示数据 */
	uint8_t UDPConnState;                             /** UDP通讯连接状态：0-连接断开；1-连接成功 */ 
	float user_var[128];							  /** 用户变量，默认值0 */
	uint8_t run_status[8];							  /** 后台程序线程状态，0停止（未配置），1运行，2暂停 */
	char end[7];
} CTRL_STATE;
/***********end 8081 data*******************/

#endif
