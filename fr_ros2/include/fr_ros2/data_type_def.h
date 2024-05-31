#ifndef _DATA_TYPE_DEF_H
#define _DATA_TYPE_DEF_H
#include "map"
#include "string"

#pragma pack(1)

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



typedef struct _FR_nonrt_state{
    uint16_t head;
    uint8_t count;
    uint16_t len;
    uint8_t prg_state;
    uint8_t error_code;
    uint8_t robot_mode;
    double j1_cur_pos;
    double j2_cur_pos;
    double j3_cur_pos;
    double j4_cur_pos;
    double j5_cur_pos;
    double j6_cur_pos;
    double cart_x_cur_pos;
    double cart_y_cur_pos;
    double cart_z_cur_pos;
    double cart_a_cur_pos;
    double cart_b_cur_pos;
    double cart_c_cur_pos;
    int tool_num;
    double j1_cur_tor;
    double j2_cur_tor;
    double j3_cur_tor;
    double j4_cur_tor;
    double j5_cur_tor;
    double j6_cur_tor;
    char prg_name[20];
    uint8_t prg_total_line;
    uint8_t prg_cur_line;
    uint8_t dgt_output_h;//DO 15-8
    uint8_t dgt_output_l;//DO 7-0
    uint8_t tl_dgt_output_l;//TOOL DO 7-0
    uint8_t dgt_input_h;//DI 15-8
    uint8_t dgt_input_l;//DI 7-0
    uint8_t tl_dgt_input_l;//TOOL DI 7-0
    double FT_Fx_data;
    double FT_Fy_data;
    double FT_Fz_data;
    double FT_Tx_data;
    double FT_Ty_data;
    double FT_Tz_data;
    uint8_t FT_ActStatus;
    uint8_t EMG;
    int robot_motion_done;
    uint8_t grip_motion_done;
    EXTERNALAXIS_STATUS exaxis1;
    EXTERNALAXIS_STATUS exaxis2;
    EXTERNALAXIS_STATUS exaxis3;
    EXTERNALAXIS_STATUS exaxis4;
    uint16_t check_sum;
}FR_nonrt_state;


typedef struct _rt_state{
    
}FR_rt_state;
#pragma pack()

typedef struct{
    double x;
    double y;
    double z;
}DescTran;

typedef struct{
    double rx;
    double ry;
    double rz;
}Rpy;

typedef struct{
    double jPos[6];
}JointPos;


typedef struct{
    double ePos[4];
}ExaxisPos;

typedef struct{
    DescTran tran;
    Rpy rpy;
}DescPose;

typedef struct{
    double fx;
    double fy;
    double fz;
    double tx;
    double ty;
    double tz;
}ForceTorque;


typedef struct{
    int circle_num;
    float circle_angle;
    float rad_init;
    float rad_add;
    float rotaxis_add;
    unsigned int rot_direction;
}SpiralParam;
#pragma pack()

#endif