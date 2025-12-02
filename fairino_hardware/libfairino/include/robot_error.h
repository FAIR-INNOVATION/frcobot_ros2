#ifndef   ROBOTERROR_H_
#define   ROBOTERROR_H_

#define     ERR_UPLOAD_FILE_NOT_FOUND              -7
#define     ERR_SAVE_FILE_PATH_NOT_FOUND           -6
#define     ERR_LUA_FILE_NOT_FOUND                 -5
#define     ERR_XMLRPC_CMD_FAILED                  -4      /* xmlrpc接口执行失败，请联系售后工程师 */
#define     ERR_XMLRPC_COM_FAILED                  -3      /*xmlrpc通讯失败，请检查网络连接以及服务器IP地址是否正确*/
#define     ERR_SOCKET_COM_FAILED                  -2      /*网络通讯异常*/
#define     ERR_OTHER                              -1      /*其他错误*/
#define     ERR_SUCCESS                            0       /*成功*/
#define     ERR_PARAM_NUM                          3       /*参数个数不一致*/
#define     ERR_PARAM_VALUE                        4       /*参数值不在合理范围*/
#define     ERR_TPD_FILE_OPEN_FAILED               8       /*文件打开失败*/
#define     ERR_EXECUTION_FAILED                   14      /*指令执行失败*/
#define     ERR_PROGRAM_IS_RUNNING                 18      /*程序正在运行*/
#define     ERR_COMPUTE_FAILED                     25      /*计算失败*/
#define     ERR_INVERSE_KINEMATICS_COMPUTE_FAILED  28      /*逆运动学计算失败*/
#define     ERR_SERVOJ_JOINT_OVERRUN               29      /*关节值超限*/
#define     ERR_NON_RESSETTABLE_FAULT              30      /*不可复位故障，请断电重启控制箱*/
#define     ERR_EXTAXIS_CONFIG_FAILURE             33      /*外部轴未处于零位，导程、分辨率设置失败*/
#define     ERR_WORKPIECE_NUM                      34      /*工件号不对*/
#define     ERR_FILENAME_TOO_LONG                  36      /*文件名太长*/
#define     ERR_STRANGE_POSE                       38      /*奇异位姿*/
#define     ERR_EXTAXIS_NOT_HOMING				   41      /*外部轴未回零*/
#define     ERR_EXTAXIS_NOT_ACTIVING			   45	   /*外部轴未激活*/
#define     ERR_EXTAXIS_NOT_CALIB				   46	   /*同步功能需要标定外部轴*/
#define     ERR_EXTAXIS_SERVO_CONFIG_FAIL		   47	   /*外部驱动器信息配置失败*/
#define     ERR_EXTAXIS_SERVO_CONFIG_OVER		   48	   /*外部轴驱动器信息获取超时*/
#define     ERR_EXTAXIS_NOT_STEP_OPERATE		   52	   /*同步功能不能使用单步操作*/
#define     ERR_NOT_ADD_CMD_QUEUE                  64      /*未加入指令队列*/
#define     ERR_CIRCLE_SPIRAL_MIDDLE_POINT1        66      /*整圆中间点1错误*/
#define     ERR_CIRCLE_SPIRAL_MIDDLE_POINT2        67      /*整圆中间点2错误*/
#define     ERR_CIRCLE_SPIRAL_MIDDLE_POINT3        68      /*整圆中间点3错误*/
#define     ERR_MOVEC_MIDDLE_POINT                 69      /*圆弧中间点错误*/
#define     ERR_MOVEC_TARGET_POINT                 70      /*圆弧目标点错误*/
#define     ERR_GRIPPER_MOTION                     73      /*夹爪运动错误*/
#define     ERR_LINE_POINT                         74      /*直线目标点错误*/
#define     ERR_CHANNEL_FAULT                      75      /*通道错误*/
#define     ERR_WAIT_TIMEOUT                       76      /*等待超时*/
#define     ERR_TPD_CMD_POINT                      82      /*TPD指令点错误*/
#define     ERR_TPD_CMD_TOOL                       83      /*TPD工具号错误*/
#define     ERR_SPLINE_POINT                       94      /*样条指令点错误*/
#define     ERR_SPIRAL_START_POINT                 108     /*螺旋线起始点错误*/
#define     ERR_TARGET_POSE_CANNOT_REACHED         112     /*目标位姿无法到达*/
#define     ERR_POINTTABLE_NOTFOUND                130

#endif
