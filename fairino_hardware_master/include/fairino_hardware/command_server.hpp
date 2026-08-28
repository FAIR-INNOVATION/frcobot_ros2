#ifndef _COMMAND_SERVER_
#define _COMMAND_SERVER_

#include "stdlib.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "fairino_msgs/srv/remote_script_content.hpp"
#include "fairino_msgs/srv/remote_cmd_interface.hpp"
#include "fairino_msgs/msg/robot_nonrt_state.hpp"
#include "mutex"
#include "sys/socket.h"
#include "sys/types.h"
#include "sys/mman.h"
#include "netinet/in.h"
#include "netinet/tcp.h"
#include "arpa/inet.h"
#include "fcntl.h"
#include <regex>
#include "data_type_def.h"
#include <queue>
#include <atomic>
#include "semaphore.h"

using remote_cmd_server_srv_msg = fairino_msgs::srv::RemoteCmdInterface;
using remote_script_srv_msg = fairino_msgs::srv::RemoteScriptContent;
using robot_feedback_msg = fairino_msgs::msg::RobotNonrtState;
#define REMOTE_CMD_SERVER_NAME  "fairino_remote_command_service"
#define REMOTE_SCRIPT_SERVER_NAME  "fairino_script_service"


class robot_command_thread:public rclcpp::Node{
public:
    robot_command_thread(const std::string node_name);
    ~robot_command_thread();
    
    //点位信息设置类
    std::string defJntPosition(std::string pos);
    std::string defCartPosition(std::string pos);
    std::string getVariable(std::string para_list);

    //信息获取类
    std::string GetVersion(std::string para);
    std::string GetMsgVersion(std::string para);
    std::string GetRobotVersion(std::string para);
    std::string GetControllerVersion(std::string para);
    std::string GetTCPOffset(std::string para);
    std::string GetDHCompensation(std::string para);
    std::string GetWeldingBreakOffState(std::string para);
    std::string GetErrorCode(std::string para);
    std::string GetInverseKin(std::string para);
    std::string GetSDKVersion(std::string para);
    std::string GetControllerIP(std::string para);
    std::string IsInDragTeach(std::string para);

    //普通设置类
    std::string DragTeachSwitch(std::string para);//拖动示教模式切换
    std::string RobotEnable(std::string para);//机械臂使能
    std::string Mode(std::string para);//手动模式，自动模式切换
    std::string SetSpeed(std::string para);
    std::string SetToolCoord(std::string para);
    std::string SetToolList(std::string para);
    std::string SetToolPoint(std::string para);
    std::string ComputeTool(std::string para);
    std::string SetTcp4RefPoint(std::string para);
    std::string ComputeTcp4(std::string para);

    std::string SetExToolCoord(std::string para);
    std::string SetExToolList(std::string para);
    std::string SetWObjCoord(std::string para);
    std::string SetWObjList(std::string para);
    std::string SetLoadWeight(std::string para);
    std::string SetLoadCoord(std::string para);
    std::string SetRobotInstallPos(std::string para);
    std::string SetRobotInstallAngle(std::string para);

    //安全配置
    std::string SetAnticollision(std::string para);
    std::string SetCollisionStrategy(std::string para);
    std::string SetLimitPositive(std::string para);
    std::string SetLimitNegtive(std::string para);
    std::string ResetAllError(std::string para);
    std::string FrictionCompensationOnOff(std::string para);
    std::string SetFrictionValue_level(std::string para);
    std::string SetFrictionValue_wall(std::string para);
    std::string SetFrictionValue_ceiling(std::string para);
    std::string SetFrictionValue_freedom(std::string para);

    //外设控制
    std::string ActGripper(std::string para);
    std::string MoveGripper(std::string para);
    
    //IO控制
    std::string SetDO(std::string para);
    std::string SetToolDO(std::string para);
    std::string SetAO(std::string para);
    std::string SetToolAO(std::string para);
    std::string SetAuxDO(std::string para);
    std::string SetAuxAO(std::string para);
    std::string GetDI(std::string para);
    std::string GetToolDI(std::string para);
    std::string WaitDI(std::string para);
    std::string WaitMultiDI(std::string para);
    std::string WaitToolDI(std::string para);
    std::string GetAI(std::string para);
    std::string GetToolAI(std::string para);

    //UDP扩展轴控制
    std::string ExtAxisServoOn(std::string para);
    std::string ExtAxisStartJog(std::string para);
    std::string ExtAxisSetHoming(std::string para);
    std::string StopExtAxisJog(std::string para);
    std::string ExtAxisGetCoord(std::string para);
    std::string ExtAxisMove(std::string para);
    std::string ExtAxisSyncMoveJ(std::string para);
    std::string ExtAxisSyncMoveL(std::string para);
    std::string ExtAxisSetRefPoint(std::string para);
    std::string ExtAxisComputeECoordSys(std::string para);
    std::string ExtAxisActiveECoordSys(std::string para);
    std::string SetAxisDHParaConfig(std::string para);
    std::string ExtDevLoadUDPDriver(std::string para);

    //运动指令
    std::string StartJOG(std::string para);
    std::string StopJOG(std::string para);
    std::string ImmStopJOG(std::string para);
    std::string MoveJ(std::string para);
    std::string MoveL(std::string para);
    std::string MoveC(std::string para);
    std::string Circle(std::string para);
    std::string ServoJ(std::string para);
    std::string SplineStart(std::string para);
    std::string SplinePTP(std::string para);
    std::string SplineEnd(std::string para);
    std::string NewSplineStart(std::string para);
    std::string NewSplinePoint(std::string para);
    std::string NewSplineEnd(std::string para);
    std::string StopMotion(std::string para);
    std::string PointsOffsetEnable(std::string para);
    std::string PointsOffsetDisable(std::string para);
    std::string NewSpiral(std::string para);
    std::string ServoMoveStart(std::string para);
    std::string ServoMoveEnd(std::string para);
    std::string ServoCart(std::string para);
    std::string MoveCart(std::string para);
    std::string PauseMotion(std::string para);
    std::string ResumeMotion(std::string para);

    //485扩展轴控制
    std::string AuxServoSetParam(std::string para);
    std::string AuxServoEnable(std::string para);
    std::string AuxServoSetControlMode(std::string para);
    std::string AuxServoSetTargetPos(std::string para);
    std::string AuxServoSetTargetSpeed(std::string para);
    std::string AuxServoSetTargetTorque(std::string para);
    std::string AuxServoHoming(std::string para);
    std::string AuxServoClearError(std::string para);
    std::string AuxServoSetStatusID(std::string para);

    //脚本控制指令
    std::string ScriptLoad(std::string para);
    std::string ScriptStart(std::string para);
    std::string ScriptStop(std::string para);
    std::string ScriptPause(std::string para);
    std::string ScriptResume(std::string para);

    //可移动装置
    std::string TractorEnable(std::string para);
    std::string TractorHoming(std::string para);
    std::string TractorMoveL(std::string para);
    std::string TractorMoveC(std::string para);
    std::string TractorStop(std::string para);
    
    //轨迹J功能
    std::string TrajectoryJUpLoad(std::string para);
    std::string TrajectoryJDelete(std::string para);
    std::string LoadTrajectoryJ(std::string para);
    std::string MoveTrajectoryJ(std::string para);
    std::string GetTrajectoryStartPose(std::string para);
    std::string GetTrajectoryPointNum(std::string para);
    std::string SetTrajectoryJSpeed(std::string para);

    //LUA脚本传输功能
    std::string LuaDownLoad(std::string para);
    std::string LuaUpload(std::string para);
    std::string LuaDelete(std::string para);
    std::string GetLuaList(std::string para);

    //点位换算功能
    std::string ComputeToolCoordWithPoints(std::string para);
    std::string ComputeWObjCoordWithPoints(std::string para);

    //焊接中断恢复功能
    std::string WeldingSetCurrent(std::string para);
    std::string WeldingSetVoltage(std::string para);
    std::string WeldingSetCheckArcInterruptionParam(std::string para);
    std::string WeldingGetCheckArcInterruptionParam(std::string para);
    std::string WeldingSetReWeldAfterBreakOffParam(std::string para);
    std::string WeldingGetReWeldAfterBreakOffParam(std::string para);
    std::string WeldingStartReWeldAfterBreakOff(std::string para);
    std::string WeldingAbortWeldAfterBreakOff(std::string para);
    

    //new-387
    std::string GetAxlePointRecordBtnState(std::string para);
    std::string GetToolDO(std::string para);
    std::string GetDO(std::string para);
    std::string WaitAI(std::string para);
    std::string WaitToolAI(std::string para);
    std::string SetSysVarValue(std::string para);
    std::string SetExTCPPoint(std::string para);
    std::string ComputeExTCF(std::string para);
    std::string SetWObjCoordPoint(std::string para);
    std::string ComputeWObjCoord(std::string para);
    std::string WaitMs(std::string para);
    std::string SetLimitNegative(std::string para);
    std::string GetRobotInstallAngle(std::string para);
    std::string GetSysVarValue(std::string para);
    std::string GetActualJointPosDegree(std::string para);
    std::string GetActualJointSpeedsDegree(std::string para);
    std::string GetActualJointAccDegree(std::string para);
    std::string GetTargetTCPCompositeSpeed(std::string para);
    std::string GetActualTCPCompositeSpeed(std::string para);
    std::string GetTargetTCPSpeed(std::string para);
    std::string GetActualTCPSpeed(std::string para);
    std::string GetActualTCPPose(std::string para);
    std::string GetActualTCPNum(std::string para);
    std::string GetActualWObjNum(std::string para);
    std::string GetActualToolFlangePose(std::string para);
    std::string GetInverseKinRef(std::string para);
    std::string GetInverseKinHasSolution(std::string para);
    std::string GetInverseKinExaxis(std::string para);
    std::string GetForwardKin(std::string para);
    std::string GetJointTorques(std::string para);
    std::string GetTargetPayload(std::string para);
    std::string GetTargetPayloadCog(std::string para);
    std::string GetWObjOffset(std::string para);
    std::string GetJointSoftLimitDeg(std::string para);
    std::string GetSystemClock(std::string para);
    std::string GetRobotCurJointsConfig(std::string para);
    std::string GetDefaultTransVel(std::string para);
    std::string GetRobotMotionDone(std::string para);
    std::string GetRobotErrorCode(std::string para);
    std::string GetRobotTeachingPoint(std::string para);
    std::string GetMotionQueueLength(std::string para);
    std::string SetTPDParam(std::string para);
    std::string SetTPDStart(std::string para);
    std::string SetWebTPDStop(std::string para);
    std::string SetTPDDelete(std::string para);
    std::string LoadTPD(std::string para);
    std::string GetTPDStartPose(std::string para);
    std::string MoveTPD(std::string para);
    std::string SetTrajectoryJForceTorque(std::string para);
    std::string SetTrajectoryJForceFx(std::string para);
    std::string SetTrajectoryJForceFy(std::string para);
    std::string SetTrajectoryJForceFz(std::string para);
    std::string SetTrajectoryJTorqueTx(std::string para);
    std::string SetTrajectoryJTorqueTy(std::string para);
    std::string SetTrajectoryJTorqueTz(std::string para);
    std::string LoadDefaultProgConfig(std::string para);
    std::string ProgramLoad(std::string para);
    std::string GetLoadedProgram(std::string para);
    std::string GetCurrentLine(std::string para);
    std::string ProgramRun(std::string para);
    std::string ProgramPause(std::string para);
    std::string ProgramResume(std::string para);
    std::string ProgramStop(std::string para);
    std::string GetProgramState(std::string para);
    std::string SetGripperConfig(std::string para);
    std::string GetGripperConfig(std::string para);
    std::string GetGripperMotionDone(std::string para);
    std::string GetGripperActivateStatus(std::string para);
    std::string GetGripperCurPosition(std::string para);
    std::string GetGripperCurSpeed(std::string para);
    std::string GetGripperCurCurrent(std::string para);
    std::string GetGripperVoltage(std::string para);
    std::string GetGripperTemp(std::string para);
    std::string GetGripperRotNum(std::string para);
    std::string GetGripperRotSpeed(std::string para);
    std::string GetGripperRotTorque(std::string para);
    std::string ComputePrePick(std::string para);
    std::string ComputePostPick(std::string para);
    std::string FT_SetConfig(std::string para);
    std::string FT_GetConfig(std::string para);
    std::string FT_Activate(std::string para);
    std::string FT_SetZero(std::string para);
    std::string FT_SetRCS(std::string para);
    std::string FT_PdIdenRecord(std::string para);
    std::string FT_PdIdenCompute(std::string para);
    std::string FT_PdCogIdenRecord(std::string para);
    std::string FT_PdCogIdenCompute(std::string para);
    std::string FT_GetForceTorqueRCS(std::string para);
    std::string FT_GetForceTorqueOrigin(std::string para);
    std::string FT_Guard(std::string para);
    std::string FT_Control(std::string para);
    std::string FT_SpiralSearch(std::string para);
    std::string FT_RotInsertion(std::string para);
    std::string FT_LinInsertion(std::string para);
    std::string FT_FindSurface(std::string para);
    std::string FT_CalCenterStart(std::string para);
    std::string FT_CalCenterEnd(std::string para);
    std::string FT_ComplianceStart(std::string para);
    std::string FT_ComplianceStop(std::string para);
    std::string LoadIdentifyDynFilterInit(std::string para);
    std::string LoadIdentifyDynVarInit(std::string para);
    std::string LoadIdentifyMain(std::string para);
    std::string LoadIdentifyGetResult(std::string para);
    std::string ConveyorStartEnd(std::string para);
    std::string ConveyorPointIORecord(std::string para);
    std::string ConveyorPointARecord(std::string para);
    std::string ConveyorRefPointRecord(std::string para);
    std::string ConveyorPointBRecord(std::string para);
    std::string ConveyorIODetect(std::string para);
    std::string ConveyorGetTrackData(std::string para);
    std::string ConveyorTrackStart(std::string para);
    std::string ConveyorTrackEnd(std::string para);
    std::string ConveyorSetParam(std::string para);
    std::string ConveyorCatchPointComp(std::string para);
    std::string TrackMoveL(std::string para);
    std::string GetSSHKeygen(std::string para);
    std::string SetSSHScpCmd(std::string para);
    std::string ComputeFileMD5(std::string para);
    std::string GetRobotEmergencyStopState(std::string para);
    std::string GetSDKComState(std::string para);
    std::string GetSafetyStopState(std::string para);
    std::string GetHardwareVersion(std::string para);
    std::string GetFirmwareVersion(std::string para);
    std::string PointTableSwitch(std::string para);
    std::string PointTableDownLoad(std::string para);
    std::string PointTableUpLoad(std::string para);
    std::string PointTableUpdateLua(std::string para);
    std::string WeaveSetPara(std::string para);
    std::string WeaveOnlineSetPara(std::string para);
    std::string WeaveStart(std::string para);
    std::string WeaveEnd(std::string para);
    std::string SegmentWeldStart(std::string para);
    std::string LoggerInit(std::string para);
    std::string SetLoggerLevel(std::string para);
    std::string AuxServoGetParam(std::string para);
    std::string AuxServoGetStatus(std::string para);
    std::string SetOaccScale(std::string para);
    std::string MoveAOStart(std::string para);
    std::string MoveAOStop(std::string para);
    std::string MoveToolAOStart(std::string para);
    std::string MoveToolAOStop(std::string para);
    std::string ExtDevUnloadUDPDriver(std::string para);
    std::string SetAuxDIFilterTime(std::string para);
    std::string SetAuxAIFilterTime(std::string para);
    std::string WaitAuxDI(std::string para);
    std::string WaitAuxAI(std::string para);
    std::string GetAuxDI(std::string para);
    std::string GetAuxAI(std::string para);
    std::string SetRefPointInExAxisEnd(std::string para);
    std::string PositionorSetRefPoint(std::string para);
    std::string PositionorComputeECoordSys(std::string para);
    std::string ExtAxisSyncMoveC(std::string para);
    std::string WireSearchStart(std::string para);
    std::string WireSearchEnd(std::string para);
    std::string GetWireSearchOffset(std::string para);
    std::string WireSearchWait(std::string para);
    std::string SetPointToDatabase(std::string para);
    std::string ArcWeldTraceControl(std::string para);
    std::string ArcWeldTraceExtAIChannelConfig(std::string para);
    std::string EndForceDragControl(std::string para);
    std::string SetForceSensorDragAutoFlag(std::string para);
    std::string GetForceAndTorqueDragState(std::string para);
    std::string SetForceSensorPayload(std::string para);
    std::string SetForceSensorPayloadCog(std::string para);
    std::string GetForceSensorPayload(std::string para);
    std::string GetForceSensorPayloadCog(std::string para);
    std::string ForceSensorAutoComputeLoad(std::string para);
    std::string ForceSensorSetSaveDataFlag(std::string para);
    std::string ForceSensorComputeLoad(std::string para);
    std::string GetSegmentWeldPoint(std::string para);
    std::string WeldingSetProcessParam(std::string para);
    std::string WeldingGetProcessParam(std::string para);
    std::string AxleSensorConfig(std::string para);
    std::string AxleSensorConfigGet(std::string para);
    std::string AxleSensorActivate(std::string para);
    std::string SetOutputResetCtlBoxDO(std::string para);
    std::string SetOutputResetCtlBoxAO(std::string para);
    std::string SetOutputResetAxleDO(std::string para);
    std::string SetOutputResetAxleAO(std::string para);
    std::string SetOutputResetExtDO(std::string para);
    std::string SetOutputResetExtAO(std::string para);
    std::string SetOutputResetSmartToolDO(std::string para);
    std::string WeaveStartSim(std::string para);
    std::string WeaveEndSim(std::string para);
    std::string WeaveInspectStart(std::string para);
    std::string WeaveInspectEnd(std::string para);
    std::string SetCollisionDetectionMethod(std::string para);
    std::string SetStaticCollisionOnOff(std::string para);
    std::string SetPowerLimit(std::string para);
    std::string ServoJTStart(std::string para);
    std::string ServoJT(std::string para);
    std::string ServoJTEnd(std::string para);
    std::string SetRobotRealtimeStateSamplePeriod(std::string para);
    std::string GetRobotRealtimeStateSamplePeriod(std::string para);
    std::string GetJointDriverTemperature(std::string para);
    std::string GetJointDriverTorque(std::string para);
    std::string ArcWeldTraceReplayStart(std::string para);
    std::string ArcWeldTraceReplayEnd(std::string para);
    std::string MultilayerOffsetTrsfToBase(std::string para);
    std::string AngularSpeedStart(std::string para);
    std::string AngularSpeedEnd(std::string para);
    std::string SoftwareUpgrade(std::string para);
    std::string GetSoftwareUpgradeState(std::string para);
    std::string GetAxleCommunicationParam(std::string para);
    std::string SetAxleCommunicationParam(std::string para);
    std::string SetAxleFileType(std::string para);
    std::string SetAxleLuaEnable(std::string para);
    std::string SetRecoverAxleLuaErr(std::string para);
    std::string GetAxleLuaEnableStatus(std::string para);
    std::string SetAxleLuaEnableDeviceType(std::string para);
    std::string GetAxleLuaEnableDeviceType(std::string para);
    std::string GetAxleLuaEnableDevice(std::string para);
    std::string SetAxleLuaGripperFunc(std::string para);
    std::string GetAxleLuaGripperFunc(std::string para);
    std::string SetCtrlOpenLuaErrCode(std::string para);
    std::string SlaveFileWrite(std::string para);
    std::string AxleLuaUpload(std::string para);
    std::string SetSysServoBootMode(std::string para);
    std::string SetWireSearchExtDIONum(std::string para);
    std::string SetWeldMachineCtrlModeExtDoNum(std::string para);
    std::string SetWeldMachineCtrlMode(std::string para);
    std::string SingularAvoidStart(std::string para);
    std::string SingularAvoidEnd(std::string para);
    std::string PtpFIRPlanningStart(std::string para);
    std::string PtpFIRPlanningEnd(std::string para);
    std::string LinArcFIRPlanningStart(std::string para);
    std::string LinArcFIRPlanningEnd(std::string para);
    std::string LaserSensorRecord(std::string para);
    std::string LaserTrackingLaserOn(std::string para);
    std::string LaserTrackingLaserOff(std::string para);
    std::string LaserTrackingTrackOn(std::string para);
    std::string LaserTrackingTrackOff(std::string para);
    std::string LaserTrackingSearchStart(std::string para);
    std::string LaserTrackingLaserOnOff(std::string para);
    std::string LaserTrackingTrackOnOff(std::string para);
    std::string LaserTrackingSearchStart_xyz(std::string para);
    std::string LaserTrackingSearchStart_point(std::string para);
    std::string LaserTrackingSearchStop(std::string para);
    std::string LaserTrackingSensorConfig(std::string para);
    std::string LaserTrackingSensorSamplePeriod(std::string para);
    std::string LoadPosSensorDriver(std::string para);
    std::string UnLoadPosSensorDriver(std::string para);
    std::string LaserSensorRecord1(std::string para);
    std::string LaserSensorReplay(std::string para);
    std::string MoveLTR(std::string para);
    std::string LaserSensorRecordandReplay(std::string para);
    std::string MoveToLaserRecordStart(std::string para);
    std::string MoveToLaserRecordEnd(std::string para);
    std::string MoveToLaserSeamPos(std::string para);
    std::string GetLaserSeamPos(std::string para);
    std::string WeaveChangeStart(std::string para);
    std::string WeaveChangeEnd(std::string para);
    std::string LoadTrajectoryLA(std::string para);
    std::string MoveTrajectoryLA(std::string para);
    std::string CustomCollisionDetectionStart(std::string para);
    std::string CustomCollisionDetectionEnd(std::string para);
    std::string AccSmoothStart(std::string para);
    std::string AccSmoothEnd(std::string para);
    std::string RbLogDownload(std::string para);
    std::string AllDataSourceDownload(std::string para);
    std::string DataPackageDownload(std::string para);
    std::string GetRobotSN(std::string para);
    std::string ShutDownRobotOS(std::string para);
    std::string ConveyorComDetect(std::string para);
    std::string ConveyorComDetectTrigger(std::string para);
    std::string WeldingSetVoltageGradualChangeStart(std::string para);
    std::string WeldingSetVoltageGradualChangeEnd(std::string para);
    std::string WeldingSetCurrentGradualChangeStart(std::string para);
    std::string WeldingSetCurrentGradualChangeEnd(std::string para);
    std::string GetSmarttoolBtnState(std::string para);
    std::string SetWideBoxTempFanMonitorParam(std::string para);
    std::string GetWideBoxTempFanMonitorParam(std::string para);
    std::string SetFocusCalibPoint(std::string para);
    std::string ComputeFocusCalib(std::string para);
    std::string FocusStart(std::string para);
    std::string FocusEnd(std::string para);
    std::string SetFocusPosition(std::string para);
    std::string SetEncoderUpgrade(std::string para);
    std::string SetJointFirmwareUpgrade(std::string para);
    std::string SetCtrlFirmwareUpgrade(std::string para);
    std::string SetEndFirmwareUpgrade(std::string para);
    std::string JointAllParamUpgrade(std::string para);
    std::string SetRobotType(std::string para);
    std::string LaserRecordPoint(std::string para);
    std::string SetExAxisRobotPlan(std::string para);
    std::string SetReConnectParam(std::string para);
    std::string GetFieldBusConfig(std::string para);
    std::string FieldBusSlaveWriteDO(std::string para);
    std::string FieldBusSlaveWriteAO(std::string para);
    std::string FieldBusSlaveReadDI(std::string para);
    std::string FieldBusSlaveReadAI(std::string para);
    std::string FieldBusSlaveWaitDI(std::string para);
    std::string FieldBusSlaveWaitAI(std::string para);
    std::string SetSuckerCtrl(std::string para);
    std::string GetSuckerState(std::string para);
    std::string WaitSuckerState(std::string para);
    std::string OpenLuaUpload(std::string para);
    std::string ImpedanceControlStartStop(std::string para);
    std::string SetTorqueDetectionSwitch(std::string para);
    std::string GetToolCoordWithID(std::string para);
    std::string GetWObjCoordWithID(std::string para);
    std::string GetExToolCoordWithID(std::string para);
    std::string GetExAxisCoordWithID(std::string para);
    std::string GetTargetPayloadWithID(std::string para);
    std::string GetCurToolCoord(std::string para);
    std::string GetCurWObjCoord(std::string para);
    std::string GetCurExToolCoord(std::string para);
    std::string GetCurExAxisCoord(std::string para);
    std::string KernelUpgrade(std::string para);
    std::string GetKernelUpgradeResult(std::string para);
    std::string CustomWeaveSetPara(std::string para);
    std::string CustomWeaveGetPara(std::string para);
    std::string JointSensitivityEnable(std::string para);
    std::string JointSensitivityCalibration(std::string para);
    std::string JointSensitivityCollect(std::string para);
    std::string Sleep(std::string para);
    std::string MotionQueueClear(std::string para);
    std::string GetSlavePortErrCounter(std::string para);
    std::string SlavePortErrCounterClear(std::string para);
    std::string SetVelFeedForwardRatio(std::string para);
    std::string GetVelFeedForwardRatio(std::string para);
    std::string RobotMCULogCollect(std::string para);
    std::string MoveToIntersectLineStart(std::string para);
    std::string MoveIntersectLine(std::string para);
    std::string JointHysteresisError(std::string para);
    std::string JointRepeatability(std::string para);
    std::string SetAdmittanceParams(std::string para);
    std::string SerCoderCompenParams(std::string para);
    std::string TCPComputeRPY(std::string para);
    std::string TCPComputeXYZ(std::string para);
    std::string TCPRecordFlangePosStart(std::string para);
    std::string TCPRecordFlangePosEnd(std::string para);
    std::string TCPGetRecordFlangePos(std::string para);

    //力控功能
    // std::string FT_InsertComp(std::string para);

    // //TCF光点标定
    std::string PhotoelectricSensorTCPCalibration(std::string para);


    std::string MoveStationary(std::string para);
    std::string GetProgramRunErrCode(std::string para);
    std::string SetAxleGenComEnable(std::string para);
    std::string GetAxleGenComCycleData(std::string para);
    std::string SndRcvAxleGenComCmdData(std::string para);
    std::string SetRobotStopOnComDisc(std::string para);
    std::string GetRobotStopOnComDisc(std::string para);


    std::string SetDIConfig(std::string para);
    std::string GetDIConfig(std::string para);
    std::string SetDOConfig(std::string para);
    std::string GetDOConfig(std::string para);
    std::string SetToolDIConfig(std::string para);
    std::string GetToolDIConfig(std::string para);
    std::string SetDIConfigLevel(std::string para);
    std::string GetDIConfigLevel(std::string para);
    std::string SetDOConfigLevel(std::string para);
    std::string GetDOConfigLevel(std::string para);
    std::string SetToolDIConfigLevel(std::string para);
    std::string GetToolDIConfigLevel(std::string para);
    std::string SetStandardDILevel(std::string para);
    std::string GetStandardDILevel(std::string para);
    std::string SetStandardDOLevel(std::string para);
    std::string GetStandardDOLevel(std::string para);
    std::string SetExAxisCmdDoneTime(std::string para);
    std::string OpenLuaDownload(std::string para);    
    std::string ExtDevGetUDPComParam(std::string para); 
    std::string ExtDevSetUDPComParam(std::string para); 
    std::string SetRobotPosToAxis(std::string para); 
    std::string ExtAxisParamConfig(std::string para); 
    std::string WeldingSetCurrentRelation(std::string para);
    std::string WeldingSetVoltageRelation(std::string para);
    std::string ARCStart(std::string para);
    std::string ARCEnd(std::string para);
    std::string SetForwardWireFeed(std::string para);
    std::string SetReverseWireFeed(std::string para);
    std::string ArcWeldTraceAIChannelCurrent(std::string para); 
    std::string ArcWeldTraceAIChannelVoltage(std::string para);
    std::string ArcWeldTraceCurrentPara(std::string para);
    std::string ArcWeldTraceVoltagePara(std::string para);
    std::string SetArcDoneExtDiNum(std::string para);
    std::string SetWeldReadyExtDiNum(std::string para);
    std::string SetExtDIWeldBreakOffRecover(std::string para);
    std::string SetWireReverseFeedExtDoNum(std::string para);
    std::string SetWireForwardFeedExtDoNum(std::string para);
    std::string SetAirControlExtDoNum(std::string para);
    std::string SetArcStartExtDoNum(std::string para);
    std::string SetCtrlOpenLUAName(std::string para);
    std::string SetExDevProtocol(std::string para);
    std::string LoadCtrlOpenLUA(std::string para);
    std::string UnloadCtrlOpenLUA(std::string para);
    std::string AuxServoSetAcc(std::string para);
    std::string AuxServoSetEmergencyStopAcc(std::string para);
    std::string WeldingGetCurrentRelation(std::string para);
    std::string WeldingGetVoltageRelation(std::string para);
    std::string GetExDevProtocol(std::string para);
    std::string AuxServoGetAcc(std::string para);
    std::string AuxServoGetEmergencyStopAcc(std::string para);
    std::string SetAspirated(std::string para);
    std::string ExtDevUDPClientComReset(std::string para);
    std::string ExtDevUDPClientComClose(std::string para);
    std::string GetCtrlOpenLUAName(std::string para);
    std::string OpenLuaDelete(std::string para);
    std::string AllOpenLuaDelete(std::string para);
    std::string SendUDPFrame(std::string para);
    std::string SetVelReducePara(std::string para);
    std::string OriginPointWeaveStart(std::string para);
    std::string OriginPointWeaveEnd(std::string para);
    std::string SetUserLEDColor(std::string para);
    std::string MoveToTPDStart(std::string para);
    std::string SetRobotRealtimeStatePeriod(std::string para);
    std::string ServoJV(std::string para);
    std::string ServoMITStart(std::string para);
    std::string ServoMITEnd(std::string para);
    std::string ServoMIT(std::string para);
    std::string SetLaserWeldingParam(std::string para);
    std::string SetLaserWeldingStartEnd(std::string para);
    std::string SetLaserWeldingEnable(std::string para);
    std::string ResetLaserWeldingErr(std::string para);
    std::string GetLaserWeldingRunningState(std::string para);
    std::string GetLaserWeldingErrState(std::string para);
    std::string GetLaserWeldingParamTarget(std::string para);
    std::string GetLaserWeldingParamActual(std::string para);
    std::string SetLaserWeldingEnableExtDoNum(std::string para);
    std::string SetLaserWeldingStartExtDoNum(std::string para);
    std::string SetLaserWeldingErrResetExtDoNum(std::string para);
    std::string SetLaserWeldingRunningStateExtDiNum(std::string para);
    std::string SetLaserWeldingErrStateExtDiNum(std::string para);
    std::string ExtAxisGetParamConfig(std::string para);
    std::string Slave(std::string para);
    std::string FeedBackIDSet(std::string para);


    void init_master_client_1();
    void init_master_client_2();


    rclcpp::TimerBase::SharedPtr reconnect_timer_1_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_2_;

    int get_current_domain_id();

    int current_domain_id_;


private:

    // 从站机器人控制相关资源
    std::shared_ptr<rclcpp::Context> master_context_1_;
    rclcpp::Node::SharedPtr master_node_1_;
    rclcpp::Client<remote_cmd_server_srv_msg>::SharedPtr master_client_1_;
    rclcpp::Subscription<robot_feedback_msg>::SharedPtr master_sub_1_;
    std::mutex msg_mutex_1_;
    std::shared_ptr<robot_feedback_msg> msg_sub_1_;
    std::atomic<bool> master_client_ready_1_{false};
    std::thread spin_thread_1_;

    std::shared_ptr<rclcpp::Context> master_context_2_;
    rclcpp::Node::SharedPtr master_node_2_;
    rclcpp::Client<remote_cmd_server_srv_msg>::SharedPtr master_client_2_;
    rclcpp::Subscription<robot_feedback_msg>::SharedPtr master_sub_2_;
    std::mutex msg_mutex_2_;
    std::shared_ptr<robot_feedback_msg>  msg_sub_2_;
    std::atomic<bool> master_client_ready_2_{false};
    std::thread spin_thread_2_;

    std::atomic<int> sub_domain_id = 1;//状态反馈数据domain id设置
    std::atomic<bool> master_client_ready_{false};
    rclcpp::Client<remote_cmd_server_srv_msg>::SharedPtr master_client_;

    
    void _state_recv_callback();
    rclcpp::Publisher<robot_feedback_msg>::SharedPtr _state_publisher;//进程内通信，用于发送状态数据字符串
    rclcpp::TimerBase::SharedPtr _locktimer1;


    void _ping_recv_callback();
    rclcpp::TimerBase::SharedPtr _locktimer2;
    bool _check_ping(const std::string& ip_address);


    rclcpp::TimerBase::SharedPtr _locktimer;

    int lose_connect_times = 0;
    int _connect_retry_SDK = 5;
    //函数指针是有作用域的，所以全局函数的指针和类内成员函数的指针定义有很大不同，这里不能用typedef
    int (robot_command_thread:: *funcP)(std::string para);

    //用于解析用户发送的ROS接口指令
    void _parseROSCommandData_callback(const std::shared_ptr<remote_cmd_server_srv_msg::Request> req,
                                    std::shared_ptr<remote_cmd_server_srv_msg::Response> res);
    void _timer_callback();
    void _splitString2List(std::string str,std::list<std::string> &list_data);
    void _splitString2Vec(std::string str,std::vector<std::string> &vector_data);
    //TODO 使用可变参数模板函数去填装SDK函数所需参数
    // template<typename T,typename ... Ts>
    // void _recurseVar(T& first_arg,Ts&... args);

    std::string _cur_func_name;
    int _cur_id;
    std::string _recv_get_data_res;//接受到的回复信息中指令反馈结果
    std::string _start_recv_res;//接受到的回复信息中指令反馈结果(start特殊情况，返回结果可能不是0和1会存在错误码)
    float _kin_res[6];
    //用于接受用户发送过来的字符串指令的service
    rclcpp::Service<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr _recv_ros_command_server;
    //用于接受用户发送过来的脚本service
    rclcpp::Service<fairino_msgs::srv::RemoteScriptContent>::SharedPtr _recv_ros_script_server;
    int _robot_install;//机械臂安装方式
    std::string _controller_ip;
    const std::map<std::string,std::string(robot_command_thread::*)(std::string)> _fr_function_list{
    {"GetVersion",&robot_command_thread::GetVersion},
    {"GetMsgVersion",&robot_command_thread::GetMsgVersion},
    {"GetRobotVersion",&robot_command_thread::GetRobotVersion},
    {"GetControllerVersion",&robot_command_thread::GetControllerVersion},
    {"Slave",&robot_command_thread::Slave},
    {"FeedBackIDSet",&robot_command_thread::FeedBackIDSet}
    };
};



#endif
