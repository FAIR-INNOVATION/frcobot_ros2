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
#include "netinet/in.h"
#include "netinet/tcp.h"
#include "arpa/inet.h"
#include "fcntl.h"
#include <regex>
#include "data_type_def.h"
#include <queue>
#include "libfairino/include/robot.h"
#include <atomic>

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
    std::string SetToolPoint(std::string para);
    std::string ComputeTool(std::string para);
    std::string SetTcp4RefPoint(std::string para);
    std::string ComputeTcp4(std::string para);
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
    std::string ARCStart(std::string para);
    std::string ARCEnd(std::string para);
    std::string WeldingSetCurrentRelation(std::string para);
    std::string WeldingSetVoltageRelation(std::string para);
    std::string WeldingGetCurrentRelation(std::string para);
    std::string WeldingGetVoltageRelation(std::string para);
    std::string WeldingSetCurrent(std::string para);
    std::string WeldingSetVoltage(std::string para);
    std::string WeaveSetPara(std::string para);
    std::string WeaveOnlineSetPara(std::string para);
    std::string WeaveStart(std::string para);
    std::string WeaveEnd(std::string para);
    std::string SetForwardWireFeed(std::string para);
    std::string SetReverseWireFeed(std::string para);
    std::string SetAspirated(std::string para);
    std::string SegmentWeldStart(std::string para);
    std::string LoggerInit(std::string para);
    std::string SetLoggerLevel(std::string para);
    std::string AuxServoGetParam(std::string para);
    std::string AuxServoGetStatus(std::string para);
    std::string GetExDevProtocol(std::string para);
    std::string SetExDevProtocol(std::string para);
    std::string SetOaccScale(std::string para);
    std::string MoveAOStart(std::string para);
    std::string MoveAOStop(std::string para);
    std::string MoveToolAOStart(std::string para);
    std::string MoveToolAOStop(std::string para);
    std::string ExtDevSetUDPComParam(std::string para);
    std::string ExtDevGetUDPComParam(std::string para);
    std::string ExtDevLoadUDPDriver(std::string para);
    std::string ExtDevUnloadUDPDriver(std::string para);
    std::string SetAuxDIFilterTime(std::string para);
    std::string SetAuxAIFilterTime(std::string para);
    std::string WaitAuxDI(std::string para);
    std::string WaitAuxAI(std::string para);
    std::string GetAuxDI(std::string para);
    std::string GetAuxAI(std::string para);
    std::string ExtDevUDPClientComReset(std::string para);
    std::string ExtDevUDPClientComClose(std::string para);
    std::string ExtAxisParamConfig(std::string para);
    std::string SetRobotPosToAxis(std::string para);
    std::string SetAxisDHParaConfig(std::string para);
    std::string ExtAxisSetRefPoint(std::string para);
    std::string ExtAxisComputeECoordSys(std::string para);
    std::string ExtAxisActiveECoordSys(std::string para);
    std::string SetRefPointInExAxisEnd(std::string para);
    std::string PositionorSetRefPoint(std::string para);
    std::string PositionorComputeECoordSys(std::string para);
    std::string ExtAxisSyncMoveL(std::string para);
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
    std::string SetAirControlExtDoNum(std::string para);
    std::string SetArcStartExtDoNum(std::string para);
    std::string SetWireReverseFeedExtDoNum(std::string para);
    std::string SetWireForwardFeedExtDoNum(std::string para);
    std::string SetArcDoneExtDiNum(std::string para);
    std::string SetWeldReadyExtDiNum(std::string para);
    std::string SetExtDIWeldBreakOffRecover(std::string para);
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
    std::string AuxServoSetAcc(std::string para);
    std::string AuxServoSetEmergencyStopAcc(std::string para);
    std::string AuxServoGetAcc(std::string para);
    std::string AuxServoGetEmergencyStopAcc(std::string para);
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
    std::string SetCtrlOpenLUAName(std::string para);
    std::string GetCtrlOpenLUAName(std::string para);
    std::string LoadCtrlOpenLUA(std::string para);
    std::string UnloadCtrlOpenLUA(std::string para);
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
    std::string ArcWeldTraceAIChannelCurrent(std::string para);
    std::string ArcWeldTraceAIChannelVoltage(std::string para);
    std::string ArcWeldTraceCurrentPara(std::string para);
    std::string ArcWeldTraceVoltagePara(std::string para);
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
    std::string PhotoelectricSensorTCPCalibration(std::string para);
    std::string MoveStationary(std::string para);
private:

    
    void _state_recv_callback();
    rclcpp::Publisher<robot_feedback_msg>::SharedPtr _state_publisher;//进程内通信，用于发送状态数据字符串
    rclcpp::TimerBase::SharedPtr _locktimer1;


    std::unique_ptr<FRRobot> _ptr_robot;//机械臂SDK库指针
    ROBOT_STATE_PKG _robot_realtime_state;//从SDK获取的机械臂实时状态结构体
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
    void _fillDescPose(std::list<std::string>& data,DescPose& pose);
    void _fillDescTran(std::list<std::string>& data,DescTran& trans);
    void _fillJointPose(std::list<std::string>& data,JointPos& pos);
    void _getRobotRTState();
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
    std::vector<JointPos> _cmd_jnt_pos_list;//存储关节数据点
    std::vector<DescPose> _cmd_cart_pos_list;//存储笛卡尔数据点
    std::string _controller_ip;
    const std::map<std::string,std::string(robot_command_thread::*)(std::string)> _fr_function_list{
    {"JNTPoint",&robot_command_thread::defJntPosition},
    {"CARTPoint",&robot_command_thread::defCartPosition},
    {"GET",&robot_command_thread::getVariable},
    {"GetVersion",&robot_command_thread::GetVersion},
    {"GetMsgVersion",&robot_command_thread::GetMsgVersion},
    {"GetRobotVersion",&robot_command_thread::GetRobotVersion},
    {"GetControllerVersion",&robot_command_thread::GetControllerVersion},
    {"GetWeldingBreakOffState",&robot_command_thread::GetWeldingBreakOffState},
    {"GetErrorCode",&robot_command_thread::GetErrorCode},
    {"GetInverseKin",&robot_command_thread::GetInverseKin},
    {"DragTeachSwitch",&robot_command_thread::DragTeachSwitch},
    {"RobotEnable",&robot_command_thread::RobotEnable},
    {"SetSpeed",&robot_command_thread::SetSpeed},
    {"Mode",&robot_command_thread::Mode},
    {"SetToolCoord",&robot_command_thread::SetToolCoord},
    {"SetToolList",&robot_command_thread::SetToolList},
    {"SetExToolCoord",&robot_command_thread::SetExToolCoord},
    {"SetExToolList",&robot_command_thread::SetExToolList},
    {"SetWObjCoord",&robot_command_thread::SetWObjCoord},
    {"SetWObjList",&robot_command_thread::SetWObjList},
    {"SetLoadWeight",&robot_command_thread::SetLoadWeight},
    {"SetLoadCoord",&robot_command_thread::SetLoadCoord},
    {"SetRobotInstallPos",&robot_command_thread::SetRobotInstallPos},
    {"SetRobotInstallAngle",&robot_command_thread::SetRobotInstallAngle},
    {"SetAnticollision",&robot_command_thread::SetAnticollision},
    {"SetCollisionStrategy",&robot_command_thread::SetCollisionStrategy},
    {"SetLimitPositive",&robot_command_thread::SetLimitPositive},
    {"SetLimitNegtive",&robot_command_thread::SetLimitNegtive},
    {"ResetAllError",&robot_command_thread::ResetAllError},
    {"FrictionCompensationOnOff",&robot_command_thread::FrictionCompensationOnOff},
    {"SetFrictionValue_level",&robot_command_thread::SetFrictionValue_level},
    {"SetFrictionValue_wall",&robot_command_thread::SetFrictionValue_wall},
    {"SetFrictionValue_ceiling",&robot_command_thread::SetFrictionValue_ceiling},
    {"SetFrictionValue_freedom",&robot_command_thread::SetFrictionValue_freedom},
    {"ActGripper",&robot_command_thread::ActGripper},
    {"MoveGripper",&robot_command_thread::MoveGripper},
    {"SetDO",&robot_command_thread::SetDO},
    {"SetToolDO",&robot_command_thread::SetToolDO},
    {"SetAO",&robot_command_thread::SetAO},
    {"SetToolAO",&robot_command_thread::SetToolAO},
    {"SetAuxDO",&robot_command_thread::SetAuxDO},
    {"SetAuxAO",&robot_command_thread::SetAuxAO},
    {"ExtAxisServoOn",&robot_command_thread::ExtAxisServoOn},
    {"ExtAxisStartJog",&robot_command_thread::ExtAxisStartJog},
    {"ExtAxisSetHoming",&robot_command_thread::ExtAxisSetHoming},
    {"ExtAxisSyncMoveJ",&robot_command_thread::ExtAxisSyncMoveJ},
    {"StopExtAxisJog",&robot_command_thread::StopExtAxisJog},
    {"ExtAxisGetCoord",&robot_command_thread::ExtAxisGetCoord},
    {"ExtAxisMove",&robot_command_thread::ExtAxisMove},
    {"StartJOG",&robot_command_thread::StartJOG},
    {"StopJOG",&robot_command_thread::StopJOG},
    {"ImmStopJOG",&robot_command_thread::ImmStopJOG},
    {"MoveJ",&robot_command_thread::MoveJ},
    {"MoveL",&robot_command_thread::MoveL},
    {"MoveC",&robot_command_thread::MoveC},
    {"Circle",&robot_command_thread::Circle},
    {"ServoJ",&robot_command_thread::ServoJ},
    {"SplineStart",&robot_command_thread::SplineStart},
    {"SplinePTP",&robot_command_thread::SplinePTP},
    {"SplineEnd",&robot_command_thread::SplineEnd},
    {"NewSplineStart",&robot_command_thread::NewSplineStart},
    {"NewSplinePoint",&robot_command_thread::NewSplinePoint},
    {"NewSplineEnd",&robot_command_thread::NewSplineEnd},
    {"StopMotion",&robot_command_thread::StopMotion},
    {"PointsOffsetEnable",&robot_command_thread::PointsOffsetEnable},
    {"PointsOffsetDisable",&robot_command_thread::PointsOffsetDisable},
    {"ScriptLoad",&robot_command_thread::ScriptLoad},
    {"ScriptStart",&robot_command_thread::ScriptStart},
    {"ScriptStop",&robot_command_thread::ScriptStop},
    {"ScriptPause",&robot_command_thread::ScriptPause},
    {"ScriptResume",&robot_command_thread::ScriptResume},
    {"AuxServoSetParam", &robot_command_thread::AuxServoSetParam},
    {"AuxServoEnable", &robot_command_thread::AuxServoEnable},
    {"AuxServoSetControlMode", &robot_command_thread::AuxServoSetControlMode},
    {"AuxServoSetTargetPos", &robot_command_thread::AuxServoSetTargetPos},
    {"AuxServoSetTargetSpeed", &robot_command_thread::AuxServoSetTargetSpeed},
    {"AuxServoSetTargetTorque", &robot_command_thread::AuxServoSetTargetTorque},
    {"AuxServoHoming", &robot_command_thread::AuxServoHoming},
    {"AuxServoClearError", &robot_command_thread::AuxServoClearError},
    {"AuxServoSetStatusID", &robot_command_thread::AuxServoSetStatusID},
    {"GetTCPOffset",&robot_command_thread::GetTCPOffset},
    {"GetDHCompensation",&robot_command_thread::GetDHCompensation}, 
    {"TractorEnable",&robot_command_thread::TractorEnable},
    {"TractorHoming",&robot_command_thread::TractorHoming},
    {"TractorMoveL",&robot_command_thread::TractorMoveL},
    {"TractorMoveC",&robot_command_thread::TractorMoveC},
    {"TractorStop",&robot_command_thread::TractorStop},
    {"TrajectoryJUpLoad",&robot_command_thread::TrajectoryJUpLoad},
    {"TrajectoryJDelete",&robot_command_thread::TrajectoryJDelete},
    {"LoadTrajectoryJ",&robot_command_thread::LoadTrajectoryJ},
    {"MoveTrajectoryJ",&robot_command_thread::MoveTrajectoryJ},
    {"GetTrajectoryStartPose",&robot_command_thread::GetTrajectoryStartPose},
    {"GetTrajectoryPointNum",&robot_command_thread::GetTrajectoryPointNum},
    {"SetTrajectoryJSpeed",&robot_command_thread::SetTrajectoryJSpeed},
    {"LuaDownLoad",&robot_command_thread::LuaDownLoad},
    {"LuaUpload",&robot_command_thread::LuaUpload},
    {"LuaDelete",&robot_command_thread::LuaDelete},
    {"GetLuaList",&robot_command_thread::GetLuaList},
    {"ComputeToolCoordWithPoints",&robot_command_thread::ComputeToolCoordWithPoints},
    {"ComputeWObjCoordWithPoints",&robot_command_thread::ComputeWObjCoordWithPoints},
    {"WeldingSetCheckArcInterruptionParam",&robot_command_thread::WeldingSetCheckArcInterruptionParam},
    {"WeldingGetCheckArcInterruptionParam",&robot_command_thread::WeldingGetCheckArcInterruptionParam},
    {"WeldingSetReWeldAfterBreakOffParam",&robot_command_thread::WeldingSetReWeldAfterBreakOffParam},
    {"WeldingGetReWeldAfterBreakOffParam",&robot_command_thread::WeldingGetReWeldAfterBreakOffParam},
    {"WeldingStartReWeldAfterBreakOff",&robot_command_thread::WeldingStartReWeldAfterBreakOff},
    {"WeldingAbortWeldAfterBreakOff",&robot_command_thread::WeldingAbortWeldAfterBreakOff},
    {"GetSDKVersion",&robot_command_thread::GetSDKVersion},
    {"GetControllerIP",&robot_command_thread::GetControllerIP},
    {"IsInDragTeach",&robot_command_thread::IsInDragTeach},
    {"NewSpiral",&robot_command_thread::NewSpiral},
    {"ServoMoveStart",&robot_command_thread::ServoMoveStart},
    {"ServoMoveEnd",&robot_command_thread::ServoMoveEnd},
    {"ServoCart",&robot_command_thread::ServoCart},
    {"MoveCart",&robot_command_thread::MoveCart},
    {"PauseMotion",&robot_command_thread::PauseMotion},
    {"ResumeMotion",&robot_command_thread::ResumeMotion},
    {"GetDI",&robot_command_thread::GetDI},
    {"GetToolDI",&robot_command_thread::GetToolDI},
    {"WaitDI",&robot_command_thread::WaitDI},
    {"WaitMultiDI",&robot_command_thread::WaitMultiDI},
    {"WaitToolDI",&robot_command_thread::WaitToolDI},
    {"GetAI",&robot_command_thread::GetAI},
    {"GetToolAI",&robot_command_thread::GetToolAI},
    {"GetAxlePointRecordBtnState",&robot_command_thread::GetAxlePointRecordBtnState},
    {"GetToolDO",&robot_command_thread::GetToolDO},
    {"GetDO",&robot_command_thread::GetDO},
    {"WaitAI",&robot_command_thread::WaitAI},
    {"WaitToolAI",&robot_command_thread::WaitToolAI},
    {"SetSysVarValue",&robot_command_thread::SetSysVarValue},
    {"SetToolPoint",&robot_command_thread::SetToolPoint},
    {"ComputeTool",&robot_command_thread::ComputeTool},
    {"SetTcp4RefPoint",&robot_command_thread::SetTcp4RefPoint},
    {"ComputeTcp4",&robot_command_thread::ComputeTcp4},
    {"SetExTCPPoint",&robot_command_thread::SetExTCPPoint},
    {"ComputeExTCF",&robot_command_thread::ComputeExTCF},
    {"SetWObjCoordPoint",&robot_command_thread::SetWObjCoordPoint},
    {"ComputeWObjCoord",&robot_command_thread::ComputeWObjCoord},
    {"WaitMs",&robot_command_thread::WaitMs},
    {"SetLimitNegative",&robot_command_thread::SetLimitNegative},
    {"GetRobotInstallAngle",&robot_command_thread::GetRobotInstallAngle},
    {"GetSysVarValue",&robot_command_thread::GetSysVarValue},
    {"GetActualJointPosDegree",&robot_command_thread::GetActualJointPosDegree},
    {"GetActualJointSpeedsDegree",&robot_command_thread::GetActualJointSpeedsDegree},
    {"GetActualJointAccDegree",&robot_command_thread::GetActualJointAccDegree},
    {"GetTargetTCPCompositeSpeed",&robot_command_thread::GetTargetTCPCompositeSpeed},
    {"GetActualTCPCompositeSpeed",&robot_command_thread::GetActualTCPCompositeSpeed},
    {"GetTargetTCPSpeed",&robot_command_thread::GetTargetTCPSpeed},
    {"GetActualTCPSpeed",&robot_command_thread::GetActualTCPSpeed},
    {"GetActualTCPPose",&robot_command_thread::GetActualTCPPose},
    {"GetActualTCPNum",&robot_command_thread::GetActualTCPNum},
    {"GetActualWObjNum",&robot_command_thread::GetActualWObjNum},
    {"GetActualToolFlangePose",&robot_command_thread::GetActualToolFlangePose},
    {"GetInverseKinRef",&robot_command_thread::GetInverseKinRef},
    {"GetInverseKinHasSolution",&robot_command_thread::GetInverseKinHasSolution},
    {"GetForwardKin",&robot_command_thread::GetForwardKin},
    {"GetJointTorques",&robot_command_thread::GetJointTorques},
    {"GetTargetPayload",&robot_command_thread::GetTargetPayload},
    {"GetTargetPayloadCog",&robot_command_thread::GetTargetPayloadCog},
    {"GetWObjOffset",&robot_command_thread::GetWObjOffset},
    {"GetJointSoftLimitDeg",&robot_command_thread::GetJointSoftLimitDeg},
    {"GetSystemClock",&robot_command_thread::GetSystemClock},
    {"GetRobotCurJointsConfig",&robot_command_thread::GetRobotCurJointsConfig},
    {"GetDefaultTransVel",&robot_command_thread::GetDefaultTransVel},
    {"GetRobotMotionDone",&robot_command_thread::GetRobotMotionDone},
    {"GetRobotErrorCode",&robot_command_thread::GetRobotErrorCode},
    {"GetRobotTeachingPoint",&robot_command_thread::GetRobotTeachingPoint},
    {"GetMotionQueueLength",&robot_command_thread::GetMotionQueueLength},
    {"SetTPDParam",&robot_command_thread::SetTPDParam},
    {"SetTPDStart",&robot_command_thread::SetTPDStart},
    {"SetWebTPDStop",&robot_command_thread::SetWebTPDStop},
    {"SetTPDDelete",&robot_command_thread::SetTPDDelete},
    {"LoadTPD",&robot_command_thread::LoadTPD},
    {"GetTPDStartPose",&robot_command_thread::GetTPDStartPose},
    {"MoveTPD",&robot_command_thread::MoveTPD},
    {"SetTrajectoryJForceTorque",&robot_command_thread::SetTrajectoryJForceTorque},
    {"SetTrajectoryJForceFx",&robot_command_thread::SetTrajectoryJForceFx},
    {"SetTrajectoryJForceFy",&robot_command_thread::SetTrajectoryJForceFy},
    {"SetTrajectoryJForceFz",&robot_command_thread::SetTrajectoryJForceFz},
    {"SetTrajectoryJTorqueTx",&robot_command_thread::SetTrajectoryJTorqueTx},
    {"SetTrajectoryJTorqueTy",&robot_command_thread::SetTrajectoryJTorqueTy},
    {"SetTrajectoryJTorqueTz",&robot_command_thread::SetTrajectoryJTorqueTz},
    {"LoadDefaultProgConfig",&robot_command_thread::LoadDefaultProgConfig},
    {"ProgramLoad",&robot_command_thread::ProgramLoad},
    {"GetLoadedProgram",&robot_command_thread::GetLoadedProgram},
    {"GetCurrentLine",&robot_command_thread::GetCurrentLine},
    {"ProgramRun",&robot_command_thread::ProgramRun},
    {"ProgramPause",&robot_command_thread::ProgramPause},
    {"ProgramResume",&robot_command_thread::ProgramResume},
    {"ProgramStop",&robot_command_thread::ProgramStop},
    {"GetProgramState",&robot_command_thread::GetProgramState},
    {"SetGripperConfig",&robot_command_thread::SetGripperConfig},
    {"GetGripperConfig",&robot_command_thread::GetGripperConfig},
    {"GetGripperMotionDone",&robot_command_thread::GetGripperMotionDone},
    {"GetGripperActivateStatus",&robot_command_thread::GetGripperActivateStatus},
    {"GetGripperCurPosition",&robot_command_thread::GetGripperCurPosition},
    {"GetGripperCurSpeed",&robot_command_thread::GetGripperCurSpeed},
    {"GetGripperCurCurrent",&robot_command_thread::GetGripperCurCurrent},
    {"GetGripperVoltage",&robot_command_thread::GetGripperVoltage},
    {"GetGripperTemp",&robot_command_thread::GetGripperTemp},
    {"GetGripperRotNum",&robot_command_thread::GetGripperRotNum},
    {"GetGripperRotSpeed",&robot_command_thread::GetGripperRotSpeed},
    {"GetGripperRotTorque",&robot_command_thread::GetGripperRotTorque},
    {"ComputePrePick",&robot_command_thread::ComputePrePick},
    {"ComputePostPick",&robot_command_thread::ComputePostPick},
    {"FT_SetConfig",&robot_command_thread::FT_SetConfig},
    {"FT_GetConfig",&robot_command_thread::FT_GetConfig},
    {"FT_Activate",&robot_command_thread::FT_Activate},
    {"FT_SetZero",&robot_command_thread::FT_SetZero},
    {"FT_SetRCS",&robot_command_thread::FT_SetRCS},
    {"FT_PdIdenRecord",&robot_command_thread::FT_PdIdenRecord},
    {"FT_PdIdenCompute",&robot_command_thread::FT_PdIdenCompute},
    {"FT_PdCogIdenRecord",&robot_command_thread::FT_PdCogIdenRecord},
    {"FT_PdCogIdenCompute",&robot_command_thread::FT_PdCogIdenCompute},
    {"FT_GetForceTorqueRCS",&robot_command_thread::FT_GetForceTorqueRCS},
    {"FT_GetForceTorqueOrigin",&robot_command_thread::FT_GetForceTorqueOrigin},
    {"FT_Guard",&robot_command_thread::FT_Guard},
    {"FT_Control",&robot_command_thread::FT_Control},
    {"FT_SpiralSearch",&robot_command_thread::FT_SpiralSearch},
    {"FT_RotInsertion",&robot_command_thread::FT_RotInsertion},
    {"FT_LinInsertion",&robot_command_thread::FT_LinInsertion},
    {"FT_FindSurface",&robot_command_thread::FT_FindSurface},
    {"FT_CalCenterStart",&robot_command_thread::FT_CalCenterStart},
    {"FT_CalCenterEnd",&robot_command_thread::FT_CalCenterEnd},
    {"FT_ComplianceStart",&robot_command_thread::FT_ComplianceStart},
    {"FT_ComplianceStop",&robot_command_thread::FT_ComplianceStop},
    {"LoadIdentifyDynFilterInit",&robot_command_thread::LoadIdentifyDynFilterInit},
    {"LoadIdentifyDynVarInit",&robot_command_thread::LoadIdentifyDynVarInit},
    {"LoadIdentifyMain",&robot_command_thread::LoadIdentifyMain},
    {"LoadIdentifyGetResult",&robot_command_thread::LoadIdentifyGetResult},
    {"ConveyorStartEnd",&robot_command_thread::ConveyorStartEnd},
    {"ConveyorPointIORecord",&robot_command_thread::ConveyorPointIORecord},
    {"ConveyorPointARecord",&robot_command_thread::ConveyorPointARecord},
    {"ConveyorRefPointRecord",&robot_command_thread::ConveyorRefPointRecord},
    {"ConveyorPointBRecord",&robot_command_thread::ConveyorPointBRecord},
    {"ConveyorIODetect",&robot_command_thread::ConveyorIODetect},
    {"ConveyorGetTrackData",&robot_command_thread::ConveyorGetTrackData},
    {"ConveyorTrackStart",&robot_command_thread::ConveyorTrackStart},
    {"ConveyorTrackEnd",&robot_command_thread::ConveyorTrackEnd},
    {"ConveyorSetParam",&robot_command_thread::ConveyorSetParam},
    {"ConveyorCatchPointComp",&robot_command_thread::ConveyorCatchPointComp},
    {"TrackMoveL",&robot_command_thread::TrackMoveL},
    {"GetSSHKeygen",&robot_command_thread::GetSSHKeygen},
    {"SetSSHScpCmd",&robot_command_thread::SetSSHScpCmd},
    {"ComputeFileMD5",&robot_command_thread::ComputeFileMD5},
    {"GetRobotEmergencyStopState",&robot_command_thread::GetRobotEmergencyStopState},
    {"GetSDKComState",&robot_command_thread::GetSDKComState},
    {"GetSafetyStopState",&robot_command_thread::GetSafetyStopState},
    {"GetHardwareVersion",&robot_command_thread::GetHardwareVersion},
    {"GetFirmwareVersion",&robot_command_thread::GetFirmwareVersion},
    {"PointTableSwitch",&robot_command_thread::PointTableSwitch},
    {"PointTableDownLoad",&robot_command_thread::PointTableDownLoad},
    {"PointTableUpLoad",&robot_command_thread::PointTableUpLoad},
    {"PointTableUpdateLua",&robot_command_thread::PointTableUpdateLua},
    {"ARCStart",&robot_command_thread::ARCStart},
    {"ARCEnd",&robot_command_thread::ARCEnd},
    {"WeldingSetCurrentRelation",&robot_command_thread::WeldingSetCurrentRelation},
    {"WeldingSetVoltageRelation",&robot_command_thread::WeldingSetVoltageRelation},
    {"WeldingGetCurrentRelation",&robot_command_thread::WeldingGetCurrentRelation},
    {"WeldingGetVoltageRelation",&robot_command_thread::WeldingGetVoltageRelation},
    {"WeldingSetCurrent",&robot_command_thread::WeldingSetCurrent},
    {"WeldingSetVoltage",&robot_command_thread::WeldingSetVoltage},
    {"WeaveSetPara",&robot_command_thread::WeaveSetPara},
    {"WeaveOnlineSetPara",&robot_command_thread::WeaveOnlineSetPara},
    {"WeaveStart",&robot_command_thread::WeaveStart},
    {"WeaveEnd",&robot_command_thread::WeaveEnd},
    {"SetForwardWireFeed",&robot_command_thread::SetForwardWireFeed},
    {"SetReverseWireFeed",&robot_command_thread::SetReverseWireFeed},
    {"SetAspirated",&robot_command_thread::SetAspirated},
    {"SegmentWeldStart",&robot_command_thread::SegmentWeldStart},
    {"LoggerInit",&robot_command_thread::LoggerInit},
    {"SetLoggerLevel",&robot_command_thread::SetLoggerLevel},
    {"AuxServoGetParam",&robot_command_thread::AuxServoGetParam},
    {"AuxServoGetStatus",&robot_command_thread::AuxServoGetStatus},
    {"GetExDevProtocol",&robot_command_thread::GetExDevProtocol},
    {"SetExDevProtocol",&robot_command_thread::SetExDevProtocol},
    {"SetOaccScale",&robot_command_thread::SetOaccScale},
    {"MoveAOStart",&robot_command_thread::MoveAOStart},
    {"MoveAOStop",&robot_command_thread::MoveAOStop},
    {"MoveToolAOStart",&robot_command_thread::MoveToolAOStart},
    {"MoveToolAOStop",&robot_command_thread::MoveToolAOStop},
    {"ExtDevSetUDPComParam",&robot_command_thread::ExtDevSetUDPComParam},
    {"ExtDevGetUDPComParam",&robot_command_thread::ExtDevGetUDPComParam},
    {"ExtDevLoadUDPDriver",&robot_command_thread::ExtDevLoadUDPDriver},
    {"ExtDevUnloadUDPDriver",&robot_command_thread::ExtDevUnloadUDPDriver},
    {"SetAuxDIFilterTime",&robot_command_thread::SetAuxDIFilterTime},
    {"SetAuxAIFilterTime",&robot_command_thread::SetAuxAIFilterTime},
    {"WaitAuxDI",&robot_command_thread::WaitAuxDI},
    {"WaitAuxAI",&robot_command_thread::WaitAuxAI},
    {"GetAuxDI",&robot_command_thread::GetAuxDI},
    {"GetAuxAI",&robot_command_thread::GetAuxAI},
    {"ExtDevUDPClientComReset",&robot_command_thread::ExtDevUDPClientComReset},
    {"ExtDevUDPClientComClose",&robot_command_thread::ExtDevUDPClientComClose},
    {"ExtAxisParamConfig",&robot_command_thread::ExtAxisParamConfig},
    {"SetRobotPosToAxis",&robot_command_thread::SetRobotPosToAxis},
    {"SetAxisDHParaConfig",&robot_command_thread::SetAxisDHParaConfig},
    {"ExtAxisSetRefPoint",&robot_command_thread::ExtAxisSetRefPoint},
    {"ExtAxisComputeECoordSys",&robot_command_thread::ExtAxisComputeECoordSys},
    {"ExtAxisActiveECoordSys",&robot_command_thread::ExtAxisActiveECoordSys},
    {"SetRefPointInExAxisEnd",&robot_command_thread::SetRefPointInExAxisEnd},
    {"PositionorSetRefPoint",&robot_command_thread::PositionorSetRefPoint},
    {"PositionorComputeECoordSys",&robot_command_thread::PositionorComputeECoordSys},
    {"ExtAxisSyncMoveL",&robot_command_thread::ExtAxisSyncMoveL},
    {"ExtAxisSyncMoveC",&robot_command_thread::ExtAxisSyncMoveC},
    {"WireSearchStart",&robot_command_thread::WireSearchStart},
    {"WireSearchEnd",&robot_command_thread::WireSearchEnd},
    {"GetWireSearchOffset",&robot_command_thread::GetWireSearchOffset},
    {"WireSearchWait",&robot_command_thread::WireSearchWait},
    {"SetPointToDatabase",&robot_command_thread::SetPointToDatabase},
    {"ArcWeldTraceControl",&robot_command_thread::ArcWeldTraceControl},
    {"ArcWeldTraceExtAIChannelConfig",&robot_command_thread::ArcWeldTraceExtAIChannelConfig},
    {"EndForceDragControl",&robot_command_thread::EndForceDragControl},
    {"SetForceSensorDragAutoFlag",&robot_command_thread::SetForceSensorDragAutoFlag},
    {"GetForceAndTorqueDragState",&robot_command_thread::GetForceAndTorqueDragState},
    {"SetForceSensorPayload",&robot_command_thread::SetForceSensorPayload},
    {"SetForceSensorPayloadCog",&robot_command_thread::SetForceSensorPayloadCog},
    {"GetForceSensorPayload",&robot_command_thread::GetForceSensorPayload},
    {"GetForceSensorPayloadCog",&robot_command_thread::GetForceSensorPayloadCog},
    {"ForceSensorAutoComputeLoad",&robot_command_thread::ForceSensorAutoComputeLoad},
    {"ForceSensorSetSaveDataFlag",&robot_command_thread::ForceSensorSetSaveDataFlag},
    {"ForceSensorComputeLoad",&robot_command_thread::ForceSensorComputeLoad},
    {"GetSegmentWeldPoint",&robot_command_thread::GetSegmentWeldPoint},
    {"WeldingSetProcessParam",&robot_command_thread::WeldingSetProcessParam},
    {"WeldingGetProcessParam",&robot_command_thread::WeldingGetProcessParam},
    {"AxleSensorConfig",&robot_command_thread::AxleSensorConfig},
    {"AxleSensorConfigGet",&robot_command_thread::AxleSensorConfigGet},
    {"AxleSensorActivate",&robot_command_thread::AxleSensorActivate},
    {"SetOutputResetCtlBoxDO",&robot_command_thread::SetOutputResetCtlBoxDO},
    {"SetOutputResetCtlBoxAO",&robot_command_thread::SetOutputResetCtlBoxAO},
    {"SetOutputResetAxleDO",&robot_command_thread::SetOutputResetAxleDO},
    {"SetOutputResetAxleAO",&robot_command_thread::SetOutputResetAxleAO},
    {"SetOutputResetExtDO",&robot_command_thread::SetOutputResetExtDO},
    {"SetOutputResetExtAO",&robot_command_thread::SetOutputResetExtAO},
    {"SetOutputResetSmartToolDO",&robot_command_thread::SetOutputResetSmartToolDO},
    {"WeaveStartSim",&robot_command_thread::WeaveStartSim},
    {"WeaveEndSim",&robot_command_thread::WeaveEndSim},
    {"WeaveInspectStart",&robot_command_thread::WeaveInspectStart},
    {"WeaveInspectEnd",&robot_command_thread::WeaveInspectEnd},
    {"SetAirControlExtDoNum",&robot_command_thread::SetAirControlExtDoNum},
    {"SetArcStartExtDoNum",&robot_command_thread::SetArcStartExtDoNum},
    {"SetWireReverseFeedExtDoNum",&robot_command_thread::SetWireReverseFeedExtDoNum},
    {"SetWireForwardFeedExtDoNum",&robot_command_thread::SetWireForwardFeedExtDoNum},
    {"SetArcDoneExtDiNum",&robot_command_thread::SetArcDoneExtDiNum},
    {"SetWeldReadyExtDiNum",&robot_command_thread::SetWeldReadyExtDiNum},
    {"SetExtDIWeldBreakOffRecover",&robot_command_thread::SetExtDIWeldBreakOffRecover},
    {"SetCollisionDetectionMethod",&robot_command_thread::SetCollisionDetectionMethod},
    {"SetStaticCollisionOnOff",&robot_command_thread::SetStaticCollisionOnOff},
    {"SetPowerLimit",&robot_command_thread::SetPowerLimit},
    {"ServoJTStart",&robot_command_thread::ServoJTStart},
    {"ServoJT",&robot_command_thread::ServoJT},
    {"ServoJTEnd",&robot_command_thread::ServoJTEnd},
    {"SetRobotRealtimeStateSamplePeriod",&robot_command_thread::SetRobotRealtimeStateSamplePeriod},
    {"GetRobotRealtimeStateSamplePeriod",&robot_command_thread::GetRobotRealtimeStateSamplePeriod},
    {"GetJointDriverTemperature",&robot_command_thread::GetJointDriverTemperature},
    {"GetJointDriverTorque",&robot_command_thread::GetJointDriverTorque},
    {"ArcWeldTraceReplayStart",&robot_command_thread::ArcWeldTraceReplayStart},
    {"ArcWeldTraceReplayEnd",&robot_command_thread::ArcWeldTraceReplayEnd},
    {"MultilayerOffsetTrsfToBase",&robot_command_thread::MultilayerOffsetTrsfToBase},
    {"AngularSpeedStart",&robot_command_thread::AngularSpeedStart},
    {"AngularSpeedEnd",&robot_command_thread::AngularSpeedEnd},
    {"SoftwareUpgrade",&robot_command_thread::SoftwareUpgrade},
    {"GetSoftwareUpgradeState",&robot_command_thread::GetSoftwareUpgradeState},
    {"AuxServoSetAcc",&robot_command_thread::AuxServoSetAcc},
    {"AuxServoSetEmergencyStopAcc",&robot_command_thread::AuxServoSetEmergencyStopAcc},
    {"AuxServoGetAcc",&robot_command_thread::AuxServoGetAcc},
    {"AuxServoGetEmergencyStopAcc",&robot_command_thread::AuxServoGetEmergencyStopAcc},
    {"GetAxleCommunicationParam",&robot_command_thread::GetAxleCommunicationParam},
    {"SetAxleCommunicationParam",&robot_command_thread::SetAxleCommunicationParam},
    {"SetAxleFileType",&robot_command_thread::SetAxleFileType},
    {"SetAxleLuaEnable",&robot_command_thread::SetAxleLuaEnable},
    {"SetRecoverAxleLuaErr",&robot_command_thread::SetRecoverAxleLuaErr},
    {"GetAxleLuaEnableStatus",&robot_command_thread::GetAxleLuaEnableStatus},
    {"SetAxleLuaEnableDeviceType",&robot_command_thread::SetAxleLuaEnableDeviceType},
    {"GetAxleLuaEnableDeviceType",&robot_command_thread::GetAxleLuaEnableDeviceType},
    {"GetAxleLuaEnableDevice",&robot_command_thread::GetAxleLuaEnableDevice},
    {"SetAxleLuaGripperFunc",&robot_command_thread::SetAxleLuaGripperFunc},
    {"GetAxleLuaGripperFunc",&robot_command_thread::GetAxleLuaGripperFunc},
    {"SetCtrlOpenLUAName",&robot_command_thread::SetCtrlOpenLUAName},
    {"GetCtrlOpenLUAName",&robot_command_thread::GetCtrlOpenLUAName},
    {"LoadCtrlOpenLUA",&robot_command_thread::LoadCtrlOpenLUA},
    {"UnloadCtrlOpenLUA",&robot_command_thread::UnloadCtrlOpenLUA},
    {"SetCtrlOpenLuaErrCode",&robot_command_thread::SetCtrlOpenLuaErrCode},
    {"SlaveFileWrite",&robot_command_thread::SlaveFileWrite},
    {"AxleLuaUpload",&robot_command_thread::AxleLuaUpload},
    {"SetSysServoBootMode",&robot_command_thread::SetSysServoBootMode},
    {"SetWireSearchExtDIONum",&robot_command_thread::SetWireSearchExtDIONum},
    {"SetWeldMachineCtrlModeExtDoNum",&robot_command_thread::SetWeldMachineCtrlModeExtDoNum},
    {"SetWeldMachineCtrlMode",&robot_command_thread::SetWeldMachineCtrlMode},
    {"SingularAvoidStart",&robot_command_thread::SingularAvoidStart},
    {"SingularAvoidEnd",&robot_command_thread::SingularAvoidEnd},
    {"PtpFIRPlanningStart",&robot_command_thread::PtpFIRPlanningStart},
    {"PtpFIRPlanningEnd",&robot_command_thread::PtpFIRPlanningEnd},
    {"LinArcFIRPlanningStart",&robot_command_thread::LinArcFIRPlanningStart},
    {"LinArcFIRPlanningEnd",&robot_command_thread::LinArcFIRPlanningEnd},
    {"LaserSensorRecord",&robot_command_thread::LaserSensorRecord},
    {"LaserTrackingLaserOn",&robot_command_thread::LaserTrackingLaserOn},
    {"LaserTrackingLaserOff",&robot_command_thread::LaserTrackingLaserOff},
    {"LaserTrackingTrackOn",&robot_command_thread::LaserTrackingTrackOn},
    {"LaserTrackingTrackOff",&robot_command_thread::LaserTrackingTrackOff},
    {"LaserTrackingSearchStart",&robot_command_thread::LaserTrackingSearchStart},
    {"LaserTrackingLaserOnOff",&robot_command_thread::LaserTrackingLaserOnOff},
    {"LaserTrackingTrackOnOff",&robot_command_thread::LaserTrackingTrackOnOff},
    {"LaserTrackingSearchStart_xyz",&robot_command_thread::LaserTrackingSearchStart_xyz},
    {"LaserTrackingSearchStart_point",&robot_command_thread::LaserTrackingSearchStart_point},
    {"LaserTrackingSearchStop",&robot_command_thread::LaserTrackingSearchStop},
    {"LaserTrackingSensorConfig",&robot_command_thread::LaserTrackingSensorConfig},
    {"LaserTrackingSensorSamplePeriod",&robot_command_thread::LaserTrackingSensorSamplePeriod},
    {"LoadPosSensorDriver",&robot_command_thread::LoadPosSensorDriver},
    {"UnLoadPosSensorDriver",&robot_command_thread::UnLoadPosSensorDriver},
    {"LaserSensorRecord1",&robot_command_thread::LaserSensorRecord1},
    {"LaserSensorReplay",&robot_command_thread::LaserSensorReplay},
    {"MoveLTR",&robot_command_thread::MoveLTR},
    {"LaserSensorRecordandReplay",&robot_command_thread::LaserSensorRecordandReplay},
    {"MoveToLaserRecordStart",&robot_command_thread::MoveToLaserRecordStart},
    {"MoveToLaserRecordEnd",&robot_command_thread::MoveToLaserRecordEnd},
    {"MoveToLaserSeamPos",&robot_command_thread::MoveToLaserSeamPos},
    {"GetLaserSeamPos",&robot_command_thread::GetLaserSeamPos},
    {"WeaveChangeStart",&robot_command_thread::WeaveChangeStart},
    {"WeaveChangeEnd",&robot_command_thread::WeaveChangeEnd},
    {"LoadTrajectoryLA",&robot_command_thread::LoadTrajectoryLA},
    {"MoveTrajectoryLA",&robot_command_thread::MoveTrajectoryLA},
    {"CustomCollisionDetectionStart",&robot_command_thread::CustomCollisionDetectionStart},
    {"CustomCollisionDetectionEnd",&robot_command_thread::CustomCollisionDetectionEnd},
    {"AccSmoothStart",&robot_command_thread::AccSmoothStart},
    {"AccSmoothEnd",&robot_command_thread::AccSmoothEnd},
    {"RbLogDownload",&robot_command_thread::RbLogDownload},
    {"AllDataSourceDownload",&robot_command_thread::AllDataSourceDownload},
    {"DataPackageDownload",&robot_command_thread::DataPackageDownload},
    {"GetRobotSN",&robot_command_thread::GetRobotSN},
    {"ShutDownRobotOS",&robot_command_thread::ShutDownRobotOS},
    {"ConveyorComDetect",&robot_command_thread::ConveyorComDetect},
    {"ConveyorComDetectTrigger",&robot_command_thread::ConveyorComDetectTrigger},
    {"ArcWeldTraceAIChannelCurrent",&robot_command_thread::ArcWeldTraceAIChannelCurrent},
    {"ArcWeldTraceAIChannelVoltage",&robot_command_thread::ArcWeldTraceAIChannelVoltage},
    {"ArcWeldTraceCurrentPara",&robot_command_thread::ArcWeldTraceCurrentPara},
    {"ArcWeldTraceVoltagePara",&robot_command_thread::ArcWeldTraceVoltagePara},
    {"WeldingSetVoltageGradualChangeStart",&robot_command_thread::WeldingSetVoltageGradualChangeStart},
    {"WeldingSetVoltageGradualChangeEnd",&robot_command_thread::WeldingSetVoltageGradualChangeEnd},
    {"WeldingSetCurrentGradualChangeStart",&robot_command_thread::WeldingSetCurrentGradualChangeStart},
    {"WeldingSetCurrentGradualChangeEnd",&robot_command_thread::WeldingSetCurrentGradualChangeEnd},
    {"GetSmarttoolBtnState",&robot_command_thread::GetSmarttoolBtnState},
    {"SetWideBoxTempFanMonitorParam",&robot_command_thread::SetWideBoxTempFanMonitorParam},
    {"GetWideBoxTempFanMonitorParam",&robot_command_thread::GetWideBoxTempFanMonitorParam},
    {"SetFocusCalibPoint",&robot_command_thread::SetFocusCalibPoint},
    {"ComputeFocusCalib",&robot_command_thread::ComputeFocusCalib},
    {"FocusStart",&robot_command_thread::FocusStart},
    {"FocusEnd",&robot_command_thread::FocusEnd},
    {"SetFocusPosition",&robot_command_thread::SetFocusPosition},
    {"SetEncoderUpgrade",&robot_command_thread::SetEncoderUpgrade},
    {"SetJointFirmwareUpgrade",&robot_command_thread::SetJointFirmwareUpgrade},
    {"SetCtrlFirmwareUpgrade",&robot_command_thread::SetCtrlFirmwareUpgrade},
    {"SetEndFirmwareUpgrade",&robot_command_thread::SetEndFirmwareUpgrade},
    {"JointAllParamUpgrade",&robot_command_thread::JointAllParamUpgrade},
    {"SetRobotType",&robot_command_thread::SetRobotType},
    {"LaserRecordPoint",&robot_command_thread::LaserRecordPoint},
    {"SetExAxisRobotPlan",&robot_command_thread::SetExAxisRobotPlan},
    {"SetReConnectParam",&robot_command_thread::SetReConnectParam},
    {"GetFieldBusConfig",&robot_command_thread::GetFieldBusConfig},
    {"FieldBusSlaveWriteDO",&robot_command_thread::FieldBusSlaveWriteDO},
    {"FieldBusSlaveWriteAO",&robot_command_thread::FieldBusSlaveWriteAO},
    {"FieldBusSlaveReadDI",&robot_command_thread::FieldBusSlaveReadDI},
    {"FieldBusSlaveReadAI",&robot_command_thread::FieldBusSlaveReadAI},
    {"FieldBusSlaveWaitDI",&robot_command_thread::FieldBusSlaveWaitDI},
    {"FieldBusSlaveWaitAI",&robot_command_thread::FieldBusSlaveWaitAI},
    {"SetSuckerCtrl",&robot_command_thread::SetSuckerCtrl},
    {"GetSuckerState",&robot_command_thread::GetSuckerState},
    {"WaitSuckerState",&robot_command_thread::WaitSuckerState},
    {"OpenLuaUpload",&robot_command_thread::OpenLuaUpload},
    {"ImpedanceControlStartStop",&robot_command_thread::ImpedanceControlStartStop},
    {"SetTorqueDetectionSwitch",&robot_command_thread::SetTorqueDetectionSwitch},
    {"GetToolCoordWithID",&robot_command_thread::GetToolCoordWithID},
    {"GetWObjCoordWithID",&robot_command_thread::GetWObjCoordWithID},
    {"GetExToolCoordWithID",&robot_command_thread::GetExToolCoordWithID},
    {"GetExAxisCoordWithID",&robot_command_thread::GetExAxisCoordWithID},
    {"GetTargetPayloadWithID",&robot_command_thread::GetTargetPayloadWithID},
    {"GetCurToolCoord",&robot_command_thread::GetCurToolCoord},
    {"GetCurWObjCoord",&robot_command_thread::GetCurWObjCoord},
    {"GetCurExToolCoord",&robot_command_thread::GetCurExToolCoord},
    {"GetCurExAxisCoord",&robot_command_thread::GetCurExAxisCoord},
    {"KernelUpgrade",&robot_command_thread::KernelUpgrade},
    {"GetKernelUpgradeResult",&robot_command_thread::GetKernelUpgradeResult},
    {"CustomWeaveSetPara",&robot_command_thread::CustomWeaveSetPara},
    {"CustomWeaveGetPara",&robot_command_thread::CustomWeaveGetPara},
    {"JointSensitivityEnable",&robot_command_thread::JointSensitivityEnable},
    {"JointSensitivityCalibration",&robot_command_thread::JointSensitivityCalibration},
    {"JointSensitivityCollect",&robot_command_thread::JointSensitivityCollect},
    {"Sleep",&robot_command_thread::Sleep},
    {"MotionQueueClear",&robot_command_thread::MotionQueueClear},
    {"GetSlavePortErrCounter",&robot_command_thread::GetSlavePortErrCounter},
    {"SlavePortErrCounterClear",&robot_command_thread::SlavePortErrCounterClear},
    {"SetVelFeedForwardRatio",&robot_command_thread::SetVelFeedForwardRatio},
    {"GetVelFeedForwardRatio",&robot_command_thread::GetVelFeedForwardRatio},
    {"RobotMCULogCollect",&robot_command_thread::RobotMCULogCollect},
    {"MoveToIntersectLineStart",&robot_command_thread::MoveToIntersectLineStart},
    {"MoveIntersectLine",&robot_command_thread::MoveIntersectLine},
    {"JointHysteresisError",&robot_command_thread::JointHysteresisError},
    {"JointRepeatability",&robot_command_thread::JointRepeatability},
    {"SetAdmittanceParams",&robot_command_thread::SetAdmittanceParams},
    {"SerCoderCompenParams",&robot_command_thread::SerCoderCompenParams},
    {"TCPComputeRPY",&robot_command_thread::TCPComputeRPY},
    {"TCPComputeXYZ",&robot_command_thread::TCPComputeXYZ},
    {"TCPRecordFlangePosStart",&robot_command_thread::TCPRecordFlangePosStart},
    {"TCPRecordFlangePosEnd",&robot_command_thread::TCPRecordFlangePosEnd},
    {"TCPGetRecordFlangePos",&robot_command_thread::TCPGetRecordFlangePos},
    {"PhotoelectricSensorTCPCalibration",&robot_command_thread::PhotoelectricSensorTCPCalibration},
    {"MoveStationary",&robot_command_thread::MoveStationary}
    };
};

#endif
