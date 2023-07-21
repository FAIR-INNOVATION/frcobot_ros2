#include "global_val.h"


std::map<std::string,int>& glob_FR_cmd_id(){
    static std::map<std::string,int> tmp{
    {"DragTeachSwitch",333},
    {"RobotEnable",632},
    {"Mode",303},
    {"SetSpeed",206},
    {"SetToolCoord",316},
    {"SetToolList",319},
    {"SetExToolCoord",330},
    {"SetExToolList",331},
    {"SetWObjCoord",251},
    {"SetWObjList",383},
    {"SetLoadWeight",306},
    {"SetLoadCoord",307},
    {"SetRobotInstallPos",337},
    {"SetRobotInstallAngle",631},//通用设置
    {"SetAnticollision",305},
    {"SetCollisionStrategy",569},
    {"SetLimitPositive",308},
    {"SetLimitNegative",309},
    {"ResetAllError",107},
    {"FrictionCompensationOnOff",338},
    {"SetFrictionValue_level",541},
    {"SetFrictionValue_wall",542},
    {"SetFrictionValue_ceiling",543},//安全设置
    {"SetFrictionValue_freedom",637},
    {"ActGripper",227},
    {"MoveGripper",228},//外设控制
    {"SetDO",204},
    {"SetToolDO",210},
    {"SetAO",209},
    {"SetToolAO",211},//IO控制
    {"StartJOG",232},
    {"StopJOG",233},
    {"ImmStopJOG",242},
    {"MoveJ",201},
    {"MoveL",203},
    {"MoveC",202},
    {"ServoJ",376},
    {"ServoJTStart",735},
    {"ServoJT",736},
    {"ServoJTEnd",737},
    {"Circle",540},
    {"NewSpiral",577},
    {"SplineStart",346},
    {"SplinePTP",347},
    {"SplineEnd",350},
    {"NewSplineStart",553},
    {"NewSplinePoint",555},
    {"NewSplineEnd",554},
    {"StopMotion",102},
    {"PointsOffsetEnable",718},
    {"PointsOffsetDisable",719},//运动指令
    {"ProgramRun",101}
};
    return tmp;
}
