#include "fairino_hardware/fairino_hardware_interface.hpp"

#include <chrono>
#include <thread>

namespace fairino_hardware {

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &sysinfo) {
  if (hardware_interface::SystemInterface::on_init(sysinfo) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  info_ = sysinfo; // info_是父类中定义的变量

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {

    // 指令部分
    if (joint.command_interfaces.size() != 1) { // 开放servoJ
      RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first "
                   "command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 关节状态部分
    if (joint.state_interfaces.size() < 1) {
      RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' has %zu state interface. At least 1 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s state interface as first state "
                   "interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
} // end on_init

std::vector<hardware_interface::StateInterface>
FairinoHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // 导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &_jnt_position_state[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &_jnt_velocity_state[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &_jnt_torque_state[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FairinoHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &_jnt_position_command[i]));
  }

  // Freedrive command interfaces
  command_interfaces.emplace_back("freedrive", "enable",        &_enable_cmd);
  command_interfaces.emplace_back("freedrive", "abort",         &_abort_cmd);
  command_interfaces.emplace_back("freedrive", "async_success", &_async_success);

  return command_interfaces;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  using namespace std::chrono_literals;
  RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
              "Starting ...please wait...");
  // 做变量的初始化工作
  _ptr_robot = std::make_unique<FRRobot>(); // 创建机器人实例
  for (int i = 0; i < 6; i++) {             // 初始化变量
    _jnt_position_command[i] = 0;
    _jnt_velocity_command[i] = 0;
    _jnt_torque_command[i] = 0;
    _jnt_position_state[i] = 0;
    _jnt_velocity_state[i] = 0;
    _jnt_torque_state[i] = 0;
  }
  _control_mode = 0; // 默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制

  // Debug publisher
  _debug_node = std::make_shared<rclcpp::Node>("fairino_hardware_debug");
  _cmd_pub = _debug_node->create_publisher<sensor_msgs::msg::JointState>(
    "joint_commands", rclcpp::SystemDefaultsQoS());
  _cmd_msg.name = {"j1", "j2", "j3", "j4", "j5", "j6"};
  _cmd_msg.position.resize(6);

  // Initialise freedrive command interface doubles
  _enable_cmd = 0.0;
  _abort_cmd  = 0.0;
  _async_success = 0.0;
  _in_freedrive = false;

  // Resend robot program service
  _resend_pending = false;
  _resend_done    = false;
  _resend_success = false;
  _resend_service = get_node()->create_service<std_srvs::srv::Trigger>(
      "/fr_arm/resend_robot_program",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
             std_srvs::srv::Trigger::Response::SharedPtr resp) {
        _resend_done = false;
        _resend_pending = true;
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!_resend_done && std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }
        resp->success = _resend_done && _resend_success.load();
        resp->message = resp->success ? "ok" : "timeout or failed";
      });

  errno_t returncode = _ptr_robot->RPC(info_.hardware_parameters.at("ip_address").c_str()); // 建立xmlrpc连接
  rclcpp::sleep_for(200ms); // 等待一段时间让控制器的rpc连接建立完毕
  if (returncode != 0) {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "机械臂SDK连接失败！请检查端口时候被占用");
    return hardware_interface::CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "机械臂SDK连接成功！");
  }
  _ptr_robot->RobotEnable(1);
  _ptr_robot->Mode(0);
  _ptr_robot->ResetAllError();
  errno_t servo_err = _ptr_robot->ServoMoveStart();
  if (servo_err != 0) {
    RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                "ServoMoveStart failed: %d", servo_err);
  }
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  // 做第一步的工作，读取当前状态数据
  JointPos jntpos;
  returncode = _ptr_robot->GetActualJointPosDegree(0, &jntpos);
  /*
  获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
  因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
  */
  if (returncode == 0) {
    for (int j = 0; j < 6; j++) {
      _jnt_position_command[j] = jntpos.jPos[j] / 180.0 * M_PI;
    }
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "初始指令位置: %f,%f,%f,%f,%f,%f", _jnt_position_command[0],
                _jnt_position_command[1], _jnt_position_command[2],
                _jnt_position_command[3], _jnt_position_command[4],
                _jnt_position_command[5]);
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "机械臂硬件启动成功!");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
              "Stopping ...please wait...");

  _resend_service.reset();

  // If we were in freedrive, exit drag mode before stopping
  if (_in_freedrive) {
    _ptr_robot->DragTeachSwitch(0);
    _ptr_robot->Mode(0);
    _in_freedrive = false;
  }

  _cmd_pub.reset();
  _debug_node.reset();
  _ptr_robot->StopMotion(); // 停止机器人
  _ptr_robot->CloseRPC();   // 销毁实例，连接断开
  _ptr_robot.release();
  RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
              "System successfully stopped!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FairinoHardwareInterface::read(
    const rclcpp::Time &time,
    const rclcpp::Duration &period) {

  JointPos state_data;
  error_t returncode = _ptr_robot->GetActualJointPosDegree(1, &state_data);
  if (returncode == 0) {
    for (int i = 0; i < 6; i++) {
      _jnt_position_state[i] = state_data.jPos[i] / 180.0 * M_PI;
    }
  } else {
    return hardware_interface::return_type::ERROR;
  }
  float speed[6];
  if (_ptr_robot->GetActualJointSpeedsDegree(1, speed) == 0) {
    for (int i = 0; i < 6; i++) {
      _jnt_velocity_state[i] = speed[i] / 180.0 * M_PI;
    }
  }
  float torques[6];
  if (_ptr_robot->GetJointTorques(1, torques) == 0) {
    for (int i = 0; i < 6; i++) {
      _jnt_torque_state[i] = torques[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
FairinoHardwareInterface::write(const rclcpp::Time &time,
                                const rclcpp::Duration &period) {
  // --- Resend robot program ---
  if (_resend_pending.exchange(false)) {
    bool ok = true;
    ok &= (_ptr_robot->ProgramStop()    == 0);
    ok &= (_ptr_robot->RobotEnable(1)   == 0);
    ok &= (_ptr_robot->Mode(0)          == 0);
    ok &= (_ptr_robot->ResetAllError()  == 0);
    ok &= (_ptr_robot->ServoMoveStart() == 0);
    ok &= (_ptr_robot->Mode(0)          == 0);
    for (int i = 0; i < 6; i++) {
      _jnt_position_command[i] = _jnt_position_state[i];
    }
    _resend_success = ok;
    _resend_done    = true;
    return hardware_interface::return_type::OK;
  }

  // --- Freedrive enable request ---
  if (_enable_cmd == 1.0) {
    _enable_cmd = 0.0;
    errno_t err_mode = _ptr_robot->Mode(1);
    errno_t err_drag = _ptr_robot->DragTeachSwitch(1);
    if (err_mode == 0 && err_drag == 0) {
      _async_success = 1.0;
      _in_freedrive = true;
    } else {
      _async_success = 0.0;
      if (err_mode == 0) {
        _ptr_robot->Mode(0);  // rollback
      }
    }
    return hardware_interface::return_type::OK;
  }

  // --- Freedrive abort request ---
  if (_abort_cmd == 1.0) {
    _abort_cmd = 0.0;
    errno_t err_drag = _ptr_robot->DragTeachSwitch(0);
    errno_t err_mode = _ptr_robot->Mode(0);
    // Sync command to actual so ServoJ starts from the current position
    for (int i = 0; i < 6; i++) {
      _jnt_position_command[i] = _jnt_position_state[i];
    }
    _ptr_robot->ServoMoveStart();
    if (err_drag == 0 && err_mode == 0) {
      _async_success = 1.0;
    } else {
      _async_success = 0.0;
    }
    _in_freedrive = false;
    return hardware_interface::return_type::OK;
  }

  // --- Pure freedrive: robot is in SDK drag mode, no joint writes ---
  if (_in_freedrive) {
    return hardware_interface::return_type::OK;
  }

  // --- Normal ServoJ path ---
  if (_control_mode == 0) { // 位置控制模式
    if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[5],
                    [](double c) { return not std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }
    JointPos cmd;
    ExaxisPos extcmd{0, 0, 0, 0};
    for (auto j = 0; j < 6; j++) {
      cmd.jPos[j] = _jnt_position_command[j] / M_PI * 180.0;
    }
    int returncode = _ptr_robot->ServoJ(&cmd, &extcmd, 0, 0, 0.008, 0, 0);
    if (returncode != 0) {
      RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                  "ServoJ指令下发错误,错误码:%d", returncode);
    }
    _cmd_msg.header.stamp = time;
    for (int j = 0; j < 6; j++) {
      _cmd_msg.position[j] = _jnt_position_command[j];
    }
    _cmd_pub->publish(_cmd_msg);
  } else if (_control_mode == 1) { // 扭矩控制模式
    if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],
                    [](double c) { return not std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }
    //_ptr_robot->write(_jnt_torque_command);//注意单位转换
  } else {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "指令发送错误:未识别当前所处控制模式");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

} // namespace fairino_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface,
                       hardware_interface::SystemInterface)
