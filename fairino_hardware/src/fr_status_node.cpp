#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <string>

#include "fairino_msgs/msg/robot_nonrt_state.hpp"
#include "libfairino/include/robot_types.h"
#include "rclcpp/rclcpp.hpp"

static constexpr int STATUS_PORT = 20004;
static constexpr uint16_t FRAME_HEAD = 0x5A5A;

static int connect_to_robot(const std::string & ip)
{
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    throw std::runtime_error("socket() failed");
  }

  int flag = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(STATUS_PORT);
  addr.sin_addr.s_addr = inet_addr(ip.c_str());

  if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(fd);
    throw std::runtime_error("connect() to " + ip + ":20004 failed");
  }
  return fd;
}

static bool recv_full(int fd, void * buf, size_t len)
{
  size_t received = 0;
  auto * ptr = static_cast<uint8_t *>(buf);
  while (received < len) {
    ssize_t n = recv(fd, ptr + received, len - received, 0);
    if (n <= 0) {
      return false;
    }
    received += n;
  }
  return true;
}

static fairino_msgs::msg::RobotNonrtState pkg_to_msg(const ROBOT_STATE_PKG & s)
{
  fairino_msgs::msg::RobotNonrtState msg;

  msg.prg_state = s.program_state;
  msg.rbt_state = s.robot_state;
  msg.rbt_main_code = s.main_code;
  msg.rbt_sub_code = s.sub_code;
  msg.robot_mode = s.robot_mode;

  msg.j1_cur_pos = s.jt_cur_pos[0];
  msg.j2_cur_pos = s.jt_cur_pos[1];
  msg.j3_cur_pos = s.jt_cur_pos[2];
  msg.j4_cur_pos = s.jt_cur_pos[3];
  msg.j5_cur_pos = s.jt_cur_pos[4];
  msg.j6_cur_pos = s.jt_cur_pos[5];

  msg.cart_x_cur_pos = s.tl_cur_pos[0];
  msg.cart_y_cur_pos = s.tl_cur_pos[1];
  msg.cart_z_cur_pos = s.tl_cur_pos[2];
  msg.cart_a_cur_pos = s.tl_cur_pos[3];
  msg.cart_b_cur_pos = s.tl_cur_pos[4];
  msg.cart_c_cur_pos = s.tl_cur_pos[5];

  msg.flange_x_cur_pos = s.flange_cur_pos[0];
  msg.flange_y_cur_pos = s.flange_cur_pos[1];
  msg.flange_z_cur_pos = s.flange_cur_pos[2];
  msg.flange_a_cur_pos = s.flange_cur_pos[3];
  msg.flange_b_cur_pos = s.flange_cur_pos[4];
  msg.flange_c_cur_pos = s.flange_cur_pos[5];

  msg.j1_actual_qd = s.actual_qd[0];
  msg.j2_actual_qd = s.actual_qd[1];
  msg.j3_actual_qd = s.actual_qd[2];
  msg.j4_actual_qd = s.actual_qd[3];
  msg.j5_actual_qd = s.actual_qd[4];
  msg.j6_actual_qd = s.actual_qd[5];

  msg.j1_actual_qdd = s.actual_qdd[0];
  msg.j2_actual_qdd = s.actual_qdd[1];
  msg.j3_actual_qdd = s.actual_qdd[2];
  msg.j4_actual_qdd = s.actual_qdd[3];
  msg.j5_actual_qdd = s.actual_qdd[4];
  msg.j6_actual_qdd = s.actual_qdd[5];

  msg.cart_lin_cmd_speed = s.target_TCP_CmpSpeed[0];
  msg.cart_rot_cmd_speed = s.target_TCP_CmpSpeed[1];
  msg.cart_x_cmd_speed = s.target_TCP_Speed[0];
  msg.cart_y_cmd_speed = s.target_TCP_Speed[1];
  msg.cart_z_cmd_speed = s.target_TCP_Speed[2];
  msg.cart_a_cmd_speed = s.target_TCP_Speed[3];
  msg.cart_b_cmd_speed = s.target_TCP_Speed[4];
  msg.cart_c_cmd_speed = s.target_TCP_Speed[5];

  msg.cart_lin_cur_speed = s.actual_TCP_CmpSpeed[0];
  msg.cart_rot_cur_speed = s.actual_TCP_CmpSpeed[1];
  msg.cart_x_cur_speed = s.actual_TCP_Speed[0];
  msg.cart_y_cur_speed = s.actual_TCP_Speed[1];
  msg.cart_z_cur_speed = s.actual_TCP_Speed[2];
  msg.cart_a_cur_speed = s.actual_TCP_Speed[3];
  msg.cart_b_cur_speed = s.actual_TCP_Speed[4];
  msg.cart_c_cur_speed = s.actual_TCP_Speed[5];

  msg.j1_cur_tor = s.jt_cur_tor[0];
  msg.j2_cur_tor = s.jt_cur_tor[1];
  msg.j3_cur_tor = s.jt_cur_tor[2];
  msg.j4_cur_tor = s.jt_cur_tor[3];
  msg.j5_cur_tor = s.jt_cur_tor[4];
  msg.j6_cur_tor = s.jt_cur_tor[5];

  msg.tool_num = s.tool;
  msg.work_num = s.user;
  msg.dgt_output_h = s.cl_dgt_output_h;
  msg.dgt_output_l = s.cl_dgt_output_l;
  msg.tl_dgt_output_l = s.tl_dgt_output_l;
  msg.dgt_input_h = s.cl_dgt_input_h;
  msg.dgt_input_l = s.cl_dgt_input_l;
  msg.tl_dgt_input_l = s.tl_dgt_input_l;
  msg.cl_analog_input_1 = s.cl_analog_input[0];
  msg.cl_analog_input_2 = s.cl_analog_input[1];
  msg.tl_anglog_input = s.tl_anglog_input;

  msg.ft_fx_raw_data = s.ft_sensor_raw_data[0];
  msg.ft_fy_raw_data = s.ft_sensor_raw_data[1];
  msg.ft_fz_raw_data = s.ft_sensor_raw_data[2];
  msg.ft_tx_raw_data = s.ft_sensor_raw_data[3];
  msg.ft_ty_raw_data = s.ft_sensor_raw_data[4];
  msg.ft_tz_raw_data = s.ft_sensor_raw_data[5];

  msg.ft_fx_data = s.ft_sensor_data[0];
  msg.ft_fy_data = s.ft_sensor_data[1];
  msg.ft_fz_data = s.ft_sensor_data[2];
  msg.ft_tx_data = s.ft_sensor_data[3];
  msg.ft_ty_data = s.ft_sensor_data[4];
  msg.ft_tz_data = s.ft_sensor_data[5];
  msg.ft_actstatus = s.ft_sensor_active;

  msg.emg = s.EmergencyStop;
  msg.motion_done = s.motion_done;
  msg.grip_motion_done = s.gripper_motiondone;
  msg.mc_queue_len = s.mc_queue_len;
  msg.collision_err = s.collisionState;
  msg.trajectory_pnum = s.trajectory_pnum;
  msg.safety_stop1_state = s.safety_stop0_state;
  msg.safety_stop2_state = s.safety_stop1_state;

  msg.gripper_fault_id = s.gripper_fault_id;
  msg.grippererro = s.gripper_fault;
  msg.gripper_active = s.gripper_active;
  msg.gripper_position = s.gripper_position;
  msg.gripper_speed = s.gripper_speed;
  msg.gripper_current = s.gripper_current;
  msg.gripper_temp = s.gripper_temp;
  msg.gripper_voltage = s.gripper_voltage;

  msg.aux_servo_id = s.aux_state.servoId;
  msg.aux_servo_err = s.aux_state.servoErrCode;
  msg.aux_servo_state = s.aux_state.servoState;
  msg.aux_servo_pos = s.aux_state.servoPos;
  msg.aux_servo_vel = s.aux_state.servoVel;
  msg.aux_servo_torque = s.aux_state.servoTorque;

  msg.ext_di_state_1 = s.extDIState[0];
  msg.ext_di_state_2 = s.extDIState[1];
  msg.ext_di_state_3 = s.extDIState[2];
  msg.ext_di_state_4 = s.extDIState[3];
  msg.ext_di_state_5 = s.extDIState[4];
  msg.ext_di_state_6 = s.extDIState[5];
  msg.ext_di_state_7 = s.extDIState[6];
  msg.ext_di_state_8 = s.extDIState[7];

  msg.ext_do_state_1 = s.extDOState[0];
  msg.ext_do_state_2 = s.extDOState[1];
  msg.ext_do_state_3 = s.extDOState[2];
  msg.ext_do_state_4 = s.extDOState[3];
  msg.ext_do_state_5 = s.extDOState[4];
  msg.ext_do_state_6 = s.extDOState[5];
  msg.ext_do_state_7 = s.extDOState[6];
  msg.ext_do_state_8 = s.extDOState[7];

  msg.ext_ai_state_1 = s.extAIState[0];
  msg.ext_ai_state_2 = s.extAIState[1];
  msg.ext_ai_state_3 = s.extAIState[2];
  msg.ext_ai_state_4 = s.extAIState[3];

  msg.ext_ao_state_1 = s.extAOState[0];
  msg.ext_ao_state_2 = s.extAOState[1];
  msg.ext_ao_state_3 = s.extAOState[2];
  msg.ext_ao_state_4 = s.extAOState[3];

  msg.rbt_enable_state = s.rbtEnableState;
  msg.end_lua_err_code = s.endLuaErrCode;
  msg.cl_analog_output_1 = s.cl_analog_output[0];
  msg.cl_analog_output_2 = s.cl_analog_output[1];
  msg.tl_analog_output = s.tl_analog_output;

  msg.gripper_rot_num = s.gripperRotNum;
  msg.gripper_rot_speed = s.gripperRotSpeed;
  msg.gripper_rot_torque = s.gripperRotTorque;

  msg.weldbreakoffstate = s.weldingBreakOffState.breakOffState;
  msg.weldarcstate = s.weldingBreakOffState.weldArcState;

  msg.j1_tgt_tor = s.jt_tgt_tor[0];
  msg.j2_tgt_tor = s.jt_tgt_tor[1];
  msg.j3_tgt_tor = s.jt_tgt_tor[2];
  msg.j4_tgt_tor = s.jt_tgt_tor[3];
  msg.j5_tgt_tor = s.jt_tgt_tor[4];
  msg.j6_tgt_tor = s.jt_tgt_tor[5];

  msg.jwide_voltage_ctrl_box_temp = s.wideVoltageCtrlBoxTemp;
  msg.wide_voltage_ctrl_box_fan_current = s.wideVoltageCtrlBoxFanCurrent;

  msg.tool_coord_x = s.toolCoord[0];
  msg.tool_coord_y = s.toolCoord[1];
  msg.tool_coord_z = s.toolCoord[2];
  msg.tool_coord_a = s.toolCoord[3];
  msg.tool_coord_b = s.toolCoord[4];
  msg.tool_coord_c = s.toolCoord[5];

  msg.wobj_coord_x = s.wobjCoord[0];
  msg.wobj_coord_y = s.wobjCoord[1];
  msg.wobj_coord_z = s.wobjCoord[2];
  msg.wobj_coord_a = s.wobjCoord[3];
  msg.wobj_coord_b = s.wobjCoord[4];
  msg.wobj_coord_c = s.wobjCoord[5];

  msg.ex_tool_coord_x = s.extoolCoord[0];
  msg.ex_tool_coord_y = s.extoolCoord[1];
  msg.ex_tool_coord_z = s.extoolCoord[2];
  msg.ex_tool_coord_a = s.extoolCoord[3];
  msg.ex_tool_coord_b = s.extoolCoord[4];
  msg.ex_tool_coord_c = s.extoolCoord[5];

  msg.ex_axis_coord_x = s.exAxisCoord[0];
  msg.ex_axis_coord_y = s.exAxisCoord[1];
  msg.ex_axis_coord_z = s.exAxisCoord[2];
  msg.ex_axis_coord_a = s.exAxisCoord[3];
  msg.ex_axis_coord_b = s.exAxisCoord[4];
  msg.ex_axis_coord_c = s.exAxisCoord[5];

  msg.load = s.load;
  msg.load_cog_x = s.loadCog[0];
  msg.load_cog_y = s.loadCog[1];
  msg.load_cog_z = s.loadCog[2];

  msg.j1_last_servoj_target = s.lastServoTarget[0];
  msg.j2_last_servoj_target = s.lastServoTarget[1];
  msg.j3_last_servoj_target = s.lastServoTarget[2];
  msg.j4_last_servoj_target = s.lastServoTarget[3];
  msg.j5_last_servoj_target = s.lastServoTarget[4];
  msg.j6_last_servoj_target = s.lastServoTarget[5];
  msg.servoj_cmd_num = s.servoJCmdNum;

  return msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fr_status_node");

  node->declare_parameter<std::string>("ip_address", "192.168.0.2");
  const std::string ip_address = node->get_parameter("ip_address").as_string();

  auto pub = node->create_publisher<fairino_msgs::msg::RobotNonrtState>("nonrt_state_data", 1);

  RCLCPP_INFO(node->get_logger(), "Connecting to %s:%d ...", ip_address.c_str(), STATUS_PORT);

  int fd = -1;
  while (rclcpp::ok()) {
    try {
      fd = connect_to_robot(ip_address);
      break;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "%s — retrying in 3s", e.what());
      rclcpp::sleep_for(std::chrono::seconds(3));
    }
  }

  if (fd < 0) {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Connected to robot status port");

  ROBOT_STATE_PKG pkg;
  while (rclcpp::ok()) {
    if (!recv_full(fd, &pkg, sizeof(pkg))) {
      RCLCPP_WARN(node->get_logger(), "Status socket disconnected, reconnecting...");
      close(fd);
      fd = -1;
      while (rclcpp::ok()) {
        try {
          fd = connect_to_robot(ip_address);
          break;
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node->get_logger(), "%s — retrying in 3s", e.what());
          rclcpp::sleep_for(std::chrono::seconds(3));
        }
      }
      continue;
    }

    if (pkg.frame_head != FRAME_HEAD) {
      // Lost sync — drain one byte and try again
      uint8_t byte;
      recv(fd, &byte, 1, 0);
      continue;
    }

    pub->publish(pkg_to_msg(pkg));
    rclcpp::spin_some(node);
  }

  if (fd >= 0) {
    close(fd);
  }
  rclcpp::shutdown();
  return 0;
}
