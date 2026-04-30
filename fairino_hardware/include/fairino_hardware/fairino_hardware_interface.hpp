#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "libfairino/include/robot.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visibility_control.h"
#include <atomic>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

namespace fairino_hardware {

class FairinoHardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  // FAIRINO_HARDWARE_PUBLIC
  // hardware_interface::CallbackReturn on_configure(const
  // rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  double _jnt_position_command[6];
  double _jnt_velocity_command[6];
  double _jnt_torque_command[6];
  double _jnt_position_state[6];
  double _jnt_velocity_state[6];
  double _jnt_torque_state[6];
  int _control_mode;
  std::unique_ptr<FRRobot> _ptr_robot;

  // Debug publisher: echoes the command sent to ServoJ each cycle
  rclcpp::Node::SharedPtr _debug_node;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _cmd_pub;
  sensor_msgs::msg::JointState _cmd_msg;

  // Freedrive command interfaces (backed by doubles, claimed by FreedriveModeController)
  double _enable_cmd{0.0};
  double _abort_cmd{0.0};
  double _async_success{0.0};

  // Internal state
  bool _in_freedrive{false};

  // Resend robot program — service on get_node(), flag bridging to write()
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _resend_service;
  std::atomic<bool> _resend_pending{false};
  std::atomic<bool> _resend_done{false};
  std::atomic<bool> _resend_success{false};
};

} // namespace fairino_hardware

#endif
