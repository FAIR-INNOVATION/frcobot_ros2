#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <std_msgs/msg/bool.hpp>

namespace fairino_controllers {

class FreedriveModeController : public controller_interface::ControllerInterface {
public:
  FreedriveModeController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Resolved command interface refs (set in on_activate, nullopt until then)
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> enable_iface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> abort_iface_;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> async_success_iface_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

  std::atomic<bool> desired_freedrive_{false};

  bool current_freedrive_{false};  // tracks actual HWI-confirmed state
  bool change_requested_{false};   // true while waiting for async_success to resolve

  static constexpr double ASYNC_WAITING{2.0};
};

}  // namespace fairino_controllers
