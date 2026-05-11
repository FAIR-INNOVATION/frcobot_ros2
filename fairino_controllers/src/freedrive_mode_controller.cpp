#include "fairino_controllers/freedrive_mode_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace fairino_controllers {

controller_interface::CallbackReturn FreedriveModeController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FreedriveModeController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"freedrive/enable", "freedrive/abort", "freedrive/async_success"}
  };
}

controller_interface::InterfaceConfiguration
FreedriveModeController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn FreedriveModeController::on_configure(
  const rclcpp_lifecycle::State &)
{
  enable_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "~/enable_freedrive", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      desired_freedrive_.store(msg->data);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FreedriveModeController::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto find_iface = [&](const std::string & name)
    -> std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  {
    auto it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&](auto & iface) { return iface.get_name() == name; });
    if (it == command_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Command interface '%s' not found", name.c_str());
      return std::nullopt;
    }
    return std::ref(*it);
  };

  enable_iface_        = find_iface("freedrive/enable");
  abort_iface_         = find_iface("freedrive/abort");
  async_success_iface_ = find_iface("freedrive/async_success");

  if (!enable_iface_ || !abort_iface_ || !async_success_iface_) {
    return controller_interface::CallbackReturn::ERROR;
  }

  enable_iface_->get().set_value(0.0);
  abort_iface_->get().set_value(0.0);
  async_success_iface_->get().set_value(0.0);

  desired_freedrive_.store(false);
  current_freedrive_ = false;
  change_requested_  = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FreedriveModeController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Best-effort abort if we were in freedrive when deactivated unexpectedly
  if (current_freedrive_ && abort_iface_) {
    async_success_iface_->get().set_value(ASYNC_WAITING);
    abort_iface_->get().set_value(1.0);
  }

  enable_iface_.reset();
  abort_iface_.reset();
  async_success_iface_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FreedriveModeController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // --- Poll async_success while a HWI operation is in flight ---
  if (change_requested_) {
    const double val = async_success_iface_->get().get_value();

    if (val == 1.0) {
      current_freedrive_ = desired_freedrive_.load();
      change_requested_  = false;
      async_success_iface_->get().set_value(0.0);
      RCLCPP_INFO(get_node()->get_logger(),
        "Freedrive %s", current_freedrive_ ? "enabled" : "disabled");
    } else if (val == 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Freedrive %s failed — HWI SDK call returned error",
        desired_freedrive_.load() ? "enable" : "disable");
      desired_freedrive_.store(current_freedrive_);  // revert
      change_requested_ = false;
      async_success_iface_->get().set_value(0.0);
    }
    // val == ASYNC_WAITING: still in-flight, wait
    return controller_interface::return_type::OK;
  }

  // --- Detect new transition request ---
  const bool want = desired_freedrive_.load();

  if (want && !current_freedrive_) {
    async_success_iface_->get().set_value(ASYNC_WAITING);
    enable_iface_->get().set_value(1.0);
    change_requested_ = true;
  } else if (!want && current_freedrive_) {
    async_success_iface_->get().set_value(ASYNC_WAITING);
    abort_iface_->get().set_value(1.0);
    change_requested_ = true;
  }

  return controller_interface::return_type::OK;
}

}  // namespace fairino_controllers

PLUGINLIB_EXPORT_CLASS(
  fairino_controllers::FreedriveModeController,
  controller_interface::ControllerInterface)
