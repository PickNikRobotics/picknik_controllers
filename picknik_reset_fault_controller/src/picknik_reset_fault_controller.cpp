// Copyright 2021, PickNik Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-11-25
 *
 */
//----------------------------------------------------------------------

#include "picknik_reset_fault_controller/picknik_reset_fault_controller.hpp"
#include <memory>
#include "hardware_interface/loaned_command_interface.hpp"

namespace picknik_reset_fault_controller
{
using hardware_interface::LoanedCommandInterface;

PicknikResetFaultController::PicknikResetFaultController()
: controller_interface::ControllerInterface(), trigger_command_srv_(nullptr)
{
}

controller_interface::InterfaceConfiguration
PicknikResetFaultController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back("reset_fault/command");
  config.names.emplace_back("reset_fault/async_success");

  return config;
}

controller_interface::InterfaceConfiguration
PicknikResetFaultController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back("reset_fault/internal_fault");

  return config;
}

CallbackReturn PicknikResetFaultController::on_init() { return CallbackReturn::SUCCESS; }

controller_interface::return_type PicknikResetFaultController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    state_.data = static_cast<bool>(state_interfaces_[StateInterfaces::IN_FAULT].get_value());
    realtime_publisher_->msg_.data = state_.data;
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

CallbackReturn PicknikResetFaultController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn PicknikResetFaultController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interfaces_[CommandInterfaces::RESET_FAULT_CMD].set_value(NO_CMD);
  command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].set_value(NO_CMD);
  try
  {
    fault_pub_ = get_node()->create_publisher<FbkType>("~/internal_fault", 1);
    realtime_publisher_ = std::make_unique<StatePublisher>(fault_pub_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }
  trigger_command_srv_ = get_node()->create_service<example_interfaces::srv::Trigger>(
    "~/reset_fault", std::bind(
                       &PicknikResetFaultController::resetFault, this, std::placeholders::_1,
                       std::placeholders::_2));

  return CallbackReturn::SUCCESS;
}

CallbackReturn PicknikResetFaultController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  trigger_command_srv_.reset();
  command_interfaces_[CommandInterfaces::RESET_FAULT_CMD].set_value(NO_CMD);
  command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].set_value(NO_CMD);

  return CallbackReturn::SUCCESS;
}

bool PicknikResetFaultController::resetFault(
  const CmdType::Request::SharedPtr /*req*/, CmdType::Response::SharedPtr resp)
{
  command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
  command_interfaces_[CommandInterfaces::RESET_FAULT_CMD].set_value(ISSUE_CMD);

  RCLCPP_INFO(get_node()->get_logger(), "Trying to reset faults on hardware controller.");

  while (command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].get_value() ==
         ASYNC_WAITING)
  {
    // Asynchronous wait until the hardware interface has set the io value
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  resp->success = static_cast<bool>(
    command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].get_value());

  RCLCPP_INFO(
    get_node()->get_logger(), "Resetting fault on hardware controller '%s'!",
    resp->success ? "succeeded" : "failed");

  return resp->success;
}

}  // namespace picknik_reset_fault_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  picknik_reset_fault_controller::PicknikResetFaultController,
  controller_interface::ControllerInterface)
