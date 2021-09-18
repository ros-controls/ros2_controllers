// Copyright 2014, SRI International
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

/// \author Sachin Chitta, Adolfo Rodriguez Tsouroukdissian

#ifndef GRIPPER_CONTROLLERS__GRIPPER_ACTION_CONTROLLER_HPP_
#define GRIPPER_CONTROLLERS__GRIPPER_ACTION_CONTROLLER_HPP_

// C++ standard
#include <cassert>
#include <memory>
#include <stdexcept>
#include <string>
// TODO(JafarAbdi): Remove experimental once the default standard is C++17
#include "experimental/optional"

// ROS
#include "rclcpp/rclcpp.hpp"

// ROS messages
#include "control_msgs/action/gripper_command.hpp"

// rclcpp_action
#include "rclcpp_action/create_server.hpp"

// ros_controls
#include "controller_interface/controller_interface.hpp"
#include "gripper_controllers/visibility_control.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_server_goal_handle.h"

// Project
#include "gripper_controllers/hardware_interface_adapter.hpp"

namespace gripper_action_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * \brief Controller for executing a gripper command action for simple
 * single-dof grippers.
 *
 * \tparam HardwareInterface Controller hardware interface. Currently \p
 * hardware_interface::HW_IF_POSITION and \p
 * hardware_interface::HW_IF_EFFORT are supported out-of-the-box.
 */
template <const char * HardwareInterface>
class GripperActionController : public controller_interface::ControllerInterface
{
public:
  /**
   * \brief Store position and max effort in struct to allow easier realtime
   * buffer usage
   */
  struct Commands
  {
    double position_;    // Last commanded position
    double max_effort_;  // Max allowed effort
  };

  GRIPPER_ACTION_CONTROLLER_PUBLIC GripperActionController();

  /**
   * @brief command_interface_configuration This controller requires the
   * position command interfaces for the controlled joints
   */
  GRIPPER_ACTION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief command_interface_configuration This controller requires the
   * position and velocity state interfaces for the controlled joints
   */
  GRIPPER_ACTION_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GRIPPER_ACTION_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  GRIPPER_ACTION_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  GRIPPER_ACTION_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  GRIPPER_ACTION_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  GRIPPER_ACTION_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  realtime_tools::RealtimeBuffer<Commands> command_;
  // pre-allocated memory that is re-used to set the realtime buffer
  Commands command_struct_, command_struct_rt_;

private:
  using GripperCommandAction = control_msgs::action::GripperCommand;
  using ActionServer = rclcpp_action::Server<GripperCommandAction>;
  using ActionServerPtr = ActionServer::SharedPtr;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommandAction>;
  using RealtimeGoalHandle =
    realtime_tools::RealtimeServerGoalHandle<control_msgs::action::GripperCommand>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;

  using HwIfaceAdapter = HardwareInterfaceAdapter<HardwareInterface>;

  bool update_hold_position_;

  bool verbose_ = false;  ///< Hard coded verbose flag to help in debugging
  std::string name_;      ///< Controller name.
  std::experimental::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
  std::experimental::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
  std::experimental::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_velocity_state_interface_;

  std::string joint_name_;  ///< Controlled joint names.

  HwIfaceAdapter hw_iface_adapter_;  ///< Adapts desired goal state to HW interface.

  RealtimeGoalHandlePtr rt_active_goal_;  ///< Currently active action goal, if any.
  control_msgs::action::GripperCommand::Result::SharedPtr pre_alloc_result_;

  rclcpp::Duration action_monitor_period_;

  // ROS API
  ActionServerPtr action_server_;

  rclcpp::TimerBase::SharedPtr goal_handle_timer_;

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GripperCommandAction::Goal> goal);

  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandle> goal_handle);

  void accepted_callback(std::shared_ptr<GoalHandle> goal_handle);

  void preempt_active_goal();

  void set_hold_position();

  rclcpp::Time last_movement_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);  ///< Store stall time
  double computed_command_;                                             ///< Computed command

  double stall_timeout_,
    stall_velocity_threshold_;  ///< Stall related parameters
  double default_max_effort_;   ///< Max allowed effort
  double goal_tolerance_;

  /**
   * \brief Check for success and publish appropriate result and feedback.
   **/
  void check_for_success(
    const rclcpp::Time & time, double error_position, double current_position,
    double current_velocity);
};

}  // namespace gripper_action_controller

#include "gripper_controllers/gripper_action_controller_impl.hpp"

#endif  // GRIPPER_CONTROLLERS__GRIPPER_ACTION_CONTROLLER_HPP_
