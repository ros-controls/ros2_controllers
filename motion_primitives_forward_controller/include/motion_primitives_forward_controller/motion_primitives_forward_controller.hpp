// Copyright (c) 2025, b»robotized
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
//
// Authors: Mathias Fuhrer

#ifndef MOTION_PRIMITIVES_FORWARD_CONTROLLER__MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
#define MOTION_PRIMITIVES_FORWARD_CONTROLLER__MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <motion_primitives_forward_controller/motion_primitives_forward_controller_parameters.hpp>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "industrial_robot_motion_interfaces/action/execute_motion.hpp"
#include "industrial_robot_motion_interfaces/msg/motion_primitive.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace motion_primitives_forward_controller
{
enum class ExecutionState : uint8_t
{
  IDLE = 0,
  EXECUTING = 1,
  SUCCESS = 2,
  ERROR = 3,
  STOPPED = 4
};

enum class MotionType : uint8_t
{
  LINEAR_JOINT = 10,
  LINEAR_CARTESIAN = 50,
  CIRCULAR_CARTESIAN = 51,

  STOP_MOTION = 66,
  RESET_STOP = 67,

  MOTION_SEQUENCE_START = 100,
  MOTION_SEQUENCE_END = 101
};

enum class ReadyForNewPrimitive : uint8_t
{
  NOT_READY = 0,
  READY = 1
};

class MotionPrimitivesForwardController : public controller_interface::ControllerInterface
{
public:
  MotionPrimitivesForwardController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<motion_primitives_forward_controller::ParamListener> param_listener_;
  motion_primitives_forward_controller::Params params_;

  using MotionPrimitive = industrial_robot_motion_interfaces::msg::MotionPrimitive;
  std::queue<std::shared_ptr<MotionPrimitive>> moprim_queue_;

  using ExecuteMotion = industrial_robot_motion_interfaces::action::ExecuteMotion;
  rclcpp_action::Server<ExecuteMotion>::SharedPtr action_server_;
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteMotion::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotion>> goal_handle);
  void goal_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotion>> goal_handle);
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotion>> pending_action_goal_;

  void reset_command_interfaces();
  bool set_command_interfaces();

  bool print_error_once_ = true;
  // cancel requested by the action server
  std::atomic<bool> cancel_requested_ = false;
  // robot stop command sent to the hardware interface
  std::atomic<bool> robot_stop_requested_ = false;
  bool was_executing_ = false;
  ExecutionState execution_status_;
  ReadyForNewPrimitive ready_for_new_primitive_;
};

}  // namespace motion_primitives_forward_controller

#endif  // MOTION_PRIMITIVES_FORWARD_CONTROLLER__MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
