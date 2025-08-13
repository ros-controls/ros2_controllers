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

#ifndef MOTION_PRIMITIVES_CONTROLLERS__MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
#define MOTION_PRIMITIVES_CONTROLLERS__MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_

#include <memory>
#include <motion_primitives_controllers/motion_primitives_forward_controller_parameters.hpp>
#include "motion_primitives_controllers/motion_primitives_base_controller.hpp"

namespace motion_primitives_controllers
{

class MotionPrimitivesForwardController : public MotionPrimitivesBaseController
{
public:
  MotionPrimitivesForwardController() = default;
  ~MotionPrimitivesForwardController() override = default;

  controller_interface::CallbackReturn on_init() override;

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

  using ExecuteMotionAction = control_msgs::action::ExecuteMotionPrimitiveSequence;
  rclcpp_action::Server<ExecuteMotionAction>::SharedPtr action_server_;
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteMotionAction::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotionAction>> goal_handle);
  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteMotionAction>> goal_handle);
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<ExecuteMotionAction>;
  realtime_tools::RealtimeThreadSafeBox<std::shared_ptr<RealtimeGoalHandle>> rt_goal_handle_;
};

}  // namespace motion_primitives_controllers

#endif  // MOTION_PRIMITIVES_CONTROLLERS__MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
