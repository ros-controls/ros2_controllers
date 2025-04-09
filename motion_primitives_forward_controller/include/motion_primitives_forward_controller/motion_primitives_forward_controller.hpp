// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef MOTION_PRIMITIVES_FORWARD_CONTROLLER__MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
#define MOTION_PRIMITIVES_FORWARD_CONTROLLER__MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <queue>

#include "controller_interface/controller_interface.hpp"
#include <motion_primitives_forward_controller/motion_primitives_forward_controller_parameters.hpp>
#include "motion_primitives_forward_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "industrial_robot_motion_interfaces/msg/motion_primitive.hpp"
#include "std_msgs/msg/int8.hpp"


namespace motion_primitives_forward_controller
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

class MotionPrimitivesForwardController : public controller_interface::ControllerInterface
{
public:
  MOTION_PRIMITIVES_CONTTROLLER_PKG__VISIBILITY_PUBLIC
  MotionPrimitivesForwardController();

  MOTION_PRIMITIVES_CONTTROLLER_PKG__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  MOTION_PRIMITIVES_CONTTROLLER_PKG__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  MOTION_PRIMITIVES_CONTTROLLER_PKG__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  MOTION_PRIMITIVES_CONTTROLLER_PKG__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MOTION_PRIMITIVES_CONTTROLLER_PKG__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MOTION_PRIMITIVES_CONTTROLLER_PKG__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  MOTION_PRIMITIVES_CONTTROLLER_PKG__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

    // state and command message types
    using ControllerReferenceMsg = industrial_robot_motion_interfaces::msg::MotionPrimitive;
    using ControllerStateMsg = std_msgs::msg::Int8;
  

protected:
  std::shared_ptr<motion_primitives_forward_controller::ParamListener> param_listener_;
  motion_primitives_forward_controller::Params params_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  // realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  // callback for topic interface
  MOTION_PRIMITIVES_CONTTROLLER_PKG__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg); // callback for reference message
  // std::atomic<bool> new_msg_available_ = false; // flag to indicate if new message is available
  void reset_command_interfaces(); // Reset all command interfaces to NaN()
  bool set_command_interfaces(); // Set command interfaces from the message

  std::queue<std::shared_ptr<ControllerReferenceMsg>> msg_queue_;
  size_t queue_size_ = 0; 
  
  bool print_error_once_ = true; // Flag to print error message only once if ExecutionState::ERROR
};

}  // namespace motion_primitives_forward_controller

#endif  // MOTION_PRIMITIVES_FORWARD_CONTROLLER__MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
