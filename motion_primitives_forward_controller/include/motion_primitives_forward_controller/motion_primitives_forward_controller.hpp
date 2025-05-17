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

  using ControllerReferenceMsg = industrial_robot_motion_interfaces::msg::MotionPrimitive;
  using ControllerStateMsg = std_msgs::msg::Int8;

protected:
  std::shared_ptr<motion_primitives_forward_controller::ParamListener> param_listener_;
  motion_primitives_forward_controller::Params params_;

  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  std::queue<std::shared_ptr<ControllerReferenceMsg>> msg_queue_;

private:
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  void reset_command_interfaces();
  bool set_command_interfaces();
  void reset_controller_reference_msg(std::shared_ptr<ControllerReferenceMsg> & msg);

  size_t queue_size_ = 0;
  std::mutex command_mutex_;
  bool print_error_once_ = true;
  bool robot_stopped_ = false;
};

}  // namespace motion_primitives_forward_controller

#endif  // MOTION_PRIMITIVES_FORWARD_CONTROLLER__MOTION_PRIMITIVES_FORWARD_CONTROLLER_HPP_
