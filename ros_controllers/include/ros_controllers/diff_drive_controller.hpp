// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROS_CONTROLLERS__DIFF_DRIVE_CONTROLLER_HPP_
#define ROS_CONTROLLERS__DIFF_DRIVE_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros_controllers/visibility_control.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include <memory>
#include <vector>

namespace ros_controllers {
class DiffDriveController : public controller_interface::ControllerInterface
{
   using Twist = geometry_msgs::msg::Twist;

 public:
   ROS_CONTROLLERS_PUBLIC
   DiffDriveController();

   ROS_CONTROLLERS_PUBLIC
   DiffDriveController(std::vector<std::string> left_wheel_names,
                       std::vector<std::string> right_wheel_names,
                       std::vector<std::string> operation_mode_names);

   ROS_CONTROLLERS_PUBLIC
   controller_interface::controller_interface_ret_t
   init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware, const std::string& controller_name) override;

   ROS_CONTROLLERS_PUBLIC
   controller_interface::controller_interface_ret_t update() override;

   ROS_CONTROLLERS_PUBLIC
   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_configure(const rclcpp_lifecycle::State& previous_state) override;

   ROS_CONTROLLERS_PUBLIC
   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_activate(const rclcpp_lifecycle::State& previous_state) override;

   ROS_CONTROLLERS_PUBLIC
   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

   ROS_CONTROLLERS_PUBLIC
   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

   ROS_CONTROLLERS_PUBLIC
   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_error(const rclcpp_lifecycle::State& previous_state) override;

   ROS_CONTROLLERS_PUBLIC
   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

 private:
   struct WheelHandle
   {
      const hardware_interface::JointStateHandle* state{nullptr};
      hardware_interface::JointCommandHandle* command{nullptr};
   };

   std::vector<std::string> left_wheel_names_{};
   std::vector<std::string> right_wheel_names_{};

   std::vector<WheelHandle> registered_left_wheel_handles_{};
   std::vector<WheelHandle> registered_right_wheel_handles_{};

   struct WheelParams
   {
      size_t wheels_per_side{1};
      double separation{0.21}; // w.r.t. the midpoint of the wheel width
      double radius{0.05};     // Assumed to be the same for both wheels
      double separation_multiplier{1.0};
      double left_radius_multiplier{1.0};
      double right_radius_multiplier{1.0};
   } wheel_params_{};

   // Timeout to consider cmd_vel commands old
   std::chrono::milliseconds cmd_vel_timeout_{500};
   std::vector<double> left_previous_commands_{};
   std::vector<double> right_previous_commands_{};

   std::vector<std::string> write_op_names_{};
   std::vector<hardware_interface::OperationModeHandle*> registered_operation_mode_handles_{};

   std::string base_frame_id_{"base_link"};
   std::string odom_frame_id_{"odom"};

   // TODO(karsten1987): eventually activate and deactive subscriber directly when its supported
   bool subscriber_is_active_{false};
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_subscriber_{nullptr};

   std::shared_ptr<Twist> velocity_msg_ptr_{nullptr};

   bool is_halted{false};

   bool reset();
   void set_op_mode(const hardware_interface::OperationMode& mode);
   void halt();
};
} // namespace ros_controllers
#endif // ROS_CONTROLLERS__DIFF_DRIVE_CONTROLLER_HPP_
