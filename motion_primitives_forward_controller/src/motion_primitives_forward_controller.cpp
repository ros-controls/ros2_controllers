// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include "motion_primitives_forward_controller/motion_primitives_forward_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "motion_primitives_forward_controller/motion_type.hpp"
#include "motion_primitives_forward_controller/execution_state.hpp"
#include "motion_primitives_forward_controller/ready_for_new_primitive.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = motion_primitives_forward_controller::MotionPrimitivesForwardController::ControllerReferenceMsg;

// reset the controller reference message to NaN
void reset_controller_reference_msg(std::shared_ptr<ControllerReferenceMsg> & msg)
{
  msg->type = 0;
  msg->blend_radius = std::numeric_limits<double>::quiet_NaN();
  
  for (auto & arg : msg->additional_arguments) {
    arg.argument_name = "";
    arg.argument_value = std::numeric_limits<double>::quiet_NaN();
  }

  for (auto & pose : msg->poses) {
      pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
      pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
      pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();

      pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
      pose.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
      pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
      pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace


namespace motion_primitives_forward_controller
{
MotionPrimitivesForwardController::MotionPrimitivesForwardController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "Initializing Motion Primitives Forward Controller");

  try
  {
    param_listener_ = std::make_shared<motion_primitives_forward_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Configuring Motion Primitives Forward Controller");

  params_ = param_listener_->get_params();

  // Check if the name is not empty
  if (params_.name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error: A name must be provided!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Check if there are exactly 25 command interfaces
  if (params_.command_interfaces.size() !=25) { // motion_type + 6 joints + 2*7 positions + blend_radius + velocity + acceleration + move_time
    RCLCPP_ERROR(get_node()->get_logger(), "Error: Exactly 25 command interfaces must be provided!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Check if there are exactly 2 state interfaces
  if (params_.state_interfaces.size() != 2) { // execution_status + ready_for_new_primitive
    RCLCPP_ERROR(get_node()->get_logger(), "Error: Exactly one state interface must be provided!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&MotionPrimitivesForwardController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg);

  queue_size_ = params_.queue_size;
  if (queue_size_ == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error: Queue size must be greater than 0!");
    return controller_interface::CallbackReturn::ERROR;
  }

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

// Function gets called when a new message is received
void MotionPrimitivesForwardController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  // Check if the type is one of the allowed motion types
  switch (msg->type)
  {
    case MotionType::STOP_MOTION:{
      RCLCPP_INFO(get_node()->get_logger(), "Received motion type: STOP_MOTION");
      reset_command_interfaces(); // Reset all command interfaces to NaN
      std::lock_guard<std::mutex> guard(command_mutex_);
      (void)command_interfaces_[0].set_value(static_cast<double>(msg->type)); // send stop command to the driver
      while (!msg_queue_.empty()) {   // clear the queue
        msg_queue_.pop();   
      }
      return;
    }

    case MotionType::LINEAR_JOINT:
      RCLCPP_INFO(get_node()->get_logger(), "Received motion type: LINEAR_JOINT (PTP)");
      // Check if joint positions are provided
      if (msg->joint_positions.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Received LINEAR_JOINT motion type without joint positions");
        return;
      } 
      break;

    case MotionType::LINEAR_CARTESIAN:
      RCLCPP_INFO(get_node()->get_logger(), "Received motion type: LINEAR_CARTESIAN (LIN)");
      // Check if poses are provided
      if (msg->poses.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Received LINEAR_CARTESIAN motion type without poses");
        return;
      }
      break;

    case MotionType::CIRCULAR_CARTESIAN:
      RCLCPP_INFO(get_node()->get_logger(), "Received motion type: CIRCULAR_CARTESIAN (CIRC)");
      // Check if poses are provided
      if (msg->poses.size() != 2) {
        RCLCPP_ERROR(get_node()->get_logger(), "Received CIRCULAR_CARTESIAN motion type without two poses");
        return;
      }
      break;

    case MotionType::MOTION_SEQUENCE_START:
      RCLCPP_INFO(get_node()->get_logger(), "Received motion type: MOTION_SEQUENCE_START");
      break;

    case MotionType::MOTION_SEQUENCE_END:
      RCLCPP_INFO(get_node()->get_logger(), "Received motion type: MOTION_SEQUENCE_END");
      break;

    // Additional motion types can be added here
    default:
      RCLCPP_ERROR(get_node()->get_logger(), "Received unknown motion type: %u", msg->type);
      return;
  }

  // Check if the queue size is exceeded
  if (msg_queue_.size() >= queue_size_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Queue size exceeded. Can't add new motion primitive.");
    return;
  }

  msg_queue_.push(msg);
}


controller_interface::InterfaceConfiguration MotionPrimitivesForwardController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Reserve memory for the command interfaces
  command_interfaces_config.names.reserve(params_.command_interfaces.size());

  // Iterate over all command interfaces from the config yaml file
  for (const auto & interface_name : params_.command_interfaces)
  {
      // Combine the joint with the interface name and add it to the list
      command_interfaces_config.names.push_back(params_.name + "/" + interface_name);
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MotionPrimitivesForwardController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Reserve memory for the state interfaces
  state_interfaces_config.names.reserve(params_.state_interfaces.size());

  // Iterate over all state interfaces from the config yaml file
  for (const auto & interface_name : params_.state_interfaces)
  {
      // Combine the joint with the interface name and add it to the list
      state_interfaces_config.names.push_back(params_.name + "/" + interface_name);
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_interfaces(); // Reset all command interfaces to NaN
  RCLCPP_INFO(get_node()->get_logger(), "Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotionPrimitivesForwardController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_interfaces(); // Reset all command interfaces to NaN
  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::return_type MotionPrimitivesForwardController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // read the status from the state interface
  // TODO(mathias31415) Is check needed if the value is .0?
  uint8_t execution_status = static_cast<int8_t>(std::round(state_interfaces_[0].get_value()));

  switch (execution_status) {
    case ExecutionState::IDLE:
      // RCLCPP_INFO(get_node()->get_logger(), "Execution state: IDLE");
      print_error_once_ = true;
      break;
    case ExecutionState::EXECUTING:
      // RCLCPP_INFO(get_node()->get_logger(), "Execution state: EXECUTING");
      print_error_once_ = true;
      break;

    case ExecutionState::SUCCESS:
      // RCLCPP_INFO(get_node()->get_logger(), "Execution state: SUCCESS");
      print_error_once_ = true;
      break;

    case ExecutionState::ERROR:
      if (print_error_once_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Execution state: ERROR");
        print_error_once_ = false;
      }
      break;
  
    default:
      RCLCPP_ERROR(get_node()->get_logger(), "Error: Unknown execution status: %d", execution_status);
      return controller_interface::return_type::ERROR;
  }

  // publish the execution_status
  state_publisher_->lock();
  state_publisher_->msg_.data = execution_status;
  state_publisher_->unlockAndPublish();

  // TODO(mathias31415) Is check needed if the value is .0?
  uint8_t ready_for_new_primitive = static_cast<int8_t>(std::round(state_interfaces_[1].get_value()));

  // sending new command?
  if(!msg_queue_.empty()) // check if new command is available
  { 
    switch (ready_for_new_primitive) {
      case ReadyForNewPrimitive::NOT_READY:{ // hw-interface is not ready for a new command
        return controller_interface::return_type::OK;
      }
      case ReadyForNewPrimitive::READY:{ // hw-interface is ready for a new command
        if(!set_command_interfaces()){ // Set the command interfaces with next motion primitive
          RCLCPP_ERROR(get_node()->get_logger(), "Error: set_command_interfaces() failed");
          return controller_interface::return_type::ERROR;
        }
        return controller_interface::return_type::OK;
      }
      default:
        RCLCPP_ERROR(get_node()->get_logger(), "Error: Unknown state for ready_for_new_primitive: %d", ready_for_new_primitive);
        return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

// Reset Command-Interfaces to nan
void MotionPrimitivesForwardController::reset_command_interfaces()
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    if (!command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN())) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset command interface %ld", i);
    }
  }
}

// Set command interfaces from the message, gets called in the update function
bool MotionPrimitivesForwardController::set_command_interfaces()
{
  std::lock_guard<std::mutex> guard(command_mutex_);
  // Get the oldest message from the queue
  std::shared_ptr<ControllerReferenceMsg> current_ref = msg_queue_.front();
  msg_queue_.pop();

  // Check if the message is valid
  if (!current_ref){
    RCLCPP_WARN(get_node()->get_logger(), "No valid reference message received");
    return false;
  }

  // Set the motion_type
  (void)command_interfaces_[0].set_value(static_cast<double>(current_ref->type)); 

  // Process joint positions if available
  if (!current_ref->joint_positions.empty())
  {
    for (size_t i = 0; i < current_ref->joint_positions.size(); ++i)
    {
      (void)command_interfaces_[i + 1].set_value(current_ref->joint_positions[i]); // q1 to q6
    }
  }
  
  // Process Cartesian poses if available
  if (!current_ref->poses.empty())
  {
    const auto & goal_pose = current_ref->poses[0].pose; // goal pose
    (void)command_interfaces_[7].set_value(goal_pose.position.x);  // pos_x
    (void)command_interfaces_[8].set_value(goal_pose.position.y);  // pos_y
    (void)command_interfaces_[9].set_value(goal_pose.position.z);  // pos_z
    (void)command_interfaces_[10].set_value(goal_pose.orientation.x); // pos_qx
    (void)command_interfaces_[11].set_value(goal_pose.orientation.y); // pos_qy
    (void)command_interfaces_[12].set_value(goal_pose.orientation.z); // pos_qz
    (void)command_interfaces_[13].set_value(goal_pose.orientation.w); // pos_qw

    // Process via poses if available (only for circular motion)
    if (current_ref->type == MotionType::CIRCULAR_CARTESIAN && current_ref->poses.size() == 2)
    {
      const auto & via_pose = current_ref->poses[1].pose; // via pose
      (void)command_interfaces_[14].set_value(via_pose.position.x);  // pos_via_x
      (void)command_interfaces_[15].set_value(via_pose.position.y);  // pos_via_y
      (void)command_interfaces_[16].set_value(via_pose.position.z);  // pos_via_z
      (void)command_interfaces_[17].set_value(via_pose.orientation.x); // pos_via_qx
      (void)command_interfaces_[18].set_value(via_pose.orientation.y); // pos_via_qy
      (void)command_interfaces_[19].set_value(via_pose.orientation.z); // pos_via_qz
      (void)command_interfaces_[20].set_value(via_pose.orientation.w); // pos_via_qw
    }
  }

  // Set blend_radius
  (void)command_interfaces_[21].set_value(current_ref->blend_radius); // blend_radius

  // Read additional arguments
  for (const auto &arg : current_ref->additional_arguments)
  {
      if (arg.argument_name == "velocity")
      {
        (void)command_interfaces_[22].set_value(arg.argument_value);
      }
      else if (arg.argument_name == "acceleration")
      {
        (void)command_interfaces_[23].set_value(arg.argument_value);
      }
      else if (arg.argument_name == "move_time")
      {
        (void)command_interfaces_[24].set_value(arg.argument_value);
      }
      else
      {
        RCLCPP_WARN(get_node()->get_logger(), "Unknown additional argument: %s", arg.argument_name.c_str());
      }
  }
  return true;
}


}  // namespace motion_primitives_forward_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motion_primitives_forward_controller::MotionPrimitivesForwardController, controller_interface::ControllerInterface)
