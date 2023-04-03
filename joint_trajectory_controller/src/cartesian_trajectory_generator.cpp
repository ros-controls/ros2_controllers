// Copyright (c) 2023 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "joint_trajectory_controller/cartesian_trajectory_generator.hpp"

#include "tf2/transform_datatypes.h"

#include "controller_interface/helpers.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{  // utility

void reset_twist_msg(geometry_msgs::msg::Twist & msg)
{
  msg.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.angular.z = std::numeric_limits<double>::quiet_NaN();
}

using ControllerReferenceMsg =
  cartesian_trajectory_generator::CartesianTrajectoryGenerator::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(ControllerReferenceMsg & msg)
{
  msg.transforms.resize(1);
  msg.transforms[0].translation.x = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].translation.y = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].translation.z = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].rotation.x = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].rotation.y = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].rotation.z = std::numeric_limits<double>::quiet_NaN();
  msg.transforms[0].rotation.w = std::numeric_limits<double>::quiet_NaN();

  msg.velocities.resize(1);
  reset_twist_msg(msg.velocities[0]);

  msg.accelerations.resize(1);
  reset_twist_msg(msg.accelerations[0]);
}

void reset_controller_reference_msg(const std::shared_ptr<ControllerReferenceMsg> & msg)
{
  reset_controller_reference_msg(*msg);
}

using ControllerFeedbackMsg =
  cartesian_trajectory_generator::CartesianTrajectoryGenerator::ControllerFeedbackMsg;

// called from RT control loop
void reset_controller_feedback_msg(ControllerFeedbackMsg & msg)
{
  msg.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.orientation.y = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
  msg.pose.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  msg.pose.covariance.fill(std::numeric_limits<double>::quiet_NaN());

  reset_twist_msg(msg.twist.twist);
  msg.twist.covariance.fill(std::numeric_limits<double>::quiet_NaN());
}
void reset_controller_feedback_msg(const std::shared_ptr<ControllerFeedbackMsg> & msg)
{
  reset_controller_feedback_msg(*msg);
}
}  // namespace

namespace cartesian_trajectory_generator
{
CartesianTrajectoryGenerator::CartesianTrajectoryGenerator()
: joint_trajectory_controller::JointTrajectoryController()
{
}

controller_interface::InterfaceConfiguration
CartesianTrajectoryGenerator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

controller_interface::CallbackReturn CartesianTrajectoryGenerator::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_configure(previous_state);
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // store joint limits for later
  configured_joint_limits_ = joint_limits_;

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();
  auto subscribers_reliable_qos = rclcpp::SystemDefaultsQoS();
  subscribers_reliable_qos.keep_all();
  subscribers_reliable_qos.reliable();

  // Reference Subscribers (reliable channel also for updates not to be missed)
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&CartesianTrajectoryGenerator::reference_callback, this, std::placeholders::_1));
  ref_subscriber_reliable_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference_reliable", subscribers_qos,
    std::bind(&CartesianTrajectoryGenerator::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg);
  input_ref_.writeFromNonRT(msg);

  // Odometry feedback
  auto feedback_callback = [&](const std::shared_ptr<ControllerFeedbackMsg> msg) -> void
  { feedback_.writeFromNonRT(msg); };
  feedback_subscriber_ = get_node()->create_subscription<ControllerFeedbackMsg>(
    "~/feedback", subscribers_qos, feedback_callback);
  std::shared_ptr<ControllerFeedbackMsg> feedback_msg = std::make_shared<ControllerFeedbackMsg>();
  reset_controller_feedback_msg(feedback_msg);
  feedback_.writeFromNonRT(feedback_msg);

  // service QoS
  auto services_qos = rclcpp::SystemDefaultsQoS();  // message queue depth
  services_qos.keep_all();
  services_qos.reliable();
  services_qos.durability_volatile();

  set_joint_limits_service_ = get_node()->create_service<SetLimitsModeSrvType>(
    "~/set_joint_limits",
    std::bind(
      &CartesianTrajectoryGenerator::set_joint_limits_service_callback, this, std::placeholders::_1,
      std::placeholders::_2),
    services_qos);

  return CallbackReturn::SUCCESS;
}

void CartesianTrajectoryGenerator::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  // store input ref for later use
  input_ref_.writeFromNonRT(msg);

  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(state, dof_);
  read_state_from_hardware(state);

  // assume for now that we are working with trajectories with one point - we don't know exactly
  // where we are in the trajectory before sampling - nevertheless this should work for the use case
  auto new_traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  new_traj_msg->joint_names = params_.joints;
  new_traj_msg->points.resize(1);
  new_traj_msg->points[0].positions.resize(
    params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  new_traj_msg->points[0].velocities.resize(
    params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  new_traj_msg->points[0].time_from_start = rclcpp::Duration::from_seconds(0.01);

  // just pass input into trajectory message
  auto assign_value_from_input = [&](
                                   const double pos_from_msg, const double vel_from_msg,
                                   const std::string & joint_name, const size_t index)
  {
    new_traj_msg->points[0].positions[index] = pos_from_msg;
    new_traj_msg->points[0].velocities[index] = vel_from_msg;
    if (std::isnan(pos_from_msg) && std::isnan(vel_from_msg))
    {
      RCLCPP_DEBUG(
        get_node()->get_logger(), "Input position and velocity for %s is NaN", joint_name.c_str());
    }
  };

  assign_value_from_input(
    msg->transforms[0].translation.x, msg->velocities[0].linear.x, params_.joints[0], 0);
  assign_value_from_input(
    msg->transforms[0].translation.y, msg->velocities[0].linear.y, params_.joints[1], 1);
  assign_value_from_input(
    msg->transforms[0].translation.z, msg->velocities[0].linear.z, params_.joints[2], 2);
  assign_value_from_input(
    msg->transforms[0].rotation.x, msg->velocities[0].angular.x, params_.joints[3], 3);
  assign_value_from_input(
    msg->transforms[0].rotation.y, msg->velocities[0].angular.y, params_.joints[4], 4);
  assign_value_from_input(
    msg->transforms[0].rotation.z, msg->velocities[0].angular.z, params_.joints[5], 5);

  add_new_trajectory_msg(new_traj_msg);
}

void CartesianTrajectoryGenerator::set_joint_limits_service_callback(
  const std::shared_ptr<SetLimitsModeSrvType::Request> request,
  std::shared_ptr<SetLimitsModeSrvType::Response> response)
{
  response->ok = true;
  if (request->names.size() != request->limits.size())
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Fields name and limits have size %zu and %zu. Their size should be equal. Ignoring "
      "service call!",
      request->names.size(), request->limits.size());
    response->ok = false;
  }

  // start with current limits
  auto new_joint_limits = joint_limits_;

  // lambda for setting new limit
  auto update_limit_from_request = [](
                                     double & new_limit_value, bool & new_limit_has_limits,
                                     const double request_limit_value,
                                     const double configured_limit_value)
  {
    // request is to reset to configured limit
    if (std::isnan(request_limit_value))
    {
      new_limit_value = configured_limit_value;
    }
    // new limit is requested
    else
    {
      new_limit_value = request_limit_value;
    }
    new_limit_has_limits = !std::isnan(new_limit_value);
  };

  // lambda for setting new position limits
  auto update_pos_limits_from_request =
    [](
      double & new_min_limit_value, double & new_max_limit_value, bool & new_limit_has_limits,
      const double request_min_limit_value, const double request_max_limit_value,
      const double configured_min_limit_value, const double configured_max_limit_value)
  {
    // request is to reset to configured min position limit
    if (std::isnan(request_min_limit_value))
    {
      new_min_limit_value = configured_min_limit_value;
    }
    // new min position limit is requested
    else
    {
      new_min_limit_value = request_min_limit_value;
    }

    // request is to reset to configured max position limit
    if (std::isnan(request_max_limit_value))
    {
      new_max_limit_value = configured_max_limit_value;
    }
    // new max position limit is requested
    else
    {
      new_max_limit_value = request_max_limit_value;
    }

    // has limits only if one of the min or max position limits is set
    new_limit_has_limits = !std::isnan(new_min_limit_value) || !std::isnan(new_max_limit_value);
  };

  for (size_t i = 0; i < request->names.size(); ++i)
  {
    auto it =
      std::find(command_joint_names_.begin(), command_joint_names_.end(), request->names[i]);
    if (it != command_joint_names_.end())
    {
      auto cmd_itf_index = std::distance(command_joint_names_.begin(), it);

      update_pos_limits_from_request(
        new_joint_limits[cmd_itf_index].min_position, new_joint_limits[cmd_itf_index].max_position,
        new_joint_limits[cmd_itf_index].has_position_limits, request->limits[i].min_position,
        request->limits[i].max_position, configured_joint_limits_[cmd_itf_index].min_position,
        configured_joint_limits_[cmd_itf_index].max_position);
      update_limit_from_request(
        new_joint_limits[cmd_itf_index].max_velocity,
        new_joint_limits[cmd_itf_index].has_velocity_limits, request->limits[i].max_velocity,
        configured_joint_limits_[cmd_itf_index].max_velocity);
      update_limit_from_request(
        new_joint_limits[cmd_itf_index].max_acceleration,
        new_joint_limits[cmd_itf_index].has_acceleration_limits,
        request->limits[i].max_acceleration,
        configured_joint_limits_[cmd_itf_index].max_acceleration);
      update_limit_from_request(
        new_joint_limits[cmd_itf_index].max_jerk, new_joint_limits[cmd_itf_index].has_jerk_limits,
        request->limits[i].max_jerk, configured_joint_limits_[cmd_itf_index].max_jerk);
      update_limit_from_request(
        new_joint_limits[cmd_itf_index].max_effort,
        new_joint_limits[cmd_itf_index].has_effort_limits, request->limits[i].max_effort,
        configured_joint_limits_[cmd_itf_index].max_effort);

      RCLCPP_INFO(
        get_node()->get_logger(), "New limits for joint %zu (%s) are: \n%s", cmd_itf_index,
        command_joint_names_[cmd_itf_index].c_str(),
        new_joint_limits[cmd_itf_index].to_string().c_str());
    }
    else
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Name '%s' is not command interface. Ignoring this entry.",
        request->names[i].c_str());
      response->ok = false;
    }
  }

  // TODO(destogl): use buffers to sync comm between threads
  joint_limits_ = new_joint_limits;
}

controller_interface::CallbackReturn CartesianTrajectoryGenerator::on_activate(
  const rclcpp_lifecycle::State &)
{
  // order all joints in the storage
  for (const auto & interface : params_.command_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_joint_names_, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", dof_,
        interface.c_str(), joint_command_interface_[index].size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  //   for (const auto & interface : state_interface_types_)
  //   {
  //     auto it =
  //     std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
  //     auto index = std::distance(allowed_interface_types_.begin(), it);
  //     if (!controller_interface::get_ordered_interfaces(
  //       state_interfaces_, joint_names_, interface, joint_state_interface_[index]))
  //     {
  //       RCLCPP_ERROR(
  //         get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", dof_,
  //                    interface.c_str(), joint_state_interface_[index].size());
  //       return controller_interface::CallbackReturn::ERROR;
  //     }
  //   }

  // Store 'home' pose
  traj_msg_home_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg_home_ptr_->header.stamp.sec = 0;
  traj_msg_home_ptr_->header.stamp.nanosec = 0;
  traj_msg_home_ptr_->points.resize(1);
  traj_msg_home_ptr_->points[0].time_from_start.sec = 0;
  traj_msg_home_ptr_->points[0].time_from_start.nanosec = 50000000;
  traj_msg_home_ptr_->points[0].positions.resize(joint_state_interface_[0].size());
  for (size_t index = 0; index < joint_state_interface_[0].size(); ++index)
  {
    traj_msg_home_ptr_->points[0].positions[index] =
      joint_state_interface_[0][index].get().get_value();
  }

  traj_external_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  traj_home_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  subscriber_is_active_ = true;
  traj_point_active_ptr_ = &traj_external_point_ptr_;

  // Initialize current state storage if hardware state has tracking offset
  read_state_from_hardware(state_current_);
  read_state_from_hardware(state_desired_);
  read_state_from_hardware(last_commanded_state_);
  // Handle restart of controller by reading from commands if
  // those are not nan
  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(state, dof_);
  if (read_state_from_command_interfaces(state))
  {
    state_current_ = state;
    state_desired_ = state;
    last_commanded_state_ = state;
  }

  return CallbackReturn::SUCCESS;
}

void CartesianTrajectoryGenerator::read_state_from_hardware(JointTrajectoryPoint & state)
{
  std::array<double, 3> orientation_angles;
  const auto measured_state = *(feedback_.readFromRT());
  tf2::Quaternion measured_q;
  tf2::fromMsg(measured_state->pose.pose.orientation, measured_q);
  tf2::Matrix3x3 m(measured_q);
  m.getRPY(orientation_angles[0], orientation_angles[1], orientation_angles[2]);

  // Assign values from the hardware
  // Position states always exist
  state.positions[0] = measured_state->pose.pose.position.x;
  state.positions[1] = measured_state->pose.pose.position.y;
  state.positions[2] = measured_state->pose.pose.position.z;
  state.positions[3] = orientation_angles[0];
  state.positions[4] = orientation_angles[1];
  state.positions[5] = orientation_angles[2];

  state.velocities[0] = measured_state->twist.twist.linear.x;
  state.velocities[1] = measured_state->twist.twist.linear.y;
  state.velocities[2] = measured_state->twist.twist.linear.z;
  state.velocities[3] = measured_state->twist.twist.angular.x;
  state.velocities[4] = measured_state->twist.twist.angular.y;
  state.velocities[5] = measured_state->twist.twist.angular.z;

  state.accelerations.clear();
}
}  // namespace cartesian_trajectory_generator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_trajectory_generator::CartesianTrajectoryGenerator,
  controller_interface::ControllerInterface)
