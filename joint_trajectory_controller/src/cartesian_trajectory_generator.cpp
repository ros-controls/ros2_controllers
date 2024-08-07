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

#include "controller_interface/helpers.hpp"
#include "eigen3/Eigen/Eigen"
#include "geometry_msgs/msg/quaternion.hpp"
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

void rpy_to_quaternion(
  std::array<double, 3> & orientation_angles, geometry_msgs::msg::Quaternion & quaternion_msg)
{
  // convert quaternion to euler angles
  tf2::Quaternion quaternion;
  quaternion.setRPY(orientation_angles[0], orientation_angles[1], orientation_angles[2]);
  quaternion_msg = tf2::toMsg(quaternion);
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
  auto qos_best_effort_history_depth_one = rclcpp::SystemDefaultsQoS();
  qos_best_effort_history_depth_one.keep_last(1);
  qos_best_effort_history_depth_one.best_effort();
  auto subscribers_reliable_qos = rclcpp::SystemDefaultsQoS();
  subscribers_reliable_qos.keep_all();
  subscribers_reliable_qos.reliable();

  // Reference Subscribers (reliable channel also for updates not to be missed)
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", qos_best_effort_history_depth_one,
    std::bind(&CartesianTrajectoryGenerator::reference_callback, this, std::placeholders::_1));
  ref_subscriber_reliable_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference_reliable", subscribers_reliable_qos,
    std::bind(&CartesianTrajectoryGenerator::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg);
  reference_world_.writeFromNonRT(msg);

  // Odometry feedback
  auto feedback_callback = [&](const std::shared_ptr<ControllerFeedbackMsg> feedback_msg) -> void
  { feedback_.writeFromNonRT(feedback_msg); };
  feedback_subscriber_ = get_node()->create_subscription<ControllerFeedbackMsg>(
    "~/feedback", qos_best_effort_history_depth_one, feedback_callback);
  // initialize feedback to null pointer since it is used to determine if we have valid data or not
  feedback_.writeFromNonRT(nullptr);

  // service QoS
  auto services_qos = rclcpp::SystemDefaultsQoS();  // message queue depth
  services_qos.keep_all();
  services_qos.reliable();
  services_qos.durability_volatile();

  cart_publisher_ = get_node()->create_publisher<CartControllerStateMsg>(
    "~/controller_state_cartesian", qos_best_effort_history_depth_one);
  cart_state_publisher_ = std::make_unique<CartStatePublisher>(cart_publisher_);

  cart_state_publisher_->lock();
  cart_state_publisher_->msg_.dof_names = params_.joints;
  cart_state_publisher_->msg_.reference_world.transforms.resize(1);
  cart_state_publisher_->msg_.reference_world.velocities.resize(1);
  cart_state_publisher_->msg_.reference_world.accelerations.resize(1);
  cart_state_publisher_->msg_.reference_local.transforms.resize(1);
  cart_state_publisher_->msg_.reference_local.velocities.resize(1);
  cart_state_publisher_->msg_.reference_local.accelerations.resize(1);
  cart_state_publisher_->msg_.feedback.transforms.resize(1);
  cart_state_publisher_->msg_.feedback.velocities.resize(1);
  cart_state_publisher_->msg_.feedback.accelerations.resize(1);
  cart_state_publisher_->msg_.feedback_local.transforms.resize(1);
  cart_state_publisher_->msg_.feedback_local.velocities.resize(1);
  cart_state_publisher_->msg_.feedback_local.accelerations.resize(1);
  cart_state_publisher_->msg_.error.transforms.resize(1);
  cart_state_publisher_->msg_.error.velocities.resize(1);
  cart_state_publisher_->msg_.error.accelerations.resize(1);
  cart_state_publisher_->msg_.output_world.transforms.resize(1);
  cart_state_publisher_->msg_.output_world.velocities.resize(1);
  cart_state_publisher_->msg_.output_world.accelerations.resize(1);
  cart_state_publisher_->msg_.output_local.transforms.resize(1);
  cart_state_publisher_->msg_.output_local.velocities.resize(1);
  cart_state_publisher_->msg_.output_local.accelerations.resize(1);
  cart_state_publisher_->unlock();

  return CallbackReturn::SUCCESS;
}

void CartesianTrajectoryGenerator::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  // store input ref for later use
  reference_world_.writeFromNonRT(msg);

  //TODO(henrygerardmoore): replace the below with multiple JTCs
  

  // assume for now that we are working with trajectories with one point - we don't know exactly
  // where we are in the trajectory before sampling - nevertheless this should work for the use case
  auto new_traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  new_traj_msg->joint_names = params_.joints;
  new_traj_msg->points.resize(1);
  new_traj_msg->points[0].positions.resize(
    params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  new_traj_msg->points[0].velocities.resize(
    params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  if (msg->time_from_start.nanosec == 0)
  {
    new_traj_msg->points[0].time_from_start = rclcpp::Duration::from_seconds(0.01);
  }
  else
  {
    new_traj_msg->points[0].time_from_start = rclcpp::Duration::from_nanoseconds(
      static_cast<rcl_duration_value_t>(msg->time_from_start.nanosec));
  }

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

  traj_external_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>();
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  subscriber_is_active_ = true;

  // Initialize current state storage if hardware state has tracking offset
  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(state, dof_);
  if (!read_state_from_hardware(state)) return CallbackReturn::ERROR;
  state_current_ = state;
  state_desired_ = state;
  last_commanded_state_ = state;
  // Handle restart of controller by reading from commands if
  // those are not nan
  if (read_state_from_command_interfaces(state))
  {
    state_current_ = state;
    state_desired_ = state;
    last_commanded_state_ = state;
  }

  return CallbackReturn::SUCCESS;
}

bool CartesianTrajectoryGenerator::read_state_from_hardware(JointTrajectoryPoint & state)
{
  std::array<double, 3> orientation_angles;
  const auto measured_state = *(feedback_.readFromRT());
  if (!measured_state) return false;

  tf2::Quaternion measured_q;

  if (
    std::isnan(measured_state->pose.pose.orientation.w) ||
    std::isnan(measured_state->pose.pose.orientation.x) ||
    std::isnan(measured_state->pose.pose.orientation.y) ||
    std::isnan(measured_state->pose.pose.orientation.z))
  {
    // if any of the orientation is NaN, revert to previous orientation
    measured_q.setRPY(state.positions[3], state.positions[4], state.positions[5]);
  }
  else
  {
    tf2::fromMsg(measured_state->pose.pose.orientation, measured_q);
  }
  tf2::Matrix3x3 m(measured_q);
  m.getRPY(orientation_angles[0], orientation_angles[1], orientation_angles[2]);

  // Assign values from the hardware
  // Position states always exist
  // if any measured position is NaN, keep previous value
  state.positions[0] = std::isnan(measured_state->pose.pose.position.x)
                         ? state.positions[0]
                         : measured_state->pose.pose.position.x;
  state.positions[1] = std::isnan(measured_state->pose.pose.position.y)
                         ? state.positions[1]
                         : measured_state->pose.pose.position.y;
  state.positions[2] = std::isnan(measured_state->pose.pose.position.z)
                         ? state.positions[2]
                         : measured_state->pose.pose.position.z;
  state.positions[3] = orientation_angles[0];
  state.positions[4] = orientation_angles[1];
  state.positions[5] = orientation_angles[2];

  // Convert measured twist which is in body frame to world frame since CTG/JTC expects state
  // in world frame

  Eigen::Quaterniond q_body_in_world(
    measured_q.w(), measured_q.x(), measured_q.y(), measured_q.z());

  // if any measured linear velocity is NaN, set to zero
  Eigen::Vector3d linear_vel_body(
    std::isnan(measured_state->twist.twist.linear.x) ? 0.0 : measured_state->twist.twist.linear.x,
    std::isnan(measured_state->twist.twist.linear.y) ? 0.0 : measured_state->twist.twist.linear.y,
    std::isnan(measured_state->twist.twist.linear.z) ? 0.0 : measured_state->twist.twist.linear.z);
  auto linear_vel_world = q_body_in_world * linear_vel_body;

  // if any measured angular velocity is NaN, set to zero
  Eigen::Vector3d angular_vel_body(
    std::isnan(measured_state->twist.twist.angular.x) ? 0.0 : measured_state->twist.twist.angular.x,
    std::isnan(measured_state->twist.twist.angular.y) ? 0.0 : measured_state->twist.twist.angular.y,
    std::isnan(measured_state->twist.twist.angular.z) ? 0.0
                                                      : measured_state->twist.twist.angular.z);
  auto angular_vel_world = q_body_in_world * angular_vel_body;

  state.velocities[0] = linear_vel_world[0];
  state.velocities[1] = linear_vel_world[1];
  state.velocities[2] = linear_vel_world[2];
  state.velocities[3] = angular_vel_world[0];
  state.velocities[4] = angular_vel_world[1];
  state.velocities[5] = angular_vel_world[2];

  state.accelerations.clear();
  return true;
}

void CartesianTrajectoryGenerator::publish_state(
  const rclcpp::Time & time, const JointTrajectoryPoint & desired_state,
  const JointTrajectoryPoint & current_state, const JointTrajectoryPoint & state_error,
  const JointTrajectoryPoint & splines_output, const JointTrajectoryPoint & ruckig_input_target,
  const JointTrajectoryPoint & ruckig_input)
{
  joint_trajectory_controller::JointTrajectoryController::publish_state(
    time, desired_state, current_state, state_error, splines_output, ruckig_input_target,
    ruckig_input);

  if (cart_state_publisher_->trylock())
  {
    cart_state_publisher_->msg_.header.stamp = time;
    cart_state_publisher_->msg_.reference_world = *(*reference_world_.readFromRT());

    auto set_multi_dof_point =
      [&](
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint & multi_dof_point,
        const JointTrajectoryPoint & traj_point)
    {
      if (traj_point.positions.size() == 6)
      {
        multi_dof_point.transforms[0].translation.x = traj_point.positions[0];
        multi_dof_point.transforms[0].translation.y = traj_point.positions[1];
        multi_dof_point.transforms[0].translation.z = traj_point.positions[2];

        std::array<double, 3> orientation_angles = {
          traj_point.positions[3], traj_point.positions[4], traj_point.positions[5]};
        geometry_msgs::msg::Quaternion quaternion;
        rpy_to_quaternion(orientation_angles, quaternion);
        multi_dof_point.transforms[0].rotation = quaternion;
      }
      if (traj_point.velocities.size() == 6)
      {
        multi_dof_point.velocities[0].linear.x = traj_point.velocities[0];
        multi_dof_point.velocities[0].linear.y = traj_point.velocities[1];
        multi_dof_point.velocities[0].linear.z = traj_point.velocities[2];
        multi_dof_point.velocities[0].angular.x = traj_point.velocities[3];
        multi_dof_point.velocities[0].angular.y = traj_point.velocities[4];
        multi_dof_point.velocities[0].angular.z = traj_point.velocities[5];
      }
      if (traj_point.accelerations.size() == 6)
      {
        multi_dof_point.accelerations[0].linear.x = traj_point.accelerations[0];
        multi_dof_point.accelerations[0].linear.y = traj_point.accelerations[1];
        multi_dof_point.accelerations[0].linear.z = traj_point.accelerations[2];
        multi_dof_point.accelerations[0].angular.x = traj_point.accelerations[3];
        multi_dof_point.accelerations[0].angular.y = traj_point.accelerations[4];
        multi_dof_point.accelerations[0].angular.z = traj_point.accelerations[5];
      }
    };

    set_multi_dof_point(cart_state_publisher_->msg_.feedback_local, current_state);
    set_multi_dof_point(cart_state_publisher_->msg_.error, state_error);
    set_multi_dof_point(cart_state_publisher_->msg_.output_world, desired_state);

    const auto measured_state = *(feedback_.readFromRT());
    cart_state_publisher_->msg_.feedback.transforms[0].translation.x =
      measured_state->pose.pose.position.x;
    cart_state_publisher_->msg_.feedback.transforms[0].translation.y =
      measured_state->pose.pose.position.y;
    cart_state_publisher_->msg_.feedback.transforms[0].translation.z =
      measured_state->pose.pose.position.z;
    cart_state_publisher_->msg_.feedback.transforms[0].rotation =
      measured_state->pose.pose.orientation;
    cart_state_publisher_->msg_.feedback.velocities[0] = measured_state->twist.twist;

    cart_state_publisher_->unlockAndPublish();
  }
}

}  // namespace cartesian_trajectory_generator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_trajectory_generator::CartesianTrajectoryGenerator,
  controller_interface::ControllerInterface)
