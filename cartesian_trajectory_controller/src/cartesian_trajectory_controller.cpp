// Copyright (c) 2026 ros2_control Development Team
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

#include "cartesian_trajectory_controller/cartesian_trajectory_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "cartesian_trajectory_controller/cartesian_trajectory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"

namespace cartesian_trajectory_controller
{

controller_interface::CallbackReturn CartesianTrajectoryController::on_init()
{
  const auto ret = JointTrajectoryController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  try
  {
    ctc_param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception initializing parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianTrajectoryController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = JointTrajectoryController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  ctc_params_ = ctc_param_listener_->get_params();

  try
  {
    kinematics_loader_ =
      std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
        ctc_params_.kinematics.plugin_package, "kinematics_interface::KinematicsInterface");
    kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
      kinematics_loader_->createUnmanagedInstance(ctc_params_.kinematics.plugin_name));
  }
  catch (const pluginlib::PluginlibException & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to load kinematics plugin '%s': %s",
      ctc_params_.kinematics.plugin_name.c_str(), e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!kinematics_->initialize(
        get_robot_description(), get_node()->get_node_parameters_interface(), "kinematics"))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize the kinematics plugin.");
    return controller_interface::CallbackReturn::ERROR;
  }

  auto qos = rclcpp::SystemDefaultsQoS();
  qos.keep_last(1);
  ref_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    "~/cartesian_reference", qos,
    std::bind(&CartesianTrajectoryController::reference_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianTrajectoryController::reference_callback(
  std::shared_ptr<trajectory_msgs::msg::MultiDOFJointTrajectory> msg)
{
  if (get_node()->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return;
  }

  auto joint_traj = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  if (!build_joint_trajectory(*msg, *joint_traj) || !validate_trajectory_msg(*joint_traj))
  {
    return;
  }

  add_new_trajectory_msg(joint_traj);
  rt_is_holding_ = false;
}

bool CartesianTrajectoryController::build_joint_trajectory(
  const trajectory_msgs::msg::MultiDOFJointTrajectory & msg,
  trajectory_msgs::msg::JointTrajectory & joint_traj)
{
  if (msg.points.empty() || state_current_.positions.size() != dof_)
  {
    return false;
  }
  if (!msg.header.frame_id.empty() && msg.header.frame_id != ctc_params_.kinematics.base)
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "Ignoring pose target: frame_id '%s' is not the base frame '%s'.",
      msg.header.frame_id.c_str(), ctc_params_.kinematics.base.c_str());
    return false;
  }

  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(state_current_.positions.data(), dof_);
  Eigen::Isometry3d current_pose;
  if (!kinematics_->calculate_link_transform(q, ctc_params_.kinematics.tip, current_pose))
  {
    return false;
  }

  std::vector<double> times;
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Quaterniond> orientations;
  if (!build_cartesian_waypoints(msg, current_pose, times, positions, orientations))
  {
    return false;
  }

  const CartesianTrajectory path(times, positions, orientations);
  return solve_ik_along_path(path, q, joint_traj);
}

bool CartesianTrajectoryController::build_cartesian_waypoints(
  const trajectory_msgs::msg::MultiDOFJointTrajectory & msg, const Eigen::Isometry3d & current_pose,
  std::vector<double> & times, std::vector<Eigen::Vector3d> & positions,
  std::vector<Eigen::Quaterniond> & orientations) const
{
  // Waypoint 0 is the current end-effector pose so the motion starts from where the robot is.
  times = {0.0};
  positions = {current_pose.translation()};
  orientations = {Eigen::Quaterniond(current_pose.rotation())};

  bool has_timing = false;
  for (const auto & point : msg.points)
  {
    const auto & t = point.time_from_start;
    has_timing = has_timing || (t.sec != 0 || t.nanosec != 0u);
  }

  for (const auto & point : msg.points)
  {
    if (point.transforms.empty())
    {
      return false;
    }
    const auto & tf = point.transforms[0];
    const Eigen::Vector3d position(tf.translation.x, tf.translation.y, tf.translation.z);
    Eigen::Quaterniond orientation(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z);
    orientation.normalize();  // shortest-arc sign alignment

    double t;
    if (has_timing)
    {
      t = rclcpp::Duration(point.time_from_start).seconds();
    }
    else  // synthesize timing from the commanded Cartesian and angular speeds
    {
      t = times.back() + min_segment_duration(
                           positions.back(), orientations.back(), position, orientation,
                           ctc_params_.max_cartesian_speed, ctc_params_.max_angular_speed,
                           ctc_params_.resample_dt);
    }
    times.push_back(t);
    positions.push_back(position);
    orientations.push_back(orientation);
  }

  return times.size() >= 2 && times.back() > 0.0;
}

bool CartesianTrajectoryController::solve_ik_along_path(
  const CartesianTrajectory & path, Eigen::VectorXd & q,
  trajectory_msgs::msg::JointTrajectory & joint_traj)
{
  const std::string & tip = ctc_params_.kinematics.tip;
  const double dt = ctc_params_.resample_dt;
  const auto steps = static_cast<size_t>(std::ceil(path.duration() / dt));

  joint_traj.joint_names = params_.joints;
  Eigen::Vector3d target_position;
  Eigen::Quaterniond target_orientation;
  for (size_t k = 1; k <= steps; ++k)
  {
    const double t = std::min(static_cast<double>(k) * dt, path.duration());
    if (!path.sample(t, target_position, target_orientation))
    {
      return false;
    }

    Eigen::Matrix<double, 7, 1> x_target;
    x_target << target_position.x(), target_position.y(), target_position.z(),
      target_orientation.x(), target_orientation.y(), target_orientation.z(),
      target_orientation.w();

    Eigen::Isometry3d current;
    if (!kinematics_->calculate_link_transform(q, tip, current))
    {
      return false;
    }
    const Eigen::Quaterniond current_orientation(current.rotation());
    Eigen::Matrix<double, 7, 1> x_current;
    x_current << current.translation().x(), current.translation().y(), current.translation().z(),
      current_orientation.x(), current_orientation.y(), current_orientation.z(),
      current_orientation.w();

    Eigen::Matrix<double, 6, 1> delta_x;
    Eigen::VectorXd delta_q;
    if (
      !kinematics_->calculate_frame_difference(x_target, x_current, 1.0, delta_x) ||
      !kinematics_->convert_cartesian_deltas_to_joint_deltas(q, delta_x, tip, delta_q))
    {
      return false;
    }
    q += delta_q;

    trajectory_msgs::msg::JointTrajectoryPoint jp;
    jp.positions.assign(q.data(), q.data() + dof_);
    jp.time_from_start = rclcpp::Duration::from_seconds(t);
    joint_traj.points.push_back(std::move(jp));
  }

  return !joint_traj.points.empty();
}

}  // namespace cartesian_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_trajectory_controller::CartesianTrajectoryController,
  controller_interface::ControllerInterface)
