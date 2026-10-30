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
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp/duration.hpp"

namespace cartesian_trajectory_controller
{
namespace
{
/// [x, y, z, qx, qy, qz, qw], the pose layout kinematics_interface expects
Eigen::Matrix<double, 7, 1> to_pose_vector(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation)
{
  Eigen::Matrix<double, 7, 1> pose;
  pose << position, orientation.x(), orientation.y(), orientation.z(), orientation.w();
  return pose;
}
}  // namespace

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

  // Disable JTC's joint-space command inputs; only ~/cartesian_reference drives this controller.
  joint_command_subscriber_.reset();
  action_server_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianTrajectoryController::reference_callback(
  std::shared_ptr<trajectory_msgs::msg::MultiDOFJointTrajectory> msg)
{
  if (!subscriber_is_active_)
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
  if (msg.points.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring trajectory: message has no points.");
    return false;
  }
  if (!msg.header.frame_id.empty() && msg.header.frame_id != ctc_params_.kinematics.base)
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "Ignoring trajectory: frame_id '%s' is not the base frame '%s'.",
      msg.header.frame_id.c_str(), ctc_params_.kinematics.base.c_str());
    return false;
  }

  // the trajectory starts from the commanded state, NaN until the first update() has commanded
  const auto commanded = rt_last_commanded_state_.get();
  const Eigen::VectorXd q_seed =
    Eigen::Map<const Eigen::VectorXd>(commanded.positions.data(), commanded.positions.size());
  if (q_seed.size() != static_cast<Eigen::Index>(dof_) || !q_seed.allFinite())
  {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring trajectory: no commanded state yet.");
    return false;
  }
  // rest if the commanded velocity is unavailable
  std::vector<double> start_velocity;
  if (
    commanded.velocities.size() == dof_ &&
    std::all_of(
      commanded.velocities.begin(), commanded.velocities.end(),
      [](double v) { return std::isfinite(v); }))
  {
    start_velocity = commanded.velocities;
  }

  Eigen::Isometry3d current_pose;
  if (!kinematics_->calculate_link_transform(q_seed, ctc_params_.kinematics.tip, current_pose))
  {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring trajectory: forward kinematics failed.");
    return false;
  }

  std::vector<double> times;
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Quaterniond> orientations;
  if (!build_cartesian_waypoints(msg, current_pose, times, positions, orientations))
  {
    return false;
  }

  Eigen::Vector3d initial_velocity = Eigen::Vector3d::Zero();
  double initial_angular_speed = 0.0;
  Eigen::Matrix<double, 6, 1> twist;
  if (
    !start_velocity.empty() &&
    kinematics_->convert_joint_deltas_to_cartesian_deltas(
      q_seed, Eigen::Map<const Eigen::VectorXd>(start_velocity.data(), dof_),
      ctc_params_.kinematics.tip, twist))
  {
    initial_velocity = twist.head<3>();
    // the angle channel is signed along the path, so project onto the first segment's axis,
    // taken in the base frame like the twist
    const Eigen::AngleAxisd rotation(orientations[1] * orientations[0].inverse());
    if (rotation.angle() > 1e-9)
    {
      initial_angular_speed = twist.tail<3>().dot(rotation.axis());
    }
  }

  const CartesianTrajectory path(
    times, positions, orientations, initial_velocity, initial_angular_speed);
  // Carry the incoming stamp so JTC's deferred-start works.
  // TODO(vedh1234): anchor at the stamped start, not at arrival
  joint_traj.header.stamp = msg.header.stamp;
  if (!solve_ik_along_path(path, q_seed, joint_traj))
  {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring trajectory: inverse kinematics failed.");
    return false;
  }

  // joint velocities for continuous acceleration
  if (!joint_trajectory_controller::fill_cubic_spline_velocities(joint_traj, start_velocity))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to solve joint velocities for the trajectory.");
    return false;
  }
  return true;
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

  const bool has_timing = std::any_of(
    msg.points.begin(), msg.points.end(), [](const auto & point)
    { return point.time_from_start.sec != 0 || point.time_from_start.nanosec != 0u; });

  for (const auto & point : msg.points)
  {
    if (point.transforms.empty())
    {
      RCLCPP_WARN(get_node()->get_logger(), "Ignoring trajectory: a point has no transform.");
      return false;
    }
    const auto & tf = point.transforms[0];
    const Eigen::Vector3d position(tf.translation.x, tf.translation.y, tf.translation.z);
    Eigen::Quaterniond orientation(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z);
    if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Ignoring trajectory: pose is not finite or not a valid rotation.");
      return false;
    }
    orientation.normalize();

    double t;
    if (has_timing)
    {
      t = rclcpp::Duration(point.time_from_start).seconds();
      if (t <= times.back())
      {
        if (t == 0.0 && times.size() == 1)
        {
          continue;  // waypoint 0 is already the current pose
        }
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Ignoring trajectory: time_from_start is not strictly increasing.");
        return false;
      }
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

  return times.size() >= 2;
}

bool CartesianTrajectoryController::solve_ik_along_path(
  const CartesianTrajectory & path, Eigen::VectorXd q,
  trajectory_msgs::msg::JointTrajectory & joint_traj)
{
  const std::string & tip = ctc_params_.kinematics.tip;
  const double dt = ctc_params_.resample_dt;
  // rounding keeps the last segment between 0.5 and 1.5 dt; the last sample is pinned to the end
  const auto steps = static_cast<size_t>(std::max(1L, std::lround(path.duration() / dt)));

  joint_traj.joint_names = params_.joints;
  joint_traj.points.reserve(steps + 1);
  // the commanded state at t=0, as JTC's prepend_commanded_state does
  trajectory_msgs::msg::JointTrajectoryPoint anchor;
  anchor.positions.assign(q.data(), q.data() + dof_);
  anchor.time_from_start = rclcpp::Duration(0, 0);
  joint_traj.points.push_back(std::move(anchor));

  Eigen::Vector3d target_position;
  Eigen::Quaterniond target_orientation;
  Eigen::VectorXd delta_q = Eigen::VectorXd::Zero(dof_);
  for (size_t k = 1; k <= steps; ++k)
  {
    const double t = (k == steps) ? path.duration() : static_cast<double>(k) * dt;
    if (!path.sample(t, target_position, target_orientation))
    {
      return false;
    }

    Eigen::Isometry3d current;
    if (!kinematics_->calculate_link_transform(q, tip, current))
    {
      return false;
    }

    Eigen::Matrix<double, 7, 1> x_current =
      to_pose_vector(current.translation(), Eigen::Quaterniond(current.rotation()));
    Eigen::Matrix<double, 7, 1> x_target = to_pose_vector(target_position, target_orientation);
    Eigen::Matrix<double, 6, 1> delta_x;
    if (
      !kinematics_->calculate_frame_difference(x_current, x_target, 1.0, delta_x) ||
      !kinematics_->convert_cartesian_deltas_to_joint_deltas(q, delta_x, tip, delta_q))
    {
      return false;
    }
    // TODO(vedh1234): respect joint limits and resolve redundancy
    q += delta_q;
    if (!q.allFinite())
    {
      return false;
    }
    trajectory_msgs::msg::JointTrajectoryPoint jp;
    jp.positions.assign(q.data(), q.data() + dof_);
    jp.time_from_start = rclcpp::Duration::from_seconds(t);
    joint_traj.points.push_back(std::move(jp));
  }

  // TODO(vedh1234): reject the trajectory if IK does not converge
  return true;
}

}  // namespace cartesian_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cartesian_trajectory_controller::CartesianTrajectoryController,
  controller_interface::ControllerInterface)
