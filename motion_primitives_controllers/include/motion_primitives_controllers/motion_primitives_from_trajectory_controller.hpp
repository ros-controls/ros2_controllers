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

#ifndef MOTION_PRIMITIVES_CONTROLLERS__MOTION_PRIMITIVES_FROM_TRAJECTORY_CONTROLLER_HPP_
#define MOTION_PRIMITIVES_CONTROLLERS__MOTION_PRIMITIVES_FROM_TRAJECTORY_CONTROLLER_HPP_

#include <memory>
#include <vector>

#include <motion_primitives_controllers/motion_primitives_from_trajectory_controller_parameters.hpp>
#include "motion_primitives_controllers/motion_primitives_base_controller.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "motion_primitives_controllers/approx_primitives_with_rdp.hpp"
#include "motion_primitives_controllers/fk_client.hpp"

namespace motion_primitives_controllers
{
enum class ApproxMode
{
  RDP_PTP,
  RDP_LIN
};

class MotionPrimitivesFromTrajectoryController : public MotionPrimitivesBaseController
{
public:
  MotionPrimitivesFromTrajectoryController() = default;
  ~MotionPrimitivesFromTrajectoryController() override = default;

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
  std::shared_ptr<motion_primitives_from_trajectory_controller::ParamListener> param_listener_;
  motion_primitives_from_trajectory_controller::Params params_;

  using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;
  rclcpp_action::Server<FollowJTrajAction>::SharedPtr action_server_;
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);
  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJTrajAction>;
  realtime_tools::RealtimeThreadSafeBox<std::shared_ptr<RealtimeGoalHandle>> rt_goal_handle_;

  std::shared_ptr<FKClient> fk_client_;

  ApproxMode approx_mode_;
  bool use_time_not_vel_and_acc_;
  double epsilon_joint_angle_;
  double epsilon_cart_position_;
  double epsilon_cart_angle_;

  double blend_radius_percentage_;
  double blend_radius_lower_limit_;
  double blend_radius_upper_limit_;
  double blend_radius_overwrite_;

  double joint_vel_overwrite_;
  double joint_acc_overwrite_;
  double max_traj_joint_vel_;
  double max_traj_joint_acc_;
  void get_max_traj_joint_vel_and_acc(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & trajectory_msg,
    double & max_traj_joint_vel, double & max_traj_joint_acc);
  double cart_vel_overwrite_;
  double cart_acc_overwrite_;
  double max_traj_cart_vel_;
  double max_traj_cart_acc_;
  void get_max_traj_cart_vel_and_acc(
    const geometry_msgs::msg::PoseArray & planned_poses_msg,
    const std::vector<double> & time_from_start, double & max_vel, double & max_acc);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr planned_trajectory_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr planned_poses_publisher_;
  rclcpp::Publisher<control_msgs::msg::MotionPrimitiveSequence>::SharedPtr
    motion_primitive_publisher_;

  // ############ Function copied from JointTrajectoryController ############
  void sort_to_local_joint_order(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const;
};

// ############ Function copied from JointTrajectoryController ############
/**
 * \return The map between \p t1 indices (implicitly encoded in return vector indices) to \p t2
 * indices. If \p t1 is <tt>"{C, B}"</tt> and \p t2 is <tt>"{A, B, C, D}"</tt>, the associated
 * mapping vector is <tt>"{2, 1}"</tt>. return empty vector if \p t1 is not a subset of \p t2.
 */
template <class T>
std::vector<size_t> mapping(const T & t1, const T & t2)
{
  // t1 must be a subset of t2
  if (t1.size() > t2.size())
  {
    return std::vector<size_t>();
  }

  std::vector<size_t> mapping_vector(t1.size());  // Return value
  for (auto t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    auto t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it)
    {
      return std::vector<size_t>();
    }
    else
    {
      const size_t t1_dist = static_cast<size_t>(std::distance(t1.begin(), t1_it));
      const size_t t2_dist = static_cast<size_t>(std::distance(t2.begin(), t2_it));
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

}  // namespace motion_primitives_controllers

#endif  // MOTION_PRIMITIVES_CONTROLLERS__MOTION_PRIMITIVES_FROM_TRAJECTORY_CONTROLLER_HPP_
