// Copyright (c) 2021 ros2_control Development Team
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

#ifndef JOINT_TRAJECTORY_CONTROLLER__JOINT_TRAJECTORY_CONTROLLER_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "control_toolbox/pid.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_trajectory_controller/interpolation_methods.hpp"
#include "joint_trajectory_controller/tolerances.hpp"
#include "joint_trajectory_controller/visibility_control.h"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_server_goal_handle.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "joint_trajectory_controller_parameters.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace rclcpp_action
{
template <typename ActionT>
class ServerGoalHandle;
}  // namespace rclcpp_action
namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace joint_trajectory_controller
{
class Trajectory;

class JointTrajectoryController : public controller_interface::ControllerInterface
{
public:
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  JointTrajectoryController();

  /**
   * @brief command_interface_configuration This controller requires the position command
   * interfaces for the controlled joints
   */
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief command_interface_configuration This controller requires the position and velocity
   * state interfaces for the controlled joints
   */
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // To reduce number of variables and to make the code shorter the interfaces are ordered in types
  // as the following constants
  const std::vector<std::string> allowed_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION,
    hardware_interface::HW_IF_EFFORT,
  };

  // Preallocate variables used in the realtime update() function
  trajectory_msgs::msg::JointTrajectoryPoint state_current_;
  trajectory_msgs::msg::JointTrajectoryPoint state_desired_;
  trajectory_msgs::msg::JointTrajectoryPoint state_error_;

  // Degrees of freedom
  size_t dof_;

  // Storing command joint names for interfaces
  std::vector<std::string> command_joint_names_;

  // Parameters from ROS for joint_trajectory_controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  trajectory_msgs::msg::JointTrajectoryPoint last_commanded_state_;
  /// Specify interpolation method. Default to splines.
  interpolation_methods::InterpolationMethod interpolation_method_{
    interpolation_methods::DEFAULT_INTERPOLATION};

  // The interfaces are defined as the types in 'allowed_interface_types_' member.
  // For convenience, for each type the interfaces are ordered so that i-th position
  // matches i-th index in joint_names_
  template <typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;

  bool has_position_state_interface_ = false;
  bool has_velocity_state_interface_ = false;
  bool has_acceleration_state_interface_ = false;
  bool has_position_command_interface_ = false;
  bool has_velocity_command_interface_ = false;
  bool has_acceleration_command_interface_ = false;
  bool has_effort_command_interface_ = false;

  /// If true, a velocity feedforward term plus corrective PID term is used
  bool use_closed_loop_pid_adapter_ = false;
  using PidPtr = std::shared_ptr<control_toolbox::Pid>;
  std::vector<PidPtr> pids_;
  // Feed-forward velocity weight factor when calculating closed loop pid adapter's command
  std::vector<double> ff_velocity_scale_;
  // reserved storage for result of the command when closed loop pid adapter is used
  std::vector<double> tmp_command_;

  // TODO(karsten1987): eventually activate and deactivate subscriber directly when its supported
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_ =
    nullptr;

  std::shared_ptr<Trajectory> * traj_point_active_ptr_ = nullptr;
  std::shared_ptr<Trajectory> traj_external_point_ptr_ = nullptr;
  std::shared_ptr<Trajectory> traj_home_point_ptr_ = nullptr;
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg_home_ptr_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
    traj_msg_external_point_ptr_;

  using ControllerStateMsg = control_msgs::msg::JointTrajectoryControllerState;
  using StatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  using StatePublisherPtr = std::unique_ptr<StatePublisher>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr publisher_;
  StatePublisherPtr state_publisher_;

  rclcpp::Duration state_publisher_period_ = rclcpp::Duration(20ms);
  rclcpp::Time last_state_publish_time_;

  using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJTrajAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  rclcpp_action::Server<FollowJTrajAction>::SharedPtr action_server_;
  RealtimeGoalHandleBuffer rt_active_goal_;  ///< Currently active action goal, if any.
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(50ms);

  // callback for topic interface
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void topic_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg);

  // callbacks for action_server_
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal);
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  // fill trajectory_msg so it matches joints controlled by this controller
  // positions set to current position, velocities, accelerations and efforts to 0.0
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void fill_partial_goal(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const;
  // sorts the joints of the incoming message to our local order
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void sort_to_local_joint_order(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg);
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool validate_trajectory_msg(const trajectory_msgs::msg::JointTrajectory & trajectory) const;
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void add_new_trajectory_msg(
    const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & traj_msg);
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool validate_trajectory_point_field(
    size_t joint_names_size, const std::vector<double> & vector_field,
    const std::string & string_for_vector_field, size_t i, bool allow_empty) const;

  SegmentTolerances default_tolerances_;

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void preempt_active_goal();
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void set_hold_position();

  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  bool reset();

  using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
  JOINT_TRAJECTORY_CONTROLLER_PUBLIC
  void publish_state(
    const JointTrajectoryPoint & desired_state, const JointTrajectoryPoint & current_state,
    const JointTrajectoryPoint & state_error);

  void read_state_from_hardware(JointTrajectoryPoint & state);

  bool read_state_from_command_interfaces(JointTrajectoryPoint & state);

private:
  bool contains_interface_type(
    const std::vector<std::string> & interface_type_list, const std::string & interface_type);

  void resize_joint_trajectory_point(
    trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size);
};

}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__JOINT_TRAJECTORY_CONTROLLER_HPP_
