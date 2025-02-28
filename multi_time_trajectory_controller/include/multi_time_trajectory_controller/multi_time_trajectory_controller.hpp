// Copyright (c) 2024 ros2_control Development Team
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

#ifndef MULTI_TIME_TRAJECTORY_CONTROLLER__MULTI_TIME_TRAJECTORY_CONTROLLER_HPP_
#define MULTI_TIME_TRAJECTORY_CONTROLLER__MULTI_TIME_TRAJECTORY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_server_goal_handle.hpp>

#include <control_toolbox/pid.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <joint_limits/joint_limiter_interface.hpp>
#include <joint_limits/joint_limits.hpp>
#include <pluginlib/class_loader.hpp>
#include <urdf/model.hpp>

#include "control_msgs/action/follow_axis_trajectory.hpp"
#include "control_msgs/msg/axis_trajectory_point.hpp"
#include "control_msgs/msg/multi_axis_trajectory.hpp"
#include "control_msgs/msg/multi_time_trajectory_controller_state.hpp"
#include "control_msgs/srv/query_trajectory_state.hpp"
#include "control_msgs/srv/reset_dofs.hpp"
#include "controller_interface/controller_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/server.hpp"
#include "tolerances.hpp"
#include "trajectory.hpp"

// auto-generated by generate_parameter_library
#include "multi_time_trajectory_controller/multi_time_trajectory_controller_parameters.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace multi_time_trajectory_controller
{

control_msgs::msg::AxisTrajectoryPoint emptyTrajectoryPoint();

class MultiTimeTrajectoryController : public controller_interface::ControllerInterface
{
public:
  MultiTimeTrajectoryController();

  /**
   * @brief command_interface_configuration
   */

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief command_interface_configuration
   */

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  using ControllerReferenceMsg = control_msgs::msg::MultiAxisTrajectory;
  using ControllerFeedbackMsg = nav_msgs::msg::Odometry;

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
  std::vector<control_msgs::msg::AxisTrajectoryPoint> state_current_;
  std::vector<control_msgs::msg::AxisTrajectoryPoint> command_current_;
  std::vector<control_msgs::msg::AxisTrajectoryPoint> state_desired_;
  std::vector<control_msgs::msg::AxisTrajectoryPoint> state_error_;
  std::vector<control_msgs::msg::AxisTrajectoryPoint> splines_state_;

  // Degrees of freedom
  size_t dof_;

  // Storing command axis names for interfaces
  std::vector<std::string> command_axis_names_;

  // Parameters from ROS for multi_time_trajectory_controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::vector<control_msgs::msg::AxisTrajectoryPoint> last_commanded_state_;
  std::vector<rclcpp::Time> last_commanded_time_;
  /// Specify interpolation method. Default to splines.
  joint_trajectory_controller::interpolation_methods::InterpolationMethod interpolation_method_{
    joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION};

  // The interfaces are defined as the types in 'allowed_interface_types_' member.
  // For convenience, for each type the interfaces are ordered so that i-th position
  // matches i-th index in axis_names_
  template <typename T>
  using AxisInterfaceRefs = std::vector<std::reference_wrapper<T>>;
  template <typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> axis_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> axis_state_interface_;

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
  // Configuration for every axis if it wraps around (ie. is continuous, position error is
  // normalized)
  std::vector<bool> axis_angle_wraparound_;

  // joint limiter configuration for JTC
  std::vector<joint_limits::JointLimits> joint_limits_;

  using JointLimiter =
    joint_limits::JointLimiterInterface<trajectory_msgs::msg::JointTrajectoryPoint>;
  std::shared_ptr<pluginlib::ClassLoader<JointLimiter>> joint_limiter_loader_;
  std::unique_ptr<JointLimiter> joint_limiter_;

  // Timeout to consider commands old
  double cmd_timeout_;
  // True if holding position or repeating last trajectory point in case of success
  realtime_tools::RealtimeBuffer<bool> rt_is_holding_;
  // TODO(karsten1987): eventually activate and deactivate subscriber directly when its supported
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<control_msgs::msg::MultiAxisTrajectory>::SharedPtr axis_command_subscriber_ =
    nullptr;

  rclcpp::Service<control_msgs::srv::QueryTrajectoryState>::SharedPtr query_state_srv_;

  std::shared_ptr<Trajectory> traj_external_point_ptr_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<control_msgs::msg::MultiAxisTrajectory>>
    traj_msg_external_point_ptr_;
  control_msgs::msg::MultiAxisTrajectory trajectory_msg_recvd_;
  std::vector<double> reset_dofs_positions_;
  bool is_reliable_update_pending_ = false;

  std::shared_ptr<control_msgs::msg::MultiAxisTrajectory> hold_position_msg_ptr_ = nullptr;

  using FollowJTrajAction = control_msgs::action::FollowAxisTrajectory;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowJTrajAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  rclcpp_action::Server<FollowJTrajAction>::SharedPtr action_server_;
  RealtimeGoalHandleBuffer rt_active_goal_;  ///< Currently active action goal, if any.
  realtime_tools::RealtimeBuffer<bool> rt_has_pending_goal_;  ///< Is there a pending action goal?
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(50ms);

  using ControllerResetDofsSrvType = control_msgs::srv::ResetDofs;

  struct ResetDofsData
  {
    bool reset;
    double position;
    double velocity;
    double acceleration;
  };
  realtime_tools::RealtimeBuffer<std::vector<ResetDofsData>> reset_dofs_flags_;
  rclcpp::Service<ControllerResetDofsSrvType>::SharedPtr reset_dofs_service_;

  using ControllerStateMsg = control_msgs::msg::MultiTimeTrajectoryControllerState;
  using StatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  using StatePublisherPtr = std::unique_ptr<StatePublisher>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr publisher_;
  StatePublisherPtr state_publisher_;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr splines_output_pub_;
  StatePublisherPtr splines_output_publisher_;

  using TrajectoryPoint = control_msgs::msg::AxisTrajectoryPoint;

  virtual void publish_state(
    const rclcpp::Time & time, const bool first_sample, const std::vector<int> & segment_start);

  // callback for topic interface

  void topic_callback(const std::shared_ptr<control_msgs::msg::MultiAxisTrajectory> msg);

  // callbacks for action_server_

  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal);

  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle);

  /**
   * Computes the error for a specific axis in the trajectory.
   *
   * @param[out] error The computed error for the axis.
   * @param[in] index The index of the axis in the trajectory.
   * @param[in] current The current state of the axes.
   * @param[in] desired The desired state of the axes.
   */

  void compute_error(
    std::vector<TrajectoryPoint> & error, const std::vector<TrajectoryPoint> & current,
    const std::vector<TrajectoryPoint> & desired) const;

  // sorts the axes of the incoming message to our local order

  void sort_to_local_axis_order(
    std::shared_ptr<control_msgs::msg::MultiAxisTrajectory> trajectory_msg) const;

  bool validate_trajectory_msg(const control_msgs::msg::MultiAxisTrajectory & trajectory) const;

  virtual void add_new_trajectory_msg(
    const std::shared_ptr<control_msgs::msg::MultiAxisTrajectory> & traj_msg,
    const bool reliable = false);

  bool validate_trajectory_point_field(
    size_t axis_names_size, const std::vector<double> & vector_field,
    const std::string & string_for_vector_field, size_t i, bool allow_empty) const;

  SegmentTolerances default_tolerances_;

  void preempt_active_goal();

  /** @brief set the current position with zero velocity and acceleration as new command
   */

  std::shared_ptr<control_msgs::msg::MultiAxisTrajectory> set_hold_position();

  /** @brief set last trajectory point to be repeated at success
   *
   * no matter if it has nonzero velocity or acceleration
   */

  std::shared_ptr<control_msgs::msg::MultiAxisTrajectory> set_success_trajectory_point();

  bool reset();

  bool has_active_trajectory() const;

  virtual bool read_state_from_hardware(std::vector<TrajectoryPoint> & state);

  /** Assign values from the command interfaces as state.
   * This is only possible if command AND state interfaces exist for the same type,
   *  therefore needs check for both.
   * @param[out] state to be filled with values from command interfaces.
   * @return true if all interfaces exists and contain non-NaN values, false otherwise.
   */
  bool read_state_from_command_interfaces(std::vector<TrajectoryPoint> & state);
  void read_commands_from_command_interfaces(std::vector<TrajectoryPoint> & commands);

  void query_state_service(
    const std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Request> request,
    std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Response> response);

  // BEGIN: helper methods
  template <typename T>
  void assign_positions_from_interface(
    std::vector<TrajectoryPoint> & trajectory_point_interface,
    const AxisInterfaceRefs<T> & axis_interface)
  {
    for (size_t index = 0; index < dof_; ++index)
    {
      trajectory_point_interface[index].position = axis_interface[index].get().get_value();
    }
  };
  template <typename T>
  void assign_velocities_from_interface(
    std::vector<TrajectoryPoint> & trajectory_point_interface,
    const AxisInterfaceRefs<T> & axis_interface)
  {
    for (size_t index = 0; index < dof_; ++index)
    {
      trajectory_point_interface[index].velocity = axis_interface[index].get().get_value();
    }
  };
  template <typename T>
  void assign_accelerations_from_interface(
    std::vector<TrajectoryPoint> & trajectory_point_interface,
    const AxisInterfaceRefs<T> & axis_interface)
  {
    for (size_t index = 0; index < dof_; ++index)
    {
      trajectory_point_interface[index].acceleration = axis_interface[index].get().get_value();
    }
  };

private:
  rclcpp::Subscription<ControllerFeedbackMsg>::SharedPtr feedback_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerFeedbackMsg>> feedback_;
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg, const bool reliable);
  ControllerFeedbackMsg last_odom_feedback_;
  ControllerReferenceMsg last_reference_;
  ControllerReferenceMsg last_reliable_reference_;
  bool current_state_initialized_{false};
  using JointTrajectoryPoint = control_msgs::msg::AxisTrajectoryPoint;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_reliable_ = nullptr;

  // for mutual exclusion of the reference callback and the update function
  std::mutex mutex_;

  void update_pids();

  bool contains_interface_type(
    const std::vector<std::string> & interface_type_list, const std::string & interface_type);

  bool initialize_current_state();

  void init_hold_position_msg();
};
}  // namespace multi_time_trajectory_controller
#endif  // MULTI_TIME_TRAJECTORY_CONTROLLER__MULTI_TIME_TRAJECTORY_CONTROLLER_HPP_
