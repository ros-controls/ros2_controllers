///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2014, SRI International
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of SRI International nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Sachin Chitta, Adolfo Rodriguez Tsouroukdissian

#pragma once


// C++ standard
#include <cassert>
#include <stdexcept>
#include <string>
#include <memory>

// ROS
#include <ros/node_handle.h>

// ROS messages
#include <control_msgs/GripperCommandAction.h>

// actionlib
#include <actionlib/server/action_server.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <realtime_tools/realtime_buffer.h>

// Project
#include <gripper_action_controller/hardware_interface_adapter.h>

namespace gripper_action_controller
{

/**
 * \brief Controller for executing a gripper command action for simple single-dof grippers.
 *
 * \tparam HardwareInterface Controller hardware interface. Currently \p hardware_interface::PositionJointInterface and
 * \p hardware_interface::EffortJointInterface are supported out-of-the-box.
 */
template <class HardwareInterface>
class GripperActionController : public controller_interface::Controller<HardwareInterface>
{
public:

  /**
   * \brief Store position and max effort in struct to allow easier realtime buffer usage
   */
  struct Commands
  {
    double position_; // Last commanded position
    double max_effort_; // Max allowed effort
  };

  GripperActionController();

  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  /*\}*/

  /** \name Real-Time Safe Functions
   *\{*/
  /** \brief Holds the current position. */
  void starting(const ros::Time& time);

  /** \brief Cancels the active action goal, if any. */
  void stopping(const ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);
  /*\}*/

  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_, command_struct_rt_; // pre-allocated memory that is re-used to set the realtime buffer

private:

  typedef actionlib::ActionServer<control_msgs::GripperCommandAction>                         ActionServer;
  typedef std::shared_ptr<ActionServer>                                                       ActionServerPtr;
  typedef ActionServer::GoalHandle                                                            GoalHandle;
  typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::GripperCommandAction>        RealtimeGoalHandle;
  typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;

  typedef HardwareInterfaceAdapter<HardwareInterface> HwIfaceAdapter;

  bool                                          update_hold_position_;

  bool                                         verbose_;            ///< Hard coded verbose flag to help in debugging
  std::string                                  name_;               ///< Controller name.
  hardware_interface::JointHandle joint_;                          ///< Handles to controlled joints.
  std::string                     joint_name_;                      ///< Controlled joint names.

  HwIfaceAdapter                               hw_iface_adapter_;   ///< Adapts desired goal state to HW interface.

  RealtimeGoalHandlePtr                        rt_active_goal_;     ///< Currently active action goal, if any.
  control_msgs::GripperCommandResultPtr        pre_alloc_result_;

  ros::Duration action_monitor_period_;

  // ROS API
  ros::NodeHandle    controller_nh_;
  ActionServerPtr    action_server_;

  ros::Timer         goal_handle_timer_;

  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void preemptActiveGoal();
  void setHoldPosition(const ros::Time& time);

  ros::Time last_movement_time_;                                    ///< Store stall time
  double computed_command_;                                         ///< Computed command

  double stall_timeout_, stall_velocity_threshold_;                 ///< Stall related parameters
  double default_max_effort_;                                       ///< Max allowed effort
  double goal_tolerance_;
  /**
   * \brief Check for success and publish appropriate result and feedback.
   **/
  void checkForSuccess(const ros::Time& time, double error_position, double current_position, double current_velocity);

};

} // namespace

#include <gripper_action_controller/gripper_action_controller_impl.h>
