// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef GRIPPER_IO_CONTROLLER__GRIPPER_IO_CONTROLLER_HPP_
#define GRIPPER_IO_CONTROLLER__GRIPPER_IO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <set>
#include <atomic>

#include "controller_interface/controller_interface.hpp"
#include "io_gripper_controller_parameters.hpp"
#include "io_gripper_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

// TODO(anyone): Replace with controller specific messages
#include <sensor_msgs/msg/joint_state.hpp>

#include "control_msgs/srv/set_config.hpp"
#include "control_msgs/msg/io_gripper_sensor.hpp"
#include "control_msgs/msg/interface_value.hpp"
#include "control_msgs/msg/dynamic_interface_values.hpp"
#include "control_msgs/action/gripper.hpp"
#include "control_msgs/action/set_gripper_config.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace io_gripper_controller
{

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};

enum class service_mode_type : std::uint8_t
{
  IDLE = 0,
  OPEN = 1,
  CLOSE = 2,
};

// TODO : rearrange it later
enum class gripper_state_type : std::uint8_t
{
  IDLE = 0,
  STORE_ORIGINAL_STATE = 1,
  SET_BEFORE_COMMAND = 2,
  CLOSE_GRIPPER = 3,
  CHECK_GRIPPER_STATE = 4,
  RESTORE_ORIGINAL_STATE = 5,
  CHECK_RESTORE_STATE = 6,
  OPEN_GRIPPER = 7,
  START_CLOSE_GRIPPER = 8,
  SET_AFTER_COMMAND = 9,
  HALTED = 10,
};

enum class reconfigure_state_type : std::uint8_t
{
  IDLE = 0,
  RECONFIGURE = 1,
  FIND_CONFIG = 2,
  SET_COMMAND = 3,
  CHECK_STATE = 4,
};

class IOGripperController : public controller_interface::ControllerInterface
{
public:
  GRIPPER_IO_CONTROLLER__VISIBILITY_PUBLIC
  IOGripperController();
  io_gripper_controller::Params params_;

  GRIPPER_IO_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  GRIPPER_IO_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  GRIPPER_IO_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GRIPPER_IO_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  GRIPPER_IO_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  GRIPPER_IO_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  GRIPPER_IO_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<std::string> command_ios_open, command_ios_close, set_before_command_open, set_after_command_open, reconfigure_command;
  std::vector<double> command_ios_open_values, command_ios_close_values, set_before_command_open_values, set_after_command_open_values, reconfigure_command_values;
  std::vector<std::string> state_ios_open, state_ios_close, set_before_command_close, set_after_command_close;
  std::vector<double> state_ios_open_values, state_ios_close_values, set_before_command_close_values, set_after_command_close_values, set_after_command_open_values_original_;
  std::string status_joint_name;
  bool is_open;
  std::unordered_map<std::string, double> command_if_ios_after_opening;
  std::unordered_map<std::string, double> original_ios_after_opening;
  std::unordered_map<std::string, double> command_if_ios_before_closing;
  std::unordered_map<std::string, double> original_ios_before_closing;

  std::unordered_set<std::string> command_if_ios, state_if_ios;

  bool setResult;


  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using OpenSrvType = std_srvs::srv::Trigger;
  using ConfigSrvType = control_msgs::srv::SetConfig;
  using ControllerStateMsg = sensor_msgs::msg::JointState;
  using EventStateMsg = sensor_msgs::msg::JointState;
  using ConfigJointMsg = sensor_msgs::msg::JointState;
  using InterfaceMsg = control_msgs::msg::DynamicInterfaceValues;
  using GripperAction = control_msgs::action::Gripper;
  using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperAction>;
  using GripperConfigAction = control_msgs::action::SetGripperConfig;
  using GoalHandleGripperConfig = rclcpp_action::ServerGoalHandle<GripperConfigAction>;

protected:
  std::shared_ptr<io_gripper_controller::ParamListener> param_listener_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  rclcpp::Service<OpenSrvType>::SharedPtr open_service_;
  rclcpp::Service<OpenSrvType>::SharedPtr close_service_;
  rclcpp::Service<ConfigSrvType>::SharedPtr configure_gripper_service_;

  rclcpp_action::Server<GripperAction>::SharedPtr gripper_action_server_;
  rclcpp_action::Server<GripperConfigAction>::SharedPtr gripper_config_action_server_;

  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;
  realtime_tools::RealtimeBuffer<service_mode_type> service_buffer_;
  realtime_tools::RealtimeBuffer<std::string> configure_gripper_buffer_;
  realtime_tools::RealtimeBuffer<gripper_state_type> gripper_state_buffer_;
  realtime_tools::RealtimeBuffer<reconfigure_state_type> reconfigure_state_buffer_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;
  using EventPublisher = realtime_tools::RealtimePublisher<EventStateMsg>;

  using ConfigPublisher = realtime_tools::RealtimePublisher<ConfigJointMsg>;
  using InterfacePublisher = realtime_tools::RealtimePublisher<InterfaceMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr g_j_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> gripper_joint_state_publisher_;

  std::vector<double> joint_state_values_;

  rclcpp::Publisher<InterfaceMsg>::SharedPtr if_publisher_;
  std::unique_ptr<InterfacePublisher> interface_publisher_;


  rclcpp::Publisher<EventStateMsg>::SharedPtr e_publisher_;
  std::unique_ptr<EventPublisher> event_publisher_;

  std::atomic<bool> reconfigureFlag_{false}, openFlag_{false}, closeFlag_{false};
  // std::atomic<bool> reconfigFlag{false};

private:
  bool find_and_set_command(const std::string & name, const double value);
  bool find_and_get_state(const std::string & name, double& value);
  bool find_and_get_command(const std::string & name, double& value);
  void init_msgs();
  void handle_gripper_state_transition_open(const gripper_state_type & state);
  void handle_gripper_state_transition_close(const gripper_state_type & state);
  void handle_reconfigure_state_transition(const reconfigure_state_type & state);
  /// \brief Function to check the parameters
  controller_interface::CallbackReturn check_parameters();
  /// Preparing the command ios and states ios vars for the command/state interface configuraiton 
  void prepare_command_and_state_ios();
  controller_interface::CallbackReturn prepare_publishers_and_services();
  void publish_gripper_joint_states();
  void publish_dynamic_interface_values();
  void publish_reconfigure_gripper_joint_states();
  void check_gripper_and_reconfigure_state();
  
  std::vector<std::string> configurations_list_;
  std::vector<io_gripper_controller::Params::ConfigurationSetup::MapConfigurations> config_map_;
  std::vector<io_gripper_controller::Params::SensorsInterfaces::MapGripperSpecificSensors> sensors_map_;
  double state_value_;
  std::string configuration_key_;
  bool check_state_ios_;
  std::string closed_state_name_;
  io_gripper_controller::Params::Close::State::MapPossibleClosedStates closed_state_values_;
  io_gripper_controller::Params::ConfigurationSetup::MapConfigurations conf_it_;
  std::vector<std::string>::iterator config_index_;

  rclcpp::CallbackGroup::SharedPtr open_service_callback_group_, close_service_callback_group_, reconfigure_service_callback_group_;


  std::shared_ptr<control_msgs::action::Gripper_Feedback> gripper_feedback_;
  std::shared_ptr<control_msgs::action::Gripper_Result> gripper_result_;
  std::shared_ptr<control_msgs::action::SetGripperConfig_Feedback> gripper_config_feedback_;
  std::shared_ptr<control_msgs::action::SetGripperConfig_Result> gripper_config_result_;



  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGripper> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);
  void execute(const std::shared_ptr<GoalHandleGripper> goal_handle);
  
  rclcpp_action::GoalResponse config_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperConfigAction::Goal> goal);


  rclcpp_action::CancelResponse config_handle_cancel(
    const std::shared_ptr<GoalHandleGripperConfig> goal_handle);

  void config_handle_accepted(const std::shared_ptr<GoalHandleGripperConfig> goal_handle);
  void config_execute(const std::shared_ptr<GoalHandleGripperConfig> goal_handle);
  





};

}  // namespace io_gripper_controller

#endif  // GRIPPER_IO_CONTROLLER__GRIPPER_IO_CONTROLLER_HPP_
