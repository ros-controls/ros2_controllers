// Copyright (c) 2025, b>>robotized by Stogl Robotics
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

#ifndef IO_GRIPPER_CONTROLLER__IO_GRIPPER_CONTROLLER_HPP_
#define IO_GRIPPER_CONTROLLER__IO_GRIPPER_CONTROLLER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <unordered_map>
#include <unordered_set>
#include "control_msgs/action/io_gripper_command.hpp"
#include "control_msgs/action/set_io_gripper_config.hpp"
#include "control_msgs/msg/dynamic_interface_values.hpp"
#include "control_msgs/msg/interface_value.hpp"
#include "control_msgs/msg/io_gripper_controller_state.hpp"
#include "control_msgs/msg/io_gripper_state.hpp"
#include "control_msgs/srv/set_io_gripper_config.hpp"
#include "controller_interface/controller_interface.hpp"
#include "io_gripper_controller_parameters.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace io_gripper_controller
{
/**
 * @enum service_mode_type
 * @brief Represents the various service modes of the gripper. These modes represents the high level
 * states of the gripper.
 *
 * - IDLE: The gripper is in an idle state, not performing any action.
 * - OPEN: The gripper is in the process of opening.
 * - CLOSE: The gripper is in the process of closing.
 */
enum class service_mode_type : std::uint8_t
{
  IDLE = 0,
  OPEN = 1,
  CLOSE = 2,
};

/**
 * @enum gripper_state_type
 * @brief Represents the various states of the gripper.
 *
 * - IDLE: The gripper is in an idle state, not performing any action.
 * - SET_BEFORE_COMMAND: Executing commands for io defined in the yaml which are required before
 * opening the gripper.
 * - CLOSE_GRIPPER: Executing commands to close the gripper.
 * - CHECK_GRIPPER_STATE: Checking the state of the gripper to make sure the gripper is closed.
 * - OPEN_GRIPPER: Executing commands to open the gripper.
 * - SET_AFTER_COMMAND: Setting the gripper state after executing a command.
 * - HALTED: The gripper operation is halted.
 */
enum class gripper_state_type : std::uint8_t
{
  IDLE = 0,
  SET_BEFORE_COMMAND = 1,
  CLOSE_GRIPPER = 2,
  CHECK_GRIPPER_STATE = 3,
  OPEN_GRIPPER = 4,
  SET_AFTER_COMMAND = 5,
  HALTED = 6,
};

/**
 * @enum reconfigure_state_type
 * @brief Represents the various states of the reconfiguration process, which means that the gripper
 * is reconfiguring to new state based on the configuration defined in the yaml params.
 *
 * - IDLE: The reconfiguration process is idle, not performing any action.
 * - SET_COMMAND: Setting the command based on the configuration.
 * - CHECK_STATE: Checking the state after setting the command.
 */
enum class reconfigure_state_type : std::uint8_t
{
  IDLE = 0,
  SET_COMMAND = 1,
  CHECK_STATE = 2,
};

class IOGripperController : public controller_interface::ControllerInterface
/**
 * @brief IOGripperController class handles the control of an IO-based gripper.
 */
{
public:
  /**
   * @brief Constructor for IOGripperController.
   */
  IOGripperController();
  io_gripper_controller::Params params_;

  /**
   * @brief Initializes the controller.
   * @return CallbackReturn indicating success or failure.
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Configures the command interfaces.
   * @return InterfaceConfiguration for command interfaces.
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Configures the state interfaces.
   * @return InterfaceConfiguration for state interfaces.
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Configures the controller.
   * @param previous_state The previous state of the lifecycle.
   * @return CallbackReturn indicating success or failure.
   */
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activates the controller.
   * @param previous_state The previous state of the lifecycle.
   * @return CallbackReturn indicating success or failure.
   */
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivates the controller.
   * @param previous_state The previous state of the lifecycle.
   * @return CallbackReturn indicating success or failure.
   */
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Updates the controller state.
   * @param time The current time.
   * @param period The time since the last update.
   * @return return_type indicating success or failure.
   */
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  struct GripperTransitionIOs
  {
    std::unordered_map<std::string, double> command_ios;
    std::unordered_map<std::string, double> state_ios;

    bool has_multiple_end_states;
    std::vector<std::string> possible_states;
    std::unordered_map<std::string, std::unordered_map<std::string, double>> multiple_states_ios;

    std::unordered_map<std::string, double> set_before_command_ios;
    std::unordered_map<std::string, double> set_before_state_ios;

    std::unordered_map<std::string, double> set_after_command_ios;
    std::unordered_map<std::string, double> set_after_state_ios;
  };

  GripperTransitionIOs open_ios_;
  GripperTransitionIOs closeios_;

  rclcpp::Time last_transition_time_;

  std::vector<std::string> command_ios_open;
  std::vector<std::string> command_ios_close;
  std::vector<std::string> set_before_command_open;
  std::vector<std::string> set_after_command_open;
  std::vector<std::string> reconfigure_command;
  std::vector<double> command_ios_open_values;
  std::vector<double> command_ios_close_values;
  std::vector<double> set_before_command_open_values;
  std::vector<double> set_after_command_open_values;
  std::vector<double> reconfigure_command_values;
  std::vector<std::string> state_ios_open;
  std::vector<std::string> state_ios_close;
  std::vector<std::string> set_before_command_close;
  std::vector<std::string> set_after_command_close;
  std::vector<double> state_ios_open_values;
  std::vector<double> state_ios_close_values;
  std::vector<double> set_before_command_close_values;
  std::vector<double> set_after_command_close_values;
  std::vector<double> set_after_command_open_values_original_;
  std::string status_joint_name;
  bool is_open;
  std::unordered_map<std::string, double> command_if_ios_after_opening;
  std::unordered_map<std::string, double> original_ios_after_opening;
  std::unordered_map<std::string, double> command_if_ios_before_closing;
  std::unordered_map<std::string, double> original_ios_before_closing;
  std::unordered_set<std::string> command_if_ios;
  std::unordered_set<std::string> state_if_ios;
  bool setResult;

  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using OpenCloseSrvType = std_srvs::srv::Trigger;
  using ConfigSrvType = control_msgs::srv::SetIOGripperConfig;
  using JointStateMsg = sensor_msgs::msg::JointState;
  using DynInterfaceMsg = control_msgs::msg::DynamicInterfaceValues;
  using GripperAction = control_msgs::action::IOGripperCommand;
  using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperAction>;
  using GripperConfigAction = control_msgs::action::SetIOGripperConfig;
  using GoalHandleGripperConfig = rclcpp_action::ServerGoalHandle<GripperConfigAction>;

protected:
  std::shared_ptr<io_gripper_controller::ParamListener> param_listener_;
  rclcpp::Service<OpenCloseSrvType>::SharedPtr open_service_;
  rclcpp::Service<OpenCloseSrvType>::SharedPtr close_service_;
  rclcpp::Service<ConfigSrvType>::SharedPtr configure_gripper_service_;
  rclcpp_action::Server<GripperAction>::SharedPtr gripper_action_server_;
  rclcpp_action::Server<GripperConfigAction>::SharedPtr gripper_config_action_server_;

  // Realtime buffer to store the state for outer gripper_service (e.g. idle, open, close)
  realtime_tools::RealtimeBuffer<service_mode_type> gripper_service_buffer_;
  // Realtime buffer to store the state for switching the gripper state (e.g. idle,
  // set_before_command, close_gripper, check_gripper_state, open_gripper, set_after_command,
  // halted)
  realtime_tools::RealtimeBuffer<gripper_state_type> gripper_state_buffer_;
  realtime_tools::RealtimeBuffer<uint8_t> gripper_open_state_buffer_;
  // Realtime buffer to store the name of the configuration which needs to be set
  realtime_tools::RealtimeBuffer<std::string> configure_gripper_buffer_;
  // Realtime buffer to store the state for switching the reconfigure state (e.g. idle,
  // set_command, check_state)
  realtime_tools::RealtimeBuffer<reconfigure_state_type> reconfigure_state_buffer_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<JointStateMsg>;
  rclcpp::Publisher<JointStateMsg>::SharedPtr g_j_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> gripper_joint_state_publisher_;
  std::vector<double> joint_state_values_;
  using InterfacePublisher = realtime_tools::RealtimePublisher<DynInterfaceMsg>;
  rclcpp::Publisher<DynInterfaceMsg>::SharedPtr if_publisher_;
  std::unique_ptr<InterfacePublisher> interface_publisher_;
  using GripperStatePublisher =
    realtime_tools::RealtimePublisher<control_msgs::msg::IOGripperControllerState>;
  rclcpp::Publisher<control_msgs::msg::IOGripperControllerState>::SharedPtr g_s_publisher_;
  std::unique_ptr<GripperStatePublisher> gripper_state_publisher_;
  std::atomic<bool> reconfigureFlag_{false};
  std::atomic<bool> openFlag_{false};
  std::atomic<bool> closeFlag_{false};

  bool check_state_ios{false};

private:
  /**
   * @brief Finds and sets a command value.
   * @param name The name of the command.
   * @param value The value to set.
   * @return True if the command was found and set, false otherwise.
   */
  bool find_and_set_command(const std::string & name, const double value);

  /**
   * @brief Finds and gets a state value.
   * @param name The name of the state.
   * @param value The value to get.
   * @return True if the state was found and retrieved, false otherwise.
   */
  bool find_and_get_state(const std::string & name, double & value);

  /**
   * @brief Finds and gets a command value.
   * @param name The name of the command.
   * @param value The value to get.
   * @return True if the command was found and retrieved, false otherwise.
   */
  bool find_and_get_command(const std::string & name, double & value);

  /**
   * @brief Handles the state transition when opening the gripper.
   * @param state The current state of the gripper.
   */
  void handle_gripper_state_transition(
    const rclcpp::Time & current_time, const GripperTransitionIOs & ios, const uint & state,
    const std::string & transition_name, std::vector<double> after_joint_states);

  /**
   * @brief Handles the state transition when closing the gripper.
   * @param state The current state of the gripper.
   */
  void handle_gripper_state_transition_close(const gripper_state_type & state);

  /**
   * @brief Handles the state transition for reconfiguration.
   * @param state The current reconfiguration state.
   */
  void handle_reconfigure_state_transition(const reconfigure_state_type & state);

  /**
   * @brief Checks the parameters of the controller.
   * @return CallbackReturn indicating success or failure.
   */
  controller_interface::CallbackReturn check_parameters();

  /**
   * @brief Prepares the command and state IOs.
   */
  void prepare_command_and_state_ios();

  /**
   * @brief Prepares the publishers and services.
   * @return CallbackReturn indicating success or failure.
   */
  controller_interface::CallbackReturn prepare_publishers_and_services();

  /**
   * @brief Publishes the gripper joint states.
   */
  void publish_gripper_joint_states();

  /**
   * @brief Publishes the dynamic interface values.
   */
  void publish_dynamic_interface_values();

  /**
   * @brief Publishes the reconfigure gripper joint states.
   */
  void publish_reconfigure_gripper_joint_states();

  /**
   * @brief Checks the gripper and reconfigure state.
   */
  void check_gripper_and_reconfigure_state();

  bool set_commands(
    const std::unordered_map<std::string, double> & command_states,
    const std::string & transition_name);
  bool check_states(
    const std::unordered_map<std::string, double> & state_ios, const std::string & transition_name);

  std::vector<std::string> configurations_list_;
  std::vector<io_gripper_controller::Params::ConfigurationSetup::MapConfigurations> config_map_;
  std::vector<io_gripper_controller::Params::SensorsInterfaces::MapGripperSpecificSensors>
    sensors_map_;
  double state_value_;
  std::string configuration_key_;
  bool check_state_ios_;
  std::string closed_state_name_;
  io_gripper_controller::Params::Close::State::MapPossibleClosedStates closed_state_values_;
  io_gripper_controller::Params::ConfigurationSetup::MapConfigurations conf_it_;
  std::vector<std::string>::iterator config_index_;
  rclcpp::CallbackGroup::SharedPtr open_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr close_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr reconfigure_service_callback_group_;
  std::shared_ptr<control_msgs::action::IOGripperCommand_Feedback> gripper_feedback_;
  std::shared_ptr<control_msgs::action::IOGripperCommand_Result> gripper_result_;
  std::shared_ptr<control_msgs::action::SetIOGripperConfig_Feedback> gripper_config_feedback_;
  std::shared_ptr<control_msgs::action::SetIOGripperConfig_Result> gripper_config_result_;

  /**
   * @brief Handles the goal for the gripper action.
   * @param uuid The UUID of the goal.
   * @param goal The goal to handle.
   * @return GoalResponse indicating acceptance or rejection of the goal.
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GripperAction::Goal> goal);

  /**
   * @brief Handles the cancellation of the gripper action.
   * @param goal_handle The handle of the goal to cancel.
   * @return CancelResponse indicating acceptance or rejection of the cancellation.
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGripper> goal_handle);

  /**
   * @brief Handles the acceptance of the gripper action.
   * @param goal_handle The handle of the accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);

  /**
   * @brief Executes the gripper action.
   * @param goal_handle The handle of the goal to execute.
   */
  void execute(const std::shared_ptr<GoalHandleGripper> goal_handle);

  /**
   * @brief Handles the goal for the gripper configuration action.
   * @param uuid The UUID of the goal.
   * @param goal The goal to handle.
   * @return GoalResponse indicating acceptance or rejection of the goal.
   */
  rclcpp_action::GoalResponse config_handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GripperConfigAction::Goal> goal);

  /**
   * @brief Handles the cancellation of the gripper configuration action.
   * @param goal_handle The handle of the goal to cancel.
   * @return CancelResponse indicating acceptance or rejection of the cancellation.
   */
  rclcpp_action::CancelResponse config_handle_cancel(
    const std::shared_ptr<GoalHandleGripperConfig> goal_handle);

  /**
   * @brief Handles the acceptance of the gripper configuration action.
   * @param goal_handle The handle of the accepted goal.
   */
  void config_handle_accepted(const std::shared_ptr<GoalHandleGripperConfig> goal_handle);

  /**
   * @brief Executes the gripper configuration action.
   * @param goal_handle The handle of the goal to execute.
   */
  void config_execute(const std::shared_ptr<GoalHandleGripperConfig> goal_handle);
};

}  // namespace io_gripper_controller

#endif  // IO_GRIPPER_CONTROLLER__IO_GRIPPER_CONTROLLER_HPP_
