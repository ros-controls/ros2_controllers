// Copyright (c) 2025, b»robotized Group
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

#ifndef gpio_tool_controller__gpio_tool_controller_HPP_
#define gpio_tool_controller__gpio_tool_controller_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <unordered_map>
#include <unordered_set>
#include "control_msgs/action/gpio_tool_command.hpp"
#include "control_msgs/action/set_gpio_tool_config.hpp"
#include "control_msgs/msg/dynamic_interface_values.hpp"
#include "control_msgs/msg/interface_value.hpp"
#include "control_msgs/msg/gpio_tool_controller_state.hpp"
#include "control_msgs/msg/gpio_tool_transition.hpp"
#include "control_msgs/srv/set_gpio_tool_config.hpp"
#include "controller_interface/controller_interface.hpp"
#include "gpio_controllers/gpio_tool_controller_parameters.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace gpio_tool_controller
{
/**
 * @enum service_mode_type
 * @brief Represents the various service modes of the tool. These modes represents the high level
 * states of the tool.
 *
 * - IDLE: The tool is in an idle state, not performing any action.
 * - DISENGAGING: The tool is in the process of opening.
 * - ENGAGING: The tool is in the process of closing.
 * - RECONFIGURING: The tool is in the process of reconfiguring to a new state.
 */
enum class ToolAction : std::uint8_t
{
  IDLE = 0,
  DISENGAGING = 1,
  ENGAGING = 2,
  RECONFIGURING = 3,
  CANCELING = 4,
};

class GpioToolController : public controller_interface::ControllerInterface
/**
 * @brief GpioToolController class handles the control of an IO-based tools, like grippers, lifts, mode control.
 */
{
public:
  /**
   * @brief Constructor for GpioToolController.
   */
  GpioToolController();

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

protected:
  gpio_tool_controller::Params params_;

  struct ToolTransitionIOs
  {
    std::vector<std::string> possible_states;

    std::unordered_map<std::string, std::unordered_map<std::string, std::pair<double, size_t>>> set_before_commands;
    std::unordered_map<std::string, std::unordered_map<std::string, std::pair<double, size_t>>> set_before_states;

    std::unordered_map<std::string, std::unordered_map<std::string, std::pair<double, size_t>>> commands;

    std::unordered_map<std::string, std::unordered_map<std::string, std::pair<double, size_t>>> states;
    std::unordered_map<std::string, std::vector<double>> states_joint_states;
    std::unordered_map<std::string, std::unordered_map<std::string, std::pair<double, size_t>>> set_after_commands;
    std::unordered_map<std::string, std::unordered_map<std::string, std::pair<double, size_t>>> set_after_states;
  };

  ToolTransitionIOs disengaged_gpios_;
  ToolTransitionIOs engaged_gpios_;
  ToolTransitionIOs reconfigure_gpios_;

  // Not needed to be atomic or protected as used only in the RT loop
  rclcpp::Time state_change_start_;
  std::string current_configuration_;
  std::string current_state_;

  std::unordered_set<std::string> command_if_ios_;
  std::unordered_set<std::string> state_if_ios_;

  using EngagingSrvType = std_srvs::srv::Trigger;
  using ResetSrvType = std_srvs::srv::Trigger;
  using ConfigSrvType = control_msgs::srv::SetGPIOToolConfig;
  using DynInterfaceMsg = control_msgs::msg::DynamicInterfaceValues;
  using EngagingActionType = control_msgs::action::GPIOToolCommand;
  using ConfigActionType = control_msgs::action::SetGPIOToolConfig;
  using GPIOToolTransition = control_msgs::msg::GPIOToolTransition;
  using ControllerStateMsg = control_msgs::msg::GPIOToolControllerState;

  std::shared_ptr<gpio_tool_controller::ParamListener> param_listener_;
  rclcpp::Service<EngagingSrvType>::SharedPtr disengaged_service_;
  rclcpp::Service<EngagingSrvType>::SharedPtr engaged_service_;
  rclcpp::Service<ConfigSrvType>::SharedPtr reconfigure_tool_service_;
  rclcpp_action::Server<EngagingActionType>::SharedPtr engaging_action_server_;
  rclcpp_action::Server<ConfigActionType>::SharedPtr config_action_server_;
  rclcpp::Service<ResetSrvType>::SharedPtr reset_service_;

  // Store current action tool is executing
  std::atomic<ToolAction> current_tool_action_{ToolAction::IDLE};
  std::atomic<uint8_t> current_tool_transition_{GPIOToolTransition::IDLE};
  std::atomic<bool> reset_halted_{false};
  std::atomic<bool> transition_time_updated_{false};
  std::atomic<std::shared_ptr<std::string>> target_configuration_;

  using ToolJointStatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr t_js_publisher_;
  std::unique_ptr<ToolJointStatePublisher> tool_joint_state_publisher_;
  std::vector<double> joint_states_values_;
  using InterfacePublisher = realtime_tools::RealtimePublisher<DynInterfaceMsg>;
  rclcpp::Publisher<DynInterfaceMsg>::SharedPtr if_publisher_;
  std::unique_ptr<InterfacePublisher> interface_publisher_;
  using ControllerStatePublisher =
    realtime_tools::RealtimePublisher<ControllerStateMsg>;
  rclcpp::Publisher<ControllerStateMsg>::SharedPtr t_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;

  EngagingSrvType::Response process_engaging_request(
    const ToolAction & requested_action, const std::string & requested_action_name);

  EngagingSrvType::Response process_reconfigure_request(const std::string & config_name);

  EngagingSrvType::Response service_wait_for_transition_end(
    const std::string & requested_action_name);

private:
  bool configuration_control_enabled_ = true;
  bool joint_states_need_publishing_ = true;

  /**
   * @brief Handles the state transition when enaging the tool.
   * @param state The current transition of the tool.
   */
  void handle_tool_state_transition(
    const rclcpp::Time & current_time, const ToolTransitionIOs & ios,
    const std::string & target_state, std::vector<double> & joint_states,
    const size_t joint_states_start_index, std::string & end_state);

  void check_tool_state_and_switch(
    const rclcpp::Time & current_time, const ToolTransitionIOs & ios,
    std::vector<double> & joint_states, const size_t joint_states_start_index,
    const std::string & output_prefix, const uint8_t next_transition,
    std::string & found_state_name, const bool warning_output = false);

  /**
   * @brief Prepares the command and state IOs.
   * \returns true if successful, false otherwise. Check the output if error has happend.
   */
  bool prepare_command_and_state_ios();

  /**
   * @brief Prepares the publishers and services.
   * @return CallbackReturn indicating success or failure.
   */
  controller_interface::CallbackReturn prepare_publishers_and_services();

  /**
   * @brief Publishes the the values from the RT loop.
   */
  void publish_topics(const rclcpp::Time & current_time);

  /**
   * @brief Checks the tools state.
   */
  void check_tool_state(const rclcpp::Time & current_time, const bool warning_output = false);

  bool set_commands(
    const std::unordered_map<std::string, std::pair<double, size_t>> & commands,
    const std::string & output_prefix,
    const uint8_t next_transition);
  bool check_states(
    const rclcpp::Time & current_time, const std::unordered_map<std::string, std::pair<double, size_t>> & states, const std::string & output_prefix, const uint8_t next_transition, const bool warning_output = false);

  std::vector<std::string> configurations_list_;
  std::vector<gpio_tool_controller::Params::ConfigurationSetup::MapConfigurations> config_map_;
  double state_value_;
  std::string configuration_key_;
  bool check_state_ios_;
  std::string closed_state_name_;
  std::vector<std::string>::iterator config_index_;
  rclcpp::CallbackGroup::SharedPtr disengaging_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr engaging_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr reconfigure_service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr reset_service_callback_group_;

  /**
   * @brief Handles the goal for the tool action.
   * @param uuid The UUID of the goal.
   * @param goal The goal to handle.
   * @return GoalResponse indicating acceptance or rejection of the goal.
   */
  rclcpp_action::GoalResponse handle_engaging_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const EngagingActionType::Goal>  goal);

  /**
   * @brief Handles the cancellation of the tool action.
   * @param goal_handle The handle of the goal to cancel.
   * @return CancelResponse indicating acceptance or rejection of the cancellation.
   */
  rclcpp_action::CancelResponse handle_engaging_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<EngagingActionType>> goal_handle);

  /**
   * @brief Handles the goal for the gripper configuration action.
   * @param uuid The UUID of the goal.
   * @param goal The goal to handle.
   * @return GoalResponse indicating acceptance or rejection of the goal.
   */
  rclcpp_action::GoalResponse handle_config_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ConfigActionType::Goal> goal);

  /**
   * @brief Handles the cancellation of the gripper configuration action.
   * @param goal_handle The handle of the goal to cancel.
   * @return CancelResponse indicating acceptance or rejection of the cancellation.
   */
  rclcpp_action::CancelResponse handle_config_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ConfigActionType>> goal_handle);

  /**
   * @brief Handles the accepted goal for the tool state and configuration changes.
   * @param goal_handle The handle of the accepted goal.
   */
  template <typename ActionT>
  void handle_action_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle);
};

}  // namespace gpio_tool_controller

#endif  // gpio_tool_controller__gpio_tool_controller_HPP_
