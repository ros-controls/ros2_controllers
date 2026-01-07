// Copyright (c) 2025, b-robotized
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

#ifndef VDA5050_SAFETY_STATE_BROADCASTER__VDA5050_SAFETY_STATE_BROADCASTER_HPP_
#define VDA5050_SAFETY_STATE_BROADCASTER__VDA5050_SAFETY_STATE_BROADCASTER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include <vda5050_safety_state_broadcaster/vda5050_safety_state_broadcaster_parameters.hpp>
#include "control_msgs/msg/vda5050_safety_state.hpp"

namespace vda5050_safety_state_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
const size_t MAX_LENGTH = 64;  // maximum length of strings to reserve

/**
 * \brief VDA5050 safety state broadcaster for all or some state in a ros2_control system.
 *
 *
 */
class Vda5050SafetyStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  Vda5050SafetyStateBroadcaster();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

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
  vda5050_safety_state_broadcaster::Params params_;

  std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::VDA5050SafetyState>>
    realtime_vda5050_safety_state_publisher_;

private:
  std::shared_ptr<vda5050_safety_state_broadcaster::ParamListener> param_listener_;
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::VDA5050SafetyState>>
    vda5050_safety_state_publisher_;
  control_msgs::msg::VDA5050SafetyState safety_state_msg;

  /**
   * @brief Determines the current E-stop state based on the state interfaces.
   * @return The E-stop type as defined in control_msgs::msg::VDA5050SafetyState.
   */
  control_msgs::msg::VDA5050SafetyState::_e_stop_type determineEstopState();

  struct InterfaceIds
  {
    size_t manual_start = 0;
    size_t remote_start = 0;
    size_t autoack_start = 0;
    size_t total_interfaces = 0;
  };

  InterfaceIds itfs_ids_;
  bool fieldViolation_value = false;
  std::string estop_msg = control_msgs::msg::VDA5050SafetyState::NONE;

  bool get_bool_itf_value(const hardware_interface::LoanedStateInterface & state_itf)
  {
    auto data_type = state_itf.get_data_type();

    if (data_type == hardware_interface::HandleDataType::BOOL)
    {
      return state_itf.get_optional<bool>().value_or(false);
    }

    return safe_double_to_bool(state_itf.get_optional<double>().value_or(kUninitializedValue));
  }

  /**
   * @brief Safely converts a double value to bool, treating NaN as false.
   * @param value The double value to convert.
   * @return true if value is not NaN and not zero, false otherwise.
   */
  bool safe_double_to_bool(double value) const
  {
    if (std::isnan(value))
    {
      return false;
    }
    return value != 0.0;
  }
};

}  // namespace vda5050_safety_state_broadcaster

#endif  // VDA5050_SAFETY_STATE_BROADCASTER__VDA5050_SAFETY_STATE_BROADCASTER_HPP_
