// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
// Authors: Daniel Azanov, Dr. Denis
//

#ifndef PID_CONTROLLER__PID_CONTROLLER_HPP_
#define PID_CONTROLLER__PID_CONTROLLER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "control_toolbox/pid_ros.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "pid_controller/pid_controller_parameters.hpp"

namespace pid_controller
{

enum class feedforward_mode_type : std::uint8_t
{
  OFF = 0,
  ON = 1,
};

class PidController : public controller_interface::ChainableControllerInterface
{
public:
  PidController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerReferenceMsg = control_msgs::msg::MultiDOFCommand;
  using ControllerMeasuredStateMsg = control_msgs::msg::MultiDOFCommand;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::MultiDOFStateStamped;

protected:
  std::shared_ptr<pid_controller::ParamListener> param_listener_;
  pid_controller::Params params_;

  std::vector<std::string> reference_and_state_dof_names_;
  size_t dof_;
  std::vector<double> measured_state_values_;

  using PidPtr = std::shared_ptr<control_toolbox::PidROS>;
  std::vector<PidPtr> pids_;
  // Feed-forward velocity weight factor when calculating closed loop pid adapter's command
  std::vector<double> feedforward_gain_;

  double reset_pid_time_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  rclcpp::Subscription<ControllerMeasuredStateMsg>::SharedPtr measured_state_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerMeasuredStateMsg>> measured_state_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_feedforward_control_service_;
  realtime_tools::RealtimeBuffer<feedforward_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  // internal methods
  void update_parameters();
  controller_interface::CallbackReturn configure_parameters();

  template <typename T>
  inline bool is_zero(T value, T tolerance = std::numeric_limits<T>::epsilon())
  {
    return std::abs(value) <= tolerance;
  }

private:
  // callback for topic interface
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace pid_controller

#endif  // PID_CONTROLLER__PID_CONTROLLER_HPP_
