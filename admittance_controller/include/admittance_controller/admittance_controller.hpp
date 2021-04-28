// Copyright (c) 2021, PickNik, Inc.
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
/// \author: Denis Stogl

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include "admittance_controller/incremental_ik_calculator.hpp"
#include "admittance_controller/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace admittance_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AdmittanceController : public controller_interface::ControllerInterface
{
public:
  ADMITTANCE_CONTROLLER_PUBLIC
  AdmittanceController();

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> interface_names_;

  std::shared_ptr<IncrementalIKCalculator> ik_;

  Eigen::VectorXd delta_x_;
  Eigen::VectorXd delta_theta_;
  // TODO(andyz): parameterize this
  std::string force_torque_sensor_frame_;
};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
