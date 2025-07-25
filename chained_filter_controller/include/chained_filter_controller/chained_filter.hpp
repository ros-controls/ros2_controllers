// Copyright 2025 ros2_control PMC
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

#ifndef CHAINED_FILTER_CONTROLLER__CHAINED_FILTER_HPP_
#define CHAINED_FILTER_CONTROLLER__CHAINED_FILTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "filters/filter_chain.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "chained_filter_controller/chained_filter_parameters.hpp"

namespace chained_filter_controller
{

class ChainedFilter : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::shared_ptr<chained_filter::ParamListener> param_listener_;
  chained_filter::Params params_;

  std::vector<std::unique_ptr<filters::FilterChain<double>>> filters_;
  std::vector<double> output_state_values_;
};
}  // namespace chained_filter_controller
#endif  // CHAINED_FILTER_CONTROLLER__CHAINED_FILTER_HPP_
