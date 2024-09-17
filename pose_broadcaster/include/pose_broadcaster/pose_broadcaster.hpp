// Copyright 2024 FZI Forschungszentrum Informatik
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
#ifndef POSE_BROADCASTER__POSE_BROADCASTER_HPP_
#define POSE_BROADCASTER__POSE_BROADCASTER_HPP_

#include <array>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "pose_broadcaster/visibility_control.h"
#include "pose_broadcaster_parameters.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace pose_broadcaster
{

class PoseBroadcaster : public controller_interface::ControllerInterface
{
public:
  POSE_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  POSE_BROADCASTER_PUBLIC controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  POSE_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

  POSE_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  POSE_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  POSE_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  POSE_BROADCASTER_PUBLIC controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::array<std::string, 6> interface_names_;
};

}  // namespace pose_broadcaster

#endif  // POSE_BROADCASTER__POSE_BROADCASTER_HPP_
