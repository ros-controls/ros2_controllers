// Copyright (c) 2021, PickNik, Inc.
// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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


#ifndef FORWARD_COMMAND_CONTROLLER__MULTI_INTERFACE_FORWARD_CONTROLLER_HPP_
#define FORWARD_COMMAND_CONTROLLER__MULTI_INTERFACE_FORWARD_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "forward_command_controller/forward_command_controller.hpp"
#include "forward_command_controller/visibility_control.h"

namespace forward_command_controller
{

class MultiInterfaceForwardController : public forward_command_controller::ForwardCommandController
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  MultiInterfaceForwardController();

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

protected:
  CallbackReturn read_parameters() override;

  std::vector<std::string> joints_interfaces_;
};

}  // namespace forward_command_controller

#endif  // FORWARD_COMMAND_CONTROLLER__MULTI_INTERFACE_FORWARD_CONTROLLER_HPP_
