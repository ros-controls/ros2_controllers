// Copyright 2020 PAL Robotics S.L.
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

#ifndef POSITION_CONTROLLERS__POSITION_CONTROLLERS_HPP_
#define POSITION_CONTROLLERS__POSITION_CONTROLLERS_HPP_

#include "forward_command_controller/forward_command_controller.hpp"
#include "position_controllers/visibility_control.h"

namespace position_controllers
{
class JointPositionController : public forward_command_controller::ForwardCommandController
{
public:
  POSITION_CONTROLLERS_PUBLIC
  JointPositionController();
};

}  // namespace position_controllers

#endif  // POSITION_CONTROLLERS__POSITION_CONTROLLERS_HPP_
