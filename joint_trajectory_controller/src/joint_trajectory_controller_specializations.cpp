// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller_specializations.hpp"

namespace position_controllers
{
JointTrajectoryController::JointTrajectoryController()
: joint_trajectory_controller::JointTrajectoryController()
{
  command_interface_types_ = {hardware_interface::HW_IF_POSITION};
}

controller_interface::return_type
JointTrajectoryController::init(const std::string & controller_name)
{
  auto ret = joint_trajectory_controller::JointTrajectoryController::init(controller_name);
  if (ret != controller_interface::return_type::SUCCESS) {
    return ret;
  }

  try {
    // undeclare command_interfaces parameter used in the general joint_trajectory_controller
    get_node()->undeclare_parameter("command_interfaces");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::SUCCESS;
}

}  // namespace position_controllers


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  position_controllers::JointTrajectoryController, controller_interface::ControllerInterface)
