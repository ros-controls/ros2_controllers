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

#ifndef FORWARD_COMMAND_CONTROLLER__FORWARD_COMMAND_CONTROLLER_HPP_
#define FORWARD_COMMAND_CONTROLLER__FORWARD_COMMAND_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "forward_command_controller/chainable_forward_controller.hpp"
#include "forward_command_controller/forward_controller.hpp"
#include "forward_command_controller/forward_controllers_base.hpp"
#include "forward_command_controller/visibility_control.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace forward_command_controller
{
/**
 * \brief Forward command controller for a set of joints.
 *
 * This class forwards the command signal down to a set of joints on the specified interface.
 *
 * \param joints Names of the joints to control.
 * \param interface_name Name of the interface to command.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 */
template <
  typename T,
  typename std::enable_if<
    std::is_convertible<T *, forward_command_controller::ForwardControllersBase *>::value,
    T>::type * = nullptr,
  typename std::enable_if<
    std::is_convertible<T *, controller_interface::ControllerInterfaceBase *>::value, T>::type * =
    nullptr>
class BaseForwardCommandController : public T
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  BaseForwardCommandController() : T() {}

protected:
  void declare_parameters() override
  {
    controller_interface::ControllerInterfaceBase::auto_declare<std::vector<std::string>>(
      "joints", std::vector<std::string>());
    controller_interface::ControllerInterfaceBase::auto_declare<std::string>("interface_name", "");
  };

  controller_interface::CallbackReturn read_parameters() override
  {
    joint_names_ = T::get_node()->get_parameter("joints").as_string_array();

    if (joint_names_.empty())
    {
      RCLCPP_ERROR(T::get_node()->get_logger(), "'joints' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    // Specialized, child controllers set interfaces before calling configure function.
    if (interface_name_.empty())
    {
      interface_name_ = T::get_node()->get_parameter("interface_name").as_string();
    }

    if (interface_name_.empty())
    {
      RCLCPP_ERROR(T::get_node()->get_logger(), "'interface_name' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    for (const auto & joint : joint_names_)
    {
      T::command_interface_names_.push_back(joint + "/" + interface_name_);
    }

    return controller_interface::CallbackReturn::SUCCESS;
  };

  std::vector<std::string> joint_names_;
  std::string interface_name_;
};

class ForwardCommandController : public BaseForwardCommandController<ForwardController>
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  ForwardCommandController() : BaseForwardCommandController<ForwardController>() {}
};

class ChainableForwardCommandController
: public BaseForwardCommandController<ChainableForwardController>
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  ChainableForwardCommandController() : BaseForwardCommandController<ChainableForwardController>()
  {
  }
};

}  // namespace forward_command_controller

#endif  // FORWARD_COMMAND_CONTROLLER__FORWARD_COMMAND_CONTROLLER_HPP_
