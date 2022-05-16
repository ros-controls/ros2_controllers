// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#ifndef FORWARD_COMMAND_CONTROLLER__MULTI_INTERFACE_FORWARD_COMMAND_CONTROLLER_HPP_
#define FORWARD_COMMAND_CONTROLLER__MULTI_INTERFACE_FORWARD_COMMAND_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "forward_command_controller/chainable_forward_controller.hpp"
#include "forward_command_controller/forward_controller.hpp"
#include "forward_command_controller/forward_controllers_base.hpp"
#include "forward_command_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace forward_command_controller
{
/**
 * \brief Multi interface forward command controller for a set of interfaces.
 *
 * This class forwards the command signal down to a set of interfaces on the specified joint.
 *
 * \param joint Name of the joint to control.
 * \param interface_names Names of the interfaces to command.
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
class BaseMultiInterfaceForwardCommandController : public T
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  BaseMultiInterfaceForwardCommandController() : T() {}

protected:
  void declare_parameters() override
  {
    controller_interface::ControllerInterfaceBase::auto_declare<std::string>("joint", joint_name_);
    controller_interface::ControllerInterfaceBase::auto_declare<std::vector<std::string>>(
      "interface_names", interface_names_);
  };

  controller_interface::CallbackReturn read_parameters() override
  {
    joint_name_ = T::get_node()->get_parameter("joint").as_string();
    interface_names_ = T::get_node()->get_parameter("interface_names").as_string_array();

    if (joint_name_.empty())
    {
      RCLCPP_ERROR(T::get_node()->get_logger(), "'joint' parameter is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (interface_names_.empty())
    {
      RCLCPP_ERROR(T::get_node()->get_logger(), "'interfaces' parameter is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    for (const auto & interface : interface_names_)

    {
      T::command_interface_names_.push_back(joint_name_ + "/" + interface);
    }

    return controller_interface::CallbackReturn::SUCCESS;
  };

  std::string joint_name_;
  std::vector<std::string> interface_names_;
};

class MultiInterfaceForwardCommandController
: public BaseMultiInterfaceForwardCommandController<ForwardController>
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  MultiInterfaceForwardCommandController()
  : BaseMultiInterfaceForwardCommandController<ForwardController>()
  {
  }
};

class ChainableMultiInterfaceForwardCommandController
: public BaseMultiInterfaceForwardCommandController<ChainableForwardController>
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  ChainableMultiInterfaceForwardCommandController()
  : BaseMultiInterfaceForwardCommandController<ChainableForwardController>()
  {
  }
};

}  // namespace forward_command_controller

#endif  // FORWARD_COMMAND_CONTROLLER__MULTI_INTERFACE_FORWARD_COMMAND_CONTROLLER_HPP_
