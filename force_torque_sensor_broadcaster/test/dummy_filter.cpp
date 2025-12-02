
// Copyright (c) 2025, PAL Robotics
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

/*
 * Authors: Oscar Martinez
 */

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "filters/increment.hpp"

namespace filters
{

template <>
bool IncrementFilter<geometry_msgs::msg::WrenchStamped>::update(
  const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
{
  if (!this->configured_)
  {
    throw std::runtime_error("Filter is not configured");
  }

  // Just increment every value
  data_out.wrench.force.x = data_in.wrench.force.x + 1;
  data_out.wrench.force.y = data_in.wrench.force.y + 1;
  data_out.wrench.force.z = data_in.wrench.force.z + 1;
  data_out.wrench.torque.x = data_in.wrench.torque.x + 1;
  data_out.wrench.torque.y = data_in.wrench.torque.y + 1;
  data_out.wrench.torque.z = data_in.wrench.torque.z + 1;

  return true;
}

}  // namespace filters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  filters::IncrementFilter<geometry_msgs::msg::WrenchStamped>,
  filters::FilterBase<geometry_msgs::msg::WrenchStamped>)
