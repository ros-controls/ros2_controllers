// Copyright 2021 PAL Robotics SL.
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

#ifndef SEMANTIC_COMPONENTS__RANGE_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__RANGE_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace semantic_components
{
class RangeSensor : public SemanticComponentInterface<sensor_msgs::msg::Range>
{
public:
  explicit RangeSensor(const std::string & name) : SemanticComponentInterface(name, 5)
  {
    interface_names_.emplace_back(name_ + "/" + "radiation_type");
    interface_names_.emplace_back(name_ + "/" + "field_of_view");
    interface_names_.emplace_back(name_ + "/" + "min_range");
    interface_names_.emplace_back(name_ + "/" + "max_range");
    interface_names_.emplace_back(name_ + "/" + "range");
    
    // Set default values to NaN
    radiation_type_ = std::numeric_limits<uint8_t>::quiet_NaN();
    field_of_view_ = std::numeric_limits<float>::quiet_NaN();
    min_range_ = std::numeric_limits<float>::quiet_NaN();
    max_range_ = std::numeric_limits<float>::quiet_NaN();
    range_ = std::numeric_limits<float>::quiet_NaN();
  }

  virtual ~RangeSensor() = default;

  /// Return angle_min
  /**
   * Return minimum angle reported by an Range
   *
   * \return minimum angle
   */
  uint8_t get_radiation_type()
  {
    size_t interface_offset = 0;
    radiation_type_ = state_interfaces_[interface_offset].get().get_int_value();
    return radiation_type_;
  }

  /// Return angle_max
  /**
   * Return maximum angle reported by an Range
   *
   * \return maximum angle
   */
  float get_field_of_view ()
  {
    size_t interface_offset = 1;
    field_of_view_ = state_interfaces_[interface_offset].get().get_value();
    return field_of_view_;
  }

  /// Return angle_increment
  /**
   * Return increment angle reported by an Range
   *
   * \return angle increment
   */
  float get_min_range()
  {
    size_t interface_offset = 2;
    min_range_ = state_interfaces_[interface_offset].get().get_value();
    return min_range_;
  }

  /// Return time_increment
  /**
   * Return time increment reported by an Range
   *
   * \return time increment
   */
  float get_max_range()
  {
    size_t interface_offset = 3;
    max_range_ = state_interfaces_[interface_offset].get().get_value();
    return max_range_;
  }

  /// Return scan_time
  /**
   * Return scan time reported by an Range
   *
   * \return scan time
   */
  float get_range()
  {
    size_t interface_offset = 4;
    range_ = state_interfaces_[interface_offset].get().get_value();
    return range_;
  }

  /// Return range_min
  /**
   * Return minimum range reported by an Range
   *
   * \return minimum range
   */

  /**
   * Return maximum range reported by an Range
   *
   * \return maximum range
   */
  
  /// Return Camera message with height and width
  /**
   * Constructs and return a Camera message from the current values.
   * \return Camera message from values;
   */
  bool get_values_as_message(sensor_msgs::msg::Range & message)
  {
    // call functions to update with the latest values
    get_radiation_type();
    get_field_of_view();
    get_min_range();
    get_max_range();
    get_range();

    // update the message values
    message.radiation_type = radiation_type_;
    message.field_of_view = field_of_view_;
    message.min_range = min_range_;
    message.max_range = max_range_;
    message.range = range_;

    return true;
  }

protected:
  // Order is: radiation_type, field_of_view, min_range, max_range, range
  uint8_t radiation_type_;
  float field_of_view_;
  float min_range_;
  float max_range_;
  float range_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__CAMERA_SENSOR_HPP_
