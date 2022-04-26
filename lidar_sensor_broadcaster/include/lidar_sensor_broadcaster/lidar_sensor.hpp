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

#ifndef SEMANTIC_COMPONENTS__LIDAR_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__LIDAR_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace semantic_components
{
class LidarSensor : public SemanticComponentInterface<sensor_msgs::msg::LaserScan>
{
public:
  explicit LidarSensor(const std::string & name) : SemanticComponentInterface(name, 9)
  {
    interface_names_.emplace_back(name_ + "/" + "angle_min");
    interface_names_.emplace_back(name_ + "/" + "angle_max");
    interface_names_.emplace_back(name_ + "/" + "angle_increment");
    interface_names_.emplace_back(name_ + "/" + "time_increment");
    interface_names_.emplace_back(name_ + "/" + "scan_time");
    interface_names_.emplace_back(name_ + "/" + "range_min");
    interface_names_.emplace_back(name_ + "/" + "range_max");
    interface_names_.emplace_back(name_ + "/" + "ranges_size");
    interface_names_.emplace_back(name_ + "/" + "intensities_size");
    interface_names_.emplace_back(name_ + "/" + "ranges");
    interface_names_.emplace_back(name_ + "/" + "intensities");
    
    // Set default values to NaN
    angle_min_ = std::numeric_limits<float>::quiet_NaN();
    angle_max_ = std::numeric_limits<float>::quiet_NaN();
    angle_increment_ = std::numeric_limits<float>::quiet_NaN();
    time_increment_ = std::numeric_limits<float>::quiet_NaN();
    scan_time_ = std::numeric_limits<float>::quiet_NaN();
    range_min_ = std::numeric_limits<float>::quiet_NaN();
    range_max_ = std::numeric_limits<float>::quiet_NaN();

    //ranges_init_ = std::numeric_limits<float>::quiet_NaN();
    ranges_size_ = std::numeric_limits<float>::quiet_NaN();
    //intensities_init_ = std::numeric_limits<float>::quiet_NaN();
    intensities_size_ = std::numeric_limits<float>::quiet_NaN();
    //ranges_.fill(ranges_.begin(), ranges_.end(), std::numeric_limits<float>::quiet_NaN());
    //intensities_.fill(intensities_.begin(), intensities_.end(), std::numeric_limits<float>::quiet_NaN());
  }

  virtual ~LidarSensor() = default;

  /// Return angle_min
  /**
   * Return minimum angle reported by an LaserScan
   *
   * \return minimum angle
   */
  float get_min_angle()
  {
    size_t interface_offset = 0;
    angle_min_ = state_interfaces_[interface_offset].get().get_value();
    return angle_min_;
  }

  /// Return angle_max
  /**
   * Return maximum angle reported by an LaserScan
   *
   * \return maximum angle
   */
  float get_max_angle()
  {
    size_t interface_offset = 1;
    angle_max_ = state_interfaces_[interface_offset].get().get_value();
    return angle_max_;
  }

  /// Return angle_increment
  /**
   * Return increment angle reported by an LaserScan
   *
   * \return angle increment
   */
  float get_angle_increment()
  {
    size_t interface_offset = 2;
    angle_increment_ = state_interfaces_[interface_offset].get().get_value();
    return angle_increment_;
  }

  /// Return time_increment
  /**
   * Return time increment reported by an LaserScan
   *
   * \return time increment
   */
  float get_time_increment()
  {
    size_t interface_offset = 3;
    time_increment_ = state_interfaces_[interface_offset].get().get_value();
    return time_increment_;
  }

  /// Return scan_time
  /**
   * Return scan time reported by an LaserScan
   *
   * \return scan time
   */
  float get_scan_time()
  {
    size_t interface_offset = 4;
    scan_time_ = state_interfaces_[interface_offset].get().get_value();
    return scan_time_;
  }

  /// Return range_min
  /**
   * Return minimum range reported by an LaserScan
   *
   * \return minimum range
   */
  float get_min_range()
  {
    size_t interface_offset = 5;
    range_min_ = state_interfaces_[interface_offset].get().get_value();
    return range_min_;
  }

  /// Return range_max
  /**
   * Return maximum range reported by an LaserScan
   *
   * \return maximum range
   */
  float get_max_range()
  {
    size_t interface_offset = 6;
    range_max_ = state_interfaces_[interface_offset].get().get_value();
    return range_max_;
  }

  /// Return ranges_size
  /**
   * Return ranges_size
   *
   * \return ranges_size
   */
  float get_ranges_size()
  {
    size_t interface_offset = 7;
    ranges_size_ = state_interfaces_[interface_offset].get().get_value();
    return ranges_size_;
  }

  /// Return intensities_size
  /**
   * Return intensities_size
   *
   * \return intensities_size
   */
  float get_intensities_size()
  {
    size_t interface_offset = 8;
    intensities_size_ = state_interfaces_[interface_offset].get().get_value();
    return intensities_size_;
  }

  /// Return ranges_init
  /**
   * Return ranges_init
   *
   * \return ranges_init
   */
  std::vector<float> get_ranges()
  {
    size_t interface_offset = 9;
    auto arrayData = state_interfaces_[interface_offset].get().get_array_value();
    ranges_.assign(arrayData.begin(), arrayData.end());
    return ranges_;
  }

  /// Return intensities_init
  /**
   * Return intensities_init
   *
   * \return intensities_init
   */
  std::vector<float> get_intensities()
  {
    size_t interface_offset = 10;
    auto arrayData = state_interfaces_[interface_offset].get().get_array_value();
    intensities_.assign(arrayData.begin(), arrayData.end());
    return intensities_;
  }

  // /// Return orientation.
  // /**
  //  * Return orientation reported by an Laser
  //  *
  //  * \return array of size 4 with orientation quaternion (x,y,z,w)
  //  */
  // std::array<double, 4> get_height()
  // {
  //   size_t interface_offset = 0;
  //   for (size_t i = 0; i < orientation_.size(); ++i)
  //   {
  //     orientation_[i] = state_interfaces_[interface_offset + i].get().get_value();
  //   }
  //   return orientation_;
  // }

  void update_ranges(std::vector<float> ranges){
    std::copy(ranges.begin(), ranges.end(), std::back_inserter(ranges_));
  }

  void update_intensities(std::vector<float> it){
    std::copy(it.begin(), it.end(), std::back_inserter(intensities_));
  }


  /// Return Camera message with height and width
  /**
   * Constructs and return a Camera message from the current values.
   * \return Camera message from values;
   */
  bool get_values_as_message(sensor_msgs::msg::LaserScan & message)
  {
    // call functions to update with the latest values
    get_min_angle();
    get_max_angle();
    get_angle_increment();
    get_time_increment();
    get_scan_time();
    get_min_range();
    get_max_range();
    get_ranges_size();
    get_intensities_size();
    get_ranges();
    get_intensities();

    // update the message values
    message.angle_min = angle_min_;
    message.angle_max = angle_max_;
    message.angle_increment = angle_increment_;
    message.time_increment = time_increment_;
    message.scan_time = scan_time_;
    message.range_min = range_min_;
    message.range_max = range_max_;
    message.ranges = ranges_;
    message.intensities = intensities_;

    //std::cout << "INIT: " << ranges_init_ << std::endl;
    //std::cout << "Size: " << ranges_size_ << std::endl;
    //std::cout << "INIT: " << intensities_init_ << std::endl;
    //std::cout << "Size: " << intensities_size_ << std::endl;

    return true;
  }

protected:
  // Order is: angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max, ranges, intensities
  float angle_min_;
  float angle_max_;
  float angle_increment_;
  float time_increment_;
  float scan_time_;
  float range_min_;
  float range_max_;
  float ranges_size_;
  float intensities_size_;
  std::vector<float> ranges_;
  std::vector<float> intensities_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__CAMERA_SENSOR_HPP_
