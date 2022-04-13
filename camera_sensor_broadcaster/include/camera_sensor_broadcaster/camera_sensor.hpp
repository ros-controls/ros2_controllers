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

#ifndef SEMANTIC_COMPONENTS__CAMERA_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__CAMERA_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace semantic_components
{
class CameraSensor : public SemanticComponentInterface<sensor_msgs::msg::CameraInfo>
{
public:
  explicit CameraSensor(const std::string & name) : SemanticComponentInterface(name, 2)
  {
    interface_names_.emplace_back(name_ + "/" + "height");
    interface_names_.emplace_back(name_ + "/" + "width");
    // interface_names_.emplace_back(name_ + "/" + "distortion_model");
    // interface_names_.emplace_back(name_ + "/" + "D");
    // interface_names_.emplace_back(name_ + "/" + "K");
    // interface_names_.emplace_back(name_ + "/" + "R");
    // interface_names_.emplace_back(name_ + "/" + "P");
    // interface_names_.emplace_back(name_ + "/" + "binning_x");
    // interface_names_.emplace_back(name_ + "/" + "binning_y");
    // interface_names_.emplace_back(name_ + "/" + "roi.width");
    // interface_names_.emplace_back(name_ + "/" + "roi.height");

    // Set default values to NaN
    height_ = std::numeric_limits<uint32_t>::quiet_NaN();
    //height_.fill(std::numeric_limits<int>::quiet_NaN());
    width_ = std::numeric_limits<uint32_t>::quiet_NaN();
    //distortion_model_.fill(std::numeric_limits<std::string>::quiet_NaN());
  }

  virtual ~CameraSensor() = default;

  /// Return height
  /**
   * Return height reported by an Camera
   *
   * \return height size
   */
  uint32_t get_height()
  {
    size_t interface_offset = 0;
    height_ = state_interfaces_[interface_offset].get().get_value();
    return height_;
  }

  /// Return width
  /**
   * Return width reported by an Camera
   *
   * \return width size
   */
  uint32_t get_width()
  {
    size_t interface_offset = 1;
    width_ = state_interfaces_[interface_offset].get().get_value();
    return width_;
  }

  /// Return distortion model
  /**
   * Return distortion model reported by an Camera
   *
   * \return distortion model as string
   */
  std::string get_distortion_model()
  {
    size_t interface_offset = 2;
    distortion_model_ = state_interfaces_[interface_offset].get().get_value();
    return distortion_model_;
  }

  // /// Return orientation.
  // /**
  //  * Return orientation reported by an Camera
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


  /// Return Camera message with height and width
  /**
   * Constructs and return a Camera message from the current values.
   * \return Camera message from values;
   */
  bool get_values_as_message(sensor_msgs::msg::CameraInfo & message)
  {
    // call get_height() and get_width()  get_distortion_model() to
    // update with the latest values
    get_height();
    get_width();
    //get_distortion_model();

    // update the message values, calibration matrices unknown
    message.height = height_;
    message.width = width_;
    //message.distortion_model = distortion_model_;

    return true;
  }

protected:
  // Order is: orientation X,Y,Z,W angular velocity X,Y,Z and linear acceleration X,Y,Z
  //std::array<double, 4> orientation_;
  //std::array<double, 3> angular_velocity_;
  //std::array<double, 3> linear_acceleration_;
  // Order is: orientation height, width, distortion_model,
  uint32_t height_;
  uint32_t width_;
  std::string distortion_model_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__CAMERA_SENSOR_HPP_
