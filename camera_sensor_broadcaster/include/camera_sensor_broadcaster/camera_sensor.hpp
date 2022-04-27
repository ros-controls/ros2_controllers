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
#include "sensor_msgs/msg/image.hpp"

namespace semantic_components
{
class CameraSensor : public SemanticComponentInterface<sensor_msgs::msg::Image>
{
public:
  explicit CameraSensor(const std::string & name) : SemanticComponentInterface(name, 7)
  {
    interface_names_.emplace_back(name_ + "/" + "height");
    interface_names_.emplace_back(name_ + "/" + "width");
    interface_names_.emplace_back(name_ + "/" + "encoding");
    interface_names_.emplace_back(name_ + "/" + "is_bigendian");
    interface_names_.emplace_back(name_ + "/" + "step");
    interface_names_.emplace_back(name_ + "/" + "data_size");
    interface_names_.emplace_back(name_ + "/" + "data");

    // Set default values to NaN
    height_ = std::numeric_limits<uint32_t>::quiet_NaN();
    //height_.fill(std::numeric_limits<int>::quiet_NaN());
    width_ = std::numeric_limits<uint32_t>::quiet_NaN();
    //distortion_model_.fill(std::numeric_limits<std::string>::quiet_NaN());
    encoding_ = std::numeric_limits<uint32_t>::quiet_NaN();
    is_bigendian_ = std::numeric_limits<uint32_t>::quiet_NaN();
    step_ = std::numeric_limits<uint32_t>::quiet_NaN();
    data_size_ = std::numeric_limits<uint32_t>::quiet_NaN();

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
    height_ = (int)state_interfaces_[interface_offset].get().get_value();
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
    width_ = (int)state_interfaces_[interface_offset].get().get_value();
    return width_;
  }

  std::string get_encoding(){
    size_t interface_offset = 2;
    //int tmp = (int) state_interfaces_[interface_offset].get().get_value(); 
    encoding_ = "rgb8"; //modificar
    return encoding_;
  }

  bool is_bigendian(){
    size_t interface_offset = 3;
    is_bigendian_ = (int) state_interfaces_[interface_offset].get().get_value();
    return is_bigendian_;
  }

  uint32_t get_step(){
    size_t interface_offset = 4;
    step_ = (int) state_interfaces_[interface_offset].get().get_value();
    return step_;
  }

  size_t get_data_size(){
    size_t interface_offset = 5;
    data_size_ = (size_t) state_interfaces_[interface_offset].get().get_value();
    return data_size_;
  }

    //std::vector<float> get_data(){
  std::vector<unsigned char, std::allocator<unsigned char>> get_data(){
    size_t interface_offset = 6;
    auto arrayData = state_interfaces_[interface_offset].get().get_array_value();
    data_.clear();
    
    for(auto data : arrayData)
      data_.push_back((unsigned char) data);
    
    //std::cout << "DATA SIZE: "  << data_.size() << std::endl;
    return data_;
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
  bool get_values_as_message(sensor_msgs::msg::Image & message)
  {
    // call get_height() and get_width()  get_distortion_model() to
    // update with the latest values
    get_height();
    get_width();
    get_encoding();
    is_bigendian();
    get_step();
    get_data_size();
    get_data();

    // update the message values, calibration matrices unknown
    message.height = height_;
    message.width = width_;
    message.encoding = encoding_;
    message.is_bigendian = is_bigendian_;
    message.step = step_;
    message.data = data_;
    //message.set__data(data_);
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
  //std::string distortion_model_;
  std::string encoding_;
  bool is_bigendian_;
  uint32_t step_;
  std::vector<unsigned char, std::allocator<unsigned char>> data_;
  size_t data_size_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__CAMERA_SENSOR_HPP_
