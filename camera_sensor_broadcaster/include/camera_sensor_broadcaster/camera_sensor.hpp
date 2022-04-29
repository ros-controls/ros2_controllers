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
    height_ = (int)state_interfaces_[interface_offset].get().get_int_value();
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
    width_ = state_interfaces_[interface_offset].get().get_int_value();
    return width_;
  }

  std::string get_encoding(){
    size_t interface_offset = 2;
    int tmp = state_interfaces_[interface_offset].get().get_int_value();
    // UNKNOWN_PIXEL_FORMAT = 0,
    // L_INT8 = 1,
    // L_INT16 = 2,
    // RGB_INT8 = 3,
    // RGBA_INT8 = 4,
    // BGRA_INT8 = 5,
    // RGB_INT16 = 6,
    // RGB_INT32 = 7,
    // BGR_INT8 = 8,
    // BGR_INT16 = 9,
    // BGR_INT32 = 10,
    // R_FLOAT16 = 11,
    // RGB_FLOAT16 = 12,
    // R_FLOAT32 = 13,
    // RGB_FLOAT32 = 14,
    // BAYER_RGGB8 = 15,
    // BAYER_BGGR8 = 16,
    // BAYER_GBRG8 = 17,
    // BAYER_GRBG8 = 18,
    if(tmp == 1)
      encoding_ = "";
    else if(tmp == 2)
      encoding_ = "";
    else if(tmp == 3)
      encoding_ = "rgb8"; 
    else if(tmp == 4)
      encoding_ = "rgba8"; 
    else if(tmp == 5)
      encoding_ = "bgra8"; 
    else if(tmp == 6)
      encoding_ = "rgb16"; 
    else if(tmp == 7)
      encoding_ = ""; 
    else if(tmp == 8)
      encoding_ = "bgr8"; 
    else if(tmp == 9)
      encoding_ = "bgr16"; 
    else if(tmp == 10)
      encoding_ = ""; 
    else if(tmp == 11)
      encoding_ = ""; 
    else if(tmp == 12)
      encoding_ = "rgb16"; 
    else if(tmp == 13)
      encoding_ = ""; 
    else if(tmp == 14)
      encoding_ = ""; 
    else if(tmp == 15)
      encoding_ = "bayer_rggb8"; 
    else if(tmp == 16)
      encoding_ = "bayer_rggr8"; 
    else if(tmp == 17)
      encoding_ = "bayer_gbrg8"; 
    else if(tmp == 18)
      encoding_ = "bayer_grbg8"; 
      
    return encoding_;
  }

  bool is_bigendian(){
    size_t interface_offset = 3;
    is_bigendian_ = state_interfaces_[interface_offset].get().get_int_value();
    return is_bigendian_;
  }

  uint32_t get_step(){
    size_t interface_offset = 4;
    step_ = state_interfaces_[interface_offset].get().get_int_value();
    return step_;
  }

  size_t get_data_size(){
    size_t interface_offset = 5;
    data_size_ = (size_t) state_interfaces_[interface_offset].get().get_int_value();
    return data_size_;
  }

    //std::vector<float> get_data(){
  std::vector<unsigned char, std::allocator<unsigned char>> get_data(){
    size_t interface_offset = 6;
    data_= state_interfaces_[interface_offset].get().get_str_value();
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