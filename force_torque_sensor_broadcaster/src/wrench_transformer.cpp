// Copyright (c) 2025, ros2_control development team
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
 * Authors: Julia Jia
 */

#include "force_torque_sensor_broadcaster/wrench_transformer.hpp"

#include <limits>
#include <memory>
#include <string>

#include "tf2/utils.hpp"

namespace force_torque_sensor_broadcaster
{

WrenchTransformer::WrenchTransformer(const rclcpp::NodeOptions & options)
: rclcpp::Node("fts_wrench_transformer", options)
{
  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void WrenchTransformer::init()
{
  try
  {
    param_listener_ = std::make_shared<force_torque_wrench_transformer::ParamListener>(
      shared_from_this());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Exception thrown during initialization: %s", e.what());
    return;
  }

  // Setup subscriber and publishers
  setup_subscriber();
  setup_publishers();
}

void WrenchTransformer::wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{

  if (!msg || msg->header.frame_id.empty()) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Invalid wrench message or frame_id is empty");
    return;
  }
  
  for (const auto & target_frame : params_.target_frames) {
    geometry_msgs::msg::WrenchStamped output_wrench;
    // preserve timestamp
    output_wrench.header.stamp = msg->header.stamp;
    output_wrench.header.frame_id = target_frame;
    if (transform_wrench(*msg, target_frame, output_wrench)) {
      auto publisher = transformed_wrench_publishers_[target_frame];
      if (publisher) {
        publisher->publish(output_wrench);
      }
    }
    else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to transform wrench for frame %s, skipping publication", target_frame.c_str());
    }
  }
}

bool WrenchTransformer::transform_wrench(
  const geometry_msgs::msg::WrenchStamped & input_wrench, const std::string & target_frame,
  geometry_msgs::msg::WrenchStamped & output_wrench)
{
  try {
    auto transform = tf_buffer_->lookupTransform(target_frame, input_wrench.header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(params_.tf_timeout));
    output_wrench.header.frame_id = target_frame;
    tf2::doTransform(input_wrench, output_wrench, transform);
    // Preserve original timestamp after transform (doTransform may modify it)
    output_wrench.header.stamp = input_wrench.header.stamp;
    return true;
  }
  catch (const tf2::TransformException & e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Transform exception: %s", e.what());
    return false;
  }
  return false;
}

void WrenchTransformer::setup_subscriber()
{
  input_topic_ = params_.broadcaster_namespace.empty() ? "~/wrench" : params_.broadcaster_namespace + "/wrench";
  if (params_.use_filtered_input) {
    input_topic_ += "_filtered";
  }
  wrench_subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    input_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&WrenchTransformer::wrench_callback, this, std::placeholders::_1));
  if (wrench_subscriber_ == nullptr) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to create wrench subscriber");
    return;
  }
}

void WrenchTransformer::setup_publishers()
{
  for (const auto & target_frame : params_.target_frames) {
    std::string topic_name = params_.output_topic_prefix + "_" + target_frame;
    transformed_wrench_publishers_[target_frame] = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      topic_name, rclcpp::SystemDefaultsQoS());
    if (transformed_wrench_publishers_[target_frame] == nullptr) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to create publisher for target frame %s", target_frame.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Created publisher for target frame %s", target_frame.c_str());
  }
}

}  // namespace force_torque_sensor_broadcaster

