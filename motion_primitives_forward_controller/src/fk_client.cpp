// Copyright (c) 2025, b»robotized
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
//
// Authors: Mathias Fuhrer

#include "motion_primitives_forward_controller/fk_client.hpp"
#include <chrono>

using namespace std::chrono_literals;

FKClient::FKClient(const rclcpp::Node::SharedPtr & node) : node_(node)
{
  client_ = node_->create_client<moveit_msgs::srv::GetPositionFK>("/compute_fk");

  while (!client_->wait_for_service(1s))
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for /compute_fk service...");
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service.");
      return;
    }
  }
}

std::optional<geometry_msgs::msg::Pose> FKClient::computeFK(
  const std::vector<std::string> & joint_names, const std::vector<double> & joint_positions,
  const std::string & from_frame, const std::string & to_link)
{
  auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
  request->header.frame_id = from_frame;
  request->fk_link_names.push_back(to_link);
  request->robot_state.joint_state.name = joint_names;
  request->robot_state.joint_state.position = joint_positions;

  auto future = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future, 3s) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = future.get();
    if (result->error_code.val == result->error_code.SUCCESS)
    {
      return result->pose_stamped[0].pose;
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "FK error: code=%d", result->error_code.val);
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "FK call timed out");
  }

  return std::nullopt;
}
