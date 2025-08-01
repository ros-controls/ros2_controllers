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

FKClient::FKClient(const std::string & node_name) : Node(node_name)
{
  fk_client_ = this->create_client<moveit_msgs::srv::GetPositionFK>("/compute_fk");

  while (!fk_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for /compute_fk service...");
  }
}

geometry_msgs::msg::Pose FKClient::computeFK(
  const std::vector<std::string> & joint_names, const std::vector<double> & joint_positions,
  const std::string & from_frame, const std::string & to_link)
{
  auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
  request->fk_link_names.push_back(to_link);

  sensor_msgs::msg::JointState joint_state;
  joint_state.name = joint_names;
  joint_state.position = joint_positions;
  request->robot_state.joint_state = joint_state;
  request->header.frame_id = from_frame;

  auto future = fk_client_->async_send_request(request);

  if (
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (!response->pose_stamped.empty())
    {
      return response->pose_stamped.front().pose;
    }
    else
    {
      throw std::runtime_error("Empty response received from FK service.");
    }
  }
  else
  {
    throw std::runtime_error("Error calling FK service.");
  }
}
