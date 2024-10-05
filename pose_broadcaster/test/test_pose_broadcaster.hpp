// Copyright 2024 FZI Forschungszentrum Informatik
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
#ifndef TEST_POSE_BROADCASTER_HPP_
#define TEST_POSE_BROADCASTER_HPP_

#include <gmock/gmock.h>

#include <array>
#include <memory>
#include <string>

#include "pose_broadcaster/pose_broadcaster.hpp"

using pose_broadcaster::PoseBroadcaster;

class PoseBroadcasterTest : public ::testing::Test
{
public:
  void SetUp();
  void TearDown();

  void SetUpPoseBroadcaster();

protected:
  const std::string pose_name_ = "test_pose";
  const std::string frame_id_ = "pose_frame";

  std::array<double, 7> pose_values_ = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7};

  hardware_interface::StateInterface pose_position_x_{pose_name_, "position.x", &pose_values_[0]};
  hardware_interface::StateInterface pose_position_y_{pose_name_, "position.x", &pose_values_[1]};
  hardware_interface::StateInterface pose_position_z_{pose_name_, "position.x", &pose_values_[2]};
  hardware_interface::StateInterface pose_orientation_x_{
    pose_name_, "orientation.x", &pose_values_[3]};
  hardware_interface::StateInterface pose_orientation_y_{
    pose_name_, "orientation.y", &pose_values_[4]};
  hardware_interface::StateInterface pose_orientation_z_{
    pose_name_, "orientation.z", &pose_values_[5]};
  hardware_interface::StateInterface pose_orientation_w_{
    pose_name_, "orientation.w", &pose_values_[6]};

  std::unique_ptr<PoseBroadcaster> pose_broadcaster_;

  void subscribe_and_get_message(geometry_msgs::msg::PoseStamped & pose_msg);
};

#endif  // TEST_POSE_BROADCASTER_HPP_
