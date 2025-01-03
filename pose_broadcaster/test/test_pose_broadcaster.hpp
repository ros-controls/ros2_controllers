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

#include "rclcpp/executors.hpp"

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
  const std::string frame_id_ = "pose_base_frame";
  const std::string tf_child_frame_id_ = "pose_frame";

  std::array<double, 7> pose_values_ = {{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7}};

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

  template <typename T>
  void subscribe_and_get_message(const std::string & topic, T & msg);
};

template <typename T>
void PoseBroadcasterTest::subscribe_and_get_message(const std::string & topic, T & msg)
{
  // Create node for subscribing
  rclcpp::Node node{"test_subscription_node"};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node.get_node_base_interface());

  // Create subscription
  typename T::SharedPtr received_msg;
  const auto msg_callback = [&](const typename T::SharedPtr sub_msg) { received_msg = sub_msg; };
  const auto subscription = node.create_subscription<T>(topic, 10, msg_callback);

  // Update controller and spin until a message is received
  // Since update doesn't guarantee a published message, republish until received
  constexpr size_t max_sub_check_loop_count = 5;
  for (size_t i = 0; !received_msg; ++i)
  {
    ASSERT_LT(i, max_sub_check_loop_count);

    pose_broadcaster_->update(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01));

    const auto timeout = std::chrono::milliseconds{5};
    const auto until = node.get_clock()->now() + timeout;
    while (!received_msg && node.get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds{10});
    }
  }

  msg = *received_msg;
}

#endif  // TEST_POSE_BROADCASTER_HPP_
