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
#include "test_pose_broadcaster.hpp"

#include <utility>
#include <vector>

using hardware_interface::LoanedStateInterface;

void PoseBroadcasterTest::SetUp() { pose_broadcaster_ = std::make_unique<PoseBroadcaster>(); }

void PoseBroadcasterTest::TearDown()
{
  try
  {
    pose_broadcaster_->get_node()->shutdown();
  }
  catch (...)
  {
    // ignore case where node is not initialized
  }
  pose_broadcaster_.reset(nullptr);  // this calls the dtor, but does not call shutdown transition
}

void PoseBroadcasterTest::SetUpPoseBroadcaster()
{
  ASSERT_EQ(
    pose_broadcaster_->init(
      "test_pose_broadcaster", "", 0, "", pose_broadcaster_->define_custom_node_options()),
    controller_interface::return_type::OK);

  std::vector<LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(pose_position_x_);
  state_interfaces.emplace_back(pose_position_y_);
  state_interfaces.emplace_back(pose_position_z_);
  state_interfaces.emplace_back(pose_orientation_x_);
  state_interfaces.emplace_back(pose_orientation_y_);
  state_interfaces.emplace_back(pose_orientation_z_);
  state_interfaces.emplace_back(pose_orientation_w_);

  pose_broadcaster_->assign_interfaces({}, std::move(state_interfaces));
}

TEST_F(PoseBroadcasterTest, Configure_Success)
{
  SetUpPoseBroadcaster();

  // Set 'pose_name' and 'frame_id' parameters
  pose_broadcaster_->get_node()->set_parameter({"pose_name", pose_name_});
  pose_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // Configure controller
  ASSERT_EQ(
    pose_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Verify command interface configuration
  const auto command_interface_conf = pose_broadcaster_->command_interface_configuration();
  EXPECT_EQ(command_interface_conf.type, controller_interface::interface_configuration_type::NONE);
  EXPECT_TRUE(command_interface_conf.names.empty());

  // Verify state interface configuration
  const auto state_interface_conf = pose_broadcaster_->state_interface_configuration();
  EXPECT_EQ(
    state_interface_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(state_interface_conf.names.size(), 7lu);
}

TEST_F(PoseBroadcasterTest, Activate_Success)
{
  SetUpPoseBroadcaster();

  // Set 'pose_name' and 'frame_id' parameters
  pose_broadcaster_->get_node()->set_parameter({"pose_name", pose_name_});
  pose_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // Configure and activate controller
  ASSERT_EQ(
    pose_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    pose_broadcaster_->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Verify command and state interface configuration
  {
    const auto command_interface_conf = pose_broadcaster_->command_interface_configuration();
    EXPECT_EQ(
      command_interface_conf.type, controller_interface::interface_configuration_type::NONE);
    EXPECT_TRUE(command_interface_conf.names.empty());

    const auto state_interface_conf = pose_broadcaster_->state_interface_configuration();
    EXPECT_EQ(
      state_interface_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
    ASSERT_EQ(state_interface_conf.names.size(), 7lu);
  }

  // Deactivate controller
  ASSERT_EQ(
    pose_broadcaster_->on_deactivate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Verify command and state interface configuration
  {
    const auto command_interface_conf = pose_broadcaster_->command_interface_configuration();
    EXPECT_EQ(
      command_interface_conf.type, controller_interface::interface_configuration_type::NONE);
    EXPECT_TRUE(command_interface_conf.names.empty());

    const auto state_interface_conf = pose_broadcaster_->state_interface_configuration();
    EXPECT_EQ(
      state_interface_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
    ASSERT_EQ(state_interface_conf.names.size(), 7lu);  // Should not change when deactivating
  }
}

TEST_F(PoseBroadcasterTest, Update_Success)
{
  SetUpPoseBroadcaster();

  // Set 'pose_name' and 'frame_id' parameters
  pose_broadcaster_->get_node()->set_parameter({"pose_name", pose_name_});
  pose_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // Configure and activate controller
  ASSERT_EQ(
    pose_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    pose_broadcaster_->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  ASSERT_EQ(
    pose_broadcaster_->update(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(PoseBroadcasterTest, PublishSuccess)
{
  SetUpPoseBroadcaster();

  // Set 'pose_name' and 'frame_id' parameters
  pose_broadcaster_->get_node()->set_parameter({"pose_name", pose_name_});
  pose_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // Set 'tf.enable' and 'tf.child_frame_id' parameters
  pose_broadcaster_->get_node()->set_parameter({"tf.enable", true});
  pose_broadcaster_->get_node()->set_parameter({"tf.child_frame_id", tf_child_frame_id_});

  // Configure and activate controller
  ASSERT_EQ(
    pose_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    pose_broadcaster_->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Subscribe to pose topic
  geometry_msgs::msg::PoseStamped pose_msg;
  subscribe_and_get_message("/test_pose_broadcaster/pose", pose_msg);

  // Verify content of pose message
  EXPECT_EQ(pose_msg.header.frame_id, frame_id_);
  EXPECT_EQ(pose_msg.pose.position.x, pose_values_[0]);
  EXPECT_EQ(pose_msg.pose.position.y, pose_values_[1]);
  EXPECT_EQ(pose_msg.pose.position.z, pose_values_[2]);
  EXPECT_EQ(pose_msg.pose.orientation.x, pose_values_[3]);
  EXPECT_EQ(pose_msg.pose.orientation.y, pose_values_[4]);
  EXPECT_EQ(pose_msg.pose.orientation.z, pose_values_[5]);
  EXPECT_EQ(pose_msg.pose.orientation.w, pose_values_[6]);

  // Subscribe to tf topic
  tf2_msgs::msg::TFMessage tf_msg;
  subscribe_and_get_message("/tf", tf_msg);

  // Verify content of tf message
  ASSERT_EQ(tf_msg.transforms.size(), 1lu);
  EXPECT_EQ(tf_msg.transforms[0].header.frame_id, frame_id_);
  EXPECT_EQ(tf_msg.transforms[0].child_frame_id, tf_child_frame_id_);
  EXPECT_EQ(tf_msg.transforms[0].transform.translation.x, pose_values_[0]);
  EXPECT_EQ(tf_msg.transforms[0].transform.translation.y, pose_values_[1]);
  EXPECT_EQ(tf_msg.transforms[0].transform.translation.z, pose_values_[2]);
  EXPECT_EQ(tf_msg.transforms[0].transform.rotation.x, pose_values_[3]);
  EXPECT_EQ(tf_msg.transforms[0].transform.rotation.y, pose_values_[4]);
  EXPECT_EQ(tf_msg.transforms[0].transform.rotation.z, pose_values_[5]);
  EXPECT_EQ(tf_msg.transforms[0].transform.rotation.w, pose_values_[6]);
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}
