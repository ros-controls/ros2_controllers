// Copyright 2023 AIT Austrian Institute of Technology
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

#include "test_pid_trajectory.hpp"

TEST_F(PidTrajectoryTest, TestEmptySetup)
{
  std::shared_ptr<TestableJointTrajectoryControllerPlugin> traj_contr =
    std::make_shared<TestableJointTrajectoryControllerPlugin>();

  std::vector<size_t> map_cmd_to_joints{};
  ASSERT_FALSE(traj_contr->initialize(node_, map_cmd_to_joints));
}

TEST_F(PidTrajectoryTest, TestSingleJoint)
{
  std::shared_ptr<TestableJointTrajectoryControllerPlugin> traj_contr =
    std::make_shared<TestableJointTrajectoryControllerPlugin>();

  std::vector<std::string> joint_names = {"joint1"};
  auto joint_names_paramv = rclcpp::ParameterValue(joint_names);

  // override parameter
  node_->declare_parameter("command_joints", joint_names_paramv);

  std::vector<size_t> map_cmd_to_joints{0};
  ASSERT_TRUE(traj_contr->initialize(node_, map_cmd_to_joints));
  node_->set_parameter(rclcpp::Parameter("gains.joint1.p", 1.0));
  ASSERT_TRUE(traj_contr->configure());
  ASSERT_TRUE(traj_contr->activate());
  ASSERT_TRUE(traj_contr->compute_control_law_non_rt(
    std::make_shared<trajectory_msgs::msg::JointTrajectory>()));
  ASSERT_TRUE(traj_contr->update_gains_rt());

  trajectory_msgs::msg::JointTrajectoryPoint traj_msg;
  traj_msg.positions.push_back(0.0);
  traj_msg.velocities.push_back(0.0);
  std::vector<double> tmp_command(1, 0.0);
  const rclcpp::Duration time_since_start(1, 0);
  const rclcpp::Duration period(1, 0);

  ASSERT_NO_THROW(traj_contr->compute_commands(
    tmp_command, traj_msg, traj_msg, traj_msg, time_since_start, period));
}

TEST_F(PidTrajectoryTest, TestMultipleJoints)
{
  std::shared_ptr<TestableJointTrajectoryControllerPlugin> traj_contr =
    std::make_shared<TestableJointTrajectoryControllerPlugin>();

  std::vector<std::string> joint_names = {"joint1", "joint2", "joint3"};
  auto joint_names_paramv = rclcpp::ParameterValue(joint_names);

  // override parameter
  node_->declare_parameter("command_joints", joint_names_paramv);

  std::vector<size_t> map_cmd_to_joints{0, 1, 2};
  ASSERT_TRUE(traj_contr->initialize(node_, map_cmd_to_joints));
  // set dynamic parameters
  node_->set_parameter(rclcpp::Parameter("gains.joint1.p", 1.0));
  node_->set_parameter(rclcpp::Parameter("gains.joint2.p", 2.0));
  node_->set_parameter(rclcpp::Parameter("gains.joint3.p", 3.0));
  ASSERT_TRUE(traj_contr->configure());
  ASSERT_TRUE(traj_contr->activate());
  ASSERT_TRUE(traj_contr->compute_control_law_non_rt(
    std::make_shared<trajectory_msgs::msg::JointTrajectory>()));
  ASSERT_TRUE(traj_contr->update_gains_rt());

  ASSERT_TRUE(traj_contr->compute_control_law_non_rt(
    std::make_shared<trajectory_msgs::msg::JointTrajectory>()));
  ASSERT_TRUE(traj_contr->update_gains_rt());

  trajectory_msgs::msg::JointTrajectoryPoint traj_msg;
  traj_msg.positions.push_back(1.0);
  traj_msg.positions.push_back(1.0);
  traj_msg.positions.push_back(1.0);
  traj_msg.velocities.push_back(0.0);
  traj_msg.velocities.push_back(0.0);
  traj_msg.velocities.push_back(0.0);
  std::vector<double> tmp_command(3, 0.0);
  const rclcpp::Duration time_since_start(1, 0);
  const rclcpp::Duration period(1, 0);

  ASSERT_NO_THROW(traj_contr->compute_commands(
    tmp_command, traj_msg, traj_msg, traj_msg, time_since_start, period));

  EXPECT_EQ(tmp_command[0], 1.0);
  EXPECT_EQ(tmp_command[1], 2.0);
  EXPECT_EQ(tmp_command[2], 3.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
