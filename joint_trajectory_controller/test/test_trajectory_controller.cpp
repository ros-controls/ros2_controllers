// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <stddef.h>

#include <array>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "gmock/gmock.h"

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "control_msgs/msg/detail/joint_trajectory_controller_state__struct.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/header.hpp"
#include "test_trajectory_controller_utils.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using lifecycle_msgs::msg::State;
using test_trajectory_controllers::TrajectoryControllerTest;
using test_trajectory_controllers::TrajectoryControllerTestParameterized;

bool is_same_sign(double val1, double val2) { return val1 * val2 >= 0.0; }

void spin(rclcpp::executors::MultiThreadedExecutor * exe) { exe->spin(); }

TEST_P(TrajectoryControllerTestParameterized, configure_state_ignores_commands)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  // const auto future_handle_ = std::async(std::launch::async, spin, &executor);

  const auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  traj_controller_->update(
    rclcpp::Time(static_cast<uint64_t>(0.5 * 1e9)), rclcpp::Duration::from_seconds(0.5));

  // hw position == 0 because controller is not activated
  EXPECT_EQ(0.0, joint_pos_[0]);
  EXPECT_EQ(0.0, joint_pos_[1]);
  EXPECT_EQ(0.0, joint_pos_[2]);

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, check_interface_names)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  const auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  std::vector<std::string> state_interface_names;
  state_interface_names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint : joint_names_)
  {
    for (const auto & interface : state_interface_types_)
    {
      state_interface_names.push_back(joint + "/" + interface);
    }
  }
  auto state_interfaces = traj_controller_->state_interface_configuration();
  EXPECT_EQ(state_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(state_interfaces.names.size(), joint_names_.size() * state_interface_types_.size());
  ASSERT_THAT(state_interfaces.names, testing::UnorderedElementsAreArray(state_interface_names));

  std::vector<std::string> command_interface_names;
  command_interface_names.reserve(joint_names_.size() * command_interface_types_.size());
  for (const auto & joint : joint_names_)
  {
    for (const auto & interface : command_interface_types_)
    {
      command_interface_names.push_back(joint + "/" + interface);
    }
  }
  auto command_interfaces = traj_controller_->command_interface_configuration();
  EXPECT_EQ(
    command_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(command_interfaces.names.size(), joint_names_.size() * command_interface_types_.size());
  ASSERT_THAT(
    command_interfaces.names, testing::UnorderedElementsAreArray(command_interface_names));
}

TEST_P(TrajectoryControllerTestParameterized, check_interface_names_with_command_joints)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  // set command_joints parameter
  const rclcpp::Parameter command_joint_names_param("command_joints", command_joint_names_);
  traj_controller_->get_node()->set_parameter({command_joint_names_param});

  const auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  std::vector<std::string> state_interface_names;
  state_interface_names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto & joint : joint_names_)
  {
    for (const auto & interface : state_interface_types_)
    {
      state_interface_names.push_back(joint + "/" + interface);
    }
  }
  auto state_interfaces = traj_controller_->state_interface_configuration();
  EXPECT_EQ(state_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(state_interfaces.names.size(), joint_names_.size() * state_interface_types_.size());
  ASSERT_THAT(state_interfaces.names, testing::UnorderedElementsAreArray(state_interface_names));

  std::vector<std::string> command_interface_names;
  command_interface_names.reserve(command_joint_names_.size() * command_interface_types_.size());
  for (const auto & joint : command_joint_names_)
  {
    for (const auto & interface : command_interface_types_)
    {
      command_interface_names.push_back(joint + "/" + interface);
    }
  }
  auto command_interfaces = traj_controller_->command_interface_configuration();
  EXPECT_EQ(
    command_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(
    command_interfaces.names.size(), command_joint_names_.size() * command_interface_types_.size());
  ASSERT_THAT(
    command_interfaces.names, testing::UnorderedElementsAreArray(command_interface_names));
}

TEST_P(TrajectoryControllerTestParameterized, activate)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  traj_controller_->get_node()->configure();
  ASSERT_EQ(traj_controller_->get_state().id(), State::PRIMARY_STATE_INACTIVE);

  auto cmd_interface_config = traj_controller_->command_interface_configuration();
  ASSERT_EQ(
    cmd_interface_config.names.size(), joint_names_.size() * command_interface_types_.size());

  auto state_interface_config = traj_controller_->state_interface_configuration();
  ASSERT_EQ(
    state_interface_config.names.size(), joint_names_.size() * state_interface_types_.size());

  ActivateTrajectoryController();
  ASSERT_EQ(traj_controller_->get_state().id(), State::PRIMARY_STATE_ACTIVE);

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, activate_deactivate)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  traj_controller_->get_node()->configure();
  ASSERT_EQ(traj_controller_->get_state().id(), State::PRIMARY_STATE_INACTIVE);

  auto cmd_interface_config = traj_controller_->command_interface_configuration();
  ASSERT_EQ(
    cmd_interface_config.names.size(), joint_names_.size() * command_interface_types_.size());

  auto state_interface_config = traj_controller_->state_interface_configuration();
  ASSERT_EQ(
    state_interface_config.names.size(), joint_names_.size() * state_interface_types_.size());

  ActivateTrajectoryController();
  ASSERT_EQ(traj_controller_->get_state().id(), State::PRIMARY_STATE_ACTIVE);
  traj_controller_->get_node()->deactivate();
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  traj_controller_->get_node()->activate();
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  executor.cancel();
}

// TEST_F(TestTrajectoryController, activation) {
//   auto traj_controller = std::make_shared<ros_controllers::JointTrajectoryController>(
//     joint_names_, op_mode_);
//   auto ret = traj_controller->init(test_robot_, controller_name_);
//   if (ret != controller_interface::return_type::OK) {
//     FAIL();
//   }
//
//   auto traj_node = traj_controller->get_node();
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(traj_node->get_node_base_interface());
//
//   auto state = traj_controller_->configure();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
//
//   state = traj_node->activate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);
//
//   // wait for the subscriber and publisher to completely setup
//   std::this_thread::sleep_for(std::chrono::seconds(2));
//
//   // send msg
//   builtin_interfaces::msg::Duration time_from_start;
//   time_from_start.sec = 1;
//   time_from_start.nanosec = 0;
//   std::vector<std::vector<double>> points {{{3.3, 4.4, 5.5}}};
//   publish(time_from_start, points, rclcpp::Time());
//   // wait for msg is be published to the system
//   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//   executor.spin_once();
//
//   traj_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//   resource_manager_->write();
//
//   // change in hw position
//   EXPECT_EQ(3.3, joint_pos_[0]);
//   EXPECT_EQ(4.4, joint_pos_[1]);
//   EXPECT_EQ(5.5, joint_pos_[2]);
//
//   executor.cancel();
// }

// TEST_F(TestTrajectoryController, reactivation) {
//   auto traj_controller = std::make_shared<ros_controllers::JointTrajectoryController>(
//     joint_names_, op_mode_);
//   auto ret = traj_controller->init(test_robot_, controller_name_);
//   if (ret != controller_interface::return_type::OK) {
//     FAIL();
//   }
//
//   auto traj_node = traj_controller->get_node();
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(traj_node->get_node_base_interface());
//
//   auto state = traj_controller_->configure();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
//
//   state = traj_node->activate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);
//
//   // wait for the subscriber and publisher to completely setup
//   std::this_thread::sleep_for(std::chrono::seconds(2));
//
//   // send msg
//   builtin_interfaces::msg::Duration time_from_start;
//   time_from_start.sec = 1;
//   time_from_start.nanosec = 0;
//   // *INDENT-OFF*
//   std::vector<std::vector<double>> points {
//     {{3.3, 4.4, 5.5}},
//     {{7.7, 8.8, 9.9}},
//     {{10.10, 11.11, 12.12}}
//   };
//   // *INDENT-ON*
//   publish(time_from_start, points, rclcpp::Time());
//   // wait for msg is be published to the system
//   std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   executor.spin_once();
//
//   traj_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//   resource_manager_->write();
//
//   // deactivated
//   // wait so controller process the second point when deactivated
//   std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   state = traj_controller_->deactivate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
//   resource_manager_->read();
//   traj_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//   resource_manager_->write();
//
//   // no change in hw position
//   EXPECT_EQ(3.3, joint_pos_[0]);
//   EXPECT_EQ(4.4, joint_pos_[1]);
//   EXPECT_EQ(5.5, joint_pos_[2]);
//
//   // reactivated
//   // wait so controller process the third point when reactivated
//   std::this_thread::sleep_for(std::chrono::milliseconds(3000));
//   state = traj_node->activate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);
//   resource_manager_->read();
//   traj_controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//   resource_manager_->write();
//
//   // change in hw position to 3rd point
//   EXPECT_EQ(10.10, joint_pos_[0]);
//   EXPECT_EQ(11.11, joint_pos_[1]);
//   EXPECT_EQ(12.12, joint_pos_[2]);
//
//   executor.cancel();
// }

TEST_P(TrajectoryControllerTestParameterized, cleanup)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  traj_controller_->update(
    rclcpp::Time(static_cast<uint64_t>(0.5 * 1e9)), rclcpp::Duration::from_seconds(0.5));

  auto state = traj_controller_->get_node()->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));

  state = traj_controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  // update for 0.25 seconds
  updateController(rclcpp::Duration::from_seconds(0.25));

  // should be home pose again
  EXPECT_NEAR(INITIAL_POS_JOINT1, joint_pos_[0], COMMON_THRESHOLD);
  EXPECT_NEAR(INITIAL_POS_JOINT2, joint_pos_[1], COMMON_THRESHOLD);
  EXPECT_NEAR(INITIAL_POS_JOINT3, joint_pos_[2], COMMON_THRESHOLD);

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, correct_initialization_using_parameters)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor, false);

  // This call is replacing the way parameters are set via launch
  SetParameters();
  traj_controller_->configure();
  auto state = traj_controller_->get_state();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());

  ActivateTrajectoryController();

  state = traj_controller_->get_state();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());
  EXPECT_EQ(INITIAL_POS_JOINT1, joint_pos_[0]);
  EXPECT_EQ(INITIAL_POS_JOINT2, joint_pos_[1]);
  EXPECT_EQ(INITIAL_POS_JOINT3, joint_pos_[2]);

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 5.5}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocities{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(), {}, points_velocities);
  traj_controller_->wait_for_trajectory(executor);

  // first update
  traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));

  // wait so controller process the second point when deactivated
  traj_controller_->update(
    rclcpp::Time(static_cast<uint64_t>(0.25 * 1e9)), rclcpp::Duration::from_seconds(0.15));
  // deactivated
  state = traj_controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  const auto allowed_delta = 0.05;
  if (traj_controller_->has_position_command_interface())
  {
    EXPECT_NEAR(3.3, joint_pos_[0], allowed_delta);
    EXPECT_NEAR(4.4, joint_pos_[1], allowed_delta);
    EXPECT_NEAR(5.5, joint_pos_[2], allowed_delta);
  }

  if (traj_controller_->has_velocity_command_interface())
  {
    EXPECT_LE(0.0, joint_vel_[0]);
    EXPECT_LE(0.0, joint_vel_[1]);
    EXPECT_LE(0.0, joint_vel_[2]);
  }

  if (traj_controller_->has_effort_command_interface())
  {
    EXPECT_LE(0.0, joint_eff_[0]);
    EXPECT_LE(0.0, joint_eff_[1]);
    EXPECT_LE(0.0, joint_eff_[2]);
  }

  // cleanup
  state = traj_controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  // TODO(anyone): should the controller even allow calling update() when it is not active?
  // update loop receives a new msg and updates accordingly
  traj_controller_->update(
    rclcpp::Time(static_cast<uint64_t>(0.35 * 1e9)), rclcpp::Duration::from_seconds(0.1));

  EXPECT_NEAR(3.3, joint_pos_[0], allowed_delta);
  EXPECT_NEAR(4.4, joint_pos_[1], allowed_delta);
  EXPECT_NEAR(5.5, joint_pos_[2], allowed_delta);

  // state = traj_controller_->get_node()->configure();
  // ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, state_topic_consistency)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, true, {});
  subscribeToState();
  updateController();

  // Spin to receive latest state
  executor.spin_some();
  auto state = getState();

  size_t n_joints = joint_names_.size();

  for (unsigned int i = 0; i < n_joints; ++i)
  {
    EXPECT_EQ(joint_names_[i], state->joint_names[i]);
  }

  // No trajectory by default, no desired state or error
  EXPECT_TRUE(state->desired.positions.empty() || state->desired.positions == INITIAL_POS_JOINTS);
  EXPECT_TRUE(state->desired.velocities.empty() || state->desired.velocities == INITIAL_VEL_JOINTS);
  EXPECT_TRUE(
    state->desired.accelerations.empty() || state->desired.accelerations == INITIAL_EFF_JOINTS);

  EXPECT_EQ(n_joints, state->actual.positions.size());
  if (
    std::find(state_interface_types_.begin(), state_interface_types_.end(), "velocity") ==
    state_interface_types_.end())
  {
    EXPECT_TRUE(state->actual.velocities.empty());
  }
  else
  {
    EXPECT_EQ(n_joints, state->actual.velocities.size());
  }
  if (
    std::find(state_interface_types_.begin(), state_interface_types_.end(), "acceleration") ==
    state_interface_types_.end())
  {
    EXPECT_TRUE(state->actual.accelerations.empty());
  }
  else
  {
    EXPECT_EQ(n_joints, state->actual.accelerations.size());
  }

  std::vector<double> zeros(3, 0);
  EXPECT_EQ(state->error.positions, zeros);
  EXPECT_TRUE(state->error.velocities.empty() || state->error.velocities == zeros);
  EXPECT_TRUE(state->error.accelerations.empty() || state->error.accelerations == zeros);
}

void TrajectoryControllerTest::test_state_publish_rate_target(int target_msg_count)
{
  rclcpp::Parameter state_publish_rate_param(
    "state_publish_rate", static_cast<double>(target_msg_count));
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor, true, {state_publish_rate_param});

  auto future_handle = std::async(std::launch::async, [&executor]() -> void { executor.spin(); });

  using control_msgs::msg::JointTrajectoryControllerState;

  const int qos_level = 10;
  int echo_received_counter = 0;
  rclcpp::Subscription<JointTrajectoryControllerState>::SharedPtr subs =
    traj_controller_->get_node()->create_subscription<JointTrajectoryControllerState>(
      controller_name_ + "/state", qos_level,
      [&](JointTrajectoryControllerState::UniquePtr) { ++echo_received_counter; });

  // update for 1second
  auto clock = rclcpp::Clock(RCL_STEADY_TIME);
  const auto start_time = clock.now();
  const rclcpp::Duration wait = rclcpp::Duration::from_seconds(1.0);
  const auto end_time = start_time + wait;
  while (clock.now() < end_time)
  {
    traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
  }

  // We may miss the last message since time allowed is exactly the time needed
  EXPECT_NEAR(target_msg_count, echo_received_counter, 1);

  executor.cancel();
}

/**
 * @brief test_state_publish_rate Test that state publish rate matches configure rate
 */
TEST_P(TrajectoryControllerTestParameterized, test_state_publish_rate)
{
  test_state_publish_rate_target(10);
}

// TEST_P(TrajectoryControllerTestParameterized, zero_state_publish_rate)
// {
//   test_state_publish_rate_target(0);
// }

// /**
//  * @brief test_jumbled_joint_order Test sending trajectories with a joint order different from
//  * internal controller order
//  */
// TEST_P(TrajectoryControllerTestParameterized, test_jumbled_joint_order)
// {
//   rclcpp::executors::SingleThreadedExecutor executor;
//   SetUpAndActivateTrajectoryController(true, {}, &executor);
//   {
//     trajectory_msgs::msg::JointTrajectory traj_msg;
//     const std::vector<std::string> jumbled_joint_names{
//       joint_names_[1], joint_names_[2], joint_names_[0]};
//     traj_msg.joint_names = jumbled_joint_names;
//     traj_msg.header.stamp = rclcpp::Time(0);
//     traj_msg.points.resize(1);

//     traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
//     traj_msg.points[0].positions.resize(3);
//     traj_msg.points[0].positions[0] = 2.0;
//     traj_msg.points[0].positions[1] = 3.0;
//     traj_msg.points[0].positions[2] = 1.0;

//     if (traj_controller_->has_velocity_command_interface())
//     {
//       traj_msg.points[0].velocities.resize(3);
//       traj_msg.points[0].velocities[0] = -0.1;
//       traj_msg.points[0].velocities[1] = -0.1;
//       traj_msg.points[0].velocities[2] = -0.1;
//     }

//     if (traj_controller_->has_effort_command_interface())
//     {
//       traj_msg.points[0].effort.resize(3);
//       traj_msg.points[0].effort[0] = -0.1;
//       traj_msg.points[0].effort[1] = -0.1;
//       traj_msg.points[0].effort[2] = -0.1;
//     }

//     trajectory_publisher_->publish(traj_msg);
//   }

//   traj_controller_->wait_for_trajectory(executor);
//   // update for 0.25 seconds
//   // TODO(destogl): Make this time a bit shorter to increase stability on the CI?
//   //                Currently COMMON_THRESHOLD is adjusted.
//   updateController(rclcpp::Duration::from_seconds(0.25));

//   if (traj_controller_->has_position_command_interface())
//   {
//     EXPECT_NEAR(1.0, joint_pos_[0], COMMON_THRESHOLD);
//     EXPECT_NEAR(2.0, joint_pos_[1], COMMON_THRESHOLD);
//     EXPECT_NEAR(3.0, joint_pos_[2], COMMON_THRESHOLD);
//   }

//   if (traj_controller_->has_velocity_command_interface())
//   {
//     EXPECT_GT(0.0, joint_vel_[0]);
//     EXPECT_GT(0.0, joint_vel_[1]);
//     EXPECT_GT(0.0, joint_vel_[2]);
//   }

//   if (traj_controller_->has_effort_command_interface())
//   {
//     EXPECT_GT(0.0, joint_eff_[0]);
//     EXPECT_GT(0.0, joint_eff_[1]);
//     EXPECT_GT(0.0, joint_eff_[2]);
//   }
// }

// /**
//  * @brief test_partial_joint_list Test sending trajectories with a subset of the controlled
//  * joints
//  */
// TEST_P(TrajectoryControllerTestParameterized, test_partial_joint_list)
// {
//   rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

//   const double initial_joint1_cmd = joint_pos_[0];
//   const double initial_joint2_cmd = joint_pos_[1];
//   const double initial_joint3_cmd = joint_pos_[2];
//   trajectory_msgs::msg::JointTrajectory traj_msg;

//   {
//     std::vector<std::string> partial_joint_names{joint_names_[1], joint_names_[0]};
//     traj_msg.joint_names = partial_joint_names;
//     traj_msg.header.stamp = rclcpp::Time(0);
//     traj_msg.points.resize(1);

//     traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
//     traj_msg.points[0].positions.resize(2);
//     traj_msg.points[0].positions[0] = 2.0;
//     traj_msg.points[0].positions[1] = 1.0;
//     traj_msg.points[0].velocities.resize(2);
//     traj_msg.points[0].velocities[0] =
//       copysign(2.0, traj_msg.points[0].positions[0] - initial_joint2_cmd);
//     traj_msg.points[0].velocities[1] =
//       copysign(1.0, traj_msg.points[0].positions[1] - initial_joint1_cmd);

//     trajectory_publisher_->publish(traj_msg);
//   }

//   traj_controller_->wait_for_trajectory(executor);
//   // update for 0.5 seconds
//   updateController(rclcpp::Duration::from_seconds(0.25));

//   double threshold = 0.001;

//   if (traj_controller_->has_position_command_interface())
//   {
//     EXPECT_NEAR(traj_msg.points[0].positions[1], joint_pos_[0], threshold);
//     EXPECT_NEAR(traj_msg.points[0].positions[0], joint_pos_[1], threshold);
//     EXPECT_NEAR(initial_joint3_cmd, joint_pos_[2], threshold)
//       << "Joint 3 command should be current position";
//   }

//   if (
//     std::find(command_interface_types_.begin(), command_interface_types_.end(), "velocity") !=
//     command_interface_types_.end())
//   {
//     // estimate the sign of the velocity
//     // joint rotates forward
//     EXPECT_TRUE(
//       is_same_sign(traj_msg.points[0].positions[0] - initial_joint2_cmd, joint_vel_[0]));
//     EXPECT_TRUE(
//       is_same_sign(traj_msg.points[0].positions[1] - initial_joint1_cmd, joint_vel_[1]));
//     EXPECT_NEAR(0.0, joint_vel_[2], threshold)
//       << "Joint 3 velocity should be 0.0 since it's not in the goal";
//   }

//   if (
//     std::find(command_interface_types_.begin(), command_interface_types_.end(), "effort") !=
//     command_interface_types_.end())
//   {
//     // estimate the sign of the velocity
//     // joint rotates forward
//     EXPECT_TRUE(
//       is_same_sign(traj_msg.points[0].positions[0] - initial_joint2_cmd, joint_eff_[0]));
//     EXPECT_TRUE(
//       is_same_sign(traj_msg.points[0].positions[1] - initial_joint1_cmd, joint_eff_[1]));
//     EXPECT_NEAR(0.0, joint_eff_[2], threshold)
//       << "Joint 3 effort should be 0.0 since it's not in the goal";
//   }
//   // TODO(anyone): add here ckecks for acceleration commands

//   executor.cancel();
// }

// /**
//  * @brief test_partial_joint_list Test sending trajectories with a subset of the controlled
//  * joints without allow_partial_joints_goal
//  */
// TEST_P(TrajectoryControllerTestParameterized, test_partial_joint_list_not_allowed)
// {
//   rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", false);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

//   const double initial_joint1_cmd = joint_pos_[0];
//   const double initial_joint2_cmd = joint_pos_[1];
//   const double initial_joint3_cmd = joint_pos_[2];
//   const double initial_joint_vel = 0.0;
//   const double initial_joint_acc = 0.0;
//   trajectory_msgs::msg::JointTrajectory traj_msg;

//   {
//     std::vector<std::string> partial_joint_names{joint_names_[1], joint_names_[0]};
//     traj_msg.joint_names = partial_joint_names;
//     traj_msg.header.stamp = rclcpp::Time(0);
//     traj_msg.points.resize(1);

//     traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
//     traj_msg.points[0].positions.resize(2);
//     traj_msg.points[0].positions[0] = 2.0;
//     traj_msg.points[0].positions[1] = 1.0;
//     traj_msg.points[0].velocities.resize(2);
//     traj_msg.points[0].velocities[0] = 2.0;
//     traj_msg.points[0].velocities[1] = 1.0;

//     trajectory_publisher_->publish(traj_msg);
//   }

//   traj_controller_->wait_for_trajectory(executor);
//   // update for 0.5 seconds
//   updateController(rclcpp::Duration::from_seconds(0.25));

//   double threshold = 0.001;
//   EXPECT_NEAR(initial_joint1_cmd, joint_pos_[0], threshold)
//     << "All joints command should be current position because goal was rejected";
//   EXPECT_NEAR(initial_joint2_cmd, joint_pos_[1], threshold)
//     << "All joints command should be current position because goal was rejected";
//   EXPECT_NEAR(initial_joint3_cmd, joint_pos_[2], threshold)
//     << "All joints command should be current position because goal was rejected";

//   if (
//     std::find(command_interface_types_.begin(), command_interface_types_.end(), "velocity") !=
//     command_interface_types_.end())
//   {
//     EXPECT_NEAR(initial_joint_vel, joint_vel_[0], threshold)
//       << "All joints velocities should be 0.0 because goal was rejected";
//     EXPECT_NEAR(initial_joint_vel, joint_vel_[1], threshold)
//       << "All joints velocities should be 0.0 because goal was rejected";
//     EXPECT_NEAR(initial_joint_vel, joint_vel_[2], threshold)
//       << "All joints velocities should be 0.0 because goal was rejected";
//   }

//   if (
//     std::find(
//       command_interface_types_.begin(), command_interface_types_.end(), "acceleration") !=
//       command_interface_types_.end())
//   {
//     EXPECT_NEAR(initial_joint_acc, joint_acc_[0], threshold)
//       << "All joints accelerations should be 0.0 because goal was rejected";
//     EXPECT_NEAR(initial_joint_acc, joint_acc_[1], threshold)
//       << "All joints accelerations should be 0.0 because goal was rejected";
//     EXPECT_NEAR(initial_joint_acc, joint_acc_[2], threshold)
//       << "All joints accelerations should be 0.0 because goal was rejected";
//   }

//   executor.cancel();
// }

// /**
//  * @brief invalid_message Test mismatched joint and reference vector sizes
//  */
// TEST_P(TrajectoryControllerTestParameterized, invalid_message)
// {
//   rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", false);
//   rclcpp::Parameter allow_integration_parameters(
//     "allow_integration_in_goal_trajectories", false);
//   rclcpp::executors::SingleThreadedExecutor executor;
//   SetUpAndActivateTrajectoryController(
//     true, {partial_joints_parameters, allow_integration_parameters}, &executor);

//   trajectory_msgs::msg::JointTrajectory traj_msg, good_traj_msg;

//   good_traj_msg.joint_names = joint_names_;
//   good_traj_msg.header.stamp = rclcpp::Time(0);
//   good_traj_msg.points.resize(1);
//   good_traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
//   good_traj_msg.points[0].positions.resize(1);
//   good_traj_msg.points[0].positions = {1.0, 2.0, 3.0};
//   good_traj_msg.points[0].velocities.resize(1);
//   good_traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
//   EXPECT_TRUE(traj_controller_->validate_trajectory_msg(good_traj_msg));

//   // Incompatible joint names
//   traj_msg = good_traj_msg;
//   traj_msg.joint_names = {"bad_name"};
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // No position data
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].positions.clear();
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // Incompatible data sizes, too few positions
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].positions = {1.0, 2.0};
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // Incompatible data sizes, too many positions
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].positions = {1.0, 2.0, 3.0, 4.0};
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // Incompatible data sizes, too few velocities
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].velocities = {1.0, 2.0};
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // Incompatible data sizes, too few accelerations
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].accelerations = {1.0, 2.0};
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // Incompatible data sizes, too few efforts
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].positions.clear();
//   traj_msg.points[0].effort = {1.0, 2.0};
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // Non-strictly increasing waypoint times
//   traj_msg = good_traj_msg;
//   traj_msg.points.push_back(traj_msg.points.front());
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));
// }

// /// With allow_integration_in_goal_trajectories parameter trajectory missing position or
// /// velocities are accepted
// TEST_P(TrajectoryControllerTestParameterized, missing_positions_message_accepted)
// {
//   rclcpp::Parameter allow_integration_parameters("allow_integration_in_goal_trajectories", true);
//   rclcpp::executors::SingleThreadedExecutor executor;
//   SetUpAndActivateTrajectoryController(true, {allow_integration_parameters}, &executor);

//   trajectory_msgs::msg::JointTrajectory traj_msg, good_traj_msg;

//   good_traj_msg.joint_names = joint_names_;
//   good_traj_msg.header.stamp = rclcpp::Time(0);
//   good_traj_msg.points.resize(1);
//   good_traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
//   good_traj_msg.points[0].positions.resize(1);
//   good_traj_msg.points[0].positions = {1.0, 2.0, 3.0};
//   good_traj_msg.points[0].velocities.resize(1);
//   good_traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
//   good_traj_msg.points[0].accelerations.resize(1);
//   good_traj_msg.points[0].accelerations = {1.0, 2.0, 3.0};
//   EXPECT_TRUE(traj_controller_->validate_trajectory_msg(good_traj_msg));

//   // No position data
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].positions.clear();
//   EXPECT_TRUE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // No position and velocity data
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].positions.clear();
//   traj_msg.points[0].velocities.clear();
//   EXPECT_TRUE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // All empty
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].positions.clear();
//   traj_msg.points[0].velocities.clear();
//   traj_msg.points[0].accelerations.clear();
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // Incompatible data sizes, too few positions
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].positions = {1.0, 2.0};
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // Incompatible data sizes, too many positions
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].positions = {1.0, 2.0, 3.0, 4.0};
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // Incompatible data sizes, too few velocities
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].velocities = {1.0};
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

//   // Incompatible data sizes, too few accelerations
//   traj_msg = good_traj_msg;
//   traj_msg.points[0].accelerations = {2.0};
//   EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));
// }

// /**
//  * @brief test_trajectory_replace Test replacing an existing trajectory
//  */
// TEST_P(TrajectoryControllerTestParameterized, test_trajectory_replace)
// {
//   rclcpp::executors::SingleThreadedExecutor executor;
//   rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);
//   SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

//   subscribeToState();

//   std::vector<std::vector<double>> points_old{{{2., 3., 4.}}};
//   std::vector<std::vector<double>> points_old_velocities{{{0.2, 0.3, 0.4}}};
//   std::vector<std::vector<double>> points_partial_new{{1.5}};
//   std::vector<std::vector<double>> points_partial_new_velocities{{0.15}};

//   const auto delay = std::chrono::milliseconds(500);
//   builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
//   publish(time_from_start, points_old, rclcpp::Time(), {}, points_old_velocities);
//   trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
//   expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
//   expected_desired.positions = {points_old[0].begin(), points_old[0].end()};
//   expected_actual.velocities = {
//     points_old_velocities[0].begin(), points_old_velocities[0].end()};
//   expected_desired.velocities = {
//     points_old_velocities[0].begin(), points_old_velocities[0].end()};
//   //  Check that we reached end of points_old trajectory
//   // Denis: delta was 0.1 with 0.2 works for me
//   waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.2);

//   RCLCPP_INFO(traj_controller_->get_node()->get_logger(), "Sending new trajectory");
//   points_partial_new_velocities[0][0] =
//     copysign(0.15, points_partial_new[0][0] - joint_state_pos_[0]);
//   publish(
//     time_from_start, points_partial_new, rclcpp::Time(), {}, points_partial_new_velocities);

//   // Replaced trajectory is a mix of previous and current goal
//   expected_desired.positions[0] = points_partial_new[0][0];
//   expected_desired.positions[1] = points_old[0][1];
//   expected_desired.positions[2] = points_old[0][2];
//   expected_desired.velocities[0] = points_partial_new_velocities[0][0];
//   expected_desired.velocities[1] = 0.0;
//   expected_desired.velocities[2] = 0.0;
//   expected_actual = expected_desired;
//   waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);
// }

// /**
//  * @brief test_ignore_old_trajectory Sending an old trajectory replacing an existing trajectory
//  */
// TEST_P(TrajectoryControllerTestParameterized, test_ignore_old_trajectory)
// {
//   rclcpp::executors::SingleThreadedExecutor executor;
//   SetUpAndActivateTrajectoryController(true, {}, &executor);
//   subscribeToState();

//   // TODO(anyone): add expectations for velocities and accelerations
//   std::vector<std::vector<double>> points_old{{{2., 3., 4.}, {4., 5., 6.}}};
//   std::vector<std::vector<double>> points_new{{{-1., -2., -3.}}};

//   RCLCPP_INFO(
//     traj_controller_->get_node()->get_logger(), "Sending new trajectory in the future");
//   const auto delay = std::chrono::milliseconds(500);
//   builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
//   publish(time_from_start, points_old, rclcpp::Time());
//   trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
//   expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
//   expected_desired = expected_actual;
//   //  Check that we reached end of points_old[0] trajectory
//   waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

//   RCLCPP_INFO(traj_controller_->get_node()->get_logger(), "Sending new trajectory in the past");
//   //  New trajectory will end before current time
//   rclcpp::Time new_traj_start = rclcpp::Clock().now() - delay - std::chrono::milliseconds(100);
//   expected_actual.positions = {points_old[1].begin(), points_old[1].end()};
//   expected_desired = expected_actual;
//   std::cout << "Sending old trajectory" << std::endl;
//   publish(time_from_start, points_new, new_traj_start);
//   waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);
// }

// TEST_P(TrajectoryControllerTestParameterized, test_ignore_partial_old_trajectory)
// {
//   rclcpp::executors::SingleThreadedExecutor executor;
//   SetUpAndActivateTrajectoryController(true, {}, &executor);
//   subscribeToState();

//   std::vector<std::vector<double>> points_old{{{2., 3., 4.}, {4., 5., 6.}}};
//   std::vector<std::vector<double>> points_new{{{-1., -2., -3.}, {-2., -4., -6.}}};

//   const auto delay = std::chrono::milliseconds(500);
//   builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
//   publish(time_from_start, points_old, rclcpp::Time());
//   trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
//   expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
//   expected_desired = expected_actual;
//   //  Check that we reached end of points_old[0]trajectory
//   waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

//   RCLCPP_INFO(
//     traj_controller_->get_node()->get_logger(), "Sending new trajectory partially in the past");
//   //  New trajectory first point is in the past, second is in the future
//   rclcpp::Time new_traj_start = rclcpp::Clock().now() - delay - std::chrono::milliseconds(100);
//   expected_actual.positions = {points_new[1].begin(), points_new[1].end()};
//   expected_desired = expected_actual;
//   publish(time_from_start, points_new, new_traj_start);
//   waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);
// }

// TEST_P(TrajectoryControllerTestParameterized, test_execute_partial_traj_in_future)
// {
//   SetUpTrajectoryController();
//   auto traj_node = traj_controller_->get_node();
//   RCLCPP_WARN(
//     traj_node->get_logger(),
//   "Test disabled until current_trajectory is taken into account when adding a new trajectory.");
//   // https://github.com/ros-controls/ros_controllers/blob/melodic-devel/
//   // joint_trajectory_controller/include/joint_trajectory_controller/init_joint_trajectory.h#L149
//   return;

//   // TODO(anyone): use SetUpAndActivateTrajectoryController method instead of the next line
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(traj_node->get_node_base_interface());
//   subscribeToState();
//   rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);
//   traj_node->set_parameter(partial_joints_parameters);
//   traj_controller_->get_node()->configure();
//   traj_controller_->get_node()->activate();

//   std::vector<std::vector<double>> full_traj{{{2., 3., 4.}, {4., 6., 8.}}};
//   std::vector<std::vector<double>> full_traj_velocities{{{0.2, 0.3, 0.4}, {0.4, 0.6, 0.8}}};
//   std::vector<std::vector<double>> partial_traj{
//     {{-1., -2.},
//      {
//        -2.,
//        -4,
//      }}};
//   std::vector<std::vector<double>> partial_traj_velocities{
//     {{-0.1, -0.2},
//      {
//        -0.2,
//        -0.4,
//      }}};
//   const auto delay = std::chrono::milliseconds(500);
//   builtin_interfaces::msg::Duration points_delay{rclcpp::Duration(delay)};
//   // Send full trajectory
//   publish(points_delay, full_traj, rclcpp::Time(), {}, full_traj_velocities);
//   // Sleep until first waypoint of full trajectory

//   trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
//   expected_actual.positions = {full_traj[0].begin(), full_traj[0].end()};
//   expected_desired = expected_actual;
//   //  Check that we reached end of points_old[0]trajectory and are starting points_old[1]
//   waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

//   // Send partial trajectory starting after full trajecotry is complete
//   RCLCPP_INFO(traj_node->get_logger(), "Sending new trajectory in the future");
//   publish(
//     points_delay, partial_traj, rclcpp::Clock().now() + delay * 2, {}, partial_traj_velocities);
//   // Wait until the end start and end of partial traj

//   expected_actual.positions = {
//     partial_traj.back()[0], partial_traj.back()[1], full_traj.back()[2]};
//   expected_desired = expected_actual;

//   waitAndCompareState(
//     expected_actual, expected_desired, executor, rclcpp::Duration(delay * (2 + 2)), 0.1);
// }

// TEST_P(TrajectoryControllerTestParameterized, test_jump_when_state_tracking_error_updated)
// {
//   rclcpp::executors::SingleThreadedExecutor executor;
//   // default if false so it will not be actually set parameter
//   rclcpp::Parameter is_open_loop_parameters("open_loop_control", false);
//   SetUpAndActivateTrajectoryController(true, {is_open_loop_parameters}, &executor, true);

//   // goal setup
//   std::vector<double> first_goal = {3.3, 4.4, 5.5};
//   std::vector<std::vector<double>> first_goal_velocities = {{0.33, 0.44, 0.55}};
//   std::vector<double> second_goal = {6.6, 8.8, 11.0};
//   std::vector<std::vector<double>> second_goal_velocities = {{0.66, 0.88, 1.1}};
//   double state_from_command_offset = 0.3;

//   // send msg
//   builtin_interfaces::msg::Duration time_from_start;
//   time_from_start.sec = 1;
//   time_from_start.nanosec = 0;
//   std::vector<std::vector<double>> points{{first_goal}};
//   publish(time_from_start, points, rclcpp::Time(), {}, first_goal_velocities);
//   traj_controller_->wait_for_trajectory(executor);
//   updateController(rclcpp::Duration::from_seconds(1.1));

//   if (traj_controller_->has_position_command_interface())
//   {
//     // JTC is executing trajectory in open-loop therefore:
//     // - internal state does not have to be updated (in this test-case it shouldn't)
//     // - internal command is updated
//     EXPECT_NEAR(INITIAL_POS_JOINT1, joint_state_pos_[0], COMMON_THRESHOLD);
//     EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);

//     // State interface should have offset from the command before starting a new trajectory
//     joint_state_pos_[0] = first_goal[0] - state_from_command_offset;

//     // Move joint further in the same direction as before (to the second goal)
//     points = {{second_goal}};
//     publish(time_from_start, points, rclcpp::Time(1.0), {}, second_goal_velocities);
//     traj_controller_->wait_for_trajectory(executor);

//     // One the first update(s) there should be a "jump" in opposite direction from command
//     // (towards the state value)
//     EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     // Expect backward commands at first
//     EXPECT_NEAR(joint_state_pos_[0], joint_pos_[0], COMMON_THRESHOLD);
//     EXPECT_GT(joint_pos_[0], joint_state_pos_[0]);
//     EXPECT_LT(joint_pos_[0], first_goal[0]);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     EXPECT_GT(joint_pos_[0], joint_state_pos_[0]);
//     EXPECT_LT(joint_pos_[0], first_goal[0]);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     EXPECT_GT(joint_pos_[0], joint_state_pos_[0]);
//     EXPECT_LT(joint_pos_[0], first_goal[0]);

//     // Finally the second goal will be commanded/reached
//     updateController(rclcpp::Duration::from_seconds(1.1));
//     EXPECT_NEAR(second_goal[0], joint_pos_[0], COMMON_THRESHOLD);

//     // State interface should have offset from the command before starting a new trajectory
//     joint_state_pos_[0] = second_goal[0] - state_from_command_offset;

//     // Move joint back to the first goal
//     points = {{first_goal}};
//     publish(time_from_start, points, rclcpp::Time());
//     traj_controller_->wait_for_trajectory(executor);

//     // One the first update(s) there should be a "jump" in the goal direction from command
//     // (towards the state value)
//     EXPECT_NEAR(second_goal[0], joint_pos_[0], COMMON_THRESHOLD);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     // Expect backward commands at first
//     EXPECT_NEAR(joint_state_pos_[0], joint_pos_[0], COMMON_THRESHOLD);
//     EXPECT_LT(joint_pos_[0], joint_state_pos_[0]);
//     EXPECT_GT(joint_pos_[0], first_goal[0]);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     EXPECT_LT(joint_pos_[0], joint_state_pos_[0]);
//     EXPECT_GT(joint_pos_[0], first_goal[0]);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     EXPECT_LT(joint_pos_[0], joint_state_pos_[0]);
//     EXPECT_GT(joint_pos_[0], first_goal[0]);

//     // Finally the first goal will be commanded/reached
//     updateController(rclcpp::Duration::from_seconds(1.1));
//     EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);
//   }

//   executor.cancel();
// }

// TEST_P(TrajectoryControllerTestParameterized, test_no_jump_when_state_tracking_error_not_updated)
// {
//   rclcpp::executors::SingleThreadedExecutor executor;
//   // default if false so it will not be actually set parameter
//   rclcpp::Parameter is_open_loop_parameters("open_loop_control", true);
//   SetUpAndActivateTrajectoryController(true, {is_open_loop_parameters}, &executor, true);

//   // goal setup
//   std::vector<double> first_goal = {3.3, 4.4, 5.5};
//   std::vector<double> second_goal = {6.6, 8.8, 11.0};
//   double state_from_command_offset = 0.3;

//   // send msg
//   builtin_interfaces::msg::Duration time_from_start;
//   time_from_start.sec = 1;
//   time_from_start.nanosec = 0;
//   std::vector<std::vector<double>> points{{first_goal}};
//   publish(time_from_start, points, rclcpp::Time());
//   traj_controller_->wait_for_trajectory(executor);
//   updateController(rclcpp::Duration::from_seconds(1.1));

//   if (traj_controller_->has_position_command_interface())
//   {
//     // JTC is executing trajectory in open-loop therefore:
//     // - internal state does not have to be updated (in this test-case it shouldn't)
//     // - internal command is updated
//     EXPECT_NEAR(INITIAL_POS_JOINT1, joint_state_pos_[0], COMMON_THRESHOLD);
//     EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);

//     // State interface should have offset from the command before starting a new trajectory
//     joint_state_pos_[0] = first_goal[0] - state_from_command_offset;

//     // Move joint further in the same direction as before (to the second goal)
//     points = {{second_goal}};
//     publish(time_from_start, points, rclcpp::Time());
//     traj_controller_->wait_for_trajectory(executor);

//     // One the first update(s) there **should not** be a "jump" in opposite direction from
//     // command (towards the state value)
//     EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     // There should not be backward commands
//     EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);
//     EXPECT_GT(joint_pos_[0], first_goal[0]);
//     EXPECT_LT(joint_pos_[0], second_goal[0]);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     EXPECT_GT(joint_pos_[0], first_goal[0]);
//     EXPECT_LT(joint_pos_[0], second_goal[0]);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     EXPECT_GT(joint_pos_[0], first_goal[0]);
//     EXPECT_LT(joint_pos_[0], second_goal[0]);

//     // Finally the second goal will be commanded/reached
//     updateController(rclcpp::Duration::from_seconds(1.1));
//     EXPECT_NEAR(second_goal[0], joint_pos_[0], COMMON_THRESHOLD);

//     // State interface should have offset from the command before starting a new trajectory
//     joint_state_pos_[0] = second_goal[0] - state_from_command_offset;

//     // Move joint back to the first goal
//     points = {{first_goal}};
//     publish(time_from_start, points, rclcpp::Time());
//     traj_controller_->wait_for_trajectory(executor);

//     // One the first update(s) there **should not** be a "jump" in the goal direction from
//     // command (towards the state value)
//     EXPECT_NEAR(second_goal[0], joint_pos_[0], COMMON_THRESHOLD);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     // There should not be a jump toward commands
//     EXPECT_NEAR(second_goal[0], joint_pos_[0], COMMON_THRESHOLD);
//     EXPECT_LT(joint_pos_[0], second_goal[0]);
//     EXPECT_GT(joint_pos_[0], first_goal[0]);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     EXPECT_GT(joint_pos_[0], first_goal[0]);
//     EXPECT_LT(joint_pos_[0], second_goal[0]);
//     traj_controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     EXPECT_GT(joint_pos_[0], first_goal[0]);
//     EXPECT_LT(joint_pos_[0], second_goal[0]);

//     // Finally the first goal will be commanded/reached
//     updateController(rclcpp::Duration::from_seconds(1.1));
//     EXPECT_NEAR(first_goal[0], joint_pos_[0], COMMON_THRESHOLD);
//   }

//   executor.cancel();
// }

// // Testing that values are read from state interfaces when hardware is started for the first
// // time and hardware state has offset --> this is indicated by NaN values in state interfaces
// TEST_P(TrajectoryControllerTestParameterized, test_hw_states_has_offset_first_controller_start)
// {
//   rclcpp::executors::SingleThreadedExecutor executor;
//   // default if false so it will not be actually set parameter
//   rclcpp::Parameter is_open_loop_parameters("open_loop_control", true);

//   // set command values to NaN
//   for (size_t i = 0; i < 3; ++i)
//   {
//     joint_pos_[i] = std::numeric_limits<double>::quiet_NaN();
//     joint_vel_[i] = std::numeric_limits<double>::quiet_NaN();
//     joint_acc_[i] = std::numeric_limits<double>::quiet_NaN();
//   }
//   SetUpAndActivateTrajectoryController(true, {is_open_loop_parameters}, &executor, true);

//   auto current_state_when_offset = traj_controller_->get_current_state_when_offset();

//   for (size_t i = 0; i < 3; ++i)
//   {
//     EXPECT_EQ(current_state_when_offset.positions[i], joint_state_pos_[i]);

//     // check velocity
//     if (
//       std::find(
//         state_interface_types_.begin(), state_interface_types_.end(),
//         hardware_interface::HW_IF_VELOCITY) != state_interface_types_.end() &&
//       std::find(
//         command_interface_types_.begin(), command_interface_types_.end(),
//         hardware_interface::HW_IF_VELOCITY) != command_interface_types_.end())
//     {
//       EXPECT_EQ(current_state_when_offset.positions[i], joint_state_pos_[i]);
//     }

//     // check acceleration
//     if (
//       std::find(
//         state_interface_types_.begin(), state_interface_types_.end(),
//         hardware_interface::HW_IF_ACCELERATION) != state_interface_types_.end() &&
//       std::find(
//         command_interface_types_.begin(), command_interface_types_.end(),
//         hardware_interface::HW_IF_ACCELERATION) != command_interface_types_.end())
//     {
//       EXPECT_EQ(current_state_when_offset.positions[i], joint_state_pos_[i]);
//     }
//   }

//   executor.cancel();
// }

// // Testing that values are read from state interfaces when hardware is started after some values
// // are set on the hardware commands
// TEST_P(TrajectoryControllerTestParameterized, test_hw_states_has_offset_later_controller_start)
// {
//   rclcpp::executors::SingleThreadedExecutor executor;
//   // default if false so it will not be actually set parameter
//   rclcpp::Parameter is_open_loop_parameters("open_loop_control", true);

//   // set command values to NaN
//   for (size_t i = 0; i < 3; ++i)
//   {
//     joint_pos_[i] = 3.1 + i;
//     joint_vel_[i] = 0.25 + i;
//     joint_acc_[i] = 0.02 + i / 10.0;
//   }
//   SetUpAndActivateTrajectoryController(true, {is_open_loop_parameters}, &executor, true);

//   auto current_state_when_offset = traj_controller_->get_current_state_when_offset();

//   for (size_t i = 0; i < 3; ++i)
//   {
//     EXPECT_EQ(current_state_when_offset.positions[i], joint_pos_[i]);

//     // check velocity
//     if (
//       std::find(
//         state_interface_types_.begin(), state_interface_types_.end(),
//         hardware_interface::HW_IF_VELOCITY) != state_interface_types_.end() &&
//       std::find(
//         command_interface_types_.begin(), command_interface_types_.end(),
//         hardware_interface::HW_IF_VELOCITY) != command_interface_types_.end())
//     {
//       EXPECT_EQ(current_state_when_offset.positions[i], joint_pos_[i]);
//     }

//     // check acceleration
//     if (
//       std::find(
//         state_interface_types_.begin(), state_interface_types_.end(),
//         hardware_interface::HW_IF_ACCELERATION) != state_interface_types_.end() &&
//       std::find(
//         command_interface_types_.begin(), command_interface_types_.end(),
//         hardware_interface::HW_IF_ACCELERATION) != command_interface_types_.end())
//     {
//       EXPECT_EQ(current_state_when_offset.positions[i], joint_pos_[i]);
//     }
//   }

//   executor.cancel();
// }

// position controllers
INSTANTIATE_TEST_SUITE_P(
  PositionTrajectoryControllers, TrajectoryControllerTestParameterized,
  ::testing::Values(
    std::make_tuple(std::vector<std::string>({"position"}), std::vector<std::string>({"position"})),
    std::make_tuple(
      std::vector<std::string>({"position"}), std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"position"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

// position_velocity controllers
INSTANTIATE_TEST_SUITE_P(
  PositionVelocityTrajectoryControllers, TrajectoryControllerTestParameterized,
  ::testing::Values(
    std::make_tuple(
      std::vector<std::string>({"position", "velocity"}), std::vector<std::string>({"position"})),
    std::make_tuple(
      std::vector<std::string>({"position", "velocity"}),
      std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"position", "velocity"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

// position_velocity_acceleration controllers
INSTANTIATE_TEST_SUITE_P(
  PositionVelocityAccelerationTrajectoryControllers, TrajectoryControllerTestParameterized,
  ::testing::Values(
    std::make_tuple(
      std::vector<std::string>({"position", "velocity", "acceleration"}),
      std::vector<std::string>({"position"})),
    std::make_tuple(
      std::vector<std::string>({"position", "velocity", "acceleration"}),
      std::vector<std::string>({"position", "velocity"})),
    std::make_tuple(
      std::vector<std::string>({"position", "velocity", "acceleration"}),
      std::vector<std::string>({"position", "velocity", "acceleration"}))));

// // only velocity controller
// INSTANTIATE_TEST_SUITE_P(
//   OnlyVelocityTrajectoryControllers, TrajectoryControllerTestParameterized,
//   ::testing::Values(
//     std::make_tuple(
//       std::vector<std::string>({"velocity"}),
//       std::vector<std::string>({"position", "velocity"})),
//     std::make_tuple(
//       std::vector<std::string>({"velocity"}),
//       std::vector<std::string>({"position", "velocity", "acceleration"}))));

// // only effort controller
// INSTANTIATE_TEST_SUITE_P(
//   OnlyEffortTrajectoryControllers, TrajectoryControllerTestParameterized,
//   ::testing::Values(
//     std::make_tuple(
//       std::vector<std::string>({"effort"}), std::vector<std::string>({"position", "velocity"})),
//     std::make_tuple(
//       std::vector<std::string>({"effort"}),
//       std::vector<std::string>({"position", "velocity", "acceleration"}))));

// TODO(destogl): this tests should be changed because we are using `generate_parameters_library`
// TEST_F(TrajectoryControllerTest, incorrect_initialization_using_interface_parameters)
// {
//   auto set_parameter_and_check_result = [&]()
//   {
//     EXPECT_EQ(traj_controller_->get_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
//     SetParameters();  // This call is replacing the way parameters are set via launch
//     traj_controller_->get_node()->configure();
//     EXPECT_EQ(traj_controller_->get_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
//   };
//
//   SetUpTrajectoryController(false);
//
//   // command interfaces: empty
//   command_interface_types_ = {};
//   set_parameter_and_check_result();
//
//   // command interfaces: bad_name
//   command_interface_types_ = {"bad_name"};
//   set_parameter_and_check_result();
//
//   // command interfaces: effort has to be only
//   command_interface_types_ = {"effort", "position"};
//   set_parameter_and_check_result();
//
//   // command interfaces: velocity - position not present
//   command_interface_types_ = {"velocity", "acceleration"};
//   set_parameter_and_check_result();
//
//   // command interfaces: acceleration without position and velocity
//   command_interface_types_ = {"acceleration"};
//   set_parameter_and_check_result();
//
//   // state interfaces: empty
//   state_interface_types_ = {};
//   set_parameter_and_check_result();
//
//   // state interfaces: cannot not be effort
//   state_interface_types_ = {"effort"};
//   set_parameter_and_check_result();
//
//   // state interfaces: bad name
//   state_interface_types_ = {"bad_name"};
//   set_parameter_and_check_result();
//
//   // state interfaces: velocity - position not present
//   state_interface_types_ = {"velocity"};
//   set_parameter_and_check_result();
//   state_interface_types_ = {"velocity", "acceleration"};
//   set_parameter_and_check_result();
//
//   // state interfaces: acceleration without position and velocity
//   state_interface_types_ = {"acceleration"};
//   set_parameter_and_check_result();
//
//   // velocity-only command interface: position - velocity not present
//   command_interface_types_ = {"velocity"};
//   state_interface_types_ = {"position"};
//   set_parameter_and_check_result();
//   state_interface_types_ = {"velocity"};
//   set_parameter_and_check_result();
//
//   // effort-only command interface: position - velocity not present
//   command_interface_types_ = {"effort"};
//   state_interface_types_ = {"position"};
//   set_parameter_and_check_result();
//   state_interface_types_ = {"velocity"};
//   set_parameter_and_check_result();
// }
