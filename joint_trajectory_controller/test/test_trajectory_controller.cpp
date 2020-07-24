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
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "control_msgs/msg/detail/joint_trajectory_controller_state__struct.hpp"
#include "controller_interface/controller_interface.hpp"
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
#include "test_robot_hardware/test_robot_hardware.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using lifecycle_msgs::msg::State;

namespace
{
const double COMMON_THRESHOLD = 0.001;
}

void
spin(rclcpp::executors::MultiThreadedExecutor * exe)
{
  exe->spin();
}

class TestTrajectoryController : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    test_robot_ = std::make_shared<test_robot_hardware::TestRobotHardware>();
    test_robot_->init();
    joint_names_ = {{test_robot_->joint_name1, test_robot_->joint_name2, test_robot_->joint_name3}};
    op_mode_ = {{test_robot_->write_op_handle_name1}};

    pub_node_ = std::make_shared<rclcpp::Node>("trajectory_publisher_");
    trajectory_publisher_ = pub_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      controller_name_ + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  }

  void SetUpTrajectoryController(bool use_local_parameters = true)
  {
    if (use_local_parameters) {
      traj_controller_ = std::make_shared<joint_trajectory_controller::JointTrajectoryController>(
        joint_names_, op_mode_);
    } else {
      traj_controller_ = std::make_shared<joint_trajectory_controller::JointTrajectoryController>();
    }
    auto ret = traj_controller_->init(test_robot_, controller_name_);
    if (ret != controller_interface::return_type::SUCCESS) {
      FAIL();
    }
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  /// Publish trajectory msgs with multiple points
  /**
   *  delay_btwn_points - delay between each points
   *  points - vector of trajectories. Each trajectory consists of 3 joints
   */
  void publish(
    const builtin_interfaces::msg::Duration & delay_btwn_points,
    const std::vector<std::array<double, 3>> & points)
  {
    int wait_count = 0;
    const auto topic = trajectory_publisher_->get_topic_name();
    while (pub_node_->count_subscribers(topic) == 0) {
      if (wait_count >= 5) {
        auto error_msg =
          std::string("publishing to ") + topic + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.joint_names = joint_names_;
    traj_msg.header.stamp.sec = 0;
    traj_msg.header.stamp.nanosec = 0;
    traj_msg.points.resize(points.size());

    builtin_interfaces::msg::Duration duration_msg;
    duration_msg.sec = delay_btwn_points.sec;
    duration_msg.nanosec = delay_btwn_points.nanosec;
    rclcpp::Duration duration(duration_msg);
    rclcpp::Duration duration_total(duration_msg);

    for (size_t index = 0; index < points.size(); ++index) {
      traj_msg.points[index].time_from_start = duration_total;

      traj_msg.points[index].positions.resize(3);
      traj_msg.points[index].positions[0] = points[index][0];
      traj_msg.points[index].positions[1] = points[index][1];
      traj_msg.points[index].positions[2] = points[index][2];
      duration_total = duration_total + duration;
    }

    trajectory_publisher_->publish(traj_msg);
  }

  std::string controller_name_ = "test_joint_trajectory_controller";

  std::shared_ptr<test_robot_hardware::TestRobotHardware> test_robot_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> op_mode_;

  rclcpp::Node::SharedPtr pub_node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

  std::shared_ptr<joint_trajectory_controller::JointTrajectoryController> traj_controller_;
};

TEST_F(TestTrajectoryController, wrong_initialization) {
  const auto uninitialized_robot = std::make_shared<test_robot_hardware::TestRobotHardware>();
  auto traj_controller = std::make_shared<joint_trajectory_controller::JointTrajectoryController>(
    joint_names_, op_mode_);
  const auto ret = traj_controller->init(uninitialized_robot, controller_name_);
  if (ret != controller_interface::return_type::SUCCESS) {
    FAIL();
  }

  const auto unconfigured_state = traj_controller->get_lifecycle_node()->configure();
  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, unconfigured_state.id());
}

TEST_F(TestTrajectoryController, correct_initialization) {
  const auto initialized_robot = std::make_shared<test_robot_hardware::TestRobotHardware>();
  initialized_robot->init();
  auto traj_controller = std::make_shared<joint_trajectory_controller::JointTrajectoryController>(
    joint_names_, op_mode_);
  const auto ret = traj_controller->init(initialized_robot, controller_name_);
  if (ret != controller_interface::return_type::SUCCESS) {
    FAIL();
  }

  const auto inactive_state = traj_controller->get_lifecycle_node()->configure();
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, inactive_state.id());
  EXPECT_EQ(1.1, initialized_robot->pos1);
  EXPECT_EQ(2.2, initialized_robot->pos2);
  EXPECT_EQ(3.3, initialized_robot->pos3);
}

TEST_F(TestTrajectoryController, configuration) {
  SetUpTrajectoryController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(traj_controller_->get_lifecycle_node()->get_node_base_interface());
  const auto future_handle_ = std::async(std::launch::async, spin, &executor);

  const auto state = traj_controller_->get_lifecycle_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  // send msg
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 1;
  time_from_start.nanosec = 0;
  std::vector<std::array<double, 3>> points {{{3.3, 4.4, 5.5}}};
  publish(time_from_start, points);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  traj_controller_->update();
  test_robot_->write();

  // no change in hw position
  EXPECT_NE(3.3, test_robot_->pos1);
  EXPECT_NE(4.4, test_robot_->pos2);
  EXPECT_NE(5.5, test_robot_->pos3);

  executor.cancel();
}

// TEST_F(TestTrajectoryController, activation) {
//   auto traj_controller = std::make_shared<ros_controllers::JointTrajectoryController>(
//     joint_names_, op_mode_);
//   auto ret = traj_controller->init(test_robot_, controller_name_);
//   if (ret != controller_interface::return_type::SUCCESS) {
//     FAIL();
//   }
//
//   auto traj_lifecycle_node = traj_controller->get_lifecycle_node();
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(traj_lifecycle_node->get_node_base_interface());
//
//   auto state = traj_lifecycle_node->configure();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
//
//   state = traj_lifecycle_node->activate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);
//
//   // wait for the subscriber and publisher to completely setup
//   std::this_thread::sleep_for(std::chrono::seconds(2));
//
//   // send msg
//   builtin_interfaces::msg::Duration time_from_start;
//   time_from_start.sec = 1;
//   time_from_start.nanosec = 0;
//   std::vector<std::array<double, 3>> points {{{3.3, 4.4, 5.5}}};
//   publish(time_from_start, points);
//   // wait for msg is be published to the system
//   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//   executor.spin_once();
//
//   traj_controller->update();
//   test_robot_->write();
//
//   // change in hw position
//   EXPECT_EQ(3.3, test_robot_->pos1);
//   EXPECT_EQ(4.4, test_robot_->pos2);
//   EXPECT_EQ(5.5, test_robot_->pos3);
//
//   executor.cancel();
// }

// TEST_F(TestTrajectoryController, reactivation) {
//   auto traj_controller = std::make_shared<ros_controllers::JointTrajectoryController>(
//     joint_names_, op_mode_);
//   auto ret = traj_controller->init(test_robot_, controller_name_);
//   if (ret != controller_interface::return_type::SUCCESS) {
//     FAIL();
//   }
//
//   auto traj_lifecycle_node = traj_controller->get_lifecycle_node();
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(traj_lifecycle_node->get_node_base_interface());
//
//   auto state = traj_lifecycle_node->configure();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
//
//   state = traj_lifecycle_node->activate();
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
//   std::vector<std::array<double, 3>> points {
//     {{3.3, 4.4, 5.5}},
//     {{7.7, 8.8, 9.9}},
//     {{10.10, 11.11, 12.12}}
//   };
//   // *INDENT-ON*
//   publish(time_from_start, points);
//   // wait for msg is be published to the system
//   std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   executor.spin_once();
//
//   traj_controller->update();
//   test_robot_->write();
//
//   // deactivated
//   // wait so controller process the second point when deactivated
//   std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   state = traj_lifecycle_node->deactivate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
//   test_robot_->read();
//   traj_controller->update();
//   test_robot_->write();
//
//   // no change in hw position
//   EXPECT_EQ(3.3, test_robot_->pos1);
//   EXPECT_EQ(4.4, test_robot_->pos2);
//   EXPECT_EQ(5.5, test_robot_->pos3);
//
//   // reactivated
//   // wait so controller process the third point when reactivated
//   std::this_thread::sleep_for(std::chrono::milliseconds(3000));
//   state = traj_lifecycle_node->activate();
//   ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);
//   test_robot_->read();
//   traj_controller->update();
//   test_robot_->write();
//
//   // change in hw position to 3rd point
//   EXPECT_EQ(10.10, test_robot_->pos1);
//   EXPECT_EQ(11.11, test_robot_->pos2);
//   EXPECT_EQ(12.12, test_robot_->pos3);
//
//   executor.cancel();
// }

TEST_F(TestTrajectoryController, cleanup) {
  SetUpTrajectoryController();

  auto traj_lifecycle_node = traj_controller_->get_lifecycle_node();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(traj_lifecycle_node->get_node_base_interface());

  auto state = traj_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());

  state = traj_lifecycle_node->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  // wait for the subscriber and publisher to completely setup
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // send msg
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 1;
  time_from_start.nanosec = 0;
  std::vector<std::array<double, 3>> points {{{3.3, 4.4, 5.5}}};
  publish(time_from_start, points);
  // wait for msg is be published to the system
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  executor.spin_once();

  traj_controller_->update();
  test_robot_->write();

  state = traj_lifecycle_node->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  traj_controller_->update();
  test_robot_->write();

  state = traj_lifecycle_node->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  traj_controller_->update();
  test_robot_->write();

  // should be home pose again
  EXPECT_NEAR(1.1, test_robot_->pos1, COMMON_THRESHOLD);
  EXPECT_NEAR(2.2, test_robot_->pos2, COMMON_THRESHOLD);
  EXPECT_NEAR(3.3, test_robot_->pos3, COMMON_THRESHOLD);

  executor.cancel();
}

TEST_F(TestTrajectoryController, correct_initialization_using_parameters) {
  SetUpTrajectoryController(false);

  // This block is replacing the way parameters are set via launch
  auto traj_lifecycle_node = traj_controller_->get_lifecycle_node();
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};
  const rclcpp::Parameter joint_parameters("joints", joint_names_);
  traj_lifecycle_node->set_parameter(joint_parameters);

  const std::vector<std::string> operation_mode_names = {"write1", "write2"};
  rclcpp::Parameter operation_mode_parameters("write_op_modes", operation_mode_names);
  traj_lifecycle_node->set_parameter(operation_mode_parameters);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(traj_lifecycle_node->get_node_base_interface());

  auto future_handle = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });

  auto state = traj_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_EQ(1.1, test_robot_->pos1);
  EXPECT_EQ(2.2, test_robot_->pos2);
  EXPECT_EQ(3.3, test_robot_->pos3);

  state = traj_lifecycle_node->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  // wait for the subscriber and publisher to completely setup
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // send msg
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 1;
  time_from_start.nanosec = 0;
  // *INDENT-OFF*
  std::vector<std::array<double, 3>> points {
    {{3.3, 4.4, 5.5}},
    {{7.7, 8.8, 9.9}},
    {{10.10, 11.11, 12.12}}
  };
  // *INDENT-ON*
  publish(time_from_start, points);
  // wait for msg is be published to the system
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // first update
  traj_controller_->update();
  test_robot_->write();

  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  traj_controller_->update();
  test_robot_->write();

  // deactivated
  state = traj_lifecycle_node->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  const auto allowed_delta = 0.05;
  EXPECT_NEAR(3.3, test_robot_->pos1, allowed_delta);
  EXPECT_NEAR(4.4, test_robot_->pos2, allowed_delta);
  EXPECT_NEAR(5.5, test_robot_->pos3, allowed_delta);

  // cleanup
  state = traj_lifecycle_node->cleanup();

  // update loop receives a new msg and updates accordingly
  traj_controller_->update();
  test_robot_->write();

  // check the traj_msg_home_ptr_ initialization code for the standard wait timing
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  traj_controller_->update();
  test_robot_->write();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  EXPECT_NEAR(1.1, test_robot_->pos1, allowed_delta);
  EXPECT_NEAR(2.2, test_robot_->pos2, allowed_delta);
  EXPECT_NEAR(3.3, test_robot_->pos3, allowed_delta);

  state = traj_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}

void test_state_publish_rate_target(
  std::shared_ptr<joint_trajectory_controller::JointTrajectoryController> traj_controller,
  int target_msg_count)
{
  // fill in some data so we wont fail
  auto traj_lifecycle_node = traj_controller->get_lifecycle_node();
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};
  const rclcpp::Parameter joint_parameters("joints", joint_names_);
  traj_lifecycle_node->set_parameter(joint_parameters);

  const std::vector<std::string> operation_mode_names = {"write1", "write2"};
  const rclcpp::Parameter operation_mode_parameters("write_op_modes", operation_mode_names);
  traj_lifecycle_node->set_parameter(operation_mode_parameters);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(traj_lifecycle_node->get_node_base_interface());

  traj_lifecycle_node->set_parameter(
    rclcpp::Parameter(
      "state_publish_rate",
      static_cast<double>(target_msg_count)));

  traj_controller->on_configure(traj_lifecycle_node->get_current_state());
  traj_controller->on_activate(traj_lifecycle_node->get_current_state());

  auto future_handle = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });

  using control_msgs::msg::JointTrajectoryControllerState;

  const int qos_level = 10;
  int echo_received_counter = 0;
  rclcpp::Subscription<JointTrajectoryControllerState>::SharedPtr subs =
    traj_lifecycle_node->create_subscription<JointTrajectoryControllerState>(
    "/state",
    qos_level,
    [&](JointTrajectoryControllerState::UniquePtr msg) {
      (void)msg;
      ++echo_received_counter;
    }
    );

  // wait for things to setup
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // update for 1second
  const auto start_time = rclcpp::Clock().now();
  const rclcpp::Duration wait = rclcpp::Duration::from_seconds(1.0);
  const auto end_time = start_time + wait;
  while (rclcpp::Clock().now() < end_time) {
    traj_controller->update();
  }

  EXPECT_EQ(target_msg_count, echo_received_counter);

  executor.cancel();
}

TEST_F(TestTrajectoryController, test_state_publish_rate) {
  SetUpTrajectoryController();
  test_state_publish_rate_target(traj_controller_, 10);
}

TEST_F(TestTrajectoryController, zero_state_publish_rate) {
  SetUpTrajectoryController();
  test_state_publish_rate_target(traj_controller_, 0);
}

TEST_F(TestTrajectoryController, test_jumbled_joint_order) {
  SetUpTrajectoryController(false);

  auto traj_lifecycle_node = traj_controller_->get_lifecycle_node();
  const std::vector<std::string> joint_names = {"joint1", "joint2", "joint3"};
  const rclcpp::Parameter joint_parameters("joints", joint_names);
  traj_lifecycle_node->set_parameter(joint_parameters);

  const std::vector<std::string> operation_mode_names = {"write1", "write2"};
  const rclcpp::Parameter operation_mode_parameters("write_op_modes", operation_mode_names);
  traj_lifecycle_node->set_parameter(operation_mode_parameters);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(traj_lifecycle_node->get_node_base_interface());

  traj_controller_->on_configure(traj_lifecycle_node->get_current_state());
  traj_controller_->on_activate(traj_lifecycle_node->get_current_state());

  auto future_handle = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });
  // wait for things to setup
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  {
    trajectory_msgs::msg::JointTrajectory traj_msg;
    const std::vector<std::string> jumbled_joint_names {
      test_robot_->joint_name2, test_robot_->joint_name3, test_robot_->joint_name1
    };
    traj_msg.joint_names = jumbled_joint_names;
    traj_msg.header.stamp = rclcpp::Time(0);
    traj_msg.points.resize(1);

    traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
    traj_msg.points[0].positions.resize(3);
    traj_msg.points[0].positions[0] = 2.0;
    traj_msg.points[0].positions[1] = 3.0;
    traj_msg.points[0].positions[2] = 1.0;

    trajectory_publisher_->publish(traj_msg);
  }

  // update for 0.5 seconds
  const auto start_time = rclcpp::Clock().now();
  const rclcpp::Duration wait = rclcpp::Duration::from_seconds(0.5);
  const auto end_time = start_time + wait;
  while (rclcpp::Clock().now() < end_time) {
    test_robot_->read();
    traj_controller_->update();
    test_robot_->write();
  }

  EXPECT_NEAR(1.0, test_robot_->pos1, COMMON_THRESHOLD);
  EXPECT_NEAR(2.0, test_robot_->pos2, COMMON_THRESHOLD);
  EXPECT_NEAR(3.0, test_robot_->pos3, COMMON_THRESHOLD);

  executor.cancel();
}

TEST_F(TestTrajectoryController, test_partial_joint_list) {
  auto traj_controller = std::make_shared<joint_trajectory_controller::JointTrajectoryController>();
  auto ret = traj_controller->init(test_robot_, controller_name_);
  if (ret != controller_interface::return_type::SUCCESS) {
    FAIL();
  }

  auto traj_lifecycle_node = traj_controller->get_lifecycle_node();
  std::vector<std::string> joint_names = {"joint1", "joint2", "joint3"};
  rclcpp::Parameter joint_parameters("joints", joint_names);
  traj_lifecycle_node->set_parameter(joint_parameters);

  std::vector<std::string> operation_mode_names = {"write1", "write2"};
  rclcpp::Parameter operation_mode_parameters("write_op_modes", operation_mode_names);
  traj_lifecycle_node->set_parameter(operation_mode_parameters);

  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);
  traj_lifecycle_node->set_parameter(partial_joints_parameters);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(traj_lifecycle_node->get_node_base_interface());

  traj_controller->on_configure(traj_lifecycle_node->get_current_state());
  traj_controller->on_activate(traj_lifecycle_node->get_current_state());

  auto future_handle = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });
  // wait for things to setup
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const double initial_joint3_cmd = test_robot_->cmd3;
  trajectory_msgs::msg::JointTrajectory traj_msg;

  {
    std::vector<std::string> partial_joint_names {
      test_robot_->joint_name2, test_robot_->joint_name1
    };
    traj_msg.joint_names = partial_joint_names;
    traj_msg.header.stamp = rclcpp::Time(0);
    traj_msg.points.resize(1);

    traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
    traj_msg.points[0].positions.resize(2);
    traj_msg.points[0].positions[0] = 2.0;
    traj_msg.points[0].positions[1] = 1.0;
    traj_msg.points[0].velocities.resize(2);
    traj_msg.points[0].velocities[0] = 2.0;
    traj_msg.points[0].velocities[1] = 1.0;

    trajectory_publisher_->publish(traj_msg);
  }

  // update for 0.5 seconds
  auto start_time = rclcpp::Clock().now();
  rclcpp::Duration wait = rclcpp::Duration::from_seconds(0.5);
  auto end_time = start_time + wait;
  while (rclcpp::Clock().now() < end_time) {
    test_robot_->read();
    traj_controller->update();
    test_robot_->write();
  }

  double threshold = 0.001;
  EXPECT_NEAR(traj_msg.points[0].positions[1], test_robot_->pos1, threshold);
  EXPECT_NEAR(traj_msg.points[0].positions[0], test_robot_->pos2, threshold);
  EXPECT_NEAR(
    initial_joint3_cmd, test_robot_->pos3,
    threshold) << "Joint 3 command should be current position";

//  Velocity commands are not sent yet
//  EXPECT_NEAR(traj_msg.points[0].velocities[1], test_robot_->vel1, threshold);
//  EXPECT_NEAR(traj_msg.points[0].velocities[0], test_robot_->vel2, threshold);
//  EXPECT_NEAR(
//    0.0, test_robot_->vel3,
//    threshold) << "Joint 3 velocity should be 0.0 since it's not in the goal";

  executor.cancel();
}
