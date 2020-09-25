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

class TestableJointTrajectoryController : public joint_trajectory_controller::
  JointTrajectoryController
{
public:
  using joint_trajectory_controller::JointTrajectoryController::validate_trajectory_msg;
  using joint_trajectory_controller::JointTrajectoryController::JointTrajectoryController;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = joint_trajectory_controller::JointTrajectoryController::on_configure(previous_state);
    joint_cmd_sub_wait_set_.add_subscription(joint_command_subscriber_);
    return ret;
  }

  /**
  * @brief wait_for_trajectory block until a new JointTrajectory is received.
  * Requires that the executor is not spinned elsewhere between the
  *  message publication and the call to this function
  *
  * @return true if new JointTrajectory msg was received, false if timeout
  */
  bool wait_for_trajectory(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    bool success = joint_cmd_sub_wait_set_.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
    if (success) {
      executor.spin_some();
    }
    return success;
  }

  rclcpp::WaitSet joint_cmd_sub_wait_set_;
};

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
    joint_names_ = test_robot_->joint_names;
    op_mode_ = {{test_robot_->write_op_handle_name1}};

    // create joint handles
    auto get_handle = [&](const std::string & joint_name, const std::string & interface_name)
      {
        auto joint_handle = std::make_shared<hardware_interface::JointHandle>(
          joint_name,
          interface_name);
        test_robot_->get_joint_handle(*joint_handle);
        return joint_handle;
      };

    joint1_pos_handle_ = get_handle("joint1", "position");
    joint2_pos_handle_ = get_handle("joint2", "position");
    joint3_pos_handle_ = get_handle("joint3", "position");
    joint1_pos_cmd_handle_ = get_handle("joint1", "position_command");
    joint2_pos_cmd_handle_ = get_handle("joint2", "position_command");
    joint3_pos_cmd_handle_ = get_handle("joint3", "position_command");

    pub_node_ = std::make_shared<rclcpp::Node>("trajectory_publisher_");
    trajectory_publisher_ = pub_node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      controller_name_ + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  }

  void SetUpTrajectoryController(bool use_local_parameters = true)
  {
    if (use_local_parameters) {
      traj_controller_ = std::make_shared<TestableJointTrajectoryController>(
        joint_names_, op_mode_);
    } else {
      traj_controller_ = std::make_shared<TestableJointTrajectoryController>();
    }
    auto ret = traj_controller_->init(test_robot_, controller_name_);
    if (ret != controller_interface::return_type::SUCCESS) {
      FAIL();
    }
  }

  void SetUpAndActivateTrajectoryController(
    bool use_local_parameters = true,
    const std::vector<rclcpp::Parameter> & parameters = {},
    rclcpp::Executor * executor = nullptr)
  {
    SetUpTrajectoryController(use_local_parameters);

    traj_lifecycle_node_ = traj_controller_->get_lifecycle_node();
    for (const auto & param : parameters) {
      traj_lifecycle_node_->set_parameter(param);
    }
    if (executor) {
      executor->add_node(traj_lifecycle_node_->get_node_base_interface());
    }

    traj_controller_->on_configure(traj_lifecycle_node_->get_current_state());
    traj_controller_->on_activate(traj_lifecycle_node_->get_current_state());
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void subscribeToState()
  {
    auto traj_lifecycle_node = traj_controller_->get_lifecycle_node();
    traj_lifecycle_node->set_parameter(
      rclcpp::Parameter(
        "state_publish_rate",
        static_cast<double>(100)));

    using control_msgs::msg::JointTrajectoryControllerState;

    auto qos = rclcpp::SensorDataQoS();
    // Needed, otherwise spin_some() returns only the oldest message in the queue
    // I do not understand why spin_some provides only one message
    qos.keep_last(1);
    state_subscriber_ =
      traj_lifecycle_node->create_subscription<JointTrajectoryControllerState>(
      "/state",
      qos,
      [&](std::shared_ptr<JointTrajectoryControllerState> msg) {
        std::lock_guard<std::mutex> guard(state_mutex_);
        state_msg_ = msg;
      }
      );
  }

  /// Publish trajectory msgs with multiple points
  /**
   *  delay_btwn_points - delay between each points
   *  points - vector of trajectories. One point per controlled joint
   *  joint_names - names of joints, if empty, will use joint_names_ up to the number of provided points
   */
  void publish(
    const builtin_interfaces::msg::Duration & delay_btwn_points,
    const std::vector<std::vector<double>> & points,
    rclcpp::Time start_time = rclcpp::Time(),
    const std::vector<std::string> & joint_names = {})
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
    if (joint_names.empty()) {
      traj_msg.joint_names = {joint_names_.begin(), joint_names_.begin() + points[0].size()};
    } else {
      traj_msg.joint_names = joint_names;
    }
    traj_msg.header.stamp = start_time;
    traj_msg.points.resize(points.size());

    builtin_interfaces::msg::Duration duration_msg;
    duration_msg.sec = delay_btwn_points.sec;
    duration_msg.nanosec = delay_btwn_points.nanosec;
    rclcpp::Duration duration(duration_msg);
    rclcpp::Duration duration_total(duration_msg);

    for (size_t index = 0; index < points.size(); ++index) {
      traj_msg.points[index].time_from_start = duration_total;

      traj_msg.points[index].positions.resize(points[index].size());
      for (size_t j = 0; j < points[index].size(); ++j) {
        traj_msg.points[index].positions[j] = points[index][j];
      }
      duration_total = duration_total + duration;
    }

    trajectory_publisher_->publish(traj_msg);
  }

  void updateController(rclcpp::Duration wait_time = rclcpp::Duration::from_seconds(0.2))
  {
    const auto start_time = rclcpp::Clock().now();
    const auto end_time = start_time + wait_time;
    while (rclcpp::Clock().now() < end_time) {
      test_robot_->read();
      traj_controller_->update();
      test_robot_->write();
    }
  }

  void waitAndCompareState(
    trajectory_msgs::msg::JointTrajectoryPoint expected_actual,
    trajectory_msgs::msg::JointTrajectoryPoint expected_desired,
    rclcpp::Executor & executor, rclcpp::Duration controller_wait_time, double allowed_delta)
  {
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      state_msg_.reset();
    }
    traj_controller_->wait_for_trajectory(executor);
    updateController(controller_wait_time);
    // Spin to receive latest state
    executor.spin_some();
    auto state_msg = getState();
    ASSERT_TRUE(state_msg);
    for (size_t i = 0; i < expected_actual.positions.size(); ++i) {
      SCOPED_TRACE("Joint " + std::to_string(i));
      EXPECT_NEAR(expected_actual.positions[i], state_msg->actual.positions[i], allowed_delta);
    }

    for (size_t i = 0; i < expected_desired.positions.size(); ++i) {
      SCOPED_TRACE("Joint " + std::to_string(i));
      EXPECT_NEAR(expected_desired.positions[i], state_msg->desired.positions[i], allowed_delta);
    }
  }

  std::shared_ptr<control_msgs::msg::JointTrajectoryControllerState> getState() const
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    return state_msg_;
  }
  void test_state_publish_rate_target(int target_msg_count);

  std::string controller_name_ = "test_joint_trajectory_controller";

  std::shared_ptr<test_robot_hardware::TestRobotHardware> test_robot_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> op_mode_;

  std::shared_ptr<hardware_interface::JointHandle> joint1_pos_handle_;
  std::shared_ptr<hardware_interface::JointHandle> joint2_pos_handle_;
  std::shared_ptr<hardware_interface::JointHandle> joint3_pos_handle_;
  std::shared_ptr<hardware_interface::JointHandle> joint1_pos_cmd_handle_;
  std::shared_ptr<hardware_interface::JointHandle> joint2_pos_cmd_handle_;
  std::shared_ptr<hardware_interface::JointHandle> joint3_pos_cmd_handle_;

  rclcpp::Node::SharedPtr pub_node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

  std::shared_ptr<TestableJointTrajectoryController> traj_controller_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> traj_lifecycle_node_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    state_subscriber_;
  mutable std::mutex state_mutex_;
  std::shared_ptr<control_msgs::msg::JointTrajectoryControllerState> state_msg_;
};

TEST_F(TestTrajectoryController, wrong_initialization) {
  const auto uninitialized_robot = std::make_shared<test_robot_hardware::TestRobotHardware>();
  auto traj_controller = std::make_shared<TestableJointTrajectoryController>(
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
  auto traj_controller = std::make_shared<TestableJointTrajectoryController>(
    joint_names_, op_mode_);
  const auto ret = traj_controller->init(initialized_robot, controller_name_);
  if (ret != controller_interface::return_type::SUCCESS) {
    FAIL();
  }

  auto get_handle = [&](const std::string & joint_name, const std::string & interface_name)
    {
      hardware_interface::JointHandle joint_handle(joint_name, interface_name);
      initialized_robot->get_joint_handle(joint_handle);
      return joint_handle;
    };

  const auto inactive_state = traj_controller->get_lifecycle_node()->configure();
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, inactive_state.id());
  EXPECT_EQ(1.1, get_handle("joint1", "position").get_value());
  EXPECT_EQ(2.1, get_handle("joint2", "position").get_value());
  EXPECT_EQ(3.1, get_handle("joint3", "position").get_value());
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
  std::vector<std::vector<double>> points {{{3.3, 4.4, 5.5}}};
  publish(time_from_start, points);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  traj_controller_->update();
  test_robot_->write();

  // no change in hw position
  EXPECT_NE(3.3, joint1_pos_handle_->get_value());
  EXPECT_NE(4.4, joint2_pos_handle_->get_value());
  EXPECT_NE(5.5, joint3_pos_handle_->get_value());

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
//   std::vector<std::vector<double>> points {{{3.3, 4.4, 5.5}}};
//   publish(time_from_start, points);
//   // wait for msg is be published to the system
//   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//   executor.spin_once();
//
//   traj_controller->update();
//   test_robot_->write();
//
//   // change in hw position
//   EXPECT_EQ(3.3, joint1_pos_handle_->get_value());
//   EXPECT_EQ(4.4, joint2_pos_handle_->get_value());
//   EXPECT_EQ(5.5, joint3_pos_handle_->get_value());
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
//   std::vector<std::vector<double>> points {
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
//   EXPECT_EQ(3.3, joint1_pos_handle_->get_value());
//   EXPECT_EQ(4.4, joint2_pos_handle_->get_value());
//   EXPECT_EQ(5.5, joint3_pos_handle_->get_value());
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
//   EXPECT_EQ(10.10, joint1_pos_handle_->get_value());
//   EXPECT_EQ(11.11, joint2_pos_handle_->get_value());
//   EXPECT_EQ(12.12, joint3_pos_handle_->get_value());
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

  // send msg
  builtin_interfaces::msg::Duration time_from_start;
  time_from_start.sec = 1;
  time_from_start.nanosec = 0;
  std::vector<std::vector<double>> points {{{3.3, 4.4, 5.5}}};
  publish(time_from_start, points);
  traj_controller_->wait_for_trajectory(executor);
  traj_controller_->update();
  test_robot_->write();

  state = traj_lifecycle_node->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  traj_controller_->update();
  test_robot_->write();

  state = traj_lifecycle_node->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  // update for 0.25 seconds
  const auto start_time = rclcpp::Clock().now();
  updateController(rclcpp::Duration::from_seconds(0.25));

  // should be home pose again
  EXPECT_NEAR(1.1, joint1_pos_handle_->get_value(), COMMON_THRESHOLD);
  EXPECT_NEAR(2.1, joint2_pos_handle_->get_value(), COMMON_THRESHOLD);
  EXPECT_NEAR(3.1, joint3_pos_handle_->get_value(), COMMON_THRESHOLD);
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

  auto state = traj_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_EQ(1.1, joint1_pos_handle_->get_value());
  EXPECT_EQ(2.1, joint2_pos_handle_->get_value());
  EXPECT_EQ(3.1, joint3_pos_handle_->get_value());

  state = traj_lifecycle_node->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points {
    {{3.3, 4.4, 5.5}},
    {{7.7, 8.8, 9.9}},
    {{10.10, 11.11, 12.12}}
  };
  // *INDENT-ON*
  publish(time_from_start, points);
  traj_controller_->wait_for_trajectory(executor);

  // first update
  traj_controller_->update();
  test_robot_->write();

  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(FIRST_POINT_TIME);
  traj_controller_->update();
  test_robot_->write();

  // deactivated
  state = traj_lifecycle_node->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  const auto allowed_delta = 0.05;

  EXPECT_NEAR(3.3, joint1_pos_handle_->get_value(), allowed_delta);
  EXPECT_NEAR(4.4, joint2_pos_handle_->get_value(), allowed_delta);
  EXPECT_NEAR(5.5, joint3_pos_handle_->get_value(), allowed_delta);

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

  EXPECT_NEAR(1.1, joint1_pos_handle_->get_value(), allowed_delta);
  EXPECT_NEAR(2.1, joint2_pos_handle_->get_value(), allowed_delta);
  EXPECT_NEAR(3.1, joint3_pos_handle_->get_value(), allowed_delta);

  state = traj_lifecycle_node->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}

TEST_F(TestTrajectoryController, state_topic_consistency) {
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {}, &executor);
  subscribeToState();
  updateController();

  // Spin to receive latest state
  executor.spin_some();
  auto state = getState();

  size_t n_joints = joint_names_.size();

  for (unsigned int i = 0; i < n_joints; ++i) {
    EXPECT_EQ(joint_names_[i], state->joint_names[i]);
  }

  // No trajectory by default, no desired state or error
  EXPECT_TRUE(state->desired.positions.empty());
  EXPECT_TRUE(state->desired.velocities.empty());
  EXPECT_TRUE(state->desired.accelerations.empty());

  EXPECT_EQ(n_joints, state->actual.positions.size());
  EXPECT_EQ(n_joints, state->actual.velocities.size());
  EXPECT_TRUE(state->actual.accelerations.empty());

  EXPECT_TRUE(state->error.positions.empty());
  EXPECT_TRUE(state->error.velocities.empty());
  EXPECT_TRUE(state->error.accelerations.empty());
}

void TestTrajectoryController::test_state_publish_rate_target(int target_msg_count)
{
  rclcpp::Parameter state_publish_rate_param(
    "state_publish_rate",
    static_cast<double>(target_msg_count));
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {state_publish_rate_param}, &executor);

  auto future_handle = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });

  using control_msgs::msg::JointTrajectoryControllerState;

  const int qos_level = 10;
  int echo_received_counter = 0;
  rclcpp::Subscription<JointTrajectoryControllerState>::SharedPtr subs =
    traj_lifecycle_node_->create_subscription<JointTrajectoryControllerState>(
    "/state",
    qos_level,
    [&](JointTrajectoryControllerState::UniquePtr msg) {
      (void)msg;
      ++echo_received_counter;
    }
    );

  // update for 1second
  const auto start_time = rclcpp::Clock().now();
  const rclcpp::Duration wait = rclcpp::Duration::from_seconds(1.0);
  const auto end_time = start_time + wait;
  while (rclcpp::Clock().now() < end_time) {
    traj_controller_->update();
  }

  // We may miss the last message since time allowed is exactly the time needed
  EXPECT_NEAR(target_msg_count, echo_received_counter, 1);

  executor.cancel();
}

/**
 * @brief test_state_publish_rate Test that state publish rate matches configure rate
 */
TEST_F(TestTrajectoryController, test_state_publish_rate) {
  test_state_publish_rate_target(10);
}

TEST_F(TestTrajectoryController, zero_state_publish_rate) {
  test_state_publish_rate_target(0);
}

/**
 * @brief test_jumbled_joint_order Test sending trajectories with a joint order different from internal controller order
 */
TEST_F(TestTrajectoryController, test_jumbled_joint_order) {
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {}, &executor);
  {
    trajectory_msgs::msg::JointTrajectory traj_msg;
    const std::vector<std::string> jumbled_joint_names {
      test_robot_->joint_names[1], test_robot_->joint_names[2], test_robot_->joint_names[0]
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

  traj_controller_->wait_for_trajectory(executor);
  // update for 0.25 seconds
  updateController(rclcpp::Duration::from_seconds(0.25));

  EXPECT_NEAR(1.0, joint1_pos_handle_->get_value(), COMMON_THRESHOLD);
  EXPECT_NEAR(2.0, joint2_pos_handle_->get_value(), COMMON_THRESHOLD);
  EXPECT_NEAR(3.0, joint3_pos_handle_->get_value(), COMMON_THRESHOLD);
}

/**
 * @brief test_partial_joint_list Test sending trajectories with a subset of the controlled joints
 */
TEST_F(TestTrajectoryController, test_partial_joint_list) {
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);

  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

  const double initial_joint3_cmd = joint3_pos_cmd_handle_->get_value();
  trajectory_msgs::msg::JointTrajectory traj_msg;

  {
    std::vector<std::string> partial_joint_names {
      test_robot_->joint_names[1], test_robot_->joint_names[0]
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

  traj_controller_->wait_for_trajectory(executor);
  // update for 0.5 seconds
  updateController(rclcpp::Duration::from_seconds(0.25));

  double threshold = 0.001;
  EXPECT_NEAR(traj_msg.points[0].positions[1], joint1_pos_handle_->get_value(), threshold);
  EXPECT_NEAR(traj_msg.points[0].positions[0], joint2_pos_handle_->get_value(), threshold);
  EXPECT_NEAR(
    initial_joint3_cmd, joint3_pos_handle_->get_value(),
    threshold) << "Joint 3 command should be current position";

//  Velocity commands are not sent yet
//  EXPECT_NEAR(traj_msg.points[0].velocities[1], test_robot_->vel1, threshold);
//  EXPECT_NEAR(traj_msg.points[0].velocities[0], test_robot_->vel2, threshold);
//  EXPECT_NEAR(
//    0.0, test_robot_->vel3,
//    threshold) << "Joint 3 velocity should be 0.0 since it's not in the goal";

  executor.cancel();
}

/**
 * @brief test_partial_joint_list Test sending trajectories with a subset of the controlled joints without allow_partial_joints_goal
 */
TEST_F(TestTrajectoryController, test_partial_joint_list_not_allowed) {
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", false);

  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

  const double initial_joint1_cmd = joint1_pos_cmd_handle_->get_value();
  const double initial_joint2_cmd = joint2_pos_cmd_handle_->get_value();
  const double initial_joint3_cmd = joint3_pos_cmd_handle_->get_value();
  trajectory_msgs::msg::JointTrajectory traj_msg;

  {
    std::vector<std::string> partial_joint_names {
      test_robot_->joint_names[1], test_robot_->joint_names[0]
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

  traj_controller_->wait_for_trajectory(executor);
  // update for 0.5 seconds
  updateController(rclcpp::Duration::from_seconds(0.25));

  double threshold = 0.001;
  EXPECT_NEAR(
    initial_joint1_cmd, joint1_pos_cmd_handle_->get_value(),
    threshold) << "All joints command should be current position because goal was rejected";
  EXPECT_NEAR(
    initial_joint2_cmd, joint2_pos_cmd_handle_->get_value(),
    threshold) << "All joints command should be current position because goal was rejected";
  EXPECT_NEAR(
    initial_joint3_cmd, joint3_pos_cmd_handle_->get_value(),
    threshold) << "All joints command should be current position because goal was rejected";

  //  Velocity commands are not sent yet
  //  EXPECT_NEAR(traj_msg.points[0].velocities[1], test_robot_->vel1, threshold);
  //  EXPECT_NEAR(traj_msg.points[0].velocities[0], test_robot_->vel2, threshold);
  //  EXPECT_NEAR(
  //    0.0, test_robot_->vel3,
  //    threshold) << "Joint 3 velocity should be 0.0 since it's not in the goal";

  executor.cancel();
}


/**
 * @brief invalid_message Test mismatched joint and reference vector sizes
 */
TEST_F(TestTrajectoryController, invalid_message) {
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", false);
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

  trajectory_msgs::msg::JointTrajectory traj_msg, good_traj_msg;

  good_traj_msg.joint_names = joint_names_;
  good_traj_msg.header.stamp = rclcpp::Time(0);
  good_traj_msg.points.resize(1);
  good_traj_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.25);
  good_traj_msg.points[0].positions.resize(1);
  good_traj_msg.points[0].positions = {1.0, 2.0, 3.0};
  good_traj_msg.points[0].velocities.resize(1);
  good_traj_msg.points[0].velocities = {-1.0, -2.0, -3.0};
  EXPECT_TRUE(traj_controller_->validate_trajectory_msg(good_traj_msg));

  // Incompatible joint names
  traj_msg = good_traj_msg;
  traj_msg.joint_names = {"bad_name"};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // No position data
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too many positions
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions = {1.0, 2.0, 3.0, 4.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few velocities
  traj_msg = good_traj_msg;
  traj_msg.points[0].velocities = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few accelerations
  traj_msg = good_traj_msg;
  traj_msg.points[0].accelerations = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Incompatible data sizes, too few efforts
  traj_msg = good_traj_msg;
  traj_msg.points[0].positions.clear();
  traj_msg.points[0].effort = {1.0, 2.0};
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));

  // Non-strictly increasing waypoint times
  traj_msg = good_traj_msg;
  traj_msg.points.push_back(traj_msg.points.front());
  EXPECT_FALSE(traj_controller_->validate_trajectory_msg(traj_msg));
}

/**
 * @brief test_trajectory_replace Test replacing an existing trajectory
 */
TEST_F(TestTrajectoryController, test_trajectory_replace) {
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);
  SetUpAndActivateTrajectoryController(true, {partial_joints_parameters}, &executor);

  subscribeToState();

  std::vector<std::vector<double>> points_old {{{2., 3., 4.}}};
  std::vector<std::vector<double>> points_partial_new {{{1.5}}};

  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
  publish(time_from_start, points_old);
  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
  expected_desired.positions = {points_old[0].begin(), points_old[0].end()};
  //  Check that we reached end of points_old trajectory
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  RCLCPP_INFO(traj_lifecycle_node_->get_logger(), "Sending new trajectory");
  publish(time_from_start, points_partial_new);
  // Replaced trajectory is a mix of previous and current goal
  expected_desired.positions[0] = points_partial_new[0][0];
  expected_desired.positions[1] = points_old[0][1];
  expected_desired.positions[2] = points_old[0][2];
  expected_actual = expected_desired;
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);
}

/**
 * @brief test_ignore_old_trajectory Sending an old trajectory replacing an existing trajectory
 */
TEST_F(TestTrajectoryController, test_ignore_old_trajectory) {
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {}, &executor);
  subscribeToState();

  std::vector<std::vector<double>> points_old {{{2., 3., 4.}, {4., 5., 6.}}};
  std::vector<std::vector<double>> points_new {{{-1., -2., -3.}}};

  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
  publish(time_from_start, points_old);
  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
  expected_desired = expected_actual;
  //  Check that we reached end of points_old[0] trajectory
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  RCLCPP_INFO(traj_lifecycle_node_->get_logger(), "Sending new trajectory in the past");
  //  New trajectory will end before current time
  rclcpp::Time new_traj_start = rclcpp::Clock().now() - delay - std::chrono::milliseconds(100);
  expected_actual.positions = {points_old[1].begin(), points_old[1].end()};
  expected_desired = expected_actual;
  publish(time_from_start, points_new, new_traj_start);
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);
}

TEST_F(TestTrajectoryController, test_ignore_partial_old_trajectory) {
  rclcpp::executors::SingleThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(true, {}, &executor);
  subscribeToState();

  std::vector<std::vector<double>> points_old {{{2., 3., 4.}, {4., 5., 6.}}};
  std::vector<std::vector<double>> points_new {{{-1., -2., -3.}, {-2., -4., -6.}}};

  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(delay)};
  publish(time_from_start, points_old);
  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {points_old[0].begin(), points_old[0].end()};
  expected_desired = expected_actual;
  //  Check that we reached end of points_old[0]trajectory
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);

  RCLCPP_INFO(traj_lifecycle_node_->get_logger(), "Sending new trajectory partially in the past");
  //  New trajectory first point is in the past, second is in the future
  rclcpp::Time new_traj_start = rclcpp::Clock().now() - delay - std::chrono::milliseconds(100);
  expected_actual.positions = {points_new[1].begin(), points_new[1].end()};
  expected_desired = expected_actual;
  publish(time_from_start, points_new, new_traj_start);
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);
}


TEST_F(TestTrajectoryController, test_execute_partial_traj_in_future) {
  SetUpTrajectoryController();
  auto traj_lifecycle_node = traj_controller_->get_lifecycle_node();
  RCLCPP_WARN(
    traj_lifecycle_node->get_logger(),
    "Test disabled until current_trajectory is taken into account when adding a new trajectory.");
  // https://github.com/ros-controls/ros_controllers/blob/melodic-devel/joint_trajectory_controller/include/joint_trajectory_controller/init_joint_trajectory.h#L149
  return;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(traj_lifecycle_node->get_node_base_interface());
  subscribeToState();
  rclcpp::Parameter partial_joints_parameters("allow_partial_joints_goal", true);
  traj_lifecycle_node->set_parameter(partial_joints_parameters);
  traj_lifecycle_node->configure();
  traj_lifecycle_node->activate();

  std::vector<std::vector<double>> full_traj {{{2., 3., 4.}, {4., 6., 8.}}};
  std::vector<std::vector<double>> partial_traj {{{-1., -2.}, {-2., -4, }}};
  const auto delay = std::chrono::milliseconds(500);
  builtin_interfaces::msg::Duration points_delay{rclcpp::Duration(delay)};
  // Send full trajectory
  publish(points_delay, full_traj);
  // Sleep until first waypoint of full trajectory

  trajectory_msgs::msg::JointTrajectoryPoint expected_actual, expected_desired;
  expected_actual.positions = {full_traj[0].begin(), full_traj[0].end()};
  expected_desired = expected_actual;
  //  Check that we reached end of points_old[0]trajectory and are starting points_old[1]
  waitAndCompareState(expected_actual, expected_desired, executor, rclcpp::Duration(delay), 0.1);


  // Send partial trajectory starting after full trajecotry is complete
  RCLCPP_INFO(traj_lifecycle_node->get_logger(), "Sending new trajectory in the future");
  publish(points_delay, partial_traj, rclcpp::Clock().now() + delay * 2);
  // Wait until the end start and end of partial traj

  expected_actual.positions = {partial_traj.back()[0], partial_traj.back()[1], full_traj.back()[2]};
  expected_desired = expected_actual;

  waitAndCompareState(
    expected_actual, expected_desired, executor, rclcpp::Duration(
      delay * (2 + 2)), 0.1);
}
