// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef TEST_TRAJECTORY_CONTROLLER_UTILS_HPP_
#define TEST_TRAJECTORY_CONTROLLER_UTILS_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

namespace
{
const double COMMON_THRESHOLD = 0.0011;  // destogl: increased for 0.0001 for stable CI builds?
const double INITIAL_POS_JOINT1 = 1.1;
const double INITIAL_POS_JOINT2 = 2.1;
const double INITIAL_POS_JOINT3 = 3.1;
const std::vector<double> INITIAL_POS_JOINTS = {
  INITIAL_POS_JOINT1, INITIAL_POS_JOINT2, INITIAL_POS_JOINT3};
const std::vector<double> INITIAL_VEL_JOINTS = {0.0, 0.0, 0.0};
const std::vector<double> INITIAL_ACC_JOINTS = {0.0, 0.0, 0.0};
}  // namespace

namespace test_trajectory_controllers
{
class TestableJointTrajectoryController
: public joint_trajectory_controller::JointTrajectoryController
{
public:
  using joint_trajectory_controller::JointTrajectoryController::JointTrajectoryController;
  using joint_trajectory_controller::JointTrajectoryController::validate_trajectory_msg;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = joint_trajectory_controller::JointTrajectoryController::on_configure(previous_state);
    // this class can still be useful without the wait set
    if (joint_command_subscriber_)
    {
      joint_cmd_sub_wait_set_.add_subscription(joint_command_subscriber_);
    }
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
    if (success)
    {
      executor.spin_some();
    }
    return success;
  }

  void set_joint_names(const std::vector<std::string> & joint_names) { joint_names_ = joint_names; }

  void set_command_interfaces(const std::vector<std::string> & command_interfaces)
  {
    command_interface_types_ = command_interfaces;
  }

  void set_state_interfaces(const std::vector<std::string> & state_interfaces)
  {
    state_interface_types_ = state_interfaces;
  }

  trajectory_msgs::msg::JointTrajectoryPoint get_current_state_when_offset()
  {
    return last_commanded_state_;
  }

  rclcpp::WaitSet joint_cmd_sub_wait_set_;
};

class TrajectoryControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  virtual void SetUp()
  {
    controller_name_ = "test_joint_trajectory_controller";

    joint_names_ = {"joint1", "joint2", "joint3"};
    joint_pos_.resize(joint_names_.size(), 0.0);
    joint_state_pos_.resize(joint_names_.size(), 0.0);
    joint_vel_.resize(joint_names_.size(), 0.0);
    joint_state_vel_.resize(joint_names_.size(), 0.0);
    joint_acc_.resize(joint_names_.size(), 0.0);
    joint_state_acc_.resize(joint_names_.size(), 0.0);
    // Default interface values - they will be overwritten by parameterized tests
    command_interface_types_ = {"position"};
    state_interface_types_ = {"position", "velocity"};

    node_ = std::make_shared<rclcpp::Node>("trajectory_publisher_");
    trajectory_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      controller_name_ + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  }

  void SetUpTrajectoryController(bool use_local_parameters = true)
  {
    traj_controller_ = std::make_shared<TestableJointTrajectoryController>();
    if (use_local_parameters)
    {
      traj_controller_->set_joint_names(joint_names_);
      traj_controller_->set_command_interfaces(command_interface_types_);
      traj_controller_->set_state_interfaces(state_interface_types_);
    }
    auto ret = traj_controller_->init(controller_name_);
    if (ret != controller_interface::return_type::OK)
    {
      FAIL();
    }
  }

  void SetParameters()
  {
    auto node = traj_controller_->get_node();
    const rclcpp::Parameter joint_names_param("joints", joint_names_);
    const rclcpp::Parameter cmd_interfaces_params("command_interfaces", command_interface_types_);
    const rclcpp::Parameter state_interfaces_params("state_interfaces", state_interface_types_);
    node->set_parameters({joint_names_param, cmd_interfaces_params, state_interfaces_params});
  }

  void SetUpAndActivateTrajectoryController(
    bool use_local_parameters = true, const std::vector<rclcpp::Parameter> & parameters = {},
    rclcpp::Executor * executor = nullptr, bool separate_cmd_and_state_values = false)
  {
    SetUpTrajectoryController(use_local_parameters);

    traj_node_ = traj_controller_->get_node();
    for (const auto & param : parameters)
    {
      traj_node_->set_parameter(param);
    }
    if (executor)
    {
      executor->add_node(traj_node_->get_node_base_interface());
    }

    // ignore velocity tolerances for this test since they aren't committed in test_robot->write()
    rclcpp::Parameter stopped_velocity_parameters("constraints.stopped_velocity_tolerance", 0.0);
    traj_node_->set_parameter(stopped_velocity_parameters);

    traj_controller_->configure();
    ActivateTrajectoryController(separate_cmd_and_state_values);
  }

  void ActivateTrajectoryController(bool separate_cmd_and_state_values = false)
  {
    std::vector<hardware_interface::LoanedCommandInterface> cmd_interfaces;
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    pos_cmd_interfaces_.reserve(joint_names_.size());
    vel_cmd_interfaces_.reserve(joint_names_.size());
    acc_cmd_interfaces_.reserve(joint_names_.size());
    pos_state_interfaces_.reserve(joint_names_.size());
    vel_state_interfaces_.reserve(joint_names_.size());
    acc_state_interfaces_.reserve(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      pos_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_pos_[i]));
      vel_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_vel_[i]));
      acc_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_ACCELERATION, &joint_acc_[i]));

      pos_state_interfaces_.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        separate_cmd_and_state_values ? &joint_state_pos_[i] : &joint_pos_[i]));
      vel_state_interfaces_.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        separate_cmd_and_state_values ? &joint_state_vel_[i] : &joint_vel_[i]));
      acc_state_interfaces_.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_ACCELERATION,
        separate_cmd_and_state_values ? &joint_state_acc_[i] : &joint_acc_[i]));

      // Add to export lists and set initial values
      cmd_interfaces.emplace_back(pos_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(INITIAL_POS_JOINTS[i]);
      cmd_interfaces.emplace_back(vel_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(INITIAL_VEL_JOINTS[i]);
      cmd_interfaces.emplace_back(acc_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(INITIAL_ACC_JOINTS[i]);
      joint_state_pos_[i] = INITIAL_POS_JOINTS[i];
      joint_state_vel_[i] = INITIAL_VEL_JOINTS[i];
      joint_state_acc_[i] = INITIAL_ACC_JOINTS[i];
      state_interfaces.emplace_back(pos_state_interfaces_.back());
      state_interfaces.emplace_back(vel_state_interfaces_.back());
      state_interfaces.emplace_back(acc_state_interfaces_.back());
    }

    traj_controller_->assign_interfaces(std::move(cmd_interfaces), std::move(state_interfaces));
    traj_controller_->activate();
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void subscribeToState()
  {
    auto traj_lifecycle_node = traj_controller_->get_node();
    traj_lifecycle_node->set_parameter(
      rclcpp::Parameter("state_publish_rate", static_cast<double>(100)));

    using control_msgs::msg::JointTrajectoryControllerState;

    auto qos = rclcpp::SensorDataQoS();
    // Needed, otherwise spin_some() returns only the oldest message in the queue
    // I do not understand why spin_some provides only one message
    qos.keep_last(1);
    state_subscriber_ = traj_lifecycle_node->create_subscription<JointTrajectoryControllerState>(
      controller_name_ + "/state", qos, [&](std::shared_ptr<JointTrajectoryControllerState> msg) {
        std::lock_guard<std::mutex> guard(state_mutex_);
        state_msg_ = msg;
      });
  }

  /// Publish trajectory msgs with multiple points
  /**
   *  delay_btwn_points - delay between each points
   *  points - vector of trajectories. One point per controlled joint
   *  joint_names - names of joints, if empty, will use joint_names_ up to the number of provided points
   */
  void publish(
    const builtin_interfaces::msg::Duration & delay_btwn_points,
    const std::vector<std::vector<double>> & points, rclcpp::Time start_time = rclcpp::Time(),
    const std::vector<std::string> & joint_names = {})
  {
    int wait_count = 0;
    const auto topic = trajectory_publisher_->get_topic_name();
    while (node_->count_subscribers(topic) == 0)
    {
      if (wait_count >= 5)
      {
        auto error_msg = std::string("publishing to ") + topic + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    trajectory_msgs::msg::JointTrajectory traj_msg;
    if (joint_names.empty())
    {
      traj_msg.joint_names = {joint_names_.begin(), joint_names_.begin() + points[0].size()};
    }
    else
    {
      traj_msg.joint_names = joint_names;
    }
    traj_msg.header.stamp = start_time;
    traj_msg.points.resize(points.size());

    builtin_interfaces::msg::Duration duration_msg;
    duration_msg.sec = delay_btwn_points.sec;
    duration_msg.nanosec = delay_btwn_points.nanosec;
    rclcpp::Duration duration(duration_msg);
    rclcpp::Duration duration_total(duration_msg);

    for (size_t index = 0; index < points.size(); ++index)
    {
      traj_msg.points[index].time_from_start = duration_total;

      traj_msg.points[index].positions.resize(points[index].size());
      for (size_t j = 0; j < points[index].size(); ++j)
      {
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
    while (rclcpp::Clock().now() < end_time)
    {
      traj_controller_->update(rclcpp::Clock().now(), rclcpp::Clock().now() - start_time);
    }
  }

  void waitAndCompareState(
    trajectory_msgs::msg::JointTrajectoryPoint expected_actual,
    trajectory_msgs::msg::JointTrajectoryPoint expected_desired, rclcpp::Executor & executor,
    rclcpp::Duration controller_wait_time, double allowed_delta)
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
    for (size_t i = 0; i < expected_actual.positions.size(); ++i)
    {
      SCOPED_TRACE("Joint " + std::to_string(i));
      // TODO(anyone): add checking for velocties and accelerations
      EXPECT_NEAR(expected_actual.positions[i], state_msg->actual.positions[i], allowed_delta);
    }

    for (size_t i = 0; i < expected_desired.positions.size(); ++i)
    {
      SCOPED_TRACE("Joint " + std::to_string(i));
      // TODO(anyone): add checking for velocties and accelerations
      EXPECT_NEAR(expected_desired.positions[i], state_msg->desired.positions[i], allowed_delta);
    }
  }

  std::shared_ptr<control_msgs::msg::JointTrajectoryControllerState> getState() const
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    return state_msg_;
  }
  void test_state_publish_rate_target(int target_msg_count);

  std::string controller_name_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

  std::shared_ptr<TestableJointTrajectoryController> traj_controller_;
  std::shared_ptr<rclcpp::Node> traj_node_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    state_subscriber_;
  mutable std::mutex state_mutex_;
  std::shared_ptr<control_msgs::msg::JointTrajectoryControllerState> state_msg_;

  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_acc_;
  std::vector<double> joint_state_pos_;
  std::vector<double> joint_state_vel_;
  std::vector<double> joint_state_acc_;
  std::vector<hardware_interface::CommandInterface> pos_cmd_interfaces_;
  std::vector<hardware_interface::CommandInterface> vel_cmd_interfaces_;
  std::vector<hardware_interface::CommandInterface> acc_cmd_interfaces_;
  std::vector<hardware_interface::StateInterface> pos_state_interfaces_;
  std::vector<hardware_interface::StateInterface> vel_state_interfaces_;
  std::vector<hardware_interface::StateInterface> acc_state_interfaces_;
};

// From the tutorial: https://www.sandordargo.com/blog/2019/04/24/parameterized-testing-with-gtest
class TrajectoryControllerTestParameterized
: public TrajectoryControllerTest,
  public ::testing::WithParamInterface<
    std::tuple<std::vector<std::string>, std::vector<std::string>>>
{
public:
  virtual void SetUp()
  {
    TrajectoryControllerTest::SetUp();
    command_interface_types_ = std::get<0>(GetParam());
    state_interface_types_ = std::get<1>(GetParam());
  }

  static void TearDownTestCase() { TrajectoryControllerTest::TearDownTestCase(); }
};

}  // namespace test_trajectory_controllers

#endif  // TEST_TRAJECTORY_CONTROLLER_UTILS_HPP_
