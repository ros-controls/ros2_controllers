// Copyright 2020 PAL Robotics SL.
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

#include <gmock/gmock.h>

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "diff_drive_controller/diff_drive_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using lifecycle_msgs::msg::State;
using testing::SizeIs;

namespace
{
const std::vector<std::string> left_wheel_names = {"left_wheel_joint"};
const std::vector<std::string> right_wheel_names = {"right_wheel_joint"};
}  // namespace

class TestableDiffDriveController : public diff_drive_controller::DiffDriveController
{
public:
  using DiffDriveController::DiffDriveController;
  std::shared_ptr<geometry_msgs::msg::TwistStamped> getLastReceivedTwist()
  {
    std::shared_ptr<geometry_msgs::msg::TwistStamped> ret;
    received_velocity_msg_ptr_.get(ret);
    return ret;
  }

  /**
   * @brief wait_for_twist block until a new twist is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function
   */
  void wait_for_twist(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(500))
  {
    auto until = get_node()->get_clock()->now() + timeout;
    while (get_node()->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  /**
   * @brief Used to get the real_time_odometry_publisher to verify its contents
   *
   * @return Copy of realtime_odometry_publisher_ object
   */
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
  get_rt_odom_publisher()
  {
    return realtime_odometry_publisher_;
  }
};

class TestDiffDriveController : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  void SetUp() override
  {
    controller_ = std::make_unique<TestableDiffDriveController>();

    pub_node = std::make_shared<rclcpp::Node>("velocity_publisher");
    velocity_publisher = pub_node->create_publisher<geometry_msgs::msg::TwistStamped>(
      controller_name + "/cmd_vel", rclcpp::SystemDefaultsQoS());
  }

  void TearDown() override
  {
    // Reset the controller before the fixture is destroyed to ensure the controller's
    // shutdown transition (which clears loaned interfaces) runs while the underlying
    // StateInterface/CommandInterface objects are still alive. LoanedStateInterface stores
    // a const reference (not a shared_ptr), so destruction order matters.
    controller_.reset();
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  /// Publish velocity msgs
  /**
   *  linear - magnitude of the linear command in the geometry_msgs::twist message
   *  angular - the magnitude of the angular command in geometry_msgs::twist message
   */
  void publish(double linear, double angular)
  {
    int wait_count = 0;
    auto topic = velocity_publisher->get_topic_name();
    while (pub_node->count_subscribers(topic) == 0)
    {
      if (wait_count >= 5)
      {
        auto error_msg = std::string("publishing to ") + topic + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    geometry_msgs::msg::TwistStamped velocity_message;
    velocity_message.header.stamp = pub_node->get_clock()->now();
    velocity_message.twist.linear.x = linear;
    velocity_message.twist.angular.z = angular;
    velocity_publisher->publish(velocity_message);
  }

  /// \brief wait for the subscriber and publisher to completely setup
  void waitForSetup()
  {
    constexpr std::chrono::seconds TIMEOUT{2};
    auto clock = pub_node->get_clock();
    auto start = clock->now();
    while (velocity_publisher->get_subscription_count() <= 0)
    {
      if ((clock->now() - start) > TIMEOUT)
      {
        FAIL();
      }
      rclcpp::spin_some(pub_node);
    }
  }

  void assignResourcesPosFeedback()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(left_wheel_pos_state_);
    state_ifs.emplace_back(right_wheel_pos_state_);

    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(left_wheel_vel_cmd_);
    command_ifs.emplace_back(right_wheel_vel_cmd_);

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void assignResourcesVelFeedback()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(left_wheel_vel_state_);
    state_ifs.emplace_back(right_wheel_vel_state_);

    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(left_wheel_vel_cmd_);
    command_ifs.emplace_back(right_wheel_vel_cmd_);

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  controller_interface::return_type InitController(
    const std::vector<std::string> left_wheel_joints_init = left_wheel_names,
    const std::vector<std::string> right_wheel_joints_init = right_wheel_names,
    const std::vector<rclcpp::Parameter> & parameters = {}, const std::string ns = "")
  {
    auto node_options = rclcpp::NodeOptions();
    std::vector<rclcpp::Parameter> parameter_overrides;

    parameter_overrides.push_back(
      rclcpp::Parameter("left_wheel_names", rclcpp::ParameterValue(left_wheel_joints_init)));
    parameter_overrides.push_back(
      rclcpp::Parameter("right_wheel_names", rclcpp::ParameterValue(right_wheel_joints_init)));
    // default parameters
    parameter_overrides.push_back(
      rclcpp::Parameter("wheel_separation", rclcpp::ParameterValue(1.0)));
    parameter_overrides.push_back(rclcpp::Parameter("wheel_radius", rclcpp::ParameterValue(0.1)));

    parameter_overrides.insert(parameter_overrides.end(), parameters.begin(), parameters.end());
    node_options.parameter_overrides(parameter_overrides);

    return controller_->init(controller_name, ns, node_options);
  }

  const std::string controller_name = "test_diff_drive_controller";
  std::unique_ptr<TestableDiffDriveController> controller_;

  std::vector<double> position_values_ = {0.1, 0.2};
  std::vector<double> velocity_values_ = {0.01, 0.02};

  hardware_interface::StateInterface left_wheel_pos_state_{
    left_wheel_names[0], HW_IF_POSITION, &position_values_[0]};
  hardware_interface::StateInterface right_wheel_pos_state_{
    right_wheel_names[0], HW_IF_POSITION, &position_values_[1]};
  hardware_interface::StateInterface left_wheel_vel_state_{
    left_wheel_names[0], HW_IF_VELOCITY, &velocity_values_[0]};
  hardware_interface::StateInterface right_wheel_vel_state_{
    right_wheel_names[0], HW_IF_VELOCITY, &velocity_values_[1]};
  hardware_interface::CommandInterface left_wheel_vel_cmd_{
    left_wheel_names[0], HW_IF_VELOCITY, &velocity_values_[0]};
  hardware_interface::CommandInterface right_wheel_vel_cmd_{
    right_wheel_names[0], HW_IF_VELOCITY, &velocity_values_[1]};

  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher;
};

TEST_F(TestDiffDriveController, init_fails_without_parameters)
{
  const auto ret = controller_->init(controller_name);
  ASSERT_EQ(ret, controller_interface::return_type::ERROR);
}

TEST_F(TestDiffDriveController, init_fails_with_only_left_or_only_right_side_defined)
{
  ASSERT_EQ(InitController(left_wheel_names, {}), controller_interface::return_type::ERROR);

  ASSERT_EQ(InitController({}, right_wheel_names), controller_interface::return_type::ERROR);
}

TEST_F(TestDiffDriveController, configure_fails_with_mismatching_wheel_side_size)
{
  ASSERT_EQ(
    InitController(left_wheel_names, {right_wheel_names[0], "extra_wheel"}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestDiffDriveController, configure_succeeds_when_wheels_are_specified)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto state_if_conf = controller_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(left_wheel_names.size() + right_wheel_names.size()));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  auto cmd_if_conf = controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, SizeIs(left_wheel_names.size() + right_wheel_names.size()));
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(TestDiffDriveController, configure_succeeds_tf_test_prefix_false_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(false)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto odometry_message = controller_->get_rt_odom_publisher()->msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  /* tf_frame_prefix_enable is false so no modifications to the frame id's */
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_link_id);
}

TEST_F(TestDiffDriveController, configure_succeeds_tf_test_prefix_true_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto odometry_message = controller_->get_rt_odom_publisher()->msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;

  /* tf_frame_prefix_enable is true and frame_prefix is not blank so should be appended to the frame
   * id's */
  ASSERT_EQ(test_odom_frame_id, frame_prefix + "/" + odom_id);
  ASSERT_EQ(test_base_frame_id, frame_prefix + "/" + base_link_id);
}

TEST_F(TestDiffDriveController, configure_succeeds_tf_blank_prefix_true_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "";

  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto odometry_message = controller_->get_rt_odom_publisher()->msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  /* tf_frame_prefix_enable is true but frame_prefix is blank so should not be appended to the frame
   * id's */
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_link_id);
}

TEST_F(TestDiffDriveController, configure_succeeds_tf_test_prefix_false_set_namespace)
{
  std::string test_namespace = "/test_namespace";

  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(false)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))},
      test_namespace),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto odometry_message = controller_->get_rt_odom_publisher()->msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  /* tf_frame_prefix_enable is false so no modifications to the frame id's */
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_link_id);
}

TEST_F(TestDiffDriveController, configure_succeeds_tf_test_prefix_true_set_namespace)
{
  std::string test_namespace = "/test_namespace";

  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))},
      test_namespace),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto odometry_message = controller_->get_rt_odom_publisher()->msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;

  /* tf_frame_prefix_enable is true and frame_prefix is not blank so should be appended to the frame
   * id's instead of the namespace*/
  ASSERT_EQ(test_odom_frame_id, frame_prefix + "/" + odom_id);
  ASSERT_EQ(test_base_frame_id, frame_prefix + "/" + base_link_id);
}

TEST_F(TestDiffDriveController, configure_succeeds_tf_blank_prefix_true_set_namespace)
{
  std::string test_namespace = "/test_namespace";
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "";

  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("tf_frame_prefix_enable", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix)),
       rclcpp::Parameter("odom_frame_id", rclcpp::ParameterValue(odom_id)),
       rclcpp::Parameter("base_frame_id", rclcpp::ParameterValue(base_link_id))},
      test_namespace),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto odometry_message = controller_->get_rt_odom_publisher()->msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  std::string ns_prefix = test_namespace.erase(0, 1) + "/";
  /* tf_frame_prefix_enable is true but frame_prefix is blank so namespace should be appended to the
   * frame id's */
  ASSERT_EQ(test_odom_frame_id, ns_prefix + odom_id);
  ASSERT_EQ(test_base_frame_id, ns_prefix + base_link_id);
}

TEST_F(TestDiffDriveController, activate_fails_without_resources_assigned)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestDiffDriveController, activate_succeeds_with_pos_resources_assigned)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  // We implicitly test that by default position feedback is required
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  assignResourcesPosFeedback();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(TestDiffDriveController, activate_succeeds_with_vel_resources_assigned)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("position_feedback", rclcpp::ParameterValue(false))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  assignResourcesVelFeedback();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(TestDiffDriveController, activate_fails_with_wrong_resources_assigned_1)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("position_feedback", rclcpp::ParameterValue(false))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  assignResourcesPosFeedback();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestDiffDriveController, activate_fails_with_wrong_resources_assigned_2)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("position_feedback", rclcpp::ParameterValue(true))}),
    controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  assignResourcesVelFeedback();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST_F(TestDiffDriveController, cleanup)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("wheel_separation", 0.4), rclcpp::Parameter("wheel_radius", 0.1)}),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  auto state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  assignResourcesPosFeedback();

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup();

  // send msg
  const double linear = 1.0;
  const double angular = 1.0;
  publish(linear, angular);
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  state = controller_->get_node()->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());

  // should be stopped
  EXPECT_EQ(0.0, left_wheel_vel_cmd_.get_value());
  EXPECT_EQ(0.0, right_wheel_vel_cmd_.get_value());

  executor.cancel();
}

TEST_F(TestDiffDriveController, correct_initialization_using_parameters)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("wheel_separation", 0.4), rclcpp::Parameter("wheel_radius", 1.0)}),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  assignResourcesPosFeedback();

  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_EQ(0.01, left_wheel_vel_cmd_.get_value());
  EXPECT_EQ(0.02, right_wheel_vel_cmd_.get_value());

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  // send msg
  const double linear = 1.0;
  const double angular = 0.0;
  publish(linear, angular);
  // wait for msg is be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(1.0, left_wheel_vel_cmd_.get_value());
  EXPECT_EQ(1.0, right_wheel_vel_cmd_.get_value());

  // deactivated
  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(0.0, left_wheel_vel_cmd_.get_value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, right_wheel_vel_cmd_.get_value()) << "Wheels are halted on deactivate()";

  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(0.0, left_wheel_vel_cmd_.get_value());
  EXPECT_EQ(0.0, right_wheel_vel_cmd_.get_value());

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}
<<<<<<< HEAD
=======

// When not in chained mode, we want to test that
// 1. The controller is configurable and all lifecycle functions work properly
// 2. command_interfaces are set to 0.0 when cmd_vel_timeout_ is exceeded and on deactivation
// 3. command_interfaces are set to correct command values the command messages are not timed-out.
// In particular, make sure that the command_interface is not set to NaN right when it starts up.
TEST_F(TestDiffDriveController, chainable_controller_unchained_mode)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("wheel_separation", 0.4), rclcpp::Parameter("wheel_radius", 1.0)}),
    controller_interface::return_type::OK);
  // choose radius = 1 so that the command values (rev/s) are the same as the linear velocity (m/s)

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(controller_->is_chainable());
  ASSERT_TRUE(controller_->set_chained_mode(false));
  ASSERT_FALSE(controller_->is_in_chained_mode());

  ASSERT_TRUE(configure_succeeds(controller_));

  assignResourcesPosFeedback();

  ASSERT_TRUE(activate_succeeds(controller_));

  waitForSetup(executor);

  // Reference interfaces should be NaN on initialization
  // (Note: ordered_exported_reference_interfaces_ is protected, but this is
  // a FRIEND_TEST so we can use it)
  for (const auto & interface : controller_->ordered_exported_reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface->get_optional().value()));
  }
  // But NaNs should not propagate to command interfaces
  // (these are set to 0.1 and 0.2 in InitController)
  ASSERT_FALSE(std::isnan(left_wheel_vel_cmd_->get_optional().value()));
  ASSERT_FALSE(std::isnan(right_wheel_vel_cmd_->get_optional().value()));

  // Check that a late command message causes the command interfaces to be set to 0.0
  const double linear = 1.0;
  publish(linear, 0.0);

  // delay enough time to trigger the timeout (cmd_vel_timeout_ = 0.5s)
  controller_->wait_for_twist(executor);
  std::this_thread::sleep_for(std::chrono::milliseconds(501));

  ASSERT_EQ(
    controller_->update(pub_node->get_clock()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(0.0, left_wheel_vel_cmd_->get_optional().value())
    << "Wheels should halt if command message is older than cmd_vel_timeout";
  EXPECT_EQ(0.0, right_wheel_vel_cmd_->get_optional().value())
    << "Wheels should halt if command message is older than cmd_vel_timeout";

  // Now check that a timely published command message sets the command interfaces to the correct
  // values
  publish(linear, 0.0);
  // wait for msg is be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(linear, left_wheel_vel_cmd_->get_optional().value());
  EXPECT_EQ(linear, right_wheel_vel_cmd_->get_optional().value());

  // Now check that the command interfaces are set to 0.0 on deactivation
  // (despite calls to update())
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(0.0, left_wheel_vel_cmd_->get_optional().value())
    << "Wheels should be halted on deactivate()";
  EXPECT_EQ(0.0, right_wheel_vel_cmd_->get_optional().value())
    << "Wheels should be halted on deactivate()";

  // cleanup
  ASSERT_TRUE(cleanup_succeeds(controller_));
  EXPECT_EQ(0.0, left_wheel_vel_cmd_->get_optional().value())
    << "Wheels should be halted on cleanup()";
  EXPECT_EQ(0.0, right_wheel_vel_cmd_->get_optional().value())
    << "Wheels should be halted on cleanup()";

  ASSERT_TRUE(configure_succeeds(controller_));

  executor.cancel();
}

// When in chained mode, we want to test that
// 1. The controller is configurable and all lifecycle functions work properly
// 2. command_interfaces are set to 0.0 on deactivation
// 3. command_interfaces are set to correct command values (not set to NaN right when it starts up)
TEST_F(TestDiffDriveController, chainable_controller_chained_mode)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("wheel_separation", 0.4), rclcpp::Parameter("wheel_radius", 1.0)}),
    controller_interface::return_type::OK);
  // choose radius = 1 so that the command values (rev/s) are the same as the linear velocity (m/s)

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(controller_->is_chainable());
  ASSERT_TRUE(controller_->set_chained_mode(true));
  ASSERT_TRUE(controller_->is_in_chained_mode());

  ASSERT_TRUE(configure_succeeds(controller_));

  assignResourcesPosFeedback();

  ASSERT_TRUE(activate_succeeds(controller_));

  waitForSetup(executor);

  // Reference interfaces should be NaN on initialization
  for (const auto & interface : controller_->ordered_exported_reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface->get_optional().value()));
  }
  // But NaNs should not propagate to command interfaces
  // (these are set to 0.1 and 0.2 in InitController)
  ASSERT_FALSE(std::isnan(left_wheel_vel_cmd_->get_optional().value()));
  ASSERT_FALSE(std::isnan(right_wheel_vel_cmd_->get_optional().value()));

  // Imitate preceding controllers by setting ordered_exported_reference_interfaces_
  // (Note: ordered_exported_reference_interfaces_ is protected, but this is
  // a FRIEND_TEST so we can use it)
  const double linear = 3.0;
  controller_->ordered_exported_reference_interfaces_[0]->set_value(linear);
  controller_->ordered_exported_reference_interfaces_[1]->set_value(0.0);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(linear, left_wheel_vel_cmd_->get_optional().value());
  EXPECT_EQ(linear, right_wheel_vel_cmd_->get_optional().value());

  // Now check that the command interfaces are set to 0.0 on deactivation
  // (despite calls to update())
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(0.0, left_wheel_vel_cmd_->get_optional().value())
    << "Wheels should be halted on deactivate()";
  EXPECT_EQ(0.0, right_wheel_vel_cmd_->get_optional().value())
    << "Wheels should be halted on deactivate()";

  // cleanup
  ASSERT_TRUE(cleanup_succeeds(controller_));
  EXPECT_EQ(0.0, left_wheel_vel_cmd_->get_optional().value())
    << "Wheels should be halted on cleanup()";
  EXPECT_EQ(0.0, right_wheel_vel_cmd_->get_optional().value())
    << "Wheels should be halted on cleanup()";

  ASSERT_TRUE(configure_succeeds(controller_));

  executor.cancel();
}

TEST_F(TestDiffDriveController, reference_interfaces_are_properly_exported)
{
  ASSERT_EQ(
    InitController(left_wheel_names, right_wheel_names), controller_interface::return_type::OK);

  ASSERT_TRUE(configure_succeeds(controller_));

  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), 2)
    << "Expected exactly 2 reference interfaces: linear and angular";

  const std::string expected_linear_prefix_name =
    std::string(controller_->get_node()->get_name()) + std::string("/linear");
  const std::string expected_angular_prefix_name =
    std::string(controller_->get_node()->get_name()) + std::string("/angular");
  const std::string expected_linear_name =
    expected_linear_prefix_name + std::string("/") + hardware_interface::HW_IF_VELOCITY;
  const std::string expected_angular_name =
    expected_angular_prefix_name + std::string("/") + hardware_interface::HW_IF_VELOCITY;

  ASSERT_EQ(reference_interfaces[0]->get_name(), expected_linear_name);
  ASSERT_EQ(reference_interfaces[1]->get_name(), expected_angular_name);

  EXPECT_EQ(reference_interfaces[0]->get_prefix_name(), expected_linear_prefix_name);
  EXPECT_EQ(reference_interfaces[0]->get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  EXPECT_EQ(reference_interfaces[1]->get_prefix_name(), expected_angular_prefix_name);
  EXPECT_EQ(reference_interfaces[1]->get_interface_name(), hardware_interface::HW_IF_VELOCITY);
}

// Make sure that the controller is properly reset when deactivated
// and accepts new commands as expected when it is activated again.
TEST_F(TestDiffDriveController, deactivate_then_activate)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("wheel_separation", 0.4), rclcpp::Parameter("wheel_radius", 1.0)}),
    controller_interface::return_type::OK);
  // choose radius = 1 so that the command values (rev/s) are the same as the linear velocity (m/s)

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(controller_->set_chained_mode(false));

  ASSERT_TRUE(configure_succeeds(controller_));

  assignResourcesPosFeedback();

  ASSERT_TRUE(activate_succeeds(controller_));

  waitForSetup(executor);

  // Reference interfaces should be NaN on initialization
  // (Note: ordered_exported_reference_interfaces_ is protected, but this is
  // a FRIEND_TEST so we can use it)
  for (const auto & interface : controller_->ordered_exported_reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface->get_optional().value()));
  }
  // But NaNs should not propagate to command interfaces
  // (these are set to 0.1 and 0.2 in InitController)
  ASSERT_FALSE(std::isnan(left_wheel_vel_cmd_->get_optional().value()));
  ASSERT_FALSE(std::isnan(right_wheel_vel_cmd_->get_optional().value()));

  // published command message sets the command interfaces to the correct values
  const double linear = 1.0;
  publish(linear, 0.0);
  // wait for msg is be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(linear, left_wheel_vel_cmd_->get_optional().value());
  EXPECT_EQ(linear, right_wheel_vel_cmd_->get_optional().value());

  // Now check that the command interfaces are set to 0.0 on deactivation
  // (despite calls to update())
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(0.0, left_wheel_vel_cmd_->get_optional().value())
    << "Wheels should be halted on deactivate()";
  EXPECT_EQ(0.0, right_wheel_vel_cmd_->get_optional().value())
    << "Wheels should be halted on deactivate()";

  // Activate again
  ASSERT_TRUE(activate_succeeds(controller_));

  waitForSetup(executor);

  // (Note: ordered_exported_reference_interfaces_ is protected, but this is
  // a FRIEND_TEST so we can use it)
  for (const auto & interface : controller_->ordered_exported_reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface->get_optional().value()))
      << "Reference interfaces should initially be NaN on activation";
  }

  EXPECT_EQ(0.0, left_wheel_vel_cmd_->get_optional().value())
    << "Wheels should still have the same command as when they were last set (on deactivation)";
  EXPECT_EQ(0.0, right_wheel_vel_cmd_->get_optional().value())
    << "Wheels should still have the same command as when they were last set (on deactivation)";

  // A new command should work as expected
  publish(linear, 0.0);
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(linear, left_wheel_vel_cmd_->get_optional().value());
  EXPECT_EQ(linear, right_wheel_vel_cmd_->get_optional().value());

  // Deactivate again and cleanup
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_TRUE(cleanup_succeeds(controller_));
  executor.cancel();
}

TEST_F(TestDiffDriveController, command_with_zero_timestamp_is_accepted_with_warning)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("wheel_separation", 0.4), rclcpp::Parameter("wheel_radius", 1.0)}),
    controller_interface::return_type::OK);
  // choose radius = 1 so that the command values (rev/s) are the same as the linear velocity (m/s)

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(controller_->set_chained_mode(false));

  ASSERT_TRUE(configure_succeeds(controller_));

  assignResourcesPosFeedback();

  ASSERT_TRUE(activate_succeeds(controller_));

  waitForSetup(executor);

  // published command message with zero timestamp sets the command interfaces to the correct values
  const double linear = 1.0;
  publish_timestamped(linear, 0.0, rclcpp::Time(0, 0, RCL_ROS_TIME));
  // wait for msg is be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(linear, left_wheel_vel_cmd_->get_optional().value());
  EXPECT_EQ(linear, right_wheel_vel_cmd_->get_optional().value());

  // Deactivate and cleanup
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_TRUE(cleanup_succeeds(controller_));
  executor.cancel();
}

// Regression test for https://github.com/ros-controls/ros2_controllers/issues/440
// A cmd_vel_timeout of 0.0 disables the timeout: an arbitrarily old command must be
// preserved instead of being overridden with zero.
TEST_F(TestDiffDriveController, zero_cmd_vel_timeout_disables_timeout)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("wheel_separation", 0.4), rclcpp::Parameter("wheel_radius", 1.0),
       rclcpp::Parameter("cmd_vel_timeout", rclcpp::ParameterValue(0.0))}),
    controller_interface::return_type::OK);
  // choose radius = 1 so that the command values (rev/s) are the same as the linear velocity (m/s)

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(controller_->set_chained_mode(false));

  ASSERT_TRUE(configure_succeeds(controller_));

  assignResourcesPosFeedback();

  ASSERT_TRUE(activate_succeeds(controller_));

  waitForSetup(executor);

  // before any command arrives the stored command is NaN: this update exercises the
  // NaN-warning path with its finite throttle period while the timeout is disabled
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // publish a command with an explicit old nonzero timestamp, so its age at update time is
  // deterministic and would be far expired for any positive cmd_vel_timeout
  const double linear = 1.0;
  const rclcpp::Time command_stamp(1, 0, RCL_ROS_TIME);
  publish_timestamped(linear, 0.0, command_stamp);
  // wait for msg is be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(
      command_stamp + rclcpp::Duration::from_seconds(10.0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_EQ(linear, left_wheel_vel_cmd_->get_optional().value())
    << "Wheels should not halt when cmd_vel_timeout is disabled";
  EXPECT_EQ(linear, right_wheel_vel_cmd_->get_optional().value())
    << "Wheels should not halt when cmd_vel_timeout is disabled";

  // Deactivate and cleanup
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_TRUE(cleanup_succeeds(controller_));
  executor.cancel();
}

TEST_F(TestDiffDriveController, odometry_set_service)
{
  // 0. Initialize and activate the controller
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("wheel_separation", 0.4), rclcpp::Parameter("wheel_radius", 1.0)}),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(configure_succeeds(controller_));
  assignResourcesPosFeedback();

  EXPECT_EQ(0.01, left_wheel_vel_cmd_->get_optional().value());
  EXPECT_EQ(0.02, right_wheel_vel_cmd_->get_optional().value());

  ASSERT_TRUE(activate_succeeds(controller_));

  rclcpp::Time test_time(0, 0, RCL_ROS_TIME);
  rclcpp::Duration period = rclcpp::Duration::from_seconds(0.1);

  // 1. Move the robot first
  publish(1.0, 0.0);
  controller_->wait_for_twist(executor);
  controller_->update(test_time, period);
  test_time += period;

  // verify initial movement
  ASSERT_GT(controller_->odometry_.getX(), 0.0);

  // 2. Stop and call odom set service
  publish(0.0, 0.0);
  controller_->wait_for_twist(executor);
  auto set_request = std::make_shared<control_msgs::srv::SetOdometry::Request>();
  auto set_response = std::make_shared<control_msgs::srv::SetOdometry::Response>();
  set_request->x = 5.0;
  set_request->y = -2.0;
  set_request->yaw = 1.57079632679;  // 90 degrees
  controller_->set_odometry(nullptr, set_request, set_response);
  EXPECT_TRUE(set_response->success);

  // run update to process and verify odom values
  controller_->update(test_time, period);
  test_time += period;
  EXPECT_NEAR(controller_->odometry_.getX(), 5.0, 1e-6);
  EXPECT_NEAR(controller_->odometry_.getY(), -2.0, 1e-6);
  EXPECT_NEAR(controller_->odometry_.getHeading(), 1.57079632679, 1e-5);  // 90 deg

  // 3. Move again to ensure it still works
  publish(1.0, 0.0);  // we move in Y now
  controller_->wait_for_twist(executor);

  // simulate the movement by updating the position feedback
  position_values_[0] += 0.1;  // left wheel moved
  position_values_[1] += 0.1;  // right wheel moved
  std::ignore = left_wheel_pos_state_->set_value(position_values_[0]);
  std::ignore = right_wheel_pos_state_->set_value(position_values_[1]);
  controller_->update(test_time, period);
  test_time += period;
  EXPECT_GT(controller_->odometry_.getY(), -2.0);

  // 4. Deactivate and cleanup
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_TRUE(cleanup_succeeds(controller_));

  executor.cancel();
}

TEST_F(TestDiffDriveController, test_open_loop_odometry_with_clamped_input)
{
  const double max_linear_vel = 0.5;
  const double max_angular_vel = 0.5;

  // Initialize the controller with open_loop enabled and strict velocity limits
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("open_loop", rclcpp::ParameterValue(true)),
       rclcpp::Parameter("linear.x.max_velocity", rclcpp::ParameterValue(max_linear_vel)),
       rclcpp::Parameter("angular.z.max_velocity", rclcpp::ParameterValue(max_angular_vel))}),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(configure_succeeds(controller_));

  assignResourcesNoFeedback();

  ASSERT_TRUE(activate_succeeds(controller_));

  waitForSetup(executor);

  const double dt = 0.1;

  // Test Linear Clamping
  const double commanded_linear = 5.0;
  publish(commanded_linear, 0.0);
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
    controller_interface::return_type::OK);

  // Odometry should reflect the clamped linear velocity
  EXPECT_NEAR(controller_->odometry_.getLinear(), max_linear_vel, 1e-3);

  // Verify that the position integration uses the clamped value (0.5 * 0.1s = 0.05m)
  EXPECT_NEAR(controller_->odometry_.getX(), max_linear_vel * dt, 1e-3);

  // Test Angular Clamping
  const double commanded_angular = 5.0;
  publish(0.0, commanded_angular);
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
    controller_interface::return_type::OK);

  // Verify the angular velocity and heading integration are properly clamped
  EXPECT_NEAR(controller_->odometry_.getAngular(), max_angular_vel, 1e-3);
  EXPECT_NEAR(controller_->odometry_.getHeading(), max_angular_vel * dt, 1e-3);

  // Safely spin down the lifecycle
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_TRUE(cleanup_succeeds(controller_));
  executor.cancel();
}

TEST_F(TestDiffDriveController, test_open_loop_odometry_with_unclamped_input)
{
  // Initialize the controller with open_loop enabled without velocity limits
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {rclcpp::Parameter("open_loop", rclcpp::ParameterValue(true))}),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(configure_succeeds(controller_));

  assignResourcesNoFeedback();

  ASSERT_TRUE(activate_succeeds(controller_));

  waitForSetup(executor);

  const double dt = 0.1;

  // Test Linear
  const double commanded_linear = 5.0;
  publish(commanded_linear, 0.0);
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
    controller_interface::return_type::OK);

  // Odometry should exactly reflect the commanded linear velocity
  EXPECT_NEAR(controller_->odometry_.getLinear(), commanded_linear, 1e-3);

  // Verify that the position integration uses the commanded value (5.0 * 0.1s = 0.5m)
  EXPECT_NEAR(controller_->odometry_.getX(), commanded_linear * dt, 1e-3);

  // Test Angular
  const double commanded_angular = 5.0;
  publish(0.0, commanded_angular);
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
    controller_interface::return_type::OK);

  // Verify the angular velocity and heading integration use the commanded value
  EXPECT_NEAR(controller_->odometry_.getAngular(), commanded_angular, 1e-3);
  EXPECT_NEAR(controller_->odometry_.getHeading(), commanded_angular * dt, 1e-3);

  // Safely spin down the lifecycle
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_TRUE(cleanup_succeeds(controller_));
  executor.cancel();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
>>>>>>> 4d3aa44 (Allow disabling diff drive command timeout (#2503))
