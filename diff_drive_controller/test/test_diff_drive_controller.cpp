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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diff_drive_controller/diff_drive_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"

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
    (void)received_velocity_msg_ptr_.get_latest(ret);
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
  void SetUp() override
  {
    // use the name of the test as the controller name (i.e, the node name) to be able to set
    // parameters from yaml for each test
    controller_name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    controller_ = std::make_unique<TestableDiffDriveController>();

    pub_node = std::make_shared<rclcpp::Node>("velocity_publisher");
    velocity_publisher = pub_node->create_publisher<geometry_msgs::msg::TwistStamped>(
      controller_name + "/cmd_vel", rclcpp::SystemDefaultsQoS());
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

    return controller_->init(controller_name, urdf_, 0, ns, node_options);
  }

  std::string controller_name;
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

  const std::string urdf_ = "";
};

TEST_F(TestDiffDriveController, init_fails_without_parameters)
{
  const auto ret =
    controller_->init(controller_name, urdf_, 0, "", controller_->define_custom_node_options());
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

TEST_F(TestDiffDriveController, test_speed_limiter)
{
  ASSERT_EQ(
    InitController(
      left_wheel_names, right_wheel_names,
      {
        rclcpp::Parameter("linear.x.max_acceleration", rclcpp::ParameterValue(2.0)),
        rclcpp::Parameter("linear.x.max_deceleration", rclcpp::ParameterValue(-4.0)),
        rclcpp::Parameter("linear.x.max_acceleration_reverse", rclcpp::ParameterValue(-8.0)),
        rclcpp::Parameter("linear.x.max_deceleration_reverse", rclcpp::ParameterValue(10.0)),
      }),
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
  publish(0.0, 0.0);
  // wait for msg is be published to the system
  controller_->wait_for_twist(executor);

  // wait for the speed limiter to fill the queue
  for (int i = 0; i < 3; ++i)
  {
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    EXPECT_NEAR(0.0, left_wheel_vel_cmd_.get_value(), 1e-3);
    EXPECT_NEAR(0.0, right_wheel_vel_cmd_.get_value(), 1e-3);
  }

  const double dt = 0.001;
  const double wheel_radius = 0.1;

  // we send four steps of acceleration, deceleration, reverse acceleration and reverse deceleration
  {
    const double linear = 1.0;
    // send msg
    publish(linear, 0.0);
    // wait for msg is be published to the system
    controller_->wait_for_twist(executor);

    // should be in acceleration limit
    const double time_acc = linear / 2.0;
    for (int i = 0; i < floor(time_acc / dt) - 1; ++i)
    {
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
        controller_interface::return_type::OK);
      EXPECT_GT(linear / wheel_radius, left_wheel_vel_cmd_.get_value())
        << "at t: " << i * dt << "s, but should be t: " << time_acc;
      EXPECT_GT(linear / wheel_radius, right_wheel_vel_cmd_.get_value())
        << "at t: " << i * dt << "s, but should be t: " << time_acc;
    }
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
      controller_interface::return_type::OK);
    EXPECT_NEAR(linear / wheel_radius, left_wheel_vel_cmd_.get_value(), 1e-3);
    EXPECT_NEAR(linear / wheel_radius, right_wheel_vel_cmd_.get_value(), 1e-3);

    // wait for the speed limiter to fill the queue
    for (int i = 0; i < 3; ++i)
    {
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
        controller_interface::return_type::OK);
      EXPECT_EQ(linear / wheel_radius, left_wheel_vel_cmd_.get_value());
      EXPECT_EQ(linear / wheel_radius, right_wheel_vel_cmd_.get_value());
    }
  }

  {
    const double linear = 0.0;
    // send msg
    publish(linear, 0.0);
    // wait for msg is be published to the system
    controller_->wait_for_twist(executor);

    // should be in acceleration limit
    const double time_acc = -1.0 / (-4.0);
    for (int i = 0; i < floor(time_acc / dt) - 1; ++i)
    {
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
        controller_interface::return_type::OK);
      EXPECT_LT(linear / wheel_radius, left_wheel_vel_cmd_.get_value())
        << "at t: " << i * dt << "s, but should be t: " << time_acc;
      EXPECT_LT(linear / wheel_radius, right_wheel_vel_cmd_.get_value())
        << "at t: " << i * dt << "s, but should be t: " << time_acc;
    }
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
      controller_interface::return_type::OK);
    EXPECT_NEAR(linear / wheel_radius, left_wheel_vel_cmd_.get_value(), 1e-3);
    EXPECT_NEAR(linear / wheel_radius, right_wheel_vel_cmd_.get_value(), 1e-3);

    // wait for the speed limiter to fill the queue
    for (int i = 0; i < 3; ++i)
    {
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
        controller_interface::return_type::OK);
      EXPECT_NEAR(linear / wheel_radius, left_wheel_vel_cmd_.get_value(), 1e-3);
      EXPECT_NEAR(linear / wheel_radius, right_wheel_vel_cmd_.get_value(), 1e-3);
    }
  }

  {
    const double linear = -1.0;
    // send msg
    publish(linear, 0.0);
    // wait for msg is be published to the system
    controller_->wait_for_twist(executor);

    // should be in acceleration limit
    const double time_acc = -1.0 / (-8.0);
    for (int i = 0; i < floor(time_acc / dt) - 1; ++i)
    {
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
        controller_interface::return_type::OK);
      EXPECT_LT(linear / wheel_radius, left_wheel_vel_cmd_.get_value())
        << "at t: " << i * dt << "s, but should be t: " << time_acc;
      EXPECT_LT(linear / wheel_radius, right_wheel_vel_cmd_.get_value())
        << "at t: " << i * dt << "s, but should be t: " << time_acc;
    }
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
      controller_interface::return_type::OK);
    EXPECT_NEAR(linear / wheel_radius, left_wheel_vel_cmd_.get_value(), 1e-3);
    EXPECT_NEAR(linear / wheel_radius, right_wheel_vel_cmd_.get_value(), 1e-3);

    // wait for the speed limiter to fill the queue
    for (int i = 0; i < 3; ++i)
    {
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
        controller_interface::return_type::OK);
      EXPECT_NEAR(linear / wheel_radius, left_wheel_vel_cmd_.get_value(), 1e-3);
      EXPECT_NEAR(linear / wheel_radius, right_wheel_vel_cmd_.get_value(), 1e-3);
    }
  }

  {
    const double linear = 0.0;
    // send msg
    publish(linear, 0.0);
    // wait for msg is be published to the system
    controller_->wait_for_twist(executor);

    // should be in acceleration limit
    const double time_acc = 1.0 / (10.0);
    for (int i = 0; i < floor(time_acc / dt) - 1; ++i)
    {
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
        controller_interface::return_type::OK);
      EXPECT_GT(linear / wheel_radius, left_wheel_vel_cmd_.get_value())
        << "at t: " << i * dt << "s, but should be t: " << time_acc;
      EXPECT_GT(linear / wheel_radius, right_wheel_vel_cmd_.get_value())
        << "at t: " << i * dt << "s, but should be t: " << time_acc;
    }
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
      controller_interface::return_type::OK);
    EXPECT_NEAR(linear / wheel_radius, left_wheel_vel_cmd_.get_value(), 1e-3);
    EXPECT_NEAR(linear / wheel_radius, right_wheel_vel_cmd_.get_value(), 1e-3);

    // wait for the speed limiter to fill the queue
    for (int i = 0; i < 3; ++i)
    {
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
        controller_interface::return_type::OK);
      EXPECT_NEAR(linear / wheel_radius, left_wheel_vel_cmd_.get_value(), 1e-3);
      EXPECT_NEAR(linear / wheel_radius, right_wheel_vel_cmd_.get_value(), 1e-3);
    }
  }
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
