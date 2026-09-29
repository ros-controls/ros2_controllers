// Copyright 2023 flochre
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

/*
 * Authors: flochre
 */

#include <tuple>
#include <utility>

#include "test_range_sensor_broadcaster.hpp"

#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"

using testing::IsEmpty;
using testing::SizeIs;

void RangeSensorBroadcasterTest::SetUp()
{
  // initialize controller
  range_broadcaster_ = std::make_unique<range_sensor_broadcaster::RangeSensorBroadcaster>();
  range_ = std::make_shared<hardware_interface::StateInterface>(sensor_name_, "range");
  std::ignore = range_->set_value(sensor_range_);
}

void RangeSensorBroadcasterTest::TearDown() { range_broadcaster_.reset(nullptr); }

controller_interface::return_type RangeSensorBroadcasterTest::init_broadcaster(
  std::string broadcaster_name,
  std::vector<rclcpp::Parameter> parameter_overrides)
{
  controller_interface::return_type result = controller_interface::return_type::ERROR;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = broadcaster_name;
  params.robot_description = "";
  params.update_rate = 0;
  params.node_namespace = "";
  params.node_options = range_broadcaster_->define_custom_node_options();
  params.node_options.parameter_overrides(parameter_overrides);

  result = range_broadcaster_->init(params);

  if (controller_interface::return_type::OK == result)
  {
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    state_interfaces.emplace_back(range_, nullptr);

    range_broadcaster_->assign_interfaces({}, std::move(state_interfaces));
  }

  return result;
}

void RangeSensorBroadcasterTest::configure_broadcaster(std::vector<rclcpp::Parameter> & parameters)
{
  // Configure the broadcaster
  for (auto parameter : parameters)
  {
    range_broadcaster_->get_node()->set_parameter(parameter);
  }
}

void RangeSensorBroadcasterTest::subscribe_and_get_message(sensor_msgs::msg::Range & range_msg)
{
  // create a new subscriber
  sensor_msgs::msg::Range::SharedPtr received_msg;
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const sensor_msgs::msg::Range::SharedPtr msg) { received_msg = msg; };
  auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::Range>(
    "/test_range_sensor_broadcaster/range", 10, subs_callback);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_subscription_node.get_node_base_interface());

  // call update to publish the test value
  // since update doesn't guarantee a published message, republish until received
  int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
  while (max_sub_check_loop_count--)
  {
    range_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
    const auto timeout = std::chrono::milliseconds{5};
    const auto until = test_subscription_node.get_clock()->now() + timeout;
    while (!received_msg && test_subscription_node.get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    // check if message has been received
    if (received_msg.get())
    {
      break;
    }
  }
  ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                            "controller/broadcaster update loop";
  ASSERT_TRUE(received_msg);

  // take message from subscription
  range_msg = *received_msg;
}

TEST_F(RangeSensorBroadcasterTest, Initialize_RangeBroadcaster_Exception)
{
  ASSERT_THROW(init_broadcaster(""), std::exception);
}

TEST_F(RangeSensorBroadcasterTest, Initialize_RangeBroadcaster_Success)
{
  ASSERT_EQ(
    init_broadcaster("test_range_sensor_broadcaster"), controller_interface::return_type::OK);
}

TEST_F(RangeSensorBroadcasterTest, Configure_RangeBroadcaster_Error_1)
{
  const auto result = init_broadcaster(
    "test_range_sensor_broadcaster",
    {{"sensor_name", ""}, {"frame_id", frame_id_}});
  if (result == controller_interface::return_type::OK)
  {
    ASSERT_FALSE(configure_succeeds(range_broadcaster_));
  }
}

TEST_F(RangeSensorBroadcasterTest, Configure_RangeBroadcaster_Error_2)
{
  const auto result = init_broadcaster(
    "test_range_sensor_broadcaster",
    {{"sensor_name", sensor_name_}, {"frame_id", ""}});
  if (result == controller_interface::return_type::OK)
  {
    ASSERT_FALSE(configure_succeeds(range_broadcaster_));
  }
}

TEST_F(RangeSensorBroadcasterTest, Configure_RangeBroadcaster_Success)
{
  // Third Test without sensor_name SUCCESS Expected
  init_broadcaster("test_range_sensor_broadcaster");

  ASSERT_TRUE(configure_succeeds(range_broadcaster_));

  // check interface configuration
  auto cmd_if_conf = range_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  auto state_if_conf = range_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(1lu));
}

TEST_F(RangeSensorBroadcasterTest, ActivateDeactivate_RangeBroadcaster_Success)
{
  init_broadcaster("test_range_sensor_broadcaster");

  ASSERT_TRUE(configure_succeeds(range_broadcaster_));

  ASSERT_TRUE(activate_succeeds(range_broadcaster_));

  // check interface configuration
  auto cmd_if_conf = range_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = range_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(1lu));
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  ASSERT_TRUE(deactivate_succeeds(range_broadcaster_));

  // check interface configuration
  cmd_if_conf = range_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  state_if_conf = range_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(1lu));  // did not change
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(RangeSensorBroadcasterTest, Update_RangeBroadcaster_Success)
{
  init_broadcaster("test_range_sensor_broadcaster");

  ASSERT_TRUE(configure_succeeds(range_broadcaster_));
  ASSERT_TRUE(activate_succeeds(range_broadcaster_));

  auto result = range_broadcaster_->update(
    range_broadcaster_->get_node()->get_clock()->now(), rclcpp::Duration::from_seconds(0.01));

  ASSERT_EQ(result, controller_interface::return_type::OK);
}

TEST_F(RangeSensorBroadcasterTest, Publish_RangeBroadcaster_Success)
{
  init_broadcaster("test_range_sensor_broadcaster");

  ASSERT_TRUE(configure_succeeds(range_broadcaster_));
  ASSERT_TRUE(activate_succeeds(range_broadcaster_));

  sensor_msgs::msg::Range range_msg;
  subscribe_and_get_message(range_msg);

  EXPECT_EQ(range_msg.header.frame_id, frame_id_);
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(static_cast<float>(sensor_range_)));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(static_cast<float>(field_of_view_)));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(static_cast<float>(min_range_)));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(static_cast<float>(max_range_)));
#if SENSOR_MSGS_VERSION_MAJOR >= 5
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(static_cast<float>(variance_)));
#endif
}

TEST_F(RangeSensorBroadcasterTest, Publish_Bandaries_RangeBroadcaster_Success)
{
  init_broadcaster("test_range_sensor_broadcaster");

  ASSERT_TRUE(configure_succeeds(range_broadcaster_));
  ASSERT_TRUE(activate_succeeds(range_broadcaster_));

  sensor_msgs::msg::Range range_msg;

  sensor_range_ = 0.10f;
  std::ignore = range_->set_value(sensor_range_);
  subscribe_and_get_message(range_msg);

  EXPECT_EQ(range_msg.header.frame_id, frame_id_);
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(static_cast<float>(sensor_range_)));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(static_cast<float>(field_of_view_)));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(static_cast<float>(min_range_)));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(static_cast<float>(max_range_)));
#if SENSOR_MSGS_VERSION_MAJOR >= 5
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(static_cast<float>(variance_)));
#endif

  sensor_range_ = 4.0;
  std::ignore = range_->set_value(sensor_range_);
  subscribe_and_get_message(range_msg);

  EXPECT_EQ(range_msg.header.frame_id, frame_id_);
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(static_cast<float>(sensor_range_)));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(static_cast<float>(field_of_view_)));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(static_cast<float>(min_range_)));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(static_cast<float>(max_range_)));
#if SENSOR_MSGS_VERSION_MAJOR >= 5
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(static_cast<float>(variance_)));
#endif
}

TEST_F(RangeSensorBroadcasterTest, Publish_OutOfBandaries_RangeBroadcaster_Success)
{
  init_broadcaster("test_range_sensor_broadcaster");

  ASSERT_TRUE(configure_succeeds(range_broadcaster_));
  ASSERT_TRUE(activate_succeeds(range_broadcaster_));

  sensor_msgs::msg::Range range_msg;

  sensor_range_ = 0.0;
  std::ignore = range_->set_value(sensor_range_);
  subscribe_and_get_message(range_msg);

  EXPECT_EQ(range_msg.header.frame_id, frame_id_);
  // Even out of boundaries you will get the out_of_range range value
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(static_cast<float>(sensor_range_)));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(static_cast<float>(field_of_view_)));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(static_cast<float>(min_range_)));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(static_cast<float>(max_range_)));
#if SENSOR_MSGS_VERSION_MAJOR >= 5
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(static_cast<float>(variance_)));
#endif

  sensor_range_ = 6.0;
  std::ignore = range_->set_value(sensor_range_);
  subscribe_and_get_message(range_msg);

  EXPECT_EQ(range_msg.header.frame_id, frame_id_);
  // Even out of boundaries you will get the out_of_range range value
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(static_cast<float>(sensor_range_)));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(static_cast<float>(field_of_view_)));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(static_cast<float>(min_range_)));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(static_cast<float>(max_range_)));
#if SENSOR_MSGS_VERSION_MAJOR >= 5
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(static_cast<float>(variance_)));
#endif
}

TEST_F(RangeSensorBroadcasterTest, ConfiguredVarianceIsPublished)
{
  init_broadcaster("test_range_sensor_broadcaster");
  ASSERT_TRUE(configure_succeeds(range_broadcaster_));
  ASSERT_TRUE(activate_succeeds(range_broadcaster_));

  sensor_msgs::msg::Range msg;
  subscribe_and_get_message(msg);
  EXPECT_FLOAT_EQ(msg.variance, static_cast<float>(variance_));
}

TEST_F(RangeSensorBroadcasterTest, AcceptedParameterUpdatesAreAppliedOrRejected)
{
  init_broadcaster("test_range_sensor_broadcaster");
  ASSERT_TRUE(configure_succeeds(range_broadcaster_));
  ASSERT_TRUE(activate_succeeds(range_broadcaster_));

  const std::string new_sensor = "updated_range_sensor";
  const std::string new_frame = "updated_range_frame";
  const int new_radiation = 0;
  const double new_fov = 0.25;
  const double new_min = 0.2;
  const double new_max = 6.0;
  const double new_variance = 0.5;

  const auto sensor_result = range_broadcaster_->get_node()->set_parameter(
    rclcpp::Parameter("sensor_name", new_sensor));
  const auto frame_result = range_broadcaster_->get_node()->set_parameter(
    rclcpp::Parameter("frame_id", new_frame));
  const auto radiation_result = range_broadcaster_->get_node()->set_parameter(
    rclcpp::Parameter("radiation_type", new_radiation));
  const auto fov_result = range_broadcaster_->get_node()->set_parameter(
    rclcpp::Parameter("field_of_view", new_fov));
  const auto min_result = range_broadcaster_->get_node()->set_parameter(
    rclcpp::Parameter("min_range", new_min));
  const auto max_result = range_broadcaster_->get_node()->set_parameter(
    rclcpp::Parameter("max_range", new_max));
  const auto variance_result = range_broadcaster_->get_node()->set_parameter(
    rclcpp::Parameter("variance", new_variance));

  EXPECT_FALSE(sensor_result.successful);
  EXPECT_FALSE(frame_result.successful);
  EXPECT_FALSE(radiation_result.successful);
  EXPECT_FALSE(fov_result.successful);
  EXPECT_FALSE(min_result.successful);
  EXPECT_FALSE(max_result.successful);
  EXPECT_FALSE(variance_result.successful);

  const auto state_if_conf = range_broadcaster_->state_interface_configuration();
  EXPECT_THAT(state_if_conf.names, testing::ElementsAre(sensor_name_ + std::string("/range")));

  sensor_msgs::msg::Range msg;
  subscribe_and_get_message(msg);
  EXPECT_EQ(msg.header.frame_id, frame_id_);
  EXPECT_EQ(msg.radiation_type, static_cast<uint8_t>(radiation_type_));
  EXPECT_FLOAT_EQ(msg.field_of_view, static_cast<float>(field_of_view_));
  EXPECT_FLOAT_EQ(msg.min_range, static_cast<float>(min_range_));
  EXPECT_FLOAT_EQ(msg.max_range, static_cast<float>(max_range_));
  EXPECT_FLOAT_EQ(msg.range, static_cast<float>(sensor_range_));
}

namespace
{
std::vector<rclcpp::Parameter> valid_initial_parameters()
{
  return {
    {"sensor_name", "range_sensor"},
    {"frame_id", "range_sensor_frame"},
    {"radiation_type", 1},
    {"field_of_view", 0.1},
    {"min_range", 0.1},
    {"max_range", 7.0},
    {"variance", 1.0},
  };
}
}  // namespace

TEST_F(RangeSensorBroadcasterTest, RejectsInvalidRadiationType)
{
  auto parameters = valid_initial_parameters();
  parameters[2] = rclcpp::Parameter("radiation_type", 2);
  EXPECT_NE(
    init_broadcaster("test_range_sensor_broadcaster", parameters),
    controller_interface::return_type::OK);
}

TEST_F(RangeSensorBroadcasterTest, RejectsNonPositiveFieldOfView)
{
  auto parameters = valid_initial_parameters();
  parameters[3] = rclcpp::Parameter("field_of_view", 0.0);
  EXPECT_NE(
    init_broadcaster("test_range_sensor_broadcaster", parameters),
    controller_interface::return_type::OK);
}

TEST_F(RangeSensorBroadcasterTest, RejectsInvertedRange)
{
  auto parameters = valid_initial_parameters();
  parameters[4] = rclcpp::Parameter("min_range", 5.0);
  parameters[5] = rclcpp::Parameter("max_range", 1.0);
  ASSERT_EQ(
    init_broadcaster("test_range_sensor_broadcaster", parameters),
    controller_interface::return_type::OK);
  EXPECT_FALSE(configure_succeeds(range_broadcaster_));
}

TEST_F(RangeSensorBroadcasterTest, RejectsNegativeVariance)
{
  auto parameters = valid_initial_parameters();
  parameters[6] = rclcpp::Parameter("variance", -0.1);
  EXPECT_NE(
    init_broadcaster("test_range_sensor_broadcaster", parameters),
    controller_interface::return_type::OK);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
