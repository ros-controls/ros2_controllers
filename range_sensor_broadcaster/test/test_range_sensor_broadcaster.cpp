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
}

void RangeSensorBroadcasterTest::TearDown() { range_broadcaster_.reset(nullptr); }

controller_interface::return_type RangeSensorBroadcasterTest::init_broadcaster(
  std::string broadcaster_name)
{
  controller_interface::return_type result = controller_interface::return_type::ERROR;
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = broadcaster_name;
  params.robot_description = "";
  params.update_rate = 0;
  params.node_namespace = "";
  params.node_options = range_broadcaster_->define_custom_node_options();

  result = range_broadcaster_->init(params);

  if (controller_interface::return_type::OK == result)
  {
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    state_interfaces.emplace_back(range_, nullptr);

    range_broadcaster_->assign_interfaces({}, std::move(state_interfaces));
  }

  return result;
}

controller_interface::CallbackReturn RangeSensorBroadcasterTest::configure_broadcaster(
  std::vector<rclcpp::Parameter> & parameters)
{
  // Configure the broadcaster
  for (auto parameter : parameters)
  {
    range_broadcaster_->get_node()->set_parameter(parameter);
  }

  return range_broadcaster_->on_configure(rclcpp_lifecycle::State());
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
  // First Test without frame_id ERROR Expected
  init_broadcaster("test_range_sensor_broadcaster");

  std::vector<rclcpp::Parameter> parameters;
  // explicitly give an empty sensor name to generate an error
  parameters.emplace_back(rclcpp::Parameter("sensor_name", ""));
  ASSERT_EQ(configure_broadcaster(parameters), controller_interface::CallbackReturn::ERROR);
}

TEST_F(RangeSensorBroadcasterTest, Configure_RangeBroadcaster_Error_2)
{
  // Second Test without sensor_name ERROR Expected
  init_broadcaster("test_range_sensor_broadcaster");

  std::vector<rclcpp::Parameter> parameters;
  // explicitly give an empty frame_id to generate an error
  parameters.emplace_back(rclcpp::Parameter("frame_id", ""));
  ASSERT_EQ(configure_broadcaster(parameters), controller_interface::CallbackReturn::ERROR);
}

TEST_F(RangeSensorBroadcasterTest, Configure_RangeBroadcaster_Success)
{
  // Third Test without sensor_name SUCCESS Expected
  init_broadcaster("test_range_sensor_broadcaster");

  ASSERT_EQ(
    range_broadcaster_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // check interface configuration
  auto cmd_if_conf = range_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  auto state_if_conf = range_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(1lu));
}

TEST_F(RangeSensorBroadcasterTest, ActivateDeactivate_RangeBroadcaster_Success)
{
  init_broadcaster("test_range_sensor_broadcaster");

  range_broadcaster_->on_configure(rclcpp_lifecycle::State());

  ASSERT_EQ(
    range_broadcaster_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  // check interface configuration
  auto cmd_if_conf = range_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = range_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(1lu));
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  ASSERT_EQ(
    range_broadcaster_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

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

  range_broadcaster_->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(
    range_broadcaster_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  auto result = range_broadcaster_->update(
    range_broadcaster_->get_node()->get_clock()->now(), rclcpp::Duration::from_seconds(0.01));

  ASSERT_EQ(result, controller_interface::return_type::OK);
}

TEST_F(RangeSensorBroadcasterTest, Publish_RangeBroadcaster_Success)
{
  init_broadcaster("test_range_sensor_broadcaster");

  range_broadcaster_->on_configure(rclcpp_lifecycle::State());
  range_broadcaster_->on_activate(rclcpp_lifecycle::State());

  sensor_msgs::msg::Range range_msg;
  subscribe_and_get_message(range_msg);

  EXPECT_EQ(range_msg.header.frame_id, frame_id_);
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(static_cast<float>(sensor_range_)));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(static_cast<float>(field_of_view_)));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(static_cast<float>(min_range_)));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(static_cast<float>(max_range_)));
#if SENSOR_MSGS_VERSION_MAJOR >= 5
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(variance_));
#endif
}

TEST_F(RangeSensorBroadcasterTest, Publish_Bandaries_RangeBroadcaster_Success)
{
  init_broadcaster("test_range_sensor_broadcaster");

  range_broadcaster_->on_configure(rclcpp_lifecycle::State());
  range_broadcaster_->on_activate(rclcpp_lifecycle::State());

  sensor_msgs::msg::Range range_msg;

  sensor_range_ = 0.10f;
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

  range_broadcaster_->on_configure(rclcpp_lifecycle::State());
  range_broadcaster_->on_activate(rclcpp_lifecycle::State());

  sensor_msgs::msg::Range range_msg;

  sensor_range_ = 0.0;
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

int main(int argc, char ** argv)
{
  testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
