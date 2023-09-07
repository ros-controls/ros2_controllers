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
  result = range_broadcaster_->init(broadcaster_name);

  if (controller_interface::return_type::OK == result)
  {
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    state_interfaces.emplace_back(range_);

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
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const sensor_msgs::msg::Range::SharedPtr) {};
  auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::Range>(
    "/test_range_sensor_broadcaster/range", 10, subs_callback);

  // call update to publish the test value
  // since update doesn't guarantee a published message, republish until received
  int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
  rclcpp::WaitSet wait_set;          // block used to wait on message
  wait_set.add_subscription(subscription);
  while (max_sub_check_loop_count--)
  {
    range_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
    // check if message has been received
    if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready)
    {
      break;
    }
  }
  ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                            "controller/broadcaster update loop";

  // take message from subscription
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(range_msg, msg_info));
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
}

TEST_F(RangeSensorBroadcasterTest, Activate_RangeBroadcaster_Success)
{
  init_broadcaster("test_range_sensor_broadcaster");

  range_broadcaster_->on_configure(rclcpp_lifecycle::State());

  ASSERT_EQ(
    range_broadcaster_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
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
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(sensor_range_));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(field_of_view_));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(min_range_));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(max_range_));
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(variance_));
}

TEST_F(RangeSensorBroadcasterTest, Publish_Bandaries_RangeBroadcaster_Success)
{
  init_broadcaster("test_range_sensor_broadcaster");

  range_broadcaster_->on_configure(rclcpp_lifecycle::State());
  range_broadcaster_->on_activate(rclcpp_lifecycle::State());

  sensor_msgs::msg::Range range_msg;

  sensor_range_ = 0.10;
  subscribe_and_get_message(range_msg);

  EXPECT_EQ(range_msg.header.frame_id, frame_id_);
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(sensor_range_));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(field_of_view_));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(min_range_));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(max_range_));
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(variance_));

  sensor_range_ = 4.0;
  subscribe_and_get_message(range_msg);

  EXPECT_EQ(range_msg.header.frame_id, frame_id_);
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(sensor_range_));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(field_of_view_));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(min_range_));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(max_range_));
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(variance_));
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
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(sensor_range_));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(field_of_view_));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(min_range_));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(max_range_));
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(variance_));

  sensor_range_ = 6.0;
  subscribe_and_get_message(range_msg);

  EXPECT_EQ(range_msg.header.frame_id, frame_id_);
  // Even out of boundaries you will get the out_of_range range value
  EXPECT_THAT(range_msg.range, ::testing::FloatEq(sensor_range_));
  EXPECT_EQ(range_msg.radiation_type, radiation_type_);
  EXPECT_THAT(range_msg.field_of_view, ::testing::FloatEq(field_of_view_));
  EXPECT_THAT(range_msg.min_range, ::testing::FloatEq(min_range_));
  EXPECT_THAT(range_msg.max_range, ::testing::FloatEq(max_range_));
  EXPECT_THAT(range_msg.variance, ::testing::FloatEq(variance_));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
