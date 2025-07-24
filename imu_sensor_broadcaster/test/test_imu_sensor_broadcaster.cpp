// Copyright 2021 PAL Robotics SL.
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
 * Authors: Subhas Das, Denis Stogl, Victor Lopez
 */

#include "test_imu_sensor_broadcaster.hpp"

#include <memory>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/convert.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "imu_sensor_broadcaster/imu_transform.hpp"

using hardware_interface::LoanedStateInterface;
using testing::IsEmpty;
using testing::SizeIs;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace

void IMUSensorBroadcasterTest::SetUpTestCase() {}

void IMUSensorBroadcasterTest::TearDownTestCase() {}

void IMUSensorBroadcasterTest::SetUp()
{
  // initialize controller
  imu_broadcaster_ = std::make_unique<FriendIMUSensorBroadcaster>();
}

void IMUSensorBroadcasterTest::TearDown() { imu_broadcaster_.reset(nullptr); }

void IMUSensorBroadcasterTest::SetUpIMUBroadcaster(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto node_options = imu_broadcaster_->define_custom_node_options();
  node_options.parameter_overrides(parameters);

  const auto result =
    imu_broadcaster_->init("test_imu_sensor_broadcaster", "", 0, "", node_options);
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(imu_orientation_x_);
  state_ifs.emplace_back(imu_orientation_y_);
  state_ifs.emplace_back(imu_orientation_z_);
  state_ifs.emplace_back(imu_orientation_w_);
  state_ifs.emplace_back(imu_angular_velocity_x_);
  state_ifs.emplace_back(imu_angular_velocity_y_);
  state_ifs.emplace_back(imu_angular_velocity_z_);
  state_ifs.emplace_back(imu_linear_acceleration_x_);
  state_ifs.emplace_back(imu_linear_acceleration_y_);
  state_ifs.emplace_back(imu_linear_acceleration_z_);

  imu_broadcaster_->assign_interfaces({}, std::move(state_ifs));
}

void IMUSensorBroadcasterTest::subscribe_and_get_message(sensor_msgs::msg::Imu & imu_msg)
{
  // create a new subscriber
  sensor_msgs::msg::Imu::SharedPtr received_msg;
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const sensor_msgs::msg::Imu::SharedPtr msg) { received_msg = msg; };
  auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::Imu>(
    "/test_imu_sensor_broadcaster/imu", 10, subs_callback);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_subscription_node.get_node_base_interface());

  // call update to publish the test value
  // since update doesn't guarantee a published message, republish until received
  int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
  while (max_sub_check_loop_count--)
  {
    imu_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
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
  imu_msg = *received_msg;
}

TEST_F(IMUSensorBroadcasterTest, SensorName_Configure_Success)
{
  SetUpIMUBroadcaster();

  // set the 'sensor_name'
  imu_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});

  // set the 'frame_id'
  imu_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure passed
  ASSERT_EQ(imu_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check interface configuration
  auto cmd_if_conf = imu_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = imu_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(10lu));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(IMUSensorBroadcasterTest, SensorName_Activate_Success)
{
  SetUpIMUBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  imu_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  imu_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure and activate success
  ASSERT_EQ(imu_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(imu_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check interface configuration
  auto cmd_if_conf = imu_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = imu_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(10lu));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  // deactivate passed
  ASSERT_EQ(imu_broadcaster_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check interface configuration
  cmd_if_conf = imu_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  state_if_conf = imu_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(10lu));  // did not change
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(IMUSensorBroadcasterTest, SensorName_Update_Success)
{
  SetUpIMUBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  imu_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  imu_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(imu_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(imu_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    imu_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(IMUSensorBroadcasterTest, SensorName_Publish_Success)
{
  SetUpIMUBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  imu_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  imu_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(imu_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(imu_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  sensor_msgs::msg::Imu imu_msg;
  subscribe_and_get_message(imu_msg);

  EXPECT_EQ(imu_msg.header.frame_id, frame_id_);
  EXPECT_EQ(imu_msg.orientation.x, sensor_values_[0]);
  EXPECT_EQ(imu_msg.orientation.y, sensor_values_[1]);
  EXPECT_EQ(imu_msg.orientation.z, sensor_values_[2]);
  EXPECT_EQ(imu_msg.orientation.w, sensor_values_[3]);
  EXPECT_EQ(imu_msg.angular_velocity.x, sensor_values_[4]);
  EXPECT_EQ(imu_msg.angular_velocity.y, sensor_values_[5]);
  EXPECT_EQ(imu_msg.angular_velocity.z, sensor_values_[6]);
  EXPECT_EQ(imu_msg.linear_acceleration.x, sensor_values_[7]);
  EXPECT_EQ(imu_msg.linear_acceleration.y, sensor_values_[8]);
  EXPECT_EQ(imu_msg.linear_acceleration.z, sensor_values_[9]);

  for (size_t i = 0; i < 9; ++i)
  {
    EXPECT_EQ(imu_msg.orientation_covariance[i], 0.0);
    EXPECT_EQ(imu_msg.angular_velocity_covariance[i], 0.0);
    EXPECT_EQ(imu_msg.linear_acceleration_covariance[i], 0.0);
  }
}

TEST_F(IMUSensorBroadcasterTest, SensorStatePublishTest_with_Calibration)
{
  SetUpIMUBroadcaster(
    {// set the params 'sensor_name' and 'frame_id'
     rclcpp::Parameter("sensor_name", sensor_name_), rclcpp::Parameter("frame_id", frame_id_),
     // use Q = ENU->NED transform, same as in test_imu_transform.cpp
     rclcpp::Parameter("calibration.roll", M_PI), rclcpp::Parameter("calibration.pitch", 0.),
     rclcpp::Parameter("calibration.yaw", M_PI_2)});

  ASSERT_EQ(imu_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(imu_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  sensor_msgs::msg::Imu imu_msg;
  subscribe_and_get_message(imu_msg);

  EXPECT_EQ(imu_msg.header.frame_id, frame_id_);

  // Transforming orientation means expressing the attitude of the new frame in the same world frame
  // (i.e. you have data in imu frame and want to ask what is the world-referenced orientation of
  // the base_link frame that is attached to this IMU). This is why the orientation change goes the
  // other way than the transform.
  tf2::Quaternion rot;
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(M_PI, 0., M_PI_2);
  tf2::convert(imu_msg.orientation, rot);
  tf2::Quaternion init(0.1826, 0.3651, 0.5477, 0.7303);  // see sensor_values_[0-3]
  EXPECT_NEAR(0, rot.angleShortestPath(init * tf_quat.inverse()), 1e-6);
  EXPECT_NEAR(imu_msg.angular_velocity.x, sensor_values_[5], 1e-5);
  EXPECT_NEAR(imu_msg.angular_velocity.y, sensor_values_[4], 1e-5);
  EXPECT_NEAR(imu_msg.angular_velocity.z, -sensor_values_[6], 1e-5);
  EXPECT_NEAR(imu_msg.linear_acceleration.x, sensor_values_[8], 1e-5);
  EXPECT_NEAR(imu_msg.linear_acceleration.y, sensor_values_[7], 1e-5);
  EXPECT_NEAR(imu_msg.linear_acceleration.z, -sensor_values_[9], 1e-5);

  for (size_t i = 0; i < 9; ++i)
  {
    EXPECT_EQ(imu_msg.orientation_covariance[i], 0.0);
    EXPECT_EQ(imu_msg.angular_velocity_covariance[i], 0.0);
    EXPECT_EQ(imu_msg.linear_acceleration_covariance[i], 0.0);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
