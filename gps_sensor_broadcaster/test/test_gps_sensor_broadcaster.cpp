// Copyright 2025 ros2_control development team
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
 * Authors: Wiktor Bajor, Jakub Delicat
 */

#include <utility>

#include <rclcpp/node.hpp>
#include <rclcpp/wait_result_kind.hpp>
#include <rclcpp/wait_set.hpp>
#include "gmock/gmock.h"
#include "gps_sensor_broadcaster/gps_sensor_broadcaster.hpp"
#include "gps_sensor_broadcaster/gps_sensor_broadcaster_parameters.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using hardware_interface::LoanedStateInterface;
using lifecycle_msgs::msg::State;
using callback_return_type =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{
constexpr uint16_t GPS_SERVICE = 1;
constexpr uint16_t COVARIANCE_TYPE_KNOWN = 3;
constexpr uint16_t COVARIANCE_TYPE_DIAGONAL_KNOWN = 2;
rclcpp::WaitResultKind wait_for(std::shared_ptr<rclcpp::SubscriptionBase> subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  return wait_set.wait().kind();
}

rclcpp::NodeOptions create_node_options_with_overriden_parameters(
  std::vector<rclcpp::Parameter> parameters)
{
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  return node_options;
}
}  // namespace

class GPSSensorBroadcasterTest : public ::testing::Test
{
public:
  GPSSensorBroadcasterTest() { rclcpp::init(0, nullptr); }

  ~GPSSensorBroadcasterTest() { rclcpp::shutdown(); }

  void SetUp()
  {
    gps_broadcaster_ = std::make_unique<gps_sensor_broadcaster::GPSSensorBroadcaster>();
  }

  void TearDown() { gps_broadcaster_.reset(nullptr); }

  template <
    semantic_components::GPSSensorOption sensor_option =
      semantic_components::GPSSensorOption::WithoutCovariance>
  void setup_gps_broadcaster()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(gps_status_, nullptr);
    state_ifs.emplace_back(gps_service_, nullptr);
    state_ifs.emplace_back(gps_latitude_, nullptr);
    state_ifs.emplace_back(gps_longitude_, nullptr);
    state_ifs.emplace_back(gps_altitude_, nullptr);
    if constexpr (sensor_option == semantic_components::GPSSensorOption::WithCovariance)
    {
      state_ifs.emplace_back(latitude_covariance_, nullptr);
      state_ifs.emplace_back(longitude_covariance_, nullptr);
      state_ifs.emplace_back(altitude_covariance_, nullptr);
    }

    gps_broadcaster_->assign_interfaces({}, std::move(state_ifs));
  }

  sensor_msgs::msg::NavSatFix subscribe_and_get_message()
  {
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::NavSatFix>(
      "/test_gps_sensor_broadcaster/gps/fix", 10,
      [](const sensor_msgs::msg::NavSatFix::SharedPtr) {});
    gps_broadcaster_->update(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
    wait_for(subscription);

    rclcpp::MessageInfo msg_info;
    sensor_msgs::msg::NavSatFix gps_msg;
    subscription->take(gps_msg, msg_info);
    return gps_msg;
  }

  controller_interface::ControllerInterfaceParams create_ctrl_params(
    const rclcpp::NodeOptions & node_options, const std::string & robot_description = "")
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_gps_sensor_broadcaster";
    params.robot_description = robot_description;
    params.update_rate = 0;
    params.node_namespace = "";
    params.node_options = node_options;
    return params;
  void expect_configure_succeeded(
    std::unique_ptr<gps_sensor_broadcaster::GPSSensorBroadcaster> & broadcaster, bool succeeded)
  {
    auto state = broadcaster->configure();

    if (succeeded)
      ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
    else
      ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  }

  void expect_activate_succeeded(
    std::unique_ptr<gps_sensor_broadcaster::GPSSensorBroadcaster> & broadcaster, bool succeeded)
  {
    auto state = broadcaster->get_node()->activate();
    if (succeeded)
      ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());
    else
      ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  }

protected:
  const rclcpp::Parameter sensor_name_param_ = rclcpp::Parameter("sensor_name", "gps_sensor");
  const std::string sensor_name_ = sensor_name_param_.get_value<std::string>();
  const rclcpp::Parameter frame_id_ = rclcpp::Parameter("frame_id", "gps_sensor_frame");
  std::array<double, 8> sensor_values_ = {{1.0, 1.0, 1.1, 2.2, 3.3, 0.5, 0.7, 0.9}};
  hardware_interface::StateInterface::SharedPtr gps_status_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "status", &sensor_values_[0]);
  hardware_interface::StateInterface::SharedPtr gps_service_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "service", &sensor_values_[1]);
  hardware_interface::StateInterface::SharedPtr gps_latitude_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "latitude", &sensor_values_[2]);
  hardware_interface::StateInterface::SharedPtr gps_longitude_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "longitude", &sensor_values_[3]);
  hardware_interface::StateInterface::SharedPtr gps_altitude_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "altitude", &sensor_values_[4]);
  hardware_interface::StateInterface::SharedPtr latitude_covariance_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "latitude_covariance", &sensor_values_[5]);
  hardware_interface::StateInterface::SharedPtr longitude_covariance_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "longitude_covariance", &sensor_values_[6]);
  hardware_interface::StateInterface::SharedPtr altitude_covariance_ =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "altitude_covariance", &sensor_values_[7]);

  std::unique_ptr<gps_sensor_broadcaster::GPSSensorBroadcaster> gps_broadcaster_;
};

TEST_F(GPSSensorBroadcasterTest, whenNoParamsAreSetThenInitShouldFail)
{
  const auto result = gps_broadcaster_->init(create_ctrl_params(
    gps_broadcaster_->define_custom_node_options(), ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::ERROR);
}

TEST_F(GPSSensorBroadcasterTest, whenOnlySensorNameIsSetThenInitShouldFail)
{
  const auto node_options = create_node_options_with_overriden_parameters({sensor_name_param_});
  const auto result = gps_broadcaster_->init(
    create_ctrl_params(node_options, ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::ERROR);
}

TEST_F(
  GPSSensorBroadcasterTest,
  whenAllRequiredArgumentsAreSetThenInitConfigureAndActivationShouldSucceed)
{
  const auto node_options =
    create_node_options_with_overriden_parameters({sensor_name_param_, frame_id_});
  const auto result = gps_broadcaster_->init(
    create_ctrl_params(node_options, ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  expect_configure_succeeded(gps_broadcaster_, true);

  expect_activate_succeeded(gps_broadcaster_, true);
}

TEST_F(
  GPSSensorBroadcasterTest, whenBroadcasterIsActiveShouldPublishNavSatMsgWithCovarianceSetToZero)
{
  const auto node_options =
    create_node_options_with_overriden_parameters({sensor_name_param_, frame_id_});
  const auto result = gps_broadcaster_->init(
    create_ctrl_params(node_options, ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  expect_configure_succeeded(gps_broadcaster_, true);

  setup_gps_broadcaster();

  expect_activate_succeeded(gps_broadcaster_, true);

  const auto gps_msg = subscribe_and_get_message();
  EXPECT_EQ(gps_msg.header.frame_id, frame_id_.get_value<std::string>());
  EXPECT_EQ(gps_msg.status.status, sensor_values_[0]);
  EXPECT_EQ(gps_msg.status.service, sensor_values_[1]);
  EXPECT_EQ(gps_msg.latitude, sensor_values_[2]);
  EXPECT_EQ(gps_msg.longitude, sensor_values_[3]);
  EXPECT_EQ(gps_msg.altitude, sensor_values_[4]);

  const std::array<double, 9> expected_covariance = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ASSERT_THAT(gps_msg.position_covariance, ::testing::ElementsAreArray(expected_covariance));
}

TEST_F(
  GPSSensorBroadcasterTest,
  whenBroadcasterIsActiveAndStaticCovarianceShouldPublishNavSatMsgWithStaticCovariance)
{
  const std::array<double, 9> static_covariance = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
  const auto node_options = create_node_options_with_overriden_parameters(
    {sensor_name_param_,
     frame_id_,
     {"static_position_covariance",
      std::vector<double>{static_covariance.begin(), static_covariance.end()}}});
  const auto result = gps_broadcaster_->init(
    create_ctrl_params(node_options, ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  expect_configure_succeeded(gps_broadcaster_, true);

  setup_gps_broadcaster();

  expect_activate_succeeded(gps_broadcaster_, true);

  const auto gps_msg = subscribe_and_get_message();
  EXPECT_EQ(gps_msg.header.frame_id, frame_id_.get_value<std::string>());
  EXPECT_EQ(gps_msg.status.status, sensor_values_[0]);
  EXPECT_EQ(gps_msg.status.service, sensor_values_[1]);
  EXPECT_EQ(gps_msg.latitude, sensor_values_[2]);
  EXPECT_EQ(gps_msg.longitude, sensor_values_[3]);
  EXPECT_EQ(gps_msg.altitude, sensor_values_[4]);

  ASSERT_THAT(gps_msg.position_covariance, ::testing::ElementsAreArray(static_covariance));
  ASSERT_THAT(gps_msg.position_covariance_type, COVARIANCE_TYPE_KNOWN);
}

TEST_F(
  GPSSensorBroadcasterTest,
  whenBroadcasterReadsCovarianceFormInterfaceThenValusesShouldBroadcastedInMsg)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {sensor_name_param_, frame_id_, {"read_covariance_from_interface", true}});

  const auto result = gps_broadcaster_->init(
    create_ctrl_params(node_options, ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  expect_configure_succeeded(gps_broadcaster_, true);

  setup_gps_broadcaster<semantic_components::GPSSensorOption::WithCovariance>();

  expect_activate_succeeded(gps_broadcaster_, true);

  const auto gps_msg = subscribe_and_get_message();
  EXPECT_EQ(gps_msg.header.frame_id, frame_id_.get_value<std::string>());
  EXPECT_EQ(gps_msg.status.status, sensor_values_[0]);
  EXPECT_EQ(gps_msg.status.service, sensor_values_[1]);
  EXPECT_EQ(gps_msg.latitude, sensor_values_[2]);
  EXPECT_EQ(gps_msg.longitude, sensor_values_[3]);
  EXPECT_EQ(gps_msg.altitude, sensor_values_[4]);

  const std::array<double, 9> expected_covariance = {
    {sensor_values_[5], 0.0, 0.0, 0.0, sensor_values_[6], 0.0, 0.0, 0.0, sensor_values_[7]}};
  ASSERT_THAT(gps_msg.position_covariance, ::testing::ElementsAreArray(expected_covariance));
  ASSERT_THAT(gps_msg.position_covariance_type, COVARIANCE_TYPE_DIAGONAL_KNOWN);
}
