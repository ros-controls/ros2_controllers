// Copyright 2026 ros2_control development team
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
 * Authors: Christian Rauch, Wiktor Bajor, Jakub Delicat
 */

#include <gmock/gmock.h>
#include <utility>

#include <class_loader/class_loader.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <magnetometer_broadcaster/magnetometer_broadcaster_parameters.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/wait_result_kind.hpp>
#include <rclcpp/wait_set.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <ros2_control_test_assets/descriptions.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

using callback_return_type =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{
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

class MagnetometerBroadcasterTest : public ::testing::Test
{
public:
  MagnetometerBroadcasterTest() { rclcpp::init(0, nullptr); }

  ~MagnetometerBroadcasterTest() { rclcpp::shutdown(); }

  void SetUp()
  {
    broadcaster = loader->createInstance<controller_interface::ControllerInterface>(
      "magnetometer_broadcaster::MagnetometerBroadcaster");
  }

  void TearDown() { broadcaster.reset(); }

  void setup_broadcaster()
  {
    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(magnetic_field_x, nullptr);
    state_ifs.emplace_back(magnetic_field_y, nullptr);
    state_ifs.emplace_back(magnetic_field_z, nullptr);

    broadcaster->assign_interfaces({}, std::move(state_ifs));
  }

  sensor_msgs::msg::MagneticField subscribe_and_get_message()
  {
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::MagneticField>(
      "/test_magnetometer_broadcaster/magnetic_field", 10,
      [](const sensor_msgs::msg::MagneticField::SharedPtr) {});
    broadcaster->update(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
    wait_for(subscription);

    rclcpp::MessageInfo msg_info;
    sensor_msgs::msg::MagneticField msg;
    subscription->take(msg, msg_info);
    return msg;
  }

  controller_interface::ControllerInterfaceParams create_ctrl_params(
    const rclcpp::NodeOptions & node_options, const std::string & robot_description = "")
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_magnetometer_broadcaster";
    params.robot_description = robot_description;
    params.update_rate = 0;
    params.node_namespace = "";
    params.node_options = node_options;
    return params;
  }

protected:
  const std::string sensor_name_ = "magnetometer";
  const rclcpp::Parameter sensor_name_param_ = rclcpp::Parameter("sensor_name", sensor_name_);
  const std::string frame_id_ = "magnetometer_frame";
  const rclcpp::Parameter frame_id_param_ = rclcpp::Parameter("frame_id", frame_id_);

  std::array<double, 3> sensor_values_ = {{20e-6, 30e-6, 40e-6}};
  hardware_interface::StateInterface::SharedPtr magnetic_field_x =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "magnetic_field.x", &sensor_values_[0]);
  hardware_interface::StateInterface::SharedPtr magnetic_field_y =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "magnetic_field.y", &sensor_values_[1]);
  hardware_interface::StateInterface::SharedPtr magnetic_field_z =
    std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, "magnetic_field.z", &sensor_values_[2]);

  std::unique_ptr<class_loader::ClassLoader> loader =
    std::make_unique<class_loader::ClassLoader>(std::string{});

  controller_interface::ControllerInterfaceSharedPtr broadcaster = nullptr;
};

TEST_F(MagnetometerBroadcasterTest, whenNoParamsAreSetThenInitShouldFail)
{
  const auto result = broadcaster->init(create_ctrl_params(
    broadcaster->define_custom_node_options(), ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::ERROR);
}

TEST_F(MagnetometerBroadcasterTest, whenOnlySensorNameIsSetThenInitShouldFail)
{
  const auto node_options = create_node_options_with_overriden_parameters({sensor_name_param_});
  const auto result = broadcaster->init(
    create_ctrl_params(node_options, ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::ERROR);
}

TEST_F(
  MagnetometerBroadcasterTest,
  whenAllRequiredArgumentsAreSetThenInitConfigureAndActivationShouldSucceed)
{
  const auto node_options =
    create_node_options_with_overriden_parameters({sensor_name_param_, frame_id_param_});
  const auto result = broadcaster->init(
    create_ctrl_params(node_options, ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::OK);
  ASSERT_EQ(broadcaster->on_configure(rclcpp_lifecycle::State()), callback_return_type::SUCCESS);
  ASSERT_EQ(broadcaster->on_activate(rclcpp_lifecycle::State()), callback_return_type::SUCCESS);
}

TEST_F(MagnetometerBroadcasterTest, whenBroadcasterIsActiveShouldPublishWithCovarianceSetToZero)
{
  const auto node_options =
    create_node_options_with_overriden_parameters({sensor_name_param_, frame_id_param_});
  const auto result = broadcaster->init(
    create_ctrl_params(node_options, ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::OK);
  ASSERT_EQ(broadcaster->on_configure(rclcpp_lifecycle::State()), callback_return_type::SUCCESS);
  setup_broadcaster();
  ASSERT_EQ(broadcaster->on_activate(rclcpp_lifecycle::State()), callback_return_type::SUCCESS);

  const auto msg = subscribe_and_get_message();
  EXPECT_EQ(msg.header.frame_id, frame_id_);
  EXPECT_EQ(msg.magnetic_field.x, sensor_values_[0]);
  EXPECT_EQ(msg.magnetic_field.y, sensor_values_[1]);
  EXPECT_EQ(msg.magnetic_field.z, sensor_values_[2]);

  const std::array<double, 9> expected_covariance = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  ASSERT_THAT(msg.magnetic_field_covariance, ::testing::ElementsAreArray(expected_covariance));
}

TEST_F(
  MagnetometerBroadcasterTest,
  whenBroadcasterIsActiveAndStaticCovarianceShouldPublishWithStaticCovariance)
{
  const std::array<double, 9> static_covariance = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
  const auto node_options = create_node_options_with_overriden_parameters(
    {sensor_name_param_,
     frame_id_param_,
     {"static_covariance",
      std::vector<double>{static_covariance.begin(), static_covariance.end()}}});
  const auto result = broadcaster->init(
    create_ctrl_params(node_options, ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::OK);
  ASSERT_EQ(broadcaster->on_configure(rclcpp_lifecycle::State()), callback_return_type::SUCCESS);
  setup_broadcaster();
  ASSERT_EQ(broadcaster->on_activate(rclcpp_lifecycle::State()), callback_return_type::SUCCESS);

  const auto msg = subscribe_and_get_message();
  EXPECT_EQ(msg.header.frame_id, frame_id_);
  EXPECT_EQ(msg.magnetic_field.x, sensor_values_[0]);
  EXPECT_EQ(msg.magnetic_field.y, sensor_values_[1]);
  EXPECT_EQ(msg.magnetic_field.z, sensor_values_[2]);

  ASSERT_THAT(msg.magnetic_field_covariance, ::testing::ElementsAreArray(static_covariance));
}
