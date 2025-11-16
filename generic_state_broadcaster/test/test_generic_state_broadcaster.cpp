// Copyright (c) 2025, PAL Robotics
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

#include <cstddef>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "test_generic_state_broadcaster.hpp"

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedStateInterface;
using std::placeholders::_1;
using testing::Each;
using testing::ElementsAreArray;
using testing::IsEmpty;
using testing::SizeIs;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace

void GenericStateBroadcasterTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void GenericStateBroadcasterTest::TearDownTestCase() { rclcpp::shutdown(); }

void GenericStateBroadcasterTest::SetUp()
{
  // initialize broadcaster
  state_broadcaster_ = std::make_unique<FriendGenericStateBroadcaster>();
}

void GenericStateBroadcasterTest::TearDown() { state_broadcaster_.reset(nullptr); }

controller_interface::return_type GenericStateBroadcasterTest::SetUpStateBroadcaster(
  const std::vector<std::string> & interfaces)
{
  RCLCPP_INFO(
    rclcpp::get_logger("GenericStateBroadcasterTest"),
    "Setting up GenericStateBroadcaster with interfaces: %d", static_cast<int>(interfaces.size()));
  auto result = init_broadcaster_and_set_parameters("", interfaces);
  if (result == controller_interface::return_type::OK)
  {
    assign_state_interfaces(interfaces);
  }
  return result;
}

controller_interface::return_type GenericStateBroadcasterTest::init_broadcaster_and_set_parameters(
  const std::string & robot_description, const std::vector<std::string> & interfaces)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "generic_state_broadcaster";
  params.robot_description = robot_description;
  params.update_rate = 0;
  params.node_namespace = "";
  params.node_options = state_broadcaster_->define_custom_node_options();
  if (!interfaces.empty())
  {
    params.node_options.parameter_overrides(
      {rclcpp::Parameter("interfaces", rclcpp::ParameterValue(interfaces))});
  }
  return state_broadcaster_->init(params);
}

void GenericStateBroadcasterTest::assign_state_interfaces(
  const std::vector<std::string> & interfaces)
{
  std::vector<LoanedStateInterface> state_ifs;
  if (interfaces.empty())
  {
    state_ifs.emplace_back(joint_1_pos_state_);
    state_ifs.emplace_back(joint_2_pos_state_);
    state_ifs.emplace_back(joint_3_pos_state_);
    state_ifs.emplace_back(joint_1_vel_state_);
    state_ifs.emplace_back(joint_2_vel_state_);
    state_ifs.emplace_back(joint_3_vel_state_);
    state_ifs.emplace_back(joint_1_eff_state_);
    state_ifs.emplace_back(joint_2_eff_state_);
    state_ifs.emplace_back(joint_3_eff_state_);
  }
  else
  {
    for (const auto & interface : interfaces)
    {
      RCLCPP_INFO(
        state_broadcaster_->get_node()->get_logger(), "Assigning interface: %s", interface.c_str());
      if (interface == joint_names_[0] + "/" + interface_names_[0])
      {
        state_ifs.emplace_back(joint_1_pos_state_);
      }
      if (interface == joint_names_[1] + "/" + interface_names_[0])
      {
        state_ifs.emplace_back(joint_2_pos_state_);
      }
      if (interface == joint_names_[2] + "/" + interface_names_[0])
      {
        state_ifs.emplace_back(joint_3_pos_state_);
      }
      if (interface == joint_names_[0] + "/" + interface_names_[1])
      {
        state_ifs.emplace_back(joint_1_vel_state_);
      }
      if (interface == joint_names_[1] + "/" + interface_names_[1])
      {
        state_ifs.emplace_back(joint_2_vel_state_);
      }
      if (interface == joint_names_[2] + "/" + interface_names_[1])
      {
        state_ifs.emplace_back(joint_3_vel_state_);
      }
      if (interface == joint_names_[0] + "/" + interface_names_[2])
      {
        state_ifs.emplace_back(joint_1_eff_state_);
      }
      if (interface == joint_names_[1] + "/" + interface_names_[2])
      {
        state_ifs.emplace_back(joint_2_eff_state_);
      }
      if (interface == joint_names_[2] + "/" + interface_names_[2])
      {
        state_ifs.emplace_back(joint_3_eff_state_);
      }
      if (interface == custom_interface_name_)
      {
        state_ifs.emplace_back(joint_X_custom_state);
      }
    }
  }

  state_broadcaster_->assign_interfaces({}, std::move(state_ifs));
}

void GenericStateBroadcasterTest::activate_and_get_state_message(
  const std::string & topic, control_msgs::msg::InterfacesValues & msg)
{
  auto node_state = state_broadcaster_->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  node_state = state_broadcaster_->get_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  control_msgs::msg::InterfacesValues::SharedPtr received_msg;
  rclcpp::Node test_node("test_node");
  auto subs_callback = [&](const control_msgs::msg::InterfacesValues::SharedPtr cb_msg)
  { received_msg = cb_msg; };
  auto subscription =
    test_node.create_subscription<control_msgs::msg::InterfacesValues>(topic, 10, subs_callback);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_node.get_node_base_interface());

  // call update to publish the test value
  // since update doesn't guarantee a published message, republish until received
  int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
  while (max_sub_check_loop_count--)
  {
    state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
    const auto timeout = std::chrono::milliseconds{5};
    const auto until = test_node.get_clock()->now() + timeout;
    while (!received_msg && test_node.get_clock()->now() < until)
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
  msg = *received_msg;
}

TEST_F(GenericStateBroadcasterTest, FailOnEmptyInterfaceListTest)
{
  // publishers not initialized yet
  ASSERT_FALSE(state_broadcaster_->names_publisher_);
  ASSERT_FALSE(state_broadcaster_->values_publisher_);

  ASSERT_EQ(SetUpStateBroadcaster({}), controller_interface::return_type::ERROR);
}

TEST_F(GenericStateBroadcasterTest, ConfigureOnValidInterfaceListTest)
{
  // publishers not initialized yet
  ASSERT_FALSE(state_broadcaster_->names_publisher_);
  ASSERT_FALSE(state_broadcaster_->values_publisher_);

  const std::vector<std::string> interfaces = {
    joint_names_[0] + "/" + interface_names_[0],
    joint_names_[1] + "/" + interface_names_[1],
    joint_names_[2] + "/" + interface_names_[2],
  };
  ASSERT_EQ(SetUpStateBroadcaster(interfaces), controller_interface::return_type::OK);
  // configure ok
  ASSERT_EQ(state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(state_broadcaster_->names_publisher_);
  ASSERT_TRUE(state_broadcaster_->values_publisher_);

  ASSERT_EQ(state_broadcaster_->values_msg_.values.size(), 3);
  ASSERT_TRUE(std::isnan(state_broadcaster_->values_msg_.values[0]));
  ASSERT_TRUE(std::isnan(state_broadcaster_->values_msg_.values[1]));
  ASSERT_TRUE(std::isnan(state_broadcaster_->values_msg_.values[2]));
  ASSERT_THAT(state_broadcaster_->names_msg_.names, ElementsAreArray(interfaces));

  ASSERT_EQ(state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(state_broadcaster_->values_msg_.values.size(), 3);
  ASSERT_TRUE(std::isnan(state_broadcaster_->values_msg_.values[0]));
  ASSERT_TRUE(std::isnan(state_broadcaster_->values_msg_.values[1]));
  ASSERT_TRUE(std::isnan(state_broadcaster_->values_msg_.values[2]));
  ASSERT_THAT(state_broadcaster_->names_msg_.names, ElementsAreArray(interfaces));

  ASSERT_EQ(
    state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  ASSERT_EQ(state_broadcaster_->values_msg_.values.size(), 3);
  ASSERT_DOUBLE_EQ(state_broadcaster_->values_msg_.values[0], joint_values_[0]);
  ASSERT_DOUBLE_EQ(state_broadcaster_->values_msg_.values[1], joint_values_[1]);
  ASSERT_DOUBLE_EQ(state_broadcaster_->values_msg_.values[2], joint_values_[2]);
  ASSERT_THAT(state_broadcaster_->names_msg_.names, ElementsAreArray(interfaces));
}

TEST_F(GenericStateBroadcasterTest, StatePublishTest)
{
  std::vector<std::string> all_interfaces;
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_name : interface_names_)
    {
      all_interfaces.push_back(joint_name + "/" + interface_name);
    }
  }
  SetUpStateBroadcaster(all_interfaces);

  control_msgs::msg::InterfacesValues values_msg;
  activate_and_get_state_message("generic_state_broadcaster/values", values_msg);

  ASSERT_THAT(values_msg.values, SizeIs(9));
  // all values are NaN since we did not set any value to the state interfaces
  const std::vector<double> expected_values = {
    joint_values_[0], joint_values_[0], joint_values_[0], joint_values_[1], joint_values_[1],
    joint_values_[1], joint_values_[2], joint_values_[2], joint_values_[2],
  };
  ASSERT_THAT(values_msg.values, ElementsAreArray(expected_values));
}
