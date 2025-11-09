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

controller_interface::return_type GenericStateBroadcasterTest::SetUpStateBroadcaster(const std::vector<std::string> & interfaces)
{
  RCLCPP_INFO(
    rclcpp::get_logger("GenericStateBroadcasterTest"),
    "Setting up GenericStateBroadcaster with interfaces: %d",
    static_cast<int>(interfaces.size()));
  auto result = init_broadcaster_and_set_parameters("", interfaces);
  if(result == controller_interface::return_type::OK)
  {
  assign_state_interfaces(interfaces);
  }
  return result;
}

controller_interface::return_type GenericStateBroadcasterTest::init_broadcaster_and_set_parameters(
  const std::string & robot_description, const std::vector<std::string> & interfaces)
{
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "joint_state_broadcaster";
  params.robot_description = robot_description;
  params.update_rate = 0;
  params.node_namespace = "";
  params.node_options = state_broadcaster_->define_custom_node_options();
  if(!interfaces.empty())
  {
    params.node_options.parameter_overrides({rclcpp::Parameter("interfaces", rclcpp::ParameterValue(interfaces))});
  }
  return state_broadcaster_->init(params);
}

void GenericStateBroadcasterTest::assign_state_interfaces(const std::vector<std::string> & interfaces)
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

  ASSERT_EQ(state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(state_broadcaster_->values_msg_.values.size(), 3);
  ASSERT_TRUE(std::isnan(state_broadcaster_->values_msg_.values[0]));
  ASSERT_TRUE(std::isnan(state_broadcaster_->values_msg_.values[1]));
  ASSERT_TRUE(std::isnan(state_broadcaster_->values_msg_.values[2]));

  ASSERT_EQ(state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), controller_interface::return_type::OK);
}


// TEST_F(GenericStateBroadcasterTest, TestCustomInterfaceWithoutMapping)
// {
//   const std::vector<std::string> JOINT_NAMES = {joint_names_[0]};
//   const std::vector<std::string> IF_NAMES = {custom_interface_name_};
//   SetUpStateBroadcaster(JOINT_NAMES, IF_NAMES);

//   // configure ok
//   ASSERT_EQ(state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

//   ASSERT_EQ(state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

//   const size_t NUM_JOINTS = JOINT_NAMES.size();

//   // check interface configuration
//   auto cmd_if_conf = state_broadcaster_->command_interface_configuration();
//   ASSERT_THAT(cmd_if_conf.names, IsEmpty());
//   EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
//   auto state_if_conf = state_broadcaster_->state_interface_configuration();
//   ASSERT_THAT(state_if_conf.names, SizeIs(JOINT_NAMES.size() * IF_NAMES.size()));
//   EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

//   // joint state initialized
//   const auto & joint_state_msg = state_broadcaster_->joint_state_msg_;
//   ASSERT_EQ(joint_state_msg.header.frame_id, frame_id_);
//   ASSERT_THAT(joint_state_msg.name, SizeIs(0));
//   ASSERT_THAT(joint_state_msg.position, SizeIs(0));
//   ASSERT_THAT(joint_state_msg.velocity, SizeIs(0));
//   ASSERT_THAT(joint_state_msg.effort, SizeIs(0));

//   // dynamic joint state initialized
//   const auto & dynamic_joint_state_msg = state_broadcaster_->dynamic_joint_state_msg_;
//   ASSERT_EQ(dynamic_joint_state_msg.header.frame_id, frame_id_);
//   ASSERT_THAT(dynamic_joint_state_msg.joint_names, SizeIs(NUM_JOINTS));
//   ASSERT_THAT(dynamic_joint_state_msg.interface_values, SizeIs(NUM_JOINTS));
//   ASSERT_THAT(dynamic_joint_state_msg.joint_names, ElementsAreArray(JOINT_NAMES));
//   ASSERT_THAT(
//     dynamic_joint_state_msg.interface_values[0].interface_names, ElementsAreArray(IF_NAMES));

//   // publishers initialized
//   ASSERT_TRUE(state_broadcaster_->joint_state_publisher_);
//   ASSERT_TRUE(state_broadcaster_->dynamic_joint_state_publisher_);
// }

// TEST_F(GenericStateBroadcasterTest, TestCustomInterfaceMapping)
// {
//   const std::vector<std::string> JOINT_NAMES = {joint_names_[0]};
//   const std::vector<std::string> IF_NAMES = {custom_interface_name_};
//   SetUpStateBroadcaster(JOINT_NAMES, IF_NAMES);

//   state_broadcaster_->get_node()->set_parameter(
//     {std::string("map_interface_to_joint_state.") + HW_IF_POSITION, custom_interface_name_});

//   // configure ok
//   ASSERT_EQ(state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

//   ASSERT_EQ(state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

//   const size_t NUM_JOINTS = JOINT_NAMES.size();

//   // check interface configuration
//   auto cmd_if_conf = state_broadcaster_->command_interface_configuration();
//   ASSERT_THAT(cmd_if_conf.names, IsEmpty());
//   EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
//   auto state_if_conf = state_broadcaster_->state_interface_configuration();
//   ASSERT_THAT(state_if_conf.names, SizeIs(JOINT_NAMES.size() * IF_NAMES.size()));
//   EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

//   // joint state initialized
//   const auto & joint_state_msg = state_broadcaster_->joint_state_msg_;
//   ASSERT_EQ(joint_state_msg.header.frame_id, frame_id_);
//   ASSERT_THAT(joint_state_msg.name, ElementsAreArray(JOINT_NAMES));
//   ASSERT_THAT(joint_state_msg.position, SizeIs(NUM_JOINTS));
//   ASSERT_THAT(joint_state_msg.velocity, SizeIs(NUM_JOINTS));
//   for (auto i = 0ul; i < NUM_JOINTS; ++i)
//   {
//     ASSERT_TRUE(std::isnan(joint_state_msg.velocity[i]));
//   }
//   ASSERT_THAT(joint_state_msg.effort, SizeIs(NUM_JOINTS));
//   for (auto i = 0ul; i < NUM_JOINTS; ++i)
//   {
//     ASSERT_TRUE(std::isnan(joint_state_msg.effort[i]));
//   }

//   // dynamic joint state initialized
//   const auto & dynamic_joint_state_msg = state_broadcaster_->dynamic_joint_state_msg_;
//   ASSERT_EQ(dynamic_joint_state_msg.header.frame_id, frame_id_);
//   ASSERT_THAT(dynamic_joint_state_msg.joint_names, SizeIs(NUM_JOINTS));
//   ASSERT_THAT(dynamic_joint_state_msg.interface_values, SizeIs(NUM_JOINTS));
//   ASSERT_THAT(dynamic_joint_state_msg.joint_names, ElementsAreArray(JOINT_NAMES));
//   ASSERT_THAT(
//     dynamic_joint_state_msg.interface_values[0].interface_names,
//     ElementsAreArray({HW_IF_POSITION}));  // mapping to this value

//   // publishers initialized
//   ASSERT_TRUE(state_broadcaster_->joint_state_publisher_);
//   ASSERT_TRUE(state_broadcaster_->dynamic_joint_state_publisher_);
// }

// TEST_F(GenericStateBroadcasterTest, TestCustomInterfaceMappingUpdate)
// {
//   const std::vector<std::string> JOINT_NAMES = {joint_names_[0]};
//   const std::vector<std::string> IF_NAMES = {custom_interface_name_};
//   SetUpStateBroadcaster(JOINT_NAMES, IF_NAMES);

//   state_broadcaster_->get_node()->set_parameter(
//     {std::string("map_interface_to_joint_state.") + HW_IF_POSITION, custom_interface_name_});

//   sensor_msgs::msg::JointState joint_state_msg;
//   activate_and_get_joint_state_message("joint_states", joint_state_msg);

//   const size_t NUM_JOINTS = JOINT_NAMES.size();

//   // joint state initialized
//   ASSERT_EQ(joint_state_msg.header.frame_id, frame_id_);
//   ASSERT_THAT(joint_state_msg.name, ElementsAreArray(JOINT_NAMES));
//   ASSERT_THAT(joint_state_msg.position, SizeIs(NUM_JOINTS));
//   ASSERT_EQ(joint_state_msg.position[0], custom_joint_value_);
//   ASSERT_THAT(joint_state_msg.velocity, SizeIs(NUM_JOINTS));
//   for (auto i = 0ul; i < NUM_JOINTS; ++i)
//   {
//     ASSERT_TRUE(std::isnan(joint_state_msg.velocity[i]));
//   }
//   ASSERT_THAT(joint_state_msg.effort, SizeIs(NUM_JOINTS));
//   for (auto i = 0ul; i < NUM_JOINTS; ++i)
//   {
//     ASSERT_TRUE(std::isnan(joint_state_msg.effort[i]));
//   }

//   // dynamic joint state initialized
//   const auto & dynamic_joint_state_msg = state_broadcaster_->dynamic_joint_state_msg_;
//   ASSERT_EQ(dynamic_joint_state_msg.header.frame_id, frame_id_);
//   ASSERT_THAT(dynamic_joint_state_msg.joint_names, SizeIs(NUM_JOINTS));
//   ASSERT_THAT(dynamic_joint_state_msg.interface_values, SizeIs(NUM_JOINTS));
//   ASSERT_THAT(dynamic_joint_state_msg.joint_names, ElementsAreArray(JOINT_NAMES));
//   ASSERT_THAT(
//     dynamic_joint_state_msg.interface_values[0].interface_names,
//     ElementsAreArray({HW_IF_POSITION}));

//   // publishers initialized
//   ASSERT_TRUE(state_broadcaster_->joint_state_publisher_);
//   ASSERT_TRUE(state_broadcaster_->dynamic_joint_state_publisher_);
// }

// TEST_F(GenericStateBroadcasterTest, UpdateTest)
// {
//   SetUpStateBroadcaster();

//   auto node_state = state_broadcaster_->configure();
//   node_state = state_broadcaster_->get_node()->activate();
//   ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
//   ASSERT_EQ(
//     state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
//     controller_interface::return_type::OK);
// }

// TEST_F(GenericStateBroadcasterTest, UpdatePerformanceTest)
// {
//   controller_interface::ControllerInterfaceParams params;
//   params.controller_name = "joint_state_broadcaster";
//   params.robot_description = "";
//   params.update_rate = 0;
//   params.node_namespace = "";
//   params.node_options = state_broadcaster_->define_custom_node_options();
//   const auto result = state_broadcaster_->init(params);
//   ASSERT_EQ(result, controller_interface::return_type::OK);

//   custom_joint_value_ = 12.34;

//   // build our own test interfaces: robot has ~500 state interfaces
//   for (auto joint = 1u; joint < 30; ++joint)
//   {
//     const auto joint_name = "joint_" + std::to_string(joint);

//     // standard
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "position", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "velocity", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "effort", &custom_joint_value_));

//     // non standard
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "mode", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "absolute_position", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "acceleration", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "current", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "torque", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "force", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "temperature_board", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "temperature_motor", &custom_joint_value_));

//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "position.kd", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "position.ki", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "position.kp", &custom_joint_value_));

//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "velocity.kd", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "velocity.ki", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "velocity.kp", &custom_joint_value_));

//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "current.kd", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "current.ki", &custom_joint_value_));
//     test_interfaces_.emplace_back(
//       std::make_shared<hardware_interface::StateInterface>(
//         joint_name, "current.kp", &custom_joint_value_));
//   }

//   RCLCPP_INFO(
//     state_broadcaster_->get_node()->get_logger(), "Number of test interfaces: %zu",
//     test_interfaces_.size());

//   std::vector<LoanedStateInterface> state_interfaces;
//   for (const auto & tif : test_interfaces_)
//   {
//     state_interfaces.emplace_back(tif, nullptr);
//   }

//   state_broadcaster_->assign_interfaces({}, std::move(state_interfaces));

//   auto node_state = state_broadcaster_->configure();
//   node_state = state_broadcaster_->get_node()->activate();
//   ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

//   if (!realtime_tools::configure_sched_fifo(50))
//   {
//     RCLCPP_WARN(
//       state_broadcaster_->get_node()->get_logger(),
//       "Could not enable FIFO RT scheduling policy: with error number <%i>(%s)", errno,
//       strerror(errno));
//   }

//   constexpr auto kNumSamples = 10000u;
//   std::vector<int64_t> measures;
//   for (auto i = 0u; i < kNumSamples; ++i)
//   {
//     const auto now = std::chrono::steady_clock::now();

//     ASSERT_EQ(
//       state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
//       controller_interface::return_type::OK);

//     // print time taken
//     const auto elapsed = std::chrono::steady_clock::now() - now;
//     const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed);

//     measures.push_back(elapsed_us.count());
//   }

//   const auto average =
//     std::accumulate(measures.begin(), measures.end(), 0.0) / static_cast<double>(measures.size());
//   const auto variance = std::accumulate(
//                           measures.begin(), measures.end(), 0.0, [average](double accum, double x)
//                           { return accum + (x - average) * (x - average); }) /
//                         static_cast<double>(measures.size());

//   RCLCPP_INFO(state_broadcaster_->get_node()->get_logger(), "Average update time: %lf us", average);
//   RCLCPP_INFO(state_broadcaster_->get_node()->get_logger(), "Variance: %lf us", variance);
// }

// void GenericStateBroadcasterTest::activate_and_get_joint_state_message(
//   const std::string & topic, sensor_msgs::msg::JointState & joint_state_msg)
// {
//   auto node_state = state_broadcaster_->configure();
//   ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

//   node_state = state_broadcaster_->get_node()->activate();
//   ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

//   sensor_msgs::msg::JointState::SharedPtr received_msg;
//   rclcpp::Node test_node("test_node");
//   auto subs_callback = [&](const sensor_msgs::msg::JointState::SharedPtr msg)
//   { received_msg = msg; };
//   auto subscription =
//     test_node.create_subscription<sensor_msgs::msg::JointState>(topic, 10, subs_callback);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(test_node.get_node_base_interface());

//   // call update to publish the test value
//   // since update doesn't guarantee a published message, republish until received
//   int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
//   while (max_sub_check_loop_count--)
//   {
//     state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     const auto timeout = std::chrono::milliseconds{5};
//     const auto until = test_node.get_clock()->now() + timeout;
//     while (!received_msg && test_node.get_clock()->now() < until)
//     {
//       executor.spin_some();
//       std::this_thread::sleep_for(std::chrono::microseconds(10));
//     }
//     // check if message has been received
//     if (received_msg.get())
//     {
//       break;
//     }
//   }
//   ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
//                                             "controller/broadcaster update loop";
//   ASSERT_TRUE(received_msg);

//   // take message from subscription
//   joint_state_msg = *received_msg;
// }

// void GenericStateBroadcasterTest::test_published_joint_state_message(const std::string & topic)
// {
//   sensor_msgs::msg::JointState joint_state_msg;
//   activate_and_get_joint_state_message(topic, joint_state_msg);

//   const size_t NUM_JOINTS = joint_names_.size();
//   ASSERT_EQ(joint_state_msg.header.frame_id, frame_id_);
//   ASSERT_THAT(joint_state_msg.name, SizeIs(NUM_JOINTS));
//   // the order in the message may be different
//   // we only check that all values in this test are present in the message
//   // and that it is the same across the interfaces
//   // for test purposes they are mapped to the same doubles
//   ASSERT_THAT(joint_state_msg.position, ElementsAreArray(joint_values_));
//   ASSERT_THAT(joint_state_msg.velocity, ElementsAreArray(joint_state_msg.position));
//   ASSERT_THAT(joint_state_msg.effort, ElementsAreArray(joint_state_msg.position));
// }

// TEST_F(GenericStateBroadcasterTest, JointStatePublishTest)
// {
//   SetUpStateBroadcaster();

//   test_published_joint_state_message("joint_states");
// }

// TEST_F(GenericStateBroadcasterTest, JointStatePublishTestLocalTopic)
// {
//   SetUpStateBroadcaster();
//   state_broadcaster_->get_node()->set_parameter({"use_local_topics", true});

//   test_published_joint_state_message("joint_state_broadcaster/joint_states");
// }

// void GenericStateBroadcasterTest::test_published_dynamic_joint_state_message(
//   const std::string & topic)
// {
//   auto node_state = state_broadcaster_->configure();
//   ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

//   node_state = state_broadcaster_->get_node()->activate();
//   ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

//   control_msgs::msg::DynamicJointState::SharedPtr dynamic_joint_state_msg;
//   rclcpp::Node test_node("test_node");
//   auto subs_callback = [&](const control_msgs::msg::DynamicJointState::SharedPtr msg)
//   { dynamic_joint_state_msg = msg; };
//   auto subscription =
//     test_node.create_subscription<control_msgs::msg::DynamicJointState>(topic, 10, subs_callback);
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(test_node.get_node_base_interface());
//   dynamic_joint_state_msg.reset();
//   ASSERT_FALSE(dynamic_joint_state_msg);

//   // call update to publish the test value
//   // since update doesn't guarantee a published message, republish until received
//   int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
//   while (max_sub_check_loop_count--)
//   {
//     state_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
//     const auto timeout = std::chrono::milliseconds{5};
//     const auto until = test_node.get_clock()->now() + timeout;
//     while (test_node.get_clock()->now() < until)
//     {
//       executor.spin_some();
//       std::this_thread::sleep_for(std::chrono::microseconds(10));
//     }
//     // check if message has been received
//     if (dynamic_joint_state_msg.get())
//     {
//       break;
//     }
//   }
//   ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
//                                             "controller/broadcaster update loop";
//   ASSERT_TRUE(dynamic_joint_state_msg);

//   const size_t NUM_JOINTS = 3;
//   const std::vector<std::string> INTERFACE_NAMES = {HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_EFFORT};
//   ASSERT_EQ(dynamic_joint_state_msg->header.frame_id, frame_id_);
//   ASSERT_THAT(dynamic_joint_state_msg->joint_names, SizeIs(NUM_JOINTS));
//   // the order in the message may be different
//   // we only check that all values in this test are present in the message
//   // and that it is the same across the interfaces
//   // for test purposes they are mapped to the same doubles
//   for (size_t i = 0; i < dynamic_joint_state_msg->joint_names.size(); ++i)
//   {
//     ASSERT_THAT(
//       dynamic_joint_state_msg->interface_values[i].interface_names,
//       ElementsAreArray(INTERFACE_NAMES));
//     const auto index_in_original_order = static_cast<size_t>(std::distance(
//       joint_names_.cbegin(),
//       std::find(
//         joint_names_.cbegin(), joint_names_.cend(), dynamic_joint_state_msg->joint_names[i])));
//     ASSERT_THAT(
//       dynamic_joint_state_msg->interface_values[i].values,
//       Each(joint_values_[index_in_original_order]));
//   }
// }

// TEST_F(GenericStateBroadcasterTest, DynamicJointStatePublishTest)
// {
//   SetUpStateBroadcaster();

//   test_published_dynamic_joint_state_message("dynamic_joint_states");
// }

// TEST_F(GenericStateBroadcasterTest, DynamicJointStatePublishTestLocalTopic)
// {
//   SetUpStateBroadcaster();
//   state_broadcaster_->get_node()->set_parameter({"use_local_topics", true});

//   test_published_dynamic_joint_state_message("joint_state_broadcaster/dynamic_joint_states");
// }

// TEST_F(GenericStateBroadcasterTest, ExtraJointStatePublishTest)
// {
//   // publishers not initialized yet
//   ASSERT_FALSE(state_broadcaster_->realtime_joint_state_publisher_);
//   ASSERT_FALSE(state_broadcaster_->realtime_dynamic_joint_state_publisher_);

//   SetUpStateBroadcaster();

//   // Add extra joints as parameters
//   const std::vector<std::string> extra_joint_names = {"extra1", "extra2", "extra3"};
//   state_broadcaster_->get_node()->set_parameter({"extra_joints", extra_joint_names});

//   // configure ok
//   ASSERT_EQ(state_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

//   ASSERT_EQ(state_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

//   std::vector<std::string> all_joint_names = joint_names_;
//   all_joint_names.insert(all_joint_names.end(), extra_joint_names.begin(), extra_joint_names.end());
//   const size_t NUM_JOINTS = all_joint_names.size();

//   // joint state initialized
//   const auto & joint_state_msg = state_broadcaster_->joint_state_msg_;
//   ASSERT_EQ(joint_state_msg.header.frame_id, frame_id_);
//   ASSERT_THAT(joint_state_msg.name, ElementsAreArray(all_joint_names));
//   ASSERT_THAT(joint_state_msg.position, SizeIs(NUM_JOINTS));
//   ASSERT_THAT(joint_state_msg.velocity, SizeIs(NUM_JOINTS));
//   ASSERT_THAT(joint_state_msg.effort, SizeIs(NUM_JOINTS));

//   // dynamic joint state initialized
//   const auto & dynamic_joint_state_msg = state_broadcaster_->dynamic_joint_state_msg_;
//   ASSERT_EQ(dynamic_joint_state_msg.header.frame_id, frame_id_);
//   ASSERT_THAT(dynamic_joint_state_msg.joint_names, SizeIs(NUM_JOINTS));
// }
