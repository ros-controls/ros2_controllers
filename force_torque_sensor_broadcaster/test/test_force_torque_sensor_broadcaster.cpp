// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
 * Authors: Subhas Das, Denis Stogl
 */

#include "test_force_torque_sensor_broadcaster.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

using hardware_interface::LoanedStateInterface;
using testing::IsEmpty;
using testing::SizeIs;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace

void ForceTorqueSensorBroadcasterTest::SetUpTestCase() {}

void ForceTorqueSensorBroadcasterTest::TearDownTestCase() {}

void ForceTorqueSensorBroadcasterTest::SetUp()
{
  // initialize controller
  fts_broadcaster_ = std::make_unique<FriendForceTorqueSensorBroadcaster>();
}

void ForceTorqueSensorBroadcasterTest::TearDown() { fts_broadcaster_.reset(nullptr); }

void ForceTorqueSensorBroadcasterTest::SetUpFTSBroadcaster(std::string node_name)
{
  const auto result =
    fts_broadcaster_->init(node_name, "", 0, "", fts_broadcaster_->define_custom_node_options());
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedStateInterface> state_ifs;
  state_ifs.emplace_back(fts_force_x_);
  state_ifs.emplace_back(fts_force_y_);
  state_ifs.emplace_back(fts_force_z_);
  state_ifs.emplace_back(fts_torque_x_);
  state_ifs.emplace_back(fts_torque_y_);
  state_ifs.emplace_back(fts_torque_z_);

  fts_broadcaster_->assign_interfaces({}, std::move(state_ifs));
}

void ForceTorqueSensorBroadcasterTest::subscribe_and_get_message(
  geometry_msgs::msg::WrenchStamped & wrench_msg, std::string & topic_name)
{
  // create a new subscriber
  geometry_msgs::msg::WrenchStamped::SharedPtr received_msg;
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  { received_msg = msg; };
  auto subscription = test_subscription_node.create_subscription<geometry_msgs::msg::WrenchStamped>(
    topic_name, 10, subs_callback);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(test_subscription_node.get_node_base_interface());

  // call update to publish the test value
  // since update doesn't guarantee a published message, republish until received
  int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
  while (max_sub_check_loop_count--)
  {
    fts_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
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
  wrench_msg = *received_msg;
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_InterfaceNames_NotSet)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // configure failed
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_InterfaceNames_Set)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set the 'sensor_name'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});

  // set the 'interface_names'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fts_sensor/force.x"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "fts_sensor/torque.z"});

  // configure failed, both 'sensor_name' and 'interface_names' supplied
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_IsEmpty_InterfaceNames_NotSet)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set the 'sensor_name' empty
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", ""});

  // configure failed, 'sensor_name' parameter empty
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_IsEmpty_SensorName_NotSet)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set the 'interface_names' empty
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", ""});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", ""});

  // configure failed, 'interface_name' parameter empty
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_Configure_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set the 'sensor_name'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});

  // set the 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure passed
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check interface configuration
  auto cmd_if_conf = fts_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = fts_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(6lu));
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_Configure_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set the 'interface_names'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fts_sensor/force.x"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "fts_sensor/torque.z"});

  RCLCPP_INFO(fts_broadcaster_->get_node()->get_logger(), "Setting up frame_id");
  // set the 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  RCLCPP_INFO(fts_broadcaster_->get_node()->get_logger(), "Calling on_configure");
  // configure passed
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_ActivateDeactivate_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set the params 'sensor_name' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure and activate success
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check interface configuration
  auto cmd_if_conf = fts_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = fts_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(6lu));
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  // deactivate passed
  ASSERT_EQ(fts_broadcaster_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check interface configuration
  cmd_if_conf = fts_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  state_if_conf = fts_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(6lu));  // did not change
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_Update_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set the params 'sensor_name' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    fts_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set the params 'interface_names' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fts_sensor/force.x"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "fts_sensor/torque.z"});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(
    fts_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_Publish_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set the params 'sensor_name' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  geometry_msgs::msg::WrenchStamped wrench_msg;
  std::string topic_name = "/test_force_torque_sensor_broadcaster/wrench";
  subscribe_and_get_message(wrench_msg, topic_name);

  ASSERT_EQ(wrench_msg.header.frame_id, frame_id_);
  ASSERT_EQ(wrench_msg.wrench.force.x, sensor_values_[0]);
  ASSERT_EQ(wrench_msg.wrench.force.y, sensor_values_[1]);
  ASSERT_EQ(wrench_msg.wrench.force.z, sensor_values_[2]);
  ASSERT_EQ(wrench_msg.wrench.torque.x, sensor_values_[3]);
  ASSERT_EQ(wrench_msg.wrench.torque.y, sensor_values_[4]);
  ASSERT_EQ(wrench_msg.wrench.torque.z, sensor_values_[5]);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_Publish_Success_with_Offsets)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  std::array<double, 3> force_offsets = {{10.0, 30.0, -50.0}};
  std::array<double, 3> torque_offsets = {{1.0, -1.2, -5.2}};
  // set the params 'sensor_name' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});
  fts_broadcaster_->get_node()->set_parameter({"offset.force.x", force_offsets[0]});
  fts_broadcaster_->get_node()->set_parameter({"offset.force.y", force_offsets[1]});
  fts_broadcaster_->get_node()->set_parameter({"offset.force.z", force_offsets[2]});
  fts_broadcaster_->get_node()->set_parameter({"offset.torque.x", torque_offsets[0]});
  fts_broadcaster_->get_node()->set_parameter({"offset.torque.y", torque_offsets[1]});
  fts_broadcaster_->get_node()->set_parameter({"offset.torque.z", torque_offsets[2]});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  geometry_msgs::msg::WrenchStamped wrench_msg;
  std::string topic_name = "/test_force_torque_sensor_broadcaster/wrench";
  subscribe_and_get_message(wrench_msg, topic_name);

  ASSERT_EQ(wrench_msg.header.frame_id, frame_id_);
  ASSERT_EQ(wrench_msg.wrench.force.x, sensor_values_[0] + force_offsets[0]);
  ASSERT_EQ(wrench_msg.wrench.force.y, sensor_values_[1] + force_offsets[1]);
  ASSERT_EQ(wrench_msg.wrench.force.z, sensor_values_[2] + force_offsets[2]);
  ASSERT_EQ(wrench_msg.wrench.torque.x, sensor_values_[3] + torque_offsets[0]);
  ASSERT_EQ(wrench_msg.wrench.torque.y, sensor_values_[4] + torque_offsets[1]);
  ASSERT_EQ(wrench_msg.wrench.torque.z, sensor_values_[5] + torque_offsets[2]);

  // Check the exported state interfaces
  const auto exported_state_interfaces = fts_broadcaster_->export_state_interfaces();
  ASSERT_EQ(exported_state_interfaces.size(), 6u);
  const std::string controller_name = fts_broadcaster_->get_node()->get_name();
  ASSERT_EQ(
    exported_state_interfaces[0]->get_name(), controller_name + "/" + sensor_name_ + "/force.x");
  ASSERT_EQ(
    exported_state_interfaces[1]->get_name(), controller_name + "/" + sensor_name_ + "/force.y");
  ASSERT_EQ(
    exported_state_interfaces[2]->get_name(), controller_name + "/" + sensor_name_ + "/force.z");
  ASSERT_EQ(
    exported_state_interfaces[3]->get_name(), controller_name + "/" + sensor_name_ + "/torque.x");
  ASSERT_EQ(
    exported_state_interfaces[4]->get_name(), controller_name + "/" + sensor_name_ + "/torque.y");
  ASSERT_EQ(
    exported_state_interfaces[5]->get_name(), controller_name + "/" + sensor_name_ + "/torque.z");
  ASSERT_EQ(exported_state_interfaces[0]->get_interface_name(), "force.x");
  ASSERT_EQ(exported_state_interfaces[1]->get_interface_name(), "force.y");
  ASSERT_EQ(exported_state_interfaces[2]->get_interface_name(), "force.z");
  ASSERT_EQ(exported_state_interfaces[3]->get_interface_name(), "torque.x");
  ASSERT_EQ(exported_state_interfaces[4]->get_interface_name(), "torque.y");
  ASSERT_EQ(exported_state_interfaces[5]->get_interface_name(), "torque.z");
  for (size_t i = 0; i < 6; ++i)
  {
    ASSERT_EQ(
      exported_state_interfaces[i]->get_prefix_name(), controller_name + "/" + sensor_name_);
    ASSERT_EQ(
      exported_state_interfaces[i]->get_value(),
      sensor_values_[i] + (i < 3 ? force_offsets[i] : torque_offsets[i - 3]));
  }
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_Publish_Success_with_Multipliers)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // some non‐trivial multipliers
  std::array<double, 3> force_multipliers = {{2.0, 0.5, -1.0}};
  std::array<double, 3> torque_multipliers = {{-2.0, 3.0, 0.0}};

  // Set the required params
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // Set all multiplier parameters
  fts_broadcaster_->get_node()->set_parameter({"multiplier.force.x", force_multipliers[0]});
  fts_broadcaster_->get_node()->set_parameter({"multiplier.force.y", force_multipliers[1]});
  fts_broadcaster_->get_node()->set_parameter({"multiplier.force.z", force_multipliers[2]});
  fts_broadcaster_->get_node()->set_parameter({"multiplier.torque.x", torque_multipliers[0]});
  fts_broadcaster_->get_node()->set_parameter({"multiplier.torque.y", torque_multipliers[1]});
  fts_broadcaster_->get_node()->set_parameter({"multiplier.torque.z", torque_multipliers[2]});

  // Configure & activate
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // Publish & grab the message
  geometry_msgs::msg::WrenchStamped wrench_msg;
  std::string topic_name = "/test_force_torque_sensor_broadcaster/wrench";
  subscribe_and_get_message(wrench_msg, topic_name);

  // Check header
  ASSERT_EQ(wrench_msg.header.frame_id, frame_id_);

  // Check that each field was scaled accordingly
  ASSERT_EQ(wrench_msg.wrench.force.x, sensor_values_[0] * force_multipliers[0]);
  ASSERT_EQ(wrench_msg.wrench.force.y, sensor_values_[1] * force_multipliers[1]);
  ASSERT_EQ(wrench_msg.wrench.force.z, sensor_values_[2] * force_multipliers[2]);
  ASSERT_EQ(wrench_msg.wrench.torque.x, sensor_values_[3] * torque_multipliers[0]);
  ASSERT_EQ(wrench_msg.wrench.torque.y, sensor_values_[4] * torque_multipliers[1]);
  ASSERT_EQ(wrench_msg.wrench.torque.z, sensor_values_[5] * torque_multipliers[2]);

  // the exported state interfaces reflect the same scaled values
  const auto exported_state_interfaces = fts_broadcaster_->export_state_interfaces();
  ASSERT_EQ(exported_state_interfaces.size(), 6u);

  const std::string controller_name = fts_broadcaster_->get_node()->get_name();
  ASSERT_EQ(
    exported_state_interfaces[0]->get_name(), controller_name + "/" + sensor_name_ + "/force.x");
  ASSERT_EQ(
    exported_state_interfaces[1]->get_name(), controller_name + "/" + sensor_name_ + "/force.y");
  ASSERT_EQ(
    exported_state_interfaces[2]->get_name(), controller_name + "/" + sensor_name_ + "/force.z");
  ASSERT_EQ(
    exported_state_interfaces[3]->get_name(), controller_name + "/" + sensor_name_ + "/torque.x");
  ASSERT_EQ(
    exported_state_interfaces[4]->get_name(), controller_name + "/" + sensor_name_ + "/torque.y");
  ASSERT_EQ(
    exported_state_interfaces[5]->get_name(), controller_name + "/" + sensor_name_ + "/torque.z");
  ASSERT_EQ(exported_state_interfaces[0]->get_interface_name(), "force.x");
  ASSERT_EQ(exported_state_interfaces[1]->get_interface_name(), "force.y");
  ASSERT_EQ(exported_state_interfaces[2]->get_interface_name(), "force.z");
  ASSERT_EQ(exported_state_interfaces[3]->get_interface_name(), "torque.x");
  ASSERT_EQ(exported_state_interfaces[4]->get_interface_name(), "torque.y");
  ASSERT_EQ(exported_state_interfaces[5]->get_interface_name(), "torque.z");
  for (size_t i = 0; i < 6; ++i)
  {
    ASSERT_EQ(
      exported_state_interfaces[i]->get_value(),
      sensor_values_[i] * (i < 3 ? force_multipliers[i] : torque_multipliers[i - 3]));
  }
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_Publish_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set the params 'interface_names' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fts_sensor/force.x"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "fts_sensor/torque.z"});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  geometry_msgs::msg::WrenchStamped wrench_msg;
  std::string topic_name = "/test_force_torque_sensor_broadcaster/wrench";
  subscribe_and_get_message(wrench_msg, topic_name);

  ASSERT_EQ(wrench_msg.header.frame_id, frame_id_);
  ASSERT_EQ(wrench_msg.wrench.force.x, sensor_values_[0]);
  ASSERT_TRUE(std::isnan(wrench_msg.wrench.force.y));
  ASSERT_TRUE(std::isnan(wrench_msg.wrench.force.z));
  ASSERT_TRUE(std::isnan(wrench_msg.wrench.torque.x));
  ASSERT_TRUE(std::isnan(wrench_msg.wrench.torque.y));
  ASSERT_EQ(wrench_msg.wrench.torque.z, sensor_values_[5]);

  // Check the exported state interfaces
  const auto exported_state_interfaces = fts_broadcaster_->export_state_interfaces();
  ASSERT_EQ(exported_state_interfaces.size(), 2u);
  const std::string controller_name = fts_broadcaster_->get_node()->get_name();
  ASSERT_EQ(exported_state_interfaces[0]->get_name(), controller_name + "/fts_sensor/force.x");
  ASSERT_EQ(exported_state_interfaces[1]->get_name(), controller_name + "/fts_sensor/torque.z");
  ASSERT_EQ(exported_state_interfaces[0]->get_prefix_name(), controller_name);
  ASSERT_EQ(exported_state_interfaces[1]->get_prefix_name(), controller_name);
  ASSERT_EQ(exported_state_interfaces[0]->get_interface_name(), "fts_sensor/force.x");
  ASSERT_EQ(exported_state_interfaces[1]->get_interface_name(), "fts_sensor/torque.z");
  ASSERT_EQ(exported_state_interfaces[0]->get_value(), sensor_values_[0]);
  ASSERT_EQ(exported_state_interfaces[1]->get_value(), sensor_values_[5]);
}

TEST_F(ForceTorqueSensorBroadcasterTest, All_InterfaceNames_Publish_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster");

  // set all the params 'interface_names' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fts_sensor/force.x"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.y", "fts_sensor/force.y"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.z", "fts_sensor/force.z"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.x", "fts_sensor/torque.x"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.y", "fts_sensor/torque.y"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "fts_sensor/torque.z"});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  geometry_msgs::msg::WrenchStamped wrench_msg;
  std::string topic_name = "/test_force_torque_sensor_broadcaster/wrench";
  subscribe_and_get_message(wrench_msg, topic_name);

  ASSERT_EQ(wrench_msg.header.frame_id, frame_id_);
  ASSERT_EQ(wrench_msg.wrench.force.x, sensor_values_[0]);
  ASSERT_EQ(wrench_msg.wrench.force.y, sensor_values_[1]);
  ASSERT_EQ(wrench_msg.wrench.force.z, sensor_values_[2]);
  ASSERT_EQ(wrench_msg.wrench.torque.x, sensor_values_[3]);
  ASSERT_EQ(wrench_msg.wrench.torque.y, sensor_values_[4]);
  ASSERT_EQ(wrench_msg.wrench.torque.z, sensor_values_[5]);

  // Check the exported state interfaces
  const auto exported_state_interfaces = fts_broadcaster_->export_state_interfaces();
  ASSERT_EQ(exported_state_interfaces.size(), 6u);
  const std::string controller_name = fts_broadcaster_->get_node()->get_name();
  ASSERT_EQ(exported_state_interfaces[0]->get_name(), controller_name + "/fts_sensor/force.x");
  ASSERT_EQ(exported_state_interfaces[1]->get_name(), controller_name + "/fts_sensor/force.y");
  ASSERT_EQ(exported_state_interfaces[2]->get_name(), controller_name + "/fts_sensor/force.z");
  ASSERT_EQ(exported_state_interfaces[3]->get_name(), controller_name + "/fts_sensor/torque.x");
  ASSERT_EQ(exported_state_interfaces[4]->get_name(), controller_name + "/fts_sensor/torque.y");
  ASSERT_EQ(exported_state_interfaces[5]->get_name(), controller_name + "/fts_sensor/torque.z");
  ASSERT_EQ(exported_state_interfaces[0]->get_interface_name(), "fts_sensor/force.x");
  ASSERT_EQ(exported_state_interfaces[1]->get_interface_name(), "fts_sensor/force.y");
  ASSERT_EQ(exported_state_interfaces[2]->get_interface_name(), "fts_sensor/force.z");
  ASSERT_EQ(exported_state_interfaces[3]->get_interface_name(), "fts_sensor/torque.x");
  ASSERT_EQ(exported_state_interfaces[4]->get_interface_name(), "fts_sensor/torque.y");
  ASSERT_EQ(exported_state_interfaces[5]->get_interface_name(), "fts_sensor/torque.z");
  for (size_t i = 0; i < 6; ++i)
  {
    ASSERT_EQ(exported_state_interfaces[0]->get_prefix_name(), controller_name);
    ASSERT_EQ(exported_state_interfaces[i]->get_value(), sensor_values_[i]);
  }
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorFilterChain_Configure_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster_with_chain");

  // configure passed
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->has_filter_chain_, true);

  // check interface configuration
  auto cmd_if_conf = fts_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = fts_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(6lu));
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  std::cout << "Finished" << std::endl;
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorFilterChain_ActivateDeactivate_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster_with_chain");

  // configure and activate success
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->has_filter_chain_, true);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check interface configuration
  auto cmd_if_conf = fts_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  auto state_if_conf = fts_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(6lu));
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  // deactivate passed
  ASSERT_EQ(fts_broadcaster_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check interface configuration
  cmd_if_conf = fts_broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  ASSERT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::NONE);
  state_if_conf = fts_broadcaster_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(6lu));  // did not change
  ASSERT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorFilterChain_Update_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster_with_chain");

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->has_filter_chain_, true);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    fts_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // Dummy wrenches
  geometry_msgs::msg::WrenchStamped wrench_in;
  geometry_msgs::msg::WrenchStamped wrench_out;

  wrench_in.wrench.force.x = 1.0;
  wrench_in.wrench.force.y = 2.0;
  wrench_in.wrench.force.z = 3.0;
  wrench_in.wrench.torque.x = 4.0;
  wrench_in.wrench.torque.y = 5.0;
  wrench_in.wrench.torque.z = 6.0;

  // Update the filter chain directly (the values from the update are just published)
  fts_broadcaster_->filter_chain_->update(wrench_in, wrench_out);

  ASSERT_EQ(wrench_out.wrench.force.x, wrench_in.wrench.force.x + 1.0);
  ASSERT_EQ(wrench_out.wrench.force.y, wrench_in.wrench.force.y + 1.0);
  ASSERT_EQ(wrench_out.wrench.force.z, wrench_in.wrench.force.z + 1.0);
  ASSERT_EQ(wrench_out.wrench.torque.x, wrench_in.wrench.torque.x + 1.0);
  ASSERT_EQ(wrench_out.wrench.torque.y, wrench_in.wrench.torque.y + 1.0);
  ASSERT_EQ(wrench_out.wrench.torque.z, wrench_in.wrench.torque.z + 1.0);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorFilterChain_Publish_Success)
{
  SetUpFTSBroadcaster("test_force_torque_sensor_broadcaster_with_chain");

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->has_filter_chain_, true);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  geometry_msgs::msg::WrenchStamped wrench_msg_filtered;
  std::string topic_name = "/test_force_torque_sensor_broadcaster_with_chain/wrench_filtered";
  subscribe_and_get_message(wrench_msg_filtered, topic_name);

  ASSERT_EQ(wrench_msg_filtered.header.frame_id, frame_id_);
  ASSERT_EQ(wrench_msg_filtered.wrench.force.x, sensor_values_[0] + 1.0);
  ASSERT_EQ(wrench_msg_filtered.wrench.force.y, sensor_values_[1] + 1.0);
  ASSERT_EQ(wrench_msg_filtered.wrench.force.z, sensor_values_[2] + 1.0);
  ASSERT_EQ(wrench_msg_filtered.wrench.torque.x, sensor_values_[3] + 1.0);
  ASSERT_EQ(wrench_msg_filtered.wrench.torque.y, sensor_values_[4] + 1.0);
  ASSERT_EQ(wrench_msg_filtered.wrench.torque.z, sensor_values_[5] + 1.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
