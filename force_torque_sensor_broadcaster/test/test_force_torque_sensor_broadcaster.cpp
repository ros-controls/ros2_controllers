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
#include <thread>
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

void ForceTorqueSensorBroadcasterTest::TearDownTestCase()
{
  // Ensure all nodes are properly shutdown
  rclcpp::shutdown();
  // Add a small delay to allow proper cleanup
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void ForceTorqueSensorBroadcasterTest::SetUp()
{
  // initialize controller
  fts_broadcaster_ = std::make_unique<FriendForceTorqueSensorBroadcaster>();
}

void ForceTorqueSensorBroadcasterTest::TearDown()
{
  // Reset the broadcaster with proper cleanup
  if (fts_broadcaster_)
  {
    if (
      fts_broadcaster_->get_lifecycle_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      if (
        fts_broadcaster_->get_lifecycle_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        ASSERT_EQ(fts_broadcaster_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
      }
      // Clean up the broadcaster
      ASSERT_EQ(fts_broadcaster_->on_cleanup(rclcpp_lifecycle::State()), NODE_SUCCESS);
    }
    fts_broadcaster_.reset(nullptr);
  }
  // Add a small delay between tests
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void ForceTorqueSensorBroadcasterTest::SetUpFTSBroadcaster()
{
  const auto result = fts_broadcaster_->init(
    "test_force_torque_sensor_broadcaster", "", 0, "",
    fts_broadcaster_->define_custom_node_options());
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
  geometry_msgs::msg::WrenchStamped & wrench_msg)
{
  // create a new subscriber
  geometry_msgs::msg::WrenchStamped::SharedPtr received_msg;
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  { received_msg = msg; };
  auto subscription = test_subscription_node.create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/test_force_torque_sensor_broadcaster/wrench", 10, subs_callback);
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
  SetUpFTSBroadcaster();

  // configure failed
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_InterfaceNames_Set)
{
  SetUpFTSBroadcaster();

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
  SetUpFTSBroadcaster();

  // set the 'sensor_name' empty
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", ""});

  // configure failed, 'sensor_name' parameter empty
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_IsEmpty_SensorName_NotSet)
{
  SetUpFTSBroadcaster();

  // set the 'interface_names' empty
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", ""});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", ""});

  // configure failed, 'interface_name' parameter empty
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_Configure_Success)
{
  SetUpFTSBroadcaster();

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
  SetUpFTSBroadcaster();

  // set the 'interface_names'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fts_sensor/force.x"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "fts_sensor/torque.z"});

  // set the 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // configure passed
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(ForceTorqueSensorBroadcasterTest, SensorName_ActivateDeactivate_Success)
{
  SetUpFTSBroadcaster();

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
  SetUpFTSBroadcaster();

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
  SetUpFTSBroadcaster();

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
  SetUpFTSBroadcaster();

  // set the params 'sensor_name' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"sensor_name", sensor_name_});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  geometry_msgs::msg::WrenchStamped wrench_msg;
  subscribe_and_get_message(wrench_msg);

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
  SetUpFTSBroadcaster();

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
  subscribe_and_get_message(wrench_msg);

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

TEST_F(ForceTorqueSensorBroadcasterTest, InterfaceNames_Publish_Success)
{
  SetUpFTSBroadcaster();

  // set the params 'interface_names' and 'frame_id'
  fts_broadcaster_->get_node()->set_parameter({"interface_names.force.x", "fts_sensor/force.x"});
  fts_broadcaster_->get_node()->set_parameter({"interface_names.torque.z", "fts_sensor/torque.z"});
  fts_broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(fts_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  geometry_msgs::msg::WrenchStamped wrench_msg;
  subscribe_and_get_message(wrench_msg);

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
  SetUpFTSBroadcaster();

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
  subscribe_and_get_message(wrench_msg);

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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
