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
#include <utility>
#include <vector>

#include "force_torque_sensor_broadcaster/force_torque_sensor_broadcaster.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

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

void ForceTorqueSensorBroadcasterTest::SetUpFTSBroadcaster()
{
  const auto result = fts_broadcaster_->init("test_force_torque_sensor_broadcaster");
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
    const auto timeout = std::chrono::milliseconds{1};
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
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
