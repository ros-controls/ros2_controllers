// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "test_force_torque_sensor_broadcaster_with_filters.hpp"

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

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace

void ForceTorqueSensorBroadcasterWithFilterTest::SetUpTestCase() {}

void ForceTorqueSensorBroadcasterWithFilterTest::TearDownTestCase() {}

void ForceTorqueSensorBroadcasterWithFilterTest::SetUp()
{
  // initialize controller
  fts_broadcaster_ = std::make_unique<FriendForceTorqueSensorBroadcasterWithFilter>();
}

void ForceTorqueSensorBroadcasterWithFilterTest::TearDown() { fts_broadcaster_.reset(nullptr); }

void ForceTorqueSensorBroadcasterWithFilterTest::SetUpFTSBroadcaster(const std::string & name)
{
  const auto result =
    fts_broadcaster_->init(name, "", 0, "", fts_broadcaster_->define_custom_node_options());
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

TEST_F(ForceTorqueSensorBroadcasterWithFilterTest, NoFilter_Configure_Success)
{
  SetUpFTSBroadcaster("FTS_NoFilter");

  // configure passed (no filter means the filter_chain is still valid)
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  geometry_msgs::msg::WrenchStamped wrench_in, wrench_out;
  wrench_in.wrench.force.x = 1.0;

  // update passed
  EXPECT_TRUE(fts_broadcaster_->filter_chain_->update(wrench_in, wrench_out));

  // no change of the data
  ASSERT_EQ(wrench_out.wrench.force.x, wrench_in.wrench.force.x);
  ASSERT_EQ(wrench_out.wrench.force.y, wrench_in.wrench.force.y);
  ASSERT_EQ(wrench_out.wrench.force.z, wrench_in.wrench.force.z);
  ASSERT_EQ(wrench_out.wrench.torque.x, wrench_in.wrench.torque.x);
  ASSERT_EQ(wrench_out.wrench.torque.y, wrench_in.wrench.torque.y);
  ASSERT_EQ(wrench_out.wrench.torque.z, wrench_in.wrench.torque.z);
}

TEST_F(ForceTorqueSensorBroadcasterWithFilterTest, SingleFilterLP_Configure_Update)
{
  SetUpFTSBroadcaster("FTS_SingleFilterLP");

  // debug helper, in case filters is not found
  pluginlib::ClassLoader<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_loader(
    "filters", "filters::FilterBase<geometry_msgs::msg::WrenchStamped>");
  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter;
  std::stringstream sstr;
  sstr << "available filters:" << std::endl;
  for (const auto & available_class : filter_loader.getDeclaredClasses())
  {
    sstr << "  " << available_class << std::endl;
  }

  // configure passed with one filter
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS) << sstr.str();
  geometry_msgs::msg::WrenchStamped wrench_in, wrench_out;
  wrench_in.header.frame_id = "world";
  wrench_in.wrench.force.x = 1.0;

  // update passed
  EXPECT_TRUE(fts_broadcaster_->filter_chain_->update(wrench_in, wrench_out));
  // udpdate twice because currently the IIR has a zero output at first iteration
  EXPECT_TRUE(fts_broadcaster_->filter_chain_->update(wrench_in, wrench_out));

  // test if the correct low-pass filter was loaded
  // sampling freq:200, damping_freq:50, damping_intensity:1 => a1 = 0.1284 , b1= 1-a1 =0.8616
  double sampling_freq = 200.0;
  double damping_freq = 50.0;
  double damping_intensity = 1.0;
  double a1, b1;
  a1 = exp(
    -1.0 / sampling_freq * (2.0 * M_PI * damping_freq) / (pow(10.0, damping_intensity / -10.0)));
  b1 = 1.0 - a1;
  // expect change
  ASSERT_EQ(wrench_out.wrench.force.x, b1 * wrench_in.wrench.force.x);
  ASSERT_EQ(wrench_out.wrench.force.y, wrench_in.wrench.force.y);
  ASSERT_EQ(wrench_out.wrench.force.z, wrench_in.wrench.force.z);
  ASSERT_EQ(wrench_out.wrench.torque.x, wrench_in.wrench.torque.x);
  ASSERT_EQ(wrench_out.wrench.torque.y, wrench_in.wrench.torque.y);
  ASSERT_EQ(wrench_out.wrench.torque.z, wrench_in.wrench.torque.z);
}

TEST_F(ForceTorqueSensorBroadcasterWithFilterTest, SingleFilterGC_Configure_Update)
{
  SetUpFTSBroadcaster("FTS_SingleFilterGC");

  // debug helper, in case filters is not found
  pluginlib::ClassLoader<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_loader(
    "filters", "filters::FilterBase<geometry_msgs::msg::WrenchStamped>");
  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter;
  std::stringstream sstr;
  sstr << "available filters:" << std::endl;
  for (const auto & available_class : filter_loader.getDeclaredClasses())
  {
    sstr << "  " << available_class << std::endl;
  }

  // configure passed with one filter
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS) << sstr.str();
  geometry_msgs::msg::WrenchStamped wrench_in, wrench_out;
  wrench_in.header.frame_id = "world";
  wrench_in.wrench.force.x = 1.0;

  // update passed
  EXPECT_TRUE(fts_broadcaster_->filter_chain_->update(wrench_in, wrench_out));

  // test if the correct gravity compensation filter was loaded (should match values in yaml file)
  double gravity_acc = 9.81;
  double mass = 5.0;
  double cog_y = 0.1;
  // with no tf different with world, sensor frame is world frame, calculation is simplified
  double grav_force_z = gravity_acc * mass;
  double grav_torque_x = cog_y * grav_force_z;
  // expect change
  geometry_msgs::msg::WrenchStamped in, out;
  in.header.frame_id = "world";
  in.wrench.force.x = 1.0;
  in.wrench.torque.x = 10.0;

  ASSERT_EQ(wrench_out.wrench.force.x, wrench_in.wrench.force.x);
  ASSERT_EQ(wrench_out.wrench.force.y, wrench_in.wrench.force.y);
  EXPECT_NEAR(wrench_out.wrench.force.z, wrench_in.wrench.force.z + grav_force_z, COMMON_THRESHOLD);
  EXPECT_NEAR(
    wrench_out.wrench.torque.x, wrench_in.wrench.torque.x + grav_torque_x, COMMON_THRESHOLD);
  ASSERT_EQ(wrench_out.wrench.torque.y, wrench_in.wrench.torque.y);
  ASSERT_EQ(wrench_out.wrench.torque.z, wrench_in.wrench.torque.z);
}

TEST_F(ForceTorqueSensorBroadcasterWithFilterTest, ChainedFilterLPGC_Configure_Update)
{
  SetUpFTSBroadcaster("FTS_ChainedFilterLPGC");

  // debug helper, in case filters is not found
  pluginlib::ClassLoader<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter_loader(
    "filters", "filters::FilterBase<geometry_msgs::msg::WrenchStamped>");
  std::shared_ptr<filters::FilterBase<geometry_msgs::msg::WrenchStamped>> filter;
  std::stringstream sstr;
  sstr << "available filters:" << std::endl;
  for (const auto & available_class : filter_loader.getDeclaredClasses())
  {
    sstr << "  " << available_class << std::endl;
  }

  // configure passed with two filters
  ASSERT_EQ(fts_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS) << sstr.str();
  geometry_msgs::msg::WrenchStamped wrench_in, wrench_out;
  wrench_in.header.frame_id = "world";
  wrench_in.wrench.force.x = 1.0;

  // update passed
  EXPECT_TRUE(fts_broadcaster_->filter_chain_->update(wrench_in, wrench_out));

  // test if the correct chained filter was loaded (should match values in yaml file)
  double sampling_freq = 200.0;
  double damping_freq = 50.0;
  double damping_intensity = 1.0;
  double a1, b1;
  a1 = exp(
    -1.0 / sampling_freq * (2.0 * M_PI * damping_freq) / (pow(10.0, damping_intensity / -10.0)));
  b1 = 1.0 - a1;

  double gravity_acc = 9.81;
  double mass = 5.0;
  double cog_y = 0.1;
  // with no tf different with world, sensor frame is world frame, calculation is simplified
  double grav_force_z = gravity_acc * mass;
  double grav_torque_x = cog_y * grav_force_z;

  geometry_msgs::msg::WrenchStamped in, out;
  in.header.frame_id = "world";
  in.wrench.force.x = 1.0;
  in.wrench.torque.x = 10.0;
  // expect change
  ASSERT_EQ(wrench_out.wrench.force.x, 0.0);
  ASSERT_EQ(wrench_out.wrench.force.y, 0.0);
  EXPECT_NEAR(wrench_out.wrench.force.z, 0.0 + grav_force_z, COMMON_THRESHOLD);
  EXPECT_NEAR(wrench_out.wrench.torque.x, 0.0 + grav_torque_x, COMMON_THRESHOLD);
  ASSERT_EQ(wrench_out.wrench.torque.y, 0.0);
  ASSERT_EQ(wrench_out.wrench.torque.z, 0.0);

  // udpdate twice because currently the IIR has a zero output at first iteration
  EXPECT_TRUE(fts_broadcaster_->filter_chain_->update(wrench_in, wrench_out));

  ASSERT_EQ(wrench_out.wrench.force.x, b1 * wrench_in.wrench.force.x);
  ASSERT_EQ(wrench_out.wrench.force.y, b1 * wrench_in.wrench.force.y);
  EXPECT_NEAR(
    wrench_out.wrench.force.z, b1 * wrench_in.wrench.force.z + grav_force_z, COMMON_THRESHOLD);
  EXPECT_NEAR(
    wrench_out.wrench.torque.x, b1 * wrench_in.wrench.torque.x + grav_torque_x, COMMON_THRESHOLD);
  ASSERT_EQ(wrench_out.wrench.torque.y, b1 * wrench_in.wrench.torque.y);
  ASSERT_EQ(wrench_out.wrench.torque.z, b1 * wrench_in.wrench.torque.z);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
