// Copyright 2025 ros2_control Development Team
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
//

#include "test_pid_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

class PidControllerDualInterfaceTest : public PidControllerFixture<TestablePidController>
{
public:
  void SetUp()
  {
    PidControllerFixture::SetUp();

    dof_names_ = {"joint1", "joint2"};

    // set up interfaces
    command_interface_ = "velocity";
    state_interfaces_ = {"position", "velocity"};
    dof_state_values_ = {
      get_joint1_state_position(), get_joint2_state_position(), get_joint1_state_velocity(),
      get_joint2_state_velocity()};
  }

  double get_joint1_state_position() const { return 10.0; }
  double get_joint2_state_position() const { return 11.0; }
  double get_joint1_state_velocity() const { return 5.0; }
  double get_joint2_state_velocity() const { return 6.0; }

  double get_joint1_reference_position() const { return 15.0; }
  double get_joint2_reference_position() const { return 16.0; }
  double get_joint1_reference_velocity() const { return 6.0; }
  double get_joint2_reference_velocity() const { return 7.0; }
};

/**
 * @brief Test the feedforward gain with chained mode with two interfaces
 *  The second interface should be derivative of the first one
 */

TEST_F(PidControllerDualInterfaceTest, test_chained_feedforward_with_gain_dual_interface)
{
  SetUpController("test_pid_controller_with_feedforward_gain_dual_interface");
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check on interfaces & pid gain parameters
  for (const auto & dof_name : dof_names_)
  {
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].p, 0.5);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].i, 0.3);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].d, 0.4);
    ASSERT_EQ(controller_->params_.gains.dof_names_map[dof_name].feedforward_gain, 1.0);
  }

  ASSERT_EQ(controller_->params_.command_interface, command_interface_);
  EXPECT_THAT(
    controller_->params_.reference_and_state_interfaces,
    testing::ElementsAreArray(state_interfaces_));
  controller_->set_chained_mode(true);

  // activate controller
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(controller_->is_in_chained_mode());

  // set up the reference interface,
  controller_->reference_interfaces_ = {
    get_joint1_reference_position(), get_joint2_reference_position(),
    get_joint1_reference_velocity(), get_joint2_reference_velocity()};

  // run update
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check the commands
  const double joint1_expected_cmd = 8.90;
  const double joint2_expected_cmd = 9.90;
  ASSERT_EQ(controller_->command_interfaces_[0].get_optional().value(), joint1_expected_cmd);
  ASSERT_EQ(controller_->command_interfaces_[1].get_optional().value(), joint2_expected_cmd);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
