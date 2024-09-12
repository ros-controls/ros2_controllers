// Copyright 2024 ros2_control Development Team
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

#include <gmock/gmock.h>

#include "lifecycle_msgs/msg/state.hpp"
#include "test_mttc_utils.hpp"

#include <data_tamer/data_tamer.hpp>

using lifecycle_msgs::msg::State;

namespace test_mttc
{

// From the tutorial: https://www.sandordargo.com/blog/2019/04/24/parameterized-testing-with-gtest
class MacVisualTest : public TrajectoryControllerTestParameterized
{
public:
  virtual void SetUp()
  {
    TrajectoryControllerTestParameterized::SetUp();
    // set up plotjuggler
  }

  static void TearDownTestCase() { TrajectoryControllerTest::TearDownTestCase(); }
};
}  // namespace test_mttc
using test_mttc::MacVisualTest;

TEST_P(MacVisualTest, visual_test)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  // zero is default value, just for demonstration
  rclcpp::Parameter cmd_timeout_parameter("cmd_timeout", 0.0);
  SetUpAndActivateTrajectoryController(executor, {cmd_timeout_parameter}, false);

  std::size_t n_axes = axis_names_.size();

  // send msg
  constexpr auto FIRST_POINT_TIME = std::chrono::milliseconds(250);
  builtin_interfaces::msg::Duration time_from_start{rclcpp::Duration(FIRST_POINT_TIME)};
  // *INDENT-OFF*
  std::vector<std::vector<double>> points{
    {{3.3, 4.4, 6.6}}, {{7.7, 8.8, 9.9}}, {{10.10, 11.11, 12.12}}};
  std::vector<std::vector<double>> points_velocity{
    {{0.01, 0.01, 0.01}}, {{0.05, 0.05, 0.05}}, {{0.06, 0.06, 0.06}}};
  // *INDENT-ON*
  publish(time_from_start, points, rclcpp::Time(0, 0, RCL_STEADY_TIME), {}, points_velocity);
  traj_controller_->wait_for_trajectory(executor);

  updateController(rclcpp::Duration(FIRST_POINT_TIME) * 4);

  // get states from class variables
  auto state_feedback = traj_controller_->get_state_feedback();
  auto state_reference = traj_controller_->get_state_reference();
  auto state_error = traj_controller_->get_state_error();

  // has the msg the correct vector sizes?
  EXPECT_EQ(n_axes, state_reference.size());

  // is the trajectory still active?
  EXPECT_TRUE(traj_controller_->has_active_traj());
  // should still hold the points from above
  for (std::size_t i = 0; i < n_axes; ++i)
  {
    EXPECT_TRUE(traj_controller_->has_nontrivial_traj(i));
  }

  EXPECT_NEAR(state_reference[0].position, points.at(2).at(0), 1e-2);
  EXPECT_NEAR(state_reference[1].position, points.at(2).at(1), 1e-2);
  EXPECT_NEAR(state_reference[2].position, points.at(2).at(2), 1e-2);
  // value of velocity is different from above due to spline interpolation
  EXPECT_GT(state_reference[0].velocity, 0.0);
  EXPECT_GT(state_reference[1].velocity, 0.0);
  EXPECT_GT(state_reference[2].velocity, 0.0);

  executor.cancel();
}
