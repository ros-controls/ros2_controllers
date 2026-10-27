// Copyright 2026 ros2_control Development Team
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

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

namespace
{
class SortingController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  SortingController(const std::string & prefix, size_t joint_count)
  {
    std::vector<std::string> joints;
    for (size_t i = 0; i < joint_count; ++i)
    {
      joints.push_back(prefix + "/joint" + std::to_string(i));
    }
    controller_interface::ControllerInterfaceParams controller_params;
    controller_params.controller_name = prefix + "_sorting";
    controller_params.update_rate = 100;
    controller_params.controller_manager_update_rate = 100;
    controller_params.node_options.parameter_overrides(
      {rclcpp::Parameter("joints", joints),
       rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position"}),
       rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position"})});
    if (init(controller_params) != controller_interface::return_type::OK)
    {
      throw std::runtime_error("Failed to initialize sorting controller");
    }
    // Sorting needs the joint count normally established during configuration.
    dof_ = joint_count;
  }

  using JointTrajectoryController::sort_to_local_joint_order;

  trajectory_msgs::msg::JointTrajectory make_trajectory(double offset) const
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = params_.joints;
    msg.points.resize(4);
    for (size_t i = 0; i < dof_; ++i)
    {
      const double value = offset + static_cast<double>(i);
      msg.points[0].positions.push_back(value);
      msg.points[0].velocities.push_back(value + 10.0);
      msg.points[0].accelerations.push_back(value + 20.0);
      msg.points[0].effort.push_back(value + 30.0);
      // The second point exercises empty optional fields after nonempty ones.
      msg.points[1].positions.push_back(value + 40.0);
      // Also cover empty positions and populated fields following empty ones.
      msg.points[2].velocities.push_back(value + 50.0);
      msg.points[3].positions.push_back(value + 60.0);
      msg.points[3].velocities.push_back(value + 70.0);
      msg.points[3].accelerations.push_back(value + 80.0);
      msg.points[3].effort.push_back(value + 90.0);
    }
    return msg;
  }
};

std::shared_ptr<trajectory_msgs::msg::JointTrajectory> reordered_trajectory(
  const trajectory_msgs::msg::JointTrajectory & expected, bool reverse = false)
{
  auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(expected);
  auto reorder = [reverse](auto & field)
  {
    if (reverse)
    {
      std::reverse(field.begin(), field.end());
    }
    else if (!field.empty())
    {
      // A cyclic shift of three or more joints distinguishes a map from its inverse.
      std::rotate(field.begin(), field.begin() + 1, field.end());
    }
  };
  reorder(msg->joint_names);
  for (auto & point : msg->points)
  {
    reorder(point.positions);
    reorder(point.velocities);
    reorder(point.accelerations);
    reorder(point.effort);
  }
  return msg;
}

void check_concurrent_sorting(size_t right_joint_count)
{
  SortingController left("left", 6);
  SortingController right("right", right_joint_count);
  const auto left_expected = left.make_trajectory(100.0);
  const auto right_expected = right.make_trajectory(1000.0);
  std::array<std::exception_ptr, 2> errors;
  std::atomic<unsigned> ready{0};
  std::atomic<bool> go{false};
  std::atomic<bool> failed{false};
  auto run = [&](
               SortingController & controller,
               const trajectory_msgs::msg::JointTrajectory & expected, std::exception_ptr & error)
  {
    ready.fetch_add(1);
    while (!go.load())
    {
      std::this_thread::yield();
    }
    try
    {
      // Stress coverage is probabilistic; ThreadSanitizer can also detect shared-buffer races.
      for (size_t i = 0; i < 10000 && !failed.load(); ++i)
      {
        auto msg = reordered_trajectory(expected);
        controller.sort_to_local_joint_order(msg);
        if (msg->points != expected.points)
        {
          failed.store(true);
        }
      }
    }
    catch (...)
    {
      error = std::current_exception();
      failed.store(true);
    }
  };
  std::thread a(run, std::ref(left), std::cref(left_expected), std::ref(errors[0]));
  std::thread b(run, std::ref(right), std::cref(right_expected), std::ref(errors[1]));
  while (ready.load() != 2)
  {
    std::this_thread::yield();
  }
  go.store(true);
  a.join();
  b.join();
  for (const auto & error : errors)
  {
    if (error)
    {
      EXPECT_NO_THROW(std::rethrow_exception(error));
    }
  }
  EXPECT_FALSE(failed.load()) << "Concurrent sorting corrupted trajectory dimensions or values";
}

class JointOrdering : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};
}  // namespace

TEST_F(JointOrdering, reorders_all_fields_and_preserves_empty_fields)
{
  SortingController controller("arm", 6);
  const auto expected = controller.make_trajectory(100.0);
  for (const bool reverse : {false, true})
  {
    SCOPED_TRACE(reverse ? "reverse" : "cyclic shift");
    auto msg = reordered_trajectory(expected, reverse);
    controller.sort_to_local_joint_order(msg);
    EXPECT_EQ(msg->points, expected.points);
  }
}

TEST_F(JointOrdering, preserves_fields_with_invalid_sizes_without_throwing)
{
  SortingController controller("arm", 6);
  const auto expected = controller.make_trajectory(100.0);
  using Point = trajectory_msgs::msg::JointTrajectoryPoint;
  for (const auto field :
       {&Point::positions, &Point::velocities, &Point::accelerations, &Point::effort})
  {
    for (const size_t invalid_size : {size_t{5}, size_t{7}})
    {
      auto msg = reordered_trajectory(expected);
      (msg->points[0].*field).resize(invalid_size);
      const auto invalid_field = msg->points[0].*field;
      ASSERT_NO_THROW(controller.sort_to_local_joint_order(msg));
      EXPECT_EQ(msg->points[0].*field, invalid_field);
    }
  }
}

TEST_F(JointOrdering, concurrent_controllers_with_equal_joint_counts)
{
  check_concurrent_sorting(6);
}

TEST_F(JointOrdering, concurrent_controllers_with_different_joint_counts)
{
  check_concurrent_sorting(2);
}
