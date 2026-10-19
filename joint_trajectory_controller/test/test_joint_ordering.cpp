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
#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

namespace
{
class SortingController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  SortingController(const std::string & prefix, size_t joint_count)
  {
    dof_ = joint_count;
    for (size_t i = 0; i < joint_count; ++i)
    {
      params_.joints.push_back(prefix + "/joint" + std::to_string(i));
    }
  }

  using JointTrajectoryController::sort_to_local_joint_order;

  trajectory_msgs::msg::JointTrajectory make_trajectory(double offset) const
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = params_.joints;
    msg.points.resize(2);
    for (size_t i = 0; i < dof_; ++i)
    {
      const double value = offset + static_cast<double>(i);
      msg.points[0].positions.push_back(value);
      msg.points[0].velocities.push_back(value + 10.0);
      msg.points[0].accelerations.push_back(value + 20.0);
      msg.points[0].effort.push_back(value + 30.0);
      // The second point exercises empty optional fields after nonempty ones.
      msg.points[1].positions.push_back(value + 40.0);
    }
    return msg;
  }
};

std::shared_ptr<trajectory_msgs::msg::JointTrajectory> reversed_trajectory(
  const trajectory_msgs::msg::JointTrajectory & expected)
{
  auto msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(expected);
  std::reverse(msg->joint_names.begin(), msg->joint_names.end());
  for (auto & point : msg->points)
  {
    std::reverse(point.positions.begin(), point.positions.end());
    std::reverse(point.velocities.begin(), point.velocities.end());
    std::reverse(point.accelerations.begin(), point.accelerations.end());
    std::reverse(point.effort.begin(), point.effort.end());
  }
  return msg;
}

void check_concurrent_sorting(size_t right_joint_count)
{
  SortingController left("left", 6);
  SortingController right("right", right_joint_count);
  std::atomic<unsigned> ready{0};
  std::atomic<bool> go{false};
  std::atomic<bool> failed{false};
  auto run = [&](SortingController & controller, double offset)
  {
    const auto expected = controller.make_trajectory(offset);
    ready.fetch_add(1);
    while (!go.load())
    {
      std::this_thread::yield();
    }
    for (size_t i = 0; i < 100000 && !failed.load(); ++i)
    {
      auto msg = reversed_trajectory(expected);
      controller.sort_to_local_joint_order(msg);
      if (msg->points != expected.points)
      {
        failed.store(true);
      }
    }
  };
  std::thread a(run, std::ref(left), 100.0);
  std::thread b(run, std::ref(right), 1000.0);
  while (ready.load() != 2)
  {
    std::this_thread::yield();
  }
  go.store(true);
  a.join();
  b.join();
  EXPECT_FALSE(failed.load()) << "Concurrent sorting corrupted trajectory dimensions or values";
}
}  // namespace

TEST(JointOrdering, reorders_all_fields_and_preserves_empty_fields)
{
  SortingController controller("arm", 6);
  const auto expected = controller.make_trajectory(100.0);
  auto msg = reversed_trajectory(expected);
  controller.sort_to_local_joint_order(msg);
  EXPECT_EQ(msg->points, expected.points);
}

TEST(JointOrdering, concurrent_controllers_with_equal_joint_counts) { check_concurrent_sorting(6); }

TEST(JointOrdering, concurrent_controllers_with_different_joint_counts)
{
  check_concurrent_sorting(2);
}
