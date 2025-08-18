// Copyright 2025 ros2_control development team
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

#ifndef TEST_SWERVE_DRIVE_CONTROLLER_HPP_
#define TEST_SWERVE_DRIVE_CONTROLLER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swerve_drive_controller/swerve_drive_controller.hpp"

namespace swerve_drive_controller
{
class SwerveDriveControllerTest : public ::testing::Test
{
protected:
  void SetUp() override;
  void SetUpController();
  void SetUpInterfaces();
  void PublishTwistStamped(double linear_x, double linear_y, double angular_z);
  void PublishTwist(double linear_x, double linear_y, double angular_z);

  std::shared_ptr<SwerveController> controller_;
  std::shared_ptr<rclcpp::Node> node_;
  std::vector<std::string> command_interfaces_;
  std::vector<std::string> state_interfaces_;
  std::vector<double> command_values_;
  std::vector<double> state_values_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_base_;
  std::vector<hardware_interface::StateInterface> state_interfaces_base_;
  std::vector<hardware_interface::LoanedCommandInterface> command_interface_handles_;
  std::vector<hardware_interface::LoanedStateInterface> state_interface_handles_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;
  bool init_successful_ = false;
};

}  // namespace swerve_drive_controller

#endif  // TEST_SWERVE_DRIVE_CONTROLLER_HPP_
