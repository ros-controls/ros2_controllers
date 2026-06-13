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

#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "test_steering_controllers_library.hpp"

class SteeringControllersLibraryTest
: public SteeringControllersLibraryFixture<TestableSteeringControllersLibrary>
{
};

// checking if all interfaces, command, state and reference are exported as expected
TEST_F(SteeringControllersLibraryTest, check_exported_interfaces)
{
  SetUpController();

  ASSERT_TRUE(configure_succeeds(controller_));

  auto cmd_if_conf = controller_->command_interface_configuration();
  ASSERT_EQ(cmd_if_conf.names.size(), joint_command_values_.size());
  EXPECT_EQ(
    cmd_if_conf.names[CMD_TRACTION_RIGHT_WHEEL],
    traction_joints_names_[0] + "/" + traction_interface_name_);
  EXPECT_EQ(
    cmd_if_conf.names[CMD_TRACTION_LEFT_WHEEL],
    traction_joints_names_[1] + "/" + traction_interface_name_);
  EXPECT_EQ(
    cmd_if_conf.names[CMD_STEER_RIGHT_WHEEL],
    steering_joints_names_[0] + "/" + steering_interface_name_);
  EXPECT_EQ(
    cmd_if_conf.names[CMD_STEER_LEFT_WHEEL],
    steering_joints_names_[1] + "/" + steering_interface_name_);
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  auto state_if_conf = controller_->state_interface_configuration();
  ASSERT_EQ(state_if_conf.names.size(), joint_state_values_.size());
  EXPECT_EQ(
    state_if_conf.names[STATE_TRACTION_RIGHT_WHEEL],
    controller_->traction_joints_state_names_[0] + "/" + traction_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_TRACTION_LEFT_WHEEL],
    controller_->traction_joints_state_names_[1] + "/" + traction_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_STEER_RIGHT_WHEEL],
    controller_->steering_joints_state_names_[0] + "/" + steering_interface_name_);
  EXPECT_EQ(
    state_if_conf.names[STATE_STEER_LEFT_WHEEL],
    controller_->steering_joints_state_names_[1] + "/" + steering_interface_name_);
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  // check ref itfs
  auto reference_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(reference_interfaces.size(), reference_interface_names_.size());
  for (size_t i = 0; i < reference_interface_names_.size(); ++i)
  {
    const std::string ref_itf_prefix_name =
      std::string(controller_->get_node()->get_name()) + "/" + reference_interface_names_[i];
    EXPECT_EQ(reference_interfaces[i]->get_prefix_name(), ref_itf_prefix_name);
    EXPECT_EQ(
      reference_interfaces[i]->get_name(),
      ref_itf_prefix_name + "/" + hardware_interface::HW_IF_VELOCITY);
    EXPECT_EQ(reference_interfaces[i]->get_interface_name(), hardware_interface::HW_IF_VELOCITY);
  }
}

// TF prefix tests
TEST_F(SteeringControllersLibraryTest, configure_succeeds_tf_prefix_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix));
  node_options.append_parameter_override("odom_frame_id", rclcpp::ParameterValue(odom_id));
  node_options.append_parameter_override("base_frame_id", rclcpp::ParameterValue(base_link_id));

  SetUpController("test_steering_controllers_library", node_options);

  ASSERT_TRUE(configure_succeeds(controller_));

  auto odometry_message = controller_->odom_state_msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;

  // frame_prefix is not blank so should be prepended to the frame id's
  ASSERT_EQ(test_odom_frame_id, frame_prefix + "/" + odom_id);
  ASSERT_EQ(test_base_frame_id, frame_prefix + "/" + base_link_id);
}
TEST_F(SteeringControllersLibraryTest, configure_succeeds_tf_blank_prefix_no_namespace)
{
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "";

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix));
  node_options.append_parameter_override("odom_frame_id", rclcpp::ParameterValue(odom_id));
  node_options.append_parameter_override("base_frame_id", rclcpp::ParameterValue(base_link_id));

  SetUpController("test_steering_controllers_library", node_options);

  ASSERT_TRUE(configure_succeeds(controller_));

  auto odometry_message = controller_->odom_state_msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;

  // frame_prefix is blank so nothing added to the frame id's
  ASSERT_EQ(test_odom_frame_id, odom_id);
  ASSERT_EQ(test_base_frame_id, base_link_id);
}
TEST_F(SteeringControllersLibraryTest, configure_succeeds_tf_prefix_set_namespace)
{
  std::string test_namespace = "/test_namespace";

  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "test_prefix";

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix));
  node_options.append_parameter_override("odom_frame_id", rclcpp::ParameterValue(odom_id));
  node_options.append_parameter_override("base_frame_id", rclcpp::ParameterValue(base_link_id));
  node_options.append_parameter_override(
    "traction_joints_names", std::vector<std::string>{"joint1", "joint2"});
  node_options.append_parameter_override(
    "steering_joints_names", std::vector<std::string>{"joint3", "joint4"});

  SetUpController("test_steering_controllers_library", node_options, test_namespace);

  ASSERT_TRUE(configure_succeeds(controller_));

  auto odometry_message = controller_->odom_state_msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;

  // frame_prefix is not blank so should be prepended to the frame id's instead of the namespace
  ASSERT_EQ(test_odom_frame_id, frame_prefix + "/" + odom_id);
  ASSERT_EQ(test_base_frame_id, frame_prefix + "/" + base_link_id);
}
TEST_F(SteeringControllersLibraryTest, configure_succeeds_tf_tilde_prefix_set_namespace)
{
  std::string test_namespace = "/test_namespace";
  std::string odom_id = "odom";
  std::string base_link_id = "base_link";
  std::string frame_prefix = "~";

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("tf_frame_prefix", rclcpp::ParameterValue(frame_prefix));
  node_options.append_parameter_override("odom_frame_id", rclcpp::ParameterValue(odom_id));
  node_options.append_parameter_override("base_frame_id", rclcpp::ParameterValue(base_link_id));
  node_options.append_parameter_override(
    "traction_joints_names", std::vector<std::string>{"joint1", "joint2"});
  node_options.append_parameter_override(
    "steering_joints_names", std::vector<std::string>{"joint3", "joint4"});

  SetUpController("test_steering_controllers_library", node_options, test_namespace);

  ASSERT_TRUE(configure_succeeds(controller_));

  auto odometry_message = controller_->odom_state_msg_;
  std::string test_odom_frame_id = odometry_message.header.frame_id;
  std::string test_base_frame_id = odometry_message.child_frame_id;
  std::string ns_prefix = test_namespace.erase(0, 1) + "/";

  // frame_prefix has tilde (~) character so node namespace should be prepended to the frame id's
  ASSERT_EQ(test_odom_frame_id, ns_prefix + odom_id);
  ASSERT_EQ(test_base_frame_id, ns_prefix + base_link_id);
}

// Tests controller update_reference_from_subscribers and
// for position_feedback behavior
// when too old msg is sent i.e age_of_last_command > ref_timeout case
TEST_F(SteeringControllersLibraryTest, test_position_feedback_ref_timeout)
{
  SetUpController();
  controller_->params_.position_feedback = true;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(configure_succeeds(controller_));
  controller_->set_chained_mode(false);
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_FALSE(controller_->is_in_chained_mode());

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  // set command statically
  const double TEST_LINEAR_VELOCITY_X = 1.5;
  const double TEST_LINEAR_VELOCITY_Y = 0.0;
  const double TEST_ANGULAR_VELOCITY_Z = 0.3;

  ControllerReferenceMsg msg;

  msg.header.stamp = controller_->get_node()->now();
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_Y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  // age_of_last_command < ref_timeout_
  auto age_of_last_command =
    controller_->get_node()->now() - controller_->input_ref_.get().header.stamp;
  ASSERT_TRUE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ(controller_->input_ref_.get().twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  ASSERT_FALSE(std::isnan(controller_->input_ref_.get().twist.linear.x));
  ASSERT_FALSE(std::isnan(controller_->input_ref_.get().twist.angular.z));

  // are the command_itfs updated?
  EXPECT_NEAR(controller_->command_interfaces_[0].get_optional().value(), 3.333333, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[1].get_optional().value(), 3.333333, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[2].get_optional().value(), 0.575875, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[3].get_optional().value(), 0.575875, 1e-6);

  // adjusting to achieve age_of_last_command > ref_timeout
  msg.header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                     rclcpp::Duration::from_seconds(0.1);
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_Y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  age_of_last_command = controller_->get_node()->now() - controller_->input_ref_.get().header.stamp;

  // adjusting to achieve age_of_last_command > ref_timeout
  msg.header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                     rclcpp::Duration::from_seconds(0.1);
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_Y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  // age_of_last_command > ref_timeout_
  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ(controller_->input_ref_.get().twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  ASSERT_TRUE(std::isnan(controller_->input_ref_.get().twist.linear.x));
  ASSERT_TRUE(std::isnan(controller_->input_ref_.get().twist.angular.z));

  // Wheel velocities should reset to 0
  EXPECT_EQ(controller_->command_interfaces_[0].get_optional().value(), 0);
  EXPECT_EQ(controller_->command_interfaces_[1].get_optional().value(), 0);

  // Steer angles should not reset
  EXPECT_NEAR(controller_->command_interfaces_[2].get_optional().value(), 0.575875, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[3].get_optional().value(), 0.575875, 1e-6);
}
// Tests controller update_reference_from_subscribers and
// for position_feedback=false behavior
// when too old msg is sent i.e age_of_last_command > ref_timeout case
TEST_F(SteeringControllersLibraryTest, test_velocity_feedback_ref_timeout)
{
  SetUpController();
  controller_->params_.position_feedback = false;

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  ASSERT_TRUE(configure_succeeds(controller_));
  controller_->set_chained_mode(false);
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_FALSE(controller_->is_in_chained_mode());

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }

  // set command statically
  const double TEST_LINEAR_VELOCITY_X = 1.5;
  const double TEST_LINEAR_VELOCITY_Y = 0.0;
  const double TEST_ANGULAR_VELOCITY_Z = 0.3;

  ControllerReferenceMsg msg;

  msg.header.stamp = controller_->get_node()->now();
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_Y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  auto age_of_last_command =
    controller_->get_node()->now() - controller_->input_ref_.get().header.stamp;

  // age_of_last_command < ref_timeout_
  ASSERT_TRUE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ(controller_->input_ref_.get().twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  ASSERT_FALSE(std::isnan(controller_->input_ref_.get().twist.linear.x));
  ASSERT_FALSE(std::isnan(controller_->input_ref_.get().twist.angular.z));

  // are the command_itfs updated?
  EXPECT_NEAR(controller_->command_interfaces_[0].get_optional().value(), 3.333333, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[1].get_optional().value(), 3.333333, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[2].get_optional().value(), 0.575875, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[3].get_optional().value(), 0.575875, 1e-6);

  // adjusting to achieve age_of_last_command > ref_timeout
  msg.header.stamp = controller_->get_node()->now() - controller_->ref_timeout_ -
                     rclcpp::Duration::from_seconds(0.1);
  msg.twist.linear.x = TEST_LINEAR_VELOCITY_X;
  msg.twist.linear.y = TEST_LINEAR_VELOCITY_Y;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = TEST_ANGULAR_VELOCITY_Z;
  controller_->input_ref_.set(msg);

  age_of_last_command = controller_->get_node()->now() - controller_->input_ref_.get().header.stamp;

  // age_of_last_command > ref_timeout_
  ASSERT_FALSE(age_of_last_command <= controller_->ref_timeout_);
  ASSERT_EQ(controller_->input_ref_.get().twist.linear.x, TEST_LINEAR_VELOCITY_X);
  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (const auto & interface : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(interface));
  }
  ASSERT_TRUE(std::isnan(controller_->input_ref_.get().twist.linear.x));
  ASSERT_TRUE(std::isnan(controller_->input_ref_.get().twist.angular.z));

  // Wheel velocities should reset to 0
  EXPECT_EQ(controller_->command_interfaces_[0].get_optional().value(), 0);
  EXPECT_EQ(controller_->command_interfaces_[1].get_optional().value(), 0);

  // Steer angles should not reset
  EXPECT_NEAR(controller_->command_interfaces_[2].get_optional().value(), 0.575875, 1e-6);
  EXPECT_NEAR(controller_->command_interfaces_[3].get_optional().value(), 0.575875, 1e-6);
}

TEST_F(SteeringControllersLibraryTest, applies_velocity_limits_to_references)
{
  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("linear.x.max_velocity", rclcpp::ParameterValue(0.1));
  node_options.append_parameter_override("angular.z.max_velocity", rclcpp::ParameterValue(0.1));
  SetUpController("test_steering_controllers_library", node_options);

  ASSERT_TRUE(configure_succeeds(controller_));
  controller_->set_chained_mode(false);
  ASSERT_TRUE(activate_succeeds(controller_));

  ControllerReferenceMsg msg;
  msg.header.stamp = controller_->get_node()->now();
  msg.twist.linear.x = 1.5;
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg.twist.angular.z = 0.3;
  controller_->input_ref_.set(msg);

  const auto expected_commands = controller_->odometry_.get_commands(
    0.1, 0.1, controller_->params_.open_loop,
    controller_->params_.reduce_wheel_speed_until_steering_reached);

  ASSERT_EQ(
    controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_NEAR(
    controller_->command_interfaces_[CMD_TRACTION_RIGHT_WHEEL].get_optional().value(),
    std::get<0>(expected_commands)[CMD_TRACTION_RIGHT_WHEEL], 1e-6);
  EXPECT_NEAR(
    controller_->command_interfaces_[CMD_TRACTION_LEFT_WHEEL].get_optional().value(),
    std::get<0>(expected_commands)[CMD_TRACTION_LEFT_WHEEL], 1e-6);
  EXPECT_NEAR(
    controller_->command_interfaces_[CMD_STEER_RIGHT_WHEEL].get_optional().value(),
    std::get<1>(expected_commands)[0], 1e-6);
  EXPECT_NEAR(
    controller_->command_interfaces_[CMD_STEER_LEFT_WHEEL].get_optional().value(),
    std::get<1>(expected_commands)[1], 1e-6);
}

TEST_F(SteeringControllersLibraryTest, test_reset_buffers_clears_limiter_state)
{
  SetUpController("test_steering_controllers_library");

  ASSERT_TRUE(configure_succeeds(controller_));
  controller_->set_chained_mode(true);
  ASSERT_TRUE(activate_succeeds(controller_));

  // Dirty all buffers that reset_buffers() is responsible for clearing.
  controller_->reference_interfaces_[0] = 1.0;
  controller_->reference_interfaces_[1] = 2.0;

  std::queue<std::array<double, 2>> dirty;
  dirty.push({{4.0, 5.0}});
  dirty.push({{6.0, 7.0}});
  std::swap(controller_->previous_two_commands_, dirty);

  ControllerReferenceMsg dirty_ref;
  dirty_ref.header.stamp = controller_->get_node()->now();
  dirty_ref.twist.linear.x = 1.0;
  dirty_ref.twist.angular.z = 2.0;
  controller_->input_ref_.set(dirty_ref);

  controller_->reset_buffers();

  for (const auto & itf : controller_->reference_interfaces_)
  {
    EXPECT_TRUE(std::isnan(itf));
  }
  ASSERT_EQ(controller_->previous_two_commands_.size(), 2u);
  EXPECT_EQ(controller_->previous_two_commands_.front(), (std::array<double, 2>{{0.0, 0.0}}));
  EXPECT_EQ(controller_->previous_two_commands_.back(), (std::array<double, 2>{{0.0, 0.0}}));

  auto reset_ref = controller_->input_ref_.get();
  EXPECT_TRUE(std::isnan(reset_ref.twist.linear.x));
  EXPECT_TRUE(std::isnan(reset_ref.twist.angular.z));
}

TEST_F(SteeringControllersLibraryTest, test_lifecycle_transitions_reset_limiter_buffers)
{
  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override("linear.x.max_acceleration", rclcpp::ParameterValue(2.0));

  SetUpController("test_steering_controllers_library", node_options);

  ASSERT_TRUE(configure_succeeds(controller_));
  controller_->set_chained_mode(true);
  ASSERT_TRUE(activate_succeeds(controller_));

  const double dt = 0.001;
  const double linear = 1.0;
  const double max_acceleration = 2.0;
  const double time_acc = linear / max_acceleration;

  // Ramp up linear speed to steady-state so limiter history has non-zero values.
  for (int i = 0; i < static_cast<int>(std::floor(time_acc / dt)) + 5; ++i)
  {
    controller_->reference_interfaces_[0] = linear;
    controller_->reference_interfaces_[1] = 0.0;
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
      controller_interface::return_type::OK);
  }
  EXPECT_NEAR(linear / WHEELS_RADIUS_, joint_command_values_[CMD_TRACTION_RIGHT_WHEEL], 1e-3);
  EXPECT_NEAR(linear, controller_->previous_two_commands_.back()[0], 1e-3);

  // Deactivate then re-activate: limiter history must be reset to zero.
  ASSERT_TRUE(deactivate_succeeds(controller_));
  EXPECT_EQ(controller_->previous_two_commands_.front(), (std::array<double, 2>{{0.0, 0.0}}));
  EXPECT_EQ(controller_->previous_two_commands_.back(), (std::array<double, 2>{{0.0, 0.0}}));

  ASSERT_TRUE(activate_succeeds(controller_));
  EXPECT_EQ(controller_->previous_two_commands_.front(), (std::array<double, 2>{{0.0, 0.0}}));
  EXPECT_EQ(controller_->previous_two_commands_.back(), (std::array<double, 2>{{0.0, 0.0}}));

  // After reactivation, requesting the same target should be limited again
  // starting from zero, not pass through immediately.
  controller_->reference_interfaces_[0] = linear;
  controller_->reference_interfaces_[1] = 0.0;
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
    controller_interface::return_type::OK);
  EXPECT_LT(joint_command_values_[CMD_TRACTION_RIGHT_WHEEL], linear / WHEELS_RADIUS_)
    << "Limiter history was not reset across lifecycle transitions; the wheel command "
       "should be ramping up from zero again.";
}

TEST_F(SteeringControllersLibraryTest, test_speed_limiter_runtime_update)
{
  // If you set a linear velocity reference without acceleration limits,
  // then the wheel velocity command (rotations/s) will be:
  // ideal_wheel_velocity_command (rotations/s) = linear_velocity_command (m/s) / wheel_radius (m).
  // (The velocity command looks like a step function).
  // However, if there are acceleration limits, then the actual wheel velocity command
  // should always be less than the ideal velocity, and should only become
  // equal at time = linear_velocity_command (m/s) / acceleration_limit (m/s^2).
  const double max_acceleration_1 = 2.0;
  const double max_acceleration_2 = 5.0;
  const double max_deceleration = -4.0;

  auto node_options = controller_->define_custom_node_options();
  node_options.append_parameter_override(
    "linear.x.max_acceleration", rclcpp::ParameterValue(max_acceleration_1));
  node_options.append_parameter_override(
    "linear.x.max_deceleration", rclcpp::ParameterValue(max_deceleration));

  SetUpController("test_steering_controllers_library", node_options);

  ASSERT_TRUE(configure_succeeds(controller_));
  controller_->set_chained_mode(true);
  ASSERT_TRUE(activate_succeeds(controller_));
  ASSERT_TRUE(controller_->is_in_chained_mode());

  const double dt = 0.001;
  const double ideal_wheel_velocity = 1.0 / WHEELS_RADIUS_;

  auto wait_for_limiter = [&](double linear_ref, double expected_vel)
  {
    for (int i = 0; i < 3; ++i)
    {
      controller_->reference_interfaces_[0] = linear_ref;
      controller_->reference_interfaces_[1] = 0.0;
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
        controller_interface::return_type::OK);
      EXPECT_NEAR(expected_vel, joint_command_values_[CMD_TRACTION_RIGHT_WHEEL], 1e-3);
    }
  };

  // Wait for the speed limiter to fill the queue.
  wait_for_limiter(0.0, 0.0);

  // Phase 1: accelerate with max_acceleration = 2.0.
  {
    const double linear = 1.0;
    const double time_acc = linear / max_acceleration_1;
    for (int i = 0; i < static_cast<int>(std::floor(time_acc / dt)) - 1; ++i)
    {
      controller_->reference_interfaces_[0] = linear;
      controller_->reference_interfaces_[1] = 0.0;
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
        controller_interface::return_type::OK);
    }
    controller_->reference_interfaces_[0] = linear;
    controller_->reference_interfaces_[1] = 0.0;
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
      controller_interface::return_type::OK);
    EXPECT_NEAR(ideal_wheel_velocity, joint_command_values_[CMD_TRACTION_RIGHT_WHEEL], 1e-3);

    // Wait for the speed limiter to fill the queue.
    wait_for_limiter(linear, ideal_wheel_velocity);
  }

  // Stop the robot.
  {
    const double linear = 0.0;
    const double time_dec = 1.0 / std::abs(max_deceleration);
    for (int i = 0; i < static_cast<int>(std::floor(time_dec / dt)) - 1; ++i)
    {
      controller_->reference_interfaces_[0] = linear;
      controller_->reference_interfaces_[1] = 0.0;
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
        controller_interface::return_type::OK);
    }
    controller_->reference_interfaces_[0] = linear;
    controller_->reference_interfaces_[1] = 0.0;
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
      controller_interface::return_type::OK);
    EXPECT_NEAR(0.0, joint_command_values_[CMD_TRACTION_RIGHT_WHEEL], 1e-3);

    // Wait for the speed limiter to fill the queue.
    wait_for_limiter(linear, 0.0);
  }

  // Phase 2: update parameter at runtime to max_acceleration = 5.0.
  {
    auto result = controller_->get_node()->set_parameter(
      rclcpp::Parameter("linear.x.max_acceleration", rclcpp::ParameterValue(max_acceleration_2)));
    ASSERT_TRUE(result.successful);
  }

  // Phase 3: accelerate with max_acceleration = 5.0.
  {
    const double linear = 1.0;
    const double time_acc_1 = linear / max_acceleration_1;
    const double time_acc_2 = linear / max_acceleration_2;
    ASSERT_LT(time_acc_2, time_acc_1);
    for (int i = 0; i < static_cast<int>(std::floor(time_acc_2 / dt)) - 1; ++i)
    {
      controller_->reference_interfaces_[0] = linear;
      controller_->reference_interfaces_[1] = 0.0;
      ASSERT_EQ(
        controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
        controller_interface::return_type::OK);
    }
    controller_->reference_interfaces_[0] = linear;
    controller_->reference_interfaces_[1] = 0.0;
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(dt)),
      controller_interface::return_type::OK);
    EXPECT_NEAR(ideal_wheel_velocity, joint_command_values_[CMD_TRACTION_RIGHT_WHEEL], 1e-3);

    // Wait for the speed limiter to fill the queue.
    wait_for_limiter(linear, ideal_wheel_velocity);
  }
}

TEST_F(SteeringControllersLibraryTest, odometry_set_service)
{
  // 0. Initialize and Activate
  SetUpController();
  ASSERT_TRUE(configure_succeeds(controller_));
  controller_->get_node()->trigger_transition(
    rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

  controller_->set_chained_mode(true);
  ASSERT_TRUE(activate_succeeds(controller_));
  controller_->get_node()->trigger_transition(
    rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  ASSERT_EQ(
    controller_->get_node()->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  const double dt = 0.02;  // 50Hz update
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(dt);
  rclcpp::Time test_time = controller_->get_node()->now();

  auto move_robot = [&](double vx, double wz)
  {
    controller_->reference_interfaces_[0] = vx;  // linear velocity
    controller_->reference_interfaces_[1] = wz;  // angular velocity

    ASSERT_EQ(controller_->update(test_time, period), controller_interface::return_type::OK);

    // Update wheel positions based on commands to simulate feedback
    for (size_t i = 0; i < NR_CMD_ITFS; ++i)
    {
      joint_state_values_[i] = controller_->command_interfaces_[i].get_optional().value();
    }
    test_time += period;
  };

  // 1. Test robot movement initially
  for (int i = 0; i < 10; ++i) move_robot(1.0, 0.0);
  ASSERT_GT(controller_->odometry_.get_x(), 0.0);

  // 2. Call the odometry set service
  auto set_request = std::make_shared<control_msgs::srv::SetOdometry::Request>();
  auto set_response = std::make_shared<control_msgs::srv::SetOdometry::Response>();
  set_request->x = 5.0;
  set_request->y = -2.0;
  set_request->yaw = 1.57079632679;

  controller_->set_odometry(nullptr, set_request, set_response);
  EXPECT_TRUE(set_response->success);
  ASSERT_EQ(controller_->update(test_time, period), controller_interface::return_type::OK);

  // Validate the expected robot pose after service call
  EXPECT_NEAR(controller_->odometry_.get_x(), 5.0, 1e-6);
  EXPECT_NEAR(controller_->odometry_.get_y(), -2.0, 1e-6);
  EXPECT_NEAR(controller_->odometry_.get_heading(), 1.57079632679, 1e-5);

  // 3. Move forward again to verify
  double start_y = controller_->odometry_.get_y();
  for (int i = 0; i < 10; ++i) move_robot(1.0, 0.0);  // we are facing +Y now
  EXPECT_GT(controller_->odometry_.get_y(), start_y);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
