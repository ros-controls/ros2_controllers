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
//
/// \author: Denis Stogl

#ifndef TEST_ADMITTANCE_CONTROLLER_HPP_
#define TEST_ADMITTANCE_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "gmock/gmock.h"

#include "admittance_controller/admittance_controller.hpp"
#include "control_msgs/msg/admittance_controller_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "ros2_control_test_assets/6d_robot_description.hpp"
#include "semantic_components/force_torque_sensor.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tf2_ros/transform_broadcaster.h"

// TODO(anyone): replace the state and command message types
using ControllerCommandWrenchMsg = geometry_msgs::msg::WrenchStamped;
using ControllerCommandPoseMsg = geometry_msgs::msg::PoseStamped;
using ControllerCommandJointMsg = trajectory_msgs::msg::JointTrajectory;
using ControllerStateMsg = control_msgs::msg::AdmittanceControllerState;

namespace
{
const double COMMON_THRESHOLD = 0.001;

constexpr auto NODE_SUCCESS =
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR =
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}

}  // namespace

// subclassing and friending so we can access member varibles
class TestableAdmittanceController
: public admittance_controller::AdmittanceController
{
  FRIEND_TEST(AdmittanceControllerTest, joint_names_parameter_not_set);
  FRIEND_TEST(AdmittanceControllerTest, interface_parameter_not_set);
  FRIEND_TEST(AdmittanceControllerTest, all_parameters_set_configure_success);
  FRIEND_TEST(AdmittanceControllerTest, check_interfaces);
  FRIEND_TEST(AdmittanceControllerTest, activate_success);
  FRIEND_TEST(AdmittanceControllerTest, receive_message_and_publish_updated_status);

public:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret =
    admittance_controller::AdmittanceController::on_configure(previous_state);
    // Only if on_configure is successful create subscription
    if (ret == CallbackReturn::SUCCESS) {
      if (admittance_->unified_mode_) {
        input_wrench_command_subscriber_wait_set_.add_subscription(input_wrench_command_subscriber_);
      }
      input_pose_command_subscriber_wait_set_.add_subscription(input_pose_command_subscriber_);
    }
    return ret;
  }

  /**
   * @brief wait_for_commands blocks until a new ControllerCommandMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   *
   * @return true if new ControllerCommandMsg msg was received, false if timeout.
   */
  bool wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds {500})
  {
    bool success = input_pose_command_subscriber_wait_set_.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;

    if (admittance_->unified_mode_) {
      success = success && input_wrench_command_subscriber_wait_set_.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
    }
    if (success) {
      executor.spin_some();
    }
    return success;
  }

private:
  rclcpp::WaitSet input_wrench_command_subscriber_wait_set_;
  rclcpp::WaitSet input_pose_command_subscriber_wait_set_;
};

class AdmittanceControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<TestableAdmittanceController>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    force_command_publisher_ = command_publisher_node_->create_publisher<ControllerCommandWrenchMsg>(
      "/test_admittance_controller/force_commands", rclcpp::SystemDefaultsQoS());
    pose_command_publisher_ = command_publisher_node_->create_publisher<ControllerCommandPoseMsg>(
      "/test_admittance_controller/pose_commands", rclcpp::SystemDefaultsQoS());
    joint_command_publisher_ = command_publisher_node_->create_publisher<ControllerCommandJointMsg>(
      "/test_admittance_controller/joint_commands", rclcpp::SystemDefaultsQoS());

    test_subscription_node_ = std::make_shared<rclcpp::Node>("test_subscription_node");
    test_broadcaster_node_ = std::make_shared<rclcpp::Node>("test_broadcaster_node");
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void TearDown()
  {
    controller_.reset(nullptr);
  }

protected:
  void SetUpController(bool set_parameters = true, bool use_joint_commands_as_input = false)
  {
    const auto result = controller_->init("test_admittance_controller");
    ASSERT_EQ(result, controller_interface::return_type::OK);

    assign_interfaces();

    if (set_parameters) {
      controller_->get_node()->set_parameter({"use_joint_commands_as_input", use_joint_commands_as_input});

      controller_->get_node()->set_parameter({"joints", joint_names_});
      controller_->get_node()->set_parameter({"command_interfaces", command_interface_types_});
      controller_->get_node()->set_parameter({"state_interfaces", state_interface_types_});
      controller_->get_node()->set_parameter({"ft_sensor_name", ft_sensor_name_});
      controller_->get_node()->set_parameter({"hardware_state_has_offset", hardware_state_has_offset_});

      controller_->get_node()->set_parameter({"IK.base", ik_base_frame_});
      controller_->get_node()->set_parameter({"IK.tip", ik_tip_frame_});
      // TODO(destogl): enable when IK support is added
//       controller_->get_node()->set_parameter({"IK.plugin", ik_group_name_});
      controller_->get_node()->set_parameter({"IK.group_name", ik_group_name_});
      controller_->get_node()->set_parameter({"robot_description", robot_description_});
      controller_->get_node()->set_parameter({"robot_description_semantic", robot_description_semantic_});

      controller_->get_node()->set_parameter({"control_frame", control_frame_});
      controller_->get_node()->set_parameter({"endeffector_frame", endeffector_frame_});
      controller_->get_node()->set_parameter({"fixed_world_frame", fixed_world_frame_});
      controller_->get_node()->set_parameter({"sensor_frame", sensor_frame_});

      controller_->get_node()->set_parameter({"admittance.selected_axes.x", admittance_selected_axes_[0]});
      controller_->get_node()->set_parameter({"admittance.selected_axes.y", admittance_selected_axes_[1]});
      controller_->get_node()->set_parameter({"admittance.selected_axes.z", admittance_selected_axes_[2]});
      controller_->get_node()->set_parameter({"admittance.selected_axes.rx", admittance_selected_axes_[3]});
      controller_->get_node()->set_parameter({"admittance.selected_axes.ry", admittance_selected_axes_[4]});
      controller_->get_node()->set_parameter({"admittance.selected_axes.rz", admittance_selected_axes_[5]});

      controller_->get_node()->set_parameter({"admittance.mass.x", admittance_mass_[0]});
      controller_->get_node()->set_parameter({"admittance.mass.y", admittance_mass_[1]});
      controller_->get_node()->set_parameter({"admittance.mass.z", admittance_mass_[2]});
      controller_->get_node()->set_parameter({"admittance.mass.rx", admittance_mass_[3]});
      controller_->get_node()->set_parameter({"admittance.mass.ry", admittance_mass_[4]});
      controller_->get_node()->set_parameter({"admittance.mass.rz", admittance_mass_[5]});

      controller_->get_node()->set_parameter({"admittance.damping.x", admittance_damping_[0]});
      controller_->get_node()->set_parameter({"admittance.damping.y", admittance_damping_[1]});
      controller_->get_node()->set_parameter({"admittance.damping.z", admittance_damping_[2]});
      controller_->get_node()->set_parameter({"admittance.damping.rx", admittance_damping_[3]});
      controller_->get_node()->set_parameter({"admittance.damping.ry", admittance_damping_[4]});
      controller_->get_node()->set_parameter({"admittance.damping.rz", admittance_damping_[5]});

      controller_->get_node()->set_parameter({"admittance.stiffness.x", admittance_stiffness_[0]});
      controller_->get_node()->set_parameter({"admittance.stiffness.y", admittance_stiffness_[1]});
      controller_->get_node()->set_parameter({"admittance.stiffness.z", admittance_stiffness_[2]});
      controller_->get_node()->set_parameter({"admittance.stiffness.rx", admittance_stiffness_[3]});
      controller_->get_node()->set_parameter({"admittance.stiffness.ry", admittance_stiffness_[4]});
      controller_->get_node()->set_parameter({"admittance.stiffness.rz", admittance_stiffness_[5]});

    }
  }

  void assign_interfaces()
  {
    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    for (auto i = 0u; i < joint_command_values_.size(); ++i) {
      command_itfs_.emplace_back(
        hardware_interface::CommandInterface(joint_names_[i], command_interface_types_[0], &joint_command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    auto sc_fts = semantic_components::ForceTorqueSensor(ft_sensor_name_);
    fts_state_names_ = sc_fts.get_state_interface_names();
    std::vector<hardware_interface::LoanedStateInterface> state_ifs;

    const size_t num_state_ifs = joint_state_values_.size() + fts_state_names_.size();
    state_itfs_.reserve(num_state_ifs);
    state_ifs.reserve(num_state_ifs);

    for (auto i = 0u; i < joint_state_values_.size(); ++i) {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface(joint_names_[i], state_interface_types_[0], &joint_state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    std::vector<std::string> fts_itf_names = {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};

    for (auto i = 0u; i < fts_state_names_.size(); ++i) {
      state_itfs_.emplace_back(
        hardware_interface::StateInterface(ft_sensor_name_, fts_itf_names[i], &fts_state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void broadcast_tfs()
  {
    static tf2_ros::TransformBroadcaster br(test_broadcaster_node_);
    geometry_msgs::msg::TransformStamped transform_stamped;

    transform_stamped.header.stamp = test_broadcaster_node_->now();
    transform_stamped.header.frame_id = fixed_world_frame_;
    transform_stamped.transform.translation.x = 1.3;
    transform_stamped.transform.translation.y = 0.5;
    transform_stamped.transform.translation.z = 0.5;
    transform_stamped.transform.rotation.x = 0;
    transform_stamped.transform.rotation.y = 0;
    transform_stamped.transform.rotation.z = 0;
    transform_stamped.transform.rotation.w = 1;

    transform_stamped.child_frame_id = ik_base_frame_;
    br.sendTransform(transform_stamped);

    transform_stamped.child_frame_id = ik_tip_frame_;
    br.sendTransform(transform_stamped);

    transform_stamped.header.frame_id = ik_tip_frame_;
    transform_stamped.transform.translation.x = 0;
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.translation.z = 0;
    transform_stamped.transform.rotation.x = 0;
    transform_stamped.transform.rotation.y = 0;
    transform_stamped.transform.rotation.z = 0;
    transform_stamped.transform.rotation.w = 1;

    transform_stamped.child_frame_id = control_frame_;
    br.sendTransform(transform_stamped);

    transform_stamped.transform.translation.z = 0.05;
    transform_stamped.child_frame_id = sensor_frame_;
    br.sendTransform(transform_stamped);

    transform_stamped.transform.translation.z = 0.2;
    transform_stamped.child_frame_id = endeffector_frame_;
    br.sendTransform(transform_stamped);
  }

  void subscribe_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr)
    {
    };
    auto subscription =
    test_subscription_node_->create_subscription<ControllerStateMsg>(
      "/test_admittance_controller/state", 10, subs_callback);

    // call update to publish the test value
    ASSERT_EQ(controller_->update(), controller_interface::return_type::OK);

    // wait for message to be passed
    ASSERT_EQ(wait_for(subscription), rclcpp::WaitResultKind::Ready);

    // take message from subscription
    rclcpp::MessageInfo msg_info;
    ASSERT_TRUE(subscription->take(msg, msg_info));
  }

  void publish_commands()
  {
    auto wait_for_topic = [&](const auto topic_name)
    {
      size_t wait_count = 0;
      while (command_publisher_node_->count_subscribers(topic_name) == 0) {
        if (wait_count >= 5) {
          auto error_msg =
          std::string("publishing to ") + topic_name + " but no node subscribes to it";
          throw std::runtime_error(error_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++wait_count;
      }
    };

    // TODO(destogl): comment in when using unified mode
//     if (controller_->admittance_->unified_mode_) {
//       wait_for_topic(force_command_publisher_->get_topic_name());
//     }

    wait_for_topic(pose_command_publisher_->get_topic_name());

    ControllerCommandWrenchMsg force_msg;
    force_msg.header.frame_id = sensor_frame_;
    force_msg.wrench.force.x = 0.1;
    force_msg.wrench.force.y = 0.2;
    force_msg.wrench.force.z = 0.3;
    force_msg.wrench.torque.x = 3.1;
    force_msg.wrench.torque.y = 3.2;
    force_msg.wrench.torque.z = 3.3;

    ControllerCommandPoseMsg pose_msg;
    pose_msg.header.frame_id = endeffector_frame_;
    pose_msg.pose.position.x = 0.1;
    pose_msg.pose.position.y = 0.2;
    pose_msg.pose.position.z = 0.3;
    pose_msg.pose.orientation.x = 0;
    pose_msg.pose.orientation.y = 0;
    pose_msg.pose.orientation.z = 0;
    pose_msg.pose.orientation.w = 1;

    // TODO(destogl): comment in when using unified mode
//     if (controller_->admittance_->unified_mode_) {
//       force_command_publisher_->publish(force_msg);
//     }
    pose_command_publisher_->publish(pose_msg);

    wait_for_topic(joint_command_publisher_->get_topic_name());

    ControllerCommandJointMsg joint_msg;
    joint_msg.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
    auto num_joints = joint_names_.size();
    trajectory_point.positions.reserve(num_joints);
    trajectory_point.velocities.resize(num_joints, 0.0);
    for (auto index = 0u; index < num_joints; ++index) {
      trajectory_point.positions.emplace_back(joint_state_values_[index]);
    }
    joint_msg.points.emplace_back(trajectory_point);

    joint_command_publisher_->publish(joint_msg);
  }

protected:
  // TODO(anyone): adjust the members as needed

  // Controller-related parameters
  const std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  const std::vector<std::string> command_interface_types_ = {"position"};
  const std::vector<std::string> state_interface_types_ = {"position"};
  const std::string ft_sensor_name_ = "ft_sensor_name";

  bool hardware_state_has_offset_ = false;

  const std::string ik_base_frame_ = "base_link";
  const std::string ik_tip_frame_ = "tool0";
  const std::string ik_group_name_ = "arm";
  const std::string robot_description_ = ros2_control_test_assets::valid_6d_robot_urdf;
  const std::string robot_description_semantic_ = ros2_control_test_assets::valid_6d_robot_srdf;

  const std::string control_frame_ = "control_frame";
  const std::string endeffector_frame_ = "endeffector_frame";
  const std::string fixed_world_frame_ = "fixed_world_frame";
  const std::string sensor_frame_ = "sensor_frame";

  std::array<bool, 6> admittance_selected_axes_ = {true, true, true, true, true, true};
  std::array<double, 6> admittance_mass_ = {5.5, 6.6, 7.7, 8.8, 9.9, 10.10};
  std::array<double, 6> admittance_damping_ = {100.1, 100.2, 100.3, 100.4, 100.5, 100.6};
  std::array<double, 6> admittance_stiffness_ = {214.1, 214.2, 214.3, 214.4, 214.5, 214.6};

  std::array<double, 6> joint_command_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 6> joint_state_values_ = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6};
  std::array<double, 6> fts_state_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<std::string> fts_state_names_;

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<TestableAdmittanceController> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerCommandWrenchMsg>::SharedPtr force_command_publisher_;
  rclcpp::Publisher<ControllerCommandPoseMsg>::SharedPtr pose_command_publisher_;
  rclcpp::Publisher<ControllerCommandJointMsg>::SharedPtr joint_command_publisher_;
  rclcpp::Node::SharedPtr test_subscription_node_;
  rclcpp::Node::SharedPtr test_broadcaster_node_;
};

// From the tutorial: https://www.sandordargo.com/blog/2019/04/24/parameterized-testing-with-gtest
class AdmittanceControllerTestParameterizedParameters
: public AdmittanceControllerTest,
public ::testing::WithParamInterface<std::tuple<std::string, rclcpp::ParameterValue>>
{
public:
  virtual void SetUp()
  {
    AdmittanceControllerTest::SetUp();
  }

  static void TearDownTestCase()
  {
    AdmittanceControllerTest::TearDownTestCase();
  }

protected:
  void SetUpController(bool set_parameters = true)
  {
    AdmittanceControllerTest::SetUpController(set_parameters);
    controller_->get_node()->undeclare_parameter(std::get<0>(GetParam()));
    controller_->get_node()->declare_parameter(std::get<0>(GetParam()), std::get<1>(GetParam()));
  }

};

#endif  // TEST_ADMITTANCE_CONTROLLER_HPP_
