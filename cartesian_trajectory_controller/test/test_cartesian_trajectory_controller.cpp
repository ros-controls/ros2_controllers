// Copyright (c) 2026 ros2_control Development Team
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

// End-to-end test: drives the real controller through its real lifecycle and update() loop with
// real command/state interfaces (the test plays mock hardware: shared command/state storage, as
// mock_components/GenericSystem does), then checks the executed joints via forward kinematics.

#include <gmock/gmock.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "cartesian_trajectory_controller/cartesian_trajectory_controller.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_interface/test_utils.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/test_asset_6d_robot_description.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"

using namespace std::chrono_literals;

using controller_interface::activate_succeeds;
using controller_interface::configure_succeeds;
using controller_interface::deactivate_succeeds;

namespace
{
constexpr char BASE[] = "base_link";
constexpr char TIP[] = "tool0";
const std::vector<std::string> JOINTS = {"joint_a1", "joint_a2", "joint_a3",
                                         "joint_a4", "joint_a5", "joint_a6"};
// Folded, well-conditioned pose (KUKA KR6 "home" from the SRDF).
const std::vector<double> HOME = {0.0, -1.5708, 1.5708, 0.0, 1.5708, 0.0};

// Exposes the protected reference_callback for direct, deterministic injection and lets the fixture
// inject parameter overrides (mirrors the pattern used by the JTC tests).
class TestableController : public cartesian_trajectory_controller::CartesianTrajectoryController
{
public:
  using cartesian_trajectory_controller::CartesianTrajectoryController::build_joint_trajectory;
  using cartesian_trajectory_controller::CartesianTrajectoryController::reference_callback;

  void set_node_options(const rclcpp::NodeOptions & options) { node_options_ = options; }
  rclcpp::NodeOptions define_custom_node_options() const override { return node_options_; }

  // JTC's joint-space command inputs, disabled by on_configure (see the .reset() calls there).
  bool has_joint_command_subscriber() const { return joint_command_subscriber_ != nullptr; }
  bool has_action_server() const { return action_server_ != nullptr; }

  rclcpp::NodeOptions node_options_;
};
}  // namespace

class CartesianTrajectoryControllerTest : public ::testing::Test
{
public:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override { controller_ = std::make_unique<TestableController>(); }
  void TearDown() override { controller_.reset(); }

protected:
  void setup_controller()
  {
    const std::vector<rclcpp::Parameter> overrides = {
      {"joints", JOINTS},
      {"command_interfaces", std::vector<std::string>{"position"}},
      {"state_interfaces", std::vector<std::string>{"position"}},
      {"kinematics.plugin_name", std::string("kinematics_interface_kdl/KinematicsInterfaceKDL")},
      {"kinematics.plugin_package", std::string("kinematics_interface")},
      {"kinematics.base", std::string(BASE)},
      {"kinematics.tip", std::string(TIP)},
      {"kinematics.alpha", 0.01},
      {"max_cartesian_speed", 0.5},
      {"max_angular_speed", 1.0},
      {"resample_dt", 0.01}};

    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(overrides);
    controller_->set_node_options(node_options);

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_cartesian_trajectory_controller";
    params.robot_description = ros2_control_test_assets::valid_6d_robot_urdf;
    params.update_rate = 100;
    params.node_namespace = "";
    params.node_options = controller_->define_custom_node_options();
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
  }

  // Command and state share the same storage, so a commanded position is immediately reflected in
  // the state the next cycle - the behaviour of mock_components/GenericSystem for a position joint.
  void assign_interfaces()
  {
    joint_values_ = HOME;
    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    for (size_t i = 0; i < JOINTS.size(); ++i)
    {
      command_storage_.emplace_back(
        std::make_shared<hardware_interface::CommandInterface>(
          JOINTS[i], hardware_interface::HW_IF_POSITION, &joint_values_[i]));
      command_ifs.emplace_back(command_storage_.back(), nullptr);
      state_storage_.emplace_back(
        std::make_shared<hardware_interface::StateInterface>(
          JOINTS[i], hardware_interface::HW_IF_POSITION, &joint_values_[i]));
      state_ifs.emplace_back(state_storage_.back(), nullptr);
    }
    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void activate()
  {
    setup_controller();
    ASSERT_TRUE(configure_succeeds(controller_));
    assign_interfaces();
    ASSERT_TRUE(activate_succeeds(controller_));
    run(1);  // one hold cycle so state_current_ is read from the hardware before any target
  }

  void run(size_t cycles)
  {
    const auto period = rclcpp::Duration::from_seconds(0.01);
    for (size_t k = 0; k < cycles; ++k)
    {
      ASSERT_EQ(controller_->update(time_, period), controller_interface::return_type::OK);
      time_ = time_ + period;
    }
  }

  static trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr make_pose_msg(
    const std::string & frame_id, const Eigen::Vector3d & p, const Eigen::Quaterniond & q,
    double time_from_start)
  {
    auto msg = std::make_shared<trajectory_msgs::msg::MultiDOFJointTrajectory>();
    msg->header.frame_id = frame_id;
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
    geometry_msgs::msg::Transform tf;
    tf.translation.x = p.x();
    tf.translation.y = p.y();
    tf.translation.z = p.z();
    tf.rotation.x = q.x();
    tf.rotation.y = q.y();
    tf.rotation.z = q.z();
    tf.rotation.w = q.w();
    point.transforms.push_back(tf);
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
    msg->points.push_back(point);
    return msg;
  }

  // Multi-pose (action-chunk) message. Empty `times` leaves time_from_start unset (-> synthesized).
  static trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr make_multi_pose_msg(
    const std::string & frame_id,
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> & poses,
    const std::vector<double> & times)
  {
    auto msg = std::make_shared<trajectory_msgs::msg::MultiDOFJointTrajectory>();
    msg->header.frame_id = frame_id;
    for (size_t i = 0; i < poses.size(); ++i)
    {
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
      geometry_msgs::msg::Transform tf;
      tf.translation.x = poses[i].first.x();
      tf.translation.y = poses[i].first.y();
      tf.translation.z = poses[i].first.z();
      tf.rotation.x = poses[i].second.x();
      tf.rotation.y = poses[i].second.y();
      tf.rotation.z = poses[i].second.z();
      tf.rotation.w = poses[i].second.w();
      point.transforms.push_back(tf);
      if (!times.empty())
      {
        point.time_from_start = rclcpp::Duration::from_seconds(times[i]);
      }
      msg->points.push_back(point);
    }
    return msg;
  }

  // A pose offset from x0 by a translation and a rotation about the base Y axis.
  static std::pair<Eigen::Vector3d, Eigen::Quaterniond> offset_pose(
    const Eigen::Isometry3d & x0, const Eigen::Vector3d & dp, double dtheta)
  {
    return {
      x0.translation() + dp, Eigen::Quaterniond(x0.rotation()) *
                               Eigen::Quaterniond(Eigen::AngleAxisd(dtheta, Eigen::Vector3d::UnitY()))};
  }

  // Independent FK (same KDL plugin) used to build the target and to check the executed joints.
  Eigen::Isometry3d fk(const std::vector<double> & q)
  {
    if (!fk_kinematics_)
    {
      fk_node_ = std::make_shared<rclcpp::Node>("fk_helper");
      fk_node_->declare_parameter("kinematics.alpha", 0.01);
      fk_node_->declare_parameter("kinematics.base", std::string(BASE));
      fk_node_->declare_parameter("kinematics.tip", std::string(TIP));
      fk_loader_ =
        std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
          "kinematics_interface", "kinematics_interface::KinematicsInterface");
      fk_kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
        fk_loader_->createUnmanagedInstance("kinematics_interface_kdl/KinematicsInterfaceKDL"));
      fk_kinematics_->initialize(
        ros2_control_test_assets::valid_6d_robot_urdf, fk_node_->get_node_parameters_interface(),
        "kinematics");
    }
    const Eigen::VectorXd qv = Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
    Eigen::Isometry3d x;
    EXPECT_TRUE(fk_kinematics_->calculate_link_transform(qv, TIP, x));
    return x;
  }

  double joint_travel() const
  {
    double d = 0.0;
    for (size_t i = 0; i < HOME.size(); ++i)
    {
      d += std::abs(joint_values_[i] - HOME[i]);
    }
    return d;
  }

  std::unique_ptr<TestableController> controller_;
  std::vector<double> joint_values_;
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> command_storage_;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> state_storage_;
  rclcpp::Time time_{0, 0, RCL_ROS_TIME};

  rclcpp::Node::SharedPtr fk_node_;
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> fk_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> fk_kinematics_;
};

// The full pipeline: a Cartesian pose target is turned into joint motion whose FK reaches the pose.
TEST_F(CartesianTrajectoryControllerTest, tracks_cartesian_pose_end_to_end)
{
  activate();
  ASSERT_LT(joint_travel(), 1e-6);  // still holding home before any target

  const Eigen::Isometry3d x0 = fk(HOME);
  const Eigen::Vector3d target_p = x0.translation() + Eigen::Vector3d(0.03, 0.02, -0.02);
  const Eigen::Quaterniond target_q =
    Eigen::Quaterniond(x0.rotation()) *
    Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitY()));

  controller_->reference_callback(make_pose_msg(BASE, target_p, target_q, 1.0));
  run(200);  // 1.0 s trajectory at 100 Hz, plus margin to settle on the final hold

  EXPECT_GT(joint_travel(), 0.01) << "joints never moved: the message was dropped";

  const Eigen::Isometry3d xf = fk(joint_values_);
  EXPECT_LT((xf.translation() - target_p).norm(), 0.01)
    << "EE position did not reach the commanded pose";
  EXPECT_LT(Eigen::Quaterniond(xf.rotation()).angularDistance(target_q), 0.05)
    << "EE orientation did not reach the commanded pose";
}

// A target expressed in an unsupported frame is rejected and produces no motion.
TEST_F(CartesianTrajectoryControllerTest, rejects_target_in_wrong_frame)
{
  activate();

  const Eigen::Isometry3d x0 = fk(HOME);
  controller_->reference_callback(make_pose_msg(
    "unsupported_frame", x0.translation() + Eigen::Vector3d(0.05, 0.0, 0.0),
    Eigen::Quaterniond(x0.rotation()), 1.0));
  run(50);

  EXPECT_LT(joint_travel(), 1e-6) << "motion produced for a target in the wrong frame";
}

// A multi-pose action chunk is traced: the EE passes through an intermediate waypoint and reaches
// the final one.
TEST_F(CartesianTrajectoryControllerTest, tracks_multi_point_chunk)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  const auto p1 = offset_pose(x0, {0.02, 0.01, -0.01}, 0.05);
  const auto p2 = offset_pose(x0, {0.04, 0.02, -0.02}, 0.10);
  const auto p3 = offset_pose(x0, {0.06, 0.03, -0.03}, 0.15);
  controller_->reference_callback(make_multi_pose_msg(BASE, {p1, p2, p3}, {1.0, 2.0, 3.0}));

  run(200);  // reach waypoint 2's time (~2.0 s)
  EXPECT_LT((fk(joint_values_).translation() - p2.first).norm(), 0.015)
    << "did not pass through the intermediate waypoint";

  run(150);  // past waypoint 3 (~3.0 s) + settle
  const Eigen::Isometry3d xf = fk(joint_values_);
  EXPECT_GT(joint_travel(), 0.01);
  EXPECT_LT((xf.translation() - p3.first).norm(), 0.01) << "did not reach the final waypoint";
  EXPECT_LT(Eigen::Quaterniond(xf.rotation()).angularDistance(p3.second), 0.05);
}

// A single-pose chunk with no time_from_start executes over a speed-synthesized duration.
TEST_F(CartesianTrajectoryControllerTest, synthesizes_timing_when_absent)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  const auto tgt = offset_pose(x0, {0.03, 0.02, -0.02}, 0.15);
  controller_->reference_callback(make_pose_msg(BASE, tgt.first, tgt.second, 0.0));  // no timing

  run(300);
  EXPECT_GT(joint_travel(), 0.01);
  const Eigen::Isometry3d xf = fk(joint_values_);
  EXPECT_LT((xf.translation() - tgt.first).norm(), 0.01);
  EXPECT_LT(Eigen::Quaterniond(xf.rotation()).angularDistance(tgt.second), 0.05);
}

// A multi-pose chunk with no timing synthesizes per-segment durations and reaches the final pose.
TEST_F(CartesianTrajectoryControllerTest, multi_point_synthesized_timing)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  const auto p1 = offset_pose(x0, {0.02, 0.01, 0.0}, 0.05);
  const auto p2 = offset_pose(x0, {0.04, 0.02, -0.02}, 0.10);
  controller_->reference_callback(make_multi_pose_msg(BASE, {p1, p2}, {}));

  run(300);
  EXPECT_GT(joint_travel(), 0.01);
  EXPECT_LT((fk(joint_values_).translation() - p2.first).norm(), 0.01);
}

// on_configure disables JTC's joint-space command inputs (Sai's requirement).
TEST_F(CartesianTrajectoryControllerTest, jtc_command_inputs_disabled)
{
  setup_controller();
  ASSERT_TRUE(configure_succeeds(controller_));
  EXPECT_FALSE(controller_->has_joint_command_subscriber());
  EXPECT_FALSE(controller_->has_action_server());
}

// A message with no points is dropped.
TEST_F(CartesianTrajectoryControllerTest, rejects_empty_message)
{
  activate();
  auto msg = std::make_shared<trajectory_msgs::msg::MultiDOFJointTrajectory>();
  msg->header.frame_id = BASE;
  controller_->reference_callback(msg);
  run(50);
  EXPECT_LT(joint_travel(), 1e-6);
}

// A point that carries no transform is dropped.
TEST_F(CartesianTrajectoryControllerTest, rejects_point_without_transform)
{
  activate();
  auto msg = std::make_shared<trajectory_msgs::msg::MultiDOFJointTrajectory>();
  msg->header.frame_id = BASE;
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(1.0);
  msg->points.push_back(point);
  controller_->reference_callback(msg);
  run(50);
  EXPECT_LT(joint_travel(), 1e-6);
}

// An empty frame_id is treated as the base frame and accepted.
TEST_F(CartesianTrajectoryControllerTest, accepts_empty_frame_id)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  const auto tgt = offset_pose(x0, {0.03, 0.02, -0.02}, 0.10);
  controller_->reference_callback(make_pose_msg("", tgt.first, tgt.second, 1.0));
  run(200);
  EXPECT_GT(joint_travel(), 0.01);
  EXPECT_LT((fk(joint_values_).translation() - tgt.first).norm(), 0.01);
}

// A target received while the controller is not active is ignored (no motion once activated).
TEST_F(CartesianTrajectoryControllerTest, ignored_when_not_active)
{
  setup_controller();
  ASSERT_TRUE(configure_succeeds(controller_));
  assign_interfaces();  // INACTIVE, joints at HOME

  const Eigen::Isometry3d x0 = fk(HOME);
  const auto tgt = offset_pose(x0, {0.05, 0.0, 0.0}, 0.0);
  controller_->reference_callback(make_pose_msg(BASE, tgt.first, tgt.second, 1.0));

  ASSERT_TRUE(activate_succeeds(controller_));
  run(200);
  EXPECT_LT(joint_travel(), 1e-6) << "a target received while inactive was executed";
}

// The controller keeps working after a deactivate/activate cycle.
TEST_F(CartesianTrajectoryControllerTest, deactivate_and_reactivate)
{
  activate();
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_TRUE(activate_succeeds(controller_));
  run(1);

  const Eigen::Isometry3d x0 = fk(joint_values_);
  const auto tgt = offset_pose(x0, {0.03, 0.02, -0.02}, 0.10);
  controller_->reference_callback(make_pose_msg(BASE, tgt.first, tgt.second, 1.0));
  run(200);
  EXPECT_GT(joint_travel(), 0.01);
  EXPECT_LT((fk(joint_values_).translation() - tgt.first).norm(), 0.01);
}

// A new chunk (immediate) replaces one still executing.
TEST_F(CartesianTrajectoryControllerTest, new_chunk_replaces_previous)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  const auto tgt_a = offset_pose(x0, {0.06, 0.0, 0.0}, 0.0);
  const auto tgt_b = offset_pose(x0, {0.0, 0.05, -0.03}, 0.15);

  controller_->reference_callback(make_pose_msg(BASE, tgt_a.first, tgt_a.second, 2.0));
  run(50);  // partway through chunk A
  controller_->reference_callback(make_pose_msg(BASE, tgt_b.first, tgt_b.second, 1.0));
  run(200);

  const Eigen::Isometry3d xf = fk(joint_values_);
  EXPECT_LT((xf.translation() - tgt_b.first).norm(), 0.01) << "did not end at chunk B";
  EXPECT_GT((xf.translation() - tgt_a.first).norm(), 0.02) << "ended at chunk A instead of B";
}

// The inherited ~/controller_state feedback publisher is kept and publishes.
TEST_F(CartesianTrajectoryControllerTest, publishes_controller_state)
{
  activate();
  auto listener = std::make_shared<rclcpp::Node>("state_listener");
  control_msgs::msg::JointTrajectoryControllerState::SharedPtr received;
  auto sub = listener->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/test_cartesian_trajectory_controller/controller_state", rclcpp::SystemDefaultsQoS(),
    [&](control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) { received = msg; });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);

  for (int i = 0; i < 200 && !received; ++i)
  {
    run(1);
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(received) << "controller_state was not published";
}

// build_joint_trajectory carries the incoming header.stamp through, so JTC's deferred-start works.
TEST_F(CartesianTrajectoryControllerTest, preserves_header_stamp)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  const auto tgt = offset_pose(x0, {0.03, 0.02, -0.02}, 0.10);
  auto msg = make_pose_msg(BASE, tgt.first, tgt.second, 1.0);
  msg->header.stamp.sec = 123;
  msg->header.stamp.nanosec = 456u;

  trajectory_msgs::msg::JointTrajectory joint_traj;
  ASSERT_TRUE(controller_->build_joint_trajectory(*msg, joint_traj));
  EXPECT_EQ(joint_traj.header.stamp.sec, 123);
  EXPECT_EQ(joint_traj.header.stamp.nanosec, 456u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
