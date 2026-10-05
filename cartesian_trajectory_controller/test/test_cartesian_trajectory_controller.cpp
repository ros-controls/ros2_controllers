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

// Runs the controller through its lifecycle and update() loop on mock position joints, and checks
// the executed motion by forward kinematics.

#include <gmock/gmock.h>

#include <chrono>
#include <cmath>
#include <limits>
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

// Largest acceleration mismatch at any interior knot.
double max_knot_acceleration_jump(const trajectory_msgs::msg::JointTrajectory & traj)
{
  double worst = 0.0;
  for (size_t k = 1; k + 1 < traj.points.size(); ++k)
  {
    const auto & a = traj.points[k - 1];
    const auto & b = traj.points[k];
    const auto & c = traj.points[k + 1];
    const double h1 =
      rclcpp::Duration(b.time_from_start).seconds() - rclcpp::Duration(a.time_from_start).seconds();
    const double h2 =
      rclcpp::Duration(c.time_from_start).seconds() - rclcpp::Duration(b.time_from_start).seconds();
    for (size_t j = 0; j < b.positions.size(); ++j)
    {
      const double end_of_left = -6.0 * (b.positions[j] - a.positions[j]) / (h1 * h1) +
                                 (2.0 * a.velocities[j] + 4.0 * b.velocities[j]) / h1;
      const double start_of_right = 6.0 * (c.positions[j] - b.positions[j]) / (h2 * h2) -
                                    (4.0 * b.velocities[j] + 2.0 * c.velocities[j]) / h2;
      worst = std::max(worst, std::abs(end_of_left - start_of_right));
    }
  }
  return worst;
}

// Exposes the protected callbacks and allows parameter overrides, as in the JTC tests.
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
  bool is_holding() const { return rt_is_holding_; }

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
  void setup_controller(double alpha = 0.01)
  {
    const std::vector<rclcpp::Parameter> overrides = {
      {"joints", JOINTS},
      {"command_interfaces", std::vector<std::string>{"position"}},
      {"state_interfaces", std::vector<std::string>{"position"}},
      {"kinematics.plugin_name", std::string("kinematics_interface_kdl/KinematicsInterfaceKDL")},
      {"kinematics.plugin_package", std::string("kinematics_interface")},
      {"kinematics.base", std::string(BASE)},
      {"kinematics.tip", std::string(TIP)},
      {"kinematics.alpha", alpha},
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

  void activate(double alpha = 0.01)
  {
    setup_controller(alpha);
    ASSERT_TRUE(configure_succeeds(controller_));
    assign_interfaces();
    ASSERT_TRUE(activate_succeeds(controller_));
    run(1);  // one hold cycle so state_current_ is read from the hardware before any target
  }

  // Run the given number of cycles, appending the joint positions after each one.
  void record(size_t cycles, std::vector<std::vector<double>> & q)
  {
    for (size_t k = 0; k < cycles; ++k)
    {
      run(1);
      q.push_back(joint_values_);
    }
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
    return make_multi_pose_msg(frame_id, {{p, q}}, {time_from_start});
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

  // A pose offset from x0 by a translation and a rotation about its own Y axis.
  static std::pair<Eigen::Vector3d, Eigen::Quaterniond> offset_pose(
    const Eigen::Isometry3d & x0, const Eigen::Vector3d & dp, double dtheta)
  {
    return {
      x0.translation() + dp,
      Eigen::Quaterniond(x0.rotation()) *
        Eigen::Quaterniond(Eigen::AngleAxisd(dtheta, Eigen::Vector3d::UnitY()))};
  }

  // Pose at progress s in [0, 1] along a sine, rotating with progress.
  static std::pair<Eigen::Vector3d, Eigen::Quaterniond> sine_pose(
    const Eigen::Isometry3d & x0, double s)
  {
    return offset_pose(x0, {0.25 * s, 0.06 * s, 0.04 * std::sin(2.0 * M_PI * s)}, 0.5 * s);
  }

  static trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr make_sine_msg(
    const Eigen::Isometry3d & x0, int first, double time_offset)
  {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> poses;
    std::vector<double> times;
    for (int i = first; i <= 20; ++i)
    {
      poses.push_back(sine_pose(x0, i / 20.0));
      times.push_back(0.2 * i - time_offset);
    }
    return make_multi_pose_msg(BASE, poses, times);
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

  static double cycle_velocity(const std::vector<std::vector<double>> & q, size_t k, size_t j)
  {
    return (q[k][j] - q[k - 1][j]) / 0.01;
  }

  // Velocity may change at the handoff only as much as it did per cycle just before it.
  static void expect_velocity_continuous_at(const std::vector<std::vector<double>> & q, size_t seam)
  {
    double max_speed = 0.0;
    for (size_t j = 0; j < JOINTS.size(); ++j)
    {
      max_speed = std::max(max_speed, std::abs(cycle_velocity(q, seam - 1, j)));
    }
    ASSERT_GT(max_speed, 5e-2) << "the arm must be moving at the handoff";
    for (size_t j = 0; j < JOINTS.size(); ++j)
    {
      double max_change_before = 0.0;
      for (size_t k = seam - 20; k < seam; ++k)
      {
        max_change_before = std::max(
          max_change_before, std::abs(cycle_velocity(q, k, j) - cycle_velocity(q, k - 1, j)));
      }
      const double change_at_seam =
        std::abs(cycle_velocity(q, seam, j) - cycle_velocity(q, seam - 1, j));
      EXPECT_LE(change_at_seam, 2.0 * max_change_before + 1e-4)
        << JOINTS[j] << " velocity jumps at the handoff";
    }
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

// A long straight move with rotation stays on the line (joint interpolation bows ~12 mm off it).
TEST_F(CartesianTrajectoryControllerTest, tracks_straight_line_with_rotation)
{
  activate();
  ASSERT_LT(joint_travel(), 1e-6);  // still holding home before any target

  const Eigen::Isometry3d x0 = fk(HOME);
  const Eigen::Quaterniond q0(x0.rotation());
  const auto tgt = offset_pose(x0, {0.20, 0.0, -0.10}, 0.4);
  const Eigen::Vector3d line = tgt.first - x0.translation();
  controller_->reference_callback(make_pose_msg(BASE, tgt.first, tgt.second, 2.0));

  double max_line_deviation = 0.0;
  double max_orientation_error = 0.0;
  for (int i = 0; i < 250; ++i)
  {
    run(1);
    const Eigen::Isometry3d x = fk(joint_values_);
    const Eigen::Vector3d r = x.translation() - x0.translation();
    const double progress = r.dot(line) / line.squaredNorm();
    max_line_deviation = std::max(max_line_deviation, (r - progress * line).norm());
    const Eigen::Quaterniond expected =
      q0 * Eigen::Quaterniond(Eigen::AngleAxisd(0.4 * progress, Eigen::Vector3d::UnitY()));
    max_orientation_error =
      std::max(max_orientation_error, Eigen::Quaterniond(x.rotation()).angularDistance(expected));
  }
  EXPECT_LT(max_line_deviation, 1e-3) << "TCP left the straight line";
  EXPECT_LT(max_orientation_error, 1e-2) << "rotation did not progress with the translation";

  const Eigen::Isometry3d xf = fk(joint_values_);
  EXPECT_LT((xf.translation() - tgt.first).norm(), 1e-3);
  EXPECT_LT(Eigen::Quaterniond(xf.rotation()).angularDistance(tgt.second), 1e-3);
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

// A curved chunk with rotation is traced through its waypoints.
TEST_F(CartesianTrajectoryControllerTest, tracks_curved_chunk_with_rotation)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  controller_->reference_callback(make_sine_msg(x0, 1, 0.0));

  std::vector<Eigen::Vector3d> curve;
  for (int j = 0; j <= 2000; ++j)
  {
    curve.push_back(sine_pose(x0, j / 2000.0).first);
  }
  const auto distance_to_curve = [&curve](const Eigen::Vector3d & p)
  {
    double d = std::numeric_limits<double>::infinity();
    for (const auto & c : curve)
    {
      d = std::min(d, (p - c).norm());
    }
    return d;
  };

  const auto quarter = sine_pose(x0, 0.25);
  const auto half = sine_pose(x0, 0.5);
  double max_path_deviation = 0.0;
  double max_orientation_error = 0.0;
  double closest_to_quarter = std::numeric_limits<double>::infinity();
  double closest_to_half = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 450; ++i)
  {
    run(1);
    const Eigen::Isometry3d x = fk(joint_values_);
    max_path_deviation = std::max(max_path_deviation, distance_to_curve(x.translation()));
    const double progress = (x.translation() - x0.translation()).x() / 0.25;
    max_orientation_error = std::max(
      max_orientation_error,
      Eigen::Quaterniond(x.rotation()).angularDistance(sine_pose(x0, progress).second));
    closest_to_quarter = std::min(closest_to_quarter, (x.translation() - quarter.first).norm());
    closest_to_half = std::min(closest_to_half, (x.translation() - half.first).norm());
  }
  EXPECT_LT(max_path_deviation, 1e-3) << "TCP left the commanded curve";
  EXPECT_LT(max_orientation_error, 1e-2)
    << "orientation did not follow the progress along the curve";
  EXPECT_LT(closest_to_quarter, 1e-3) << "did not pass through the sine peak";
  EXPECT_LT(closest_to_half, 1e-3) << "did not pass through the midpoint";

  const auto last = sine_pose(x0, 1.0);
  const Eigen::Isometry3d xf = fk(joint_values_);
  EXPECT_LT((xf.translation() - last.first).norm(), 1e-3) << "did not reach the final waypoint";
  EXPECT_LT(Eigen::Quaterniond(xf.rotation()).angularDistance(last.second), 1e-3);
}

// Acceleration is continuous within a chunk, and velocity across chunks.
TEST_F(CartesianTrajectoryControllerTest, joint_motion_is_continuous_within_and_across_chunks)
{
  // default damping undershoots the first IK step of each chunk
  activate(1e-4);
  const Eigen::Isometry3d x0 = fk(HOME);

  const auto chunk_a = make_sine_msg(x0, 1, 0.0);
  trajectory_msgs::msg::JointTrajectory traj_a;
  ASSERT_TRUE(controller_->build_joint_trajectory(*chunk_a, traj_a));
  EXPECT_LT(max_knot_acceleration_jump(traj_a), 1e-6) << "acceleration jumps inside the chunk";

  controller_->reference_callback(chunk_a);
  std::vector<std::vector<double>> q = {joint_values_};
  record(150, q);
  const size_t seam = q.size();

  // the rest of the same curve, re-sent mid-motion
  const auto chunk_b = make_sine_msg(x0, 8, 1.5);
  trajectory_msgs::msg::JointTrajectory traj_b;
  ASSERT_TRUE(controller_->build_joint_trajectory(*chunk_b, traj_b));
  EXPECT_LT(max_knot_acceleration_jump(traj_b), 1e-6) << "acceleration jumps inside the chunk";

  controller_->reference_callback(chunk_b);
  record(20, q);

  expect_velocity_continuous_at(q, seam);
}

// A planner-style first point at t=0 does not make the arm stop at every waypoint.
TEST_F(CartesianTrajectoryControllerTest, first_point_at_zero_is_the_start_pose)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> poses;
  std::vector<double> times;
  for (int i = 0; i <= 4; ++i)
  {
    poses.push_back(offset_pose(x0, {0.05 * i, 0.0, -0.025 * i}, 0.0));
    times.push_back(0.5 * i);
  }
  controller_->reference_callback(make_multi_pose_msg(BASE, poses, times));

  std::vector<Eigen::Vector3d> tcp = {x0.translation()};
  for (int i = 0; i < 250; ++i)
  {
    run(1);
    tcp.push_back(fk(joint_values_).translation());
  }
  // the spline dips to ~2/3 of peak speed between waypoints; stopping drops it near zero
  double min_speed = std::numeric_limits<double>::infinity();
  double max_speed = 0.0;
  for (size_t k = 50; k <= 150; ++k)
  {
    const double speed = (tcp[k] - tcp[k - 1]).norm() / 0.01;
    min_speed = std::min(min_speed, speed);
    max_speed = std::max(max_speed, speed);
  }
  EXPECT_GT(min_speed, 0.5 * max_speed) << "arm slows down at the waypoints";
  EXPECT_LT((tcp.back() - poses.back().first).norm(), 1e-3);
}

// A planner-style chunk arriving mid-motion keeps the velocity.
TEST_F(CartesianTrajectoryControllerTest, first_point_at_zero_keeps_velocity_across_chunks)
{
  activate(1e-4);
  const Eigen::Isometry3d x0 = fk(HOME);
  const auto a = offset_pose(x0, {0.20, 0.06, -0.10}, 0.4);
  controller_->reference_callback(make_pose_msg(BASE, a.first, a.second, 2.0));
  std::vector<std::vector<double>> q = {joint_values_};
  record(60, q);
  const size_t seam = q.size();

  const Eigen::Isometry3d now = fk(joint_values_);
  const auto chunk = make_multi_pose_msg(
    BASE, {{now.translation(), Eigen::Quaterniond(now.rotation())}, a}, {0.0, 1.4});
  controller_->reference_callback(chunk);
  record(20, q);
  expect_velocity_continuous_at(q, seam);
}

// Non-increasing times reject the message.
TEST_F(CartesianTrajectoryControllerTest, rejects_non_increasing_times)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  const auto p1 = offset_pose(x0, {0.03, 0.0, 0.0}, 0.0);
  const auto p2 = offset_pose(x0, {0.06, 0.0, 0.0}, 0.0);
  for (const auto & times :
       {std::vector<double>{0.5, 0.5}, std::vector<double>{1.0, 0.5},
        std::vector<double>{-0.5, 1.0}})
  {
    const auto msg = make_multi_pose_msg(BASE, {p1, p2}, times);
    trajectory_msgs::msg::JointTrajectory joint_traj;
    EXPECT_FALSE(controller_->build_joint_trajectory(*msg, joint_traj))
      << "accepted times " << times[0] << ", " << times[1];
    controller_->reference_callback(msg);
  }
  run(150);
  EXPECT_LT(joint_travel(), 1e-6);
}

// Durations that are not an exact multiple of resample_dt in floating point are still accepted.
TEST_F(CartesianTrajectoryControllerTest, accepts_durations_with_rounding_error)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  const auto tgt = offset_pose(x0, {0.05, 0.0, -0.02}, 0.1);
  for (double duration : {0.07, 1.11, 2.49})
  {
    trajectory_msgs::msg::JointTrajectory joint_traj;
    ASSERT_TRUE(controller_->build_joint_trajectory(
      *make_pose_msg(BASE, tgt.first, tgt.second, duration), joint_traj))
      << "rejected a " << duration << " s trajectory";
    EXPECT_DOUBLE_EQ(
      rclcpp::Duration(joint_traj.points.back().time_from_start).seconds(), duration);
  }
  controller_->reference_callback(make_pose_msg(BASE, tgt.first, tgt.second, 1.11));
  run(150);
  EXPECT_LT((fk(joint_values_).translation() - tgt.first).norm(), 1e-3);
}

// A chunk arriving while the tool rotates about a base axis keeps the angular velocity.
TEST_F(CartesianTrajectoryControllerTest, keeps_rotation_velocity_about_base_axes)
{
  for (int a = 0; a < 3; ++a)
  {
    SCOPED_TRACE("base axis " + std::string(1, "XYZ"[a]));
    const Eigen::Vector3d axis = Eigen::Vector3d::Unit(a);
    SetUp();
    activate(1e-4);
    const Eigen::Isometry3d x0 = fk(HOME);
    const Eigen::Vector3d p = x0.translation() + Eigen::Vector3d(0.10, 0.05, -0.05);
    const Eigen::Quaterniond q =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.4, axis)) * Eigen::Quaterniond(x0.rotation());
    controller_->reference_callback(make_pose_msg(BASE, p, q, 2.0));
    std::vector<std::vector<double>> joints = {joint_values_};
    record(60, joints);
    const size_t seam = joints.size();
    controller_->reference_callback(make_pose_msg(BASE, p, q, 1.4));
    record(20, joints);
    expect_velocity_continuous_at(joints, seam);
  }
}

// A pose with a NaN or a zero quaternion is rejected.
TEST_F(CartesianTrajectoryControllerTest, rejects_invalid_pose)
{
  activate();
  const Eigen::Isometry3d x0 = fk(HOME);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Eigen::Quaterniond q0(x0.rotation());
  const Eigen::Vector3d p0 = x0.translation();
  using Pose = std::pair<Eigen::Vector3d, Eigen::Quaterniond>;
  for (const auto & [p, q] :
       {Pose{Eigen::Vector3d(nan, 0.0, 0.0), q0}, Pose{p0, Eigen::Quaterniond(nan, 0.0, 0.0, 1.0)},
        Pose{p0, Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)}})
  {
    const auto msg = make_pose_msg(BASE, p, q, 1.0);
    trajectory_msgs::msg::JointTrajectory joint_traj;
    EXPECT_FALSE(controller_->build_joint_trajectory(*msg, joint_traj));
    controller_->reference_callback(msg);
  }
  run(150);
  EXPECT_TRUE(Eigen::Map<const Eigen::VectorXd>(joint_values_.data(), 6).allFinite());
  EXPECT_LT(joint_travel(), 1e-6);
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
  EXPECT_LT((xf.translation() - tgt.first).norm(), 1e-3);
  EXPECT_LT(Eigen::Quaterniond(xf.rotation()).angularDistance(tgt.second), 1e-3);
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
  EXPECT_LT((fk(joint_values_).translation() - p2.first).norm(), 1e-3);
}

// on_configure disables JTC's joint-space command inputs.
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
  EXPECT_LT((fk(joint_values_).translation() - tgt.first).norm(), 1e-3);
}

// A target received while the controller is inactive is not accepted.
TEST_F(CartesianTrajectoryControllerTest, ignored_when_not_active)
{
  activate();
  ASSERT_TRUE(deactivate_succeeds(controller_));
  ASSERT_TRUE(controller_->is_holding());

  const Eigen::Isometry3d x0 = fk(HOME);
  const auto tgt = offset_pose(x0, {0.05, 0.0, 0.0}, 0.0);
  controller_->reference_callback(make_pose_msg(BASE, tgt.first, tgt.second, 1.0));
  EXPECT_TRUE(controller_->is_holding()) << "a target received while inactive was accepted";
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
  EXPECT_LT((fk(joint_values_).translation() - tgt.first).norm(), 1e-3);
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
  EXPECT_LT((xf.translation() - tgt_b.first).norm(), 1e-3) << "did not end at chunk B";
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
