// Copyright 2020 PAL Robotics S.L.
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

#pragma once

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2_ros/transform_listener.h>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <string>

#ifndef TEST_COMMON_HPP_
#define TEST_COMMON_HPP_

// Floating-point value comparison threshold
constexpr double EPS = 0.01;
constexpr double POSITION_TOLERANCE = 0.01;               // 1 cm-s precision
constexpr double VELOCITY_TOLERANCE = 0.02;               // 2 cm-s-1 precision
constexpr double JERK_LINEAR_VELOCITY_TOLERANCE = 0.10;   // 10 cm-s-1 precision
constexpr double JERK_ANGULAR_VELOCITY_TOLERANCE = 0.05;  // 3 deg-s-1 precision
constexpr double ORIENTATION_TOLERANCE = 0.03;            // 1.72 degree precision

constexpr auto DIFF_DRIVE_CONTROLLER_NAME = "diffbot_controller";
constexpr auto DEFAULT_ODOM_FRAME_ID = "odom";
constexpr auto DEFAULT_BASE_FRAME_ID = "base_link";

using namespace std::chrono_literals;   // NOLINT(build/namespaces)

/*
 * DiffDriveControllerTest fixture makes the following assumptions:
 * 1. There exists an external simulated clock publisher
 * 2. There exists a DiffDriveController with name DIFF_DRIVE_CONTROLLER_NAME running
 * 3. All DiffDriveController topics are remapped to ${DIFF_DRIVE_CONTROLLER_NAME}/topic_name
 */

class DiffDriveControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Use simulated time
    node_->set_parameter({"use_sim_time", true});
    executor_->add_node(node_);
    executor_task_fut_ = std::async(std::launch::async, [this]() {this->executor_->spin();});
  }

  void TearDown() override {executor_->cancel();}

  DiffDriveControllerTest()
  : received_first_odom_(false),
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
    node_(std::make_shared<rclcpp::Node>("diffbot_controller_test")),
    logger_(node_->get_logger()),
    cmd_pub_(node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        std::string(DIFF_DRIVE_CONTROLLER_NAME) + "/cmd_vel", 100)),
    odom_sub_(node_->create_subscription<nav_msgs::msg::Odometry>(
        std::string(DIFF_DRIVE_CONTROLLER_NAME) + "/odom", 100,
        std::bind(&DiffDriveControllerTest::odom_callback, this, std::placeholders::_1))),
    vel_out_sub_(node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        std::string(DIFF_DRIVE_CONTROLLER_NAME) + "/cmd_vel_out", 100,
        std::bind(&DiffDriveControllerTest::cmd_vel_out_callback, this, std::placeholders::_1))),
    joint_traj_controller_state_sub_(
      node_->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        std::string(DIFF_DRIVE_CONTROLLER_NAME) + "/wheel_joint_controller_state", 100,
        std::bind(
          &DiffDriveControllerTest::joint_trajectory_controller_state_callback, this,
          std::placeholders::_1)))
  {
  }

  nav_msgs::msg::Odometry::ConstSharedPtr get_last_odom() {return last_odom_;}
  geometry_msgs::msg::TwistStamped::ConstSharedPtr get_last_cmd_vel_out()
  {
    return last_cmd_vel_out_;
  }
  control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr
  get_last_wheel_joint_controller_state()
  {
    return last_joint_traj_controller_state_;
  }

  void publish(geometry_msgs::msg::Twist cmd_vel)
  {
    auto cmd_vel_stamped = geometry_msgs::msg::TwistStamped();
    cmd_vel_stamped.header.stamp = node_->now();
    cmd_vel_stamped.twist = cmd_vel;
    cmd_pub_->publish(cmd_vel_stamped);
  }

  [[nodiscard]] bool is_controller_alive() const
  {
    return (odom_sub_->get_publisher_count() > 0) && (cmd_pub_->get_subscription_count() > 0);
  }

  /**
   * \brief Waits for DiffDriverController instance with DIFF_DRIVE_CONTROLLER_NAME name to activate.
   * \return returns true only if it is successfully confirmed.
   */
  [[nodiscard]] bool wait_for_controller() const
  {
    static constexpr auto max_wait_time = 10'000ms;

    RCLCPP_DEBUG(logger_, "Waiting for controller started");
    auto system_clk = rclcpp::Clock(rcl_clock_type_t::RCL_SYSTEM_TIME);
    const auto start_time = system_clk.now();
    const auto time_limit = start_time + max_wait_time;
    auto get_lifecycle_state_client = node_->create_client<lifecycle_msgs::srv::GetState>(
      std::string(DIFF_DRIVE_CONTROLLER_NAME) + "/get_state");

    if (!get_lifecycle_state_client->wait_for_service(max_wait_time)) {
      RCLCPP_ERROR(
        logger_,
        "Timed out waiting for /get_state service. Please make sure DiffDriveController node "
        "process is running.");
      return false;
    }

    using lifecycle_msgs::msg::State;
    auto diff_drive_controller_current_state = State::PRIMARY_STATE_UNKNOWN;
    while (diff_drive_controller_current_state != State::PRIMARY_STATE_ACTIVE) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          logger_, "rclcpp::ok() returned false while waiting for controller to activate");
        return false;
      }

      auto dur_left = time_limit - system_clk.now();
      if (dur_left.seconds() <= 0) {
        RCLCPP_ERROR(logger_, "Timed out waiting for DiffDriveController node to activate.");
        return false;
      }

      auto fut = get_lifecycle_state_client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState_Request>());
      // Wait for future to complete
      auto fut_status = fut.wait_for(dur_left.to_chrono<std::chrono::nanoseconds>());
      if (fut_status != std::future_status::ready) {
        continue;
      }

      const auto & res = fut.get();
      diff_drive_controller_current_state = res->current_state.id;
      std::this_thread::sleep_for(100ms);
      RCLCPP_DEBUG_STREAM_THROTTLE(
        logger_, system_clk, 500,
        "DiffDriveController state at time " << system_clk.now().seconds() << ": " <<
          res->current_state.label);
    }

    // Another sanity check to make sure subscriptions/publishers are alive.
    while (!is_controller_alive()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          logger_, "rclcpp::ok() returned false while waiting for controller to activate.");
        return false;
      }

      auto dur_left = time_limit - system_clk.now();
      if (dur_left.seconds() <= 0) {
        RCLCPP_ERROR(logger_, "Timed out waiting for DiffDriveController node to activate.");
        return false;
      }

      RCLCPP_DEBUG_THROTTLE(logger_, system_clk, 500, "Waiting for DiffDriveController node");
      std::this_thread::sleep_for(100ms);
    }

    // Make sure ROS TIME is active
    auto sim_clk = node_->get_clock();
    if (!sim_clk->ros_time_is_active()) {
      RCLCPP_ERROR(
        logger_,
        "ROS TIME is not active for this node. Make sure 'use_sim_time' parameter is set.");
      return false;
    }
    // Make sure ROS TIME is ticking
    auto sim_start = sim_clk->now();
    while (sim_clk->now() <= sim_start) {
      std::this_thread::sleep_for(100ms);

      auto dur_left = time_limit - system_clk.now();
      if (dur_left.seconds() <= 0) {
        RCLCPP_ERROR(
          logger_,
          "Timed out waiting for simulated clock to move. Sim start time: %f, Sim current time: %f",
          sim_start.seconds(), sim_clk->now().seconds());
        return false;
      }
    }

    RCLCPP_DEBUG_STREAM(
      logger_, "Waiting for controller completed successfully,  Simulated time(sec): " <<
        node_->get_clock()->now().seconds() <<
        " Actual: " << system_clk.now().seconds());
    return true;
  }

  [[nodiscard]] bool has_received_first_odom() const {return received_first_odom_;}

  /**
   * \brief Sleep using simulated clock. Implementation is loosely based off of how
   * rclcpp::Duration::sleep() works in ROS1.
   * https://github.com/ros/roscpp_core/blob/eabafec8f9b554d5389a1aa1b3c8145c07faaefb/rostime/src/time.cpp#L384
   * \param dur Duration to sleep
   * \param max_dur_multiplier Multiple of the duration that determines the maximum wait duration.
   *                           Ex. dur=10s, max_dur_multiplier= 1.5 => maximum wait duration = 15s.
    * \return true if slept for simulated clock duration
   */
  bool sim_sleep_for(const rclcpp::Duration & dur, double max_dur_multiplier = 2.0) const
  {
    static constexpr auto SLEEP_STEP = 1ms;  // Step used in ROS1

    auto sim_clk = node_->get_clock();
    // Make sure ROS TIME is active before attempting to sleep.
    if (!sim_clk->ros_time_is_active()) {
      RCLCPP_ERROR(
        logger_, "sim_sleep_for() method requires ROS TIME to be active. Aborting sleep.");
      return false;
    }

    const auto sim_start = sim_clk->now();
    const auto sim_end = sim_start + dur;

    auto sys_clk = rclcpp::Clock(rcl_clock_type_t::RCL_SYSTEM_TIME);
    const auto sys_start = sys_clk.now();
    const auto time_limit =
      sys_start + max_dur_multiplier * dur.to_chrono<std::chrono::nanoseconds>();

    RCLCPP_DEBUG_STREAM(
      logger_, "Sleeping for " << dur.seconds() <<
        " seconds using simulated clock. Start time Sim: " <<
        sim_start.seconds() << ", System: " << sys_start.seconds());
    while (sim_clk->now() < sim_end) {
      std::this_thread::sleep_for(SLEEP_STEP);

      // If time jumps backwards from when we started sleeping, return immediately
      if (sim_clk->now() < sim_start) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "Simulated time jumped backward from start time. Stopping sleep. Sim started time: " <<
            sim_start.seconds() << ", Sim current time: " << sim_clk->now().seconds());
        return false;
      }
      // if time limit is exceeded, return immediately
      if (sys_clk.now() > time_limit) {
        RCLCPP_WARN(logger_, "Simulation sleep exceeded wait threshold. Stopping sleep");
        return false;
      }
    }
    RCLCPP_DEBUG_STREAM(
      logger_, "Simulation sleep finished. Duration (sec) Simulated: " <<
      (sim_clk->now() - sim_start).seconds() <<
        ", System: " << rclcpp::Duration(sys_clk.now() - sys_start).seconds());
    return true;
  }

  [[nodiscard]] bool wait_for_odom_msgs() const
  {
    static const rclcpp::Duration timeout = 2s;

    auto time_limit = node_->now() + timeout;
    while (!has_received_first_odom()) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(logger_, "rclcpp::ok() returned false while waiting for first odom message.");
        return false;
      }
      if (node_->now() > time_limit) {
        RCLCPP_ERROR(logger_, "wait_for_odom_msgs() waiting time expired.");
        return false;
      }

      sim_sleep_for(100ms);
    }

    return has_received_first_odom();
  }

  [[nodiscard]] bool check_subscription_is_alive(
    rclcpp::SubscriptionBase::SharedPtr sub, const rclcpp::Duration & timeout = 1000ms) const
  {
    auto num_publishers = sub->get_publisher_count();
    const auto time_limit = node_->now() + timeout;
    while ((num_publishers == 0) && (node_->now() < time_limit)) {
      sim_sleep_for(100ms);
      num_publishers = sub->get_publisher_count();
    }
    return num_publishers > 0;
  }

  [[nodiscard]] bool is_publishing_cmd_vel_out(const rclcpp::Duration & timeout = 1000ms) const
  {
    return check_subscription_is_alive(vel_out_sub_, timeout);
  }

  [[nodiscard]] bool is_publishing_joint_trajectory_controller_state(
    const rclcpp::Duration & timeout = 1000ms) const
  {
    return check_subscription_is_alive(joint_traj_controller_state_sub_, timeout);
  }

  std::shared_ptr<tf2_ros::Buffer> & get_tf_buffer()
  {
    std::lock_guard<std::mutex> lg(tf_mutex_);
    if (tf_buffer_ == nullptr) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tf_buffer_->setUsingDedicatedThread(true);
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
    }

    return tf_buffer_;
  }

  rclcpp::Node::SharedPtr & get_node() {return node_;}

private:
  bool received_first_odom_;
  rclcpp::Executor::SharedPtr executor_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_out_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    joint_traj_controller_state_sub_;
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr last_cmd_vel_out_;
  control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr
    last_joint_traj_controller_state_;
  std::future<void> executor_task_fut_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::mutex tf_mutex_;

  void odom_callback(nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    RCLCPP_DEBUG_STREAM(
      logger_, "Callback received at " <<
        odom->header.stamp.sec << ": pos.x: " << odom->pose.pose.position.x <<
        ", orient.z: " << odom->pose.pose.orientation.z << ", lin_est: " <<
        odom->twist.twist.linear.x << ", ang_est: " << odom->twist.twist.angular.z);
    last_odom_ = odom;
    received_first_odom_ = true;
  }

  void cmd_vel_out_callback(geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel_out)
  {
    RCLCPP_DEBUG_STREAM(
      logger_, "Callback received at " << cmd_vel_out->header.stamp.sec <<
        ": lin: " << cmd_vel_out->twist.linear.x <<
        ", ang: " << cmd_vel_out->twist.angular.z);
    last_cmd_vel_out_ = cmd_vel_out;
  }

  void joint_trajectory_controller_state_callback(
    control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr joint_traj_controller_state)
  {
    RCLCPP_DEBUG_STREAM(
      logger_, "Joint trajectory controller state callback received:\n" <<
        joint_traj_controller_state);

    last_joint_traj_controller_state_ = joint_traj_controller_state;
  }
};

inline tf2::Quaternion tf_quat_from_geom_quat(const geometry_msgs::msg::Quaternion & quat)
{
  return tf2::Quaternion(quat.x, quat.y, quat.z, quat.w);
}

#endif  // TEST_COMMON_HPP_
