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

#ifndef DIFFBOT_HPP_
#define DIFFBOT_HPP_

#include <limits>
#include <memory>
#include <sstream>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/robot_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_srvs/srv/empty.hpp"

template<unsigned int NUM_JOINTS = 2>
class Diffbot : public hardware_interface::RobotHardware
{
public:
  hardware_interface::return_type init() override
  {
    // Setup service used for testing NaN read in test_diff_drive_nan.cpp
    node_ = std::make_shared<rclcpp::Node>("diffbot_robot_hardware");
    running_ = true;  // Running by default.
    start_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "start_diffbot",
      std::bind(&Diffbot::start_callback, this, std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "stop_diffbot",
      std::bind(&Diffbot::stop_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize raw data
    std::fill_n(pos_, NUM_JOINTS, 0);
    std::fill_n(vel_, NUM_JOINTS, 0);
    std::fill_n(eff_, NUM_JOINTS, 0);
    std::fill_n(cmd_, NUM_JOINTS, 0);
    op_status_ = hardware_interface::OperationMode::INACTIVE;

    // Connect and register the joint state and velocity interface
    hardware_interface::return_type ret;

    op_handle_ = hardware_interface::OperationModeHandle("op_status", &op_status_);
    ret = register_operation_mode_handle(&op_handle_);
    if (ret != hardware_interface::return_type::OK) {
      RCLCPP_WARN(node_->get_logger(), "Cannot register operation handle: op_status");
      return ret;
    }

    for (auto index = 0u; index < NUM_JOINTS; index++) {
      std::ostringstream joint_name_os;
      joint_name_os << "wheel_" << index << "_joint";

      // register actuators and joints
      register_joint(joint_name_os.str(), "position", pos_[index]);
      register_joint(joint_name_os.str(), "velocity", vel_[index]);
      register_joint(joint_name_os.str(), "effort", eff_[index]);
      // register_joint(joint_name_os.str(), "position_command", pos_dflt_values[index]);
      register_joint(joint_name_os.str(), "velocity_command", cmd_[index]);
      // register_joint(joint_name_os.str(), "effort_command", eff_dflt_values[index]);
    }
    return hardware_interface::return_type::OK;
  }

  [[nodiscard]] rclcpp::Duration get_period() const {return period_;}

  // Diffbot assumes read is called with frequency of 1/period_.
  hardware_interface::return_type read() override
  {
    // Read the joint state of the robot into the hardware interface
    if (running_) {
      for (auto i = 0u; i < NUM_JOINTS; ++i) {
        pos_[i] += vel_[i] * get_period().seconds();  // update position
        vel_[i] = cmd_[i];                            // might add smoothing here later
      }
    } else {
      std::fill_n(pos_, NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
      std::fill_n(vel_, NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write() override
  {
    // Write the commands to the joints
    std::ostringstream os;
    for (auto i = 0u; i < NUM_JOINTS - 1; ++i) {
      os << cmd_[i] << ", ";
    }
    os << cmd_[NUM_JOINTS - 1];

    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Commands for joints: " << os.str());
    return hardware_interface::return_type::OK;
  }

  rclcpp::Node::SharedPtr & get_node() {return node_;}

  void start_callback(
    std_srvs::srv::Empty::Request::SharedPtr /*req*/,
    std_srvs::srv::Empty::Response::SharedPtr /*res*/)
  {
    running_ = true;
    // Reset state
    std::fill_n(pos_, NUM_JOINTS, 0);
    std::fill_n(vel_, NUM_JOINTS, 0);
    std::fill_n(eff_, NUM_JOINTS, 0);
    std::fill_n(cmd_, NUM_JOINTS, 0);
  }

  void stop_callback(
    std_srvs::srv::Empty::Request::SharedPtr /*req*/,
    std_srvs::srv::Empty::Response::SharedPtr /*res*/)
  {
    running_ = false;
  }

private:
  double pos_[NUM_JOINTS];
  double vel_[NUM_JOINTS];
  double eff_[NUM_JOINTS];
  double cmd_[NUM_JOINTS];
  hardware_interface::OperationMode op_status_;

  rclcpp::Node::SharedPtr node_;

  bool running_;
  rclcpp::ServiceBase::SharedPtr start_srv_;
  rclcpp::ServiceBase::SharedPtr stop_srv_;

  hardware_interface::OperationModeHandle op_handle_;

  rclcpp::Duration period_ =
    std::chrono::milliseconds(10);  // Period assumed to elapse between each read
};

/**
 * \brief Ensure that a ROS TIME simulation is active
 * \param sim_clk
 * \param logger
 */
void ensure_sim_clock_is_active(
  const rclcpp::Clock::SharedPtr & sim_clk, const rclcpp::Logger & logger)
{
  auto attempt = 0;
  while (!sim_clk->ros_time_is_active()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    if (attempt > 4) {
      RCLCPP_FATAL(
        logger,
        "Failed to activate ROS TIME simulation. Make sure 'use_sim_time' is set for a node "
        "clock.");
      throw std::runtime_error("Failed to activate ROS TIME simulation");
    }
    ++attempt;
  }
}

constexpr auto DIFF_DRIVE_CONTROLLER_NAME = "diffbot_controller";
constexpr auto CONTROLLER_MANAGER_NAME = "diffbot_controller_manager";

/**
 * \brief Main function used for running the Diffbot instance
 * \tparam NUM_JOINTS The number of wheels of Diffbot
 * \param argc main's argc
 * \param argv main's argv
 * \return exit code
 */
template<unsigned int NUM_JOINTS = 2>
int diffbot_main_runner(int argc, char * const * argv)
{
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto fut = std::async(std::launch::async, [&exec]() {exec->spin();});
  auto logger = rclcpp::get_logger("diffbot_control_loop");

  // Initialize Diffbot RobotHardware
  auto robot = std::make_shared<Diffbot<NUM_JOINTS>>();
  if (robot->init() != hardware_interface::return_type::OK) {
    RCLCPP_ERROR(logger, "Failed to initialize Diffbot");
    return EXIT_FAILURE;
  }
  auto robot_hw_node = robot->get_node();
  robot_hw_node->set_parameter({"use_sim_time", true});
  exec->add_node(robot_hw_node);
  ensure_sim_clock_is_active(robot_hw_node->get_clock(), robot_hw_node->get_logger());

  // Set up ControllerManager with DiffDriveController using Diffbot. Use sim time.
  auto cm =
    std::make_shared<controller_manager::ControllerManager>(robot, exec, CONTROLLER_MANAGER_NAME);
  cm->set_parameter({"use_sim_time", true});
  exec->add_node(cm);
  ensure_sim_clock_is_active(cm->get_clock(), cm->get_logger());

  auto ddc = cm->load_controller(
    DIFF_DRIVE_CONTROLLER_NAME, "diff_drive_controller/DiffDriveController");  // Load via pluginlib
  const auto ddc_node = ddc->get_lifecycle_node();
  ddc_node->set_parameter({"use_sim_time", true});
  ensure_sim_clock_is_active(ddc_node->get_clock(), ddc_node->get_logger());

  // Use controller manager node as sim time publisher
  auto sim_clk_pub =
    cm->template create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::SystemDefaultsQoS());
  auto sim_clk = *cm->get_clock().get();
  auto sim_time = rclcpp::Time(0L, RCL_ROS_TIME);
  const auto single_loop_period = robot->get_period();
  auto sys_clk = rclcpp::Clock(RCL_SYSTEM_TIME);

  // Loop approximated to iterate once every single_loop_period.
  while (rclcpp::ok()) {
    auto begin = sys_clk.now();
    if (robot->read() != hardware_interface::return_type::OK) {
      RCLCPP_WARN(logger, "Diffbot failed to read.");
    }
    if (cm->update() != controller_interface::return_type::SUCCESS) {
      RCLCPP_WARN(logger, "Controller manager failed to update.");
    }
    if (robot->write() != hardware_interface::return_type::OK) {
      RCLCPP_WARN(logger, "Diffbot failed to read.");
    }
    auto end = sys_clk.now();

    auto actual_elapsed_secs = (end - begin).seconds();
    auto wait_time = single_loop_period.seconds() - actual_elapsed_secs;
    if (wait_time < 0.0) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger, sys_clk, 100,
        "Control cycle is taking too much time, elapsed: " << actual_elapsed_secs);
    } else {
      RCLCPP_DEBUG_STREAM_THROTTLE(
        logger, sys_clk, 1000, "Control cycle elapsed: " << actual_elapsed_secs);
      std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));
    }

    auto clk_msg = rosgraph_msgs::msg::Clock();
    clk_msg.clock = sim_time;
    sim_clk_pub->publish(clk_msg);
    sim_time = sim_time + single_loop_period;
  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}

#endif  // DIFFBOT_HPP_
