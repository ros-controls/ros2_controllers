/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Bence Magyar, Enrique Fernández
 * Author: Brighten Lee
 */

#include "ros_controllers/diff_drive_controller.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace ros_controllers
{

DiffDriveController::DiffDriveController()
    : controller_interface::ControllerInterface(),
      left_joint_names_({}),
      right_joint_names_({}),
      wheel_joints_size_(0),
      wheel_separation_(0.0),
      wheel_radius_(0.0),
      wheel_separation_multiplier_(1.0),
      left_wheel_radius_multiplier_(1.0),
      right_wheel_radius_multiplier_(1.0),
      open_loop_(false),
      enable_odom_tf_(true),
      base_frame_id_("base_link"),
      odom_frame_id_("odom"),
      pose_covariance_diagonal_({}),
      twist_covariance_diagonal_({}),
      command_struct_(),
      subscriber_is_active_(false),
      allow_multiple_cmd_vel_publishers_(true),
      velocity_rolling_window_size_(10),
      cmd_vel_timeout_(0.5),
      last1_cmd_(),
      last0_cmd_(),
      publish_cmd_(false),
      previous_time_(0.0)
{
}

DiffDriveController::DiffDriveController(const std::vector<std::string> &left_joint_names,
                                         const std::vector<std::string> &right_joint_names)
    : controller_interface::ControllerInterface(),
      left_joint_names_(left_joint_names),
      right_joint_names_(right_joint_names),
      wheel_joints_size_(0),
      wheel_separation_(0.0),
      wheel_radius_(0.0),
      wheel_separation_multiplier_(1.0),
      left_wheel_radius_multiplier_(1.0),
      right_wheel_radius_multiplier_(1.0),
      open_loop_(false),
      enable_odom_tf_(true),
      base_frame_id_("base_link"),
      odom_frame_id_("odom"),
      pose_covariance_diagonal_({}),
      twist_covariance_diagonal_({}),
      command_struct_(),
      subscriber_is_active_(false),
      allow_multiple_cmd_vel_publishers_(true),
      velocity_rolling_window_size_(10),
      cmd_vel_timeout_(0.5),
      last1_cmd_(),
      last0_cmd_(),
      publish_cmd_(false),
      previous_time_(0.0)
{
}

controller_interface::controller_interface_ret_t
DiffDriveController::init(std::weak_ptr<hardware_interface::RobotHardware> robot_hardware,
                          const std::string &controller_name)
{
  auto ret = ControllerInterface::init(robot_hardware, controller_name);
  if (ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
    return ret;

  lifecycle_node_->declare_parameter("left_joint_names");
  lifecycle_node_->declare_parameter("right_joint_names");
  lifecycle_node_->declare_parameter("pose_covariance_diagonal");
  lifecycle_node_->declare_parameter("twist_covariance_diagonal");
  lifecycle_node_->declare_parameter("wheel_separation");
  lifecycle_node_->declare_parameter("wheel_radius");
  lifecycle_node_->declare_parameter("wheel_separation_multiplier");
  lifecycle_node_->declare_parameter("wheel_radius_multiplier");
  lifecycle_node_->declare_parameter("open_loop");
  lifecycle_node_->declare_parameter("cmd_vel_timeout");
  lifecycle_node_->declare_parameter("allow_multiple_cmd_vel_publishers");
  lifecycle_node_->declare_parameter("base_frame_id");
  lifecycle_node_->declare_parameter("odom_frame_id");
  lifecycle_node_->declare_parameter("enable_odom_tf");
  lifecycle_node_->declare_parameter("publish_cmd");
  lifecycle_node_->declare_parameter("velocity_rolling_window_size");

  lifecycle_node_->declare_parameter("linear.x.has_velocity_limits");
  lifecycle_node_->declare_parameter("linear.x.has_acceleration_limits");
  lifecycle_node_->declare_parameter("linear.x.has_jerk_limits");
  lifecycle_node_->declare_parameter("linear.x.max_velocity");
  lifecycle_node_->declare_parameter("linear.x.min_velocity");
  lifecycle_node_->declare_parameter("linear.x.max_acceleration");
  lifecycle_node_->declare_parameter("linear.x.min_acceleration");
  lifecycle_node_->declare_parameter("linear.x.max_jerk");
  lifecycle_node_->declare_parameter("linear.x.min_jerk");

  lifecycle_node_->declare_parameter("angular.z.has_velocity_limits");
  lifecycle_node_->declare_parameter("angular.z.has_acceleration_limits");
  lifecycle_node_->declare_parameter("angular.z.has_jerk_limits");
  lifecycle_node_->declare_parameter("angular.z.max_velocity");
  lifecycle_node_->declare_parameter("angular.z.min_velocity");
  lifecycle_node_->declare_parameter("angular.z.max_acceleration");
  lifecycle_node_->declare_parameter("angular.z.min_acceleration");
  lifecycle_node_->declare_parameter("angular.z.max_jerk");
  lifecycle_node_->declare_parameter("angular.z.min_jerk");

  odometry_ = std::make_shared<Odometry>();
  limiter_lin_ = std::make_shared<SpeedLimiter>();
  limiter_ang_ = std::make_shared<SpeedLimiter>();

  return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
  (void)previous_state;

  auto logger = lifecycle_node_->get_logger();

  if (!reset())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

  lifecycle_node_->get_parameter<std::vector<std::string>>("left_joint_names", left_joint_names_);
  lifecycle_node_->get_parameter<std::vector<std::string>>("right_joint_names", right_joint_names_);
  lifecycle_node_->get_parameter<std::vector<double>>("pose_covariance_diagonal", pose_covariance_diagonal_);
  lifecycle_node_->get_parameter<std::vector<double>>("twist_covariance_diagonal", twist_covariance_diagonal_);
  lifecycle_node_->get_parameter<bool>("open_loop", open_loop_);
  lifecycle_node_->get_parameter<double>("wheel_separation", wheel_separation_);
  lifecycle_node_->get_parameter<double>("wheel_radius", wheel_radius_);
  lifecycle_node_->get_parameter<double>("wheel_separation_multiplier", wheel_separation_multiplier_);
  lifecycle_node_->get_parameter<double>("wheel_radius_multiplier", left_wheel_radius_multiplier_);
  lifecycle_node_->get_parameter<double>("wheel_radius_multiplier", right_wheel_radius_multiplier_);
  lifecycle_node_->get_parameter<double>("cmd_vel_timeout", cmd_vel_timeout_);
  lifecycle_node_->get_parameter<bool>("allow_multiple_cmd_vel_publishers", allow_multiple_cmd_vel_publishers_);
  lifecycle_node_->get_parameter<std::string>("base_frame_id", base_frame_id_);
  lifecycle_node_->get_parameter<std::string>("odom_frame_id", odom_frame_id_);
  lifecycle_node_->get_parameter<bool>("enable_odom_tf", enable_odom_tf_);
  lifecycle_node_->get_parameter<bool>("publish_cmd", publish_cmd_);
  lifecycle_node_->get_parameter<int>("velocity_rolling_window_size", velocity_rolling_window_size_);

  lifecycle_node_->get_parameter<bool>("linear.x.has_velocity_limits", limiter_lin_->has_velocity_limits);
  lifecycle_node_->get_parameter<bool>("linear.x.has_acceleration_limits", limiter_lin_->has_acceleration_limits);
  lifecycle_node_->get_parameter<bool>("linear.x.has_jerk_limits", limiter_lin_->has_jerk_limits);
  lifecycle_node_->get_parameter<double>("linear.x.max_velocity", limiter_lin_->max_velocity);
  lifecycle_node_->get_parameter<double>("linear.x.min_velocity", limiter_lin_->min_velocity);
  lifecycle_node_->get_parameter<double>("linear.x.max_acceleration", limiter_lin_->max_acceleration);
  lifecycle_node_->get_parameter<double>("linear.x.min_acceleration", limiter_lin_->min_acceleration);
  lifecycle_node_->get_parameter<double>("linear.x.max_jerk", limiter_lin_->max_jerk);
  lifecycle_node_->get_parameter<double>("linear.x.min_jerk", limiter_lin_->min_jerk);

  lifecycle_node_->get_parameter<bool>("angular.z.has_velocity_limits", limiter_ang_->has_velocity_limits);
  lifecycle_node_->get_parameter<bool>("angular.z.has_acceleration_limits", limiter_ang_->has_acceleration_limits);
  lifecycle_node_->get_parameter<bool>("angular.z.has_jerk_limits", limiter_ang_->has_jerk_limits);
  lifecycle_node_->get_parameter<double>("angular.z.max_velocity", limiter_ang_->max_velocity);
  lifecycle_node_->get_parameter<double>("angular.z.min_velocity", limiter_ang_->min_velocity);
  lifecycle_node_->get_parameter<double>("angular.z.max_acceleration", limiter_ang_->max_acceleration);
  lifecycle_node_->get_parameter<double>("angular.z.min_acceleration", limiter_ang_->min_acceleration);
  lifecycle_node_->get_parameter<double>("angular.z.max_jerk", limiter_ang_->max_jerk);
  lifecycle_node_->get_parameter<double>("angular.z.min_jerk", limiter_ang_->min_jerk);

  odometry_->setVelocityRollingWindowSize(velocity_rolling_window_size_);

  if (left_joint_names_.empty() || right_joint_names_.empty())
  {
    RCLCPP_ERROR(logger, "No joint names specified");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (left_joint_names_.size() != right_joint_names_.size())
  {
    RCLCPP_ERROR(logger, "The number of left wheels[%d] and the number of right wheels[%d] are different",
                 left_joint_names_.size(), right_joint_names_.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  wheel_joints_size_ = left_joint_names_.size();

  RCLCPP_INFO(logger, "Wheel separation will be multiplied by %f",
              wheel_separation_multiplier_);
  RCLCPP_INFO(logger, "Left wheel radius will be multiplied by %f",
              left_wheel_radius_multiplier_);
  RCLCPP_INFO(logger, "Right wheel radius will be multiplied by %f",
              right_wheel_radius_multiplier_);
  RCLCPP_INFO(logger, "Velocity commands will be considered old if they are older than %f",
              cmd_vel_timeout_);
  RCLCPP_INFO(logger, "Allow mutiple cmd_vel publishers is %s",
              (allow_multiple_cmd_vel_publishers_ ? "enabled" : "disabled"));
  RCLCPP_INFO(logger, "Base frame id set to %s",
              base_frame_id_.c_str());
  RCLCPP_INFO(logger, "Odometry frame id set to %s",
              odom_frame_id_.c_str());
  RCLCPP_INFO(logger, "Publishing to tf is %s",
              (enable_odom_tf_ ? "enabled" : "disabled"));

  if (auto robot_hardware = robot_hardware_.lock())
  {
    registered_left_joint_state_handles_.resize(left_joint_names_.size());
    for (size_t index = 0; index < left_joint_names_.size(); ++index)
    {
      auto ret = robot_hardware->get_joint_state_handle(
          left_joint_names_[index].c_str(), &registered_left_joint_state_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK)
      {
        RCLCPP_WARN(logger, "Unable to obtain joint state handle for %s",
                    left_joint_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }

    registered_right_joint_state_handles_.resize(right_joint_names_.size());
    for (size_t index = 0; index < right_joint_names_.size(); ++index)
    {
      auto ret = robot_hardware->get_joint_state_handle(
          right_joint_names_[index].c_str(), &registered_right_joint_state_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK)
      {
        RCLCPP_WARN(logger, "Unable to obtain joint state handle for %s",
                    right_joint_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }

    registered_left_joint_cmd_handles_.resize(left_joint_names_.size());
    for (size_t index = 0; index < left_joint_names_.size(); ++index)
    {
      auto ret = robot_hardware->get_joint_command_handle(
          left_joint_names_[index].c_str(), &registered_left_joint_cmd_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK)
      {
        RCLCPP_WARN(logger, "Unable to obtain joint command handle for %s",
                    left_joint_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }

    registered_right_joint_cmd_handles_.resize(right_joint_names_.size());
    for (size_t index = 0; index < right_joint_names_.size(); ++index)
    {
      auto ret = robot_hardware->get_joint_command_handle(
          right_joint_names_[index].c_str(), &registered_right_joint_cmd_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK)
      {
        RCLCPP_WARN(logger, "Unable to obtain joint command handle for %s",
                    right_joint_names_[index].c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
  }
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

  if (registered_left_joint_state_handles_.empty() ||
      registered_right_joint_state_handles_.empty() ||
      registered_left_joint_cmd_handles_.empty() ||
      registered_right_joint_cmd_handles_.empty())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

  if (publish_cmd_)
  {
    pub_cmd_vel_ = lifecycle_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_out", rclcpp::SystemDefaultsQoS());
    rp_cmd_vel_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(pub_cmd_vel_);
  }

  sub_command_ = lifecycle_node_->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1));

  pub_odom_ = lifecycle_node_->create_publisher<nav_msgs::msg::Odometry>(
      "~/odom", rclcpp::SystemDefaultsQoS());
  rp_odom_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(pub_odom_);

  pub_tf_odom_ = lifecycle_node_->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
  rp_tf_odom_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(pub_tf_odom_);

  const double ws = wheel_separation_multiplier_ * wheel_separation_;
  const double lwr = left_wheel_radius_multiplier_ * wheel_radius_;
  const double rwr = right_wheel_radius_multiplier_ * wheel_radius_;
  odometry_->setWheelParams(ws, lwr, rwr);
  RCLCPP_INFO(logger, "Odometry params : wheel separation %f, left wheel radius %f, right wheel radius %f", ws, lwr, rwr);

  rp_odom_->msg_.header.frame_id = odom_frame_id_;
  rp_odom_->msg_.child_frame_id = base_frame_id_;
  rp_odom_->msg_.pose.pose.position.z = 0.0;
  rp_odom_->msg_.pose.covariance = {
      pose_covariance_diagonal_[0], 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, pose_covariance_diagonal_[1], 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, pose_covariance_diagonal_[2], 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, pose_covariance_diagonal_[3], 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, pose_covariance_diagonal_[4], 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, pose_covariance_diagonal_[5]};

  rp_odom_->msg_.twist.twist.linear.y = 0.0;
  rp_odom_->msg_.twist.twist.linear.z = 0.0;
  rp_odom_->msg_.twist.twist.angular.x = 0.0;
  rp_odom_->msg_.twist.twist.angular.y = 0.0;
  rp_odom_->msg_.twist.covariance = {
      twist_covariance_diagonal_[0], 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, twist_covariance_diagonal_[1], 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, twist_covariance_diagonal_[2], 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, twist_covariance_diagonal_[3], 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, twist_covariance_diagonal_[4], 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, twist_covariance_diagonal_[5]};

  rp_tf_odom_->msg_.transforms.resize(1);
  rp_tf_odom_->msg_.transforms[0].transform.translation.z = 0.0;
  rp_tf_odom_->msg_.transforms[0].child_frame_id = base_frame_id_;
  rp_tf_odom_->msg_.transforms[0].header.frame_id = odom_frame_id_;

  previous_time_ = rclcpp::Clock().now();
  command_struct_.stamp = rclcpp::Clock().now();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::controller_interface_ret_t
DiffDriveController::update()
{
  rclcpp::Time cur_time = rclcpp::Clock().now();

  const double ws = wheel_separation_multiplier_ * wheel_separation_;
  const double lwr = left_wheel_radius_multiplier_ * wheel_radius_;
  const double rwr = right_wheel_radius_multiplier_ * wheel_radius_;
  odometry_->setWheelParams(ws, lwr, rwr);

  if (open_loop_)
    odometry_->updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, cur_time);
  else
  {
    double left_pos = 0.0;
    double right_pos = 0.0;
    for (size_t i = 0; i < wheel_joints_size_; ++i)
    {
      const double lp = registered_left_joint_state_handles_[i]->get_position();
      const double rp = registered_right_joint_state_handles_[i]->get_position();
      if (std::isnan(lp) || std::isnan(rp))
        return controller_interface::CONTROLLER_INTERFACE_RET_ERROR;

      left_pos += lp;
      right_pos += rp;
    }
    left_pos /= wheel_joints_size_;
    right_pos /= wheel_joints_size_;

    odometry_->update(left_pos, right_pos, cur_time);
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_->getHeading());

  if (pub_odom_->is_activated() && rp_odom_->trylock())
  {
    rp_odom_->msg_.header.stamp = cur_time;
    rp_odom_->msg_.pose.pose.position.x = odometry_->getX();
    rp_odom_->msg_.pose.pose.position.y = odometry_->getY();
    rp_odom_->msg_.pose.pose.orientation.x = orientation.x();
    rp_odom_->msg_.pose.pose.orientation.y = orientation.y();
    rp_odom_->msg_.pose.pose.orientation.z = orientation.z();
    rp_odom_->msg_.pose.pose.orientation.w = orientation.w();
    rp_odom_->msg_.twist.twist.linear.x = odometry_->getLinear();
    rp_odom_->msg_.twist.twist.angular.z = odometry_->getAngular();
    rp_odom_->unlockAndPublish();
  }

  if (enable_odom_tf_ && pub_tf_odom_->is_activated() && rp_tf_odom_->trylock())
  {
    rp_tf_odom_->msg_.transforms[0].header.stamp = cur_time;
    rp_tf_odom_->msg_.transforms[0].transform.translation.x = odometry_->getX();
    rp_tf_odom_->msg_.transforms[0].transform.translation.y = odometry_->getY();
    rp_tf_odom_->msg_.transforms[0].transform.rotation.x = orientation.x();
    rp_tf_odom_->msg_.transforms[0].transform.rotation.y = orientation.y();
    rp_tf_odom_->msg_.transforms[0].transform.rotation.z = orientation.z();
    rp_tf_odom_->msg_.transforms[0].transform.rotation.w = orientation.w();
    rp_tf_odom_->unlockAndPublish();
  }

  // Retreive current velocity command and time step
  Commands cur_cmd = *(command_.readFromRT());

  const double dt = cur_time.seconds() - cur_cmd.stamp.seconds();

  // Brake if cmd_vel has timeout
  if (dt > cmd_vel_timeout_)
  {
    cur_cmd.lin = 0.0;
    cur_cmd.ang = 0.0;
  }

  const double cmd_dt = cur_time.seconds() - previous_time_.seconds();
  previous_time_ = cur_time;

  limiter_lin_->limit(cur_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
  limiter_ang_->limit(cur_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

  last1_cmd_ = last0_cmd_;
  last0_cmd_ = cur_cmd;

  // Publish limited velocity
  if (publish_cmd_ && pub_cmd_vel_->is_activated() && rp_cmd_vel_->trylock())
  {
    rp_cmd_vel_->msg_.header.stamp = cur_time;
    rp_cmd_vel_->msg_.twist.linear.x = cur_cmd.lin;
    rp_cmd_vel_->msg_.twist.angular.z = cur_cmd.ang;
    rp_cmd_vel_->unlockAndPublish();
  }

  // Compute wheels velocities:
  const double vel_left = (cur_cmd.lin - cur_cmd.ang * ws / 2.0) / lwr;
  const double vel_right = (cur_cmd.lin + cur_cmd.ang * ws / 2.0) / rwr;

  // Set wheels velocities:
  for (size_t i = 0; i < wheel_joints_size_; ++i)
  {
    registered_left_joint_cmd_handles_[i]->set_cmd(vel_left);
    registered_right_joint_cmd_handles_[i]->set_cmd(vel_right);
  }

  return controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  (void)previous_state;

  subscriber_is_active_ = true;
  pub_odom_->on_activate();
  pub_tf_odom_->on_activate();
  if (publish_cmd_)
    pub_cmd_vel_->on_activate();

  RCLCPP_INFO(lifecycle_node_->get_logger(),
              "Lifecycle subscriber and publisher are currently active.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  (void)previous_state;

  subscriber_is_active_ = false;
  pub_odom_->on_deactivate();
  pub_tf_odom_->on_deactivate();
  if (publish_cmd_)
    pub_cmd_vel_->on_deactivate();

  RCLCPP_WARN(lifecycle_node_->get_logger(),
              "Lifecycle subscriber and publisher are currently inactive. Can't subscribe and publish messages");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
  (void)previous_state;

  odometry_->resetOdometry();

  if (!reset())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_error(const rclcpp_lifecycle::State &previous_state)
{
  (void)previous_state;

  if (!reset())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
DiffDriveController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
  (void)previous_state;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void DiffDriveController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (subscriber_is_active_)
  {
    if (!allow_multiple_cmd_vel_publishers_)
    {
      // TODO
    }

    command_struct_.lin = msg->linear.x;
    command_struct_.ang = msg->angular.z;
    command_struct_.stamp = rclcpp::Clock().now();
    command_.writeFromNonRT(command_struct_);
  }
  else
    RCLCPP_ERROR(lifecycle_node_->get_logger(),
                 "Can't accept new commands. subscriber is inactive");
}

bool DiffDriveController::reset()
{
  registered_left_joint_state_handles_.clear();
  registered_right_joint_state_handles_.clear();
  registered_left_joint_cmd_handles_.clear();
  registered_right_joint_cmd_handles_.clear();

  subscriber_is_active_ = false;
  sub_command_.reset();

  return true;
}

} // namespace ros_controllers

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    ros_controllers::DiffDriveController, controller_interface::ControllerInterface)