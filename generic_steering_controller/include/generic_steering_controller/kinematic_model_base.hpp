#ifndef GENERIC_STEERING_CONTROLLER__KINEMATIC_MODEL_BASE_HPP_
#define GENERIC_STEERING_CONTROLLER__KINEMATIC_MODEL_BASE_HPP_

// C++ Standard Library
#include <memory>
#include <string>
#include <utility>
#include <vector>

// ROS Messages
#include "control_msgs/msg/steering_controller_status.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

// rclcpp
#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace kinematic_model
{
/**
 * @brief Abstract base class for kinematic model plugins.
 *
 * This class defines the standard interface for all kinematic models that can be loaded
 * by the GenericSteeringController.
 */
class KinematicModelBase
{
public:
  virtual ~KinematicModelBase() = default;

  /**
   * @brief Configure the kinematic model plugin.
   * @param node The node interface for the controller.
   * @param traction_joint_names A list of names for the traction (drive) joints.
   * @param steering_joint_names A list of names for the steering joints.
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::vector<std::string> & traction_joint_names,
    const std::vector<std::string> & steering_joint_names) = 0;

  /**
   * @brief Update the internal reference command from the subscriber.
   * @param twist_msg The command message from the subscriber.
   * @param time The current time.
   */
  virtual void update_reference(
    const geometry_msgs::msg::Twist & twist_msg, const rclcpp::Time & time) = 0;
  /**
   * @brief Get the latest odometry message calculated by the plugin.
   * @param time The current time.
   * @return A unique_ptr to the odometry message.
   */
  virtual std::unique_ptr<nav_msgs::msg::Odometry> get_odometry_message(
    const rclcpp::Duration & time) = 0;

  /**
   * \brief Calculates inverse kinematics for the desired linear and angular velocities
   * \param v_bx     Desired linear velocity of the robot in x_b-axis direction
   * \param omega_bz Desired angular velocity of the robot around x_z-axis
   * \param open_loop If false, the IK will be calculated using measured steering angle
   * \param reduce_wheel_speed_until_steering_reached Reduce wheel speed until the steering angle
   * has been reached
   * \return Tuple of velocity commands and steering commands
   */
  virtual std::tuple<std::vector<double>, std::vector<double>> get_commands(
    const double v_bx, const double omega_bz, const bool open_loop = true,
    const bool reduce_wheel_speed_until_steering_reached = false) = 0;

   /**
   * @brief Update all joint states at once.
   * @param all_states Vector containing all measured joint states (traction first, then steering).
   */
  virtual void update_states(const std::unordered_map<std::string, double> & all_states)
  {
    states = all_states;
  }

protected:
  // Node for plugins to get parameters, etc.
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Store joint names for later use
  std::vector<std::string> traction_joint_names_;
  std::vector<std::string> steering_joint_names_;

  // Store the latest state from the hardware
  std::vector<double> traction_states_;
  std::vector<double> steering_states_;

  std::unordered_map<std::string, double> states;
};

}  // namespace generic_steering_controller

#endif  // GENERIC_STEERING_CONTROLLER__KINEMATIC_MODEL_BASE_HPP_
