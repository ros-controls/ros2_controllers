/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   gravity_compensation_pd_controller.hpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Sept 4, 2025
 *
 * This module implements a PD controller with gravity compensation.
 *
 * -------------------------------------------------------------------
 */

#ifndef GRAVITY_COMPENSATION_PD_CONTROLLER__GRAVITY_COMPENSATION_PD_CONTROLLER_HPP_
#define GRAVITY_COMPENSATION_PD_CONTROLLER__GRAVITY_COMPENSATION_PD_CONTROLLER_HPP_

#include <Eigen/Core>

#include <rclcpp/duration.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <pluginlib/class_loader.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <inverse_dynamics_solver/inverse_dynamics_solver.h>

#include "gravity_compensation_pd_controller/gravity_compensation_pd_controller_parameters.hpp"
namespace gravity_compensation_pd_controller
{

struct RobotJointState
{
  std::vector<double> positions;
  std::vector<double> velocities;
};

/**
 * @class GravityCompensationPDController
 * @brief A class representing a PD controller with gravity compensation.
 *
 * This class implements the \c controller_interface::ChainableControllerInterface interface.
 * All of the public methods override the corresponding methods of the \c controller_interface::ChainableControllerInterface class.
 * Please refer to the documentation of the base class for more details.
 */
class GravityCompensationPDController : public controller_interface::ChainableControllerInterface
{
public:

  GravityCompensationPDController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

protected:
  // The following methods are overridden from the base class. Refer to the base class documentation for details.
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  bool on_set_chained_mode(bool chained_mode) override;
  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * @brief Computes the joint effort command using a PD control law with optional gravity compensation.
   *
   * This method calculates the joint effort command according to the control law described in the README file.
   * If \c compensate_gravity parameter is disabled, the control law reduces to a standard PD controller.
   * The resulting effort command is stored in the \c joint_command_ attribute.
   */
  void compute_control_law_();

  /**
   * @brief Reads the state interfaces and updates the internal joint state.
   *
   * @return True if successful, false otherwise.
   */
  [[nodiscard]] bool read_state_interfaces_();

  /**
   * @brief Writes the command interfaces with the computed joint effort command.
   *
   * @return True if successful, false otherwise.
   */
  [[nodiscard]] bool write_command_interfaces_();

  /**
   * @brief Reads the reference interfaces and updates the internal joint reference.
   *
   * @return True if successful, false otherwise.
   */
  [[nodiscard]] bool read_reference_interfaces_();

  /**
   * @brief Reads the joint effort limits from the URDF file populating the torque_limits_ member.
   *
   * @return True if successful, false otherwise.
   */
  bool read_joint_effort_limits_from_urdf();

  /**
   * @brief Plugin loader for the inverse dynamics solver.
   */
  pluginlib::ClassLoader<inverse_dynamics_solver::InverseDynamicsSolver> dynamics_solver_loader_;

  /**
   * @brief Shared pointer to the inverse dynamics solver used for gravity compensation.
   */
  std::shared_ptr<inverse_dynamics_solver::InverseDynamicsSolver> dynamics_solver_;

  /**
   * @brief Variables to store the joint command, reference, and last reference.
   */
  std::vector<double> joint_command_, joint_reference_, last_joint_reference_;

  /**
   * @brief Shared pointer to the parameter listener responsible for handling the controller's parameters.
   */
  std::shared_ptr<gravity_compensation_pd_controller::ParamListener> parameter_handler_;

  /**
   * @brief Vector to store the torque limits for each joint.
   */
  Eigen::VectorXd torque_limits_;

  /**
   * @brief Eigen vector to store the position error between the reference and current joint positions.
   */
  Eigen::VectorXd position_error_;

  /**
   * @brief Eigen vector to store the computed joint effort command.
   */
  Eigen::VectorXd eigen_effort_command_;

  /**
   * @brief Diagonal matrix for proportional gains (Kp) used in the PD control law.
   */
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> Kp_;

  /**
   * @brief Diagonal matrix for derivative gains (Kd) used in the PD control law.
   */
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> Kd_;

  /**
   * @brief Constant to store the duration of the throttle interval as an integral value in milliseconds.
   */
  static constexpr unsigned short DURATION_MS_{ 1000 };

  /**
   * @brief Number of joints to control.
   */
  std::size_t num_joints_{ 0 };

  /**
   * @brief Internal variable to store the current joint state of the robot.
   */
  RobotJointState robot_joint_state_;

};

}  // namespace gravity_compensation_pd_controller

#endif  // GRAVITY_COMPENSATION_PD_CONTROLLER__GRAVITY_COMPENSATION_PD_CONTROLLER_HPP_
