// Copyright 2024 ros2_control Development Team
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

#ifndef TEST_MTTC_UTILS_HPP_
#define TEST_MTTC_UTILS_HPP_

#include <gmock/gmock.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <rclcpp/executor.hpp>
#include "control_msgs/msg/axis_trajectory_point.hpp"
#include "control_msgs/msg/multi_axis_trajectory.hpp"
#include "control_msgs/msg/multi_time_trajectory_controller_state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "multi_time_trajectory_controller/multi_time_trajectory_controller.hpp"

namespace
{
std::size_t const DOF = 3;
const double COMMON_THRESHOLD = 0.001;
const double INITIAL_POS_JOINT1 = 1.1;
const double INITIAL_POS_JOINT2 = 2.1;
const double INITIAL_POS_JOINT3 = 3.1;
const std::vector<double> INITIAL_POS_JOINTS = {
  INITIAL_POS_JOINT1, INITIAL_POS_JOINT2, INITIAL_POS_JOINT3};
const std::vector<double> INITIAL_VEL_JOINTS = {0.0, 0.0, 0.0};
const std::vector<double> INITIAL_ACC_JOINTS = {0.0, 0.0, 0.0};
const std::vector<double> INITIAL_EFF_JOINTS = {0.0, 0.0, 0.0};

bool is_same_sign_or_zero(double val1, double val2)
{
  return val1 * val2 > 0.0 || (val1 == 0.0 && val2 == 0.0);
}

}  // namespace

namespace test_mttc
{
class TestableMultiTimeTrajectoryController
: public multi_time_trajectory_controller::MultiTimeTrajectoryController
{
public:
  using multi_time_trajectory_controller::MultiTimeTrajectoryController::
    MultiTimeTrajectoryController;
  using multi_time_trajectory_controller::MultiTimeTrajectoryController::validate_trajectory_msg;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret =
      multi_time_trajectory_controller::MultiTimeTrajectoryController::on_configure(previous_state);
    return ret;
  }

  rclcpp::NodeOptions define_custom_node_options() const override { return node_options_; }

  /**
   * @brief wait_for_trajectory block until a new JointTrajectory is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function
   */
  void wait_for_trajectory(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{10})
  {
    auto until = get_node()->get_clock()->now() + timeout;
    while (get_node()->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  void set_axis_names(const std::vector<std::string> & axis_names) { params_.axes = axis_names; }

  void set_command_axis_names(const std::vector<std::string> & command_axis_names)
  {
    command_axis_names_ = command_axis_names;
  }

  void set_command_interfaces(const std::vector<std::string> & command_interfaces)
  {
    params_.command_interfaces = command_interfaces;
  }

  void set_state_interfaces(const std::vector<std::string> & state_interfaces)
  {
    params_.state_interfaces = state_interfaces;
  }

  void trigger_declare_parameters() { param_listener_->declare_params(); }

  void testable_compute_error(
    std::vector<control_msgs::msg::AxisTrajectoryPoint> & error,
    const std::vector<control_msgs::msg::AxisTrajectoryPoint> & current,
    const std::vector<control_msgs::msg::AxisTrajectoryPoint> & desired)
  {
    compute_error(error, current, desired);
  }

  std::vector<control_msgs::msg::AxisTrajectoryPoint> get_current_state_when_offset() const
  {
    return last_commanded_state_;
  }
  bool has_position_state_interface() const { return has_position_state_interface_; }

  bool has_velocity_state_interface() const { return has_velocity_state_interface_; }

  bool has_acceleration_state_interface() const { return has_acceleration_state_interface_; }

  bool has_position_command_interface() const { return has_position_command_interface_; }

  bool has_velocity_command_interface() const { return has_velocity_command_interface_; }

  bool has_acceleration_command_interface() const { return has_acceleration_command_interface_; }

  bool has_effort_command_interface() const { return has_effort_command_interface_; }

  bool use_closed_loop_pid_adapter() const { return use_closed_loop_pid_adapter_; }

  bool is_open_loop() const { return params_.open_loop_control; }

  std::vector<PidPtr> get_pids() const { return pids_; }

  multi_time_trajectory_controller::SegmentTolerances get_tolerances() const
  {
    return default_tolerances_;
  }

  bool has_active_traj() const { return has_active_trajectory(); }

  bool has_trivial_traj(std::size_t axis) const
  {
    return has_active_trajectory() && traj_external_point_ptr_->has_nontrivial_msg(axis) == false;
  }

  bool has_nontrivial_traj(std::size_t axis)
  {
    return has_active_trajectory() && traj_external_point_ptr_->has_nontrivial_msg(axis);
  }

  double get_cmd_timeout() { return cmd_timeout_; }

  void set_node_options(const rclcpp::NodeOptions & node_options) { node_options_ = node_options; }

  std::vector<control_msgs::msg::AxisTrajectoryPoint> get_state_feedback()
  {
    return state_current_;
  }
  std::vector<control_msgs::msg::AxisTrajectoryPoint> get_state_reference()
  {
    return state_desired_;
  }
  std::vector<control_msgs::msg::AxisTrajectoryPoint> get_state_error() { return state_error_; }

  rclcpp::NodeOptions node_options_;
};

class TrajectoryControllerTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  virtual void SetUp()
  {
    controller_name_ = "test_joint_trajectory_controller";

    axis_names_ = {"joint1", "joint2", "joint3"};
    command_axis_names_ = {
      "following_controller/joint1", "following_controller/joint2", "following_controller/joint3"};
    joint_pos_.resize(axis_names_.size(), 0.0);
    joint_state_pos_.resize(axis_names_.size(), 0.0);
    joint_vel_.resize(axis_names_.size(), 0.0);
    joint_state_vel_.resize(axis_names_.size(), 0.0);
    joint_acc_.resize(axis_names_.size(), 0.0);
    joint_state_acc_.resize(axis_names_.size(), 0.0);
    joint_eff_.resize(axis_names_.size(), 0.0);
    // Default interface values - they will be overwritten by parameterized tests
    command_interface_types_ = {"position"};
    state_interface_types_ = {"position", "velocity"};

    node_ = std::make_shared<rclcpp::Node>("trajectory_publisher_");
    trajectory_publisher_ = node_->create_publisher<control_msgs::msg::MultiAxisTrajectory>(
      controller_name_ + "/joint_trajectory", rclcpp::SystemDefaultsQoS());
  }

  void SetUpTrajectoryController(
    rclcpp::Executor & executor, const std::vector<rclcpp::Parameter> & parameters = {},
    const std::string & urdf = "")
  {
    auto ret = SetUpTrajectoryControllerLocal(parameters, urdf);
    if (ret != controller_interface::return_type::OK)
    {
      FAIL();
    }
    executor.add_node(traj_controller_->get_node()->get_node_base_interface());
  }

  controller_interface::return_type SetUpTrajectoryControllerLocal(
    const std::vector<rclcpp::Parameter> & parameters = {}, const std::string & urdf = "")
  {
    traj_controller_ = std::make_shared<TestableMultiTimeTrajectoryController>();

    auto node_options = rclcpp::NodeOptions();
    std::vector<rclcpp::Parameter> parameter_overrides;
    parameter_overrides.push_back(rclcpp::Parameter("axes", axis_names_));
    parameter_overrides.push_back(
      rclcpp::Parameter("command_interfaces", command_interface_types_));
    parameter_overrides.push_back(rclcpp::Parameter("state_interfaces", state_interface_types_));
    parameter_overrides.insert(parameter_overrides.end(), parameters.begin(), parameters.end());
    node_options.parameter_overrides(parameter_overrides);
    traj_controller_->set_node_options(node_options);

    return traj_controller_->init(
      controller_name_, urdf, 0, "", traj_controller_->define_custom_node_options());
  }

  void SetPidParameters(double p_value = 0.0, double ff_value = 1.0)
  {
    traj_controller_->trigger_declare_parameters();
    auto node = traj_controller_->get_node();

    for (size_t i = 0; i < axis_names_.size(); ++i)
    {
      const std::string prefix = "gains." + axis_names_[i];
      const rclcpp::Parameter k_p(prefix + ".p", p_value);
      const rclcpp::Parameter k_i(prefix + ".i", 0.0);
      const rclcpp::Parameter k_d(prefix + ".d", 0.0);
      const rclcpp::Parameter i_clamp(prefix + ".i_clamp", 0.0);
      const rclcpp::Parameter ff_velocity_scale(prefix + ".ff_velocity_scale", ff_value);
      node->set_parameters({k_p, k_i, k_d, i_clamp, ff_velocity_scale});
    }
  }

  void SetUpAndActivateTrajectoryController(
    rclcpp::Executor & executor, const std::vector<rclcpp::Parameter> & parameters = {},
    bool separate_cmd_and_state_values = false, double k_p = 0.0, double ff = 1.0,
    const std::vector<double> & initial_pos_joints = INITIAL_POS_JOINTS,
    const std::vector<double> & initial_vel_joints = INITIAL_VEL_JOINTS,
    const std::vector<double> & initial_acc_joints = INITIAL_ACC_JOINTS,
    const std::vector<double> & initial_eff_joints = INITIAL_EFF_JOINTS,
    const std::string & urdf = "")
  {
    auto has_nonzero_vel_param =
      std::find_if(
        parameters.begin(), parameters.end(), [](const rclcpp::Parameter & param)
        { return param.get_name() == "allow_nonzero_velocity_at_trajectory_end"; }) !=
      parameters.end();

    std::vector<rclcpp::Parameter> parameters_local = parameters;
    if (!has_nonzero_vel_param)
    {
      // add this to simplify tests, if not set already
      parameters_local.emplace_back("allow_nonzero_velocity_at_trajectory_end", true);
    }
    // read-only parameters have to be set before init -> won't be read otherwise
    SetUpTrajectoryController(executor, parameters_local, urdf);

    // set pid parameters before configure
    SetPidParameters(k_p, ff);
    traj_controller_->get_node()->configure();

    ActivateTrajectoryController(
      separate_cmd_and_state_values, initial_pos_joints, initial_vel_joints, initial_acc_joints,
      initial_eff_joints);
  }

  rclcpp_lifecycle::State ActivateTrajectoryController(
    bool separate_cmd_and_state_values = false,
    const std::vector<double> & initial_pos_joints = INITIAL_POS_JOINTS,
    const std::vector<double> & initial_vel_joints = INITIAL_VEL_JOINTS,
    const std::vector<double> & initial_acc_joints = INITIAL_ACC_JOINTS,
    const std::vector<double> & initial_eff_joints = INITIAL_EFF_JOINTS)
  {
    std::vector<hardware_interface::LoanedCommandInterface> cmd_interfaces;
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    pos_cmd_interfaces_.reserve(axis_names_.size());
    vel_cmd_interfaces_.reserve(axis_names_.size());
    acc_cmd_interfaces_.reserve(axis_names_.size());
    eff_cmd_interfaces_.reserve(axis_names_.size());
    pos_state_interfaces_.reserve(axis_names_.size());
    vel_state_interfaces_.reserve(axis_names_.size());
    acc_state_interfaces_.reserve(axis_names_.size());
    for (size_t i = 0; i < axis_names_.size(); ++i)
    {
      pos_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        axis_names_[i], hardware_interface::HW_IF_POSITION, &joint_pos_[i]));
      vel_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        axis_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_vel_[i]));
      acc_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        axis_names_[i], hardware_interface::HW_IF_ACCELERATION, &joint_acc_[i]));
      eff_cmd_interfaces_.emplace_back(hardware_interface::CommandInterface(
        axis_names_[i], hardware_interface::HW_IF_EFFORT, &joint_eff_[i]));

      pos_state_interfaces_.emplace_back(hardware_interface::StateInterface(
        axis_names_[i], hardware_interface::HW_IF_POSITION,
        separate_cmd_and_state_values ? &joint_state_pos_[i] : &joint_pos_[i]));
      vel_state_interfaces_.emplace_back(hardware_interface::StateInterface(
        axis_names_[i], hardware_interface::HW_IF_VELOCITY,
        separate_cmd_and_state_values ? &joint_state_vel_[i] : &joint_vel_[i]));
      acc_state_interfaces_.emplace_back(hardware_interface::StateInterface(
        axis_names_[i], hardware_interface::HW_IF_ACCELERATION,
        separate_cmd_and_state_values ? &joint_state_acc_[i] : &joint_acc_[i]));

      // Add to export lists and set initial values
      cmd_interfaces.emplace_back(pos_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(initial_pos_joints[i]);
      cmd_interfaces.emplace_back(vel_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(initial_vel_joints[i]);
      cmd_interfaces.emplace_back(acc_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(initial_acc_joints[i]);
      cmd_interfaces.emplace_back(eff_cmd_interfaces_.back());
      cmd_interfaces.back().set_value(initial_eff_joints[i]);
      if (separate_cmd_and_state_values)
      {
        joint_state_pos_[i] = INITIAL_POS_JOINTS[i];
        joint_state_vel_[i] = INITIAL_VEL_JOINTS[i];
        joint_state_acc_[i] = INITIAL_ACC_JOINTS[i];
      }
      state_interfaces.emplace_back(pos_state_interfaces_.back());
      state_interfaces.emplace_back(vel_state_interfaces_.back());
      state_interfaces.emplace_back(acc_state_interfaces_.back());
    }

    traj_controller_->assign_interfaces(std::move(cmd_interfaces), std::move(state_interfaces));
    return traj_controller_->get_node()->activate();
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void subscribeToState(rclcpp::Executor & executor)
  {
    auto traj_lifecycle_node = traj_controller_->get_node();

    using control_msgs::msg::MultiTimeTrajectoryControllerState;

    auto qos = rclcpp::SensorDataQoS();
    // Needed, otherwise spin_some() returns only the oldest message in the queue
    // I do not understand why spin_some provides only one message
    qos.keep_last(1);
    state_subscriber_ =
      traj_lifecycle_node->create_subscription<MultiTimeTrajectoryControllerState>(
        controller_name_ + "/controller_state", qos,
        [&](std::shared_ptr<MultiTimeTrajectoryControllerState> msg)
        {
          std::lock_guard<std::mutex> guard(state_mutex_);
          state_msg_ = msg;
        });
    const auto timeout = std::chrono::milliseconds{10};
    const auto until = traj_lifecycle_node->get_clock()->now() + timeout;
    while (traj_lifecycle_node->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  /// Publish trajectory msgs with multiple points
  /**
   *  delay_btwn_points - delay between each point
   *  points_positions - vector of trajectory-positions. One point per controlled joint
   *  joint_names - names of joints, if empty, will use joint_names_ up to the number of provided
   * points
   *  points - vector of trajectory-velocities. One point per controlled joint
   */
  void publish(
    const builtin_interfaces::msg::Duration & delay_btwn_points,
    const std::vector<std::vector<double>> & points_positions, rclcpp::Time start_time,
    const std::vector<std::string> & joint_names = {},
    const std::vector<std::vector<double>> & points_velocities = {})
  {
    int wait_count = 0;
    const auto topic = trajectory_publisher_->get_topic_name();
    while (node_->count_subscribers(topic) == 0)
    {
      if (wait_count >= 5)
      {
        auto error_msg = std::string("publishing to ") + topic + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    control_msgs::msg::MultiAxisTrajectory multi_traj_msg;
    std::size_t num_axes = axis_names_.size();
    multi_traj_msg.axis_trajectories.resize(num_axes);
    if (joint_names.empty())
    {
      multi_traj_msg.axis_names = {
        axis_names_.begin(),
        axis_names_.begin() + static_cast<int64_t>(points_positions[0].size())};
    }
    else
    {
      multi_traj_msg.axis_names = joint_names;
    }
    multi_traj_msg.header.stamp = start_time;

    for (std::size_t i = 0; i < num_axes; ++i)
    {
      auto & traj_msg = multi_traj_msg.axis_trajectories[i];

      traj_msg.axis_points.resize(
        points_positions.size(), multi_time_trajectory_controller::emptyTrajectoryPoint());

      builtin_interfaces::msg::Duration duration_msg;
      duration_msg.sec = delay_btwn_points.sec;
      duration_msg.nanosec = delay_btwn_points.nanosec;
      rclcpp::Duration duration(duration_msg);
      rclcpp::Duration duration_total(duration_msg);

      for (size_t index = 0; index < points_positions.size(); ++index)
      {
        traj_msg.axis_points[index].time_from_start = duration_total;

        traj_msg.axis_points[index].position = points_positions[index][i];
        traj_msg.axis_points[index].time_from_start = duration_total;
        duration_total += duration;
      }

      for (size_t index = 0; index < points_velocities.size(); ++index)
      {
        traj_msg.axis_points[index].velocity = points_velocities[index][i];
      }
    }

    trajectory_publisher_->publish(multi_traj_msg);
  }

  /**
   * @brief a wrapper for update() method of JTC, running synchronously with the clock
   * @param wait_time - the time span for updating the controller
   * @param update_rate - the rate at which the controller is updated
   *
   * @note use the faster updateControllerAsync() if no subscriptions etc.
   * have to be used from the waitSet/executor
   */
  void updateController(
    rclcpp::Duration wait_time = rclcpp::Duration::from_seconds(0.2),
    const rclcpp::Duration update_rate = rclcpp::Duration::from_seconds(0.01))
  {
    auto clock = rclcpp::Clock(RCL_STEADY_TIME);
    const auto start_time = clock.now();
    const auto end_time = start_time + wait_time;
    auto previous_time = start_time;

    while (clock.now() <= end_time)
    {
      auto now = clock.now();
      traj_controller_->update(now, now - previous_time);
      previous_time = now;
      std::this_thread::sleep_for(update_rate.to_chrono<std::chrono::milliseconds>());
    }
  }

  /**
   * @brief a wrapper for update() method of JTC, running asynchronously from the clock
   * @return the time at which the update finished
   * @param wait_time - the time span for updating the controller
   * @param start_time - the time at which the update should start
   * @param update_rate - the rate at which the controller is updated
   *
   * @note this is faster than updateController() and can be used if no subscriptions etc.
   * have to be used from the waitSet/executor
   */
  rclcpp::Time updateControllerAsync(
    rclcpp::Duration wait_time = rclcpp::Duration::from_seconds(0.2),
    rclcpp::Time start_time = rclcpp::Time(0, 0, RCL_STEADY_TIME),
    const rclcpp::Duration update_rate = rclcpp::Duration::from_seconds(0.01))
  {
    if (start_time == rclcpp::Time(0, 0, RCL_STEADY_TIME))
    {
      start_time = rclcpp::Clock(RCL_STEADY_TIME).now();
    }
    const auto end_time = start_time + wait_time;
    auto time_counter = start_time;
    while (time_counter <= end_time)
    {
      traj_controller_->update(time_counter, update_rate);
      time_counter += update_rate;
    }
    return end_time;
  }

  rclcpp::Time waitAndCompareState(
    std::vector<control_msgs::msg::AxisTrajectoryPoint> const & expected_actual,
    std::vector<control_msgs::msg::AxisTrajectoryPoint> const & expected_desired,
    rclcpp::Executor & executor, rclcpp::Duration controller_wait_time, double allowed_delta,
    rclcpp::Time start_time = rclcpp::Time(0, 0, RCL_STEADY_TIME))
  {
    {
      std::lock_guard<std::mutex> guard(state_mutex_);
      state_msg_.reset();
    }
    traj_controller_->wait_for_trajectory(executor);
    auto end_time = updateControllerAsync(controller_wait_time, start_time);

    // get states from class variables
    auto state_feedback = traj_controller_->get_state_feedback();
    auto state_reference = traj_controller_->get_state_reference();

    for (size_t i = 0; i < expected_actual.size(); ++i)
    {
      SCOPED_TRACE("Joint " + std::to_string(i));
      // TODO(anyone): add checking for velocities and accelerations
      if (traj_controller_->has_position_command_interface())
      {
        EXPECT_NEAR(expected_actual[i].position, state_feedback[i].position, allowed_delta);
      }
    }

    for (size_t i = 0; i < expected_desired.size(); ++i)
    {
      SCOPED_TRACE("Joint " + std::to_string(i));
      // TODO(anyone): add checking for velocities and accelerations
      if (traj_controller_->has_position_command_interface())
      {
        EXPECT_NEAR(expected_desired[i].position, state_reference[i].position, allowed_delta);
      }
    }

    return end_time;
  }

  std::shared_ptr<control_msgs::msg::MultiTimeTrajectoryControllerState> getState() const
  {
    std::lock_guard<std::mutex> guard(state_mutex_);
    return state_msg_;
  }

  void expectCommandPoint(
    std::vector<double> const & position, std::size_t axis,
    std::vector<double> const & velocity = {0.0, 0.0, 0.0})
  {
    // it should be holding the given point
    // i.e., active but trivial trajectory (one point only)
    EXPECT_TRUE(traj_controller_->has_trivial_traj(axis));

    if (traj_controller_->use_closed_loop_pid_adapter() == false)
    {
      if (traj_controller_->has_position_command_interface())
      {
        EXPECT_NEAR(position.at(0), joint_pos_[0], COMMON_THRESHOLD);
        EXPECT_NEAR(position.at(1), joint_pos_[1], COMMON_THRESHOLD);
        EXPECT_NEAR(position.at(2), joint_pos_[2], COMMON_THRESHOLD);
      }

      if (traj_controller_->has_velocity_command_interface())
      {
        EXPECT_EQ(velocity.at(0), joint_vel_[0]);
        EXPECT_EQ(velocity.at(1), joint_vel_[1]);
        EXPECT_EQ(velocity.at(2), joint_vel_[2]);
      }

      if (traj_controller_->has_acceleration_command_interface())
      {
        EXPECT_EQ(0.0, joint_acc_[0]);
        EXPECT_EQ(0.0, joint_acc_[1]);
        EXPECT_EQ(0.0, joint_acc_[2]);
      }

      if (traj_controller_->has_effort_command_interface())
      {
        EXPECT_EQ(0.0, joint_eff_[0]);
        EXPECT_EQ(0.0, joint_eff_[1]);
        EXPECT_EQ(0.0, joint_eff_[2]);
      }
    }
    else  // traj_controller_->use_closed_loop_pid_adapter() == true
    {
      // velocity or effort PID?
      // --> set kp > 0.0 in test
      if (traj_controller_->has_velocity_command_interface())
      {
        for (size_t i = 0; i < 3; i++)
        {
          EXPECT_TRUE(is_same_sign_or_zero(
            position.at(i) - pos_state_interfaces_[i].get_value(), joint_vel_[i]))
            << "test position point " << position.at(i) << ", position state is "
            << pos_state_interfaces_[i].get_value() << ", velocity command is " << joint_vel_[i];
        }
      }
      if (traj_controller_->has_effort_command_interface())
      {
        for (size_t i = 0; i < 3; i++)
        {
          EXPECT_TRUE(is_same_sign_or_zero(
            position.at(i) - pos_state_interfaces_[i].get_value(), joint_eff_[i]))
            << "test position point " << position.at(i) << ", position state is "
            << pos_state_interfaces_[i].get_value() << ", effort command is " << joint_eff_[i];
        }
      }
    }
  }

  void expectHoldingPointDeactivated(std::vector<double> const & point)
  {
    // it should be holding the given point, but no active trajectory
    EXPECT_FALSE(traj_controller_->has_active_traj());

    if (traj_controller_->has_position_command_interface())
    {
      EXPECT_NEAR(point.at(0), joint_pos_[0], COMMON_THRESHOLD);
      EXPECT_NEAR(point.at(1), joint_pos_[1], COMMON_THRESHOLD);
      EXPECT_NEAR(point.at(2), joint_pos_[2], COMMON_THRESHOLD);
    }

    if (traj_controller_->has_velocity_command_interface())
    {
      EXPECT_EQ(0.0, joint_vel_[0]);
      EXPECT_EQ(0.0, joint_vel_[1]);
      EXPECT_EQ(0.0, joint_vel_[2]);
    }

    if (traj_controller_->has_acceleration_command_interface())
    {
      EXPECT_EQ(0.0, joint_acc_[0]);
      EXPECT_EQ(0.0, joint_acc_[1]);
      EXPECT_EQ(0.0, joint_acc_[2]);
    }

    if (traj_controller_->has_effort_command_interface())
    {
      EXPECT_EQ(0.0, joint_eff_[0]);
      EXPECT_EQ(0.0, joint_eff_[1]);
      EXPECT_EQ(0.0, joint_eff_[2]);
    }
  }

  /**
   * @brief compares the joint names and interface types of the controller with the given ones
   */
  void compare_joints(
    std::vector<std::string> const & state_joint_names,
    std::vector<std::string> const & command_joint_names)
  {
    std::vector<std::string> state_interface_names;
    state_interface_names.reserve(state_joint_names.size() * state_interface_types_.size());
    for (const auto & joint : state_joint_names)
    {
      for (const auto & interface : state_interface_types_)
      {
        state_interface_names.push_back(joint + "/" + interface);
      }
    }
    auto state_interfaces = traj_controller_->state_interface_configuration();
    EXPECT_EQ(
      state_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);
    EXPECT_EQ(
      state_interfaces.names.size(), state_joint_names.size() * state_interface_types_.size());
    ASSERT_THAT(state_interfaces.names, testing::UnorderedElementsAreArray(state_interface_names));

    std::vector<std::string> command_interface_names;
    command_interface_names.reserve(command_joint_names.size() * command_interface_types_.size());
    for (const auto & joint : command_joint_names)
    {
      for (const auto & interface : command_interface_types_)
      {
        command_interface_names.push_back(joint + "/" + interface);
      }
    }
    auto command_interfaces = traj_controller_->command_interface_configuration();
    EXPECT_EQ(
      command_interfaces.type, controller_interface::interface_configuration_type::INDIVIDUAL);
    EXPECT_EQ(
      command_interfaces.names.size(),
      command_joint_names.size() * command_interface_types_.size());
    ASSERT_THAT(
      command_interfaces.names, testing::UnorderedElementsAreArray(command_interface_names));
  }

  std::string controller_name_;

  std::vector<std::string> axis_names_;
  std::vector<std::string> command_axis_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<control_msgs::msg::MultiAxisTrajectory>::SharedPtr trajectory_publisher_;

  std::shared_ptr<TestableMultiTimeTrajectoryController> traj_controller_;
  rclcpp::Subscription<control_msgs::msg::MultiTimeTrajectoryControllerState>::SharedPtr
    state_subscriber_;
  mutable std::mutex state_mutex_;
  std::shared_ptr<control_msgs::msg::MultiTimeTrajectoryControllerState> state_msg_;

  std::vector<control_msgs::msg::AxisTrajectoryPoint> axis_state;
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_acc_;
  std::vector<double> joint_eff_;
  std::vector<double> joint_state_pos_;
  std::vector<double> joint_state_vel_;
  std::vector<double> joint_state_acc_;
  std::vector<hardware_interface::CommandInterface> pos_cmd_interfaces_;
  std::vector<hardware_interface::CommandInterface> vel_cmd_interfaces_;
  std::vector<hardware_interface::CommandInterface> acc_cmd_interfaces_;
  std::vector<hardware_interface::CommandInterface> eff_cmd_interfaces_;
  std::vector<hardware_interface::StateInterface> pos_state_interfaces_;
  std::vector<hardware_interface::StateInterface> vel_state_interfaces_;
  std::vector<hardware_interface::StateInterface> acc_state_interfaces_;
};

// From the tutorial: https://www.sandordargo.com/blog/2019/04/24/parameterized-testing-with-gtest
class TrajectoryControllerTestParameterized
: public TrajectoryControllerTest,
  public ::testing::WithParamInterface<
    std::tuple<std::vector<std::string>, std::vector<std::string>>>
{
public:
  virtual void SetUp()
  {
    TrajectoryControllerTest::SetUp();
    command_interface_types_ = std::get<0>(GetParam());
    state_interface_types_ = std::get<1>(GetParam());
  }

  static void TearDownTestCase() { TrajectoryControllerTest::TearDownTestCase(); }
};

}  // namespace test_mttc

#endif  // TEST_MTTC_UTILS_HPP_
