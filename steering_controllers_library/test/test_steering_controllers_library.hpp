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

#ifndef TEST_STEERING_CONTROLLERS_LIBRARY_HPP_
#define TEST_STEERING_CONTROLLERS_LIBRARY_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "steering_controllers_library/steering_controllers_library.hpp"

using ControllerStateMsg =
  steering_controllers_library::SteeringControllersLibrary::AckermanControllerState;
using ControllerReferenceMsg =
  steering_controllers_library::SteeringControllersLibrary::ControllerTwistReferenceMsg;

// NOTE: Testing steering_controllers_library for ackermann vehicle configuration only

// name constants for state interfaces
static constexpr size_t STATE_TRACTION_RIGHT_WHEEL = 0;
static constexpr size_t STATE_TRACTION_LEFT_WHEEL = 1;
static constexpr size_t STATE_STEER_RIGHT_WHEEL = 2;
static constexpr size_t STATE_STEER_LEFT_WHEEL = 3;

// name constants for command interfaces
static constexpr size_t CMD_TRACTION_RIGHT_WHEEL = 0;
static constexpr size_t CMD_TRACTION_LEFT_WHEEL = 1;
static constexpr size_t CMD_STEER_RIGHT_WHEEL = 2;
static constexpr size_t CMD_STEER_LEFT_WHEEL = 3;

static constexpr size_t NR_STATE_ITFS = 4;
static constexpr size_t NR_CMD_ITFS = 4;
static constexpr size_t NR_REF_ITFS = 2;

static constexpr double WHEELBASE_ = 3.24644;
static constexpr double FRONT_WHEEL_TRACK_ = 2.12321;
static constexpr double REAR_WHEEL_TRACK_ = 1.76868;
static constexpr double FRONT_WHEELS_RADIUS_ = 0.45;
static constexpr double REAR_WHEELS_RADIUS_ = 0.45;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace
// namespace

// subclassing and friending so we can access member variables
class TestableSteeringControllersLibrary
: public steering_controllers_library::SteeringControllersLibrary
{
  FRIEND_TEST(SteeringControllersLibraryTest, check_exported_intefaces);
  FRIEND_TEST(SteeringControllersLibraryTest, test_both_update_methods_for_ref_timeout);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret =
      steering_controllers_library::SteeringControllersLibrary::on_configure(previous_state);
    // Only if on_configure is successful create subscription
    if (ret == CallbackReturn::SUCCESS)
    {
      ref_subscriber_wait_set_.add_subscription(ref_subscriber_twist_);
    }
    return ret;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ref_itfs = on_export_reference_interfaces();
    return steering_controllers_library::SteeringControllersLibrary::on_activate(previous_state);
  }

  /**
   * @brief wait_for_command blocks until a new ControllerReferenceMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   *
   * @return true if new ControllerReferenceMsg msg was received, false if timeout.
   */
  bool wait_for_command(
    rclcpp::Executor & executor, rclcpp::WaitSet & subscriber_wait_set,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    bool success = subscriber_wait_set.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
    if (success)
    {
      executor.spin_some();
    }
    return success;
  }

  bool wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    return wait_for_command(executor, ref_subscriber_wait_set_, timeout);
  }

  // implementing methods which are declared virtual in the steering_controllers_library.hpp
  void initialize_implementation_parameter_listener()
  {
    param_listener_ = std::make_shared<steering_controllers_library::ParamListener>(get_node());
  }

  controller_interface::CallbackReturn configure_odometry()
  {
    set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);
    odometry_.set_wheel_params(FRONT_WHEELS_RADIUS_, WHEELBASE_, REAR_WHEEL_TRACK_);
    odometry_.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool update_odometry(const rclcpp::Duration & /*period*/) { return true; }

private:
  rclcpp::WaitSet ref_subscriber_wait_set_;
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class SteeringControllersLibraryFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      "/test_steering_controllers_library/reference", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(const std::string controller_name = "test_steering_controllers_library")
  {
    ASSERT_EQ(controller_->init(controller_name), controller_interface::return_type::OK);

    if (position_feedback_ == true)
    {
      traction_interface_name_ = "position";
    }
    else
    {
      traction_interface_name_ = "velocity";
    }

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    command_itfs_.emplace_back(hardware_interface::CommandInterface(
      rear_wheels_names_[0], traction_interface_name_,
      &joint_command_values_[CMD_TRACTION_RIGHT_WHEEL]));
    command_ifs.emplace_back(command_itfs_.back());

    command_itfs_.emplace_back(hardware_interface::CommandInterface(
      rear_wheels_names_[1], steering_interface_name_,
      &joint_command_values_[CMD_TRACTION_LEFT_WHEEL]));
    command_ifs.emplace_back(command_itfs_.back());

    command_itfs_.emplace_back(hardware_interface::CommandInterface(
      front_wheels_names_[0], steering_interface_name_,
      &joint_command_values_[CMD_STEER_RIGHT_WHEEL]));
    command_ifs.emplace_back(command_itfs_.back());

    command_itfs_.emplace_back(hardware_interface::CommandInterface(
      front_wheels_names_[1], steering_interface_name_,
      &joint_command_values_[CMD_STEER_LEFT_WHEEL]));
    command_ifs.emplace_back(command_itfs_.back());

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_itfs_.reserve(joint_state_values_.size());
    state_ifs.reserve(joint_state_values_.size());

    state_itfs_.emplace_back(hardware_interface::StateInterface(
      rear_wheels_names_[0], traction_interface_name_,
      &joint_state_values_[STATE_TRACTION_RIGHT_WHEEL]));
    state_ifs.emplace_back(state_itfs_.back());

    state_itfs_.emplace_back(hardware_interface::StateInterface(
      rear_wheels_names_[1], traction_interface_name_,
      &joint_state_values_[STATE_TRACTION_LEFT_WHEEL]));
    state_ifs.emplace_back(state_itfs_.back());

    state_itfs_.emplace_back(hardware_interface::StateInterface(
      front_wheels_names_[0], steering_interface_name_,
      &joint_state_values_[STATE_STEER_RIGHT_WHEEL]));
    state_ifs.emplace_back(state_itfs_.back());

    state_itfs_.emplace_back(hardware_interface::StateInterface(
      front_wheels_names_[1], steering_interface_name_,
      &joint_state_values_[STATE_STEER_LEFT_WHEEL]));
    state_ifs.emplace_back(state_itfs_.back());

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void subscribe_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr) {};
    auto subscription = test_subscription_node.create_subscription<ControllerStateMsg>(
      "/test_steering_controllers_library/controller_state", 10, subs_callback);

    // call update to publish the test value
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);

    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    rclcpp::WaitSet wait_set;          // block used to wait on message
    wait_set.add_subscription(subscription);
    while (max_sub_check_loop_count--)
    {
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01));
      // check if message has been received
      if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready)
      {
        break;
      }
    }
    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";

    // take message from subscription
    rclcpp::MessageInfo msg_info;
    ASSERT_TRUE(subscription->take(msg, msg_info));
  }

  void publish_commands(const double linear = 0.1, const double angular = 0.2)
  {
    auto wait_for_topic = [&](const auto topic_name)
    {
      size_t wait_count = 0;
      while (command_publisher_node_->count_subscribers(topic_name) == 0)
      {
        if (wait_count >= 5)
        {
          auto error_msg =
            std::string("publishing to ") + topic_name + " but no node subscribes to it";
          throw std::runtime_error(error_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++wait_count;
      }
    };

    wait_for_topic(command_publisher_->get_topic_name());

    ControllerReferenceMsg msg;
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;

    command_publisher_->publish(msg);
  }

protected:
  // Controller-related parameters
  double reference_timeout_ = 2.0;
  bool front_steering_ = true;
  bool open_loop_ = false;
  unsigned int velocity_rolling_window_size_ = 10;
  bool position_feedback_ = false;
  bool use_stamped_vel_ = true;
  std::vector<std::string> rear_wheels_names_ = {"rear_right_wheel_joint", "rear_left_wheel_joint"};
  std::vector<std::string> front_wheels_names_ = {
    "front_right_steering_joint", "front_left_steering_joint"};
  std::vector<std::string> joint_names_ = {
    rear_wheels_names_[0], rear_wheels_names_[1], front_wheels_names_[0], front_wheels_names_[1]};

  std::vector<std::string> rear_wheels_preceeding_names_ = {
    "pid_controller/rear_right_wheel_joint", "pid_controller/rear_left_wheel_joint"};
  std::vector<std::string> front_wheels_preceeding_names_ = {
    "pid_controller/front_right_steering_joint", "pid_controller/front_left_steering_joint"};
  std::vector<std::string> preceeding_joint_names_ = {
    rear_wheels_preceeding_names_[0], rear_wheels_preceeding_names_[1],
    front_wheels_preceeding_names_[0], front_wheels_preceeding_names_[1]};

  double wheelbase_ = 3.24644;
  double front_wheel_track_ = 2.12321;
  double rear_wheel_track_ = 1.76868;
  double front_wheels_radius_ = 0.45;
  double rear_wheels_radius_ = 0.45;

  std::array<double, 4> joint_state_values_ = {0.5, 0.5, 0.0, 0.0};
  std::array<double, 4> joint_command_values_ = {1.1, 3.3, 2.2, 4.4};

  std::array<std::string, 2> joint_reference_interfaces_ = {"linear/velocity", "angular/position"};
  std::string steering_interface_name_ = "position";
  // defined in setup
  std::string traction_interface_name_ = "";
  std::string preceeding_prefix_ = "pid_controller";

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<CtrlType> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;
};

#endif  // TEST_STEERING_CONTROLLERS_LIBRARY_HPP_
