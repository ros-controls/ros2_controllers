// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef TEMPLATES__ROS2_CONTROL__CONTROLLER__TEST_GRIPPER_IO_CONTROLLER_HPP_
#define TEMPLATES__ROS2_CONTROL__CONTROLLER__TEST_GRIPPER_IO_CONTROLLER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "io_gripper_controller/io_gripper_controller.hpp"
#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

// TODO(anyone): replace the state and command message types
using ControllerStateMsg = io_gripper_controller::IOGripperController::ControllerStateMsg;
using OpenCloseSrvType = io_gripper_controller::IOGripperController::OpenCloseSrvType;
using ControllerModeSrvType = io_gripper_controller::IOGripperController::ControllerModeSrvType;
using EventStateMsg = io_gripper_controller::IOGripperController::EventStateMsg;
using ConfigSrvType = control_msgs::srv::SetConfig;

using GripperAction = io_gripper_controller::IOGripperController::GripperAction;
using GoalHandleGripperAction = rclcpp_action::ClientGoalHandle<GripperAction>;
using GripperConfigAction = io_gripper_controller::IOGripperController::GripperConfigAction;
using GoalHandleGripperConfigAction = rclcpp_action::ClientGoalHandle<GripperConfigAction>;
using JointStateMsg = sensor_msgs::msg::JointState;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace

// subclassing and friending so we can access member variables
class TestableIOGripperController : public io_gripper_controller::IOGripperController
{
  FRIEND_TEST(IOGripperControllerTest, AllParamsSetSuccess);
  FRIEND_TEST(IOGripperControllerTest, AllParamNotSetFailure);
  FRIEND_TEST(IOGripperControllerTest, OpenGripperService);
  FRIEND_TEST(IOGripperControllerTest, CloseGripperService);
  FRIEND_TEST(IOGripperControllerTest, OpenCloseGripperAction);
  FRIEND_TEST(IOGripperControllerTest, ReconfigureGripperService);
  FRIEND_TEST(IOGripperControllerTest, ReconfigureGripperAction);
  
  FRIEND_TEST(IOGripperControllerTest, DefaultParametersNotSet);
  FRIEND_TEST(IOGripperControllerTest, OpeningCommandParametersNotSet);
  FRIEND_TEST(IOGripperControllerTest, ClosingCommandsParametersNotSet);
  FRIEND_TEST(IOGripperControllerTest, DifferentCommandsParametersSet);
  FRIEND_TEST(IOGripperControllerTest, OpenedStatesParametersNotSet);
  FRIEND_TEST(IOGripperControllerTest, ClosedStatesParametersNotSet);
  FRIEND_TEST(IOGripperControllerTest, DifferentStatesParametersNotSet);
  FRIEND_TEST(IOGripperControllerTest, all_parameters_set_configure_success);


public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = io_gripper_controller::IOGripperController::on_configure(previous_state);
    return ret;
  }
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class IOGripperControllerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();


    subscription_caller_node_ = std::make_shared<rclcpp::Node>("subscription_caller");
    joint_state_sub_ = subscription_caller_node_->create_subscription<JointStateMsg>(
      "/joint_states", 1,
      [this](const JointStateMsg::SharedPtr msg) {
        joint_state_sub_msg_ = msg;
        RCLCPP_INFO(rclcpp::get_logger("test_io_gripper_controller"), "Received joint state");
      });


    service_caller_node_ = std::make_shared<rclcpp::Node>("service_caller");
    close_gripper_service_client_ = service_caller_node_->create_client<OpenCloseSrvType>(
      "/test_io_gripper_controller/gripper_close");
    open_gripper_service_client_ = service_caller_node_->create_client<OpenCloseSrvType>(
      "/test_io_gripper_controller/gripper_open");

    configure_gripper_service_client_ = service_caller_node_->create_client<ConfigSrvType>(
      "/test_io_gripper_controller/reconfigure_to");

    // action client
    action_caller_node_ = std::make_shared<rclcpp::Node>("action_caller");

    gripper_action_client_ = rclcpp_action::create_client<GripperAction>(
      action_caller_node_, "/test_io_gripper_controller/gripper_action");

    gripper_config_action_client_ = rclcpp_action::create_client<GripperConfigAction>(
      action_caller_node_, "/test_io_gripper_controller/reconfigure_gripper_action");
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(const std::string controller_name = "test_io_gripper_controller")
  {
    ASSERT_EQ(
      controller_->init(controller_name, "", 0, "", controller_->define_custom_node_options()),
      controller_interface::return_type::OK);

    // setting the command state interfaces manually
    std::vector<hardware_interface::LoanedCommandInterface> command_itfs;
    command_itfs.reserve(3); // TODO (Sachin) : change this some variable later

    command_itfs.emplace_back(greif_oeffen_wqg1_cmd_);
    command_itfs.emplace_back(greif_schliess_wqg2_cmd_);
    command_itfs.emplace_back(bremse_wqg7_cmd_);
    command_itfs.emplace_back(stich_125_wqg5_cmd_);
    command_itfs.emplace_back(stich_250_wqg6_cmd_);

    std::vector<hardware_interface::LoanedStateInterface> state_itfs_;
    state_itfs_.reserve(2);

    state_itfs_.emplace_back(greif_geoff_bg01_state_);
    state_itfs_.emplace_back(greif_geschl_bg02_state_);
    state_itfs_.emplace_back(stich_125_bg03_state_);
    state_itfs_.emplace_back(stich_250_bg04_state_);
    state_itfs_.emplace_back(bau_teil_abfrage_bg06_state_);

    controller_->assign_interfaces(std::move(command_itfs), std::move(state_itfs_));
  }  

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<GripperAction>>> callOpenGripperAction(rclcpp::Executor & executor)
  {
    auto goal = GripperAction::Goal();
    goal.open = true;

    bool wait_for_server_ret = gripper_action_client_->wait_for_action_server(std::chrono::milliseconds(500));
    EXPECT_TRUE(wait_for_server_ret);
    if (!wait_for_server_ret)
    {
      throw std::runtime_error("Action server is not available!");
    }

    auto future = gripper_action_client_->async_send_goal(goal);

    return future;

    // goal->open = true;
    // auto future = gripper_action_client_->async_send_goal(goal);
    // EXPECT_EQ(executor.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
  }

  std::shared_ptr<OpenCloseSrvType::Response> call_close_service(rclcpp::Executor & executor)
  {
    auto request = std::make_shared<OpenCloseSrvType::Request>();

    bool wait_for_service_ret =
      close_gripper_service_client_->wait_for_service(std::chrono::milliseconds(500));
    EXPECT_TRUE(wait_for_service_ret);
    if (!wait_for_service_ret)
    {
      throw std::runtime_error("Services is not available!");
    }
    auto result = close_gripper_service_client_->async_send_request(request);
    EXPECT_EQ(executor.spin_until_future_complete(result), rclcpp::FutureReturnCode::SUCCESS);

    return result.get();
  }

    std::shared_ptr<OpenCloseSrvType::Response> call_open_service(rclcpp::Executor & executor)
  {
    auto request = std::make_shared<OpenCloseSrvType::Request>();

    bool wait_for_service_ret =
      open_gripper_service_client_->wait_for_service(std::chrono::milliseconds(500));
    EXPECT_TRUE(wait_for_service_ret);
    if (!wait_for_service_ret)
    {
      throw std::runtime_error("Services is not available!");
    }
    auto result = open_gripper_service_client_->async_send_request(request);
    EXPECT_EQ(executor.spin_until_future_complete(result), rclcpp::FutureReturnCode::SUCCESS);

    return result.get();
  }
  
  void setup_parameters()
  {
    controller_->get_node()->set_parameter({"use_action", false});
    controller_->get_node()->set_parameter({"open_close_joints", open_close_joints});
    controller_->get_node()->set_parameter({"open.joint_states", open_joint_states});
    controller_->get_node()->set_parameter({"open.set_before_command.high", open_set_before_command_high});
    controller_->get_node()->set_parameter({"open.set_before_command.low", open_set_before_command_low});
    controller_->get_node()->set_parameter({"open.command.high", open_command_high});
    controller_->get_node()->set_parameter({"open.command.low", open_command_low});
    controller_->get_node()->set_parameter({"open.state.high", open_state_high});
    controller_->get_node()->set_parameter({"open.state.low", open_state_low});
    controller_->get_node()->set_parameter({"open.set_after_command.high", open_set_after_command_high});
    controller_->get_node()->set_parameter({"open.set_after_command.low", open_set_after_command_low});

    controller_->get_node()->set_parameter({"possible_closed_states", possible_closed_states});
    controller_->get_node()->set_parameter({"close.joint_states", close_joint_states});
    controller_->get_node()->set_parameter({"close.set_before_command.high", close_set_before_command_high});
    controller_->get_node()->set_parameter({"close.set_before_command.low", close_set_before_command_low});
    controller_->get_node()->set_parameter({"close.command.high", close_command_high});
    controller_->get_node()->set_parameter({"close.command.low", close_command_low});
    controller_->get_node()->set_parameter({"close.state.empty_close.joint_states", close_joint_states});
    controller_->get_node()->set_parameter({"close.state.empty_close.high", close_state_high});
    controller_->get_node()->set_parameter({"close.state.empty_close.low", close_state_low});
    controller_->get_node()->set_parameter({"close.state.empty_close.set_after_command_high", close_set_after_command_high});
    controller_->get_node()->set_parameter({"close.state.empty_close.set_after_command_low", close_set_after_command_low});
    controller_->get_node()->set_parameter({"close.state.full_close.joint_states", close_joint_states});
    controller_->get_node()->set_parameter({"close.state.full_close.high", close_state_low});
    controller_->get_node()->set_parameter({"close.state.full_close.low", close_state_high});
    controller_->get_node()->set_parameter({"close.state.full_close.set_after_command_high", close_set_after_command_high});
    controller_->get_node()->set_parameter({"close.state.full_close.set_after_command_low", close_set_after_command_low});


    controller_->get_node()->set_parameter({"configurations", configurations_list});
    controller_->get_node()->set_parameter({"configuration_joints", configuration_joints});
    controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.joint_states", stichmass_joint_states});
    controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.command_high", stichmass_command_high});
    controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.command_low", stichmass_command_low});
    controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.state_high", stichmass_state_high});
    controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.state_low", stichmass_state_low});

    controller_->get_node()->set_parameter({"gripper_specific_sensors", gripper_specific_sensors});
    controller_->get_node()->set_parameter({"sensors_interfaces.hohenabfrage.input", gripper_interfaces_input});
  }

  void setup_parameters_fail()
  {
    controller_->get_node()->set_parameter({"use_action", false});
    controller_->get_node()->set_parameter({"open_close_joints", ""});
    controller_->get_node()->set_parameter({"open.joint_states", open_joint_states});
    controller_->get_node()->set_parameter({"open.set_before_command.high", open_set_before_command_high});
    controller_->get_node()->set_parameter({"open.set_before_command.low", open_set_before_command_low});
    controller_->get_node()->set_parameter({"open.command.high", open_command_high});
    controller_->get_node()->set_parameter({"open.command.low", open_command_low});
    controller_->get_node()->set_parameter({"open.state.high", open_state_high});
    controller_->get_node()->set_parameter({"open.state.low", open_state_low});
    controller_->get_node()->set_parameter({"open.set_after_command.high", open_set_after_command_high});
    controller_->get_node()->set_parameter({"open.set_after_command.low", open_set_after_command_low});

    controller_->get_node()->set_parameter({"possible_closed_states", possible_closed_states});
    controller_->get_node()->set_parameter({"close.joint_states", close_joint_states});
    controller_->get_node()->set_parameter({"close.set_before_command.high", close_set_before_command_high});
    controller_->get_node()->set_parameter({"close.set_before_command.low", close_set_before_command_low});
    controller_->get_node()->set_parameter({"close.command.high", close_command_high});
    controller_->get_node()->set_parameter({"close.command.low", close_command_low});
    controller_->get_node()->set_parameter({"close.state.empty_close.joint_states", close_joint_states});
    controller_->get_node()->set_parameter({"close.state.empty_close.high", close_state_high});
    controller_->get_node()->set_parameter({"close.state.empty_close.low", close_state_low});
    controller_->get_node()->set_parameter({"close.state.empty_close.set_after_command_high", close_set_after_command_high});
    controller_->get_node()->set_parameter({"close.state.empty_close.set_after_command_low", close_set_after_command_low});
    controller_->get_node()->set_parameter({"close.state.full_close.joint_states", close_joint_states});
    controller_->get_node()->set_parameter({"close.state.full_close.high", close_state_low});
    controller_->get_node()->set_parameter({"close.state.full_close.low", close_state_high});
    controller_->get_node()->set_parameter({"close.state.full_close.set_after_command_high", close_set_after_command_high});
    controller_->get_node()->set_parameter({"close.state.full_close.set_after_command_low", close_set_after_command_low});


    controller_->get_node()->set_parameter({"configurations", configurations_list});
    controller_->get_node()->set_parameter({"configuration_joints", configuration_joints});
    controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.joint_states", stichmass_joint_states});
    controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.command_high", stichmass_command_high});
    controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.command_low", stichmass_command_low});
    controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.state_high", stichmass_state_high});
    controller_->get_node()->set_parameter({"configuration_setup.stichmass_125.state_low", stichmass_state_low});

    controller_->get_node()->set_parameter({"gripper_specific_sensors", gripper_specific_sensors});
    controller_->get_node()->set_parameter({"sensors_interfaces.hohenabfrage.input", gripper_interfaces_input});
  }

protected:
  // Controller-related parameters
  std::vector<std::string> open_close_joints = {"gripper_clamp_jaw"};

  std::vector<double> open_joint_states = {0.0};
  std::vector<std::string> open_set_before_command_high = {"EL2008/Bremse_WQG7"};
  std::vector<std::string> open_set_before_command_low = {"EL2008/Greiferteil_Schliessen_WQG2"};
  std::vector<std::string> open_set_after_command_high = {"EL2008/Bremse_WQG7"};
  std::vector<std::string> open_set_after_command_low = {"EL2008/Greiferteil_Schliessen_WQG2"};
  std::vector<std::string> open_command_high = {"EL2008/Greiferteil_Oeffnen_WQG1"};
  std::vector<std::string> open_command_low = {"EL2008/Greiferteil_Schliessen_WQG2"};
  std::vector<std::string> open_state_high = {"EL1008/Greifer_Geoeffnet_BG01"};
  std::vector<std::string> open_state_low = {"EL1008/Greifer_Geschloschen_BG02"};

  std::vector<std::string> possible_closed_states = {"empty_close", "full_close"};
  std::vector<double> close_joint_states = {0.08};
  std::vector<std::string> close_set_before_command_high = {"EL2008/Bremse_WQG7"};
  std::vector<std::string> close_set_before_command_low = {"EL2008/Greiferteil_Oeffnen_WQG1"};
  std::vector<std::string> close_set_after_command_high = {"EL2008/Bremse_WQG7"};
  std::vector<std::string> close_set_after_command_low = {"EL2008/Greiferteil_Oeffnen_WQG1"};
  std::vector<std::string> close_command_high = {"EL2008/Greiferteil_Schliessen_WQG2"};
  std::vector<std::string> close_command_low = {"EL2008/Greiferteil_Oeffnen_WQG1"};
  std::vector<std::string> close_state_high = {"EL1008/Greifer_Geschloschen_BG02"};
  std::vector<std::string> close_state_low = {"EL1008/Bauteilabfrage_BG06"};

  std::vector<std::string> configurations_list = {"stichmass_125"};
  std::vector<std::string> configuration_joints = {"gripper_gripper_distance_joint"};

  std::vector<double> stichmass_joint_states = {0.125};
  std::vector<std::string> stichmass_command_high = {"EL2008/Stichmass_125_WQG5"};
  std::vector<std::string> stichmass_command_low = {"EL2008/Stichmass_250_WQG6"};
  std::vector<std::string> stichmass_state_high = {"EL1008/Stichmass_125mm_BG03"};
  std::vector<std::string> stichmass_state_low = {"EL1008/Stichmass_250mm_BG04"};

  std::vector<std::string> gripper_specific_sensors = {"hohenabfrage"};
  std::string gripper_interfaces_input = {"EL1008/Hohenabfrage_BG5"};



  


  std::vector<std::string> joint_names_ = {"gripper_joint", "finger_joint"};
  std::vector<std::string> state_joint_names_ = {"gripper_joint"};
  std::string interface_name_ = "gpio";
  double joint_value_opened_ = 75.0;
  double joint_value_closed_ = 30.0;
  std::array<double, 2> joint_command_values_ = {0.0, 0.0};
  std::array<double, 1> joint_state_values_ = {0.0};

  std::array<double, 2> joint_command_opened = {1.0, 0.0};
  std::array<double, 2> joint_command_closed = {0.0, 0.0};

  hardware_interface::CommandInterface joint_1_gpio_cmd_{joint_names_[0], interface_name_, &joint_command_values_[0]};
  hardware_interface::CommandInterface joint_2_gpio_cmd_{joint_names_[1], interface_name_, &joint_command_values_[1]};

  std::array<double, 5> command_ios_values_ = {0.0, 1.0, 0.0, 0.0, 0.0};
  std::array<double, 5> state_ios_values_ = {1.0, 0.0, 1.0, 0.0, 1.0};

  hardware_interface::CommandInterface greif_oeffen_wqg1_cmd_{"EL2008", "Greiferteil_Oeffnen_WQG1", &command_ios_values_[0]};
  hardware_interface::CommandInterface greif_schliess_wqg2_cmd_{"EL2008", "Greiferteil_Schliessen_WQG2", &command_ios_values_[1]};
  hardware_interface::CommandInterface bremse_wqg7_cmd_{"EL2008", "Bremse_WQG7", &command_ios_values_[2]};
  hardware_interface::CommandInterface stich_125_wqg5_cmd_{"EL2008", "Stichmass_125_WQG5", &command_ios_values_[3]};
  hardware_interface::CommandInterface stich_250_wqg6_cmd_{"EL2008", "Stichmass_250_WQG6", &command_ios_values_[4]};

  hardware_interface::StateInterface greif_geoff_bg01_state_{"EL1008", "Greifer_Geoeffnet_BG01", &state_ios_values_[0]};
  hardware_interface::StateInterface greif_geschl_bg02_state_{"EL1008", "Greifer_Geschloschen_BG02", &state_ios_values_[1]};
  hardware_interface::StateInterface stich_125_bg03_state_{"EL1008", "Stichmass_125mm_BG03", &state_ios_values_[2]};
  hardware_interface::StateInterface stich_250_bg04_state_{"EL1008", "Stichmass_250mm_BG04", &state_ios_values_[3]};
  hardware_interface::StateInterface bau_teil_abfrage_bg06_state_{"EL1008", "Bauteilabfrage_BG06", &state_ios_values_[4]};

  JointStateMsg::SharedPtr joint_state_sub_msg_ = std::make_shared<JointStateMsg>();
  

  // Test related parameters
  std::unique_ptr<TestableIOGripperController> controller_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
  rclcpp::Client<OpenCloseSrvType>::SharedPtr close_gripper_service_client_;
  rclcpp::Client<OpenCloseSrvType>::SharedPtr open_gripper_service_client_;
  rclcpp::Client<ConfigSrvType>::SharedPtr configure_gripper_service_client_;
  rclcpp_action::Client<GripperAction>::SharedPtr gripper_action_client_;
  rclcpp_action::Client<GripperConfigAction>::SharedPtr gripper_config_action_client_;
  rclcpp::Node::SharedPtr subscription_caller_node_, service_caller_node_, action_caller_node_;
};


class IOGripperControllerTest : public IOGripperControllerFixture<TestableIOGripperController>
{
};
#endif  // TEMPLATES__ROS2_CONTROL__CONTROLLER__TEST_GRIPPER_IO_CONTROLLER_HPP_
