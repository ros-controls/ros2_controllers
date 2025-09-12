// Copyright (c) 2025, b»robotized by Stogl Robotics
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

#ifndef GPIO_TOOL_CONTROLLER__TEST_GPIO_TOOL_CONTROLLER_HPP_
#define GPIO_TOOL_CONTROLLER__TEST_GPIO_TOOL_CONTROLLER_HPP_

#include <gmock/gmock.h>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include "gpio_controllers/gpio_tool_controller.hpp"

// // TODO(anyone): replace the state and command message types
// using JointStateMsg = io_gripper_controller::IOGripperController::JointStateMsg;
// using OpenCloseSrvType = io_gripper_controller::IOGripperController::OpenCloseSrvType;
// using ControllerModeSrvType = io_gripper_controller::IOGripperController::ControllerModeSrvType;
// using ConfigSrvType = io_gripper_controller::IOGripperController::
//   ConfigSrvType;  //  control_msgs::srv::SetIOGripperConfig;

// using GripperAction = io_gripper_controller::IOGripperController::GripperAction;
// using GoalHandleGripperAction = rclcpp_action::ClientGoalHandle<GripperAction>;
// using GripperConfigAction = io_gripper_controller::IOGripperController::GripperConfigAction;
// using GoalHandleGripperConfigAction = rclcpp_action::ClientGoalHandle<GripperConfigAction>;
// using JointStateMsg = sensor_msgs::msg::JointState;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace

// subclassing and friending so we can access member variables
class TestableGpioToolController : public gpio_tool_controller::GpioToolController
{
  FRIEND_TEST(GpioToolControllerTest, AllParamsSetSuccess);
  FRIEND_TEST(GpioToolControllerTest, AllParamNotSetFailure);
  // FRIEND_TEST(GpioToolControllerTest, OpenGripperService);
  // FRIEND_TEST(GpioToolControllerTest, CloseGripperService);
  // FRIEND_TEST(GpioToolControllerTest, OpenCloseGripperAction);
  // FRIEND_TEST(GpioToolControllerTest, ReconfigureGripperService);
  // FRIEND_TEST(GpioToolControllerTest, ReconfigureGripperAction);

  // FRIEND_TEST(GpioToolControllerTest, DefaultParametersNotSet);
  // FRIEND_TEST(GpioToolControllerTest, OpeningCommandParametersNotSet);
  // FRIEND_TEST(GpioToolControllerTest, ClosingCommandsParametersNotSet);
  // FRIEND_TEST(GpioToolControllerTest, DifferentCommandsParametersSet);
  // FRIEND_TEST(GpioToolControllerTest, OpenedStatesParametersNotSet);
  // FRIEND_TEST(GpioToolControllerTest, ClosedStatesParametersNotSet);
  // FRIEND_TEST(GpioToolControllerTest, DifferentStatesParametersNotSet);
  // FRIEND_TEST(GpioToolControllerTest, all_parameters_set_configure_success);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ret = gpio_tool_controller::GpioToolController::on_configure(previous_state);
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

    // subscription_caller_node_ = std::make_shared<rclcpp::Node>("subscription_caller");
    // joint_state_sub_ = subscription_caller_node_->create_subscription<JointStateMsg>(
    //   "/joint_states", 1,
    //   [this](const JointStateMsg::SharedPtr msg)
    //   {
    //     joint_state_sub_msg_ = msg;
    //     RCLCPP_INFO(rclcpp::get_logger("test_io_gripper_controller"), "Received joint state");
    //   });

    // service_caller_node_ = std::make_shared<rclcpp::Node>("service_caller");
    // close_gripper_service_client_ = service_caller_node_->create_client<OpenCloseSrvType>(
    //   "/test_io_gripper_controller/gripper_close");
    // open_gripper_service_client_ = service_caller_node_->create_client<OpenCloseSrvType>(
    //   "/test_io_gripper_controller/gripper_open");

    // configure_gripper_service_client_ = service_caller_node_->create_client<ConfigSrvType>(
    //   "/test_io_gripper_controller/reconfigure_to");

    // // action client
    // action_caller_node_ = std::make_shared<rclcpp::Node>("action_caller");

    // gripper_action_client_ = rclcpp_action::create_client<GripperAction>(
    //   action_caller_node_, "/test_io_gripper_controller/gripper_action");

    // gripper_config_action_client_ = rclcpp_action::create_client<GripperConfigAction>(
    //   action_caller_node_, "/test_io_gripper_controller/reconfigure_gripper_action");
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(
    const std::string controller_name = "test_gpio_tool_controller",
    const std::vector<rclcpp::Parameter> & parameters = {})
  {
    RCLCPP_INFO(rclcpp::get_logger("IOGripperControllerTest"), "initializing controller");
    auto node_options = controller_->define_custom_node_options();
    node_options.parameter_overrides(parameters);

    ASSERT_EQ(
      controller_->init(controller_name, "", 0, "", node_options),
      controller_interface::return_type::OK);
    RCLCPP_INFO(rclcpp::get_logger("IOGripperControllerTest"), "initialized successfully");

    // // setting the command state interfaces manually
    // std::vector<hardware_interface::LoanedCommandInterface> command_itfs;
    // command_itfs.reserve(3);  // TODO(Sachin) : change this some variable later

    // command_itfs.emplace_back(greif_oeffen_wqg1_cmd_);
    // command_itfs.emplace_back(greif_schliess_wqg2_cmd_);
    // command_itfs.emplace_back(bremse_wqg7_cmd_);
    // command_itfs.emplace_back(stich_125_wqg5_cmd_);
    // command_itfs.emplace_back(stich_250_wqg6_cmd_);

    // std::vector<hardware_interface::LoanedStateInterface> state_itfs_;
    // state_itfs_.reserve(2);

    // state_itfs_.emplace_back(greif_geoff_bg01_state_);
    // state_itfs_.emplace_back(greif_geschl_bg02_state_);
    // state_itfs_.emplace_back(stich_125_bg03_state_);
    // state_itfs_.emplace_back(stich_250_bg04_state_);
    // state_itfs_.emplace_back(bau_teil_abfrage_bg06_state_);

    // controller_->assign_interfaces(std::move(command_itfs), std::move(state_itfs_));
  }

  // std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<GripperAction>>>
  // callOpenGripperAction(rclcpp::Executor & executor)
  // {
  //   auto goal = GripperAction::Goal();
  //   goal.open = true;

  //   bool wait_for_server_ret =
  //     gripper_action_client_->wait_for_action_server(std::chrono::milliseconds(500));
  //   EXPECT_TRUE(wait_for_server_ret);
  //   if (!wait_for_server_ret)
  //   {
  //     throw std::runtime_error("Action server is not available!");
  //   }

  //   auto future = gripper_action_client_->async_send_goal(goal);

  //   return future;

  //   // goal->open = true;
  //   // auto future = gripper_action_client_->async_send_goal(goal);
  //   // EXPECT_EQ(executor.spin_until_future_complete(future), rclcpp::FutureReturnCode::SUCCESS);
  // }

  // std::shared_ptr<OpenCloseSrvType::Response> call_close_service(rclcpp::Executor & executor)
  // {
  //   auto request = std::make_shared<OpenCloseSrvType::Request>();

  //   bool wait_for_service_ret =
  //     close_gripper_service_client_->wait_for_service(std::chrono::milliseconds(500));
  //   EXPECT_TRUE(wait_for_service_ret);
  //   if (!wait_for_service_ret)
  //   {
  //     throw std::runtime_error("Services is not available!");
  //   }
  //   auto result = close_gripper_service_client_->async_send_request(request);
  //   EXPECT_EQ(executor.spin_until_future_complete(result), rclcpp::FutureReturnCode::SUCCESS);

  //   return result.get();
  // }

  // std::shared_ptr<OpenCloseSrvType::Response> call_open_service(rclcpp::Executor & executor)
  // {
  //   auto request = std::make_shared<OpenCloseSrvType::Request>();

  //   bool wait_for_service_ret =
  //     open_gripper_service_client_->wait_for_service(std::chrono::milliseconds(500));
  //   EXPECT_TRUE(wait_for_service_ret);
  //   if (!wait_for_service_ret)
  //   {
  //     throw std::runtime_error("Services is not available!");
  //   }
  //   auto result = open_gripper_service_client_->async_send_request(request);
  //   EXPECT_EQ(executor.spin_until_future_complete(result), rclcpp::FutureReturnCode::SUCCESS);

  //   return result.get();
  // }

  void setup_parameters()
  {
    controller_->get_node()->set_parameter({"use_action", true});
    controller_->get_node()->set_parameter({"timeout", 5.0});
    controller_->get_node()->set_parameter({"tolerance", 0.00001});
    controller_->get_node()->set_parameter(
      {"engaged_joints", std::vector<std::string>{"gripper_clamp_jaw"}});

    // Disengaged state
    controller_->get_node()->set_parameter({"disengaged.name", std::string("open")});
    controller_->get_node()->set_parameter({"disengaged.joint_states", std::vector<double>{0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_command.interfaces",
       std::vector<std::string>{"Release_Break_valve", "Release_Something"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_command.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_state.interfaces", std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_state.values", std::vector<double>{0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.command.interfaces", std::vector<std::string>{"Open_valve", "Close_valve"}});
    controller_->get_node()->set_parameter(
      {"disengaged.command.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.state.interfaces", std::vector<std::string>{"Opened_signal", "Closed_signal"}});
    controller_->get_node()->set_parameter(
      {"disengaged.state.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_command.interfaces",
       std::vector<std::string>{"Release_Break_valve", "Release_Something"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_command.values", std::vector<double>{0.0, 1.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_state.interfaces", std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_state.values", std::vector<double>{1.0}});

    // Possible engaged states
    controller_->get_node()->set_parameter(
      {"possible_engaged_states", std::vector<std::string>{"close_empty", "close_full"}});

    // Engaged state
    controller_->get_node()->set_parameter(
      {"engaged.set_before_command.interfaces",
       std::vector<std::string>{"Release_Break_valve", "Release_Something"}});
    controller_->get_node()->set_parameter(
      {"engaged.set_before_command.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.set_before_state.interfaces", std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"engaged.set_before_state.values", std::vector<double>{0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.command.interfaces", std::vector<std::string>{"Close_valve", "Open_valve"}});
    controller_->get_node()->set_parameter(
      {"engaged.command.values", std::vector<double>{1.0, 0.0}});

    // Engaged states: close_empty
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.joint_states", std::vector<double>{0.16}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.interfaces",
       std::vector<std::string>{"Closed_signal", "Part_Grasped_signal"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_command_interfaces",
       std::vector<std::string>{"Release_Something", "Release_Break_valve"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_command_values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_state_interfaces",
       std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_state_values", std::vector<double>{1.0}});

    // Engaged states: close_full
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.joint_states", std::vector<double>{0.08}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.interfaces",
       std::vector<std::string>{"Closed_signal", "Part_Grasped_signal"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.values", std::vector<double>{0.0, 1.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_command_interfaces",
       std::vector<std::string>{"Release_Something", "Release_Break_valve"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_command_values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_state_interfaces",
       std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_state_values", std::vector<double>{1.0}});

    // Configurations
    controller_->get_node()->set_parameter({"configurations", std::vector<std::string>{}});
    controller_->get_node()->set_parameter({"configuration_joints", std::vector<std::string>{}});

    // Tool specific sensors
    controller_->get_node()->set_parameter({"tool_specific_sensors", std::vector<std::string>{}});
  }

  void setup_parameters_fail()
  {
    controller_->get_node()->set_parameter({"use_action", true});
    controller_->get_node()->set_parameter({"timeout", 5.0});
    controller_->get_node()->set_parameter({"tolerance", 0.00001});
    controller_->get_node()->set_parameter({"engaged_joints", std::vector<std::string>{}});

    // Disengaged state
    controller_->get_node()->set_parameter({"disengaged.name", std::string("open")});
    controller_->get_node()->set_parameter({"disengaged.joint_states", std::vector<double>{0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_command.interfaces",
       std::vector<std::string>{"Release_Break_valve", "Release_Something"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_command.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_state.interfaces", std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_before_state.values", std::vector<double>{0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.command.interfaces", std::vector<std::string>{"Open_valve", "Close_valve"}});
    controller_->get_node()->set_parameter(
      {"disengaged.command.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.state.interfaces", std::vector<std::string>{"Opened_signal", "Closed_signal"}});
    controller_->get_node()->set_parameter(
      {"disengaged.state.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_command.interfaces",
       std::vector<std::string>{"Release_Break_valve", "Release_Something"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_command.values", std::vector<double>{0.0, 1.0}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_state.interfaces", std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"disengaged.set_after_state.values", std::vector<double>{1.0}});

    // Possible engaged states
    controller_->get_node()->set_parameter({"possible_engaged_states", std::vector<std::string>{}});

    // Engaged state
    controller_->get_node()->set_parameter(
      {"engaged.set_before_command.interfaces",
       std::vector<std::string>{"Release_Break_valve", "Release_Something"}});
    controller_->get_node()->set_parameter(
      {"engaged.set_before_command.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.set_before_state.interfaces", std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"engaged.set_before_state.values", std::vector<double>{0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.command.interfaces", std::vector<std::string>{"Close_valve", "Open_valve"}});
    controller_->get_node()->set_parameter(
      {"engaged.command.values", std::vector<double>{1.0, 0.0}});

    // Engaged states: close_empty
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.joint_states", std::vector<double>{0.16}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.interfaces",
       std::vector<std::string>{"Closed_signal", "Part_Grasped_signal"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_command_interfaces",
       std::vector<std::string>{"Release_Something", "Release_Break_valve"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_command_values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_state_interfaces",
       std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_empty.set_after_state_values", std::vector<double>{1.0}});

    // Engaged states: close_full
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.joint_states", std::vector<double>{0.08}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.interfaces",
       std::vector<std::string>{"Closed_signal", "Part_Grasped_signal"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.values", std::vector<double>{0.0, 1.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_command_interfaces",
       std::vector<std::string>{"Release_Something", "Release_Break_valve"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_command_values", std::vector<double>{1.0, 0.0}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_state_interfaces",
       std::vector<std::string>{"Break_Engaged"}});
    controller_->get_node()->set_parameter(
      {"engaged.states.close_full.set_after_state_values", std::vector<double>{1.0}});

    // Configurations
    controller_->get_node()->set_parameter({"configurations", std::vector<std::string>{}});
    controller_->get_node()->set_parameter({"configuration_joints", std::vector<std::string>{}});

    // Tool specific sensors
    controller_->get_node()->set_parameter({"tool_specific_sensors", std::vector<std::string>{}});
  }

protected:
  // Controller-related parameters
  // std::vector<std::string> open_close_joints = {"gripper_clamp_jaw"};

  // std::vector<double> open_joint_states = {0.0};
  // std::vector<std::string> open_set_before_command_high = {"Release_Break_valve"};
  // std::vector<std::string> open_set_before_command_low = {"Release_Something"};
  // std::vector<std::string> open_set_after_command_high = {"Release_Break_valve"};
  // std::vector<std::string> open_set_after_command_low = {"Release_Something"};
  // std::vector<std::string> open_command_high = {"Open_valve"};
  // std::vector<std::string> open_command_low = {"Close_valve"};
  // std::vector<std::string> open_state_high = {"Opened_signal"};
  // std::vector<std::string> open_state_low = {"Closed_signal"};

  // std::vector<std::string> possible_closed_states = {"empty_close", "full_close"};
  // std::vector<double> close_joint_states = {0.08};
  // std::vector<std::string> close_set_before_command_high = {"Release_Break_valve"};
  // std::vector<std::string> close_set_before_command_low = {"Open_valve"};
  // std::vector<std::string> close_set_after_command_high = {"Release_Break_valve"};
  // std::vector<std::string> close_set_after_command_low = {"Open_valve"};
  // std::vector<std::string> close_command_high = {"Release_Something"};
  // std::vector<std::string> close_command_low = {"Open_valve"};
  // std::vector<std::string> close_state_high = {"Closed_signal"};
  // std::vector<std::string> close_state_low = {"Part_Grasped_signal"};

  // std::vector<std::string> configurations_list = {"narrow_objects"};
  // std::vector<std::string> configuration_joints = {"gripper_gripper_distance_joint"};

  // std::vector<double> stichmass_joint_states = {0.125};
  // std::vector<std::string> stichmass_command_high = {"Narrow_Configuration_Cmd"};
  // std::vector<std::string> stichmass_command_low = {"Wide_Configuration_Cmd"};
  // std::vector<std::string> stichmass_state_high = {"Narrow_Configuraiton_Signal"};
  // std::vector<std::string> stichmass_state_low = {"Narrow_Configuraiton_Signal"};

  // std::vector<std::string> gripper_specific_sensors = {"part_sensor_top"};
  // std::string gripper_interfaces_input = {"Part_Sensor_Top_signal"};

  // std::vector<std::string> joint_names_ = {"gripper_joint", "finger_joint"};
  // std::vector<std::string> state_joint_names_ = {"gripper_joint"};
  // std::string interface_name_ = "gpio";
  // double joint_value_opened_ = 75.0;
  // double joint_value_closed_ = 30.0;
  // std::array<double, 2> joint_command_values_ = {0.0, 0.0};
  // std::array<double, 1> joint_state_values_ = {0.0};

  // std::array<double, 2> joint_command_opened = {1.0, 0.0};
  // std::array<double, 2> joint_command_closed = {0.0, 0.0};

  // hardware_interface::CommandInterface joint_1_gpio_cmd_{
  //   joint_names_[0], interface_name_, &joint_command_values_[0]};
  // hardware_interface::CommandInterface joint_2_gpio_cmd_{
  //   joint_names_[1], interface_name_, &joint_command_values_[1]};

  // std::array<double, 5> command_ios_values_ = {0.0, 1.0, 0.0, 0.0, 0.0};
  // std::array<double, 5> state_ios_values_ = {1.0, 0.0, 1.0, 0.0, 1.0};

  // hardware_interface::CommandInterface greif_oeffen_wqg1_cmd_{
  //   "EL2008", "Greiferteil_Oeffnen_WQG1", &command_ios_values_[0]};
  // hardware_interface::CommandInterface greif_schliess_wqg2_cmd_{
  //   "EL2008", "Greiferteil_Schliessen_WQG2", &command_ios_values_[1]};
  // hardware_interface::CommandInterface bremse_wqg7_cmd_{
  //   "EL2008", "Bremse_WQG7", &command_ios_values_[2]};
  // hardware_interface::CommandInterface stich_125_wqg5_cmd_{
  //   "EL2008", "Stichmass_125_WQG5", &command_ios_values_[3]};
  // hardware_interface::CommandInterface stich_250_wqg6_cmd_{
  //   "EL2008", "Stichmass_250_WQG6", &command_ios_values_[4]};

  // hardware_interface::StateInterface greif_geoff_bg01_state_{
  //   "EL1008", "Greifer_Geoeffnet_BG01", &state_ios_values_[0]};
  // hardware_interface::StateInterface greif_geschl_bg02_state_{
  //   "EL1008", "Greifer_Geschloschen_BG02", &state_ios_values_[1]};
  // hardware_interface::StateInterface stich_125_bg03_state_{
  //   "EL1008", "Stichmass_125mm_BG03", &state_ios_values_[2]};
  // hardware_interface::StateInterface stich_250_bg04_state_{
  //   "EL1008", "Stichmass_250mm_BG04", &state_ios_values_[3]};
  // hardware_interface::StateInterface bau_teil_abfrage_bg06_state_{
  //   "EL1008", "Bauteilabfrage_BG06", &state_ios_values_[4]};

  const std::vector<std::string> possible_engaged_states = {"close_empty", "close_full"};

  // JointStateMsg::SharedPtr joint_state_sub_msg_ = std::make_shared<JointStateMsg>();

  // Test related parameters
  std::unique_ptr<TestableGpioToolController> controller_;
  // rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
  // rclcpp::Client<OpenCloseSrvType>::SharedPtr close_gripper_service_client_;
  // rclcpp::Client<OpenCloseSrvType>::SharedPtr open_gripper_service_client_;
  // rclcpp::Client<ConfigSrvType>::SharedPtr configure_gripper_service_client_;
  // rclcpp_action::Client<GripperAction>::SharedPtr gripper_action_client_;
  // rclcpp_action::Client<GripperConfigAction>::SharedPtr gripper_config_action_client_;
  // rclcpp::Node::SharedPtr subscription_caller_node_, service_caller_node_, action_caller_node_;
};

class GpioToolControllerTest : public IOGripperControllerFixture<TestableGpioToolController>
{
};
#endif  // GPIO_TOOL_CONTROLLER__TEST_GPIO_TOOL_CONTROLLER_HPP_
