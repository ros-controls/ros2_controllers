// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "gmock/gmock.h"
#include "gpio_controllers/gpio_command_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/wait_result.hpp"
#include "rclcpp/wait_set.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
const auto hardware_resources_with_gpio =
  R"(
  <ros2_control name="TestSystemHardware" type="system">
    <gpio name="gpio1">
      <state_interface name="dig.1"/>
    </gpio>
  </ros2_control>
)";

const auto minimal_robot_urdf_with_gpio = std::string(ros2_control_test_assets::urdf_head) +
                                          std::string(hardware_resources_with_gpio) +
                                          std::string(ros2_control_test_assets::urdf_tail);
}  // namespace

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using CmdType = control_msgs::msg::DynamicInterfaceGroupValues;
using StateType = control_msgs::msg::DynamicInterfaceGroupValues;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using lifecycle_msgs::msg::State;

namespace
{
rclcpp::NodeOptions create_node_options_with_overriden_parameters(
  std::vector<rclcpp::Parameter> parameters)
{
  auto node_options = rclcpp::NodeOptions();
  node_options.parameter_overrides(parameters);
  return node_options;
}
}  // namespace

class FriendGpioCommandController : public gpio_controllers::GpioCommandController
{
  FRIEND_TEST(
    GpioCommandControllerTestSuite,
    WhenGivenCommandWithValuesForAllInterfacesThenValueOfInterfacesShouldBeUpdated);
  FRIEND_TEST(
    GpioCommandControllerTestSuite,
    WhenGivenCommandWithOnlyOneGpioThenInterfacesValuesShouldBeUpdated);
  FRIEND_TEST(
    GpioCommandControllerTestSuite,
    WhenCommandContainsMoreValuesThenInterfacesNameForGpioUpdateShouldReturnFalse);
  FRIEND_TEST(
    GpioCommandControllerTestSuite,
    WhenCommandContainsMoreInterfacesNameThenValuesForGpioUpdateShouldReturnFalse);
  FRIEND_TEST(
    GpioCommandControllerTestSuite,
    WhenGivenCommandInterfacesInDifferentOrderThenValueOfInterfacesShouldBeUpdated);
  FRIEND_TEST(
    GpioCommandControllerTestSuite,
    WhenGivenCmdContainsWrongGpioInterfacesOrWrongGpioNameThenCommandInterfacesShouldNotBeUpdated);
};

class GpioCommandControllerTestSuite : public ::testing::Test
{
public:
  GpioCommandControllerTestSuite()
  {
    rclcpp::init(0, nullptr);
    node = std::make_unique<rclcpp::Node>("node");
  }

  ~GpioCommandControllerTestSuite() { rclcpp::shutdown(); }

  void SetUp() { controller_ = std::make_unique<FriendGpioCommandController>(); }

  void TearDown() { controller_.reset(nullptr); }

  void setup_command_and_state_interfaces()
  {
    std::vector<LoanedCommandInterface> command_interfaces;
    command_interfaces.emplace_back(gpio_1_1_dig_cmd, nullptr);
    command_interfaces.emplace_back(gpio_1_2_dig_cmd, nullptr);
    command_interfaces.emplace_back(gpio_2_ana_cmd, nullptr);

    std::vector<LoanedStateInterface> state_interfaces;
    state_interfaces.emplace_back(gpio_1_1_dig_state, nullptr);
    state_interfaces.emplace_back(gpio_1_2_dig_state, nullptr);
    state_interfaces.emplace_back(gpio_2_ana_state, nullptr);

    controller_->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
  }

  void move_to_activate_state(controller_interface::return_type result_of_initialization)
  {
    ASSERT_EQ(result_of_initialization, controller_interface::return_type::OK);

    ASSERT_TRUE(is_configure_succeeded(controller_));

    setup_command_and_state_interfaces();

    ASSERT_TRUE(is_activate_succeeded(controller_));
  }

  void stop_test_when_message_cannot_be_published(int max_sub_check_loop_count)
  {
    ASSERT_GE(max_sub_check_loop_count, 0)
      << "Test was unable to publish a message through controller/broadcaster update loop";
  }

  void assert_default_command_and_state_values()
  {
    ASSERT_EQ(gpio_1_1_dig_cmd->get_optional().value(), gpio_commands.at(0));
    ASSERT_EQ(gpio_1_2_dig_cmd->get_optional().value(), gpio_commands.at(1));
    ASSERT_EQ(gpio_2_ana_cmd->get_optional().value(), gpio_commands.at(2));
    ASSERT_EQ(gpio_1_1_dig_state->get_optional().value(), gpio_states.at(0));
    ASSERT_EQ(gpio_1_2_dig_state->get_optional().value(), gpio_states.at(1));
    ASSERT_EQ(gpio_2_ana_state->get_optional().value(), gpio_states.at(2));
  }

  void update_controller_loop()
  {
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
  }

  CmdType createGpioCommand(
    const std::vector<std::string> & gpios_names,
    const std::vector<control_msgs::msg::InterfaceValue> & interface_values)
  {
    CmdType command;
    command.interface_groups = gpios_names;
    command.interface_values = interface_values;
    return command;
  }

  control_msgs::msg::InterfaceValue createInterfaceValue(
    const std::vector<std::string> & interfaces_names,
    const std::vector<double> & interfaces_values)
  {
    control_msgs::msg::InterfaceValue output;
    output.interface_names = interfaces_names;
    output.values = interfaces_values;
    return output;
  }

  void wait_one_miliseconds_for_timeout()
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller_->get_node()->get_node_base_interface());
    const auto timeout = std::chrono::milliseconds{1};
    const auto until = controller_->get_node()->get_clock()->now() + timeout;
    while (controller_->get_node()->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  int wait_for_subscription(
    std::shared_ptr<rclcpp::SubscriptionBase> subscription, int max_sub_check_loop_count = 5)
  {
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(subscription);
    while (max_sub_check_loop_count--)
    {
      controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
      if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready)
      {
        break;
      }
    }
    return max_sub_check_loop_count;
  }

  bool is_configure_succeeded(const std::unique_ptr<FriendGpioCommandController> & controller)
  {
    auto state = controller->configure();
    return State::PRIMARY_STATE_INACTIVE == state.id();
  }

  bool is_configure_failed(const std::unique_ptr<FriendGpioCommandController> & controller)
  {
    auto state = controller->configure();
    return State::PRIMARY_STATE_UNCONFIGURED == state.id();
  }

  bool is_activate_succeeded(const std::unique_ptr<FriendGpioCommandController> & controller)
  {
    auto state = controller->get_node()->activate();
    return State::PRIMARY_STATE_ACTIVE == state.id();
  }

  bool is_activate_failed(const std::unique_ptr<FriendGpioCommandController> & controller)
  {
    auto state = controller->get_node()->activate();
    return State::PRIMARY_STATE_UNCONFIGURED == state.id();
  }

  std::unique_ptr<FriendGpioCommandController> controller_;

  controller_interface::ControllerInterfaceParams create_ctrl_params(
    const rclcpp::NodeOptions & node_options, const std::string & robot_description = "")
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_gpio_command_controller";
    params.robot_description = robot_description;
    params.update_rate = 0;
    params.node_namespace = "";
    params.node_options = node_options;
    return params;
  }
  const std::vector<std::string> gpio_names{"gpio1", "gpio2"};
  std::vector<double> gpio_commands{1.0, 0.0, 3.1};
  std::vector<double> gpio_states{1.0, 0.0, 3.1};

  CommandInterface::SharedPtr gpio_1_1_dig_cmd =
    std::make_shared<CommandInterface>(gpio_names.at(0), "dig.1", &gpio_commands.at(0));
  CommandInterface::SharedPtr gpio_1_2_dig_cmd =
    std::make_shared<CommandInterface>(gpio_names.at(0), "dig.2", &gpio_commands.at(1));
  CommandInterface::SharedPtr gpio_2_ana_cmd =
    std::make_shared<CommandInterface>(gpio_names.at(1), "ana.1", &gpio_commands.at(2));

  StateInterface::SharedPtr gpio_1_1_dig_state =
    std::make_shared<StateInterface>(gpio_names.at(0), "dig.1", &gpio_states.at(0));
  StateInterface::SharedPtr gpio_1_2_dig_state =
    std::make_shared<StateInterface>(gpio_names.at(0), "dig.2", &gpio_states.at(1));
  StateInterface::SharedPtr gpio_2_ana_state =
    std::make_shared<StateInterface>(gpio_names.at(1), "ana.1", &gpio_states.at(2));
  std::unique_ptr<rclcpp::Node> node;
};

TEST_F(GpioCommandControllerTestSuite, WhenNoParametersAreSetInitShouldFail)
{
  const auto result = controller_->init(create_ctrl_params(
    controller_->define_custom_node_options(), ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::ERROR);
}

TEST_F(GpioCommandControllerTestSuite, WhenGpiosParameterIsEmptyInitShouldFail)
{
  const auto node_options =
    create_node_options_with_overriden_parameters({{"gpios", std::vector<std::string>{}}});
  const auto result = controller_->init(create_ctrl_params(node_options));

  ASSERT_EQ(result, controller_interface::return_type::ERROR);
}

TEST_F(GpioCommandControllerTestSuite, WhenInterfacesParameterForGpioIsEmptyInitShouldNotFail)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", std::vector<std::string>{"gpio1"}},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{}}});
  const auto result = controller_->init(create_ctrl_params(node_options));

  ASSERT_EQ(result, controller_interface::return_type::OK);
}

TEST_F(GpioCommandControllerTestSuite, WhenInterfacesParameterForGpioIsNotSetInitShouldNotFail)
{
  const auto node_options =
    create_node_options_with_overriden_parameters({{"gpios", std::vector<std::string>{"gpio1"}}});
  const auto result = controller_->init(create_ctrl_params(node_options));

  ASSERT_EQ(result, controller_interface::return_type::OK);
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenGpiosAreSetAndInterfacesAreSetForAllGpiosThenInitShouldSuccess)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});

  const auto result = controller_->init(create_ctrl_params(node_options));
  ASSERT_EQ(result, controller_interface::return_type::OK);
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenCommandAndStateInterfacesAreEmptyAndNoInterfacesAreSetInUrdfConfigurationShouldFail)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", std::vector<std::string>{"gpio1"}},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{}}});
  const auto result = controller_->init(
    create_ctrl_params(node_options, ros2_control_test_assets::minimal_robot_urdf));
  ASSERT_EQ(result, controller_interface::return_type::OK);

  ASSERT_TRUE(is_configure_failed(controller_));
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenStateInterfaceAreNotConfiguredButAreSetInUrdfForConfiguredGpiosThenConfigureShouldSucceed)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", std::vector<std::string>{"gpio1"}},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{}}});
  const auto result =
    controller_->init(create_ctrl_params(node_options, minimal_robot_urdf_with_gpio));

  ASSERT_EQ(result, controller_interface::return_type::OK);

  ASSERT_TRUE(is_configure_succeeded(controller_));
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenStateInterfaceAreNotConfiguredAndProvidedUrdfIsEmptyThenConfigureShouldFail)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", std::vector<std::string>{"gpio1"}},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{}}});
  const auto result = controller_->init(create_ctrl_params(node_options));

  ASSERT_EQ(result, controller_interface::return_type::OK);

  ASSERT_TRUE(is_configure_failed(controller_));
}

TEST_F(GpioCommandControllerTestSuite, ConfigureAndActivateParamsSuccess)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});

  move_to_activate_state(
    controller_->init("test_gpio_command_controller", "", 0, "", node_options));
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenAssignedCommandInterfacesDoNotMatchInterfacesFromParamsThenControllerShouldFailOnActivation)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});
  const auto result = controller_->init("test_gpio_command_controller", "", 0, "", node_options);
  ASSERT_EQ(result, controller_interface::return_type::OK);

  ASSERT_TRUE(is_configure_succeeded(controller_));

  std::vector<LoanedCommandInterface> command_interfaces;
  command_interfaces.emplace_back(gpio_1_1_dig_cmd, nullptr);
  command_interfaces.emplace_back(gpio_1_2_dig_cmd, nullptr);

  std::vector<LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(gpio_1_1_dig_state, nullptr);
  state_interfaces.emplace_back(gpio_1_2_dig_state, nullptr);
  state_interfaces.emplace_back(gpio_2_ana_state, nullptr);

  controller_->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));

  ASSERT_TRUE(is_activate_failed(controller_));
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenAssignedStateInterfacesDoNotMatchInterfacesFromParamsThenControllerShouldFailOnActivation)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});
  const auto result = controller_->init(create_ctrl_params(node_options));

  ASSERT_EQ(result, controller_interface::return_type::OK);

  ASSERT_TRUE(is_configure_succeeded(controller_));

  std::vector<LoanedCommandInterface> command_interfaces;
  command_interfaces.emplace_back(gpio_1_1_dig_cmd, nullptr);
  command_interfaces.emplace_back(gpio_1_2_dig_cmd, nullptr);
  command_interfaces.emplace_back(gpio_2_ana_cmd, nullptr);

  std::vector<LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(gpio_1_1_dig_state, nullptr);
  state_interfaces.emplace_back(gpio_1_2_dig_state, nullptr);

  controller_->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));

  ASSERT_TRUE(is_activate_failed(controller_));
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenCommandInterfacesDontMatchStatesButBothMatchAssignedOnesThenOnActivationShouldSucceed)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});

  move_to_activate_state(
    controller_->init("test_gpio_command_controller", "", 0, "", node_options));
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenThereWasNoCommandForGpiosThenCommandInterfacesShouldHaveDefaultValues)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});

  move_to_activate_state(controller_->init(create_ctrl_params(node_options)));
  assert_default_command_and_state_values();
  update_controller_loop();
  assert_default_command_and_state_values();
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenCommandContainsMoreValuesThenInterfacesNameForGpioUpdateShouldReturnFalse)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});
  move_to_activate_state(controller_->init(create_ctrl_params(node_options)));

  const auto command = createGpioCommand(
    {"gpio1", "gpio2"}, {createInterfaceValue({"dig.1", "dig.2"}, {0.0, 1.0, 1.0}),
                         createInterfaceValue({"ana.1"}, {30.0})});
  controller_->rt_command_.set(command);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::ERROR);
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenCommandContainsMoreInterfacesNameThenValuesForGpioUpdateShouldReturnFalse)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});
  move_to_activate_state(controller_->init(create_ctrl_params(node_options)));

  const auto command = createGpioCommand(
    {"gpio1", "gpio2"},
    {createInterfaceValue({"dig.1", "dig.2"}, {0.0}), createInterfaceValue({"ana.1"}, {30.0})});
  controller_->rt_command_.set(command);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::ERROR);
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenGivenCommandWithValuesForAllInterfacesThenValueOfInterfacesShouldBeUpdated)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});
  move_to_activate_state(controller_->init(create_ctrl_params(node_options)));

  const auto command = createGpioCommand(
    {"gpio1", "gpio2"}, {createInterfaceValue({"dig.1", "dig.2"}, {0.0, 1.0}),
                         createInterfaceValue({"ana.1"}, {30.0})});
  controller_->rt_command_.set(command);
  update_controller_loop();

  ASSERT_EQ(gpio_1_1_dig_cmd->get_optional().value(), 0.0);
  ASSERT_EQ(gpio_1_2_dig_cmd->get_optional().value(), 1.0);
  ASSERT_EQ(gpio_2_ana_cmd->get_optional().value(), 30.0);
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenGivenCommandInterfacesInDifferentOrderThenValueOfInterfacesShouldBeUpdated)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});
  move_to_activate_state(controller_->init(create_ctrl_params(node_options)));

  const auto command = createGpioCommand(
    {"gpio2", "gpio1"}, {createInterfaceValue({"ana.1"}, {30.0}),
                         createInterfaceValue({"dig.2", "dig.1"}, {1.0, 0.0})});
  controller_->rt_command_.set(command);
  update_controller_loop();

  ASSERT_EQ(gpio_1_1_dig_cmd->get_optional().value(), 0.0);
  ASSERT_EQ(gpio_1_2_dig_cmd->get_optional().value(), 1.0);
  ASSERT_EQ(gpio_2_ana_cmd->get_optional().value(), 30.0);
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenGivenCommandWithOnlyOneGpioThenInterfacesValuesShouldBeUpdated)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});
  move_to_activate_state(controller_->init(create_ctrl_params(node_options)));

  const auto command =
    createGpioCommand({"gpio1"}, {createInterfaceValue({"dig.1", "dig.2"}, {0.0, 1.0})});
  controller_->rt_command_.set(command);
  update_controller_loop();

  ASSERT_EQ(gpio_1_1_dig_cmd->get_optional().value(), 0.0);
  ASSERT_EQ(gpio_1_2_dig_cmd->get_optional().value(), 1.0);
  ASSERT_EQ(gpio_2_ana_cmd->get_optional().value(), gpio_commands[2]);
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenGivenCmdContainsWrongGpioInterfacesOrWrongGpioNameThenCommandInterfacesShouldNotBeUpdated)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});
  move_to_activate_state(controller_->init(create_ctrl_params(node_options)));

  const auto command = createGpioCommand(
    {"gpio1", "gpio3"}, {createInterfaceValue({"dig.3", "dig.4"}, {20.0, 25.0}),
                         createInterfaceValue({"ana.1"}, {21.0})});
  controller_->rt_command_.set(command);
  update_controller_loop();

  ASSERT_EQ(gpio_1_1_dig_cmd->get_optional().value(), gpio_commands.at(0));
  ASSERT_EQ(gpio_1_2_dig_cmd->get_optional().value(), gpio_commands.at(1));
  ASSERT_EQ(gpio_2_ana_cmd->get_optional().value(), gpio_commands.at(2));
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenGivenCommandWithValuesForAllInterfacesViaTopicThenValueOfInterfacesShouldBeUpdated)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});
  move_to_activate_state(controller_->init(create_ctrl_params(node_options)));

  auto command_pub = node->create_publisher<CmdType>(
    std::string(controller_->get_node()->get_name()) + "/commands", rclcpp::SystemDefaultsQoS());
  const auto command = createGpioCommand(
    {"gpio1", "gpio2"}, {createInterfaceValue({"dig.1", "dig.2"}, {0.0, 1.0}),
                         createInterfaceValue({"ana.1"}, {30.0})});
  command_pub->publish(command);
  wait_one_miliseconds_for_timeout();
  update_controller_loop();

  ASSERT_EQ(gpio_1_1_dig_cmd->get_optional().value(), 0.0);
  ASSERT_EQ(gpio_1_2_dig_cmd->get_optional().value(), 1.0);
  ASSERT_EQ(gpio_2_ana_cmd->get_optional().value(), 30.0);
}

TEST_F(GpioCommandControllerTestSuite, ControllerShouldPublishGpioStatesWithCurrentValues)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"command_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1", "dig.2"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});
  move_to_activate_state(controller_->init(create_ctrl_params(node_options)));

  auto subscription = node->create_subscription<StateType>(
    std::string(controller_->get_node()->get_name()) + "/gpio_states", 10,
    [&](const StateType::SharedPtr) {});

  stop_test_when_message_cannot_be_published(wait_for_subscription(subscription));

  StateType gpio_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(gpio_state_msg, msg_info));

  ASSERT_EQ(gpio_state_msg.interface_groups.size(), gpio_names.size());
  ASSERT_EQ(gpio_state_msg.interface_groups.at(0), "gpio1");
  ASSERT_EQ(gpio_state_msg.interface_groups.at(1), "gpio2");
  ASSERT_EQ(gpio_state_msg.interface_values.at(0).interface_names.at(0), "dig.1");
  ASSERT_EQ(gpio_state_msg.interface_values.at(0).interface_names.at(1), "dig.2");
  ASSERT_EQ(gpio_state_msg.interface_values.at(1).interface_names.at(0), "ana.1");
  ASSERT_EQ(gpio_state_msg.interface_values.at(0).values.at(0), 1.0);
  ASSERT_EQ(gpio_state_msg.interface_values.at(0).values.at(1), 0.0);
  ASSERT_EQ(gpio_state_msg.interface_values.at(1).values.at(0), 3.1);
}

TEST_F(
  GpioCommandControllerTestSuite,
  WhenStateInterfaceAreNotConfiguredButSetInUrdfForConfiguredGpiosThenThatStatesShouldBePublished)
{
  const std::vector<std::string> single_gpio{"gpio1"};
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", single_gpio},
     {"command_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1"}}});

  std::vector<LoanedCommandInterface> command_interfaces;
  command_interfaces.emplace_back(gpio_1_1_dig_cmd, nullptr);

  std::vector<LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(gpio_1_1_dig_state, nullptr);
  state_interfaces.emplace_back(gpio_2_ana_state, nullptr);

  const auto result_of_initialization =
    controller_->init(create_ctrl_params(node_options, minimal_robot_urdf_with_gpio));
  ASSERT_EQ(result_of_initialization, controller_interface::return_type::OK);

  ASSERT_TRUE(is_configure_succeeded(controller_));

  controller_->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));

  ASSERT_TRUE(is_activate_succeeded(controller_));

  auto subscription = node->create_subscription<StateType>(
    std::string(controller_->get_node()->get_name()) + "/gpio_states", 10,
    [&](const StateType::SharedPtr) {});

  stop_test_when_message_cannot_be_published(wait_for_subscription(subscription));

  StateType gpio_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(gpio_state_msg, msg_info));

  ASSERT_EQ(gpio_state_msg.interface_groups.size(), single_gpio.size());
  ASSERT_EQ(gpio_state_msg.interface_groups.at(0), "gpio1");
  ASSERT_EQ(gpio_state_msg.interface_values.at(0).interface_names.at(0), "dig.1");
  ASSERT_EQ(gpio_state_msg.interface_values.at(0).values.at(0), 1.0);
}

TEST_F(
  GpioCommandControllerTestSuite,
  ControllerShouldPublishGpioStatesWithCurrentValuesWhenOnlyStateInterfacesAreSet)
{
  const auto node_options = create_node_options_with_overriden_parameters(
    {{"gpios", gpio_names},
     {"state_interfaces.gpio1.interfaces", std::vector<std::string>{"dig.1"}},
     {"state_interfaces.gpio2.interfaces", std::vector<std::string>{"ana.1"}}});

  std::vector<LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(gpio_1_1_dig_state, nullptr);
  state_interfaces.emplace_back(gpio_2_ana_state, nullptr);
  const auto result_of_initialization = controller_->init(create_ctrl_params(node_options));
  ASSERT_EQ(result_of_initialization, controller_interface::return_type::OK);

  ASSERT_TRUE(is_configure_succeeded(controller_));

  controller_->assign_interfaces({}, std::move(state_interfaces));

  ASSERT_TRUE(is_activate_succeeded(controller_));

  auto subscription = node->create_subscription<StateType>(
    std::string(controller_->get_node()->get_name()) + "/gpio_states", 10,
    [&](const StateType::SharedPtr) {});

  stop_test_when_message_cannot_be_published(wait_for_subscription(subscription));

  StateType gpio_state_msg;
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(gpio_state_msg, msg_info));

  ASSERT_EQ(gpio_state_msg.interface_groups.size(), gpio_names.size());
  ASSERT_EQ(gpio_state_msg.interface_groups.at(0), "gpio1");
  ASSERT_EQ(gpio_state_msg.interface_groups.at(1), "gpio2");
  ASSERT_EQ(gpio_state_msg.interface_values.at(0).interface_names.at(0), "dig.1");
  ASSERT_EQ(gpio_state_msg.interface_values.at(1).interface_names.at(0), "ana.1");
  ASSERT_EQ(gpio_state_msg.interface_values.at(0).values.at(0), 1.0);
  ASSERT_EQ(gpio_state_msg.interface_values.at(1).values.at(0), 3.1);
}
