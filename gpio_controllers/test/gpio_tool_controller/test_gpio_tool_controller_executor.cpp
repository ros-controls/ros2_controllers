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

// End-to-end tests with a running ROS executor.
//
// Two threads run concurrently:
//   • executor_thread_ – spins a MultiThreadedExecutor so that service/action
//     callbacks (including the blocking service_wait_for_transition_end) can
//     progress without starving the test.
//   • controller_thread_ – calls controller_->update() at ~100 Hz, followed by
//     a hardware-simulator callback that updates state interfaces in response to
//     command interface values, mimicking real hardware feedback.
//
// Hardware simulator response for the standard setup_parameters() fixture:
//   • Close_valve=1   → Closed_signal=1, Opened_signal=0
//   • Open_valve=1    → Opened_signal=1, Closed_signal=0
//   • Release_Something=1 → Break_Engaged=1
//   • Release_Break_valve=1, Release_Something=0 → Break_Engaged=0
//
// This is sufficient to drive the 6-step state machine through all transitions
// for both engage and disengage actions.

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "control_msgs/action/gpio_tool_command.hpp"
#include "control_msgs/action/set_gpio_tool_config.hpp"
#include "control_msgs/msg/gpio_tool_controller_state.hpp"
#include "control_msgs/srv/set_gpio_tool_config.hpp"
#include "test_gpio_tool_controller.hpp"

// ============================================================================
// Executor-test fixture
// ============================================================================

class GpioToolControllerExecutorTest : public IOGripperControllerFixture<TestableGpioToolController>
{
public:
  void TearDown() override
  {
    StopController();
    ShutdownExecutor();
    IOGripperControllerFixture::TearDown();
  }

  // Create the executor, add the controller node and a separate client node,
  // then spin in a background thread.
  void SetUpExecutor()
  {
    client_node_ = std::make_shared<rclcpp::Node>("test_client_node");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(controller_->get_node()->get_node_base_interface());
    executor_->add_node(client_node_);
    executor_future_ = std::async(std::launch::async, [this]() { executor_->spin(); });

    // Give the executor a moment to start before sending requests.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Start the RT update loop in a background thread.
  // hw_sim is called after every update() to simulate hardware feedback.
  void StartControllerThread(std::function<void()> hw_sim = {})
  {
    stop_controller_ = false;
    controller_thread_ = std::thread(
      [this, hw_sim]()
      {
        int64_t current_ns = 0;
        constexpr int64_t step_ns = 10'000'000;  // 10 ms → 100 Hz
        while (!stop_controller_)
        {
          controller_->update(
            rclcpp::Time(current_ns, RCL_ROS_TIME), rclcpp::Duration::from_nanoseconds(step_ns));
          if (hw_sim)
          {
            hw_sim();
          }
          current_ns += step_ns;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      });
  }

  void StopController()
  {
    stop_controller_ = true;
    if (controller_thread_.joinable())
    {
      controller_thread_.join();
    }
  }

  void ShutdownExecutor()
  {
    if (executor_)
    {
      executor_->cancel();
      if (executor_future_.valid())
      {
        executor_future_.wait();
      }
    }
  }

  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::future<void> executor_future_;
  std::thread controller_thread_;
  std::atomic<bool> stop_controller_{false};
};

// ============================================================================
// Hardware simulator
//
// Called from the controller update thread after every update().  It mirrors
// the command interface values back onto the state interfaces so that the
// controller's CHECK_COMMAND and CHECK_AFTER_COMMAND transitions can advance.
// ============================================================================

namespace
{
// Returns a hardware simulator lambda for the standard setup_parameters() fixture.
// capture-by-reference: state_values_ and cmd_name_to_index_ / state_name_to_index_
// are owned by the fixture and outlive the lambda.
auto make_hw_sim(IOGripperControllerFixture<TestableGpioToolController> & fx)
{
  return [&fx]()
  {
    // Main valve commands → tool state
    if (fx.GetCmdValue("Close_valve") > 0.5)
    {
      fx.SetStateValue("Closed_signal", 1.0);
      fx.SetStateValue("Opened_signal", 0.0);
    }
    else if (fx.GetCmdValue("Open_valve") > 0.5)
    {
      fx.SetStateValue("Opened_signal", 1.0);
      fx.SetStateValue("Closed_signal", 0.0);
    }

    // After-command signals → brake state
    if (fx.GetCmdValue("Release_Something") > 0.5)
    {
      fx.SetStateValue("Break_Engaged", 1.0);
    }
    else if (fx.GetCmdValue("Release_Break_valve") > 0.5)
    {
      fx.SetStateValue("Break_Engaged", 0.0);
    }
  };
}

// Full configure + setup_parameters + activate helper.
void prepare_for_executor_test(
  GpioToolControllerExecutorTest & fx, const std::vector<std::string> & possible_states,
  const std::string & initial_hw_state, bool use_action = false)
{
  fx.SetUpController(
    "test_gpio_tool_controller", {rclcpp::Parameter("possible_engaged_states", possible_states),
                                  rclcpp::Parameter("use_action", use_action)});
  fx.setup_parameters();
  fx.controller_->get_node()->set_parameter({"use_action", use_action});

  ASSERT_EQ(
    fx.controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  fx.SetupInterfaces();
  fx.SetInitialHardwareState(initial_hw_state);
  ASSERT_EQ(
    fx.controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

// Configure + setup_parameters_with_config + activate helper for configuration
// action / service tests. The initial_hw_state must be one of the values
// understood by SetInitialHardwareState (e.g. "narrow_objects").
void prepare_for_executor_test_with_config(
  GpioToolControllerExecutorTest & fx, const std::string & initial_hw_state,
  bool use_action = false, double timeout = 5.0)
{
  const std::vector<std::string> possible_states = {"close_empty", "close_full"};
  fx.SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_states),
     rclcpp::Parameter(
       "configurations", std::vector<std::string>{"narrow_objects", "wide_objects"}),
     rclcpp::Parameter("configuration_joints", std::vector<std::string>{"gripper_distance_joint"}),
     rclcpp::Parameter("use_action", use_action), rclcpp::Parameter("timeout", timeout)});
  fx.setup_parameters_with_config();
  fx.controller_->get_node()->set_parameter({"use_action", use_action});
  fx.controller_->get_node()->set_parameter({"timeout", timeout});

  ASSERT_EQ(
    fx.controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  fx.SetupInterfaces();
  fx.SetInitialHardwareState(initial_hw_state);
  ASSERT_EQ(
    fx.controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

// Hardware simulator that also responds to configuration command interfaces.
auto make_config_hw_sim(IOGripperControllerFixture<TestableGpioToolController> & fx)
{
  return [&fx]()
  {
    // Main valve commands → tool state
    if (fx.GetCmdValue("Close_valve") > 0.5)
    {
      fx.SetStateValue("Closed_signal", 1.0);
      fx.SetStateValue("Opened_signal", 0.0);
    }
    else if (fx.GetCmdValue("Open_valve") > 0.5)
    {
      fx.SetStateValue("Opened_signal", 1.0);
      fx.SetStateValue("Closed_signal", 0.0);
    }

    // After-command signals → brake state
    if (fx.GetCmdValue("Release_Something") > 0.5)
    {
      fx.SetStateValue("Break_Engaged", 1.0);
    }
    else if (fx.GetCmdValue("Release_Break_valve") > 0.5)
    {
      fx.SetStateValue("Break_Engaged", 0.0);
    }

    // Configuration commands → configuration signals
    if (fx.GetCmdValue("Wide_Configuration_Cmd") > 0.5)
    {
      fx.SetStateValue("Wide_Configuration_Signal", 1.0);
      fx.SetStateValue("Narrow_Configuraiton_Signal", 0.0);
    }
    else if (fx.GetCmdValue("Narrow_Configuration_Cmd") > 0.5)
    {
      fx.SetStateValue("Narrow_Configuraiton_Signal", 1.0);
      fx.SetStateValue("Wide_Configuration_Signal", 0.0);
    }
  };
}
}  // namespace

// ============================================================================
// Service-mode tests (use_action=false)
// ============================================================================

// ---------------------------------------------------------------------------
// Calling ~/engaged service while in "open" state completes successfully.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerExecutorTest, ServiceEngageCompletesSuccessfully)
{
  prepare_for_executor_test(*this, possible_engaged_states, "open", false);
  SetUpExecutor();
  StartControllerThread(make_hw_sim(*this));

  auto client =
    client_node_->create_client<std_srvs::srv::Trigger>("/test_gpio_tool_controller/engaged");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  // The service blocks until the state machine completes (≤ a few seconds).
  auto status = future.wait_for(std::chrono::seconds(15));
  ASSERT_EQ(status, std::future_status::ready);

  auto response = future.get();
  EXPECT_TRUE(response->success);

  // The controller must have returned to IDLE and reached a known engaged state.
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_NE(controller_->get_current_state(), "open");
}

// ---------------------------------------------------------------------------
// Calling ~/open service while in "close_empty" state completes successfully.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerExecutorTest, ServiceDisengageCompletesSuccessfully)
{
  prepare_for_executor_test(*this, possible_engaged_states, "close_empty", false);
  SetUpExecutor();
  StartControllerThread(make_hw_sim(*this));

  auto client =
    client_node_->create_client<std_srvs::srv::Trigger>("/test_gpio_tool_controller/open");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  auto status = future.wait_for(std::chrono::seconds(15));
  ASSERT_EQ(status, std::future_status::ready);

  auto response = future.get();
  EXPECT_TRUE(response->success);

  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_EQ(controller_->get_current_state(), "open");
}

// ============================================================================
// Action-mode tests (use_action=true)
// ============================================================================

// ---------------------------------------------------------------------------
// Sending an engage action goal from "open" state succeeds.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerExecutorTest, ActionEngageGoalSucceeds)
{
  using EngagingActionType = control_msgs::action::GPIOToolCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<EngagingActionType>;
  using GoalOptions = rclcpp_action::Client<EngagingActionType>::SendGoalOptions;

  prepare_for_executor_test(*this, possible_engaged_states, "open", true);
  SetUpExecutor();
  StartControllerThread(make_hw_sim(*this));

  auto action_client = rclcpp_action::create_client<EngagingActionType>(
    client_node_, "/test_gpio_tool_controller/engaged_open");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)));

  std::atomic<bool> result_received{false};
  rclcpp_action::ResultCode result_code{rclcpp_action::ResultCode::UNKNOWN};
  bool result_success{false};

  GoalOptions opts;
  opts.result_callback = [&](const GoalHandle::WrappedResult & result)
  {
    result_code = result.code;
    result_success = result.result->success;
    result_received = true;
  };

  auto goal = EngagingActionType::Goal();
  goal.engage = true;
  auto goal_handle_future = action_client->async_send_goal(goal, opts);

  // Wait for acceptance
  ASSERT_EQ(goal_handle_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);  // goal was accepted

  // Wait for result callback
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!result_received && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_TRUE(result_received);

  EXPECT_EQ(result_code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result_success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
}

// ---------------------------------------------------------------------------
// Sending a disengage action goal from "close_empty" state succeeds.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerExecutorTest, ActionDisengageGoalSucceeds)
{
  using EngagingActionType = control_msgs::action::GPIOToolCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<EngagingActionType>;
  using GoalOptions = rclcpp_action::Client<EngagingActionType>::SendGoalOptions;

  prepare_for_executor_test(*this, possible_engaged_states, "close_empty", true);
  SetUpExecutor();
  StartControllerThread(make_hw_sim(*this));

  auto action_client = rclcpp_action::create_client<EngagingActionType>(
    client_node_, "/test_gpio_tool_controller/engaged_open");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)));

  std::atomic<bool> result_received{false};
  rclcpp_action::ResultCode result_code{rclcpp_action::ResultCode::UNKNOWN};
  bool result_success{false};

  GoalOptions opts;
  opts.result_callback = [&](const GoalHandle::WrappedResult & result)
  {
    result_code = result.code;
    result_success = result.result->success;
    result_received = true;
  };

  auto goal = EngagingActionType::Goal();
  goal.engage = false;  // disengage
  auto goal_handle_future = action_client->async_send_goal(goal, opts);

  ASSERT_EQ(goal_handle_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!result_received && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_TRUE(result_received);

  EXPECT_EQ(result_code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result_success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  EXPECT_EQ(controller_->get_current_state(), "open");
}

// ---------------------------------------------------------------------------
// Cancelling an in-progress action results in ABORTED (CANCELING → HALTED).
//
// The hardware simulator is intentionally absent so that the action stalls in
// CHECK_COMMAND (hardware never confirms the closed state).  The action server
// aborts when the check-command timeout fires (the cancel signal cannot
// interrupt handle_action_accepted because both share a MutuallyExclusive
// callback group).  A short 1-second timeout keeps the test fast.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerExecutorTest, ActionCancelGoalResultsInAbort)
{
  using EngagingActionType = control_msgs::action::GPIOToolCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<EngagingActionType>;
  using GoalOptions = rclcpp_action::Client<EngagingActionType>::SendGoalOptions;

  // Use a short timeout so the test doesn't block for 5 seconds.
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_engaged_states),
     rclcpp::Parameter("use_action", true), rclcpp::Parameter("timeout", 1.0)});
  setup_parameters();
  controller_->get_node()->set_parameter({"use_action", true});
  controller_->get_node()->set_parameter({"timeout", 1.0});
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetupInterfaces();
  SetInitialHardwareState("open");
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);

  SetUpExecutor();

  // No hw_sim – action will stall in CHECK_COMMAND waiting for Closed_signal.
  StartControllerThread();

  auto action_client = rclcpp_action::create_client<EngagingActionType>(
    client_node_, "/test_gpio_tool_controller/engaged_open");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)));

  std::atomic<bool> result_received{false};
  rclcpp_action::ResultCode result_code{rclcpp_action::ResultCode::UNKNOWN};

  GoalOptions opts;
  opts.result_callback = [&](const GoalHandle::WrappedResult & result)
  {
    result_code = result.code;
    result_received = true;
  };

  auto goal = EngagingActionType::Goal();
  goal.engage = true;
  auto goal_handle_future = action_client->async_send_goal(goal, opts);

  ASSERT_EQ(goal_handle_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);  // goal was accepted and is now running

  // Cancel the goal shortly after it is accepted.  The cancel signal cannot
  // immediately preempt handle_action_accepted (MutuallyExclusive callback
  // group), so the action will eventually abort via the 1-second CHECK_COMMAND
  // timeout instead.  In both cases the final result code is ABORTED.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  auto cancel_future = action_client->async_cancel_goal(goal_handle);
  // Don't assert on cancel_future – the cancel may be superseded by timeout abort.
  cancel_future.wait_for(std::chrono::seconds(3));

  // Wait for the result callback (should arrive within ~2 s of the 1 s timeout).
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!result_received && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_TRUE(result_received);

  EXPECT_EQ(result_code, rclcpp_action::ResultCode::ABORTED);
}

// ============================================================================
// Configuration action tests (use_action=true, configurations enabled)
// ============================================================================

// ---------------------------------------------------------------------------
// Sending a reconfigure action goal from "narrow_objects" succeeds when the
// hw_sim responds to the configuration command interfaces.
// Covers handle_config_goal(), handle_config_cancel(), and the
// handle_action_accepted<ConfigActionType> success path.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerExecutorTest, ConfigActionGoalSucceeds)
{
  using ConfigActionType = control_msgs::action::SetGPIOToolConfig;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ConfigActionType>;
  using GoalOptions = rclcpp_action::Client<ConfigActionType>::SendGoalOptions;

  prepare_for_executor_test_with_config(*this, "narrow_objects", true);
  ASSERT_EQ(controller_->get_current_action(), ToolAction::IDLE);
  ASSERT_EQ(controller_->get_current_state(), "open");

  SetUpExecutor();
  StartControllerThread(make_config_hw_sim(*this));

  auto action_client = rclcpp_action::create_client<ConfigActionType>(
    client_node_, "/test_gpio_tool_controller/reconfigure");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)));

  std::atomic<bool> result_received{false};
  rclcpp_action::ResultCode result_code{rclcpp_action::ResultCode::UNKNOWN};
  bool result_success{false};

  GoalOptions opts;
  opts.result_callback = [&](const GoalHandle::WrappedResult & result)
  {
    result_code = result.code;
    result_success = result.result->success;
    result_received = true;
  };

  auto goal = ConfigActionType::Goal();
  goal.config_name = "wide_objects";
  auto goal_handle_future = action_client->async_send_goal(goal, opts);

  ASSERT_EQ(goal_handle_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);  // goal accepted

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
  while (!result_received && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_TRUE(result_received);

  EXPECT_EQ(result_code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result_success);
  EXPECT_EQ(controller_->get_current_action(), ToolAction::IDLE);
}

// ---------------------------------------------------------------------------
// Sending a reconfigure action goal while the tool is engaged (not disengaged)
// causes the goal to be REJECTED by handle_config_goal().
// Covers the !result.success → REJECT path in handle_config_goal()
// (gpio_tool_controller.cpp lines 1313-1318).
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerExecutorTest, ConfigActionGoalRejectedWhenEngaged)
{
  using ConfigActionType = control_msgs::action::SetGPIOToolConfig;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ConfigActionType>;
  using GoalOptions = rclcpp_action::Client<ConfigActionType>::SendGoalOptions;

  // Start with close_empty tool state + narrow_objects config so on_activate() succeeds.
  const std::vector<std::string> possible_states = {"close_empty", "close_full"};
  SetUpController(
    "test_gpio_tool_controller",
    {rclcpp::Parameter("possible_engaged_states", possible_states),
     rclcpp::Parameter(
       "configurations", std::vector<std::string>{"narrow_objects", "wide_objects"}),
     rclcpp::Parameter("configuration_joints", std::vector<std::string>{"gripper_distance_joint"}),
     rclcpp::Parameter("use_action", true)});
  setup_parameters_with_config();
  controller_->get_node()->set_parameter({"use_action", true});
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  SetupInterfaces();
  // Set close_empty tool state + narrow_objects configuration
  SetStateValue("Closed_signal", 1.0);
  SetStateValue("Narrow_Configuraiton_Signal", 1.0);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->get_current_state(), "close_empty");

  SetUpExecutor();
  // No controller thread needed – goal is rejected before any state machine runs.

  auto action_client = rclcpp_action::create_client<ConfigActionType>(
    client_node_, "/test_gpio_tool_controller/reconfigure");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)));

  std::atomic<bool> response_received{false};
  std::shared_ptr<GoalHandle> received_goal_handle{nullptr};

  GoalOptions opts;
  opts.goal_response_callback = [&](const std::shared_ptr<GoalHandle> & gh)
  {
    received_goal_handle = gh;
    response_received = true;
  };

  auto goal = ConfigActionType::Goal();
  goal.config_name = "wide_objects";
  action_client->async_send_goal(goal, opts);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!response_received && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_TRUE(response_received);
  // Goal must have been rejected: goal handle is null when the server sends REJECT.
  EXPECT_EQ(received_goal_handle, nullptr);
}

// ---------------------------------------------------------------------------
// Cancelling a running reconfigure action causes it to be ABORTED.
//
// No hw_sim → SET_COMMAND fires but CHECK_COMMAND stalls (Wide_Configuration_Signal
// is never asserted).  The cancel sets CANCELING; the 1-second timeout then fires
// HALTED; handle_action_accepted<ConfigActionType> aborts the goal.
// Covers handle_config_cancel() (gpio_tool_controller.cpp line 1324) and the
// HALTED abort path in handle_action_accepted<ConfigActionType>.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerExecutorTest, ConfigActionCancelGoalResultsInAbort)
{
  using ConfigActionType = control_msgs::action::SetGPIOToolConfig;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ConfigActionType>;
  using GoalOptions = rclcpp_action::Client<ConfigActionType>::SendGoalOptions;

  // Short timeout so the test completes quickly.
  prepare_for_executor_test_with_config(*this, "narrow_objects", true, /*timeout=*/1.0);
  ASSERT_EQ(controller_->get_current_action(), ToolAction::IDLE);

  SetUpExecutor();
  // No hw_sim – action stalls in CHECK_COMMAND waiting for Wide_Configuration_Signal.
  StartControllerThread();

  auto action_client = rclcpp_action::create_client<ConfigActionType>(
    client_node_, "/test_gpio_tool_controller/reconfigure");
  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)));

  std::atomic<bool> result_received{false};
  rclcpp_action::ResultCode result_code{rclcpp_action::ResultCode::UNKNOWN};

  GoalOptions opts;
  opts.result_callback = [&](const GoalHandle::WrappedResult & result)
  {
    result_code = result.code;
    result_received = true;
  };

  auto goal = ConfigActionType::Goal();
  goal.config_name = "wide_objects";
  auto goal_handle_future = action_client->async_send_goal(goal, opts);

  ASSERT_EQ(goal_handle_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);  // goal accepted, now running

  // Cancel shortly after acceptance.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  auto cancel_future = action_client->async_cancel_goal(goal_handle);
  cancel_future.wait_for(std::chrono::seconds(3));

  // Wait for result callback (should arrive within ~2 s of the 1-second CHECK_COMMAND timeout).
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!result_received && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_TRUE(result_received);
  EXPECT_EQ(result_code, rclcpp_action::ResultCode::ABORTED);
}

// ============================================================================
// service_wait_for_transition_end() HALTED path
// ============================================================================

// ---------------------------------------------------------------------------
// When the reconfigure service is called but hardware never confirms the
// target configuration, the state machine times out (HALTED) and
// service_wait_for_transition_end() returns success=false.
// Covers gpio_tool_controller.cpp lines 1031-1036.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerExecutorTest, ServiceReconfigureHaltedReturnsFailure)
{
  using ConfigSrvType = control_msgs::srv::SetGPIOToolConfig;

  // service mode (use_action=false), short 1-second timeout.
  prepare_for_executor_test_with_config(
    *this, "narrow_objects", /*use_action=*/false, /*timeout=*/1.0);
  ASSERT_EQ(controller_->get_current_action(), ToolAction::IDLE);

  SetUpExecutor();
  // No hw_sim – reconfigure stalls at CHECK_COMMAND → 1-second timeout → HALTED.
  StartControllerThread();

  auto client =
    client_node_->create_client<ConfigSrvType>("/test_gpio_tool_controller/reconfigure");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto request = std::make_shared<ConfigSrvType::Request>();
  request->config_name = "wide_objects";
  auto future = client->async_send_request(request);

  // The service blocks in service_wait_for_transition_end() until HALTED is detected.
  auto status = future.wait_for(std::chrono::seconds(10));
  ASSERT_EQ(status, std::future_status::ready);

  auto response = future.get();
  // HALTED path must produce success=false
  EXPECT_FALSE(response->success);
}

// ============================================================================
// publish_topics() output validation
// ============================================================================

// ---------------------------------------------------------------------------
// After update() is called the controller publishes ~/controller_state with
// the current tool state and IDLE transition.
// Verifies that publish_topics() emits correctly populated messages.
// ---------------------------------------------------------------------------
TEST_F(GpioToolControllerExecutorTest, PublishTopicsMatchControllerState)
{
  using ControllerStateMsg = control_msgs::msg::GPIOToolControllerState;

  prepare_for_executor_test(*this, possible_engaged_states, "open", false);
  ASSERT_EQ(controller_->get_current_state(), "open");

  SetUpExecutor();

  std::atomic<bool> msg_received{false};
  ControllerStateMsg received_msg;
  auto sub = client_node_->create_subscription<ControllerStateMsg>(
    "/test_gpio_tool_controller/controller_state", rclcpp::SystemDefaultsQoS(),
    [&](const ControllerStateMsg::SharedPtr msg)
    {
      received_msg = *msg;
      msg_received = true;
    });

  StartControllerThread();

  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!msg_received && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  StopController();

  ASSERT_TRUE(msg_received);
  EXPECT_EQ(received_msg.state, "open");
  EXPECT_EQ(received_msg.current_transition.state, GPIOToolTransition::IDLE);
}
