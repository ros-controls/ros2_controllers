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
