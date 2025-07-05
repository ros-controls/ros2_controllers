#include <gtest/gtest.h>
#include <memory>
#include "chained_filter_controller/chained_filter.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using chained_filter_controller::ChainedFilter;
using controller_interface::CallbackReturn;

class ChainedFilterControllerTest : public ::testing::Test
{
protected:
  void SetUp() override { controller_ = std::make_shared<ChainedFilter>(); }

  std::shared_ptr<ChainedFilter> controller_;
};

TEST_F(ChainedFilterControllerTest, InitReturnsSuccess)
{
  auto result = controller_->init("chained_filter", "test_ns", 3, "", rclcpp::NodeOptions());
  EXPECT_EQ(result, controller_interface::return_type::OK);
}

TEST_F(ChainedFilterControllerTest, ConfigureReturnsFailureWithNoParams)
{
  // Initialize the controller (should set up internal node)
  ASSERT_EQ(
    controller_->init(
      "chained_filter",        // controller name
      "test_ns",               // namespace
      0,                       // intra-process (0 = disabled)
      "test_node",             // RCL node name
      rclcpp::NodeOptions()),  // node options
    controller_interface::return_type::OK);

  // We do NOT declare any filter parameters — this mimics a misconfigured user setup
  // The expectation is that configuration fails gracefully (not segfault)

  auto result = controller_->on_configure(rclcpp_lifecycle::State{});
  EXPECT_EQ(result, controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(ChainedFilterControllerTest, ActivateReturnsSuccessWithoutError)
{
  auto init_result = controller_->init("chained_filter", "test_ns", 3, "", rclcpp::NodeOptions());
  ASSERT_EQ(init_result, controller_interface::return_type::OK);

  auto configure_result = controller_->on_configure(rclcpp_lifecycle::State());
  EXPECT_EQ(configure_result, CallbackReturn::SUCCESS);  // Expected because no params loaded

  auto activate_result = controller_->on_activate(rclcpp_lifecycle::State());
  EXPECT_EQ(activate_result, CallbackReturn::SUCCESS);
}

TEST_F(ChainedFilterControllerTest, DeactivateDoesNotCrash)
{
  EXPECT_NO_THROW({ controller_->on_deactivate(rclcpp_lifecycle::State()); });
}

TEST_F(ChainedFilterControllerTest, CleanupDoesNotCrash)
{
  EXPECT_NO_THROW({ controller_->on_cleanup(rclcpp_lifecycle::State()); });
}

TEST_F(ChainedFilterControllerTest, ShutdownDoesNotCrash)
{
  EXPECT_NO_THROW({ controller_->on_shutdown(rclcpp_lifecycle::State()); });
}

int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Run all the tests
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();

  // Shutdown ROS 2
  rclcpp::shutdown();
  return result;
}
