// Copyright (c) 2025, ros2_control development team
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

/*
 * Authors: Julia Jia
 */

#include <gmock/gmock.h>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "force_torque_sensor_broadcaster/wrench_transformer.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(18, 0, 0)
#include "rclcpp/node_interfaces/node_interfaces.hpp"
#endif
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.h"

class TestWrenchTransformer : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  void setup_static_transform(
    const std::string & parent_frame, const std::string & child_frame, double x = 0.0,
    double y = 0.0, double z = 0.0, double qx = 0.0, double qy = 0.0, double qz = 0.0,
    double qw = 1.0)
  {
    auto tf_node = std::make_shared<rclcpp::Node>("static_tf_broadcaster");
    executor_->add_node(tf_node);
#if RCLCPP_VERSION_GTE(18, 0, 0)
    tf2_ros::StaticTransformBroadcaster tf_broadcaster(
      rclcpp::node_interfaces::NodeInterfaces(
        tf_node->get_node_parameters_interface(), tf_node->get_node_topics_interface()));
#else
    tf2_ros::StaticTransformBroadcaster tf_broadcaster(tf_node);
#endif

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = tf_node->get_clock()->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation.x = qx;
    transform.transform.rotation.y = qy;
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;

    tf_broadcaster.sendTransform(transform);
    executor_->spin_some(std::chrono::milliseconds(100));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::shared_ptr<force_torque_sensor_broadcaster::WrenchTransformer> create_transformer_node(
    const std::string & broadcaster_namespace = "test_broadcaster", bool use_filtered_input = false,
    const std::vector<std::string> & target_frames = {"base_link"},
    const std::string & output_topic_prefix = "~/wrench_transformed", double tf_timeout = 0.1)
  {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters;
    parameters.emplace_back("broadcaster_namespace", broadcaster_namespace);
    parameters.emplace_back("use_filtered_input", use_filtered_input);
    parameters.emplace_back("target_frames", target_frames);
    parameters.emplace_back("output_topic_prefix", output_topic_prefix);
    parameters.emplace_back("tf_timeout", tf_timeout);
    options.parameter_overrides(parameters);

    auto node = std::make_shared<force_torque_sensor_broadcaster::WrenchTransformer>(options);
    executor_->add_node(node);
    node->init();
    return node;
  }

  void wait_for_discovery()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    executor_->spin_some(std::chrono::milliseconds(100));
  }

  void wait_for_publisher(
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber,
    int max_attempts = 10)
  {
    while (max_attempts-- > 0 && subscriber->get_publisher_count() == 0)
    {
      executor_->spin_some(std::chrono::milliseconds(50));
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  geometry_msgs::msg::WrenchStamped create_test_wrench(
    rclcpp::Clock::SharedPtr clock, const std::string & frame_id = "sensor_frame")
  {
    geometry_msgs::msg::WrenchStamped test_msg;
    test_msg.header.stamp = clock->now();
    test_msg.header.frame_id = frame_id;
    test_msg.wrench.force.x = 1.0;
    test_msg.wrench.force.y = 2.0;
    test_msg.wrench.force.z = 3.0;
    test_msg.wrench.torque.x = 0.1;
    test_msg.wrench.torque.y = 0.2;
    test_msg.wrench.torque.z = 0.3;
    return test_msg;
  }

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr create_input_publisher(
    const std::string & topic_name)
  {
    auto publisher_node = std::make_shared<rclcpp::Node>("test_publisher");
    auto publisher = publisher_node->create_publisher<geometry_msgs::msg::WrenchStamped>(
      topic_name, rclcpp::SystemDefaultsQoS());
    executor_->add_node(publisher_node);
    return publisher;
  }

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr create_output_subscriber(
    const std::string & topic_name,
    std::function<void(const geometry_msgs::msg::WrenchStamped::SharedPtr)> callback)
  {
    auto subscriber_node = std::make_shared<rclcpp::Node>("test_subscriber");
    auto subscriber = subscriber_node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topic_name, rclcpp::SystemDefaultsQoS(), callback);
    executor_->add_node(subscriber_node);
    return subscriber;
  }
};

TEST_F(TestWrenchTransformer, NodeInitialization)
{
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("target_frames", std::vector<std::string>{"base_link"});

  options.parameter_overrides(parameters);

  auto node = std::make_shared<force_torque_sensor_broadcaster::WrenchTransformer>(options);
  executor_->add_node(node);
  node->init();

  ASSERT_NE(node, nullptr);
  EXPECT_EQ(std::string(node->get_name()), "fts_wrench_transformer");
}

TEST_F(TestWrenchTransformer, MultipleTargetFrames)
{
  auto node = create_transformer_node("test_broadcaster", false, {"base_link", "end_effector"});

  executor_->spin_some(std::chrono::milliseconds(100));

  // Verify publishers exist by creating test subscribers
  // Use fully qualified topic name since ~/ expands to the transformer node's namespace
  auto test_sub_node = std::make_shared<rclcpp::Node>("test_sub_check");
  auto base_link_sub = test_sub_node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/fts_wrench_transformer/wrench_transformed_base_link", rclcpp::SystemDefaultsQoS(),
    [](const geometry_msgs::msg::WrenchStamped::SharedPtr) {});
  auto end_effector_sub = test_sub_node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/fts_wrench_transformer/wrench_transformed_end_effector", rclcpp::SystemDefaultsQoS(),
    [](const geometry_msgs::msg::WrenchStamped::SharedPtr) {});
  executor_->add_node(test_sub_node);

  wait_for_discovery();

  // Wait for each publisher to be discovered
  wait_for_publisher(base_link_sub);
  wait_for_publisher(end_effector_sub);

  ASSERT_NE(node, nullptr);
  EXPECT_GT(base_link_sub->get_publisher_count(), 0u)
    << "Publisher not found on /fts_wrench_transformer/wrench_transformed_base_link";
  EXPECT_GT(end_effector_sub->get_publisher_count(), 0u)
    << "Publisher not found on /fts_wrench_transformer/wrench_transformed_end_effector";
}

TEST_F(TestWrenchTransformer, PublishSubscribeFlow)
{
  // Create transformer node first so its TF listener can receive transforms
  auto transformer_node = create_transformer_node();
  executor_->spin_some(std::chrono::milliseconds(100));

  // Setup static transform from sensor_frame to base_link
  setup_static_transform("base_link", "sensor_frame", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

  // Wait longer for TF buffer to receive the transform
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  executor_->spin_some(std::chrono::milliseconds(200));

  // Create publisher and subscriber using helper functions
  auto input_publisher = create_input_publisher("test_broadcaster/wrench");
  geometry_msgs::msg::WrenchStamped::SharedPtr received_msg;
  auto output_subscriber = create_output_subscriber(
    "/fts_wrench_transformer/wrench_transformed_base_link",
    [&received_msg](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    { received_msg = msg; });

  wait_for_discovery();

  // Wait for publisher to be discovered by the output subscriber
  wait_for_publisher(output_subscriber);

  // Publish test message
  auto test_msg = create_test_wrench(transformer_node->get_clock());
  input_publisher->publish(test_msg);

  // Wait for message to be processed
  int max_attempts = 20;
  while (max_attempts-- > 0 && !received_msg)
  {
    executor_->spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Verify message was received and transformed
  ASSERT_NE(transformer_node, nullptr);
  ASSERT_TRUE(received_msg != nullptr) << "Transformed message was not received";
  EXPECT_EQ(received_msg->header.frame_id, "base_link");
  // Compare timestamp components (tf2::doTransform may modify timestamp slightly)
  EXPECT_EQ(received_msg->header.stamp.sec, test_msg.header.stamp.sec);
  EXPECT_EQ(received_msg->header.stamp.nanosec, test_msg.header.stamp.nanosec);
  // With identity transform, values should be the same
  EXPECT_DOUBLE_EQ(received_msg->wrench.force.x, test_msg.wrench.force.x);
  EXPECT_DOUBLE_EQ(received_msg->wrench.force.y, test_msg.wrench.force.y);
  EXPECT_DOUBLE_EQ(received_msg->wrench.force.z, test_msg.wrench.force.z);
  EXPECT_DOUBLE_EQ(received_msg->wrench.torque.x, test_msg.wrench.torque.x);
  EXPECT_DOUBLE_EQ(received_msg->wrench.torque.y, test_msg.wrench.torque.y);
  EXPECT_DOUBLE_EQ(received_msg->wrench.torque.z, test_msg.wrench.torque.z);
}

TEST_F(TestWrenchTransformer, NonIdentityTransform)
{
  // Create transformer node first so its TF listener can receive transforms
  auto transformer_node = create_transformer_node();
  executor_->spin_some(std::chrono::milliseconds(100));

  // Setup non-identity transform: 90-degree rotation around z-axis
  // Use case: sensors can be mounted rotated (e.g., 90° around z) on links,
  // grippers, or end-effectors, requiring rotation transforms.
  // Quaternion for 90° rotation around z-axis: For a rotation of angle θ around axis (x,y,z),
  // the quaternion is (x*sin(θ/2), y*sin(θ/2), z*sin(θ/2), cos(θ/2)).
  // For 90° around z-axis: θ=90°, axis=(0,0,1), so (0, 0, sin(45°), cos(45°)) = (0, 0, 0.7071,
  // 0.7071)
  setup_static_transform("base_link", "sensor_frame", 0.0, 0.0, 0.0, 0.0, 0.0, 0.7071, 0.7071);

  // Wait longer for TF buffer to receive the transform
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  executor_->spin_some(std::chrono::milliseconds(200));

  // Create publisher and subscriber using helper functions
  auto input_publisher = create_input_publisher("test_broadcaster/wrench");
  geometry_msgs::msg::WrenchStamped::SharedPtr received_msg;
  auto output_subscriber = create_output_subscriber(
    "/fts_wrench_transformer/wrench_transformed_base_link",
    [&received_msg](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    { received_msg = msg; });

  wait_for_discovery();

  // Wait for publisher to be discovered by the output subscriber
  wait_for_publisher(output_subscriber);

  // Publish test message with known values
  auto test_msg = create_test_wrench(transformer_node->get_clock());
  input_publisher->publish(test_msg);

  // Wait for message to be processed
  int max_attempts = 20;
  while (max_attempts-- > 0 && !received_msg)
  {
    executor_->spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Verify message was received and transformed
  ASSERT_NE(transformer_node, nullptr);
  ASSERT_TRUE(received_msg != nullptr) << "Transformed message was not received";
  EXPECT_EQ(received_msg->header.frame_id, "base_link");

  // With 90° counterclockwise rotation around z-axis, the transformation is:
  // x' = -y, y' = x, z' = z (z-component unchanged since rotation is around z-axis)
  // Force: (1.0, 2.0, 3.0) -> x'=-2.0, y'=1.0, z'=3.0 -> (-2.0, 1.0, 3.0)
  // Torque: (0.1, 0.2, 0.3) -> x'=-0.2, y'=0.1, z'=0.3 -> (-0.2, 0.1, 0.3)
  EXPECT_NEAR(received_msg->wrench.force.x, -2.0, 1e-5);
  EXPECT_NEAR(received_msg->wrench.force.y, 1.0, 1e-5);
  EXPECT_NEAR(received_msg->wrench.force.z, 3.0, 1e-5);
  EXPECT_NEAR(received_msg->wrench.torque.x, -0.2, 1e-5);
  EXPECT_NEAR(received_msg->wrench.torque.y, 0.1, 1e-5);
  EXPECT_NEAR(received_msg->wrench.torque.z, 0.3, 1e-5);

  // Verify values are different from input (non-identity transform applied)
  EXPECT_NE(received_msg->wrench.force.x, test_msg.wrench.force.x);
  EXPECT_NE(received_msg->wrench.force.y, test_msg.wrench.force.y);
  EXPECT_NE(received_msg->wrench.torque.x, test_msg.wrench.torque.x);
  EXPECT_NE(received_msg->wrench.torque.y, test_msg.wrench.torque.y);
}

TEST_F(TestWrenchTransformer, PublishSubscribeMultipleFrames)
{
  // Create transformer node first so its TF listener can receive transforms
  auto transformer_node =
    create_transformer_node("test_broadcaster", false, {"base_link", "end_effector"});
  executor_->spin_some(std::chrono::milliseconds(100));

  // Setup static transforms
  setup_static_transform("base_link", "sensor_frame", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  setup_static_transform("base_link", "end_effector", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

  // Wait longer for TF buffer to receive the transforms
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  executor_->spin_some(std::chrono::milliseconds(200));

  // Create publisher and subscribers using helper functions
  auto input_publisher = create_input_publisher("test_broadcaster/wrench");
  geometry_msgs::msg::WrenchStamped::SharedPtr received_base_link;
  geometry_msgs::msg::WrenchStamped::SharedPtr received_end_effector;
  auto base_link_subscriber = create_output_subscriber(
    "/fts_wrench_transformer/wrench_transformed_base_link",
    [&received_base_link](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    { received_base_link = msg; });
  auto end_effector_subscriber = create_output_subscriber(
    "/fts_wrench_transformer/wrench_transformed_end_effector",
    [&received_end_effector](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    { received_end_effector = msg; });

  wait_for_discovery();

  // Wait for publishers to be discovered by both subscribers
  wait_for_publisher(base_link_subscriber);
  wait_for_publisher(end_effector_subscriber);

  // Publish test message
  auto test_msg = create_test_wrench(transformer_node->get_clock());
  input_publisher->publish(test_msg);

  // Wait for messages to be processed
  int max_attempts = 20;
  while (max_attempts-- > 0 && (!received_base_link || !received_end_effector))
  {
    executor_->spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Verify messages were received and transformed
  ASSERT_NE(transformer_node, nullptr);
  ASSERT_TRUE(received_base_link != nullptr)
    << "Transformed message for base_link was not received";
  ASSERT_TRUE(received_end_effector != nullptr)
    << "Transformed message for end_effector was not received";
  EXPECT_EQ(received_base_link->header.frame_id, "base_link");
  EXPECT_EQ(received_end_effector->header.frame_id, "end_effector");
}

TEST_F(TestWrenchTransformer, TransformFailureNoPublication)
{
  // Create transformer node with target frame
  auto transformer_node = create_transformer_node("test_broadcaster", false, {"base_link"});
  executor_->spin_some(std::chrono::milliseconds(100));

  // Skip setting up transform - this will cause transform_wrench to return false
  // Wait a bit to ensure TF buffer is ready
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some(std::chrono::milliseconds(100));

  // Create publisher and subscriber using helper functions
  auto input_publisher = create_input_publisher("test_broadcaster/wrench");
  bool message_received = false;
  create_output_subscriber(
    "/fts_wrench_transformer/wrench_transformed_base_link",
    [&message_received](const geometry_msgs::msg::WrenchStamped::SharedPtr)
    { message_received = true; });

  wait_for_discovery();

  // Publish test message with frame_id that has no transform available
  auto test_msg = create_test_wrench(transformer_node->get_clock(), "unknown_frame");
  input_publisher->publish(test_msg);

  // Wait for message processing - should NOT receive any message
  int max_attempts = 10;
  while (max_attempts-- > 0)
  {
    executor_->spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Verify transform_wrench would return false by checking transform doesn't exist
  auto tf_check_node = std::make_shared<rclcpp::Node>("tf_check_node");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(tf_check_node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  executor_->add_node(tf_check_node);
  executor_->spin_some(std::chrono::milliseconds(100));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  bool transform_exists = false;
  try
  {
    tf_buffer->lookupTransform(
      "base_link", "unknown_frame", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
    transform_exists = true;
  }
  catch (const tf2::TransformException &)
  {
    transform_exists = false;
  }

  // Verify no message was received and transform doesn't exist (transform_wrench returned false)
  ASSERT_NE(transformer_node, nullptr);
  EXPECT_FALSE(transform_exists)
    << "Transform should not exist, confirming transform_wrench would return false";
  EXPECT_FALSE(message_received) << "Message should not be published when transform fails";
}

TEST_F(TestWrenchTransformer, InitExceptionHandling)
{
  // Test that init() handles exceptions gracefully when ParamListener throws
  // Use invalid parameter type to trigger exception in ParamListener
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> parameters;
  // Provide wrong type for target_frames (string instead of vector<string>)
  // This should cause ParamListener to throw when getting parameters
  parameters.emplace_back("target_frames", std::string("invalid_type"));

  options.parameter_overrides(parameters);

  auto node = std::make_shared<force_torque_sensor_broadcaster::WrenchTransformer>(options);
  executor_->add_node(node);

  // init() should catch the exception and return early without crashing
  ASSERT_NE(node, nullptr);
  EXPECT_NO_THROW(node->init());
}

TEST_F(TestWrenchTransformer, InvalidMessageHandling)
{
  // Create transformer node
  auto transformer_node = create_transformer_node();
  executor_->spin_some(std::chrono::milliseconds(100));

  // Setup static transform
  setup_static_transform("base_link", "sensor_frame", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  executor_->spin_some(std::chrono::milliseconds(200));

  // Create publisher and subscriber using helper functions
  auto input_publisher = create_input_publisher("test_broadcaster/wrench");
  bool message_received = false;
  create_output_subscriber(
    "/fts_wrench_transformer/wrench_transformed_base_link",
    [&message_received](const geometry_msgs::msg::WrenchStamped::SharedPtr)
    { message_received = true; });

  wait_for_discovery();

  // Test case 1: Publish message with empty frame_id
  auto invalid_msg = create_test_wrench(transformer_node->get_clock(), "");
  input_publisher->publish(invalid_msg);

  // Wait for message processing
  int max_attempts = 10;
  while (max_attempts-- > 0)
  {
    executor_->spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Verify no message was published (callback should return early)
  ASSERT_NE(transformer_node, nullptr);
  EXPECT_FALSE(message_received) << "Message with empty frame_id should not trigger publication";
}

TEST_F(TestWrenchTransformer, FilteredInputTopic)
{
  // Test that transformer subscribes to wrench_filtered topic when use_filtered_input is true
  auto transformer_node = create_transformer_node("test_broadcaster", true, {"base_link"});
  executor_->spin_some(std::chrono::milliseconds(100));

  // Setup static transform
  setup_static_transform("base_link", "sensor_frame", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  executor_->spin_some(std::chrono::milliseconds(200));

  // Create publisher and subscriber using helper functions
  auto input_publisher = create_input_publisher("test_broadcaster/wrench_filtered");
  geometry_msgs::msg::WrenchStamped::SharedPtr received_msg;
  auto output_subscriber = create_output_subscriber(
    "/fts_wrench_transformer/wrench_transformed_base_link",
    [&received_msg](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    { received_msg = msg; });

  wait_for_discovery();

  // Wait for publisher to be discovered by the transformer's subscriber
  wait_for_publisher(output_subscriber);

  // Publish test message to filtered topic
  auto test_msg = create_test_wrench(transformer_node->get_clock());
  input_publisher->publish(test_msg);

  // Wait for message to be processed
  int max_attempts = 20;
  while (max_attempts-- > 0 && !received_msg)
  {
    executor_->spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Verify message was received and transformed from filtered topic
  ASSERT_NE(transformer_node, nullptr);
  ASSERT_TRUE(received_msg != nullptr)
    << "Transformed message from filtered topic was not received";
  EXPECT_EQ(received_msg->header.frame_id, "base_link");
}

TEST_F(TestWrenchTransformer, RunWrenchTransformerFunction)
{
  // Directly test the run_wrench_transformer() function extracted from main()
  // This tests the actual code path that main() executes

  // Shutdown ROS2 first since SetUp already initialized it
  // run_wrench_transformer calls rclcpp::init() internally
  rclcpp::shutdown();

  // Prepare test arguments with parameters (simulating launch with --ros-args)
  // This simulates how the node would be launched: ros2 run ... --ros-args -p
  // target_frames:=[base_link]
  int argc = 4;
  char arg0[] = "test_wrench_transformer";
  char arg1[] = "--ros-args";
  char arg2[] = "-p";
  char arg3[] = "target_frames:=[base_link]";
  char * argv[] = {arg0, arg1, arg2, arg3, nullptr};

  // Test that run_wrench_transformer executes without crashing
  // Since it calls rclcpp::spin() which blocks, run it in a separate thread
  // and verify it initializes correctly
  std::atomic<bool> function_started{false};
  std::thread test_thread(
    [&]()
    {
      function_started = true;
      // This will block at rclcpp::spin(), but initialization should complete first
      force_torque_sensor_broadcaster::run_wrench_transformer(argc, argv);
    });

  // Wait for function to start and initialize (node creation and init() call)
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(function_started);

  // Shutdown ROS2 to unblock the spin in the thread
  // This will cause rclcpp::spin() to return and the function to complete
  rclcpp::shutdown();

  // Wait for thread to complete (run_wrench_transformer should finish after shutdown)
  if (test_thread.joinable())
  {
    test_thread.join();
  }

  // Re-initialize ROS2 for other tests (TearDown will handle final shutdown)
  rclcpp::init(0, nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
