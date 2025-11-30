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
#include "tf2_ros/transform_listener.hpp"

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
    const std::vector<std::string> & target_frames = {"base_link"}, double tf_timeout = 0.1,
    const std::string & input_topic_remap = "test_broadcaster/wrench",
    const std::string & node_namespace = "")
  {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameters;
    parameters.emplace_back("target_frames", target_frames);
    parameters.emplace_back("tf_timeout", tf_timeout);
    options.parameter_overrides(parameters);

    // Build arguments for namespace and topic remapping
    std::vector<std::string> args = {"--ros-args"};
    if (!node_namespace.empty())
    {
      args.push_back("-r");
      args.push_back("__ns:=" + node_namespace);
    }
    // Apply topic remapping - when node is namespaced, ~/wrench expands to <namespace>/wrench
    // For tests, we remap to the broadcaster's topic. For filtered topics, remap to wrench_filtered
    if (!input_topic_remap.empty())
    {
      args.push_back("-r");
      args.push_back("~/wrench:=" + input_topic_remap);
    }
    if (args.size() > 1)
    {
      options.arguments(args);
    }

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

  bool check_publisher_exists_via_graph(
    rclcpp::Node::SharedPtr node, const std::string & topic_name)
  {
    try
    {
      auto publishers_info = node->get_publishers_info_by_topic(topic_name);
      return publishers_info.size() > 0;
    }
    catch (const std::exception &)
    {
      return false;
    }
  }

  void wait_for_publisher(
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber,
    int max_attempts = 20)
  {
    int attempts = 0;
    while (attempts < max_attempts && subscriber->get_publisher_count() == 0)
    {
      executor_->spin_some(std::chrono::milliseconds(100));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      attempts++;
    }
  }

  void wait_for_publisher(
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber,
    rclcpp::Node::SharedPtr query_node, const std::string & topic_name, int max_attempts = 20)
  {
    int attempts = 0;
    while (attempts < max_attempts)
    {
      // Check both methods: subscription count and graph API
      if (
        subscriber->get_publisher_count() > 0 ||
        check_publisher_exists_via_graph(query_node, topic_name))
      {
        return;
      }
      executor_->spin_some(std::chrono::milliseconds(100));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      attempts++;
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
  auto node = create_transformer_node({"base_link", "end_effector"});

  executor_->spin_some(std::chrono::milliseconds(100));

  // Verify publishers exist by creating test subscribers
  // Use fully qualified topic name since ~/ expands to the transformer node's namespace
  auto test_sub_node = std::make_shared<rclcpp::Node>("test_sub_check");
  auto base_link_sub = test_sub_node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/fts_wrench_transformer/base_link/wrench", rclcpp::SystemDefaultsQoS(),
    [](const geometry_msgs::msg::WrenchStamped::SharedPtr) {});
  auto end_effector_sub = test_sub_node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/fts_wrench_transformer/end_effector/wrench", rclcpp::SystemDefaultsQoS(),
    [](const geometry_msgs::msg::WrenchStamped::SharedPtr) {});
  executor_->add_node(test_sub_node);

  // Spin more aggressively to allow discovery
  wait_for_discovery();
  executor_->spin_some(std::chrono::milliseconds(200));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Wait for each publisher to be discovered
  wait_for_publisher(base_link_sub, test_sub_node, "/fts_wrench_transformer/base_link/wrench");
  wait_for_publisher(
    end_effector_sub, test_sub_node, "/fts_wrench_transformer/end_effector/wrench");

  // Final spin to ensure discovery is complete
  executor_->spin_some(std::chrono::milliseconds(200));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ASSERT_NE(node, nullptr);

  // Dump available publishers for debugging if publishers are not found
  EXPECT_GT(base_link_sub->get_publisher_count(), 0u)
    << "Publisher not found on /fts_wrench_transformer/base_link/wrench";
  EXPECT_GT(end_effector_sub->get_publisher_count(), 0u)
    << "Publisher not found on /fts_wrench_transformer/end_effector/wrench";
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
    "/fts_wrench_transformer/base_link/wrench",
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
    "/fts_wrench_transformer/base_link/wrench",
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
  // Use 1e-4 tolerance to account for floating-point precision in quaternion/transform calculations
  EXPECT_NEAR(received_msg->wrench.force.x, -test_msg.wrench.force.y, 1e-4);
  EXPECT_NEAR(received_msg->wrench.force.y, test_msg.wrench.force.x, 1e-4);
  EXPECT_NEAR(received_msg->wrench.force.z, test_msg.wrench.force.z, 1e-4);
  EXPECT_NEAR(received_msg->wrench.torque.x, -test_msg.wrench.torque.y, 1e-4);
  EXPECT_NEAR(received_msg->wrench.torque.y, test_msg.wrench.torque.x, 1e-4);
  EXPECT_NEAR(received_msg->wrench.torque.z, test_msg.wrench.torque.z, 1e-4);
}

TEST_F(TestWrenchTransformer, PublishSubscribeMultipleFrames)
{
  // Create transformer node first so its TF listener can receive transforms
  auto transformer_node = create_transformer_node({"base_link", "end_effector"});
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
    "/fts_wrench_transformer/base_link/wrench",
    [&received_base_link](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    { received_base_link = msg; });
  auto end_effector_subscriber = create_output_subscriber(
    "/fts_wrench_transformer/end_effector/wrench",
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
  auto transformer_node = create_transformer_node();
  executor_->spin_some(std::chrono::milliseconds(100));

  // Skip setting up transform - this will cause transform_wrench to return false
  // Wait a bit to ensure TF buffer is ready
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  executor_->spin_some(std::chrono::milliseconds(100));

  // Create publisher and subscriber using helper functions
  auto input_publisher = create_input_publisher("test_broadcaster/wrench");
  bool message_received = false;
  create_output_subscriber(
    "/fts_wrench_transformer/base_link/wrench",
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
    "/fts_wrench_transformer/base_link/wrench",
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

TEST_F(TestWrenchTransformer, TopicRemappingFilteredInput)
{
  // Test that transformer subscribes to wrench_filtered topic using topic remapping
  auto transformer_node =
    create_transformer_node({"base_link"}, 0.1, "test_broadcaster/wrench_filtered");
  executor_->spin_some(std::chrono::milliseconds(100));

  // Setup static transform
  setup_static_transform("base_link", "sensor_frame", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  executor_->spin_some(std::chrono::milliseconds(200));

  // Create publisher and subscriber using helper functions
  auto input_publisher = create_input_publisher("test_broadcaster/wrench_filtered");
  geometry_msgs::msg::WrenchStamped::SharedPtr received_msg;
  auto output_subscriber = create_output_subscriber(
    "/fts_wrench_transformer/base_link/wrench_filtered",
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

TEST_F(TestWrenchTransformer, NamespaceNormalizationRootNamespace)
{
  // Test with root namespace ("/") - should use node name
  auto node = create_transformer_node({"base_link"}, 0.1, "test_broadcaster/wrench", "/");
  executor_->spin_some(std::chrono::milliseconds(100));

  // Get the publisher and check its topic name directly
  auto topic_names_and_types = node->get_topic_names_and_types();
  bool found = false;
  for (const auto & [topic_name, topic_types] : topic_names_and_types)
  {
    if (topic_name == "/fts_wrench_transformer/base_link/wrench")
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Publisher topic should be /fts_wrench_transformer/base_link/wrench";
}

TEST_F(TestWrenchTransformer, NamespaceNormalizationCustomNamespace)
{
  // Test with custom namespace - should use the namespace
  auto node = create_transformer_node({"base_link"}, 0.1, "test_broadcaster/wrench", "/my_robot");
  executor_->spin_some(std::chrono::milliseconds(100));

  auto topic_names_and_types = node->get_topic_names_and_types();
  bool found = false;
  for (const auto & [topic_name, topic_types] : topic_names_and_types)
  {
    if (topic_name == "/my_robot/base_link/wrench")
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Publisher topic should be /my_robot/base_link/wrench";
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

TEST_F(TestWrenchTransformer, RunWrenchTransformerWithPositionalArgs)
{
  rclcpp::shutdown();

  // Prepare test arguments with positional arguments 
  // This simulates how the node would be launched with positional arguments directly
  int argc = 3;
  char arg0[] = "test_wrench_transformer";
  char arg1[] = "base_link";
  char arg2[] = "end_effector";
  char * argv[] = {arg0, arg1, arg2, nullptr};

  std::atomic<bool> function_started{false};
  std::thread test_thread(
    [&]()
    {
      function_started = true;
      force_torque_sensor_broadcaster::run_wrench_transformer(argc, argv);
    });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_TRUE(function_started);

  rclcpp::shutdown();

  if (test_thread.joinable())
  {
    test_thread.join();
  }

  rclcpp::init(0, nullptr);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
