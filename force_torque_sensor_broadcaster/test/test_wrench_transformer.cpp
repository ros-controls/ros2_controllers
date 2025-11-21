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
#include <chrono>
#include <filesystem>
#include <fstream>
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
#include "tf2_ros/static_transform_broadcaster.hpp"

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
    tf2_ros::StaticTransformBroadcaster tf_broadcaster(tf_node);

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

  // Create publisher node to send input messages
  auto publisher_node = std::make_shared<rclcpp::Node>("test_publisher");
  auto input_publisher = publisher_node->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "test_broadcaster/wrench", rclcpp::SystemDefaultsQoS());
  executor_->add_node(publisher_node);

  // Create subscriber node to receive output messages
  geometry_msgs::msg::WrenchStamped::SharedPtr received_msg;
  auto subscriber_node = std::make_shared<rclcpp::Node>("test_subscriber");
  auto output_subscriber = subscriber_node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/fts_wrench_transformer/wrench_transformed_base_link", rclcpp::SystemDefaultsQoS(),
    [&received_msg](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    { received_msg = msg; });
  executor_->add_node(subscriber_node);

  wait_for_discovery();

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

  // Create publisher node to send input messages
  auto publisher_node = std::make_shared<rclcpp::Node>("test_publisher");
  auto input_publisher = publisher_node->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "test_broadcaster/wrench", rclcpp::SystemDefaultsQoS());
  executor_->add_node(publisher_node);

  // Create subscriber nodes to receive output messages for both frames
  geometry_msgs::msg::WrenchStamped::SharedPtr received_base_link;
  geometry_msgs::msg::WrenchStamped::SharedPtr received_end_effector;

  auto subscriber_node = std::make_shared<rclcpp::Node>("test_subscriber");
  auto base_link_subscriber =
    subscriber_node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/fts_wrench_transformer/wrench_transformed_base_link", rclcpp::SystemDefaultsQoS(),
      [&received_base_link](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
      { received_base_link = msg; });
  auto end_effector_subscriber =
    subscriber_node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/fts_wrench_transformer/wrench_transformed_end_effector", rclcpp::SystemDefaultsQoS(),
      [&received_end_effector](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
      { received_end_effector = msg; });
  executor_->add_node(subscriber_node);

  wait_for_discovery();

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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
