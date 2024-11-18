# Copyright (c) 2024 AIT - Austrian Institute of Technology GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Christoph Froehlich

import pytest
import unittest
import time

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from launch_testing_ros import WaitForTopics

import launch_testing.markers
import rclpy
import launch_ros.actions
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


# Executes the given launch file and checks if all nodes can be started
@pytest.mark.launch_test
def generate_test_description():

    params = PathJoinSubstitution(
        [
            FindPackageShare("ros2_controllers_test_nodes"),
            "test",
            "rrbot_joint_trajectory_publisher.yaml",
        ]
    )

    pub_node = launch_ros.actions.Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        parameters=[params],
        output="both",
    )

    return LaunchDescription([pub_node, ReadyToTest()])


# This is our test fixture. Each method is a test case.
# These run alongside the processes specified in generate_test_description()
class TestFixture(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = Node("test_node")

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_start(self):
        start = time.time()
        found = False
        while time.time() - start < 2.0 and not found:
            found = "publisher_position_trajectory_controller" in self.node.get_node_names()
            time.sleep(0.1)
        assert found, "publisher_position_trajectory_controller not found!"

    def test_check_if_topic_published(self):
        topic = "/position_trajectory_controller/joint_trajectory"
        wait_for_topics = WaitForTopics([(topic, JointTrajectory)], timeout=20.0)
        assert wait_for_topics.wait(), f"Topic '{topic}' not found!"
        msgs = wait_for_topics.received_messages(topic)
        msg = msgs[0]
        assert len(msg.joint_names) == 2, "Wrong number of joints in message"
        wait_for_topics.shutdown()


@launch_testing.post_shutdown_test()
# These tests are run after the processes in generate_test_description() have shut down.
class TestPublisherShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
