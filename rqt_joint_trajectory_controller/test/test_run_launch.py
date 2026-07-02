# Copyright (c) 2026 ros-controls Maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import signal
import sys
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node

node_name = "rqt_joint_trajectory_controller_node"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                executable="rqt_joint_trajectory_controller",
                package="rqt_joint_trajectory_controller",
                name=node_name,
                additional_env={"QT_QPA_PLATFORM": "offscreen"},
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestFixture(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = Node("test_node")

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_start(self, proc_output):
        time.sleep(2)
        assert node_name in self.node.get_node_names()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check if the process exited normally."""
        allowable_exit_codes = [0]

        # On Python 3.14+, rqt_gui_py can intermittently abort during SIGINT teardown
        # when rclpy reports an invalid context from the spinner thread.
        # TODO(anyone): remove when https://github.com/ros-visualization/rqt/issues/357 is resolved
        if sys.version_info >= (3, 14):
            allowable_exit_codes.append(-signal.SIGABRT)

        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=allowable_exit_codes
        )
