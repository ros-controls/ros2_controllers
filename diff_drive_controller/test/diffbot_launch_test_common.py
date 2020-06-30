# Copyright 2020 Open Source Robotics Foundation, Inc.
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
import os

from typing import Optional, SupportsFloat

import launch
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros
from launch_ros.parameters_type import SomeParametersDict
import launch_testing.actions
import xacro


def generate_diffbot_test_description(*args,
                                      gtest_name: SomeSubstitutionsType,
                                      exec_name: SomeSubstitutionsType = 'diffbot',
                                      param_yaml: SomeSubstitutionsType = 'diffbot.yaml',
                                      robot_urdf: SomeSubstitutionsType = 'diffbot.xacro',
                                      additional_params: Optional[SomeParametersDict] = None,
                                      timeout: SupportsFloat = 40.0):
    if additional_params is None:
        additional_params = {}

    # Parse xacro file
    xacro_file = os.path.join(os.path.dirname(__file__), 'config/', robot_urdf)
    robot_description = xacro.process(xacro_file)

    diffbot_node = launch_ros.actions.Node(
        node_executable=PathJoinSubstitution([LaunchConfiguration('test_binary_dir'), exec_name]),
        parameters=[
            PathJoinSubstitution([os.path.dirname(__file__), 'config/', param_yaml]),
            {'robot_description': robot_description},
            additional_params
        ],
        remappings=[('cmd_vel', 'diffbot_controller/cmd_vel'),
                    ('odom', 'diffbot_controller/odom'),
                    ('cmd_vel_out', 'diffbot_controller/cmd_vel_out'),
                    ('wheel_joint_controller_state',
                     'diffbot_controller/wheel_joint_controller_state')]
    )

    diffbot_gtest = launch_testing.actions.GTest(
        path=PathJoinSubstitution([LaunchConfiguration('test_binary_dir'), gtest_name]),
        timeout=timeout, output='screen')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='test_binary_dir',
                                             description='Binary directory of package '
                                                         'containing test executables'),
        diffbot_node,
        diffbot_gtest,
        launch_testing.actions.ReadyToTest()
    ]), {'diffbot_node': diffbot_node, 'diffbot_gtest': diffbot_gtest}
