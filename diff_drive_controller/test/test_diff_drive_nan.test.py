# Copyright 2020 PAL Robotics S.L.
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
import sys
import unittest
import launch_testing.asserts

# Import test description for diffbot located in same directory.
sys.path.append(os.path.dirname(__file__))
from diffbot_launch_test_common import generate_diffbot_test_description


def generate_test_description():
    return generate_diffbot_test_description(
        gtest_name='test_diff_drive_nan',
        additional_params={
            'publish_limited_velocity': True
        },
        timeout=40.0)


class TestGTestProcessActive(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, diffbot_gtest, diffbot_node):
        proc_info.assertWaitForShutdown(diffbot_gtest, timeout=40.0)


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, diffbot_gtest, diffbot_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=diffbot_gtest)
