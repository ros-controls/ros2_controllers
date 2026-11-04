# Copyright 2026 ros2_control Development Team
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

"""Unit tests for how the plugin handles unknown or outdated controller lists."""

from types import SimpleNamespace
from unittest.mock import Mock

from rqt_joint_trajectory_controller import joint_trajectory_controller as jtc_module

JointTrajectoryController = jtc_module.JointTrajectoryController


def test_update_jtc_list_keeps_selection_if_controller_list_is_unknown(monkeypatch):
    update_combo = Mock()
    monkeypatch.setattr(jtc_module, "update_combo", update_combo)
    plugin = SimpleNamespace(
        _list_controllers=Mock(return_value=None),
        _widget=SimpleNamespace(jtc_combo=Mock()),
        _robot_joint_limits={},
        _running_jtc=[],
    )
    plugin._running_jtc_info = lambda: JointTrajectoryController._running_jtc_info(plugin)

    JointTrajectoryController._update_jtc_list(plugin)

    update_combo.assert_not_called()
    plugin._widget.jtc_combo.clear.assert_not_called()


def test_load_jtc_ignores_controller_that_is_not_running_anymore():
    plugin = SimpleNamespace(_running_jtc=[], _jtc_name="gone_controller", _joint_names=[])

    JointTrajectoryController._load_jtc(plugin)

    assert plugin._joint_names == []
