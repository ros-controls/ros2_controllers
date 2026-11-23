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


class _FakeCombo:
    """Minimal QComboBox stand-in: a list of items and a selected index."""

    def __init__(self, items=()):
        self.items = list(items)
        self.index = 0 if self.items else -1

    def count(self):
        return len(self.items)

    def itemText(self, i):
        return self.items[i]

    def findText(self, text):
        return self.items.index(text) if text in self.items else -1

    def setCurrentIndex(self, i):
        self.index = i

    def currentText(self):
        return self.items[self.index] if self.index >= 0 else ""


def test_restore_settings_selects_saved_jtc_once_controller_list_arrives(monkeypatch):
    def fake_update_combo(combo, names):
        combo.items = list(names)

    monkeypatch.setattr(jtc_module, "update_combo", fake_update_combo)
    jtc = SimpleNamespace(
        name="arm_controller",
        type="joint_trajectory_controller/JointTrajectoryController",
        state="active",
        required_state_interfaces=["joint1/position"],
    )
    responses = iter([None, [jtc]])  # first call: no response received yet
    plugin = SimpleNamespace(
        _list_controllers=lambda: next(responses),
        _widget=SimpleNamespace(
            cm_combo=_FakeCombo(["/controller_manager"]), jtc_combo=_FakeCombo()
        ),
        _robot_joint_limits={"joint1": {}},
        _running_jtc=[],
        _pending_jtc_name=None,
        _update_cm_list=lambda: None,
    )
    plugin._running_jtc_info = lambda: JointTrajectoryController._running_jtc_info(plugin)
    plugin._update_jtc_list = lambda: JointTrajectoryController._update_jtc_list(plugin)
    settings = SimpleNamespace(
        value={"cm_ns": "/controller_manager", "jtc_name": "arm_controller"}.get
    )

    JointTrajectoryController.restore_settings(plugin, None, settings)
    assert plugin._widget.jtc_combo.currentText() == ""

    plugin._update_jtc_list()  # next timer tick, response has arrived

    assert plugin._widget.jtc_combo.currentText() == "arm_controller"


def test_restore_settings_selects_saved_jtc_once_robot_description_arrives(monkeypatch):
    def fake_update_combo(combo, names):
        combo.items = list(names)

    joint_limits = iter([{}, {"joint1": {}}])  # first call: no robot_description received yet
    monkeypatch.setattr(jtc_module, "update_combo", fake_update_combo)
    monkeypatch.setattr(jtc_module, "get_joint_limits", lambda *args: next(joint_limits))
    jtc = SimpleNamespace(
        name="arm_controller",
        type="joint_trajectory_controller/JointTrajectoryController",
        state="active",
        required_state_interfaces=["joint1/position"],
    )
    plugin = SimpleNamespace(
        _list_controllers=lambda: [jtc],
        _widget=SimpleNamespace(
            cm_combo=_FakeCombo(["/controller_manager"]), jtc_combo=_FakeCombo()
        ),
        _node=None,
        _robot_joint_limits={},
        _running_jtc=[],
        _pending_jtc_name=None,
        _update_cm_list=lambda: None,
    )
    plugin._running_jtc_info = lambda: JointTrajectoryController._running_jtc_info(plugin)
    plugin._update_jtc_list = lambda: JointTrajectoryController._update_jtc_list(plugin)
    settings = SimpleNamespace(
        value={"cm_ns": "/controller_manager", "jtc_name": "arm_controller"}.get
    )

    JointTrajectoryController.restore_settings(plugin, None, settings)
    assert plugin._widget.jtc_combo.currentText() == ""

    plugin._update_jtc_list()  # next timer tick, robot_description has arrived

    assert plugin._widget.jtc_combo.currentText() == "arm_controller"


def test_update_cmd_cb_sends_nothing_before_first_controller_state(monkeypatch):
    monkeypatch.setattr(jtc_module.rclpy, "ok", lambda: True)
    plugin = SimpleNamespace(
        _cmd_pub=Mock(),
        _joint_names=["joint1"],
        _joint_pos={"joint1": {"command": 0.3}},  # slider moved, no state received yet
        _robot_joint_limits={"joint1": {"max_velocity": 1.0}},
        _min_traj_dur=0.5,
        _speed_scale=0.5,
    )

    JointTrajectoryController._update_cmd_cb(plugin)

    plugin._cmd_pub.publish.assert_not_called()
