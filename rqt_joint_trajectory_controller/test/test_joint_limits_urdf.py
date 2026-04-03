# Copyright 2024 ros2_control Development Team
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

"""Unit tests for parse_joint_limits() in joint_limits_urdf.py.

All tests are ROS-free. They call parse_joint_limits() directly with
URDF strings and assert on the returned dict. No node, no topic, no
robot_description publisher is needed.

Run with:
    pytest rqt_joint_trajectory_controller/test/test_joint_limits_urdf.py -v
"""

import math
import pytest

from rqt_joint_trajectory_controller.joint_limits_urdf import parse_joint_limits

# ---------------------------------------------------------------------------
# Small URDF builder helpers
#
# Real robot URDFs are hundreds of lines long, but for testing we only need
# the minimum valid XML that exercises one specific behaviour. These helpers
# let each test build exactly what it needs in two or three lines.
# ---------------------------------------------------------------------------


def _robot(*joint_snippets):
    """Wrap joint snippets inside a minimal valid <robot> element."""
    body = "\n".join(joint_snippets)
    return f'<?xml version="1.0"?><robot name="r"><link name="base"/>{body}</robot>'


def _revolute(name, lower, upper, velocity):
    """A revolute joint with explicit position and velocity limits."""
    return (
        f'<link name="{name}_link"/>'
        f'<joint name="{name}" type="revolute">'
        f'<parent link="base"/><child link="{name}_link"/>'
        f'<limit lower="{lower}" upper="{upper}" velocity="{velocity}" effort="10"/>'
        f"</joint>"
    )


def _continuous(name, velocity):
    """A continuous joint — no lower/upper attributes."""
    return (
        f'<link name="{name}_link"/>'
        f'<joint name="{name}" type="continuous">'
        f'<parent link="base"/><child link="{name}_link"/>'
        f'<limit velocity="{velocity}" effort="10"/>'
        f"</joint>"
    )


def _fixed(name):
    """A fixed joint — no limits element at all."""
    return (
        f'<link name="{name}_link"/>'
        f'<joint name="{name}" type="fixed">'
        f'<parent link="base"/><child link="{name}_link"/>'
        f"</joint>"
    )


# ---------------------------------------------------------------------------
# Group 1: Revolute joint — the most common joint type in a robot arm.
# The function must return exactly the values written in the URDF.
# ---------------------------------------------------------------------------


def test_revolute_joint_appears_in_result():
    result = parse_joint_limits(_robot(_revolute("j1", -1.5, 1.5, 2.0)), ["j1"])
    assert "j1" in result


def test_revolute_joint_min_position():
    result = parse_joint_limits(_robot(_revolute("j1", -1.5, 1.5, 2.0)), ["j1"])
    assert result["j1"]["min_position"] == pytest.approx(-1.5)


def test_revolute_joint_max_position():
    result = parse_joint_limits(_robot(_revolute("j1", -1.5, 1.5, 2.0)), ["j1"])
    assert result["j1"]["max_position"] == pytest.approx(1.5)


def test_revolute_joint_max_velocity():
    result = parse_joint_limits(_robot(_revolute("j1", -1.5, 1.5, 2.0)), ["j1"])
    assert result["j1"]["max_velocity"] == pytest.approx(2.0)


def test_revolute_joint_has_position_limits_true():
    # Revolute joints are bounded — the GUI slider should enforce limits
    result = parse_joint_limits(_robot(_revolute("j1", -1.5, 1.5, 2.0)), ["j1"])
    assert result["j1"]["has_position_limits"] is True


# ---------------------------------------------------------------------------
# Group 2: Continuous joint — like a wheel, no position bounds.
# When lower/upper are absent, minidom returns "", float("") raises
# ValueError, and our code must default to -pi / +pi so the slider
# has a usable range.
# ---------------------------------------------------------------------------


def test_continuous_joint_appears_in_result():
    result = parse_joint_limits(_robot(_continuous("wheel", 5.0)), ["wheel"])
    assert "wheel" in result


def test_continuous_joint_min_defaults_to_minus_pi():
    result = parse_joint_limits(_robot(_continuous("wheel", 5.0)), ["wheel"])
    assert result["wheel"]["min_position"] == pytest.approx(-math.pi)


def test_continuous_joint_max_defaults_to_plus_pi():
    result = parse_joint_limits(_robot(_continuous("wheel", 5.0)), ["wheel"])
    assert result["wheel"]["max_position"] == pytest.approx(math.pi)


def test_continuous_joint_has_position_limits_false():
    # Continuous joints are unbounded — the GUI should not enforce limits
    result = parse_joint_limits(_robot(_continuous("wheel", 5.0)), ["wheel"])
    assert result["wheel"]["has_position_limits"] is False


def test_continuous_joint_velocity_is_preserved():
    result = parse_joint_limits(_robot(_continuous("wheel", 5.0)), ["wheel"])
    assert result["wheel"]["max_velocity"] == pytest.approx(5.0)


# ---------------------------------------------------------------------------
# Group 3: Fixed joints — they have no DOF and must be completely ignored.
# ---------------------------------------------------------------------------


def test_fixed_joint_not_in_result():
    result = parse_joint_limits(_robot(_fixed("camera_mount")), [])
    assert "camera_mount" not in result


def test_urdf_with_only_fixed_joints_returns_empty_dict():
    result = parse_joint_limits(_robot(_fixed("j1"), _fixed("j2")), [])
    assert result == {}


# ---------------------------------------------------------------------------
# Group 4: Multiple joints — all non-fixed joints must appear in the result.
# This mirrors a real robot arm with mixed joint types.
# ---------------------------------------------------------------------------


def test_multiple_joints_all_present():
    urdf = _robot(
        _revolute("shoulder", -1.0, 1.0, 1.0),
        _revolute("elbow", -2.0, 2.0, 1.5),
        _continuous("wrist_roll", 3.0),
        _fixed("tool_mount"),
    )
    result = parse_joint_limits(urdf, ["shoulder", "elbow", "wrist_roll"])
    assert set(result.keys()) == {"shoulder", "elbow", "wrist_roll"}


def test_multiple_joints_fixed_excluded():
    urdf = _robot(
        _revolute("shoulder", -1.0, 1.0, 1.0),
        _fixed("tool_mount"),
    )
    result = parse_joint_limits(urdf, [])
    assert "tool_mount" not in result


def test_multiple_joints_individual_limits_correct():
    urdf = _robot(
        _revolute("shoulder", -1.0, 1.0, 1.0),
        _revolute("elbow", -2.0, 2.0, 1.5),
    )
    result = parse_joint_limits(urdf, ["shoulder", "elbow"])
    assert result["elbow"]["min_position"] == pytest.approx(-2.0)
    assert result["shoulder"]["max_velocity"] == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# Group 5: Safety controller soft limits.
# When use_smallest_joint_limits=True, soft limits should narrow the range.
# When False, only the hard limits from <limit> should be used.
# This is entirely our application logic — minidom knows nothing about it.
# ---------------------------------------------------------------------------


def test_soft_limits_narrow_range_when_flag_true():
    urdf = _robot(
        '<link name="j1_link"/>'
        '<joint name="j1" type="revolute">'
        '<parent link="base"/><child link="j1_link"/>'
        '<limit lower="-2.0" upper="2.0" velocity="1.0" effort="10"/>'
        '<safety_controller soft_lower_limit="-1.0" soft_upper_limit="1.0"'
        ' k_position="100" k_velocity="10"/>'
        "</joint>"
    )
    result = parse_joint_limits(urdf, ["j1"], use_smallest_joint_limits=True)
    assert result["j1"]["min_position"] == pytest.approx(-1.0)
    assert result["j1"]["max_position"] == pytest.approx(1.0)


def test_soft_limits_ignored_when_flag_false():
    urdf = _robot(
        '<link name="j1_link"/>'
        '<joint name="j1" type="revolute">'
        '<parent link="base"/><child link="j1_link"/>'
        '<limit lower="-2.0" upper="2.0" velocity="1.0" effort="10"/>'
        '<safety_controller soft_lower_limit="-1.0" soft_upper_limit="1.0"'
        ' k_position="100" k_velocity="10"/>'
        "</joint>"
    )
    result = parse_joint_limits(urdf, ["j1"], use_smallest_joint_limits=False)
    assert result["j1"]["min_position"] == pytest.approx(-2.0)
    assert result["j1"]["max_position"] == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# Group 6: Mimic joints — follow another joint mechanically.
# They cannot be independently commanded, so the GUI must not show a
# slider for them. They must be absent from free_joints.
# ---------------------------------------------------------------------------


def test_mimic_joint_excluded_from_result():
    urdf = _robot(
        _revolute("driver", -1.0, 1.0, 1.0),
        '<link name="follower_link"/>'
        '<joint name="follower" type="revolute">'
        '<parent link="base"/><child link="follower_link"/>'
        '<limit lower="-1.0" upper="1.0" velocity="1.0" effort="5"/>'
        '<mimic joint="driver" multiplier="1.0" offset="0.0"/>'
        "</joint>",
    )
    result = parse_joint_limits(urdf, ["driver"])
    assert "follower" not in result


def test_driver_joint_present_when_follower_is_mimic():
    urdf = _robot(
        _revolute("driver", -1.0, 1.0, 1.0),
        '<link name="follower_link"/>'
        '<joint name="follower" type="revolute">'
        '<parent link="base"/><child link="follower_link"/>'
        '<limit lower="-1.0" upper="1.0" velocity="1.0" effort="5"/>'
        '<mimic joint="driver" multiplier="1.0" offset="0.0"/>'
        "</joint>",
    )
    result = parse_joint_limits(urdf, ["driver"])
    assert "driver" in result


# ---------------------------------------------------------------------------
# Group 7: Error cases — our application logic, not minidom's.
# minidom parses all of these successfully and returns data.
# Our code is the one that decides they are errors.
# ---------------------------------------------------------------------------


def test_missing_limit_tag_for_required_joint_raises():
    """Joint in joints_names with no <limit> element at all must raise.

    minidom parses this fine — joint.getElementsByTagName("limit") just
    returns an empty list, and [0] raises IndexError. Our except block
    is what turns that into a meaningful exception message.
    """
    urdf = _robot(
        '<link name="j_link"/>'
        '<joint name="j" type="revolute">'
        '<parent link="base"/><child link="j_link"/>'
        "</joint>"
    )
    with pytest.raises(Exception, match="Missing limits tag"):
        parse_joint_limits(urdf, ["j"])


def test_missing_limit_tag_for_unrequired_joint_skipped_silently():
    """Joint NOT in joints_names with no <limit> is silently ignored.

    The URDF may describe joints that this controller does not manage.
    Those joints do not need limits.
    """
    urdf = _robot(
        '<link name="j_link"/>'
        '<joint name="j" type="revolute">'
        '<parent link="base"/><child link="j_link"/>'
        "</joint>"
    )
    result = parse_joint_limits(urdf, [])
    assert "j" not in result


def test_revolute_joint_missing_lower_upper_raises():
    """Revolute joint with no lower/upper attributes raises.

    minidom returns "" for absent attributes. float("") raises ValueError.
    Our except block turns that into a meaningful message for non-continuous
    joints. This is our own logic — worth testing.
    """
    urdf = _robot(
        '<link name="j_link"/>'
        '<joint name="j" type="revolute">'
        '<parent link="base"/><child link="j_link"/>'
        '<limit velocity="1.0" effort="5"/>'
        "</joint>"
    )
    with pytest.raises(Exception, match="Missing lower/upper position limits"):
        parse_joint_limits(urdf, ["j"])


def test_missing_velocity_raises():
    """Joint with no velocity attribute raises.

    minidom returns "" for absent velocity. float("") raises ValueError.
    Our except block turns that into a meaningful message. Our own logic.
    """
    urdf = _robot(
        '<link name="j_link"/>'
        '<joint name="j" type="revolute">'
        '<parent link="base"/><child link="j_link"/>'
        '<limit lower="-1.0" upper="1.0" effort="5"/>'
        "</joint>"
    )
    with pytest.raises(Exception, match="Missing velocity limits"):
        parse_joint_limits(urdf, ["j"])
