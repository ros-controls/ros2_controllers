#!/usr/bin/env python

# Copyright 2022 PAL Robotics S.L.
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

# Code inspired on the joint_state_publisher package by David Lu!!!
# https://github.com/ros/robot_model/blob/indigo-devel/
# joint_state_publisher/joint_state_publisher/joint_state_publisher

import xml.etree.ElementTree as ET
from math import pi

import rclpy
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot

description = ""


# Tags defined as direct children of <robot> in the URDF specification.
# Any other tag is a vendor extension (ros2_control, gazebo, xacro
# remnants, etc.) that urdf_parser_py rejects with an AssertionError
# during strict validation. We whitelist the standard tags and strip
# everything else before parsing.
_URDF_STANDARD_TAGS = frozenset({"link", "joint", "transmission", "material"})


def callback(msg):
    global description
    description = msg.data


def subscribe_to_robot_description(node, key="robot_description"):
    qos_profile = rclpy.qos.QoSProfile(depth=1)
    qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

    node.create_subscription(String, key, callback, qos_profile)


def _strip_non_urdf_tags(urdf_string):
    """
    Remove non-URDF extension tags from a robot description string.

    Robot descriptions published to /robot_description commonly carry
    vendor-specific extensions like <ros2_control> and <gazebo> alongside
    the standard URDF elements. urdf_parser_py validates strictly and
    raises AssertionError on any unknown tag, so we keep only the tags
    defined by the URDF specification and drop the rest.
    """
    root = ET.fromstring(urdf_string)
    # Iterate over a list copy because we are mutating root during the loop
    for child in list(root):
        if child.tag not in _URDF_STANDARD_TAGS:
            root.remove(child)
    return ET.tostring(root, encoding="unicode")


def parse_joint_limits(urdf_string, joints_names, use_smallest_joint_limits=True):
    """
    Parse joint position and velocity limits from a URDF XML string.

    This function contains all the real parsing logic and has no dependency
    on ROS, global variables, or any running infrastructure. It accepts the
    URDF as a plain Python string, which makes it directly unit-testable
    without needing a ROS node or a robot_description topic.

    Parameters
    ----------
    urdf_string : str
        A complete URDF XML document as a string.
    joints_names : list[str]
        The joints that the active controller manages. A joint in this list
        that is missing its <limit> element will raise an Exception. Joints
        NOT in this list that are missing limits are silently skipped.
    use_smallest_joint_limits : bool
        When True, safety_controller soft limits narrow the reported range.

    Returns
    -------
    dict
        Maps joint name to a dict with keys:
        min_position, max_position, has_position_limits, max_velocity.

    """
    use_small = use_smallest_joint_limits
    use_mimic = True

    free_joints = {}
    dependent_joints = {}

    # Strip vendor extensions before strict urdf_parser_py validation.
    # Parsing errors on the cleaned URDF propagate to the caller, which
    # matches the old minidom-based behavior on malformed input.
    cleaned = _strip_non_urdf_tags(urdf_string)
    robot = Robot.from_xml_string(cleaned)

    for joint in robot.joints:
        if joint.type == "fixed":
            # Fixed joints have no DOF, so no slider is needed in the GUI
            continue

        name = joint.name

        # No <limit> element at all means urdf_parser_py sets joint.limit to None
        if joint.limit is None:
            if name in joints_names:
                raise Exception(
                    f"Missing limits tag for the joint : {name} in the robot_description!"
                )
            # Joint is not managed by this controller, so we skip silently
            continue

        # urdf_parser_py defaults lower/upper to 0.0 when the attributes are
        # absent in the URDF. We treat min >= max as "limits missing" and
        # either fall back to -pi..pi for continuous joints or raise for
        # revolute joints where valid bounds are required.
        minval = joint.limit.lower if joint.limit.lower is not None else 0.0
        maxval = joint.limit.upper if joint.limit.upper is not None else 0.0

        if minval >= maxval:
            if joint.type == "continuous":
                minval = -pi
                maxval = pi
            else:
                raise Exception(
                    f"Missing lower/upper position limits for the joint"
                    f" : {name} of type : {joint.type} in the robot_description!"
                )

        if joint.limit.velocity is None:
            raise Exception(
                f"Missing velocity limits for the joint"
                f" : {name} of type : {joint.type} in the robot_description!"
            )
        maxvel = joint.limit.velocity

        # Optionally narrow the range with safety_controller soft limits
        if use_small and joint.safety_controller is not None:
            if joint.safety_controller.soft_lower_limit is not None:
                minval = max(minval, joint.safety_controller.soft_lower_limit)
            if joint.safety_controller.soft_upper_limit is not None:
                maxval = min(maxval, joint.safety_controller.soft_upper_limit)

        # Mimic joints follow another joint and go into dependent_joints
        if use_mimic and joint.mimic is not None:
            entry = {"parent": joint.mimic.joint}
            if joint.mimic.multiplier is not None:
                entry["factor"] = joint.mimic.multiplier
            if joint.mimic.offset is not None:
                entry["offset"] = joint.mimic.offset
            dependent_joints[name] = entry
            continue

        free_joints[name] = {
            "min_position": minval,
            "max_position": maxval,
            "has_position_limits": joint.type != "continuous",
            "max_velocity": maxvel,
        }

    return free_joints


def get_joint_limits(node, joints_names, use_smallest_joint_limits=True):
    """
    ROS-aware wrapper around parse_joint_limits.

    Waits for the robot_description topic to publish, then delegates all
    real parsing work to parse_joint_limits(). This separation means
    parse_joint_limits() can be tested without any ROS infrastructure.
    """
    count = 0
    while description == "" and count < 10:
        print("Waiting for the robot_description!")
        count += 1
        rclpy.spin_once(node, timeout_sec=1.0)

    if description == "":
        return {}

    return parse_joint_limits(description, joints_names, use_smallest_joint_limits)
