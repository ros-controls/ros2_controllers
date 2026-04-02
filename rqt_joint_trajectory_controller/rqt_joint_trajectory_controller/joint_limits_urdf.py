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

from math import pi

import xml.etree.ElementTree as ET
import rclpy
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot

description = ""


def callback(msg):
    global description
    description = msg.data


def subscribe_to_robot_description(node, key="robot_description"):
    qos_profile = rclpy.qos.QoSProfile(depth=1)
    qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

    node.create_subscription(String, key, callback, qos_profile)


def get_joint_limits(node, joints_names, use_smallest_joint_limits=True):
    use_small = use_smallest_joint_limits
    use_mimic = True

    count = 0
    while description == "" and count < 10:
        print("Waiting for the robot_description!")
        count += 1
        rclpy.spin_once(node, timeout_sec=1.0)

    free_joints = {}
    dependent_joints = {}

    if description != "":
        try:
            # urdf_parser_py does not recognize non-URDF tags such as
            # <ros2_control> and raises an AssertionError.
            # Strip them before parsing.
            root = ET.fromstring(description)
            for tag_name in ["ros2_control"]:
                for element in root.findall(tag_name):
                    root.remove(element)
            cleaned_description = ET.tostring(root, encoding="unicode")
            robot = Robot.from_xml_string(cleaned_description)
        except BaseException as e:
            print(f"Unexpected error: {type(e)}")
            return free_joints

        for joint in robot.joints:
            if joint.type == "fixed":
                continue
            name = joint.name

            if joint.limit is None:
                if name in joints_names:
                    raise Exception(
                        f"Missing limits tag for the joint : {name} in the robot_description!"
                    )
                continue

            minval = joint.limit.lower if joint.limit.lower is not None else 0.0
            maxval = joint.limit.upper if joint.limit.upper is not None else 0.0

            # urdf_parser_py defaults lower/upper to 0.0 when not
            # specified in the URDF, so check for invalid range.
            if minval >= maxval:
                if joint.type == "continuous":
                    minval = -pi
                    maxval = pi
                else:
                    raise Exception(
                        f"Missing lower/upper position limits for the joint : {name}"
                        f" of type : {joint.type} in the robot_description!"
                    )

            if joint.limit.velocity is None:
                raise Exception(
                    f"Missing velocity limits for the joint : {name}"
                    f" of type : {joint.type} in the robot_description!"
                )
            maxvel = joint.limit.velocity

            if use_small and joint.safety_controller is not None:
                if joint.safety_controller.soft_lower_limit is not None:
                    minval = max(minval, joint.safety_controller.soft_lower_limit)
                if joint.safety_controller.soft_upper_limit is not None:
                    maxval = min(maxval, joint.safety_controller.soft_upper_limit)

            if use_mimic and joint.mimic is not None:
                entry = {"parent": joint.mimic.joint}
                if joint.mimic.multiplier is not None:
                    entry["factor"] = joint.mimic.multiplier
                if joint.mimic.offset is not None:
                    entry["offset"] = joint.mimic.offset
                dependent_joints[name] = entry
                continue

            if name in dependent_joints:
                continue

            free_joints[name] = {
                "min_position": minval,
                "max_position": maxval,
                "has_position_limits": joint.type != "continuous",
                "max_velocity": maxvel,
            }

    return free_joints
