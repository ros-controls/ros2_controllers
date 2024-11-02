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

# TODO: Use urdf_parser_py.urdf instead. I gave it a try, but got
#  Exception: Required attribute not set in XML: upper
# upper is an optional attribute, so I don't understand what's going on
# See comments in https://github.com/ros/urdfdom/issues/36

import xml.dom.minidom
from math import pi

import rclpy
from std_msgs.msg import String

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
    global description
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
        robot = xml.dom.minidom.parseString(description).getElementsByTagName("robot")[0]

        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == "joint":
                jtype = child.getAttribute("type")
                if jtype == "fixed":
                    continue
                name = child.getAttribute("name")

                if name in joints_names:
                    try:
                        limit = child.getElementsByTagName("limit")[0]
                        try:
                            minval = float(limit.getAttribute("lower"))
                            maxval = float(limit.getAttribute("upper"))
                        except ValueError:
                            if jtype == "continuous":
                                minval = -pi
                                maxval = pi
                            else:
                                raise Exception(
                                    f"Missing lower/upper position limits for the joint : {name} of type : {jtype} in the robot_description!"
                                )
                        try:
                            maxvel = float(limit.getAttribute("velocity"))
                        except ValueError:
                            raise Exception(
                                f"Missing velocity limits for the joint : {name} of type : {jtype} in the robot_description!"
                            )
                    except IndexError:
                        raise Exception(
                            f"Missing limits tag for the joint : {name} in the robot_description!"
                        )
                    safety_tags = child.getElementsByTagName("safety_controller")
                    if use_small and len(safety_tags) == 1:
                        tag = safety_tags[0]
                        if tag.hasAttribute("soft_lower_limit"):
                            minval = max(minval, float(tag.getAttribute("soft_lower_limit")))
                        if tag.hasAttribute("soft_upper_limit"):
                            maxval = min(maxval, float(tag.getAttribute("soft_upper_limit")))

                    mimic_tags = child.getElementsByTagName("mimic")
                    if use_mimic and len(mimic_tags) == 1:
                        tag = mimic_tags[0]
                        entry = {"parent": tag.getAttribute("joint")}
                        if tag.hasAttribute("multiplier"):
                            entry["factor"] = float(tag.getAttribute("multiplier"))
                        if tag.hasAttribute("offset"):
                            entry["offset"] = float(tag.getAttribute("offset"))

                        dependent_joints[name] = entry
                        continue

                    if name in dependent_joints:
                        continue

                    joint = {"min_position": minval, "max_position": maxval}
                    joint["has_position_limits"] = jtype != "continuous"
                    joint["max_velocity"] = maxvel
                    free_joints[name] = joint
    return free_joints
