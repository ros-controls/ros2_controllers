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

import xml.etree.ElementTree as ET
from urdf_parser_py import urdf
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


def get_joint_limits(node, use_smallest_joint_limits=True):
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

        root = ET.fromstring(description)
        for rc in root.findall("ros2_control"):
            root.remove(rc)

        cleaned_description = ET.tostring(root, encoding="unicode")

        robot = urdf.Robot.from_xml_string(cleaned_description)

        # Find all non-fixed joints
        for joint in robot.joints:

            if joint.type == "fixed":
                continue

            if joint.type == "continuous":
                minval = -pi
                maxval = pi
            else:
                if joint.limit:
                    minval = joint.limit.lower
                    maxval = joint.limit.upper
    
                else:
                    raise Exception(
                        f"Missing lower/upper position limits for the joint : {joint.name} of type : {joint.type} in the robot_description!"
                    )
            try:
                maxvel = float(joint.limit.velocity)
            except ValueError:
                raise Exception(
                    f"Missing velocity limits for the joint : {joint.name} of type : {joint.type} in the robot_description!"
                )

            if use_small and joint.safety_controller:
                if joint.safety_controller.soft_lower_limit:
                    minval = max(
                        minval, float(joint.safety_controller.soft_lower_limit)
                    )
                if joint.safety_controller.soft_upper_limit:
                    maxval = min(
                        maxval, float(joint.safety_controller.soft_upper_limit)
                    )

            if use_mimic and joint.mimic:

                entry = {"parent": joint.mimic.joint}
                if joint.mimic.multiplier:
                    entry["factor"] = float(joint.mimic.multiplier)
                if joint.mimic.offset:
                    entry["offset"] = float(joint.mimic.offset)
                dependent_joints[joint.name] = entry
                continue
            if joint.name in dependent_joints:
                continue
            joint_dic = {"min_position": minval, "max_position": maxval}
            joint_dic["has_position_limits"] = joint.type != "continuous"
            joint_dic["max_velocity"] = maxvel
            free_joints[joint.name] = joint_dic
    return free_joints
