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
import rclpy.subscription
from std_msgs.msg import String

description = ""
robot_description_subscriber_created = False
subscription = None
_robot_description_topic = ""
_spin_count = 0


def callback(msg):
    global description
    description = msg.data


def subscribe_to_robot_description(
    node, key="robot_description"
) -> rclpy.subscription.Subscription:
    global robot_description_subscriber_created, subscription, _robot_description_topic, _spin_count
    qos_profile = rclpy.qos.QoSProfile(depth=1)
    qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

    _robot_description_topic = key
    _spin_count = 0
    subscription = node.create_subscription(String, key, callback, qos_profile)
    robot_description_subscriber_created = True
    return subscription


def unsubscribe_to_robot_description(node) -> rclpy.subscription.Subscription:
    if subscription is not None:
        node.destroy_subscription(subscription)


def get_joint_limits(node, joints_names, use_smallest_joint_limits=True):
    if not robot_description_subscriber_created:
        return {}

    use_small = use_smallest_joint_limits
    use_mimic = True

    global _spin_count

    count = 0
    while description == "" and count < 10:
        count += 1
        _spin_count += 1
        rclpy.spin_once(node, timeout_sec=1.0)
        if _spin_count % 5 == 0:
            node.get_logger().info(
                f'Waiting for robot description on topic "{_robot_description_topic}" ...'
            )

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
                            if name in joints_names:
                                node.get_logger().warn(
                                    f"Joint '{name}' of type '{jtype}' has missing/empty "
                                    f"lower/upper position limits in the robot_description. "
                                    f"Slider will be displayed but disabled."
                                )
                            free_joints[name] = {
                                "min_position": -2 * pi,
                                "max_position": 2 * pi,
                                "has_position_limits": False,
                                "max_velocity": 1.0,
                            }
                            continue
                    try:
                        maxvel = float(limit.getAttribute("velocity"))
                    except ValueError:
                        raise Exception(
                            f"Missing velocity limits for the joint : {name} of type : {jtype} in the robot_description!"
                        )
                except IndexError:
                    if name in joints_names:
                        print(
                            f"Warning: joint '{name}' has no <limit> tag in the "
                            f"robot_description. Slider will be displayed but disabled."
                        )
                        free_joints[name] = {
                            "min_position": -2 * pi,
                            "max_position": 2 * pi,
                            "has_position_limits": False,
                            "max_velocity": 1.0,
                        }
                    continue
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
