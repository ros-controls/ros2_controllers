#!/usr/bin/env python3

# Copyright (c) 2025, b»robotized
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
#
# Authors: Mathias Fuhrer

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseArray
from control_msgs.msg import MotionPrimitiveSequence
from sensor_msgs.msg import JointState

import csv
from datetime import datetime
import threading
import os
import sys
import time

# Constants for motion primitive types --> defined in control_msg and moprim_controller
# Would be better to import these from the actual message definition
PRIMITIVE_TYPE_SEQUENCE_START = 100
PRIMITIVE_TYPE_SEQUENCE_END = 101
PRIMITIVE_TYPE_LINEAR_JOINT = 0
PRIMITIVE_TYPE_LINEAR_CARTESIAN = 50

data_dir = "src/ros2_controllers/motion_primitives_forward_controller/data"


class MotionPrimitiveCollector(Node):
    def __init__(self):
        super().__init__("motion_primitive_collector")

        self.trajectory_msg = None
        self.poses_msg = None
        self.motion_primitives_msg = None
        self.executed_joint_states = []
        self.recording_joint_states = False

        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/motion_primitive_from_trajectory_controller/planned_trajectory",
            self.trajectory_callback,
            1,
        )
        self.poses_sub = self.create_subscription(
            PoseArray,
            "/motion_primitive_from_trajectory_controller/planned_poses",
            self.poses_callback,
            1,
        )
        self.motion_primitive_sub = self.create_subscription(
            MotionPrimitiveSequence,
            "/motion_primitive_from_trajectory_controller/approximated_motion_primitives",
            self.motion_primitive_callback,
            1,
        )
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        self.get_logger().info("Waiting for trajectory, poses, and motion primitives...")

    def trajectory_callback(self, msg):
        if self.trajectory_msg is None:
            self.trajectory_msg = msg
            self.get_logger().info("Received planned_trajectory.")
            self.recording_joint_states = True
            self.get_logger().info("Recording of /joint_states started. Press ENTER to stop.")
            threading.Thread(target=self._wait_for_enter_and_stop_recording, daemon=True).start()

    def _wait_for_enter_and_stop_recording(self):
        try:
            print("Press ENTER to stop recording or wait 60 seconds...")
            sys.stdin.readline()
        except (EOFError, OSError):
            self.get_logger().warn("No stdin available. Using 60s timeout.")
            time.sleep(60)
        finally:
            self.recording_joint_states = False
            self.get_logger().info("Stopped recording joint_states.")
            self.save_executed_joint_states()
            self.check_and_export_all()

    def poses_callback(self, msg):
        if self.poses_msg is None:
            self.poses_msg = msg
            self.get_logger().info("Received planned_poses.")

    def motion_primitive_callback(self, msg):
        if self.motion_primitives_msg is None:
            self.motion_primitives_msg = msg
            self.get_logger().info("Received motion primitives.")
            self.check_and_export_motion_primitives()

    def joint_states_callback(self, msg):
        if self.recording_joint_states:
            t = self.get_clock().now().seconds_nanoseconds()
            self.executed_joint_states.append((t, msg))

    def check_and_export_motion_primitives(self):
        sequence = self.motion_primitives_msg.motions
        if not sequence:
            self.get_logger().error("Motion primitive sequence is empty.")
            return

        types = {p.type for p in sequence}
        if len(types) > 1:
            self.get_logger().error(
                f"Mixed motion primitive types found: {types}. Only one type allowed."
            )
            rclpy.shutdown()
            return

        primitive_type = sequence[0].type
        if primitive_type == PRIMITIVE_TYPE_LINEAR_JOINT:
            filename = f"{data_dir}/trajectory_{self.timestamp}_reduced_PTP.csv"
            self.save_joint_primitives(sequence, filename)
        elif primitive_type == PRIMITIVE_TYPE_LINEAR_CARTESIAN:
            filename = f"{data_dir}/trajectory_{self.timestamp}_reduced_LIN.csv"
            self.save_cartesian_primitives(sequence, filename)
        else:
            self.get_logger().error(f"Unsupported primitive type: {primitive_type}")
            rclpy.shutdown()
            return
        self.get_logger().info("Motion primitives saved.")

    def check_and_export_all(self):
        if self.trajectory_msg and self.poses_msg and self.motion_primitives_msg:
            if self.recording_joint_states:
                self.get_logger().info(
                    "Waiting for joint_states recording to finish before exporting."
                )
                return
            self.save_trajectory_and_poses()
            self.save_executed_joint_states()
            self.get_logger().info("All data saved. Exiting.")
            self.destroy_node()
            rclpy.shutdown()

    def save_joint_primitives(self, primitives, filename):
        joint_names = (
            self.trajectory_msg.joint_names
            if self.trajectory_msg
            else [f"joint_{i}" for i in range(len(primitives[0].joint_positions))]
        )

        folder = os.path.dirname(filename)
        if folder and not os.path.exists(folder):
            os.makedirs(folder)

        with open(filename, mode="w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([f"{name}_pos" for name in joint_names])
            for p in primitives:
                writer.writerow(p.joint_positions)

        self.get_logger().info(f"Saved joint motion primitives to {filename}")

    def save_cartesian_primitives(self, primitives, filename):
        with open(filename, mode="w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(
                ["pose_x", "pose_y", "pose_z", "pose_qx", "pose_qy", "pose_qz", "pose_qw"]
            )
            for p in primitives:
                if not p.poses:
                    self.get_logger().warn("Skipping primitive with no poses.")
                    continue
                pose = p.poses[0].pose  # PoseStamped -> Pose
                writer.writerow(
                    [
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]
                )
        self.get_logger().info(f"Saved cartesian motion primitives to {filename}")

    def save_trajectory_and_poses(self):
        traj_points = self.trajectory_msg.points
        poses = self.poses_msg.poses

        if len(traj_points) != len(poses):
            self.get_logger().error(
                f"Mismatch: {len(traj_points)} trajectory points vs {len(poses)} poses"
            )
            return

        filename = f"{data_dir}/trajectory_{self.timestamp}_planned.csv"
        with open(filename, mode="w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            joint_names = self.trajectory_msg.joint_names
            writer.writerow(
                ["time_from_start"]
                + [f"{name}_pos" for name in joint_names]
                + ["pose_x", "pose_y", "pose_z", "pose_qx", "pose_qy", "pose_qz", "pose_qw"]
            )
            for point, pose in zip(traj_points, poses):
                t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                row = (
                    [t]
                    + list(point.positions)
                    + [
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]
                )
                writer.writerow(row)
        self.get_logger().info(f"Saved planned trajectory and poses to {filename}")

    def save_executed_joint_states(self):
        filename = f"{data_dir}/trajectory_{self.timestamp}_executed.csv"
        with open(filename, mode="w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            if not self.executed_joint_states:
                self.get_logger().warn("No joint_states recorded.")
                return
            first_msg = self.executed_joint_states[0][1]
            joint_names = list(first_msg.name)
            header = (
                ["timestamp"]
                + [f"{name}_pos" for name in joint_names]
                + [f"{name}_vel" for name in joint_names]
            )
            writer.writerow(header)

            for (sec, nsec), msg in self.executed_joint_states:
                t = sec + nsec * 1e-9
                row = [t] + list(msg.position) + list(msg.velocity)
                writer.writerow(row)
        self.get_logger().info(f"Saved executed joint_states to {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = MotionPrimitiveCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
