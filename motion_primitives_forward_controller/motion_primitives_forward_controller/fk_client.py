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
from moveit_msgs.srv import GetPositionFK


class FKClient(Node):
    def __init__(self):
        rclpy.init()
        super().__init__("fk_client")
        self.client = self.create_client(GetPositionFK, "/compute_fk")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /compute_fk service...")

    def compute_fk(self, joint_names, joint_positions, from_frame="base", to_link="tool0"):
        request = GetPositionFK.Request()
        request.header.frame_id = from_frame
        request.fk_link_names = [to_link]
        request.robot_state.joint_state.name = joint_names
        request.robot_state.joint_state.position = joint_positions

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.done():
            result = future.result()
            if result and result.error_code.val == 1:
                return result.pose_stamped[0].pose
            else:
                self.get_logger().warn(f"FK error: code={result.error_code.val}")
        else:
            self.get_logger().error("FK call timed out")

        return None

    def shutdown(self):
        self.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    fk = FKClient()
    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    pose = fk.compute_fk(joint_names, [0.0, 1.0, 0.5, 0.0, 0.0, 0.0])
    if pose:
        print(pose)
    fk.shutdown()
