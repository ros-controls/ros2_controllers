# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
# This is the copyright notice.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# This line indicates that the code is licensed under the Apache License 2.0.
# you may not use this file except in compliance with the License.
# This reminds you that you must comply with the License.
# You may obtain a copy of the License at
# This line shows where to obtain a copy of the License.
#
#     http://www.apache.org/licenses/LICENSE-2.0
# This is the URL for the Apache License 2.0.
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# This states that the software is provided "AS IS" without warranties.
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# This further clarifies that no warranties are provided.
# See the License for the specific language governing permissions and
# limitations under the License.
# This line reminds you to review the License for details.
#
# Authors: Denis Štogl, Lovro Ivanov
# These are the authors of the code.
#

import rclpy  # Import the ROS2 Python client library for ROS communication
from rclpy.node import Node  # Import the Node class to create ROS2 nodes
from builtin_interfaces.msg import Duration  # Import the Duration message type for representing time durations

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # Import message types for joint trajectories
from sensor_msgs.msg import JointState  # Import the JointState message type for receiving joint state information


class PublisherJointTrajectory(Node):  # Define a new class 'PublisherJointTrajectory' that inherits from Node
    def __init__(self):  # Define the constructor method for the class
        super().__init__("publisher_position_trajectory_controller")  # Initialize the Node with the name "publisher_position_trajectory_controller"
        # Declare all parameters
        self.declare_parameter("controller_name", "position_trajectory_controller")  # Declare the "controller_name" parameter with a default value
        self.declare_parameter("wait_sec_between_publish", 6)  # Declare the "wait_sec_between_publish" parameter with a default value of 6 seconds
        self.declare_parameter("goal_names", ["pos1", "pos2"])  # Declare the "goal_names" parameter with a default list of goal names
        self.declare_parameter("joints", [""])  # Declare the "joints" parameter with a default list containing an empty string
        self.declare_parameter("check_starting_point", False)  # Declare the "check_starting_point" parameter with a default value of False

        # Read parameters
        controller_name = self.get_parameter("controller_name").value  # Retrieve the value of the "controller_name" parameter
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value  # Retrieve the publish interval from the parameters
        goal_names = self.get_parameter("goal_names").value  # Retrieve the list of goal names from the parameters
        self.joints = self.get_parameter("joints").value  # Retrieve the list of joint names from the parameters and store it in the instance variable
        self.check_starting_point = self.get_parameter("check_starting_point").value  # Retrieve the boolean value for checking the starting point
        self.starting_point = {}  # Initialize an empty dictionary to store starting point limits for each joint

        if self.joints is None or len(self.joints) == 0:  # Check if the "joints" parameter is not set or is empty
            raise Exception('"joints" parameter is not set!')  # Raise an exception if the "joints" parameter is missing

        # starting point stuff
        if self.check_starting_point:  # If checking the starting point is enabled...
            # declare nested params
            for name in self.joints:  # For each joint in the "joints" list...
                param_name_tmp = "starting_point_limits" + "." + name  # Construct the parameter name for starting point limits for this joint
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])  # Declare the parameter with default limits (from -2π to 2π)
                self.starting_point[name] = self.get_parameter(param_name_tmp).value  # Retrieve and store the starting point limits for this joint

            for name in self.joints:  # Validate that each joint has exactly two starting point limits defined
                if len(self.starting_point[name]) != 2:  # If the starting point limits list does not have exactly two values...
                    raise Exception('"starting_point" parameter is not set correctly!')  # Raise an exception indicating a configuration error
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )  # Create a subscription to the "joint_states" topic with a queue size of 10 and set the callback to joint_state_callback
        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point  # Set starting_point_ok to True if check_starting_point is False; otherwise, it remains False until verified

        self.joint_state_msg_received = False  # Initialize a flag to track if a joint state message has been received

        # Read all positions from parameters
        self.goals = []  # List of JointTrajectoryPoint; initialize an empty list to store goal trajectories
        for name in goal_names:  # Iterate over each goal name provided in the parameters
            self.declare_parameter(name, rclpy.Parameter.Type.DOUBLE_ARRAY)  # Declare a parameter for the goal expecting an array of doubles
            point = JointTrajectoryPoint()  # Create a new JointTrajectoryPoint instance for this goal

            def get_sub_param(sub_param):  # Define a helper function to retrieve sub-parameters for the current goal
                param_name = name + "." + sub_param  # Construct the full parameter name by combining the goal name and sub-parameter name
                self.declare_parameter(param_name, [float()])  # Declare the sub-parameter with a default value (list with one float)
                param_value = self.get_parameter(param_name).value  # Retrieve the value of the sub-parameter

                float_values = []  # Initialize an empty list to store float-converted values

                if len(param_value) != len(self.joints):  # Check if the number of provided values does not match the number of joints
                    return [False, float_values]  # Return False and an empty list if the check fails

                float_values = [float(value) for value in param_value]  # Convert all values in the parameter to floats

                return [True, float_values]  # Return True and the list of float values if successful

            one_ok = False  # Initialize a flag to indicate if at least one sub-parameter (positions, velocities, etc.) is valid

            [ok, values] = get_sub_param("positions")  # Attempt to retrieve the "positions" sub-parameter for the goal
            if ok:
                point.positions = values  # If valid, assign the positions to the trajectory point
                one_ok = True  # Mark that a valid sub-parameter was found

            [ok, values] = get_sub_param("velocities")  # Attempt to retrieve the "velocities" sub-parameter for the goal
            if ok:
                point.velocities = values  # If valid, assign the velocities to the trajectory point
                one_ok = True  # Mark that a valid sub-parameter was found

            [ok, values] = get_sub_param("accelerations")  # Attempt to retrieve the "accelerations" sub-parameter for the goal
            if ok:
                point.accelerations = values  # If valid, assign the accelerations to the trajectory point
                one_ok = True  # Mark that a valid sub-parameter was found

            [ok, values] = get_sub_param("effort")  # Attempt to retrieve the "effort" sub-parameter for the goal
            if ok:
                point.effort = values  # If valid, assign the effort to the trajectory point
                one_ok = True  # Mark that a valid sub-parameter was found

            if one_ok:  # If at least one of the sub-parameters was valid...
                point.time_from_start = Duration(sec=4)  # Set the time_from_start for the trajectory point to 4 seconds
                self.goals.append(point)  # Append the trajectory point to the list of goals
                self.get_logger().info(f'Goal "{name}" has definition {point}')  # Log the successful definition of the goal

            else:
                self.get_logger().warn(
                    f'Goal "{name}" definition is wrong. This goal will not be used. '
                    "Use the following structure: \n<goal_name>:\n  "
                    "positions: [joint1, joint2, joint3, ...]\n  "
                    "velocities: [v_joint1, v_joint2, ...]\n  "
                    "accelerations: [a_joint1, a_joint2, ...]\n  "
                    "effort: [eff_joint1, eff_joint2, ...]"
                )  # Log a warning if the goal definition is incorrect and will be ignored

        if len(self.goals) < 1:  # If no valid goals were found...
            self.get_logger().error("No valid goal found. Exiting...")  # Log an error indicating that no valid goals exist
            exit(1)  # Exit the program with an error code

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"  # Construct the topic name for publishing based on the controller name

        self.get_logger().info(
            f"Publishing {len(goal_names)} goals on topic '{publish_topic}' every "
            f"{wait_sec_between_publish} s"
        )  # Log information about the publishing topic, the number of goals, and the interval between publishes

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)  # Create a publisher for JointTrajectory messages on the constructed topic with a queue size of 1
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)  # Create a timer that calls the timer_callback at the specified interval
        self.i = 0  # Initialize a counter to cycle through the list of goals

    def timer_callback(self):  # Define the timer callback function that gets called at regular intervals
        if self.starting_point_ok:  # If the starting point is verified to be within the allowed limits...
            self.get_logger().info(f"Sending goal {self.goals[self.i]}.")  # Log the goal that is about to be sent
            traj = JointTrajectory()  # Create a new JointTrajectory message
            traj.joint_names = self.joints  # Set the joint names for the trajectory to the list of joints
            traj.points.append(self.goals[self.i])  # Append the current goal (trajectory point) to the message
            self.publisher_.publish(traj)  # Publish the trajectory message to the designated topic

            self.i += 1  # Increment the goal counter to send the next goal next time
            self.i %= len(self.goals)  # Wrap the counter around if it exceeds the number of available goals

        elif self.check_starting_point and not self.joint_state_msg_received:  # If checking is enabled but no joint state message has been received...
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )  # Log a warning indicating that the starting configuration could not be verified
        else:
            self.get_logger().warn("Start configuration is not within configured limits!")  # Log a warning if the starting configuration is outside the allowed limits

    def joint_state_callback(self, msg):  # Define the callback function to process incoming JointState messages
        if not self.joint_state_msg_received:  # If this is the first joint state message received...
            # check start state
            limit_exceeded = [False] * len(msg.name)  # Initialize a list to track whether each joint's position is within limits
            for idx, enum in enumerate(msg.name):  # Iterate over each joint name and its index in the JointState message
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):  # Check if the joint's position is outside its defined starting point limits
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")  # Log a warning if a joint exceeds its limits
                    limit_exceeded[idx] = True  # Mark that this joint has exceeded its limits

            if any(limit_exceeded):  # If any joint has exceeded its limits...
                self.starting_point_ok = False  # Set the starting_point_ok flag to False
            else:
                self.starting_point_ok = True  # Otherwise, confirm that the starting point is acceptable

            self.joint_state_msg_received = True  # Mark that a joint state message has been received
        else:
            return  # If a joint state message has already been processed, do nothing


def main(args=None):  # Define the main function
    rclpy.init(args=args)  # Initialize the ROS2 Python client library with command-line arguments

    publisher_joint_trajectory = PublisherJointTrajectory()  # Create an instance of the PublisherJointTrajectory node

    try:
        rclpy.spin(publisher_joint_trajectory)  # Spin the node so that it processes incoming messages and timer callbacks
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Keyboard interrupt received. Shutting down node.")  # Handle a keyboard interrupt or external shutdown
    except Exception as e:
        print(f"Unhandled exception: {e}")  # Print any other unhandled exceptions


if __name__ == "__main__":  # Check if the script is being executed as the main program
    main()  # Call the main function to start the node
