#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Ryan Shim, Gilbert, David Swarbrick

# import math
# import numpy
# import sys
# import termios
#
# from geometry_msgs.msg import Twist
# from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from datetime import datetime, timezone
import numpy as np

from turtlebot3_example.turtlebot3_position_control.turtlebot3_position_control import (
    Turtlebot3PositionControl,
)

terminal_msg = """
Turtlebot3 Position Control
------------------------------------------------------
From the current pose,
x: goal position x (unit: m)
y: goal position y (unit: m)
theta: goal orientation (range: -180 ~ 180, unit: deg)
------------------------------------------------------
"""


class Turtlebot3Logger(Turtlebot3PositionControl):
    def __init__(self):
        super().__init__()
        dt = datetime.now(timezone.utc)
        self.logging_file = open(
            "Turtlebot_position_log-{}-{}-{}-{}:{}.csv".format(
                dt.year, dt.month, dt.day, dt.hour, dt.minute
            ),
            "w",
        )

        self.logging_file.write(
            "Hour, Minute, Second, Microsecond, x(m), y(m), Theta(rad),\n"
        )
        self.test_positions = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 90.0),
            (-1.0, 0.0, 90.0),
            (0.0, -1.0, -180.0),
        ]
        self.test_pos_theta_rad = False
        self.test_pos_loop = False
        self.test_pos_index = 0

        self.get_logger().info("Turtlebot3 logging control node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""

    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(
            msg.pose.pose.orientation
        )
        dt = datetime.now(timezone.utc)
        self.logging_file.write(
            "{},{},{},{},{},{},{},\n".format(
                dt.hour,
                dt.minute,
                dt.second,
                dt.microsecond,
                self.last_pose_x,
                self.last_pose_y,
                self.last_pose_theta,
            )
        )
        self.init_odom_state = True

    def get_key(self):
        """Overwriting the get_key command to provide """
        if self.test_pos_index >= len(self.test_positions):
            if self.test_pos_loop:
                test_pos_index = 0
            else:
                print("Reached end of test positions, do you want to loop? ")
                t = input("(y/n)")
                if t == "y":
                    self.test_pos_index = 0
                else:
                    return 0.0, 0.0, 0.0

        test_pos = self.test_positions[self.test_pos_index]

        if not self.test_pos_theta_rad:
            theta = np.deg2rad(test_pos[2])
        else:
            theta = test_pos[2]

        # Write target position to file using zero timestamp to differentiate from logged positions
        self.logging_file.write(
            "{},{},{},{},{},{},{},\n".format(
                00, 00, 00, 00, test_pos[0], test_pos[1], theta
            )
        )

        self.test_pos_index += 1
        return test_pos[0], test_pos[1], theta

    # def update_callback(self):
    #     if self.init_odom_state is True:
    #         self.generate_path()

    # def generate_path(self):
    #     twist = Twist()
    #
    #     if self.get_key_state is False:
    #         input_x, input_y, input_theta = self.get_key()
    #         self.goal_pose_x = self.last_pose_x + input_x
    #         self.goal_pose_y = self.last_pose_y + input_y
    #         self.goal_pose_theta = self.last_pose_theta + input_theta
    #         self.get_key_state = True
    #
    #     else:
    #         # Step 1: Turn
    #         if self.step == 1:
    #             path_theta = math.atan2(
    #                 self.goal_pose_y - self.last_pose_y,
    #                 self.goal_pose_x - self.last_pose_x,
    #             )
    #             angle = path_theta - self.last_pose_theta
    #             angular_velocity = 0.1  # unit: rad/s
    #
    #             twist, self.step = Turtlebot3Path.turn(
    #                 angle, angular_velocity, self.step
    #             )
    #
    #         # Step 2: Go Straight
    #         elif self.step == 2:
    #             distance = math.sqrt(
    #                 (self.goal_pose_x - self.last_pose_x) ** 2
    #                 + (self.goal_pose_y - self.last_pose_y) ** 2
    #             )
    #             linear_velocity = 0.1  # unit: m/s
    #
    #             twist, self.step = Turtlebot3Path.go_straight(
    #                 distance, linear_velocity, self.step
    #             )
    #
    #         # Step 3: Turn
    #         elif self.step == 3:
    #             angle = self.goal_pose_theta - self.last_pose_theta
    #             angular_velocity = 0.1  # unit: rad/s
    #
    #             twist, self.step = Turtlebot3Path.turn(
    #                 angle, angular_velocity, self.step
    #             )
    #
    #         # Reset
    #         elif self.step == 4:
    #             self.step = 1
    #             self.get_key_state = False
    #
    #         self.cmd_vel_pub.publish(twist)
