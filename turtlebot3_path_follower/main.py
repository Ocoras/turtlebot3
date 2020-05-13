#!/usr/bin/env python3

import rclpy

from turtlebot3_follow_path import Turtlebot3PathFollower


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_position_control = Turtlebot3PathFollower()
    rclpy.spin(turtlebot3_position_control)
    turtlebot3_position_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
