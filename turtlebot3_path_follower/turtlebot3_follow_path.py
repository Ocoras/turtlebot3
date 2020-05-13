"""A path-following robot controller for the Turtlebot3"""

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile

from turtlebot3_example.turtlebot3_position_control.turtlebot_velocity_calc import (
    straight_to_point,
    turn_to_right_direction_at_point,
    Point,
)


class Turtlebot3PathFollower(Node):
    def __init__(self):
        super().__init__("turtlebot3_path_follower")

        self.current_position = Point(0.0, 0.0, 0.0)
        self.path_to_be_followed = [  # x, y, theta (initialise in degrees)
            Point(0.0, 0.0, 0.0),
            Point(1.0, 0.0, 90.0),
            Point(1.0, 1.0, 180.0),
            Point(0.0, 1.0, -90.0),
        ]
        self.path_index = -1
        self.target_reached = False
        self.update_target_position()
        # If wanted
        # self.load_path("path.csv")

        self.init_odom_state = False
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", qos)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, qos
        )
        self.get_logger().info("Turtlebot3 path following node has been initialised.")

    @staticmethod
    def euler_distance_between_point(p1, p2):
        return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def odom_callback(self, msg):
        """Update current position when receiving an odometry message, and publish a new velocity"""
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        _, _, self.current_position.theta = self.euler_from_quaternion(
            msg.pose.pose.orientation
        )

        self.init_odom_state = True

        if (
            euler_distance_between_point(self.current_position, self.target_position)
            < 0.01
            and not self.target_reached
        ):
            # Within 1cm of the destination, now check the orientation
            if (
                abs(self.current_position.theta - self.target_position.theta)
                < np.pi / 32
            ):
                # Within +/- 5.6 degrees of desired orientation
                self.target_reached = True
                # Publish an empty Twist to stop motors moving
                velocity = Twist()
                self.cmd_vel_pub.publish(velocity)
                # Update to next position
                self.update_target_position()

            else:
                self.target_reached = False
                velocity = turn_to_right_direction_at_point(
                    self.current_position, self.target_position
                )
                self.cmd_vel_pub.publish(velocity)
        else:
            self.target_reached = False
            velocity = straight_to_point(self.current_position, self.target_position)
            self.cmd_vel_pub.publish(velocity)

    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def update_target_position(self):
        """Get the next target position in the path"""
        self.path_index += 1
        self.target_reached = False
        try:
            self.target_position = self.path_to_be_followed[self.path_index]
        except IndexError:
            print("Reached end of path")
            self.target_reached = True
            self.target_position = self.current_position

    def load_path(file):
        path = []
        f = open(file)
        for line in f:
            x, y, theta = line.split(",")
            path.append(Point(float(x), float(y), float(theta)))
        f.close()
        self.path_to_be_followed = path
