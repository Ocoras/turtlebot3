"""Velocity calculations for the Turtlebot Controller"""
import numpy as np

# from geometry_msgs.msg import Twist
class Point:
    def __init__(self, x=0.0, y=0.0, theta_deg=0.0):
        self.x = x
        self.y = y
        self.theta = np.deg2rad(theta_deg)


# Uncomment the below two classes for testing purposes, and comment out Twist import
class Pointz:
    def __init__(self, z=0.0):
        self.z = z


class Twist:
    def __init__(self, x=0.0, z=0.0):
        self.linear = Point(x, 0.0, 0.0)
        self.angular = Pointz(z)


def linear_velocity_calculation(distance):
    v = 0.0
    if distance > 0.3:  # full speed ahead
        v = 0.05
    elif distance > 0.1:  # not yet at zero
        # divide the distance by 10 and round to lowest speed
        v = np.floor(distance * 10) / 100
    elif distance > 0.01:
        # slowest speed until 1cm accuracy achieved
        v = 0.01
    else:
        v = 0.0
    return v


def angular_velocity_calculation(theta):
    """Calculate the angular velocity required to reduce theta to zero."""
    a = 0.0
    if abs(theta) < np.pi / 32:
        # Heading roughly straight
        a = 0.0
    elif abs(theta) < np.pi / 4:
        a = -0.1
    elif abs(theta) < np.pi / 2:
        a = -0.2
    else:
        # Turn fastest back to zero
        # This used to have a theta>pi limit, however numerical errors can make pi slightly more than pi
        # In any such case we still wish to reduce theta.
        a = -0.3
    if theta >= 0:
        # Theta was positive, so want negative angular velocity to subtract
        return a
    else:
        # Theta was negative, so need positive angular velocity to return to 0
        return -a


def straight_to_point(current_position, target_position):
    velocity_target = Twist()
    x_err = target_position.x - current_position.x
    y_err = target_position.y - current_position.y

    distance = np.sqrt((x_err) ** 2 + (y_err) ** 2)
    target_bearing = np.arctan2(y_err, x_err)
    theta_error = current_position.theta - target_bearing

    if abs(theta_error) > np.pi:
        # Can turn in the other direction to get there faster
        if theta_error >= 0:
            # if theta was positive, make it negative now
            theta_error = -(2 * np.pi - abs(theta_error))
        else:
            theta_error = 2 * np.pi - abs(theta_error)

    if abs(theta_error) < np.pi / 8:
        # keep heading forwards, adjusting angular velocity to keep straight.
        velocity_target.linear.x = linear_velocity_calculation(distance)
        velocity_target.angular.z = angular_velocity_calculation(theta_error)

    elif np.pi / 8 <= abs(theta_error) < 3 * np.pi / 4:
        # Stop and adjust if more than 45 degrees off track
        velocity_target.linear.x = 0.0
        velocity_target.angular.z = angular_velocity_calculation(theta_error)
    else:
        # Move backwards only if a short distance, otherwise turn around.
        if distance > 0.1:
            velocity_target.linear.x = 0.0
            velocity_target.angular.z = angular_velocity_calculation(theta_error)
        else:
            velocity_target.linear.x = -linear_velocity_calculation(distance)
            velocity_target.angular.z = 0.0

    return velocity_target


def turn_to_right_direction_at_point(current_position, target_position):
    velocity_target = Twist()
    theta_error = current_position.theta - target_position.theta
    velocity_target.angular.z = angular_velocity_calculation(theta_error)
    return velocity_target
