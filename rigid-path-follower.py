import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy

# from collections import namedtuple

# Point = namedtuple("Point", "x y theta")
# Velocity = namedtuple("Velocity", "x z")
class Velocity:
    def __init__(self, x, z):
        self.x = x
        self.z = z


class Point:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


# path_to_be_followed = [  # x, y, theta
#     Point(1.0, 0.0, 0.0),
#     Point(0.0, 1.0, 90.0),
#     Point(-1.0, 0.0, 90.0),
#     Point(0.0, -1.0, -180.0),
# ]

path_to_be_followed = [  # x, y, theta
    Point(1.0, 0.0, 0.0),
    Point(1.0, 1.0, 90.0),
    Point(0.0, 1.0, 180.0),
    Point(0.0, 0.0, -90.0),
]


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
    a = 0.0
    if abs(theta) < np.pi / 16:
        # Heading roughly straight
        a = 0.0
    elif abs(theta) < np.pi / 4:
        a = -0.1
    elif abs(theta) < np.pi / 2:
        a = -0.2
    elif abs(theta) < np.pi:
        # Turn fastest back to zero
        a = -0.3
    else:
        # In odd case where abs(theta) > pi, stop turning as probably error elsewhere
        a = 0.0
    if theta >= 0:
        # Theta was positive, so want negative angular velocity to subtract
        return -a
    else:
        # Theta was negative, so need positive angular velocity to return to 0
        return a


def straight_to_point(current_position, target_position):
    velocity_target = Velocity(0.0, 0.0)
    x_err = target_position.x - current_position.x
    y_err = target_position.y - current_position.y

    distance = np.sqrt((x_err) ** 2 + (y_err) ** 2)
    path_theta = np.arctan2(y_err, x_err)

    if path_theta < np.pi / 4 and path_theta > -np.pi / 4:
        # keep heading forwards
        # print("Heading Forwards")
        velocity_target.x = linear_velocity_calculation(distance)
        velocity_target.z = angular_velocity_calculation(path_theta)

    elif path_theta >= np.pi / 4 and path_theta < 3 * np.pi / 4:
        # need to turn left
        # print("Turning Left")
        velocity_target.x = 0.0
        velocity_target.z = angular_velocity_calculation(path_theta)

    elif path_theta <= -np.pi / 4 and path_theta > -3 * np.pi / 4:
        # need to turn right
        # print("Turning Right")
        velocity_target.x = 0.0
        velocity_target.z = angular_velocity_calculation(path_theta)

    else:
        # Moving backwards
        # print("Heading Backwards")
        velocity_target.x = -linear_velocity_calculation(distance)
        velocity_target.z = 0.0
    return velocity_target


def turn_to_right_direction_at_point(current_position, target_position):
    velocity_target = Velocity(0.0, 0.0)
    theta_error = target_position.theta - current_position.theta
    velocity_target.z = angular_velocity_calculation(theta_error)


def simulate_one_timestep(position, velocity, timestep):
    v_x = velocity.x * np.cos(position.theta)
    v_y = velocity.x * np.sin(position.theta)
    position.x = position.x + timestep * v_x
    position.y = position.y + timestep * v_y
    position.theta = position.theta + timestep * velocity.z
    return position


pos = Point(0.0, 0.1, 0.0)
for d in path_to_be_followed:
    travelled_points = [pos]
    for i in range(10000):
        v = straight_to_point(pos, d)
        pos = deepcopy(simulate_one_timestep(pos, v, 0.1))
        travelled_points.append(pos)
    plt.plot([p.x for p in travelled_points], [p.y for p in travelled_points], "-o")
plt.show()
