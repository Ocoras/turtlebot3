import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy


class Velocity:
    def __init__(self, x, z):
        self.x = x
        self.z = z


class Point:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


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
    if abs(theta) < np.pi / 16:
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
    velocity_target = Velocity(0.0, 0.0)
    x_err = target_position.x - current_position.x
    y_err = target_position.y - current_position.y

    distance = np.sqrt((x_err) ** 2 + (y_err) ** 2)
    target_bearing = np.arctan2(y_err, x_err)
    # print(np.rad2deg(target_bearing))
    theta_error = current_position.theta - target_bearing

    # if abs(theta_error) > np.pi:
    #     # Can turn in the other direction to get there faster
    #     if theta_error >= 0:
    #         # if theta was positive, make it negative now
    #         theta_error = -(2 * np.pi - abs(theta_error))
    #     else:
    #         theta_error = 2 * np.pi - abs(theta_error)

    # print("Theta error on bearing", np.rad2deg(theta_error))
    if abs(theta_error) < np.pi / 4:
        # keep heading forwards, adjusting angular velocity to keep straight.
        velocity_target.x = linear_velocity_calculation(distance)
        velocity_target.z = angular_velocity_calculation(theta_error)

    elif np.pi / 4 <= abs(theta_error) < 3 * np.pi / 4:
        # Stop and adjust if more than 90 degrees off track
        velocity_target.x = 0.0
        velocity_target.z = angular_velocity_calculation(theta_error)
    else:
        print("backwards")
        # Move backwards only if a short distance, otherwise turn around.
        if distance > 0.1:
            velocity_target.x = 0.0
            velocity_target.z = angular_velocity_calculation(theta_error)
        else:
            velocity_target.x = -linear_velocity_calculation(distance)
            velocity_target.z = 0.0

    return velocity_target


def turn_to_right_direction_at_point(current_position, target_position):
    velocity_target = Velocity(0.0, 0.0)
    theta_error = current_position.theta - target_position.theta
    velocity_target.z = angular_velocity_calculation(theta_error)
    return velocity_target


def simulate_one_timestep(position, velocity, timestep):
    v_x = velocity.x * np.cos(position.theta)
    v_y = velocity.x * np.sin(position.theta)
    position.x = position.x + timestep * v_x
    position.y = position.y + timestep * v_y
    position.theta = position.theta + timestep * velocity.z
    # Correct for theta going out of bounds
    if position.theta > np.pi:
        alpha = position.theta - np.pi
        position.theta = -np.pi + alpha
    if position.theta < -np.pi:
        beta = abs(position.theta) - np.pi
        position.theta = np.pi - beta

    return position


path_to_be_followed = [  # x, y, theta
    Point(0.0, 0.0, 0.0),
    Point(1.0, 0.0, 90.0),
    Point(1.0, 1.0, 180.0),
    Point(0.0, 1.0, -90.0),
]


debug = True
while_loop_limit = 5000
dt = 0.1


def print_data(pos, d, v):
    if debug:
        print(
            "Currently at x: {:5.2f} y: {:5.2f}, moving in direction {:7.2f} towards x:{:5.2f} y:{:5.2f}, velocity x: {:5.2f}, z:{:5.2f}".format(
                pos.x, pos.y, np.rad2deg(pos.theta), d.x, d.y, v.x, v.z
            )
        )


start_pos = Point(-3.0, -3.0, 0.0)
pos = deepcopy(start_pos)
for d in path_to_be_followed:
    i = 0
    travelled_points = [pos]
    moving = True
    while moving and i < while_loop_limit:
        v = straight_to_point(pos, d)
        pos = deepcopy(simulate_one_timestep(pos, v, dt))
        travelled_points.append(pos)
        moving = not (abs(v.x) < 1e-3 and abs(v.z) < 1e-3)
        print_data(pos, d, v)
        i += 1
    if i == while_loop_limit:
        print("Maxed Out")
    i = 0
    # moving = True
    # # print("Arrived, Turning to Right Direction")
    # while moving and i < while_loop_limit:
    #     v = turn_to_right_direction_at_point(pos, d)
    #     pos = deepcopy(simulate_one_timestep(pos, v, dt))
    #     moving = not (abs(v.x) < 1e-3 and abs(v.z) < 1e-3)
    #     print_data(pos, d, v)
    #     i += 1
    if i == while_loop_limit:
        print("Maxed Out")
    # plt.figure()
    plt.plot([p.x for p in travelled_points], [p.y for p in travelled_points], "-")
plt.show()
