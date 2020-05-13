"""A file to simulate the robot controller for testing purposes"""

import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
from random import random

from turtlebot3_velocity_calc import (
    linear_velocity_calculation,
    angular_velocity_calculation,
    straight_to_point,
    turn_to_right_direction_at_point,
    Point,
    Twist,
)

# Recreating parts of turtlebot3_follow_path.py to test:
def euler_distance_between_point(p1, p2):
    return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def odom_callback(pos, target):
    v = Twist()
    if euler_distance_between_point(pos, target) < 0.01:
        if abs(pos.theta - target.theta) < np.pi / 32:
            target_reached = True
            p = pos
        else:
            target_reached = False
            v = turn_to_right_direction_at_point(pos, target)

    else:
        target_reached = False
        v = straight_to_point(pos, target)
    return v, target_reached


# Simulation here


def simulate_one_timestep(position, velocity, timestep):
    v_x = velocity.linear.x * np.cos(position.theta)
    v_y = velocity.linear.x * np.sin(position.theta)
    x = position.x + timestep * v_x
    y = position.y + timestep * v_y
    theta = position.theta + timestep * velocity.angular.z
    # Correct for theta going out of bounds
    if theta > np.pi:
        alpha = position.theta - np.pi
        theta = -np.pi + alpha
    if theta < -np.pi:
        beta = abs(position.theta) - np.pi
        theta = np.pi - beta

    # Construct a new point to return, although that takes a degree based input
    return Point(x, y, np.rad2deg(theta))


start_pos = Point(0.0, 0.0, 0.0)


def load_path(file):
    path = []
    f = open(file)
    for line in f:
        x, y, theta = line.split(",")
        path.append(Point(float(x), float(y), float(theta)))
    f.close()
    return path


path_to_be_followed = load_path("path-close-grid.csv")
debug = False
while_loop_limit = 1000
dt = 0.1
true_pos = start_pos
reported_pos = start_pos
simulation_accuracy = 0.1

fig, ax = plt.subplots()
ax.set_title("Robot Path Simulation")
ax.set_xlabel("x(m)")
ax.set_xlim([-0.1, 1.1])
ax.set_ylabel("y(m)")
ax.set_ylim([-0.1, 1.1])
ax.set_aspect("equal", "box")

trajectory = []

for target in path_to_be_followed:
    i = 0
    travelled_points = [true_pos]
    target_reached = False
    while not target_reached and i < while_loop_limit:
        vel, target_reached = odom_callback(reported_pos, target)
        new_pos = simulate_one_timestep(true_pos, vel, dt)
        travelled_points.append(new_pos)

        # Introduce some error in position reporting!
        r = random()
        if r <= simulation_accuracy:
            # Only when greater than the accuracy value update the recorded measurement
            reported_pos = new_pos
        # Update the true position so it can be recorded
        true_pos = new_pos

        if debug:
            print(
                "Currently at x: {:5.2f} y: {:5.2f}, moving in direction {:7.2f} towards x:{:5.2f} y:{:5.2f} theta:{:7.2f}".format(
                    true_pos.x,
                    true_pos.y,
                    np.rad2deg(true_pos.theta),
                    target.x,
                    target.y,
                    np.rad2deg(target.theta),
                )
            )
        i += 1
    if i == while_loop_limit:
        print("Maxed out while")
    trajectory.append(travelled_points)
single_traj = []
for t in trajectory:
    single_traj += t
ax.plot(
    [p.x for p in single_traj],
    [p.y for p in single_traj],
    markersize=0.5,
    color="tab:red",
    label="Simulated Path",
)
ax.plot(
    [a for a in [target.x for target in path_to_be_followed]],
    [b for b in [target.y for target in path_to_be_followed]],
    linestyle="--",
    linewidth=2,
    color="tab:blue",
    label="Desired Path",
)
plt.legend()
plt.show()
