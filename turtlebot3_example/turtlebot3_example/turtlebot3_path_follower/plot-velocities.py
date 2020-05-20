import numpy as np
import matplotlib.pyplot as plt
from turtlebot3_velocity_calc import (
    straight_to_point,
    Point,
)

# Set font to Helvetica
font = {
    "family": "Helvetica",
}
plt.rc("font", **font)

dim = 11
spacing = 0.1
offset_x = -0.3
offset_y = 0
zero = Point()

velocities = []
hfont = {"fontname": "Helvetica"}
fig, ax = plt.subplots()
ax.set_title("Velocity at Relative Position from Target", **hfont)
ax.set_xlabel("x(m)", **hfont)
ax.set_xlim([-0.6 + offset_x, 0.6 + offset_x])
ax.set_ylabel("y(m)", **hfont)
ax.set_ylim([-0.6, 0.6])
ax.set_aspect("equal", "box")

for j in range(dim):
    row = []
    y = -round((dim - 1) / 2) * spacing + j * spacing + offset_y
    for i in range(dim):
        x = -round((dim - 1) / 2) * spacing + i * spacing + offset_x
        p = Point(x, y)
        v = straight_to_point(p, zero)
        # print(v.linear.x)
        if abs(v.linear.x) > 0 or abs(v.angular.z) > 0:
            ax.arrow(
                x,
                y,
                0.3 * v.linear.x,
                0.3 * v.angular.z,
                # length_includes_head=True,
                # width=0.005,
                head_width=0.02,
                label="Velocity",
            )
        row.append(v)
    velocities.append(row)
ax.plot(0, 0, "ro", label="Target")
plt.legend()

fig.savefig(
    "/home/david/Documents/Cambridge/Master's Project/Final Report/img/robotcontrollerlarge.png",
    dpi=300,
    bbox_inches="tight",
)
plt.show()
