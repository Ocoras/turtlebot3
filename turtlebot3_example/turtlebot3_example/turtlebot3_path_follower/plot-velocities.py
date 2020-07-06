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
offset_x = -0.25
offset_y = 0
limit = (((dim - 1) * spacing) / 2) + spacing
zero = Point()

velocities = []
hfont = {"fontname": "Helvetica"}
fig, ax = plt.subplots(figsize=[4, 3])
ax.set_title("Linear Velocity at Relative Position ", **hfont)
ax.set_xlabel("x(m)", **hfont)
ax.set_ylabel("y(m)", **hfont)
# ax.set_xlim([-limit + offset_x, limit + offset_x])
# ax.set_ylim([-limit, limit])
ax.set_xlim([-limit + offset_x, 0.1])
ax.set_ylim([-0.35, 0.35])
ax.set_aspect("equal", "box")

for j in range(dim):
    row = []
    y = -round((dim - 1) / 2) * spacing + j * spacing + offset_y
    for i in range(dim):
        x = -round((dim - 1) / 2) * spacing + i * spacing + offset_x
        p = Point(x, y)
        v = straight_to_point(p, zero)
        # print(v.linear.x)
        if abs(v.linear.x) > 0:
            # print(x, y)
            ax.arrow(
                x,
                y,
                v.linear.x,
                0,
                # length_includes_head=True,
                # width=0.005,
                head_width=0.01,
                label="Linear",
            )
        # if abs(v.angular.z) > 0:
        #     ax.arrow(
        #         x,
        #         y,
        #         x,
        #         0.3 * v.angular.z,
        #         # length_includes_head=True,
        #         # width=0.005,
        #         head_width=0.02,
        #         label="Angular",
        #     )
        row.append(v)
    velocities.append(row)
v_5 = straight_to_point(Point(0.05, 0), zero)
print(v_5.linear.x)
# ax.arrow(0.05, 0, v.linear.x, 0)
ax.plot(0, 0, "ro", label="Target")
plt.legend()

fig.savefig(
    "/home/david/Documents/Cambridge/Master's Project/Final Report/img/robotcontrollerlarge.png",
    dpi=300,
    bbox_inches="tight",
)
plt.show()
