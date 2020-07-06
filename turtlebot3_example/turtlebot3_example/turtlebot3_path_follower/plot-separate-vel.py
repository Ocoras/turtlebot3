import numpy as np
import matplotlib.pyplot as plt

## Arrow legend info
from matplotlib.legend_handler import HandlerPatch
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection


def make_legend_arrow(legend, orig_handle, xdescent, ydescent, width, height, fontsize):
    p = mpatches.FancyArrow(
        0,
        0.5 * height,
        width,
        0,
        length_includes_head=True,
        head_width=0.75 * height,
        overhang=0,
        color="black",
    )
    return p


from turtlebot3_velocity_calc import (
    straight_to_point,
    Point,
)

font = {
    "family": "Helvetica",
}
plt.rc("font", **font)


n = 21
x_lim = [-0.35, 0.15]
y_lim = [-0.2, 0.2]
arrow_scale = 0.5

hfont = {"fontname": "Helvetica"}
fig, ax = plt.subplots(figsize=[5, 4])
ax.set_title("Linear Velocity at Relative Position ", **hfont)
ax.set_xlabel("x(m)", **hfont)
ax.set_ylabel("y(m)", **hfont)
ax.set_xlim(x_lim)
ax.set_ylim(y_lim)
ax.set_aspect("equal", "box")
plt.grid()
plt.yticks(np.linspace(*y_lim, 5))
ax.set_axisbelow(True)
x, x_step = np.linspace(x_lim[0], x_lim[1], n, endpoint=True, retstep=True)
y, y_step = np.linspace(y_lim[0], y_lim[1], n, endpoint=True, retstep=True)

print("Configured n={}, x_step={}, y_step ={}".format(n, x_step, y_step))
zero = Point()
current_orientation = 0
for y_val in y:
    for x_val in x:
        p = Point(x_val, y_val)

        angle = current_orientation
        v = straight_to_point(p, zero)
        # print(
        #     "x= {:6.2f}, y= {:6.2f}, ang= {:7.2f}, v = {:6.2f}".format(
        #         x_val, y_val, np.rad2deg(angle), v.linear.x
        #     )
        # )

        if abs(v.linear.x) > 0:
            ar = ax.arrow(
                x_val,
                y_val,
                arrow_scale * v.linear.x * np.cos(angle),
                arrow_scale * v.linear.x * np.sin(angle),
                # length_includes_head=True,
                # width=0.006,
                head_width=0.005,
                head_length=0.005,
                overhang=0.9,
                color="black",
            )
(target,) = ax.plot(0, 0, "ro", label="Target")
plt.legend(
    [target, ar],
    ["Target", "Linear Velocity"],
    handler_map={ar: HandlerPatch(patch_func=make_legend_arrow),},
)

fig.savefig(
    "/home/david/Documents/Cambridge/Master's Project/Final Report/img/robotcontrollervel.png",
    dpi=300,
    bbox_inches="tight",
)

fig, ax = plt.subplots(figsize=[5, 5])
ax.set_title("Angular Velocity at Bearing from Target", **hfont)
ax.set_xlabel("x(m)", **hfont)
ax.set_ylabel("y(m)", **hfont)
plt.grid()
plt.yticks(np.linspace(-1, 1, 5))
ax.set_axisbelow(True)


theta, theta_step = np.linspace(-np.pi, np.pi, 16, retstep=True, endpoint=False)
# theta = theta + np.pi / 16
print(np.rad2deg(theta_step))
r = 0.75
for t in theta:
    p = Point(np.cos(t), np.sin(t))
    v = straight_to_point(zero, p)
    if abs(v.angular.z) > 0:
        print(
            "x= {:6.2f}, y= {:6.2f}, ang= {:7.2f}, v = {:6.2f}".format(
                np.cos(t), np.sin(t), np.rad2deg(t), v.angular.z
            )
        )
        dx = arrow_scale * v.angular.z * np.cos(t + np.pi / 2)
        dy = arrow_scale * v.angular.z * np.sin(t + np.pi / 2)
        ar = ax.arrow(
            r * np.cos(t) - dx,
            r * np.sin(t) - dy,
            dx,
            dy,
            length_includes_head=True,
            # width=0.006,
            head_width=0.04,
            head_length=0.03,
            overhang=0,
            color="black",
        )
    # if abs(v.linear.x) > 0:
    #     # Plot linear component
    #     ax.arrow(
    #         0,
    #         0,
    #         5 * np.cos(t) * v.linear.x,
    #         5 * np.sin(t) * v.linear.x,
    #         length_includes_head=True,
    #         head_width=0.04,
    #         head_length=0.03,
    #         overhang=0.9,
    #         color="black",
    #     )
# (origin,) = ax.plot(0, 0, "rx", label="Centre of Robot")
rect = mpatches.Rectangle([0, 0], 0.05, 0.1)
circ = mpatches.Circle([0, 0], 0.075, color="black")
ell1 = mpatches.Ellipse([0, 0.086], 0.06, 0.03)
ell2 = mpatches.Ellipse([0, -0.086], 0.06, 0.03)
patches = [circ, ell1, ell2]
collection = PatchCollection(patches)
ax.add_collection(collection)

robot = ax.arrow(
    0,
    0,
    0.6,
    0,
    head_width=0.1,
    head_length=0.1,
    overhang=0,
    length_includes_head=True,
    color="tab:blue",
)
ang = np.pi / 8
ax.plot(
    [0, 2 * np.cos(-ang)], [0, 2 * np.sin(-ang)], ls="--", color="grey",
)
(limit,) = ax.plot(
    [0, 2 * np.cos(ang)],
    [0, 2 * np.sin(ang)],
    ls="--",
    color="grey",
    label="Linear Motion Limits",
)

x_lim = [-1, 1]
y_lim = [-1, 1]
ax.set_xlim(x_lim)
ax.set_ylim(y_lim)
ax.set_aspect("equal", "box")
# handles, labels = ax.get_legend_handles_labels()

# ax.legend(handles, labels)

plt.legend(
    [robot, limit, ar],
    ["Robot Forward Direction", "Linear Motion Limits", "Angular Velocity at Bearing",],
    handler_map={
        robot: HandlerPatch(patch_func=make_legend_arrow),
        ar: HandlerPatch(patch_func=make_legend_arrow),
    },
    loc="upper left",
)
fig.savefig(
    "/home/david/Documents/Cambridge/Master's Project/Final Report/img/robotcontrollerang.png",
    dpi=300,
    bbox_inches="tight",
)

plt.show()
