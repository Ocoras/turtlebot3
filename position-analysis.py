from datetime import datetime, timezone
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

year = 2020
month = 2
day = 13
time = [12, 23]
# time = [14, 29]

data = pd.read_csv(
    "../../../Turtlebot_position_log-{}-{}-{}-{}:{}.csv".format(
        year, month, day, *time
    ),
    header=0,
    parse_dates=[0],
    infer_datetime_format=True,
    # skiprows=[6, 210],  # Need to handle rows from previous iterations that are too long
)
print(data.head())

# self.fig_start_end = plt.figure(1)
# self.fig_start_end_ax = self.fig_start_end.add_subplot(1, 1, 1, aspect=1)
# self.fig_start_end_ax.set_title("Q1: Robot Positions")
plt.figure(1)
plt.title(
    "Path Taken By Robot During Test at {}:{} on {}-{}-{}".format(
        *time, day, month, year
    )
)
plt.plot(data["x(m)"], data["y(m)"])
plt.ylabel("y(m)")
plt.xlabel("x(m)")

data["Timestep"] = data["Timestamp"] - data["Timestamp"].shift(-1)


plt.show()

# class dataPoint:
#     def __init__(self, dt, x, y, theta):
#         self.dt = dt
#         self.x = x
#         self.y = y
#         self.theta = theta

#
# year = 2020
# month = 2
# day = 3
# time = [13, 46]
# f = open("Turtlebot_position_log-{}-{}-{}-{}:{}.csv".format(year, month, day, *time))
# headers = f.readline()
#
#
# index = 0
# target = (0, 0, 0)
#
# data_sorted_by_targets = [[]]
# targets = [target]
# for line in f:
#     hour, minute, second, microsecond, x, y, theta = line.split(",")
#     if hour == minute == second == microsecond == "00":
#         target = (x, y, theta)
#         targets.append(target)
#         index += 1
#         data_sorted_by_targets.append([])
#         print("Found new target")
#
#     else:
#         dt = datetime(
#             year, month, day, hour, minute, second, microsecond, tzinfo=timezone.utc
#         )
#         point = dataPoint(dt, x, y, theta)
#         data_sorted_by_targets[index].append(point)
