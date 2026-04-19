import csv
import matplotlib.pyplot as plt
import math
import os

log_file = 'nav_log_20260416_225724.csv'
xs, ys, yaws = [], [], []

with open(log_file, 'r') as f:
    reader = csv.reader(f)
    for row in reader:
        if len(row) >= 4:
            try:
                xs.append(float(row[1]))
                ys.append(float(row[2]))
                yaws.append(float(row[3]))
            except ValueError:
                pass

plt.figure(figsize=(10,10))
plt.plot(xs, ys, 'r-', label='Actual Robot Path')
if xs:
    plt.scatter(xs[0], ys[0], c='green', marker='o', s=100, label='Start')
    plt.scatter(xs[-1], ys[-1], c='blue', marker='x', s=100, label='End')

    for i in range(0, len(xs), max(1, len(xs)//30)):
        plt.arrow(xs[i], ys[i], 
                  0.05 * math.cos(yaws[i]), 
                  0.05 * math.sin(yaws[i]),
                  head_width=0.03, head_length=0.05, fc='k', ec='k', alpha=0.5)

plt.title('Robot Telemetry Path (XY + Heading)')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid()
plt.legend()
plt.axis('equal')

out_path = '/mnt/c/SOLIDWORKS Data/browser/Projects/Robotics Assignment/Robot/artifacts/telemetry_plot.png'
plt.savefig(out_path)
print(f"Telemetry plotted to {out_path}")
