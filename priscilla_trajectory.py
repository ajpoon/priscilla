import matplotlib.pyplot as plt
import numpy as np
import math

theta_1 = []
theta_2 = []
x_vals = []
y_vals = []

l1 = 6
l2 = 7

def compute_thetas(x_val, y_val):
    r_val = math.sqrt(x_val**2 + y_val**2)

    acos_arg_1 = (l1**2 + l2**2 - r_val**2) / (2 * l1 * l2)
    acos_arg_2 = (l1**2 - l2**2 + r_val**2) / (2 * l1 * r_val)

    theta_2_val = math.pi - math.acos(acos_arg_1)
    theta_1_val = math.atan2(y_val, x_val) - math.acos(acos_arg_2)

    # Forward kinematics (end-effector position)
    x_fk = (l2 * math.cos(theta_1_val + theta_2_val)) + (l1 * math.cos(theta_1_val))
    y_fk = (l2 * math.sin(theta_1_val + theta_2_val)) + (l1 * math.sin(theta_1_val))

    x_vals.append(x_fk)
    y_vals.append(y_fk)

    return math.degrees(theta_1_val), math.degrees(theta_2_val)

# ----------------------------
# 30 points for the semicircle
# ----------------------------

for i in range(360, 180, -6):
    x_val = 3 + (5 * math.cos(math.radians(i)))
    y_val = 10 + (5 * math.sin(math.radians(i)))

    theta_1_val, theta_2_val = compute_thetas(x_val, y_val)
    theta_1.append(theta_1_val)
    theta_2.append(theta_2_val)

# -------------------------
# 30 points for the diameter
# -------------------------

x_start = 3 + 5 * math.cos(math.radians(180))
y_start = 10 + 5 * math.sin(math.radians(180))

x_end = 3 + 5 * math.cos(math.radians(360))
y_end = 10 + 5 * math.sin(math.radians(360))

for t in range(30):
    alpha = t / 29
    x_val = x_start + alpha * (x_end - x_start)
    y_val = y_start + alpha * (y_end - y_start)

    theta_1_val, theta_2_val = compute_thetas(x_val, y_val)
    theta_1.append(theta_1_val)
    theta_2.append(theta_2_val)

time_values = list(range(60))

# -------------------------
# TIME PLOT (ANGLES + XY)
# -------------------------

fig, ax1 = plt.subplots()

# Angles
ax1.plot(time_values, theta_1, marker='o', label='θ₁(t)')
ax1.plot(time_values, theta_2, marker='s', label='θ₂(t)')
ax1.set_xlabel('Time (t)')
ax1.set_ylabel('Angle (degrees)')
ax1.grid()

for i in range(0, len(time_values), 10):
    ax1.annotate(f"{theta_1[i]:.1f}°", (time_values[i], theta_1[i]),
                 textcoords="offset points", xytext=(0, 8), ha='center')
    ax1.annotate(f"{theta_2[i]:.1f}°", (time_values[i], theta_2[i]),
                 textcoords="offset points", xytext=(0, -12), ha='center')

# Position
ax2 = ax1.twinx()
ax2.plot(time_values, x_vals, linestyle='--', label='x(t)')
ax2.plot(time_values, y_vals, linestyle=':', label='y(t)')
ax2.set_ylabel('Position (x, y)')

for i in range(0, len(time_values), 10):
    ax2.annotate(f"x={x_vals[i]:.1f}", (time_values[i], x_vals[i]),
                 textcoords="offset points", xytext=(10, 0), ha='left')
    ax2.annotate(f"y={y_vals[i]:.1f}", (time_values[i], y_vals[i]),
                 textcoords="offset points", xytext=(10, -10), ha='left')

lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()
ax1.legend(lines_1 + lines_2, labels_1 + labels_2)

plt.title('θ₁(t), θ₂(t), and (x,y) vs Time')
plt.show()

# -------------------------
# XY TRAJECTORY PLOT
# -------------------------

plt.figure()

# -------- Version 2 (your current path) --------
plt.plot(x_vals, y_vals, marker='o', label='Version 2 (End Effector Path)')

# Ideal semicircle for Version 2
ideal_x = []
ideal_y = []
for i in range(360, 180, -6):
    ideal_x.append(3 + (5 * math.cos(math.radians(i))))
    ideal_y.append(10 + (5 * math.sin(math.radians(i))))

plt.plot(ideal_x, ideal_y, linestyle='--', label='Version 2 Target')

# -------- Version 1 (d = 2 cm) --------
small_x = []
small_y = []

radius = 1  # diameter = 2 cm
center_x = 2
center_y = 10

for i in range(360, 180, -6):
    small_x.append(center_x + radius * math.cos(math.radians(i)))
    small_y.append(center_y + radius * math.sin(math.radians(i)))

plt.plot(small_x, small_y, linestyle='-.', marker='x', label='Version 1 Trajectory')

# -------- Motor position --------
plt.scatter(0, 0, marker='s', s=100, label='Motor (0,0)')
plt.annotate("(0,0)", (0, 0), textcoords="offset points", xytext=(5,5))

# -------- Formatting --------
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Semicircle Trajectories (XY Plane)')

plt.axis('equal')
plt.gca().invert_yaxis()  # positive Y downward (south)

plt.grid()
plt.legend()

plt.show()

# -------------------------
# PRINT MOTOR ANGLES
# -------------------------

print([int(round(val)) for val in theta_1])
print([int(round(val)) for val in theta_2])
