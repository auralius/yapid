import numpy as np
import matplotlib.pyplot as plt
import csv

# ============================================================
# Discrete PID matrices
# ============================================================
Adc = np.array([
    [1.00000000, 0.00000000],
    [0.00000000, 0.36787944],
], dtype=float)

Bdc = np.array([
    [0.05000000],
    [0.63212056],
], dtype=float)

Cdc = np.array([
    [2.00000000, -10.00000000],
], dtype=float)

Ddc = np.array([
    [15.00000000],
], dtype=float)

# ============================================================
# Discrete plant matrices
# ============================================================
Adp = np.array([
    [0.99507010, 0.04893156],
    [-0.19572622, 0.95592486],
], dtype=float)

Bdp = np.array([
    [0.00123247],
    [0.04893156],
], dtype=float)

Cdp = np.array([
    [1.00000000, 0.00000000],
], dtype=float)

Ddp = np.array([
    [0.00000000],
], dtype=float)

# ============================================================
# Simulation settings
# ============================================================
Ts = 0.05
Tend = 10.0
steps = int(Tend / Ts)

r = 1.0

# ============================================================
# Initial states
# ============================================================
xc = np.zeros((2, 1), dtype=float)  # controller state
xp = np.zeros((2, 1), dtype=float)  # plant state

# Keep explicit y memories so the ordering matches Arduino:
# Arduino reads plant.Y(0) before plant.Compute(u)
# and reads pid.Y(0) right after pid.Compute(e)
y_pid = 0.0
y_plant = 0.0

# ============================================================
# Logs
# ============================================================
rows = []

# ============================================================
# Main loop
# Matches Arduino logic:
#
# y = plant.Y(0)
# e = r - y
# pid.Compute(e)
# u = pid.Y(0)
# plant.Compute(u)
# log t,r,y,e,u,xc1,xc2,xp1,xp2
# ============================================================
for k in range(1, steps + 1):
    t = k * Ts

    # current plant output, before plant update
    y = y_plant

    # error
    e = r - y

    # ---- PID.Compute(e) ----
    # y_pid = Cdc*xc + Ddc*e
    y_pid = float((Cdc @ xc + Ddc * e)[0, 0])

    # xc = Adc*xc + Bdc*e
    xc = Adc @ xc + Bdc * e

    # Arduino reads pid.Y(0) after pid.Compute(e)
    u = y_pid

    # ---- plant.Compute(u) ----
    # y_plant = Cdp*xp + Ddp*u
    y_plant = float((Cdp @ xp + Ddp * u)[0, 0])

    # xp = Adp*xp + Bdp*u
    xp = Adp @ xp + Bdp * u

    rows.append([
        t,
        r,
        y,
        e,
        u,
        float(xc[0, 0]),
        float(xc[1, 0]),
        float(xp[0, 0]),
        float(xp[1, 0]),
    ])

# ============================================================
# Save CSV
# ============================================================
csv_path = "yapid_ss_validation.csv"

with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["t", "r", "y", "e", "u", "xc1", "xc2", "xp1", "xp2"])
    writer.writerows(rows)

print(f"Saved: {csv_path}")

# ============================================================
# Convert logs to arrays for plotting
# ============================================================
data = np.array(rows, dtype=float)

t_log   = data[:, 0]
r_log   = data[:, 1]
y_log   = data[:, 2]
e_log   = data[:, 3]
u_log   = data[:, 4]
xc1_log = data[:, 5]
xc2_log = data[:, 6]
xp1_log = data[:, 7]
xp2_log = data[:, 8]

# ============================================================
# Plot output tracking
# ============================================================
plt.figure(figsize=(8, 4))
plt.plot(t_log, r_log, label="r")
plt.plot(t_log, y_log, label="y")
plt.grid(True)
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Output")
plt.title("Closed-loop response")
plt.tight_layout()
plt.show()

# ============================================================
# Plot control signal
# ============================================================
plt.figure(figsize=(8, 4))
plt.plot(t_log, u_log, label="u")
plt.grid(True)
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Control")
plt.title("Control signal")
plt.tight_layout()
plt.show()

# ============================================================
# Optional: plot states
# ============================================================
plt.figure(figsize=(8, 4))
plt.plot(t_log, xc1_log, label="xc1")
plt.plot(t_log, xc2_log, label="xc2")
plt.grid(True)
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Controller states")
plt.title("PID states")
plt.tight_layout()
plt.show()

plt.figure(figsize=(8, 4))
plt.plot(t_log, xp1_log, label="xp1")
plt.plot(t_log, xp2_log, label="xp2")
plt.grid(True)
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Plant states")
plt.title("Plant states")
plt.tight_layout()
plt.show()