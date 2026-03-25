import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.signal import cont2discrete

from yapid_utils import generate_pid_ss, generate_second_order_plant_ss


# ============================================================
# User parameters
# ============================================================
Ts = 0.05
Tend = 10.0
steps = int(Tend / Ts)
r = 1.0

# PID parameters
Kp = 5.0
Ki = 2.0
Kd = 0.5
N = 20.0

# Plant parameters
wn = 2.0
zeta = 0.2
K_plant = 1.0

# ============================================================
# Generate discrete PID matrices using yapid_utils.py
# ============================================================
Adc, Bdc, Cdc, Ddc = generate_pid_ss(
    Kp=Kp,
    Ki=Ki,
    Kd=Kd,
    N=N,
    Ts=Ts,
    method="zoh"
)

# ============================================================
# Generate discrete plant matrices
# ============================================================
Adp, Bdp, Cdp, Ddp = generate_second_order_plant_ss(
    wn=wn,
    zeta=zeta,
    K=K_plant,
    Ts=Ts,
    method="zoh"
)

# ============================================================
# Initial states
# ============================================================
xc = np.zeros((2, 1), dtype=float)  # controller state
xp = np.zeros((2, 1), dtype=float)  # plant state

# Explicit outputs to match Arduino ordering
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
# ============================================================
for k in range(1, steps + 1):
    t = k * Ts

    # Current plant output before plant state update
    y = y_plant

    # Error
    e = r - y

    # PID output from current controller state
    y_pid = float((Cdc @ xc + Ddc * e)[0, 0])

    # PID state update
    xc = Adc @ xc + Bdc * e

    # Arduino reads pid.Y(0) after Compute(e)
    u = y_pid

    # Plant output from current plant state
    y_plant = float((Cdp @ xp + Ddp * u)[0, 0])

    # Plant state update
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
# Plot states
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