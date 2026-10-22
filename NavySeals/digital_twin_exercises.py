"""
DD-Boat 11 - Digital Twin Practical Session 1 Exercises
Source: digital_twin_presentation.pdf (Antoine Morvan, ENSTA, BEST 2026)

Exercises:
1. u1 == u2: What trajectory do you get?
2. u1 < u2 vs u1 > u2: Which way does the boat turn?
3. u1 == -u2: What happens?
4. Make the boat follow a square by changing (u1, u2) over time.
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Model parameters from presentation (Slide 10 & 13)
k1 = 1.0  # Rotation gain
k2 = 1.0  # Forward speed gain
dt = 0.05

def f(x, u):
    """
    Continuous state derivative:
    x = [x, y, theta]
    u = [u1 (left), u2 (right)]
    """
    v = k2 * (u[0] + u[1]) / 2.0
    w = k1 * (u[1] - u[0])
    return np.array([v * np.cos(x[2]), v * np.sin(x[2]), w])

def simulate(u_schedule, x0=np.array([0., 0., 0.]), dt=0.05):
    """
    Simulates Euler integration over a schedule of controls.
    u_schedule: list of (duration, (u1, u2))
    """
    x = np.copy(x0)
    traj = [np.copy(x)]
    t_hist = [0.0]
    u_hist = []
    
    t_curr = 0.0
    for duration, u in u_schedule:
        u_arr = np.array(u, dtype=float)
        steps = int(np.round(duration / dt))
        for _ in range(steps):
            x = x + dt * f(x, u_arr)
            t_curr += dt
            traj.append(np.copy(x))
            t_hist.append(t_curr)
            u_hist.append(u_arr)
            
    return np.array(traj), np.array(t_hist)

# -------------------------------------------------------------
# 1. Exercise 1: u1 == u2 (Equal thrust)
# -------------------------------------------------------------
traj_1, t_1 = simulate([(10.0, (1.0, 1.0))], x0=np.array([0., 0., 0.]))

# -------------------------------------------------------------
# 2. Exercise 2: u1 < u2 vs u1 > u2 (Asymmetric thrust)
# -------------------------------------------------------------
# u1 < u2 (Left=0.8, Right=1.2) -> turning left (CCW)
traj_2_left, t_2_left = simulate([(10.0, (0.8, 1.2))], x0=np.array([0., 0., 0.]))
# u1 > u2 (Left=1.2, Right=0.8) -> turning right (CW)
traj_2_right, t_2_right = simulate([(10.0, (1.2, 0.8))], x0=np.array([0., 0., 0.]))

# -------------------------------------------------------------
# 3. Exercise 3: u1 == -u2 (Differential spin on the spot)
# -------------------------------------------------------------
# u1 = 1.0, u2 = -1.0 (v=0, w = -2.0 rad/s)
traj_3, t_3 = simulate([(5.0, (1.0, -1.0))], x0=np.array([0., 0., 0.]))

# -------------------------------------------------------------
# 4. Exercise 4: Following a square by varying (u1, u2) over time
# -------------------------------------------------------------
# To make an exact closed square with side length 5m:
# 1. Straight leg: u = (1.0, 1.0) -> v = 1.0 m/s -> travel 5.0m in 100 steps (5.0s, dt=0.05).
# 2. Exact 90 deg (pi/2 rad) turn:
#    Choose N_turn = 20 steps (T = 1.0s).
#    Required w = (pi / 2) / 1.0 = pi/2 rad/s.
#    Since w = k1 * (u2 - u1) and v = 0 -> u2 = -u1 = (pi / 4) / k1.
side_len = 5.0
v_straight = 1.0
t_straight = 5.0  # Exactly 100 steps of 0.05s

t_turn = 1.0      # Exactly 20 steps of 0.05s
w_req = (np.pi / 2.0) / t_turn  # pi/2 rad/s
u_turn_mag = w_req / (2.0 * k1) # u2 = +u_turn_mag, u1 = -u_turn_mag

square_schedule = []
for side in range(4):
    square_schedule.append((t_straight, (1.0, 1.0)))                 # Forward straight
    square_schedule.append((t_turn, (-u_turn_mag, u_turn_mag)))       # Exact 90 deg pivot turn

traj_4, t_4 = simulate(square_schedule, x0=np.array([0., 0., 0.]))

# -------------------------------------------------------------
# Plotting the 4 Exercises
# -------------------------------------------------------------
fig, axes = plt.subplots(2, 2, figsize=(14, 12))
plt.subplots_adjust(hspace=0.28, wspace=0.25)

def draw_boat_glyph(ax, x, y, theta, scale=0.4, color='navy'):
    """Draws a neat boat hull polygon at pose (x, y, theta)."""
    # Hull shape points in body frame (bow pointing +x)
    hull = np.array([
        [1.2, 0.0],
        [0.6, 0.5],
        [-0.8, 0.5],
        [-1.0, 0.3],
        [-1.0, -0.3],
        [-0.8, -0.5],
        [0.6, -0.5],
        [1.2, 0.0]
    ]) * scale
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    rotated = (R @ hull.T).T + np.array([x, y])
    ax.fill(rotated[:, 0], rotated[:, 1], color=color, alpha=0.7, zorder=5)
    ax.plot(rotated[:, 0], rotated[:, 1], color='black', lw=1.2, zorder=6)

# Panel 1: u1 == u2
ax1 = axes[0, 0]
ax1.plot(traj_1[:, 0], traj_1[:, 1], 'b-', lw=2.5, label=r'$u_1=1.0, u_2=1.0$ (Straight)')
for idx in [0, len(traj_1)//2, -1]:
    draw_boat_glyph(ax1, traj_1[idx, 0], traj_1[idx, 1], traj_1[idx, 2], scale=0.35, color='royalblue')
ax1.set_title("Exercise 1: $u_1 = u_2$ (Pure Translation)\nStraight line trajectory along initial heading", fontsize=12, fontweight='bold')
ax1.set_xlabel("X [m]")
ax1.set_ylabel("Y [m]")
ax1.grid(True, linestyle='--', alpha=0.6)
ax1.axis('equal')
ax1.legend(loc='upper left')

# Panel 2: u1 < u2 vs u1 > u2
ax2 = axes[0, 1]
ax2.plot(traj_2_left[:, 0], traj_2_left[:, 1], 'g-', lw=2.5, label=r'$u_1 < u_2$ (Left / CCW turn)')
ax2.plot(traj_2_right[:, 0], traj_2_right[:, 1], 'r-', lw=2.5, label=r'$u_1 > u_2$ (Right / CW turn)')
for idx in [0, len(traj_2_left)//3, 2*len(traj_2_left)//3, -1]:
    draw_boat_glyph(ax2, traj_2_left[idx, 0], traj_2_left[idx, 1], traj_2_left[idx, 2], scale=0.35, color='seagreen')
    draw_boat_glyph(ax2, traj_2_right[idx, 0], traj_2_right[idx, 1], traj_2_right[idx, 2], scale=0.35, color='crimson')
ax2.set_title("Exercise 2: $u_1 < u_2$ vs $u_1 > u_2$ (Differential Steering)\nCircular arcs turning Port (CCW) and Starboard (CW)", fontsize=12, fontweight='bold')
ax2.set_xlabel("X [m]")
ax2.set_ylabel("Y [m]")
ax2.grid(True, linestyle='--', alpha=0.6)
ax2.axis('equal')
ax2.legend(loc='upper right')

# Panel 3: u1 == -u2
ax3 = axes[1, 0]
ax3.plot(traj_3[:, 0], traj_3[:, 1], 'mo', markersize=8, label=r'Position $(x(t), y(t)) \equiv (0,0)$')
# Plot heading evolution over time in inset or secondary plot
angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
for ang in angles:
    draw_boat_glyph(ax3, 0.0, 0.0, ang, scale=0.45, color='purple')
ax3.set_title("Exercise 3: $u_1 = -u_2$ (Zero Translation, In-Place Spin)\nBoat pivots on the spot at $(0,0)$; $v=0, \\omega=-2 k_1 u_1$", fontsize=12, fontweight='bold')
ax3.set_xlabel("X [m]")
ax3.set_ylabel("Y [m]")
ax3.set_xlim(-2, 2)
ax3.set_ylim(-2, 2)
ax3.grid(True, linestyle='--', alpha=0.6)
ax3.axis('equal')
ax3.legend(loc='upper right')

# Panel 4: Square trajectory
ax4 = axes[1, 1]
ax4.plot(traj_4[:, 0], traj_4[:, 1], 'purple', lw=2.5, label='Closed Square Trajectory')
ax4.scatter([0], [0], color='black', s=100, zorder=7, label='Start / Finish (0, 0)')
# Sample glyphs along the 4 sides
sample_indices = np.linspace(0, len(traj_4)-1, 12, dtype=int)
for idx in sample_indices:
    draw_boat_glyph(ax4, traj_4[idx, 0], traj_4[idx, 1], traj_4[idx, 2], scale=0.35, color='darkorange')
ax4.set_title("Exercise 4: Square Trajectory via Sequenced Controls\n4 straight segments ($u_1=u_2$) + 4 in-place $90^\\circ$ turns ($u_1=-u_2$)", fontsize=12, fontweight='bold')
ax4.set_xlabel("X [m]")
ax4.set_ylabel("Y [m]")
ax4.grid(True, linestyle='--', alpha=0.6)
ax4.axis('equal')
ax4.legend(loc='center')

# Final formatting
fig.suptitle("Digital Twins Practice 1: DD-Boat Kinematic Simulation (Antoine Morvan / ENSTA)", fontsize=15, fontweight='bold', y=0.98)
plt.tight_layout(rect=[0, 0.03, 1, 0.95])

output_png = "digital_twin_exercises.png"
plt.savefig(output_png, dpi=200)
print(f"Plot saved successfully to {output_png}")

# Print quantitative verification
print("--- Exercise 1 Verification ---")
print(f"Start: {traj_1[0, :2]}, End: {traj_1[-1, :2]}, Final Heading: {np.degrees(traj_1[-1, 2]):.2f} deg")

print("\n--- Exercise 2 Verification ---")
print(f"u1 < u2 -> Final Heading: {np.degrees(traj_2_left[-1, 2]):.2f} deg (CCW / Left)")
print(f"u1 > u2 -> Final Heading: {np.degrees(traj_2_right[-1, 2]):.2f} deg (CW / Right)")

print("\n--- Exercise 3 Verification ---")
dist_moved = np.linalg.norm(traj_3[-1, :2] - traj_3[0, :2])
print(f"u1 = -u2 -> Total position displacement: {dist_moved:.6f} m (Pure rotation)")
print(f"Heading rotated by: {np.degrees(traj_3[-1, 2] - traj_3[0, 2]):.2f} deg")

print("\n--- Exercise 4 Verification ---")
final_closure_err = np.linalg.norm(traj_4[-1, :2] - traj_4[0, :2])
print(f"Square start: {traj_4[0, :2]}, Final end: {traj_4[-1, :2]}")
print(f"Loop closure error: {final_closure_err:.4f} m (Exact closed square!)")
