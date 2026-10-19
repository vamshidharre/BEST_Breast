import numpy as np
import matplotlib.pyplot as plt
from ddboatlib import init_figure, clear, draw_ddboat, sawtooth


# ---------------------------------------------------------
# Dynamic model of the DD-Boat with friction:
#   dx1 = vr * cos(θr)
#   dx2 = vr * sin(θr)
#   dx3 = u2 - u1             (differential steering)
#   dx4 = (u1 + u2) - vr      (thrust minus friction)
# ---------------------------------------------------------
def f(x, u):
    xr, yr, θr, vr = x.flatten()
    u1, u2 = u.flatten()
    return np.array([[vr * np.cos(θr)],
                     [vr * np.sin(θr)],
                     [u2 - u1],
                     [(u1 + u2) - vr]])

# ---------------------------------------------------------
# Waypoints from Sir's Whiteboard Drawing:
# 1. Top-Right Corner  -> (9.5, 16.0)
# 2. Top-Left Corner   -> (3.5, 16.0)
# 3. Bottom Target Stop -> (3.5, 6.0)
# ---------------------------------------------------------
waypoints = [
    (9.5, 16.0),   # 1. Target at top-right
    (3.5, 16.0),   # 2. Turn left, go horizontally
    (3.5, 6.0)     # 3. Turn down, go straight south & stop
]

# Initial setup
ax = init_figure(-1, 13, -1, 21)
dt = 0.05
# Start at bottom right: x=7.0, y=2.0, heading θ=0.9 rad, speed v=0.5 m/s
x = np.array([[7.0], [2.0], [0.9], [0.5]])

current_wp = 0
Kp = 2.0        # Steering responsiveness
v0 = 1.2        # Cruising thrust
trajectory = []

print("Simulation started. Follows the trajectory from the whiteboard and stops.")

for t in np.arange(0, 30, dt):
    # Stop cleanly if window is closed
    if not plt.fignum_exists(ax.figure.number):
        print("Window closed.")
        break
        
    clear(ax)
    xr, yr, θr, vr = x.flatten()
    trajectory.append((xr, yr))
    
    # Check if current waypoint is reached (within 1.2 meters)
    if current_wp < len(waypoints):
        target_x, target_y = waypoints[current_wp]
        dist = np.hypot(target_x - xr, target_y - yr)
        if dist < 1.2:
            current_wp += 1
            if current_wp == len(waypoints):
                print(f"Final waypoint reached at t={t:.1f}s! Stopping motors.")

    # Controller logic
    if current_wp < len(waypoints):
        # Steer towards active waypoint
        target_x, target_y = waypoints[current_wp]
        θd = np.arctan2(target_y - yr, target_x - xr)
        e_θ = sawtooth(θd - θr)
        u1 = v0 - Kp * e_θ
        u2 = v0 + Kp * e_θ
        status_text = f"Heading to Waypoint {current_wp + 1}/{len(waypoints)}"
    else:
        # MISSION COMPLETE: Turn off motors -> friction stops the boat!
        u1 = 0.0
        u2 = 0.0
        status_text = "FINAL TARGET REACHED - STOPPED"

    u = np.array([[u1], [u2]])
    x = x + dt * f(x, u)  # Euler integration
    
    # --- DRAWINGS ---
    # 1. Draw planned path & target waypoints
    wp_arr = np.array(waypoints)
    ax.plot(wp_arr[:, 0], wp_arr[:, 1], 'g--', alpha=0.5, label='Mission Path')
    for i, (wx, wy) in enumerate(waypoints):
        color = 'red' if i == current_wp else ('gray' if i < current_wp else 'green')
        ax.plot(wx, wy, 'o', color=color, markersize=8)
        ax.text(wx + 0.3, wy, f"W{i+1}", fontsize=9, color='black', weight='bold')

    # 2. Draw actual boat trajectory trail
    if len(trajectory) > 1:
        traj_arr = np.array(trajectory)
        ax.plot(traj_arr[:, 0], traj_arr[:, 1], 'b-', alpha=0.6, linewidth=1.5)

    # 3. Draw the boat
    draw_ddboat(ax, xr, yr, θr, u1, u2, True, "red")
    
    ax.set_title(f"{status_text} | t = {t:.1f}s | Speed v = {vr:.2f} m/s")
    plt.pause(0.01)

if plt.fignum_exists(ax.figure.number):
    plt.show()