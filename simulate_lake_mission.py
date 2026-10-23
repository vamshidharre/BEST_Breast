#!/usr/bin/env python3
"""
DD-Boat 11 - Guerlédan Lake Digital Twin Simulation
===================================================
Coordinates provided by Professor:
  Point A: 48.198863° N, -3.015070° W
  Point B: 48.198259° N, -3.017114° W

Mission:
  1. Launch from Boat's current GPS position (Dock: 48.198817° N, -3.014243° W)
  2. Navigate to Point A (Start Gate / Shore Buoy)
  3. Track line / transit to Point B (Lake Buoy, 165.7m away)
  4. Perform Regatta Shuttle Loop (A <-> B)
  5. Validate using Prof. Luc Jaulin's inner-product crossing criterion:
     <b - a, m - b> > 0
"""

import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

REF_LAT = 48.199111
REF_LON = -3.014930
R_EARTH = 6371000.0
LAT_RAD = math.radians(REF_LAT)

def latlon_to_meters(lat, lon):
    dlat = math.radians(lat - REF_LAT)
    dlon = math.radians(lon - REF_LON)
    x = dlon * R_EARTH * math.cos(LAT_RAD)
    y = dlat * R_EARTH
    return x, y

def sawtooth(a):
    return math.atan2(math.sin(a), math.cos(a))

# Real coordinates from Professor
lat_A, lon_A = 48.198863, -3.015070
lat_B, lon_B = 48.198259, -3.017114
lat_boat, lon_boat = 48.198817, -3.014243

xA, yA = latlon_to_meters(lat_A, lon_A)
xB, yB = latlon_to_meters(lat_B, lon_B)
x_dock, y_dock = latlon_to_meters(lat_boat, lon_boat)

print(f"[COORDINATES] Point A: X={xA:+.2f}m, Y={yA:+.2f}m")
print(f"[COORDINATES] Point B: X={xB:+.2f}m, Y={yB:+.2f}m")
print(f"[COORDINATES] Dock:    X={x_dock:+.2f}m, Y={y_dock:+.2f}m")
dist_AB = math.hypot(xB - xA, yB - yA)
bearing_AB = (math.degrees(math.atan2(xB - xA, yB - yA)) + 360) % 360
print(f"[COURSE] Distance A -> B: {dist_AB:.1f} m | Bearing: {bearing_AB:.1f}°")

# ---------------------------------------------------------------------------
# Hydrodynamic Model (Prof. Luc Jaulin's non-linear DD-Boat dynamics)
# State: [x, y, theta, v]
# ---------------------------------------------------------------------------
def boat_step(state, u1, u2, dt=0.08, wind_vx=-0.12, wind_vy=-0.08):
    x, y, theta, v = state
    # Scale commands (-115 to +115) to physical thruster force
    t1 = u1 / 75.0 * 0.95
    t2 = u2 / 75.0 * 0.95
    omega = (t1 - t2) * 1.55 # turning rate (rad/s)
    forward_thrust = 0.5 * (t1 + t2)
    drag = 0.85 * v * abs(v)
    a = forward_thrust - drag
    v_next = max(0.0, v + a * dt)
    theta_next = sawtooth(theta + omega * dt)
    # Nautical frame (x=East, y=North)
    x_next = x + (v * math.sin(theta) + wind_vx) * dt
    y_next = y + (v * math.cos(theta) + wind_vy) * dt
    return [x_next, y_next, theta_next, v_next]

# ---------------------------------------------------------------------------
# Run 5-Minute Simulation of Regatta Loop
# ---------------------------------------------------------------------------
def run_simulation():
    # Initial state at the dock pointing toward A
    init_hdg = math.atan2(xA - x_dock, yA - y_dock)
    state = [x_dock, y_dock, init_hdg, 0.0]
    
    # Target waypoints sequence: Dock -> A -> B -> A -> B ...
    circuit = [
        (xA, yA, "Gate A (Shore)"),
        (xB, yB, "Gate B (Lake)"),
    ]
    
    target_idx = 0
    t = 0.0
    dt = 0.08
    t_max = 300.0 # 5 minutes race
    
    hist_x, hist_y, hist_t = [], [], []
    hist_u1, hist_u2 = [], []
    hist_err = []
    
    score = 0
    laps = 0
    prev_wp = (x_dock, y_dock)
    ARRIVAL_RADIUS = 3.5
    CRUISE_SPEED = 75
    PIVOT_SPEED = 80
    Kp = 30.0
    pivot_latch = 0
    
    print("\n--- Starting 5-Minute Digital Twin Simulation ---")
    
    while t < t_max:
        rx, ry, rtheta, rv = state
        hist_x.append(rx)
        hist_y.append(ry)
        hist_t.append(t)
        
        # Add realistic GPS noise (+-0.8m) and IMU compass noise (+-1.5°)
        meas_x = rx + np.random.normal(0, 0.5)
        meas_y = ry + np.random.normal(0, 0.5)
        meas_theta = sawtooth(rtheta + np.random.normal(0, 0.03))
        
        tx, ty, label = circuit[target_idx]
        dist = math.hypot(tx - meas_x, ty - meas_y)
        
        # Prof. Luc Jaulin's perpendicular finish plane test:
        # <target - prev, current - target> > 0
        leg_dx = tx - prev_wp[0]
        leg_dy = ty - prev_wp[1]
        passed = ((meas_x - tx) * leg_dx + (meas_y - ty) * leg_dy) > 0
        
        if dist <= ARRIVAL_RADIUS or (dist <= 5.0 and passed):
            score += 1
            print(f"[{t:5.1f}s] CLEARED {label}! Distance={dist:.1f}m -> Total Score: {score} pts")
            prev_wp = (tx, ty)
            target_idx = (target_idx + 1) % len(circuit)
            if target_idx == 0:
                laps += 1
            pivot_latch = 0
            continue
            
        # Desired heading to target
        theta_d = math.atan2(tx - meas_x, ty - meas_y)
        e_theta = sawtooth(theta_d - meas_theta)
        e_deg = math.degrees(e_theta)
        hist_err.append(e_deg)
        
        # Pivot with hysteresis vs cruise
        if pivot_latch == 0 and abs(e_deg) > 40.0:
            pivot_latch = 1 if e_theta > 0 else -1
        elif pivot_latch != 0 and abs(e_deg) < 20.0:
            pivot_latch = 0
            
        if pivot_latch != 0:
            u1 = int(+pivot_latch * PIVOT_SPEED)
            u2 = int(-pivot_latch * PIVOT_SPEED)
        else:
            u1 = int(np.clip(CRUISE_SPEED + Kp * e_theta, 0, 115))
            u2 = int(np.clip(CRUISE_SPEED - Kp * e_theta, 0, 115))
            
        hist_u1.append(u1)
        hist_u2.append(u2)
        
        state = boat_step(state, u1, u2, dt)
        t += dt
        
    print(f"\n[RESULT] 5-Minute Simulation Completed! Total Gates Cleared: {score} ({laps} full round trips).")
    return hist_x, hist_y, hist_t, hist_u1, hist_u2, score, laps

# Run
hx, hy, ht, hu1, hu2, score, laps = run_simulation()

# ---------------------------------------------------------------------------
# Generate High-Resolution Trajectory & Telemetry Plots
# ---------------------------------------------------------------------------
fig = plt.figure(figsize=(15, 7))

# 1. 2D Map Trajectory
ax1 = fig.add_subplot(1, 2, 1)
ax1.set_title(f"DD-Boat 11 Regatta Simulation (Guerlédan Lake)\nScore: {score} Gates Cleared in 5 Minutes ({laps} Laps)", 
              fontsize=12, fontweight='bold')

# Plot Track
ax1.plot(hx, hy, 'b-', linewidth=2.0, alpha=0.85, label='Boat Track (Hydrodynamics + Wind + Noise)')

# Plot Buoys
ax1.plot(xA, yA, 'go', markersize=12, label=f'Point A: Shore Buoy ({lat_A:.5f}, {lon_A:.5f})')
ax1.plot(xB, yB, 'ro', markersize=12, label=f'Point B: Lake Buoy ({lat_B:.5f}, {lon_B:.5f})')
ax1.plot(x_dock, y_dock, 'ms', markersize=10, label=f'Dock Launch ({lat_boat:.5f}, {lon_boat:.5f})')

# Draw course line
ax1.plot([xA, xB], [yA, yB], 'k--', linewidth=1.5, alpha=0.6, label=f'Course Line ({dist_AB:.1f} m, Bearing {bearing_AB:.1f}°)')

# Acceptance circles
circleA = plt.Circle((xA, yA), 3.5, color='g', fill=False, linestyle=':', linewidth=2)
circleB = plt.Circle((xB, yB), 3.5, color='r', fill=False, linestyle=':', linewidth=2)
ax1.add_patch(circleA)
ax1.add_patch(circleB)

ax1.text(xA + 4, yA - 2, "POINT A\n(Shore)", fontsize=10, fontweight='bold', color='darkgreen')
ax1.text(xB - 35, yB - 6, "POINT B\n(Lake)", fontsize=10, fontweight='bold', color='darkred')
ax1.text(x_dock + 4, y_dock + 2, "LAUNCH", fontsize=9, fontweight='bold', color='purple')

ax1.set_xlabel("East (meters relative to Guerlédan Ref)", fontsize=11)
ax1.set_ylabel("North (meters relative to Guerlédan Ref)", fontsize=11)
ax1.grid(True, linestyle=':', alpha=0.6)
ax1.axis('equal')
ax1.legend(loc='upper right', fontsize=8)

# 2. Motor Thrusts and Speed
ax2 = fig.add_subplot(2, 2, 2)
ax2.set_title("Thruster Commands u1 (Left) and u2 (Right)", fontsize=11, fontweight='bold')
ax2.plot(ht[:len(hu1)], hu1, 'r-', alpha=0.7, label='u1 Left Motor')
ax2.plot(ht[:len(hu2)], hu2, 'b-', alpha=0.7, label='u2 Right Motor')
ax2.set_xlabel("Time (s)", fontsize=10)
ax2.set_ylabel("Motor PWM (0-115)", fontsize=10)
ax2.set_xlim(0, 120) # show first 2 minutes for clarity
ax2.grid(True, linestyle=':', alpha=0.6)
ax2.legend(loc='lower right', fontsize=8)

# 3. Distance to Lake vs Time
ax3 = fig.add_subplot(2, 2, 4)
ax3.set_title("Course Distance Progress (A <-> B Round Trips)", fontsize=11, fontweight='bold')
dists_A = [math.hypot(x - xA, y - yA) for x, y in zip(hx, hy)]
ax3.plot(ht, dists_A, 'g-', linewidth=1.5, label='Distance from Point A (m)')
ax3.set_xlabel("Time (s)", fontsize=10)
ax3.set_ylabel("Distance (m)", fontsize=10)
ax3.set_xlim(0, 300)
ax3.grid(True, linestyle=':', alpha=0.6)
ax3.legend(loc='upper right', fontsize=8)

plt.tight_layout()
plot_path = "lake_regatta_simulation.png"
plt.savefig(plot_path, dpi=200)
print(f"\n[OK] High-resolution simulation plot saved to: {plot_path}")
