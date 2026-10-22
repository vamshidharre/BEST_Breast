#!/usr/bin/env python3
"""
DD-Boat 11 - Full Mission & Regatta Digital Twin Simulator
=========================================================
Tests the exact autonomous control laws on simulated boat hydrodynamics
BEFORE deploying to the water:
1. Simulates realistic water drag, boat inertia, and steering response
2. Simulates wind / wave drift
3. Simulates realistic GPS sensor noise (+- 1.5m)
4. Evaluates Waypoint Mission & Regatta Shuttle Loop
5. Generates high-resolution trajectory validation plots
"""

import math
import numpy as np
import matplotlib
matplotlib.use('Agg') # Headless safe plotting
import matplotlib.pyplot as plt

def sawtooth(a):
    return math.atan2(math.sin(a), math.cos(a))

# ---------------------------------------------------------------------------
# Hydrodynamic Model of DD-Boat (Prof. Luc Jaulin's state space model)
# State vector: [x, y, theta, v]
#   x: East (m)
#   y: North (m)
#   theta: Nautical heading (rad, 0=N, +pi/2=E)
#   v: Forward velocity (m/s)
# ---------------------------------------------------------------------------
def boat_step(state, u1, u2, dt=0.05, wind_vx=0.15, wind_vy=0.10):
    x, y, theta, v = state
    
    # Scale Arduino motor commands (-120 to +120) to physical thrust
    # Speed 70 corresponds to ~ 0.9 m/s terminal velocity
    t1 = u1 / 70.0 * 0.9
    t2 = u2 / 70.0 * 0.9
    
    # Differential steering: turning rate in rad/s
    # In nautical compass (0=N, 90=E), increasing theta is clockwise
    # When Left motor is faster (t1 > t2), boat turns clockwise (d_theta > 0)
    omega = (t1 - t2) * 1.6  # turning rate
    
    # Forward thrust and quadratic water drag
    forward_thrust = 0.5 * (t1 + t2)
    drag = 0.9 * v * abs(v)
    a = forward_thrust - drag
    
    # Integration
    v_next = v + a * dt
    theta_next = theta + omega * dt
    
    # In nautical frame: vx = v * sin(theta), vy = v * cos(theta)
    # Add external wind / wave current drift
    x_next = x + (v * math.sin(theta) + wind_vx) * dt
    y_next = y + (v * math.cos(theta) + wind_vy) * dt
    
    return [x_next, y_next, sawtooth(theta_next), v_next]


def simulate_waypoint_mission():
    print("[1/2] Simulating 15m x 15m Waypoint Circuit with Sharp Pivots...")
    
    # Start at origin
    state = [0.0, 0.0, 0.0, 0.0] # at (0,0), pointing North, speed 0
    BOX = 15.0
    waypoints = [
        (BOX, 0.0),    # WP1: East
        (BOX, BOX),    # WP2: North-East
        (0.0, BOX),    # WP3: North
        (0.0, 0.0)     # WP4: Home
    ]
    
    wp_idx = 0
    ARRIVAL_RADIUS = 2.2  # Tighter arrival to reach the corners
    CRUISE_SPEED = 70
    PIVOT_SPEED = 75
    Kp = 28.0
    min_dist = 999.0
    
    dt = 0.05
    t_max = 90.0 # 90 seconds
    
    traj_x = []
    traj_y = []
    mode_history = []
    
    t = 0.0
    while t < t_max and wp_idx < len(waypoints):
        rx, ry, rtheta, rv = state
        traj_x.append(rx)
        traj_y.append(ry)
        
        # Add realistic GPS noise (+- 0.8m) to simulate what sensors see
        meas_x = rx + np.random.normal(0, 0.6)
        meas_y = ry + np.random.normal(0, 0.6)
        meas_theta = rtheta + np.random.normal(0, 0.05)
        
        tx, ty = waypoints[wp_idx]
        dist = math.hypot(tx - meas_x, ty - meas_y)
        
        # Corner validation: reaches within 2.2m or passes within 3.0m
        if dist <= ARRIVAL_RADIUS or (min_dist <= 3.0 and dist > min_dist + 0.6):
            wp_idx += 1
            min_dist = 999.0
            if wp_idx >= len(waypoints):
                break
            tx, ty = waypoints[wp_idx]
            state = boat_step(state, 0, 0, dt=0.4) # momentum kill
            t += 0.4
            continue
            
        if dist < min_dist:
            min_dist = dist
            
        theta_d = math.atan2(tx - meas_x, ty - meas_y)
        e_theta = sawtooth(theta_d - meas_theta)
        e_deg = math.degrees(e_theta)
        
        # Sharp Pivot vs Cruise
        if abs(e_deg) > 35.0:
            mode = "PIVOT"
            if e_theta > 0:
                u1 = +PIVOT_SPEED
                u2 = -PIVOT_SPEED
            else:
                u1 = -PIVOT_SPEED
                u2 = +PIVOT_SPEED
        else:
            mode = "CRUISE"
            u1 = int(np.clip(CRUISE_SPEED + Kp * e_theta, 0, 110))
            u2 = int(np.clip(CRUISE_SPEED - Kp * e_theta, 0, 110))
            
        mode_history.append(mode)
        state = boat_step(state, u1, u2, dt)
        t += dt
        
    print(f"      Completed {wp_idx}/{len(waypoints)} waypoints in {t:.1f}s.")
    return traj_x, traj_y, waypoints


def simulate_regatta_shuttle():
    print("[2/2] Simulating Autonomous Regatta Shuttle Loop (Buoy A <-> Buoy B)...")
    
    # Gate A at (5, 0), Gate B at (25, 20)
    gate_a = (5.0, 0.0)
    gate_b = (25.0, 20.0)
    gates = [gate_a, gate_b]
    names = ["Gate A (Green)", "Gate B (Red)"]
    current_gate = 0
    
    state = [0.0, 0.0, math.radians(45.0), 0.0]
    
    R_ARRIVAL = 2.2
    CRUISE_SPEED = 75
    PIVOT_SPEED = 80
    Kp = 30.0
    
    score = 0
    t = 0.0
    t_max = 120.0 # 2 minutes race test
    dt = 0.05
    
    reg_x = []
    reg_y = []
    
    min_dist = 999.0
    while t < t_max:
        rx, ry, rtheta, rv = state
        reg_x.append(rx)
        reg_y.append(ry)
        
        meas_x = rx + np.random.normal(0, 0.6)
        meas_y = ry + np.random.normal(0, 0.6)
        meas_theta = rtheta + np.random.normal(0, 0.05)
        
        gx, gy = gates[current_gate]
        dist = math.hypot(gx - meas_x, gy - meas_y)
        
        # Rounding gate: reach within 2.2m or pass within 3.0m
        if dist <= R_ARRIVAL or (min_dist <= 3.0 and dist > min_dist + 0.6):
            score += 1
            current_gate = 1 - current_gate
            min_dist = 999.0
            gx, gy = gates[current_gate]
            state = boat_step(state, 0, 0, dt=0.4)
            t += 0.4
            continue
            
        if dist < min_dist:
            min_dist = dist
            
        theta_d = math.atan2(gx - meas_x, gy - meas_y)
        e_theta = sawtooth(theta_d - meas_theta)
        e_deg = math.degrees(e_theta)
        
        # 180° Turnaround vs Cruise
        if abs(e_deg) > 35.0:
            if e_theta > 0:
                u1 = +PIVOT_SPEED
                u2 = -PIVOT_SPEED
            else:
                u1 = -PIVOT_SPEED
                u2 = +PIVOT_SPEED
        else:
            u1 = int(np.clip(CRUISE_SPEED + Kp * e_theta, 0, 115))
            u2 = int(np.clip(CRUISE_SPEED - Kp * e_theta, 0, 115))
            
        state = boat_step(state, u1, u2, dt)
        t += dt
        
    print(f"      Regatta completed! Total Gates Cleared: {score} in {t:.1f}s.")
    return reg_x, reg_y, gates, score


def main():
    wp_x, wp_y, waypoints = simulate_waypoint_mission()
    reg_x, reg_y, gates, reg_score = simulate_regatta_shuttle()
    
    # -------------------------------------------------------------
    # Plotting Simulation Results
    # -------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Plot 1: 15m Waypoint Circuit
    ax1.set_title("Waypoint Circuit Simulation (15m x 15m)", fontsize=13, fontweight='bold')
    ax1.plot(wp_x, wp_y, 'b-', linewidth=2.5, label='Boat Trajectory (Hydrodynamic + Noise)')
    
    for i, (wx, wy) in enumerate(waypoints):
        ax1.plot(wx, wy, 'ro', markersize=10)
        ax1.text(wx + 0.6, wy + 0.6, f"WP{i+1}", fontsize=11, fontweight='bold', color='darkred')
        circle = plt.Circle((wx, wy), 2.2, color='r', fill=False, linestyle='--', alpha=0.5)
        ax1.add_patch(circle)
        
    ax1.plot(wp_x[0], wp_y[0], 'go', markersize=11, label='Launch Start (0,0)')
    ax1.plot(wp_x[-1], wp_y[-1], 'ms', markersize=10, label='Finish')
    ax1.set_xlabel("East (meters)", fontsize=11)
    ax1.set_ylabel("North (meters)", fontsize=11)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.axis('equal')
    ax1.legend(loc='upper left', fontsize=9)
    
    # Plot 2: Regatta Shuttle Loop
    ax2.set_title(f"Regatta Shuttle Loop Simulation (Score: {reg_score} Gates)", fontsize=13, fontweight='bold')
    ax2.plot(reg_x, reg_y, 'g-', linewidth=2.0, label='Regatta Shuttle Path')
    
    ax2.plot(gates[0][0], gates[0][1], 'go', markersize=12, label='Gate A (Green Buoy)')
    ax2.plot(gates[1][0], gates[1][1], 'ro', markersize=12, label='Gate B (Red Buoy)')
    ax2.text(gates[0][0] + 0.8, gates[0][1], "GATE A", fontsize=11, fontweight='bold', color='green')
    ax2.text(gates[1][0] + 0.8, gates[1][1], "GATE B", fontsize=11, fontweight='bold', color='red')
    
    ax2.add_patch(plt.Circle(gates[0], 2.2, color='g', fill=False, linestyle='--', alpha=0.5))
    ax2.add_patch(plt.Circle(gates[1], 2.2, color='r', fill=False, linestyle='--', alpha=0.5))
    
    ax2.set_xlabel("East (meters)", fontsize=11)
    ax2.set_ylabel("North (meters)", fontsize=11)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.axis('equal')
    ax2.legend(loc='upper left', fontsize=9)
    
    plt.tight_layout()
    output_path = "C:/Users/vamsh/Desktop/simulation_validation_tight.png"
    plt.savefig(output_path, dpi=200)
    plt.savefig("d:/Personal_Projects/BEST_Breast/simulation_validation_tight.png", dpi=200)
    print(f"\n[OK] High-resolution simulation plot saved to:\n     {output_path}")

if __name__ == "__main__":
    main()
