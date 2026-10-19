"""
Single DD-Boat Trajectory Tracking Simulator
Course: BEST 2026 / Luc Jaulin (ENSTA Bretagne)
Exercise: Feedback Linearization for DD-Boat trajectory following
"""

import numpy as np
import matplotlib.pyplot as plt
from ddboatlib import init_figure, clear, draw_ddboat

def f(x, u):
    """
    State equations of the DD-Boat:
    x = [xr, yr, θr, vr]^T
    u = [u1, u2]^T (angular rate demand, forward acceleration demand)
    """
    xr, yr, θr, vr = x.flatten()
    u1, u2 = u.flatten()
    return np.array([[vr * np.cos(θr)], [vr * np.sin(θr)], [u1], [u2]])

def A_inv(θ, v):
    """
    Invert decoupling matrix:
    [ẍ; ÿ] = [[-v*sin(θ), cos(θ)], [v*cos(θ), sin(θ)]] * [u1; u2]
    """
    # Prevent division by zero near standstill
    v_safe = v if abs(v) > 0.05 else (0.05 if v >= 0 else -0.05)
    return np.array([
        [-np.sin(θ) / v_safe, np.cos(θ) / v_safe],
        [np.cos(θ), np.sin(θ)]
    ])

def control(x, t):
    """
    Feedback linearization controller to track elliptical trajectory:
    x_hat(t) = 3*sin(ω*t) + 6
    y_hat(t) = 5*cos(ω*t) + 10
    with ω = 0.1 rad/s
    """
    omega = 0.1
    # Target reference position and its derivatives
    p_hat = np.array([[3 * np.sin(omega * t) + 6], [5 * np.cos(omega * t) + 10]])
    p_hat_dot = np.array([[3 * omega * np.cos(omega * t)], [-5 * omega * np.sin(omega * t)]])
    p_hat_ddot = np.array([[-3 * (omega**2) * np.sin(omega * t)], [-5 * (omega**2) * np.cos(omega * t)]])
    
    # Current position and velocity
    xr, yr, θr, vr = x.flatten()
    p = np.array([[xr], [yr]])
    p_dot = np.array([[vr * np.cos(θr)], [vr * np.sin(θr)]])
    
    # Desired closed-loop acceleration: ë + 2λė + λ²e = 0 (pole placement at -λ)
    lam = 1.0
    w = p_hat_ddot - 2 * lam * (p_dot - p_hat_dot) - (lam**2) * (p - p_hat)
    
    # Compute control input [u1, u2]^T
    u = A_inv(θr, vr) @ w
    return u, p_hat

def run_simulation(duration=65.0, dt=0.05, save_fig=True):
    ax = init_figure(-1, 13, -1, 21)
    x = np.array([[8.0], [8.0], [1.0], [1.0]])
    
    # Static ellipse geometry for background display
    s = np.arange(0, 2 * np.pi, 0.02)
    ellipse_x = 3 * np.sin(s) + 6
    ellipse_y = 5 * np.cos(s) + 10
    
    trajectory = []
    
    print(f"Starting simulation for {duration}s (ellipse period ~ 62.8s)...")
    fig = ax.figure
    for t in np.arange(0, duration, dt):
        if not plt.fignum_exists(fig.number):
            print("Window closed. Stopping simulation.")
            break
        clear(ax)
        
        # Plot reference track
        ax.plot(ellipse_x, ellipse_y, 'b--', linewidth=1.5, label='Target Ellipse')
        
        # Calculate control and state update
        u, p_hat = control(x, t)
        x = x + dt * f(x, u)
        
        xr, yr, θr, vr = x.flatten()
        trajectory.append((xr, yr))
        
        # Plot target point and trail
        ax.plot(p_hat[0, 0], p_hat[1, 0], 'kx', markersize=8, mew=2, label='Target Point')
        if len(trajectory) > 1:
            traj_arr = np.array(trajectory)
            ax.plot(traj_arr[:, 0], traj_arr[:, 1], 'r-', alpha=0.5, linewidth=1)
        
        # Draw DD-Boat
        draw_ddboat(ax, xr, yr, θr, 0, 0, True, "red")
        
        ax.set_title(f"BEST 2026 - Single DD-Boat Tracker | t = {t:.1f}s | v = {vr:.2f} m/s")
        plt.pause(0.001)
        
    print("Simulation completed.")
    if save_fig:
        plt.savefig("single_boat_trajectory.png", dpi=150, bbox_inches='tight')
        print("Saved trajectory to single_boat_trajectory.png")
    plt.show()

if __name__ == '__main__':
    run_simulation()
