"""
Multi-Boat Train Simulator (Boats A, B, and C)
Course: BEST 2026 / Luc Jaulin (ENSTA Bretagne)
Exercise 2: Train of DD-Boats
- Robot A (Red) tracks the ellipse using feedback linearization.
- Robot B (Green) follows a virtual attachment point behind A at distance ℓ.
- Robot C (Blue) follows a virtual attachment point behind B at distance ℓ.
"""

import numpy as np
import matplotlib.pyplot as plt
from ddboatlib import init_figure, clear, draw_ddboat

def f(x, u):
    """
    DD-Boat state equations:
    x = [xr, yr, θr, vr]^T
    u = [u1, u2]^T
    """
    xr, yr, θr, vr = x.flatten()
    u1, u2 = u.flatten()
    return np.array([[vr * np.cos(θr)], [vr * np.sin(θr)], [u1], [u2]])

def A_inv(θ, v):
    """
    Invert decoupling matrix:
    [ẍ; ÿ] = [[-v*sin(θ), cos(θ)], [v*cos(θ), sin(θ)]] * [u1; u2]
    """
    v_safe = v if abs(v) > 0.05 else (0.05 if v >= 0 else -0.05)
    return np.array([
        [-np.sin(θ) / v_safe, np.cos(θ) / v_safe],
        [np.cos(θ), np.sin(θ)]
    ])

def control_leader(xa, t):
    """
    Lead Boat A controller: tracks the elliptical reference trajectory.
    """
    omega = 0.1
    p_hat = np.array([[3 * np.sin(omega * t) + 6], [5 * np.cos(omega * t) + 10]])
    p_hat_dot = np.array([[3 * omega * np.cos(omega * t)], [-5 * omega * np.sin(omega * t)]])
    p_hat_ddot = np.array([[-3 * (omega**2) * np.sin(omega * t)], [-5 * (omega**2) * np.cos(omega * t)]])
    
    xr, yr, θr, vr = xa.flatten()
    p = np.array([[xr], [yr]])
    p_dot = np.array([[vr * np.cos(θr)], [vr * np.sin(θr)]])
    
    lam = 1.0
    w = p_hat_ddot - 2 * lam * (p_dot - p_hat_dot) - (lam**2) * (p - p_hat)
    u = A_inv(θr, vr) @ w
    return u, p_hat

def control_follower(x_follower, x_leader, u_leader, ell=1.5):
    """
    Follower Boat controller: tracks virtual target point behind leader at distance ell.
    """
    p_lead = x_leader[:2]
    θ_lead = x_leader[2, 0]
    v_lead = x_leader[3, 0]
    u1_lead = u_leader[0, 0]
    
    # Virtual attachment point behind leader
    p_hat = p_lead - ell * np.array([[np.cos(θ_lead)], [np.sin(θ_lead)]])
    p_hat_dot = (np.array([[v_lead * np.cos(θ_lead)], [v_lead * np.sin(θ_lead)]]) 
                 - ell * u1_lead * np.array([[-np.sin(θ_lead)], [np.cos(θ_lead)]]))
    
    xr, yr, θr, vr = x_follower.flatten()
    p = np.array([[xr], [yr]])
    p_dot = np.array([[vr * np.cos(θr)], [vr * np.sin(θr)]])
    
    lam = 1.0
    w = - 2 * lam * (p_dot - p_hat_dot) - (lam**2) * (p - p_hat)
    u = A_inv(θr, vr) @ w
    return u, p_hat

def run_train_simulation(duration=65.0, dt=0.05, save_fig=True):
    ax = init_figure(-1, 13, -1, 21)
    
    # Initial conditions
    xa = np.array([[8.0], [8.0], [1.0], [1.0]])
    xb = np.array([[5.0], [5.0], [1.0], [1.0]])
    xc = np.array([[2.0], [2.0], [1.0], [1.0]])
    
    # Static ellipse geometry
    s = np.arange(0, 2 * np.pi, 0.02)
    ellipse_x = 3 * np.sin(s) + 6
    ellipse_y = 5 * np.cos(s) + 10
    
    traj_a, traj_b, traj_c = [], [], []
    ell = 1.5
    
    print(f"Starting Train simulation (A -> B -> C) for {duration}s...")
    fig = ax.figure
    for t in np.arange(0, duration, dt):
        if not plt.fignum_exists(fig.number):
            print("Window closed. Stopping train simulation.")
            break
        clear(ax)
        
        # Plot target ellipse
        ax.plot(ellipse_x, ellipse_y, 'k--', alpha=0.6, label='Track')
        
        # Compute control laws
        ua, p_hat_a = control_leader(xa, t)
        ub, p_hat_b = control_follower(xb, xa, ua, ell=ell)
        uc, p_hat_c = control_follower(xc, xb, ub, ell=ell)
        
        # Update states
        xa = xa + dt * f(xa, ua)
        xb = xb + dt * f(xb, ub)
        xc = xc + dt * f(xc, uc)
        
        traj_a.append((xa[0, 0], xa[1, 0]))
        traj_b.append((xb[0, 0], xb[1, 0]))
        traj_c.append((xc[0, 0], xc[1, 0]))
        
        # Draw target points
        ax.plot(p_hat_a[0, 0], p_hat_a[1, 0], 'rx', markersize=6, mew=1.5)
        ax.plot(p_hat_b[0, 0], p_hat_b[1, 0], 'gx', markersize=6, mew=1.5)
        ax.plot(p_hat_c[0, 0], p_hat_c[1, 0], 'bx', markersize=6, mew=1.5)
        
        # Draw trails
        if len(traj_a) > 1:
            arr_a = np.array(traj_a)
            arr_b = np.array(traj_b)
            arr_c = np.array(traj_c)
            ax.plot(arr_a[:, 0], arr_a[:, 1], 'r-', alpha=0.4)
            ax.plot(arr_b[:, 0], arr_b[:, 1], 'g-', alpha=0.4)
            ax.plot(arr_c[:, 0], arr_c[:, 1], 'b-', alpha=0.4)
            
        # Draw DD-Boats
        draw_ddboat(ax, xa[0, 0], xa[1, 0], xa[2, 0], 0, 0, True, "red")
        draw_ddboat(ax, xb[0, 0], xb[1, 0], xb[2, 0], 0, 0, False, "green")
        draw_ddboat(ax, xc[0, 0], xc[1, 0], xc[2, 0], 0, 0, False, "blue")
        
        ax.set_title(f"Train of DD-Boats: Red(A) -> Green(B) -> Blue(C) | t = {t:.1f}s")
        plt.pause(0.001)
        
    print("Train simulation completed.")
    if save_fig:
        plt.savefig("train_trajectory.png", dpi=150, bbox_inches='tight')
        print("Saved train trajectory to train_trajectory.png")
    plt.show()

if __name__ == '__main__':
    run_train_simulation()
