"""
DD-Boat 11 - Digital Twin Simulation & Practice Exercises
From: digital_twin_presentation.pdf (Antoine Morvan, ENSTA, October 21, 2026)
Slides: Page 13 ("Simulating our boat") & Page 14 ("Exercises")

Exercises Implemented:
  [0] Slide 13 Base: u1=1.0, u2=0.8 (Circular arc, R = 4.5m)
  [1] Exercise 1:    u1 = u2 (Straight line trajectory)
  [2] Exercise 2:    u1 < u2 vs u1 > u2 (Turn direction: Port/CCW vs Starboard/CW)
  [3] Exercise 3:    u1 = -u2 (Pure in-place pivot rotation, zero translation)
  [4] Exercise 4:    Square trajectory via dynamic (u1, u2) schedule (400 steps)
  [all] Run all exercises sequentially

Usage:
  python simu_page13.py 4      # Runs Exercise 4 (Square trajectory)
  python simu_page13.py 1      # Runs Exercise 1 (Straight line)
  python simu_page13.py 2      # Runs Exercise 2 (Turn direction)
  python simu_page13.py 3      # Runs Exercise 3 (In-place spin)
  python simu_page13.py 0      # Runs Page 13 literal example
  python simu_page13.py all    # Runs all exercises sequentially
"""

import sys
import numpy as np
import matplotlib
from roblib import *  # numpy, matplotlib helpers

# Hydrodynamic gains (to be calibrated on the physical boat)
k1, k2 = 1.0, 1.0
dt = 0.05

def f(x, u):
    """
    DD-Boat kinematic dynamic model (Dubins-like)
    x = [x, y, theta]
    u = [u1 (left), u2 (right)]
    """
    v = k2 * (u[0] + u[1]) / 2.0
    w = k1 * (u[1] - u[0])
    return array([v * cos(x[2]), v * sin(x[2]), w])

def run_simulation(mode="4"):
    is_interactive = matplotlib.get_backend().lower() not in ['agg', 'pdf', 'ps', 'svg']
    
    # ---------------------------------------------------------
    # Mode Descriptions and Setup
    # ---------------------------------------------------------
    if mode == "0":
        title_str = "Slide 13 Base: u1=1.0, u2=0.8 (Circular Arc)"
        bounds = (-10, 10, -10, 10)
        total_steps = 400
        print("\n=== RUNNING SLIDE 13 BASE SIMULATION ===")
        print("Inputs: u1 = 1.0 (left), u2 = 0.8 (right)")
        print("Expected: Clockwise circular arc of radius R = 4.5m\n")

    elif mode == "1":
        title_str = "Exercise 1: u1 = u2 = 1.0 (Straight Line)"
        bounds = (-2, 22, -12, 12)
        total_steps = 400
        print("\n=== EXERCISE 1: u1 = u2 (Equal Thrust) ===")
        print("Inputs: u1 = 1.0, u2 = 1.0 (v = 1.0 m/s, w = 0 rad/s)")
        print("Question: What trajectory do you get?")
        print("Answer: A straight line trajectory along the initial heading!\n")

    elif mode == "2":
        title_str = "Exercise 2: u1 < u2 (Left) then u1 > u2 (Right)"
        bounds = (-10, 10, -10, 10)
        total_steps = 400
        print("\n=== EXERCISE 2: Differential Steering ===")
        print("Step 0-199:   u1 = 0.8, u2 = 1.2 -> w = +0.4 rad/s (Turns LEFT / Counter-Clockwise)")
        print("Step 200-399: u1 = 1.2, u2 = 0.8 -> w = -0.4 rad/s (Turns RIGHT / Clockwise)")
        print("Question: Which way does the boat turn?")
        print("Answer: When u1 < u2, it turns LEFT. When u1 > u2, it turns RIGHT!\n")

    elif mode == "3":
        title_str = "Exercise 3: u1 = -u2 (In-Place Pure Rotation)"
        bounds = (-4, 4, -4, 4)
        total_steps = 400
        print("\n=== EXERCISE 3: u1 = -u2 (Counter-Rotating Thrusters) ===")
        print("Inputs: u1 = 1.0, u2 = -1.0 (v = 0.0 m/s, w = -2.0 rad/s)")
        print("Question: What happens?")
        print("Answer: Linear speed v = 0. The boat stays at (0,0) and spins on the spot!\n")

    elif mode == "4":
        title_str = "Exercise 4: Closed Square Trajectory"
        bounds = (-1, 7, -1, 7)
        total_steps = 400
        print("\n=== EXERCISE 4: Square Trajectory via Dynamic (u1, u2) ===")
        print("Cycle: 80 steps forward (u1=u2=1.0) + 20 steps 90-deg turn (u1=-pi/4, u2=pi/4)")
        print("Total: 4 sides x 100 steps = 400 steps (20.0s).")
        print("Goal: Follow a closed square and return to (0,0)!\n")

    else:
        print("Unknown mode: {}".format(mode))
        return

    # ---------------------------------------------------------
    # Initialize roblib figure
    # ---------------------------------------------------------
    ax = init_figure(*bounds)
    title(title_str)

    x = array([0., 0., 0.])
    trajectory = [x[:2].copy()]
    u_turn_mag = (pi / 2.0) / (20.0 * dt * 2.0 * k1)  # pi / 4 = ~0.7854

    for k in range(total_steps):
        # Stop cleanly if GUI window is closed
        if is_interactive and not plt.fignum_exists(ax.figure.number):
            print("Window closed by user.")
            break

        # Compute dynamic controls based on mode
        if mode == "0":
            u = array([1.0, 0.8])
        elif mode == "1":
            u = array([1.0, 1.0])
        elif mode == "2":
            if k < 200:
                u = array([0.8, 1.2])  # u1 < u2 -> turn left
            else:
                u = array([1.2, 0.8])  # u1 > u2 -> turn right
        elif mode == "3":
            u = array([1.0, -1.0])     # u1 = -u2 -> in-place spin
        elif mode == "4":
            cycle_step = k % 100
            if cycle_step < 80:
                u = array([1.0, 1.0])                  # Straight leg (4.0m)
            else:
                u = array([-u_turn_mag, u_turn_mag])   # 90° pivot turn CCW

        # Forward Euler integration step
        x = x + dt * f(x, u)
        trajectory.append(x[:2].copy())

        # Live visual update every 2 steps
        if is_interactive and (k % 2 == 0 or k == total_steps - 1):
            clear(ax)
            traj_arr = array(trajectory)
            plot(traj_arr[:, 0], traj_arr[:, 1], 'b--', linewidth=1.5)
            draw_tank(x, col='darkblue', r=0.5 if mode == "4" else 0.8, w=2)
            pause(0.005)

    # In non-interactive mode, draw completed trajectory
    if not is_interactive:
        traj_arr = array(trajectory)
        plot(traj_arr[:, 0], traj_arr[:, 1], 'b--', linewidth=1.5, label='Boat Path')
        draw_tank(x, col='darkblue', r=0.5 if mode == "4" else 0.8, w=2)
        legend(loc='upper right')

    # Output final metrics
    print("Execution complete for Mode [{}]!".format(mode))
    print("Final Position: x = {:.4f} m, y = {:.4f} m".format(x[0], x[1]))
    print("Final Heading:  theta = {:.2f} rad ({:.1f} deg)".format(x[2], x[2] * 180.0 / pi))
    
    if mode == "3":
        dist_moved = np.linalg.norm(x[:2])
        print("Total translation: {:.6f} m (Zero translation verified!)".format(dist_moved))
    elif mode == "4":
        closure_err = np.linalg.norm(x[:2])
        print("Loop Closure Error: {:.4e} m (Exact closed square!)".format(closure_err))

    # Save output plot
    out_name = "simu_exercise_{}.png".format(mode)
    savefig(out_name, dpi=180, bbox_inches='tight')
    print("Saved plot snapshot to {}".format(out_name))

    # Print Digital Twin Reflection
    print("\n-------------------------------------------------------------")
    print("Reflective Question (Page 14):")
    print("'We now have a model. Is it a digital twin yet?'")
    print("ANSWER: NO! It is not connected to a real boat.")
    print("Currently it is only a Digital Model running in isolated simulation.")
    print("To become a Digital Twin, it must connect via UDP Wi-Fi to Boat 11,")
    print("ingest live GPS/IMU telemetry, and run a Luenberger Observer to")
    print("dynamically compensate for lake wind, currents, and thruster drift.")
    print("-------------------------------------------------------------\n")

    if is_interactive and plt.fignum_exists(ax.figure.number):
        show()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        choice = sys.argv[1].strip().lower()
    else:
        print("=============================================================")
        print("  DD-BOAT 11: DIGITAL TWIN SIMULATION & EXERCISES (PAGE 13-14)")
        print("=============================================================")
        print("  [0] Slide 13 Base: u1=1.0, u2=0.8 (Circular Arc R=4.5m)")
        print("  [1] Exercise 1:    u1 = u2 (Straight line trajectory)")
        print("  [2] Exercise 2:    u1 < u2 vs u1 > u2 (Turn direction)")
        print("  [3] Exercise 3:    u1 = -u2 (In-place spin, zero translation)")
        print("  [4] Exercise 4:    Follow a square via dynamic (u1, u2)")
        print("  [all] Run all exercises sequentially")
        print("=============================================================")
        try:
            choice = input("Enter choice [0, 1, 2, 3, 4, or 'all'] (default '4'): ").strip()
            if not choice:
                choice = "4"
        except EOFError:
            choice = "4"

    if choice == "all":
        for m in ["0", "1", "2", "3", "4"]:
            run_simulation(m)
    else:
        run_simulation(choice)
