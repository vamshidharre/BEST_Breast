#!/usr/bin/env python3
"""
DD-Boat 11 - Closed-Loop Heading Hold Controller
=================================================
Maintains a desired compass heading (theta_d) using the calibrated IMU
and Prof. Luc Jaulin's sawtooth proportional steering law:
    e_theta = sawtooth(theta_d - theta)
    u1 = v0 - Kp * e_theta   (Left Thruster)
    u2 = v0 + Kp * e_theta   (Right Thruster)

Usage:
    python3 heading_hold.py [target_heading_deg] [speed] [duration_sec]
Example:
    python3 heading_hold.py 90 50 15   # Head East (90 deg) at speed 50 for 15 seconds
"""

import sys
import os
import time
import json
import math
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

def sawtooth(a):
    """Periodic identity function in [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))

def load_calibration():
    """Loads calibration matrices from calib_boat11.json if available."""
    calib_file = "calib_boat11.json"
    if os.path.exists(calib_file):
        try:
            with open(calib_file, "r") as f:
                data = json.load(f)
            xbar = np.array(data["xbar"]).reshape((3, 1))
            invA = np.array(data["invA"]).reshape((3, 3))
            print(f"[OK] Loaded calibration parameters from '{calib_file}'.")
            return invA, xbar
        except Exception as e:
            print(f"[WARN] Error reading {calib_file}: {e}")
    
    print("[NOTE] No calibration file found. Using default uncalibrated identity.")
    return np.eye(3), np.zeros((3, 1))

def get_calibrated_heading(imu, invA, xbar):
    """Reads raw magnetometer and computes heading in radians and degrees."""
    raw = np.array(imu.read_mag_raw()).reshape((3, 1))
    calib = invA @ (raw - xbar)
    mx, my, _ = calib.flatten()
    
    # Heading angle in trigonometric radians
    theta_rad = math.atan2(my, mx)
    heading_deg = (math.degrees(theta_rad) + 360.0) % 360.0
    return theta_rad, heading_deg

def main():
    # Parse CLI arguments
    target_deg = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    v0 = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    Kp = 35.0  # Steering gain

    target_rad = math.radians(target_deg)

    print("=" * 60)
    print("      DD-BOAT 11 : CLOSED-LOOP HEADING CONTROLLER         ")
    print(f"      Target Heading: {target_deg:.1f}° | Base Speed: {v0:.0f} | Duration: {duration:.0f}s")
    print("=" * 60)

    try:
        import imu9_driver_v2 as imudrv
        import arduino_driver_v2 as arddrv
        imu = imudrv.Imu9IO()
        ard = arddrv.ArduinoIO()
    except Exception as e:
        print(f"[ERROR] Could not initialize hardware: {e}")
        return

    invA, xbar = load_calibration()

    start_time = time.time()
    print("\nController engaged. Press Ctrl+C at any time for emergency stop.\n")

    try:
        while time.time() - start_time < duration:
            t_elapsed = time.time() - start_time
            theta_rad, current_deg = get_calibrated_heading(imu, invA, xbar)
            
            # Heading error using sawtooth
            e_theta = sawtooth(target_rad - theta_rad)
            e_deg = math.degrees(e_theta)

            # Differential thrust command
            u1 = int(np.clip(v0 - Kp * e_theta, -120, 120))
            u2 = int(np.clip(v0 + Kp * e_theta, -120, 120))

            ard.send_arduino_cmd_motor(u1, u2)

            print(f"\rt={t_elapsed:4.1f}s | Target: {target_deg:5.1f}° | Current: {current_deg:5.1f}° | Error: {e_deg:+5.1f}° | Cmd: ({u1:+4d}, {u2:+4d})", end="", flush=True)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nManual override / Ctrl+C detected!")
    finally:
        ard.send_arduino_cmd_motor(0, 0)
        print("\nMotors stopped safely.")

if __name__ == "__main__":
    main()
