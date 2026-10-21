#!/usr/bin/env python3
"""
DD-Boat 11 - Robust Compass Calibration & Live Verification
============================================================
Compatible with Python 3.5+ (Requires ONLY numpy)

Heading Convention:
- Geographic Marine Compass (Clockwise):
  North = 0°, East = 90°, South = 180°, West = 270°

Usage:
    python3 calib.py
"""

import sys
import os
import time
import json
import math
import numpy as np
from numpy.linalg import inv, norm

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

INCLINATION_RAD = np.radians(64.0)

def init_hardware():
    imu = None
    try:
        import imu9_driver_v2 as imudrv
        imu = imudrv.Imu9IO()
        print("[OK] Pololu 9-DOF IMU initialized.")
    except Exception as e:
        print("[ERROR] IMU driver failed: {}".format(e))
    return imu

def compute_heading_from_calib(xraw, invA, xbar):
    """
    Calibrates raw reading and computes geographic marine compass heading in degrees [0, 360):
    0° = North, 90° = East, 180° = South, 270° = West
    """
    x_centered = xraw.reshape((3, 1)) - xbar.reshape((3, 1))
    y = invA @ x_centered
    mx, my, _ = y.flatten()

    # Marine Compass convention: Clockwise from North
    # North: mx > 0, my = 0 -> 0°
    # East:  mx = 0, my < 0 -> 90° (using -my)
    # South: mx < 0, my = 0 -> 180°
    # West:  mx = 0, my > 0 -> 270° (using -my)
    heading_rad = math.atan2(-my, mx)
    heading_deg = (math.degrees(heading_rad) + 360.0) % 360.0
    return heading_deg, (mx, my)

def heading_to_rose(h):
    dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    return dirs[int((h + 11.25) / 22.5) % 16]

# ---------------------------------------------------------------------------
# MODE 1: 360° FLAT ROTATION CALIBRATION (FAST & IMMUNE TO TILT ERRORS)
# ---------------------------------------------------------------------------
def calibrate_horizontal_turn(imu, duration=15.0):
    print("\n" + "=" * 55)
    print("MODE 1: 360° FLAT ROTATION CALIBRATION")
    print("=" * 55)
    print("Keep the boat FLAT (on table or ground, away from metal/speakers).")
    print("Rotate the boat slowly in a FULL CIRCLE (or two) for 15 seconds.")
    try:
        input("Press ENTER when ready to start rotating...")
    except NameError:
        raw_input("Press ENTER when ready to start rotating...")

    print("\nRecording magnetic samples... ROTATE THE BOAT NOW...")
    samples = []
    start = time.time()
    while time.time() - start < duration:
        t_left = duration - (time.time() - start)
        x, y, z = imu.read_mag_raw()
        samples.append([x, y, z])
        print("\rRemaining: {:4.1f}s | Captured: {:4d} samples | Current: ({:+6d}, {:+6d})".format(
            t_left, len(samples), x, y), end="", flush=True)
        time.sleep(0.04)

    print("\n\nProcessing captured circle...")
    arr = np.array(samples)
    min_x, max_x = np.min(arr[:, 0]), np.max(arr[:, 0])
    min_y, max_y = np.min(arr[:, 1]), np.max(arr[:, 1])
    min_z, max_z = np.min(arr[:, 2]), np.max(arr[:, 2])

    print("X-range: [{:d}, {:d}] (span: {:d})".format(int(min_x), int(max_x), int(max_x - min_x)))
    print("Y-range: [{:d}, {:d}] (span: {:d})".format(int(min_y), int(max_y), int(max_y - min_y)))

    # Hard-iron offset (center of the circle)
    xbar = np.array([
        [(min_x + max_x) / 2.0],
        [(min_y + max_y) / 2.0],
        [(min_z + max_z) / 2.0]
    ])

    # Soft-iron scale equalization
    span_x = (max_x - min_x) / 2.0
    span_y = (max_y - min_y) / 2.0
    avg_span = (span_x + span_y) / 2.0

    # Transformation matrix invA
    invA = np.array([
        [avg_span / max(span_x, 1.0), 0.0, 0.0],
        [0.0, avg_span / max(span_y, 1.0), 0.0],
        [0.0, 0.0, 1.0]
    ]) / avg_span

    # Align with North: Prompt user to point boat North to set phase offset
    print("\nNow point the boat's bow directly toward MAGNETIC NORTH.")
    try:
        input("Press ENTER when boat is pointing North...")
    except NameError:
        raw_input("Press ENTER when boat is pointing North...")

    # Read North reference
    north_samples = [imu.read_mag_raw() for _ in range(30)]
    mean_north = np.mean(north_samples, axis=0).reshape((3, 1))
    y_ref = invA @ (mean_north - xbar)
    theta_offset = math.atan2(-y_ref[1, 0], y_ref[0, 0])
    print("Aligning North reference (offset: {:+.1f}°)...".format(math.degrees(theta_offset)))

    # Rotation matrix to align North with 0°
    R_align = np.array([
        [math.cos(-theta_offset),  math.sin(-theta_offset), 0.0],
        [-math.sin(-theta_offset), math.cos(-theta_offset), 0.0],
        [0.0, 0.0, 1.0]
    ])
    invA = R_align @ invA

    return invA, xbar

# ---------------------------------------------------------------------------
# MODE 2: 4-POSE JAULIN/ZERR CALIBRATION (WITH FIXED SO(3) ALIGNMENT)
# ---------------------------------------------------------------------------
def calibrate_4poses(imu):
    print("\n" + "=" * 55)
    print("MODE 2: 4-POSE JAULIN CALIBRATION")
    print("=" * 55)
    poses = [
        ("POSE 1/4: NORTH", "Hold boat FLAT pointing bow toward MAGNETIC NORTH.", "North"),
        ("POSE 2/4: SOUTH (INVERTED)", "Turn boat UPSIDE-DOWN, bow pointing SOUTH.", "South"),
        ("POSE 3/4: WEST", "Hold boat FLAT pointing bow toward WEST.", "West"),
        ("POSE 4/4: UP", "Hold boat VERTICAL pointing bow toward the SKY.", "Up")
    ]
    collected = {}
    for title, inst, key in poses:
        print("\n>>> {} <<<".format(title))
        print("    {}".format(inst))
        try:
            input("Press ENTER when steady in position...")
        except NameError:
            raw_input("Press ENTER when steady in position...")
        
        time.sleep(0.5)
        raw_list = [imu.read_mag_raw() for _ in range(50)]
        mean_v = np.mean(raw_list, axis=0).reshape((3, 1))
        
        if np.any(np.abs(mean_v) > 32000):
            print("[WARNING] SENSOR SATURATION DETECTED ({}). Keep away from magnets/speakers!".format(mean_v.flatten()))
        
        collected[key] = mean_v
        print("Captured vector: [{:.0f}, {:.0f}, {:.0f}]".format(mean_v[0,0], mean_v[1,0], mean_v[2,0]))

    xN, xS, xW, xU = collected["North"], collected["South"], collected["West"], collected["Up"]
    
    xbar = 0.5 * (xN + xS)

    vN = xN - xbar
    vW = xW - xbar
    vU = xU - xbar

    I = INCLINATION_RAD
    yN = np.array([[math.cos(I)], [0.0], [-math.sin(I)]])
    yW = np.array([[0.0], [-math.cos(I)], [-math.sin(I)]])
    yU = np.cross(yN.flatten(), yW.flatten()).reshape((3, 1))

    V = np.hstack([vN / norm(vN), vW / norm(vW), vU / norm(vU)])
    Y = np.hstack([yN, yW, yU])

    R = Y @ inv(V)
    mean_radius = np.mean([norm(vN), norm(vW), norm(vU)])
    invA = R / max(mean_radius, 1.0)
    
    return invA, xbar

# ---------------------------------------------------------------------------
# MAIN SCRIPT
# ---------------------------------------------------------------------------
def main():
    print("=" * 60)
    print("      DD-BOAT 11 : COMPASS CALIBRATION SYSTEM             ")
    print("      (Marine Compass: 0°=N, 90°=E, 180°=S, 270°=W)       ")
    print("=" * 60)

    imu = init_hardware()
    if imu is None:
        return

    # Check if existing calibration exists and offer to test it immediately
    if os.path.exists("calib_boat11.json"):
        print("\nFound existing calibration file 'calib_boat11.json'!")
        print("  [T] Test existing calibration (Live compass check)")
        print("  [1] Recalibrate: 360° Flat Circle (Fast, recommended)")
        print("  [2] Recalibrate: 4-Pose Jaulin Method")
        try:
            choice = input("\nEnter T, 1, or 2 [default=T]: ").strip().upper()
        except NameError:
            choice = raw_input("\nEnter T, 1, or 2 [default=T]: ").strip().upper()
        
        if choice == "1":
            invA, xbar = calibrate_horizontal_turn(imu)
        elif choice == "2":
            invA, xbar = calibrate_4poses(imu)
        else:
            with open("calib_boat11.json") as f:
                d = json.load(f)
            xbar = np.array(d["xbar"])
            invA = np.array(d["invA"])
            print("Loaded existing calibration parameters.")
    else:
        print("\nSelect calibration method:")
        print("  [1] Turn Boat in a 360° Flat Circle (Fast, robust, recommended)")
        print("  [2] 4-Pose Jaulin Method (North, South-inv, West, Up)")
        try:
            choice = input("\nEnter 1 or 2 [default=1]: ").strip()
        except NameError:
            choice = raw_input("\nEnter 1 or 2 [default=1]: ").strip()

        if choice == "2":
            invA, xbar = calibrate_4poses(imu)
        else:
            invA, xbar = calibrate_horizontal_turn(imu)

    # Save to JSON and TXT
    calib_data = {
        "boat_id": 11,
        "timestamp": time.time(),
        "xbar": xbar.tolist(),
        "invA": invA.tolist()
    }
    with open("calib_boat11.json", "w") as f:
        json.dump(calib_data, f, indent=4)
    np.savetxt("calib_xbar.txt", xbar)
    np.savetxt("calib_invA.txt", invA)

    # Live verification loop
    print("\n" + "=" * 55)
    print("LIVE COMPASS VERIFICATION (Press Ctrl+C to exit):")
    print("Rotate the boat to test (0°=North, 90°=East, 180°=South, 270°=West)...")
    print("=" * 55)

    try:
        while True:
            raw = np.array(imu.read_mag_raw()).reshape((3, 1))
            heading, (mx, my) = compute_heading_from_calib(raw, invA, xbar)
            rose = heading_to_rose(heading)
            print("\rHeading: {:6.1f}° [{:3s}] | Calibrated: mx={:+6.2f}, my={:+6.2f}".format(
                heading, rose, mx, my), end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nCompass verification finished! Boat 11 is calibrated and ready.")

if __name__ == "__main__":
    main()
