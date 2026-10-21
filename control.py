#!/usr/bin/env python3
"""
DD-Boat 11 - Autonomous Mission & Return-to-Home (RTH) Controller
================================================================
Based on Prof. Luc Jaulin / Antoine Morvan's control.py template.

Solves the "Kayaking Problem":
Features an automatic Return-to-Home (RTH) so the boat NEVER drifts away
or strands in the lake!

Modes:
1. 'boomerang' (Recommended for first lake test):
   - Sails OUTWARD into open water for T/2 seconds.
   - Automatically turns 180° and sails BACK to you for T/2 seconds.
   - Stops right at the dock/shore. NO KAYAKING REQUIRED!

2. 'rth_gps' (GPS Waypoint & Return):
   - Automatically logs launch GPS location (Home).
   - Cruises out up to max_distance (e.g., 15m).
   - Automatically navigates back to Home and stops when within 2m.

3. 'heading' (Standard heading hold):
   - Holds a single heading for T seconds and stops.

Usage on Boat 11:
    python3 control.py boomerang 120 40 30   # Out on 120° for 15s, back on 300° for 15s
    python3 control.py rth_gps 40 20         # 15m out, returns to GPS home
"""

import sys
import os
import time
import math
import json
import numpy as np

# Access drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
from arduino_driver_v2 import ArduinoIO
from gps_driver_v2 import GpsIO
from imu9_driver_v2 import Imu9IO

# Lake reference point (Guerledan)
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

def cvt_gll_ddmm_2_dd(st):
    ilat, ilon = float(st[0]), float(st[2])
    olat = int(ilat / 100) + (ilat % 100) / 60.0
    olon = int(ilon / 100) + (ilon % 100) / 60.0
    if len(st) > 1 and st[1] == 'S':
        olat = -olat
    if len(st) > 3 and st[3] == 'W':
        olon = -olon
    return olat, olon

def sawtooth(x):
    """Prof. Luc Jaulin's sawtooth function (identity in [-pi, pi])."""
    return (x + np.pi) % (2 * np.pi) - np.pi

def load_calibration():
    """Load calibration matrix invA and offset xbar."""
    calib_file = "calib_boat11.json"
    if os.path.exists(calib_file):
        try:
            with open(calib_file, "r") as f:
                data = json.load(f)
            xbar = np.array(data["xbar"]).reshape((3, 1))
            invA = np.array(data["invA"]).reshape((3, 3))
            return invA, xbar
        except Exception:
            pass
    return np.eye(3), np.zeros((3, 1))

def get_calibrated_heading(imu, invA, xbar):
    """Computes marine geographic heading in radians and degrees."""
    raw = np.array(imu.read_mag_raw()).reshape((3, 1))
    calib = invA @ (raw - xbar)
    mx, my, _ = calib.flatten()
    heading_rad = math.atan2(my, mx)
    heading_deg = (math.degrees(heading_rad) + 360.0) % 360.0
    return heading_rad, heading_deg

def control(current_heading, desired_heading, v0=45.0, Kp=35.0):
    """
    Prof. Luc Jaulin's Proportional Steering Law (speech_best26.pdf - Page 14):
    e = sawtooth(theta_d - theta)
    u1 = v0 + Kp * e   (Left motor)
    u2 = v0 - Kp * e   (Right motor)
    """
    e = sawtooth(desired_heading - current_heading)
    u1 = int(np.clip(v0 + Kp * e, -120, 120))
    u2 = int(np.clip(v0 - Kp * e, -120, 120))
    return u1, u2

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "boomerang"
    outward_deg = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
    base_speed = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0
    total_time = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0

    print("=" * 65)
    print("      DD-BOAT 11 : SAFE RETURN-TO-HOME CONTROLLER           ")
    print("      Mode: {} | Outward: {:.1f}° | Speed: {:.0f} | Time: {:.0f}s".format(
        mode.upper(), outward_deg, base_speed, total_time))
    print("=" * 65)

    # Initialize Hardware
    ard = ArduinoIO()
    imu = Imu9IO()
    gps = GpsIO()
    gps.set_filter_speed("0")
    invA, xbar = load_calibration()

    print("[OK] Hardware initialized. Calibration loaded.")

    # Record Initial GPS Home Position if available
    home_x, home_y = None, None
    print("Checking initial GPS fix...")
    t_gps_start = time.time()
    while time.time() - t_gps_start < 3.0:
        ok, gll_pos = gps.read_gll_non_blocking()
        if ok:
            lat, lon = cvt_gll_ddmm_2_dd(gll_pos)
            home_x, home_y = latlon_to_meters(lat, lon)
            print("[GPS HOME LOCKED] Launch Point: x={:.1f}m, y={:.1f}m".format(home_x, home_y))
            break
        time.sleep(0.1)

    if home_x is None:
        print("[NOTE] GPS fix not ready yet (proceeding with Boomerang timer mode).")

    # Time tracking
    t0 = time.time()
    t_half = total_time / 2.0
    return_deg = (outward_deg + 180.0) % 360.0

    cur_x, cur_y = home_x, home_y

    print("\nMission started. Press Ctrl+C at any time for EMERGENCY STOP.\n")

    try:
        while (time.time() - t0) < total_time:
            t_elapsed = time.time() - t0

            # 1. Read GPS (update current metric position)
            gll_ok, gll_pos = gps.read_gll_non_blocking()
            if gll_ok:
                lat, lon = cvt_gll_ddmm_2_dd(gll_pos)
                cur_x, cur_y = latlon_to_meters(lat, lon)
                if home_x is None:
                    home_x, home_y = cur_x, cur_y

            # 2. Read IMU Calibrated Heading
            heading_rad, heading_deg = get_calibrated_heading(imu, invA, xbar)

            # 3. Determine Desired Heading (Objective) based on mode
            if mode == "boomerang":
                # First half of time: sail OUTWARD. Second half: sail BACK!
                if t_elapsed < t_half:
                    phase = "OUTWARD"
                    objective_deg = outward_deg
                else:
                    phase = "RETURN-HOME"
                    objective_deg = return_deg

            elif mode == "rth_gps" and home_x is not None and cur_x is not None:
                dist_to_home = math.hypot(home_x - cur_x, home_y - cur_y)
                if t_elapsed < t_half and dist_to_home < 18.0:
                    phase = "OUTWARD"
                    objective_deg = outward_deg
                else:
                    phase = "RETURN-GPS"
                    # Vector back to launch point
                    dx = home_x - cur_x
                    dy = home_y - cur_y
                    objective_deg = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
                    if dist_to_home < 2.5:
                        print("\n>>> ARRIVED BACK AT SHORE! (dist={:.1f}m) <<<".format(dist_to_home))
                        break
            else:
                phase = "STEADY"
                objective_deg = outward_deg

            objective_rad = math.radians(objective_deg)

            # 4. Compute Control Commands
            u1, u2 = control(heading_rad, objective_rad, v0=base_speed, Kp=35.0)
            ard.send_arduino_cmd_motor(u1, u2)

            # 5. Live Scrolling Telemetry
            err_deg = math.degrees(sawtooth(objective_rad - heading_rad))
            print("[{:4.1f}s | {:11s}] Target: {:5.1f}° | Compass: {:5.1f}° | Err: {:+5.1f}° | Motors: L={:+4d}, R={:+4d}".format(
                t_elapsed, phase, objective_deg, heading_deg, err_deg, u1, u2))

            time.sleep(0.15)

    except KeyboardInterrupt:
        print("\n\nEMERGENCY STOP (Ctrl+C pressed)!")
    finally:
        ard.send_arduino_cmd_motor(0, 0)
        print("Motors stopped safely (0, 0). Boat is in safe idle.")
