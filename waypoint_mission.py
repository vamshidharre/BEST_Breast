#!/usr/bin/env python3
"""
DD-Boat 11 - Autonomous GPS Waypoint Mission
============================================
Complete autonomous surface vehicle mission controller:
1. Acquires real-time GPS position (GPGLL) and calibrated compass heading.
2. Projects coordinates to local Cartesian metric frame (x=East, y=North).
3. Sequences through mission waypoints with arrival detection.
4. Steers using Prof. Luc Jaulin's sawtooth heading error law.
5. Shuts down cleanly and logs telemetry to mission_log.csv.

Usage:
    python3 waypoint_mission.py
"""

import sys
import os
import time
import math
import json
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

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

def sawtooth(a):
    return math.atan2(math.sin(a), math.cos(a))

def load_calibration():
    calib_file = "calib_boat11.json"
    if os.path.exists(calib_file):
        try:
            with open(calib_file, "r") as f:
                data = json.load(f)
            return np.array(data["invA"]), np.array(data["xbar"]).reshape((3, 1))
        except Exception:
            pass
    return np.eye(3), np.zeros((3, 1))

def get_heading(imu, invA, xbar):
    raw = np.array(imu.read_mag_raw()).reshape((3, 1))
    calib = invA @ (raw - xbar)
    mx, my, _ = calib.flatten()
    return math.atan2(my, mx)

def main():
    print("=" * 60)
    print("      DD-BOAT 11 : AUTONOMOUS WAYPOINT MISSION            ")
    print("=" * 60)

    try:
        import gps_driver_v2 as gpddrv
        import imu9_driver_v2 as imudrv
        import arduino_driver_v2 as arddrv
        gps = gpddrv.GpsIO()
        imu = imudrv.Imu9IO()
        ard = arddrv.ArduinoIO()
        gps.set_filter_speed("0")
        print("[OK] All hardware drivers connected.")
    except Exception as e:
        print(f"[ERROR] Driver connection failed: {e}")
        return

    invA, xbar = load_calibration()

    # Wait for first GPS fix to initialize starting position
    print("Waiting for initial GPS fix...")
    cur_x, cur_y = None, None
    while cur_x is None:
        ok, data = gps.read_gll_non_blocking()
        if ok:
            lat, lon = cvt_gll_ddmm_2_dd(data)
            cur_x, cur_y = latlon_to_meters(lat, lon)
            print(f"[GPS LOCKED] Initial position: X={cur_x:.1f}m, Y={cur_y:.1f}m")
        time.sleep(0.2)

    # Mission relative waypoints: 10m square survey around starting position
    # (Easily modified to absolute coordinates)
    waypoints_metric = [
        (cur_x + 10.0, cur_y + 0.0),    # Leg 1: East 10m
        (cur_x + 10.0, cur_y + 10.0),   # Leg 2: North 10m
        (cur_x + 0.0,  cur_y + 10.0),   # Leg 3: West 10m
        (cur_x + 0.0,  cur_y + 0.0),    # Leg 4: Return to start
    ]

    wp_index = 0
    ARRIVAL_RADIUS = 2.5  # meters
    BASE_SPEED = 45.0
    Kp = 40.0

    log_file = "mission_log.csv"
    with open(log_file, "w") as f:
        f.write("time,x,y,target_x,target_y,wp_idx,heading_deg,error_deg,u1,u2\n")

    print(f"\nMission armed with {len(waypoints_metric)} waypoints. Starting navigation!\n")

    try:
        while wp_index < len(waypoints_metric):
            # 1. Update GPS position
            ok, data = gps.read_gll_non_blocking()
            if ok:
                lat, lon = cvt_gll_ddmm_2_dd(data)
                cur_x, cur_y = latlon_to_meters(lat, lon)

            # 2. Check distance to active waypoint
            tx, ty = waypoints_metric[wp_index]
            dist = math.hypot(tx - cur_x, ty - cur_y)

            if dist < ARRIVAL_RADIUS:
                print(f"\n>>> WAYPOINT {wp_index + 1} REACHED (dist={dist:.1f}m)! Moving to next...")
                wp_index += 1
                if wp_index >= len(waypoints_metric):
                    break
                tx, ty = waypoints_metric[wp_index]

            # 3. Guidance: Desired heading angle to target
            theta_d = math.atan2(ty - cur_y, tx - cur_x)

            # 4. Navigation: Current heading from calibrated IMU
            theta = get_heading(imu, invA, xbar)

            # 5. Control: Differential thrust
            e_theta = sawtooth(theta_d - theta)
            u1 = int(np.clip(BASE_SPEED - Kp * e_theta, -100, 100))
            u2 = int(np.clip(BASE_SPEED + Kp * e_theta, -100, 100))

            ard.send_arduino_cmd_motor(u1, u2)

            # Telemetry logging & console output
            h_deg = (math.degrees(theta) + 360) % 360
            e_deg = math.degrees(e_theta)
            print(f"\rWP {wp_index+1}/{len(waypoints_metric)} | Dist: {dist:4.1f}m | Pos: ({cur_x:5.1f}, {cur_y:5.1f}) | Heading: {h_deg:5.1f}° | Err: {e_deg:+5.1f}° | Cmd: ({u1:+3d}, {u2:+3d})", end="", flush=True)

            with open(log_file, "a") as f:
                f.write(f"{time.time():.2f},{cur_x:.2f},{cur_y:.2f},{tx:.2f},{ty:.2f},{wp_index},{h_deg:.1f},{e_deg:.1f},{u1},{u2}\n")

            time.sleep(0.05)

        print("\n\nMISSION ACCOMPLISHED! All waypoints reached.")

    except KeyboardInterrupt:
        print("\n\nMission interrupted by user (Ctrl+C).")
    finally:
        ard.send_arduino_cmd_motor(0, 0)
        print("Motors stopped. Mission log saved to 'mission_log.csv'.")

if __name__ == "__main__":
    main()
