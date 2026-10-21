#!/usr/bin/env python3
"""
DD-Boat 11 - Autonomous GPS Waypoint Mission
============================================
Compatible with Python 3.5+
Complete autonomous surface vehicle mission controller:
1. Acquires real-time GPS position (GPGLL) and calibrated compass heading.
2. Projects coordinates to local Cartesian metric frame (x=East, y=North).
3. Sequences through mission waypoints with arrival detection.
4. Steers using Prof. Luc Jaulin's sawtooth heading error law.
5. Shuts down cleanly and logs telemetry to mission_log.csv.

Heading convention:
    Marine Clockwise Compass: 0° = North, 90° = East, 180° = South, 270° = West

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
        print("[ERROR] Driver connection failed: {}".format(e))
        return

    invA, xbar = load_calibration()

    print("Waiting for initial GPS fix...")
    cur_x, cur_y = None, None
    while cur_x is None:
        ok, data = gps.read_gll_non_blocking()
        if ok:
            lat, lon = cvt_gll_ddmm_2_dd(data)
            cur_x, cur_y = latlon_to_meters(lat, lon)
            print("[GPS LOCKED] Initial position: X={:.1f}m, Y={:.1f}m".format(cur_x, cur_y))
        time.sleep(0.2)

    waypoints_metric = [
        (cur_x + 10.0, cur_y + 0.0),
        (cur_x + 10.0, cur_y + 10.0),
        (cur_x + 0.0,  cur_y + 10.0),
        (cur_x + 0.0,  cur_y + 0.0),
    ]

    wp_index = 0
    ARRIVAL_RADIUS = 2.5
    BASE_SPEED = 45.0
    Kp = 40.0

    log_file = "mission_log.csv"
    with open(log_file, "w") as f:
        f.write("time,x,y,target_x,target_y,wp_idx,heading_deg,error_deg,u1,u2\n")

    print("\nMission armed with {} waypoints. Starting navigation!\n".format(len(waypoints_metric)))

    try:
        while wp_index < len(waypoints_metric):
            ok, data = gps.read_gll_non_blocking()
            if ok:
                lat, lon = cvt_gll_ddmm_2_dd(data)
                cur_x, cur_y = latlon_to_meters(lat, lon)

            tx, ty = waypoints_metric[wp_index]
            dist = math.hypot(tx - cur_x, ty - cur_y)

            if dist < ARRIVAL_RADIUS:
                print("\n>>> WAYPOINT {} REACHED (dist={:.1f}m)! Moving to next...".format(wp_index + 1, dist))
                wp_index += 1
                if wp_index >= len(waypoints_metric):
                    break
                tx, ty = waypoints_metric[wp_index]

            theta_d = math.atan2(tx - cur_x, ty - cur_y)
            theta = get_heading(imu, invA, xbar)

            e_theta = sawtooth(theta_d - theta)
            # Prof. Luc Jaulin's proportional steering law (speech_best26.pdf):
            u1 = int(np.clip(BASE_SPEED + Kp * e_theta, -100, 100))
            u2 = int(np.clip(BASE_SPEED - Kp * e_theta, -100, 100))

            ard.send_arduino_cmd_motor(u1, u2)

            h_deg = (math.degrees(theta) + 360) % 360
            e_deg = math.degrees(e_theta)
            print("WP {}/{} | Dist: {:4.1f}m | Pos: ({:5.1f}, {:5.1f}) | Heading: {:5.1f}° | Err: {:+5.1f}° | Motors: ({:+3d}, {:+3d})".format(
                wp_index+1, len(waypoints_metric), dist, cur_x, cur_y, h_deg, e_deg, u1, u2))

            with open(log_file, "a") as f:
                f.write("{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{},{:.1f},{:.1f},{},{}\n".format(
                    time.time(), cur_x, cur_y, tx, ty, wp_index, h_deg, e_deg, u1, u2))

            time.sleep(0.2)

        print("\n\nMISSION ACCOMPLISHED! All waypoints reached.")

    except KeyboardInterrupt:
        print("\n\nMission interrupted by user (Ctrl+C).")
    finally:
        ard.send_arduino_cmd_motor(0, 0)
        print("Motors stopped. Mission log saved to 'mission_log.csv'.")

if __name__ == "__main__":
    main()
