#!/usr/bin/env python3
"""
DD-Boat 11 - High-Precision Autonomous Waypoint Mission
======================================================
Features:
1. Sharp In-Place Pivot Turns (Tank Turn) at corners when |error| > 35°
2. High-speed straight-line tracking when aligned (|error| <= 35°)
3. Realistic Lake Scale: 15m x 15m rectangular circuit
4. Robust corner passage detection (no infinite overshoots)
5. Continuous logging to mission_log.csv
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
    # Geographic marine compass: 0=N, 90=E, 180=S, 270=W
    return math.atan2(my, mx)

def main():
    print("=" * 65)
    print("   DD-BOAT 11 : SHARP-TURN AUTONOMOUS WAYPOINT MISSION    ")
    print("=" * 65)

    try:
        import gps_driver_v2 as gpddrv
        import imu9_driver_v2 as imudrv
        import arduino_driver_v2 as arddrv
        gps = gpddrv.GpsIO()
        imu = imudrv.Imu9IO()
        ard = arddrv.ArduinoIO()
        gps.set_filter_speed("0")
        ard.send_arduino_cmd_motor(0, 0)
        print("[OK] Drivers connected and motors initialized at 0.")
    except Exception as e:
        print("[ERROR] Driver connection failed: {}".format(e))
        return

    invA, xbar = load_calibration()

    print("Acquiring GPS fix...")
    cur_x, cur_y = None, None
    while cur_x is None:
        ok, data = gps.read_gll_non_blocking()
        if ok:
            lat, lon = cvt_gll_ddmm_2_dd(data)
            cur_x, cur_y = latlon_to_meters(lat, lon)
            print("[GPS LOCKED] Start Position: X={:.1f}m, Y={:.1f}m".format(cur_x, cur_y))
        time.sleep(0.2)

    # 15m x 15m clean rectangular course
    BOX_SIZE = 15.0
    waypoints_metric = [
        (cur_x + BOX_SIZE, cur_y + 0.0),       # WP1: 15m East
        (cur_x + BOX_SIZE, cur_y + BOX_SIZE),  # WP2: 15m North-East
        (cur_x + 0.0,      cur_y + BOX_SIZE),  # WP3: 15m North
        (cur_x + 0.0,      cur_y + 0.0),       # WP4: Back Home
    ]

    wp_index = 0
    min_dist_to_wp = 999.0
    ARRIVAL_RADIUS = 2.2  # meters (tight cornering)
    CRUISE_SPEED = 70     # straight line thrust
    PIVOT_SPEED = 75      # sharp turn thrust (in-place rotation)
    Kp = 28.0

    log_file = "mission_log.csv"
    with open(log_file, "w") as f:
        f.write("time,x,y,target_x,target_y,wp_idx,heading_deg,error_deg,u1,u2\n")

    print("\nMission started with 4 waypoints (15m box). Press Ctrl+C to abort.\n")

    try:
        while wp_index < len(waypoints_metric):
            ok, data = gps.read_gll_non_blocking()
            if ok:
                lat, lon = cvt_gll_ddmm_2_dd(data)
                cur_x, cur_y = latlon_to_meters(lat, lon)

            tx, ty = waypoints_metric[wp_index]
            dist = math.hypot(tx - cur_x, ty - cur_y)

            # Check waypoint arrival or corner clip:
            # Reaches within 2.2m or passes within 3.0m
            if dist <= ARRIVAL_RADIUS or (min_dist_to_wp <= 3.0 and dist > min_dist_to_wp + 0.6):
                print("\n" + "=" * 55)
                print(">>> WAYPOINT {} REACHED (dist={:.1f}m, closest={:.1f}m)!".format(
                    wp_index + 1, dist, min_dist_to_wp))
                wp_index += 1
                min_dist_to_wp = 999.0
                if wp_index >= len(waypoints_metric):
                    break
                print(">>> Turning towards Waypoint {}/{}...".format(wp_index + 1, len(waypoints_metric)))
                print("=" * 55 + "\n")
                tx, ty = waypoints_metric[wp_index]
                # Brief pause at corner to kill forward momentum before turning
                ard.send_arduino_cmd_motor(0, 0)
                time.sleep(0.4)
                continue

            if dist < min_dist_to_wp:
                min_dist_to_wp = dist

            # Nautical desired heading (0=N, 90=E)
            theta_d = math.atan2(tx - cur_x, ty - cur_y)
            theta = get_heading(imu, invA, xbar)
            e_theta = sawtooth(theta_d - theta)
            e_deg = math.degrees(e_theta)

            # -------------------------------------------------------------
            # SHARP TURN CONTROLLER:
            # If heading error is large (> 35°), do an IN-PLACE PIVOT TURN!
            # Zero forward motion: one motor +PIVOT, other -PIVOT.
            # -------------------------------------------------------------
            if abs(e_deg) > 35.0:
                mode = "PIVOT"
                if e_theta > 0:
                    # Turn Clockwise: Left forward, Right reverse
                    u1 = +PIVOT_SPEED
                    u2 = -PIVOT_SPEED
                else:
                    # Turn Counter-Clockwise: Left reverse, Right forward
                    u1 = -PIVOT_SPEED
                    u2 = +PIVOT_SPEED
            else:
                # Aligned with waypoint: CRUISE FORWARD with smooth steering trim
                mode = "CRUISE"
                u1 = int(np.clip(CRUISE_SPEED + Kp * e_theta, 0, 110))
                u2 = int(np.clip(CRUISE_SPEED - Kp * e_theta, 0, 110))

            ard.send_arduino_cmd_motor(u1, u2)

            h_deg = (math.degrees(theta) + 360) % 360
            print("[{:6s}] WP {}/{} | Dist: {:4.1f}m | Hdg: {:5.1f}° | Err: {:+5.1f}° | M: ({:+4d}, {:+4d})".format(
                mode, wp_index + 1, len(waypoints_metric), dist, h_deg, e_deg, u1, u2))

            with open(log_file, "a") as f:
                f.write("{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{},{:.1f},{:.1f},{},{}\n".format(
                    time.time(), cur_x, cur_y, tx, ty, wp_index, h_deg, e_deg, u1, u2))

            time.sleep(0.12)

        print("\n\nMISSION COMPLETE! DD-Boat returned home.")

    except KeyboardInterrupt:
        print("\n\nCtrl+C detected - stopping motors.")
    finally:
        ard.send_arduino_cmd_motor(0, 0)
        print("Motors safe (0, 0).")

if __name__ == "__main__":
    main()
