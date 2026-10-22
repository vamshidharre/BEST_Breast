#!/usr/bin/env python3
"""
DD-Boat 11 - High-Precision Autonomous Waypoint Mission (Production V3)
=====================================================================
Fixes & Enhancements:
1. GPS Buffer Flush: Drains all pending serial buffer lines so position is FRESH (0 lag)
2. No-Fix Rejection: Ignores empty/zero coordinate frames
3. Latching Pivot Turn: Smooth in-place rotation with hysteresis (no 180° jitter)
4. Passed-Line Waypoint Validation: Uses vector dot-product perpendicular crossing test
5. Fast Straight Cruise with Proportional Steering
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
    # Load from same directory as this script regardless of current working directory
    calib_file = os.path.join(os.path.dirname(__file__), "calib_boat11.json")
    if os.path.exists(calib_file):
        try:
            with open(calib_file, "r") as f:
                data = json.load(f)
            print("[OK] Loaded calibration from: {}".format(calib_file))
            return np.array(data["invA"]).reshape((3, 3)), np.array(data["xbar"]).reshape((3, 1))
        except Exception as e:
            print("[WARN] Error parsing {}: {}".format(calib_file, e))
    print("[WARN] calib_boat11.json not found! Running with uncalibrated identity.")
    return np.eye(3), np.zeros((3, 1))

def get_heading(imu, invA, xbar):
    raw = np.array(imu.read_mag_raw()).reshape((3, 1))
    calib = invA @ (raw - xbar)
    mx, my, _ = calib.flatten()
    return math.atan2(my, mx)

def read_freshest_gps_fix(gps, max_drain=35):
    """
    Drains all accumulated lines in the serial buffer so we never steer
    on old, backlogged GPS data. Returns (lat, lon) or None.
    """
    fresh_fix = None
    for _ in range(max_drain):
        if hasattr(gps, 'ser') and gps.ser.in_waiting == 0:
            break
        ok, data = gps.read_gll_non_blocking()
        if ok and len(data) > 3:
            # Filter out no-fix sentences (0.0, 0.0)
            if data[0] != 0.0 and data[2] != 0.0:
                fresh_fix = cvt_gll_ddmm_2_dd(data)
    return fresh_fix

def main():
    print("=" * 65)
    print("   DD-BOAT 11 : ZERO-LAG AUTONOMOUS WAYPOINT MISSION (V3) ")
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

    print("Acquiring fresh GPS lock (draining buffer)...")
    cur_x, cur_y = None, None
    while cur_x is None:
        fix = read_freshest_gps_fix(gps)
        if fix is not None:
            lat, lon = fix
            cur_x, cur_y = latlon_to_meters(lat, lon)
            print("[GPS LOCKED] Realtime Start Position: X={:.1f}m, Y={:.1f}m".format(cur_x, cur_y))
        time.sleep(0.15)

    BOX_SIZE = 15.0
    waypoints_metric = [
        (cur_x + BOX_SIZE, cur_y + 0.0),       # WP1: 15m East
        (cur_x + BOX_SIZE, cur_y + BOX_SIZE),  # WP2: 15m North-East
        (cur_x + 0.0,      cur_y + BOX_SIZE),  # WP3: 15m North
        (cur_x + 0.0,      cur_y + 0.0),       # WP4: Home
    ]

    wp_index = 0
    prev_wp_x, prev_wp_y = cur_x, cur_y
    min_dist_to_wp = 999.0
    ARRIVAL_RADIUS = 2.5
    CRUISE_SPEED = 70
    PIVOT_SPEED = 75
    Kp = 28.0

    pivot_latch_dir = 0 # -1 for CCW, +1 for CW, 0 for inactive
    last_valid_gps_time = time.time()
    GPS_TIMEOUT = 3.0 # Cut motors if no GPS fix for 3.0 seconds (Safety Watchdog)

    log_file = "mission_log.csv"
    with open(log_file, "w") as f:
        f.write("time,x,y,target_x,target_y,wp_idx,heading_deg,error_deg,u1,u2\n")

    print("\nMission armed (15m box). Zero lag + GPS watchdog enabled. Press Ctrl+C to stop.\n")

    try:
        while wp_index < len(waypoints_metric):
            now = time.time()
            # 1. Always drain serial buffer to get the freshest live GPS reading
            fix = read_freshest_gps_fix(gps)
            if fix is not None:
                lat, lon = fix
                cur_x, cur_y = latlon_to_meters(lat, lon)
                last_valid_gps_time = now

            # GPS DROPOUT SAFETY WATCHDOG:
            if (now - last_valid_gps_time) > GPS_TIMEOUT:
                ard.send_arduino_cmd_motor(0, 0)
                print("\r[GPS LOST] No fix for > {:.1f}s! Motors cut for safety...     ".format(
                    now - last_valid_gps_time), end="", flush=True)
                time.sleep(0.1)
                continue

            tx, ty = waypoints_metric[wp_index]
            dist = math.hypot(tx - cur_x, ty - cur_y)

            # 2. Prof. Luc Jaulin's perpendicular finish-line crossing condition:
            # Vector (target - prev_wp) dot (current_pos - target) > 0 means boat passed the waypoint line
            leg_dx = tx - prev_wp_x
            leg_dy = ty - prev_wp_y
            passed_line = ((cur_x - tx) * leg_dx + (cur_y - ty) * leg_dy) > 0

            # Waypoint Arrival Check
            if dist <= ARRIVAL_RADIUS or (min_dist_to_wp <= 3.5 and passed_line):
                print("\n" + "=" * 55)
                print(">>> WAYPOINT {} VALIDATED (dist={:.1f}m, passed={})!".format(
                    wp_index + 1, dist, passed_line))
                prev_wp_x, prev_wp_y = tx, ty
                wp_index += 1
                min_dist_to_wp = 999.0
                pivot_latch_dir = 0
                if wp_index >= len(waypoints_metric):
                    break
                print(">>> Advancing to Waypoint {}/{}...".format(wp_index + 1, len(waypoints_metric)))
                print("=" * 55 + "\n")
                tx, ty = waypoints_metric[wp_index]
                ard.send_arduino_cmd_motor(0, 0)
                time.sleep(0.3)
                continue

            if dist < min_dist_to_wp:
                min_dist_to_wp = dist

            # 3. Nautical bearing & heading error
            theta_d = math.atan2(tx - cur_x, ty - cur_y)
            theta = get_heading(imu, invA, xbar)
            e_theta = sawtooth(theta_d - theta)
            e_deg = math.degrees(e_theta)

            # 4. In-Place Pivot Turn with Hysteresis Latching (prevents 180° flip-flop)
            if pivot_latch_dir == 0 and abs(e_deg) > 40.0:
                pivot_latch_dir = 1 if e_theta > 0 else -1
            elif pivot_latch_dir != 0 and abs(e_deg) < 20.0:
                pivot_latch_dir = 0 # Exit pivot mode when aligned within 20°

            if pivot_latch_dir != 0:
                mode = "PIVOT"
                u1 = int(+pivot_latch_dir * PIVOT_SPEED)
                u2 = int(-pivot_latch_dir * PIVOT_SPEED)
            else:
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

            time.sleep(0.08)

        print("\n\nMISSION COMPLETE! Returned to launch home.")

    except KeyboardInterrupt:
        print("\n\nCtrl+C detected - stopping motors.")
    finally:
        ard.send_arduino_cmd_motor(0, 0)
        print("Motors safe (0, 0).")

if __name__ == "__main__":
    main()
