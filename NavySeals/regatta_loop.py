#!/usr/bin/env python3
"""
DD-Boat 11 - Autonomous Regatta Competition Controller
======================================================
Course: BEST 2026 (Guerledan Lake)
Challenge: Exercise 13 (DD Regatta), Chapter 4 of drones.pdf

Features:
- Relative Gate Positioning: Automatically creates Gate A (15m East) and Gate B (25m NE) from launch spot
- In-Place Pivot 180° Turnaround at gates (|error| > 35°)
- Fast Straight Cruise between gates (|error| <= 35°)
- Automatic Lap & Score tracking
- Outputs both regatta_log.csv and regatta_track.kml for jury inspection
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

# Lake Guerledan Reference
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
    return (x + np.pi) % (2 * np.pi) - np.pi

def load_calibration():
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
    raw = np.array(imu.read_mag_raw()).reshape((3, 1))
    calib = invA @ (raw - xbar)
    mx, my, _ = calib.flatten()
    heading_rad = math.atan2(my, mx)
    heading_deg = (math.degrees(heading_rad) + 360.0) % 360.0
    return heading_rad, heading_deg

def save_kml_track(track_points, filename="regatta_track.kml"):
    coord_str = " ".join(["{:.7f},{:.7f},0.0".format(p[1], p[0]) for p in track_points])
    kml = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2">',
        '  <Document>',
        '    <name>DD-Boat 11 Regatta Run</name>',
        '    <Style id="regattaLine"><LineStyle><color>ff00aa00</color><width>4</width></LineStyle></Style>',
        '    <Placemark>',
        '      <name>Regatta Trajectory</name>',
        '      <styleUrl>#regattaLine</styleUrl>',
        '      <LineString><coordinates>{}</coordinates></LineString>'.format(coord_str),
        '    </Placemark>',
        '  </Document>',
        '</kml>'
    ]
    with open(filename, 'w') as f:
        f.write('\n'.join(kml))
    print("[OK] Saved jury KML track to '{}'.".format(filename))

def main():
    print("=" * 65)
    print("       DD-BOAT 11 : AUTONOMOUS REGATTA RUNNER (5 MIN)        ")
    print("       Exercise 13 (DD Regatta) - Friday Competition         ")
    print("=" * 65)

    # Initialize Hardware
    ard = ArduinoIO()
    imu = Imu9IO()
    gps = GpsIO()
    gps.set_filter_speed("0")
    invA, xbar = load_calibration()

    print("[OK] Hardware connected. Calibration parameters loaded.")

    # Wait for initial GPS fix to establish local reference
    print("Waiting for initial GPS fix before countdown...")
    cur_x, cur_y = None, None
    while cur_x is None:
        ok, gll_pos = gps.read_gll_non_blocking()
        if ok:
            lat, lon = cvt_gll_ddmm_2_dd(gll_pos)
            cur_x, cur_y = latlon_to_meters(lat, lon)
            print("[GPS READY] Start Position: x={:.1f}m, y={:.1f}m".format(cur_x, cur_y))
            break
        time.sleep(0.1)

    # Relative competition gates from current launch position:
    # Gate A is 12m East, Gate B is 28m East-North-East
    GATE_A_METRIC = (cur_x + 10.0, cur_y + 0.0)
    GATE_B_METRIC = (cur_x + 25.0, cur_y + 15.0)

    GATES = [GATE_A_METRIC, GATE_B_METRIC]
    GATE_NAMES = ["GATE A (Green Buoy)", "GATE B (Red Buoy)"]
    current_gate_idx = 0

    R_ARRIVAL = 2.2        # Tighter arrival to round the buoy
    CRUISE_SPEED = 75      # Fast dash between gates
    PIVOT_SPEED = 80       # Sharp 180° turnaround
    Kp = 30.0
    RACE_DURATION = 300.0  # 5 minutes competition window

    log_file = "regatta_log.csv"
    with open(log_file, "w") as f:
        f.write("timestamp,t_elapsed,lat,lon,x,y,target_gate,gate_x,gate_y,dist,heading,score\n")

    track_points = []
    score = 0
    min_dist_to_gate = 999.0

    print("\n>>> REGATTA RACE STARTED! 5-MINUTE COUNTDOWN ENGAGED <<<")
    print("Press Ctrl+C at any time for EMERGENCY STOP.\n")

    t_start = time.time()

    try:
        while (time.time() - t_start) < RACE_DURATION:
            now = time.time()
            t_elapsed = now - t_start
            t_remaining = max(0.0, RACE_DURATION - t_elapsed)

            # 1. Update GPS
            ok, gll_pos = gps.read_gll_non_blocking()
            if ok:
                lat, lon = cvt_gll_ddmm_2_dd(gll_pos)
                cur_x, cur_y = latlon_to_meters(lat, lon)
                track_points.append((lat, lon))

            # 2. Distance and Vector to Active Gate
            target_x, target_y = GATES[current_gate_idx]
            dx = target_x - cur_x
            dy = target_y - cur_y
            dist_to_gate = math.hypot(dx, dy)

            # 3. Gate Crossing Check (inside 2.2m or closest pass <= 3.0m)
            if dist_to_gate <= R_ARRIVAL or (min_dist_to_gate <= 3.0 and dist_to_gate > min_dist_to_gate + 0.6):
                score += 1
                crossed_gate = GATE_NAMES[current_gate_idx]
                current_gate_idx = 1 - current_gate_idx  # Toggle target buoy
                next_gate = GATE_NAMES[current_gate_idx]
                min_dist_to_gate = 999.0
                print("\n" + "*" * 60)
                print(">>> CROSSED {}! SCORE = {} PTS <<<".format(crossed_gate, score))
                print(">>> 180° PIVOT TO: {} <<<".format(next_gate))
                print("*" * 60 + "\n")
                # Momentum cut
                ard.send_arduino_cmd_motor(0, 0)
                time.sleep(0.4)
                target_x, target_y = GATES[current_gate_idx]
                dx = target_x - cur_x
                dy = target_y - cur_y
                dist_to_gate = math.hypot(dx, dy)
                continue

            if dist_to_gate < min_dist_to_gate:
                min_dist_to_gate = dist_to_gate

            # 4. Heading & Steering Law
            theta_desired_rad = math.atan2(dx, dy)  # Marine angle (0=N, +90=E)
            heading_rad, heading_deg = get_calibrated_heading(imu, invA, xbar)
            e_theta = sawtooth(theta_desired_rad - heading_rad)
            e_deg = math.degrees(e_theta)

            # 5. In-Place Pivot (180° turnaround) vs Straight Dash
            if abs(e_deg) > 35.0:
                mode = "PIVOT"
                if e_theta > 0:
                    u1 = +PIVOT_SPEED
                    u2 = -PIVOT_SPEED
                else:
                    u1 = -PIVOT_SPEED
                    u2 = +PIVOT_SPEED
            else:
                mode = "CRUISE"
                u1 = int(np.clip(CRUISE_SPEED + Kp * e_theta, 0, 115))
                u2 = int(np.clip(CRUISE_SPEED - Kp * e_theta, 0, 115))

            ard.send_arduino_cmd_motor(u1, u2)

            # 6. Live Scrolling Telemetry
            print("[{:5.1f}s | {:6s}] Target: {:18s} (Dist: {:4.1f}m) | Score: {:2d} | Hdg: {:5.1f}° | Err: {:+5.1f}°".format(
                t_elapsed, mode, GATE_NAMES[current_gate_idx], dist_to_gate, score, heading_deg, e_deg))

            # 7. Log entry
            with open(log_file, "a") as f:
                f.write("{:.2f},{:.1f},{:.7f},{:.7f},{:.2f},{:.2f},{},{:.2f},{:.2f},{:.2f},{:.1f},{}\n".format(
                    now, t_elapsed, lat, lon, cur_x, cur_y, GATE_NAMES[current_gate_idx], target_x, target_y, dist_to_gate, heading_deg, score))

            time.sleep(0.12)

        print("\n\n" + "=" * 60)
        print(">>> 5-MINUTE RACE TIME EXPIRED! FINAL SCORE: {} POINTS <<<".format(score))
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nManual override / Ctrl+C detected.")
    finally:
        ard.send_arduino_cmd_motor(0, 0)
        print("[OK] Motors shut down. Final telemetry saved to '{}'.".format(log_file))
        if track_points:
            save_kml_track(track_points, "regatta_track.kml")

if __name__ == "__main__":
    main()
