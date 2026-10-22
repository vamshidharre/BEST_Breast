#!/usr/bin/env python3
"""
DD-Boat 11 - Autonomous Regatta Competition Circuit Controller
==============================================================
Official Task: Exercise 13 (DD Regatta), Page 76 of drones.pdf
Prof. Luc Jaulin & Prof. Benoit Zerr

Competition Rules:
- 5-Minute autonomous window.
- Green Gate (Eastbound crossing = +1 pt).
- Red Gate (Westbound crossing = +1 pt).
- Wrong direction crossing = -1 pt penalty!
- Winning Strategy: Counter-Clockwise Loop through both gates continuously!

Features:
- Zero-lag GPS buffer flushing (reads freshest live fix every iteration).
- No-fix coordinate rejection.
- Smooth heading control with in-place pivot turns for sharp corners.
- Logs regatta_log.csv and regatta_track.kml for the jury.
"""

import sys
import os
import time
import math
import json
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
from arduino_driver_v2 import ArduinoIO
from gps_driver_v2 import GpsIO
from imu9_driver_v2 import Imu9IO

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
    calib_file = os.path.join(os.path.dirname(__file__), "calib_boat11.json")
    if os.path.exists(calib_file):
        try:
            with open(calib_file, "r") as f:
                data = json.load(f)
            print("[OK] Loaded calibration from: {}".format(calib_file))
            return np.array(data["invA"]).reshape((3, 3)), np.array(data["xbar"]).reshape((3, 1))
        except Exception as e:
            print("[WARN] Error loading {}: {}".format(calib_file, e))
    print("[WARN] Running with uncalibrated identity.")
    return np.eye(3), np.zeros((3, 1))

def get_calibrated_heading(imu, invA, xbar):
    raw = np.array(imu.read_mag_raw()).reshape((3, 1))
    calib = invA @ (raw - xbar)
    mx, my, _ = calib.flatten()
    heading_rad = math.atan2(my, mx)
    heading_deg = (math.degrees(heading_rad) + 360.0) % 360.0
    return heading_rad, heading_deg

def read_freshest_gps_fix(gps, max_drain=35):
    fresh_fix = None
    for _ in range(max_drain):
        if hasattr(gps, 'ser') and gps.ser.in_waiting == 0:
            break
        ok, data = gps.read_gll_non_blocking()
        if ok and len(data) > 3:
            if data[0] != 0.0 and data[2] != 0.0:
                fresh_fix = cvt_gll_ddmm_2_dd(data)
    return fresh_fix

def save_kml_track(track_points, filename="regatta_track.kml"):
    coord_str = " ".join(["{:.7f},{:.7f},0.0".format(p[1], p[0]) for p in track_points])
    kml = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2">',
        '  <Document>',
        '    <name>DD-Boat 11 Official Regatta Run</name>',
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
    print("       DD-BOAT 11 : EXERCISE 13 REGATTA COMPETITION LOOP     ")
    print("       (Counter-Clockwise Non-Stop Gate Scoring Loop)        ")
    print("=" * 65)

    ard = ArduinoIO()
    imu = Imu9IO()
    gps = GpsIO()
    gps.set_filter_speed("0")
    invA, xbar = load_calibration()

    print("Flushing GPS buffer & locking initial position...")
    cur_x, cur_y = None, None
    while cur_x is None:
        fix = read_freshest_gps_fix(gps)
        if fix is not None:
            lat, lon = fix
            cur_x, cur_y = latlon_to_meters(lat, lon)
            print("[GPS READY] Launch Start: x={:.1f}m, y={:.1f}m".format(cur_x, cur_y))
        time.sleep(0.15)

    # -------------------------------------------------------------
    # Exercise 13 Circuit (Counter-Clockwise Scoring Loop):
    # Generates a clean 16m x 15m rectangular corridor around the gates:
    # Point 1 -> Cross Green Gate Eastbound (+1 PT) -> Point 3 ->
    # Point 4 -> Cross Red Gate Westbound (+1 PT) -> Point 6 -> loop!
    # -------------------------------------------------------------
    circuit = [
        (cur_x + 2.0,  cur_y + 2.5),   # Point 1: 4m before Green Gate entry
        (cur_x + 10.0, cur_y + 2.5),   # Point 2: 4m clear past Green Gate (+1 PT)
        (cur_x + 14.0, cur_y + 8.5),   # Point 3: Round North-East Corner
        (cur_x + 10.0, cur_y + 14.5),  # Point 4: 4m before Red Gate entry
        (cur_x + 2.0,  cur_y + 14.5),  # Point 5: 4m clear past Red Gate (+1 PT)
        (cur_x - 2.0,  cur_y + 8.5),   # Point 6: Round South-West Corner back to start
    ]

    wp_idx = 0
    prev_x, prev_y = cur_x, cur_y
    min_dist = 999.0
    ARRIVAL_RADIUS = 2.5
    CRUISE_SPEED = 75
    PIVOT_SPEED = 80
    Kp = 30.0
    RACE_DURATION = 300.0 # 5 minutes

    pivot_latch_dir = 0
    score = 0
    laps = 0
    last_valid_gps_time = time.time()
    GPS_TIMEOUT = 3.0 # Cut motors if GPS lost for 3.0 seconds

    log_file = "regatta_log.csv"
    with open(log_file, "w") as f:
        f.write("time,x,y,tx,ty,wp_idx,score,laps,heading,error,u1,u2\n")

    track_points = []
    t_start = time.time()
    print("\n>>> REGATTA RACE STARTED! 5-MINUTE COMPETITION COUNTDOWN ENGAGED <<<\n")

    try:
        while (time.time() - t_start) < RACE_DURATION:
            now = time.time()
            t_elapsed = now - t_start
            t_rem = max(0.0, RACE_DURATION - t_elapsed)

            fix = read_freshest_gps_fix(gps)
            if fix is not None:
                lat, lon = fix
                cur_x, cur_y = latlon_to_meters(lat, lon)
                track_points.append((lat, lon))
                last_valid_gps_time = now

            # GPS Dropout Safety Watchdog
            if (now - last_valid_gps_time) > GPS_TIMEOUT:
                ard.send_arduino_cmd_motor(0, 0)
                print("\r[GPS DROPOUT] No satellites for > {:.1f}s! Motors cut...       ".format(
                    now - last_valid_gps_time), end="", flush=True)
                time.sleep(0.1)
                continue

            tx, ty = circuit[wp_idx]
            dist = math.hypot(tx - cur_x, ty - cur_y)

            # Perpendicular finish line crossing detection
            leg_dx = tx - prev_x
            leg_dy = ty - prev_y
            passed = ((cur_x - tx) * leg_dx + (cur_y - ty) * leg_dy) > 0

            if dist <= ARRIVAL_RADIUS or (min_dist <= 3.5 and passed):
                # Scoring check: Point 2 is Green Gate, Point 5 is Red Gate
                if wp_idx == 1:
                    score += 1
                    print("\n*** SCORED GREEN GATE EASTBOUND! TOTAL = {} PTS ***\n".format(score))
                elif wp_idx == 4:
                    score += 1
                    print("\n*** SCORED RED GATE WESTBOUND! TOTAL = {} PTS ***\n".format(score))

                prev_x, prev_y = tx, ty
                wp_idx = (wp_idx + 1) % len(circuit)
                if wp_idx == 0:
                    laps += 1
                    print("\n=== LAP {} COMPLETED! SCORE: {} PTS ===\n".format(laps, score))

                min_dist = 999.0
                pivot_latch_dir = 0
                tx, ty = circuit[wp_idx]
                continue

            if dist < min_dist:
                min_dist = dist

            theta_d = math.atan2(tx - cur_x, ty - cur_y)
            heading_rad, heading_deg = get_calibrated_heading(imu, invA, xbar)
            e_theta = sawtooth(theta_d - heading_rad)
            e_deg = math.degrees(e_theta)

            # Pivot with hysteresis vs cruise
            if pivot_latch_dir == 0 and abs(e_deg) > 40.0:
                pivot_latch_dir = 1 if e_theta > 0 else -1
            elif pivot_latch_dir != 0 and abs(e_deg) < 20.0:
                pivot_latch_dir = 0

            if pivot_latch_dir != 0:
                mode = "PIVOT"
                u1 = int(+pivot_latch_dir * PIVOT_SPEED)
                u2 = int(-pivot_latch_dir * PIVOT_SPEED)
            else:
                mode = "CRUISE"
                u1 = int(np.clip(CRUISE_SPEED + Kp * e_theta, 0, 115))
                u2 = int(np.clip(CRUISE_SPEED - Kp * e_theta, 0, 115))

            ard.send_arduino_cmd_motor(u1, u2)

            print("[{:5.1f}s | Rem:{:3.0f}s] WP:{}/{} | Dist:{:4.1f}m | Score:{:2d} | Laps:{} | Hdg:{:5.1f}° | Err:{:+5.1f}°".format(
                t_elapsed, t_rem, wp_idx + 1, len(circuit), dist, score, laps, heading_deg, e_deg))

            with open(log_file, "a") as f:
                f.write("{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{},{},{},{:.1f},{:.1f},{},{}\n".format(
                    time.time(), cur_x, cur_y, tx, ty, wp_idx, score, laps, heading_deg, e_deg, u1, u2))

            time.sleep(0.08)

        print("\n\n>>> RACE FINISHED! TOTAL SCORE: {} POINTS ({} LAPS) <<<".format(score, laps))

    except KeyboardInterrupt:
        print("\n\nManual override / Ctrl+C detected.")
    finally:
        ard.send_arduino_cmd_motor(0, 0)
        print("Motors stopped safely (0,0).")
        if track_points:
            save_kml_track(track_points, "regatta_track.kml")

if __name__ == "__main__":
    main()
