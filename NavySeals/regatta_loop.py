#!/usr/bin/env python3
"""
DD-Boat 11 - Autonomous Regatta Competition Circuit Controller
==============================================================
Official Task: Exercise 13 (DD Regatta), Page 76 of drones.pdf
Prof. Luc Jaulin & Prof. Benoit Zerr

Competition Rules:
- 5-Minute autonomous window.
- Each gate is a line segment. Crossing it in its "+1" direction scores +1,
  crossing it the other way scores -1.

How it works:
- Gate endpoints (absolute lat/lon, decimal degrees) and each gate's +1
  direction come from regatta_gates.json next to this script, or from a path
  given as the first argument. See regatta_gates.example.json.
- For every gate an entry waypoint 4 m before the gate's middle and an exit
  waypoint 4 m after it are generated. The boat loops
  entry A -> exit A -> entry B -> exit B -> entry A ...
- The score counts real crossings of the gate segments by the GPS track.
- Zero-lag GPS drain, no-fix rejection, 3 s GPS dropout watchdog.
- Refuses to start without compass calibration or a valid gate file.
- Logs regatta_log.csv and regatta_track.kml (track + gates) for the jury.

Usage:
    python3 regatta_loop.py [gates.json]
"""

import sys
import os
import time
import math
import json
import signal
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
from arduino_driver_v2 import ArduinoIO
from gps_driver_v2 import GpsIO
from imu9_driver_v2 import Imu9IO

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

REF_LAT = 48.199111
REF_LON = -3.014930
R_EARTH = 6371000.0
LAT_RAD = math.radians(REF_LAT)

GATE_OFFSET_M = 4.0          # entry / exit waypoints this far either side of each gate's middle
MAX_GATE_DISTANCE_M = 200.0  # refuse to race if a gate is further than this from the launch point
MIN_CLEARANCE_M = 1.0        # transit legs must pass at least this far from every gate

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
    """Return (invA, xbar), or None if the calibration file is missing or unreadable."""
    calib_file = os.path.join(SCRIPT_DIR, "calib_boat11.json")
    if not os.path.exists(calib_file):
        print("[ERROR] Compass calibration not found: {}".format(calib_file))
        return None
    try:
        with open(calib_file, "r") as f:
            data = json.load(f)
        invA = np.array(data["invA"], dtype=float).reshape((3, 3))
        xbar = np.array(data["xbar"], dtype=float).reshape((3, 1))
    except Exception as e:
        print("[ERROR] Cannot read compass calibration {}: {}".format(calib_file, e))
        return None
    print("[OK] Loaded calibration from: {}".format(calib_file))
    return invA, xbar

def get_calibrated_heading(imu, invA, xbar):
    raw = np.array(imu.read_mag_raw()).reshape((3, 1))
    calib = invA @ (raw - xbar)
    mx, my, _ = calib.flatten()
    heading_rad = math.atan2(-my, mx)
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

# ---------------------------------------------------------------------------
# Gates
# ---------------------------------------------------------------------------
def load_gates(path):
    """Read the gate file. Returns a list of gate dicts, or None on any problem."""
    if not os.path.exists(path):
        print("[ERROR] Gate file not found: {}".format(path))
        print("        Copy regatta_gates.example.json to regatta_gates.json and enter the real gate coordinates.")
        return None
    try:
        with open(path, "r") as f:
            data = json.load(f)
        gates = []
        for g in data["gates"]:
            gates.append({
                "name": str(g["name"]),
                "p1": (float(g["p1"][0]), float(g["p1"][1])),
                "p2": (float(g["p2"][0]), float(g["p2"][1])),
                "plus_heading_deg": float(g["plus_heading_deg"]),
            })
    except Exception as e:
        print("[ERROR] Cannot read gate file {}: {}".format(path, e))
        return None
    if len(gates) < 2:
        print("[ERROR] Need at least 2 gates in {}, found {}.".format(path, len(gates)))
        return None
    print("[OK] Loaded {} gates from: {}".format(len(gates), path))
    return gates

def build_course(gates, offset):
    """Convert gates to local metres and generate entry/exit waypoints.

    Each gate gets a unit normal n pointing in its +1 crossing direction.
    Returns (course, circuit), or None if a gate definition is unusable.
    circuit is a list of (x, y, label) waypoints in race order.
    """
    course = []
    for g in gates:
        ax, ay = latlon_to_meters(g["p1"][0], g["p1"][1])
        bx, by = latlon_to_meters(g["p2"][0], g["p2"][1])
        gx, gy = bx - ax, by - ay
        length = math.hypot(gx, gy)
        if length < 0.5:
            print("[ERROR] Gate {} is only {:.2f} m long - check its endpoints.".format(g["name"], length))
            return None
        nx, ny = gy / length, -gx / length
        h = math.radians(g["plus_heading_deg"])
        alignment = nx * math.sin(h) + ny * math.cos(h)   # nautical: x = East, y = North
        if abs(alignment) < 0.5:
            print("[ERROR] Gate {}: plus_heading_deg {:.0f} is almost parallel to the gate.".format(
                g["name"], g["plus_heading_deg"]))
            return None
        if alignment < 0:
            nx, ny = -nx, -ny
        mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
        course.append({
            "name": g["name"], "p1": g["p1"], "p2": g["p2"],
            "a": (ax, ay), "b": (bx, by), "n": (nx, ny), "length": length,
            "entry": (mx - offset * nx, my - offset * ny),
            "exit": (mx + offset * nx, my + offset * ny),
        })
    circuit = []
    for gate in course:
        circuit.append((gate["entry"][0], gate["entry"][1], gate["name"] + " entry"))
        circuit.append((gate["exit"][0], gate["exit"][1], gate["name"] + " exit"))
    return course, circuit

def point_segment_distance(p, a, b):
    abx, aby = b[0] - a[0], b[1] - a[1]
    l2 = abx * abx + aby * aby
    t = 0.0
    if l2 > 0:
        t = max(0.0, min(1.0, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / l2))
    return math.hypot(p[0] - (a[0] + t * abx), p[1] - (a[1] + t * aby))

def segment_distance(p, q, a, b):
    """Shortest distance between segments p-q and a-b (0 if they intersect)."""
    def orient(o, s, t):
        return (s[0] - o[0]) * (t[1] - o[1]) - (s[1] - o[1]) * (t[0] - o[0])
    if (orient(a, b, p) > 0) != (orient(a, b, q) > 0) and (orient(p, q, a) > 0) != (orient(p, q, b) > 0):
        return 0.0
    return min(point_segment_distance(p, a, b), point_segment_distance(q, a, b),
               point_segment_distance(a, p, q), point_segment_distance(b, p, q))

def check_course(course, circuit):
    """Every leg except a gate's own entry->exit leg must stay clear of every gate,
    otherwise the loop itself would cross a gate the wrong way. Returns True if OK."""
    ok = True
    n = len(circuit)
    for i in range(n):
        p, q = circuit[i], circuit[(i + 1) % n]
        for k, gate in enumerate(course):
            if i == 2 * k:
                continue    # this is the gate's own crossing leg
            d = segment_distance(p[:2], q[:2], gate["a"], gate["b"])
            if d < MIN_CLEARANCE_M:
                print("[ERROR] Leg '{}' -> '{}' passes {:.1f} m from gate {} (needs {:.1f} m).".format(
                    p[2], q[2], d, gate["name"], MIN_CLEARANCE_M))
                ok = False
    return ok

def gate_crossing(p, q, gate):
    """+1 if the move p->q crosses the gate segment in its +1 direction,
    -1 if it crosses the other way, 0 if it does not cross the segment."""
    ax, ay = gate["a"]
    bx, by = gate["b"]
    nx, ny = gate["n"]
    sp = (p[0] - ax) * nx + (p[1] - ay) * ny   # signed distance to the gate line, + = scoring side
    sq = (q[0] - ax) * nx + (q[1] - ay) * ny
    if (sp < 0) == (sq < 0):
        return 0
    t = sp / (sp - sq)
    cx, cy = p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])
    gx, gy = bx - ax, by - ay
    u = ((cx - ax) * gx + (cy - ay) * gy) / (gx * gx + gy * gy)
    if u < 0.0 or u > 1.0:
        return 0
    return 1 if sq >= 0 else -1

def save_kml_track(track_points, course, filename="regatta_track.kml"):
    coord_str = " ".join(["{:.7f},{:.7f},0.0".format(p[1], p[0]) for p in track_points])
    kml = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2">',
        '  <Document>',
        '    <name>DD-Boat 11 Official Regatta Run</name>',
        '    <Style id="regattaLine"><LineStyle><color>ff00aa00</color><width>4</width></LineStyle></Style>',
        '    <Style id="gateLine"><LineStyle><color>ff00ffff</color><width>6</width></LineStyle></Style>',
        '    <Placemark>',
        '      <name>Regatta Trajectory</name>',
        '      <styleUrl>#regattaLine</styleUrl>',
        '      <LineString><coordinates>{}</coordinates></LineString>'.format(coord_str),
        '    </Placemark>',
    ]
    for gate in course:
        kml += [
            '    <Placemark>',
            '      <name>Gate {}</name>'.format(gate["name"]),
            '      <styleUrl>#gateLine</styleUrl>',
            '      <LineString><coordinates>{:.7f},{:.7f},0.0 {:.7f},{:.7f},0.0</coordinates></LineString>'.format(
                gate["p1"][1], gate["p1"][0], gate["p2"][1], gate["p2"][0]),
            '    </Placemark>',
        ]
    kml += ['  </Document>', '</kml>']
    with open(filename, 'w') as f:
        f.write('\n'.join(kml))
    print("[OK] Saved jury KML track to '{}'.".format(filename))

def main():
    print("=" * 65)
    print("       DD-BOAT 11 : EXERCISE 13 REGATTA COMPETITION LOOP     ")
    print("=" * 65)

    # Everything that can be checked without hardware is checked first.
    calib = load_calibration()
    if calib is None:
        print("[ABORT] Refusing to race without compass calibration. Run calib.py first.")
        sys.exit(1)
    invA, xbar = calib

    gates_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(SCRIPT_DIR, "regatta_gates.json")
    gates = load_gates(gates_file)
    if gates is None:
        print("[ABORT] No valid gate definitions.")
        sys.exit(1)
    built = build_course(gates, GATE_OFFSET_M)
    if built is None:
        print("[ABORT] Gate definitions are not usable.")
        sys.exit(1)
    course, circuit = built
    if not check_course(course, circuit):
        print("[ABORT] This gate layout makes the loop cross a gate the wrong way. Check the gate order and +1 directions.")
        sys.exit(1)

    ard = ArduinoIO()
    imu = Imu9IO()
    gps = GpsIO()
    gps.set_filter_speed("0")
    ard.send_arduino_cmd_motor(0, 0)

    print("Flushing GPS buffer & locking initial position...")
    cur_x, cur_y = None, None
    while cur_x is None:
        fix = read_freshest_gps_fix(gps)
        if fix is not None:
            lat, lon = fix
            cur_x, cur_y = latlon_to_meters(lat, lon)
            print("[GPS READY] Launch Start: x={:.1f}m, y={:.1f}m".format(cur_x, cur_y))
        time.sleep(0.15)

    # Sanity-check the gates against where the boat actually is
    for gate in course:
        mx = (gate["a"][0] + gate["b"][0]) / 2.0
        my = (gate["a"][1] + gate["b"][1]) / 2.0
        d = math.hypot(mx - cur_x, my - cur_y)
        print("    Gate {:<8s} {:5.1f} m long, middle {:6.1f} m from launch".format(gate["name"], gate["length"], d))
        if d > MAX_GATE_DISTANCE_M:
            ard.send_arduino_cmd_motor(0, 0)
            print("[ABORT] Gate {} is {:.0f} m away (limit {:.0f} m). Check the gate coordinates.".format(
                gate["name"], d, MAX_GATE_DISTANCE_M))
            sys.exit(1)
    # Start at the first waypoint the boat can reach without crossing a gate
    # (e.g. launching on the far side of a gate would otherwise score -1).
    start_idx = 0
    for i in range(len(circuit)):
        if all(segment_distance((cur_x, cur_y), circuit[i][:2], g["a"], g["b"]) >= MIN_CLEARANCE_M
               for g in course):
            start_idx = i
            break
    else:
        print("[WARN] Every first leg from the launch point passes a gate. It may score -1 at the start.")
    if start_idx:
        print("[INFO] Starting at '{}' so the first leg does not cross a gate.".format(circuit[start_idx][2]))
    print("Circuit (local metres):")
    for wx, wy, label in circuit:
        print("    {:<16s} x={:7.1f}  y={:7.1f}".format(label, wx, wy))

    wp_idx = start_idx
    prev_x, prev_y = cur_x, cur_y
    min_dist = 999.0
    ARRIVAL_RADIUS = 3.5
    CRUISE_SPEED = 85
    PIVOT_SPEED = 85
    Kp = 35.0
    RACE_DURATION = 300.0 # 5 minutes

    pivot_latch_dir = 0
    score = 0
    laps = 0
    last_valid_gps_time = time.time()
    last_dropout_msg = 0.0
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
                new_x, new_y = latlon_to_meters(lat, lon)
                # Score real crossings of the gate segments by the GPS track
                for gate in course:
                    s = gate_crossing((cur_x, cur_y), (new_x, new_y), gate)
                    if s:
                        score += s
                        print("\n*** {} GATE {}: {:+d} -> TOTAL = {} PTS ***\n".format(
                            "CROSSED" if s > 0 else "WRONG-WAY CROSSING OF", gate["name"], s, score))
                cur_x, cur_y = new_x, new_y
                track_points.append((lat, lon))
                last_valid_gps_time = now

            # GPS Dropout Safety Watchdog
            if (now - last_valid_gps_time) > GPS_TIMEOUT:
                ard.send_arduino_cmd_motor(0, 0)
                if now - last_dropout_msg >= 1.0:
                    print("[GPS DROPOUT] No fix for {:.1f}s - motors cut, waiting for GPS...".format(
                        now - last_valid_gps_time))
                    last_dropout_msg = now
                time.sleep(0.1)
                continue

            tx, ty, label = circuit[wp_idx]
            dist = math.hypot(tx - cur_x, ty - cur_y)

            # Perpendicular finish line crossing detection
            leg_dx = tx - prev_x
            leg_dy = ty - prev_y
            passed = ((cur_x - tx) * leg_dx + (cur_y - ty) * leg_dy) > 0

            if dist <= ARRIVAL_RADIUS or (min_dist <= 3.5 and passed):
                prev_x, prev_y = tx, ty
                wp_idx = (wp_idx + 1) % len(circuit)
                if wp_idx == 0:
                    laps += 1
                    print("\n=== LAP {} COMPLETED! SCORE: {} PTS ===\n".format(laps, score))
                min_dist = 999.0
                pivot_latch_dir = 0
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
                u1 = int(+pivot_latch_dir * PIVOT_SPEED)
                u2 = int(-pivot_latch_dir * PIVOT_SPEED)
            else:
                u1 = int(np.clip(CRUISE_SPEED + Kp * e_theta, 0, 115))
                u2 = int(np.clip(CRUISE_SPEED - Kp * e_theta, 0, 115))

            ard.send_arduino_cmd_motor(u1, u2)

            print("[{:5.1f}s | Rem:{:3.0f}s] -> {:<12s} | Dist:{:4.1f}m | Score:{:2d} | Laps:{} | Hdg:{:5.1f}° | Err:{:+5.1f}°".format(
                t_elapsed, t_rem, label, dist, score, laps, heading_deg, e_deg))

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
            save_kml_track(track_points, course, "regatta_track.kml")

if __name__ == "__main__":
    main()
