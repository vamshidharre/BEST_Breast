#!/usr/bin/env python3
"""
DD-Boat 11 - Instant Live Compass Cross-Check (No prompts, just live reading)
=============================================================================
Usage:
    python3 check_compass.py
"""

import os
import sys
import time
import json
import math
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import imu9_driver_v2 as imudrv

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
calib_file = os.path.join(SCRIPT_DIR, "calib_boat11.json")

if not os.path.exists(calib_file):
    print("[ERROR] Cannot find calibration file: %s" % calib_file)
    sys.exit(1)

with open(calib_file) as f:
    d = json.load(f)

xbar = np.array(d["xbar"]).reshape((3, 1))
invA = np.array(d["invA"]).reshape((3, 3))

imu = imudrv.Imu9IO()
dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]

print("=" * 65)
print("     DD-BOAT 11 : LIVE CALIBRATED COMPASS CHECK")
print("=" * 65)
print("Comparing against your phone compass:")
print("  North = 0°,  East = 90°,  South = 180°,  West = 270°")
print("Press Ctrl+C to stop.\n")

sample = 0
try:
    while True:
        sample += 1
        raw = np.array(imu.read_mag_raw()).reshape((3, 1))
        calib = invA @ (raw - xbar)
        mx, my, _ = calib.flatten()
        heading_rad = math.atan2(-my, mx)
        heading_deg = (math.degrees(heading_rad) + 360.0) % 360.0
        rose = dirs[int((heading_deg + 11.25) / 22.5) % 16]
        print("[#%04d] Heading: %5.1f° [%-3s] | Calibrated: mx=%+.3f, my=%+.3f" % (
            sample, heading_deg, rose, mx, my))
        time.sleep(0.2)
except KeyboardInterrupt:
    print("\n\nFinished compass check.")
