#!/usr/bin/env python3
"""
DD-Boat 11 - Initial Hardware Smoke Test
=========================================
Compatible with Python 3.5+
Tests:
1. Arduino communication & motors (gentle short pulse)
2. IMU (Pololu 9-DOF: Accel, Gyro, Mag)
3. GPS (NMEA GPGLL reading)
"""

import sys
import os
import time
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

print("========================================")
print("     DD-BOAT 11 HARDWARE SMOKE TEST     ")
print("========================================\n")

# 1. IMU TEST
print("[1/3] Testing IMU (Pololu 9-DOF)...")
try:
    import imu9_driver_v2 as imudrv
    imu = imudrv.Imu9IO()
    xacc, yacc, zacc = imu.read_accel_raw()
    xgyr, ygyr, zgyr = imu.read_gyro_raw()
    xmag, ymag, zmag = imu.read_mag_raw()
    
    print("  -> Accel Raw: x={:6d}, y={:6d}, z={:6d}".format(xacc, yacc, zacc))
    print("  -> Gyro  Raw: x={:6d}, y={:6d}, z={:6d}".format(xgyr, ygyr, zgyr))
    print("  -> Mag   Raw: x={:6d}, y={:6d}, z={:6d}".format(xmag, ymag, zmag))
    print("  [OK] IMU is communicating properly.\n")
except Exception as e:
    print("  [ERROR] IMU test failed: {}\n".format(e))

# 2. GPS TEST
print("[2/3] Testing GPS (waiting up to 10s for fix)...")
try:
    import gps_driver_v2 as gpddrv
    gps = gpddrv.GpsIO()
    gps.set_filter_speed("0")
    
    start_time = time.time()
    gll_received = False
    
    while time.time() - start_time < 10.0:
        gll_ok, gll_data = gps.read_gll_non_blocking()
        if gll_ok:
            print("  -> Raw GLL data received: {}".format(gll_data))
            gll_received = True
            break
        time.sleep(0.1)
        
    if gll_received:
        print("  [OK] GPS receiver is active.\n")
    else:
        print("  [WARNING] No GPS GLL sentence received within 10s (move antenna near open sky if indoors).\n")
except Exception as e:
    print("  [ERROR] GPS test failed: {}\n".format(e))

# 3. MOTOR & ARDUINO TEST
print("[3/3] Testing Arduino & Thrusters...")
print("  CAUTION: Ensure propellers are clear of hands/cables or submerged safely!")
try:
    import arduino_driver_v2 as arddrv
    ard = arddrv.ArduinoIO()
    
    print("  -> Testing LEFT motor (speed = 40 for 1.0s)...")
    ard.send_arduino_cmd_motor(40, 0)
    time.sleep(1.0)
    ard.send_arduino_cmd_motor(0, 0)
    time.sleep(0.5)
    
    print("  -> Testing RIGHT motor (speed = 40 for 1.0s)...")
    ard.send_arduino_cmd_motor(0, 40)
    time.sleep(1.0)
    ard.send_arduino_cmd_motor(0, 0)
    
    print("  [OK] Motors and Arduino respond properly.\n")
except Exception as e:
    print("  [ERROR] Motor test failed: {}\n".format(e))

print("========================================")
print("     SMOKE TEST COMPLETED               ")
print("========================================")
