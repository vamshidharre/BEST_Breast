#!/usr/bin/env python3
"""
DD-Boat 11 - Battery & Hardware Telemetry Probe
================================================
Compatible with Python 3.5.3 (Raspbian Stretch)

This script inspects all possible hardware sources for battery voltage:
1. Arduino Mega status ("S;") and energy saver ("E;") responses
2. Raspberry Pi system throttling & power rail status (vcgencmd)
3. I2C Bus 1 address scan to detect any potential onboard ADC / fuel gauge
"""

import sys
import os
import subprocess
import time

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

print("==================================================")
print("     DD-BOAT 11 BATTERY & TELEMETRY PROBE         ")
print("==================================================\n")

# ----------------------------------------------------------------------
# 1. Check Raspberry Pi 5V Rail & Throttling Status
# ----------------------------------------------------------------------
print("[1/3] Checking Raspberry Pi Power Rail & Throttle Flags...")
try:
    cmd_out = subprocess.check_output(["vcgencmd", "get_throttled"]).decode("utf-8").strip()
    # Format is throttled=0x0
    code_str = cmd_out.split("=")[-1]
    code = int(code_str, 16)
    
    print("  -> Raw throttle register: {}".format(cmd_out))
    if code == 0:
        print("  -> [OK] No undervoltage detected. Pi 5V power supply is healthy.")
    else:
        flags = []
        if code & 0x1:
            flags.append("Under-voltage currently detected (<4.63V on Pi rail)")
        if code & 0x2:
            flags.append("ARM frequency currently capped")
        if code & 0x4:
            flags.append("Currently throttled")
        if code & 0x10000:
            flags.append("Under-voltage has previously occurred since boot")
        if code & 0x20000:
            flags.append("Frequency capping has previously occurred")
        if code & 0x40000:
            flags.append("Throttling has previously occurred")
        print("  -> [WARNING] Power issues logged: {}".format("; ".join(flags)))
except Exception as e:
    print("  -> Could not query vcgencmd (running on non-Pi or missing binary): {}".format(e))

print("")

# ----------------------------------------------------------------------
# 2. Check Arduino Serial Interface (Status, Energy Saver, Raw Responses)
# ----------------------------------------------------------------------
print("[2/3] Checking Arduino MEGA 2560 for Telemetry / Voltage Strings...")
try:
    import arduino_driver_v2 as arddrv
    ard = arddrv.ArduinoIO(calibrate=False)
    
    status_str = ard.get_arduino_status(timeout=1.5)
    print("  -> Arduino Status ('S;'): '{}'".format(status_str))
    
    ensv_str = ard.get_arduino_energy_saver(timeout=1.5)
    print("  -> Arduino Energy Saver ('E;'): '{}'".format(ensv_str))
    
    raw_rc = ard.get_arduino_raw_rc_chan(timeout=1.5)
    print("  -> Arduino Raw RC ('X;'): '{}'".format(raw_rc))
    
    # Check if any raw lines contain digits, volts, or ADC readings
    print("  [OK] Arduino queries executed successfully.")
except Exception as e:
    print("  -> [ERROR] Arduino probe failed or not connected: {}".format(e))

print("")

# ----------------------------------------------------------------------
# 3. Check I2C Bus for Battery Fuel Gauge / ADC Chips
# ----------------------------------------------------------------------
print("[3/3] Scanning I2C Bus 1 for Active Devices...")
try:
    i2c_out = subprocess.check_output(["i2cdetect", "-y", "1"]).decode("utf-8")
    print("  -> Active I2C map:")
    for line in i2c_out.splitlines():
        print("     " + line)
    print("\n  Known standard DD-Boat devices:")
    print("    - 0x1e: ST LIS3MDL Magnetometer (IMU)")
    print("    - 0x6b: ST LSM6DS33 Accel/Gyro (IMU)")
    print("    - 0x48 / 0x4d: Microchip TC74 Temperature sensors (if equipped)")
    print("  (Any 0x40/0x48 ADS1115 or INA219 indicates a dedicated voltage ADC).")
except Exception as e:
    print("  -> Could not run i2cdetect: {}".format(e))

print("\n==================================================")
print("     PROBE COMPLETE                               ")
print("==================================================")
