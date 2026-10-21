#!/usr/bin/env python3
"""
DD-Boat 11 - Propeller Power Test (Page 27 of best_ddboat.pdf)
=============================================================
Tests the 2 brushless motors via Arduino MEGA 2560 and ESC.
Performs an in-place rotation for 2 seconds (left=100, right=-100),
then stops both motors (0, 0).

Requirements before running:
1. Propellers must be clear of hands/cables or safely in water.
2. Remote Control Switch F must be DOWN (Autonomous Mode).
3. ESC startup melody must have finished (beeped 3 times).
"""

import sys
import os
import time

# Access to drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
import arduino_driver_v2 as arddrv

def main():
    print("=" * 55)
    print("  DD-BOAT 11 : PROPELLER MOTOR TEST (Page 27)  ")
    print("=" * 55)
    print("Connecting to Arduino...")
    
    try:
        ard = arddrv.ArduinoIO()
        print("[OK] Arduino connected.")
    except Exception as e:
        print("[ERROR] Failed to connect to Arduino: {}".format(e))
        return

    left_speed = 100
    right_speed = -left_speed

    print("\n[SAFETY CHECK]")
    print("- Ensure propellers are elevated / clear of obstacles.")
    print("- Remote control Switch F should be DOWN (Autonomous).")
    print("\nStarting in-place turn: Left = {}, Right = {} for 2.0s...".format(
        left_speed, right_speed))
    
    ard.send_arduino_cmd_motor(left_speed, right_speed)
    time.sleep(2.0)
    
    print("Stopping motors...")
    ard.send_arduino_cmd_motor(0, 0)
    print("[OK] Motor test complete. Both motors stopped (0, 0).\n")

if __name__ == "__main__":
    main()
