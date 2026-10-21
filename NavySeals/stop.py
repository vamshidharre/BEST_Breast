#!/usr/bin/env python3
"""
DD-Boat 11 - Instant Emergency Stop
====================================
Immediately commands both thrusters to 0 and shuts down propulsion.
Usage on boat:
    python3 stop.py
"""

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

try:
    import arduino_driver_v2 as arddrv
    ard = arddrv.ArduinoIO()
    ard.send_arduino_cmd_motor(0, 0)
    print(">> MOTORS STOPPED (0, 0) <<")
except Exception as e:
    print("Error communicating with Arduino:", e)
