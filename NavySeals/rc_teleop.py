#!/usr/bin/env python3
"""
DD-Boat 11 - Keyboard RC Teleoperation Controller
=================================================
Compatible with Python 3.5+ (Raspberry Pi & Raspbian Stretch)

Turns your laptop keyboard into a wireless RC transmitter over SSH!
Control DD-Boat 11 in real-time using Arrow Keys or WASD.

Controls:
    [▲] Up Arrow    / [W] : Forward
    [▼] Down Arrow  / [S] : Reverse / Backward
    [◄] Left Arrow  / [A] : Turn Left in-place
    [►] Right Arrow / [D] : Turn Right in-place
    [Spacebar]      / [X] : Immediate All-Stop (0, 0)
    [+] / [-]             : Increase / Decrease speed power (step of 10)
    [Q] / Ctrl+C          : Exit & shutdown motors

Safety Watchdog (Deadman Switch):
    If no key is pressed within 0.5 seconds, motors automatically drop to (0, 0)
    to prevent the boat from running away if Wi-Fi drops!
"""

import sys
import os
import time
import select
import tty
import termios

# Access drivers
sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))
from arduino_driver_v2 import ArduinoIO
from battery_monitor import BatteryMonitor

BANNER = """
==============================================================
       DD-BOAT 11 : KEYBOARD RC TELEOPERATION SYSTEM
==============================================================
 Controls:
    [▲] Up / [W]    : Forward          [+] : Increase Speed
    [▼] Down / [S]  : Reverse          [-] : Decrease Speed
    [◄] Left / [A]  : Turn Left        [Space] : Stop (0, 0)
    [►] Right / [D] : Turn Right       [Q] : Quit
--------------------------------------------------------------
 SAFETY WATCHDOG: Motors auto-stop after 0.5s of no key input.
==============================================================
"""

def get_key(timeout=0.05):
    """Reads a single keypress or ANSI escape sequence non-blockingly."""
    fd = sys.stdin.fileno()
    rlist, _, _ = select.select([fd], [], [], timeout)
    if not rlist:
        return None
    
    ch = sys.stdin.read(1)
    if ch == '\x1b':  # Start of ANSI escape code (Arrow keys)
        # Check if more characters follow
        rlist, _, _ = select.select([fd], [], [], 0.05)
        if rlist:
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                rlist, _, _ = select.select([fd], [], [], 0.05)
                if rlist:
                    ch3 = sys.stdin.read(1)
                    if ch3 == 'A':
                        return 'UP'
                    elif ch3 == 'B':
                        return 'DOWN'
                    elif ch3 == 'C':
                        return 'RIGHT'
                    elif ch3 == 'D':
                        return 'LEFT'
        return 'ESC'
    return ch

def main():
    print(BANNER)

    # Initialize Arduino
    try:
        ard = ArduinoIO()
        ard.send_arduino_cmd_motor(0, 0)
        print("[OK] Connected to Arduino. Motors armed at 0, 0.\n")
    except Exception as e:
        print("[ERROR] Could not connect to Arduino: {}".format(e))
        return

    # Initialize Battery Monitor
    battery = BatteryMonitor()
    print("[OK] Battery Monitor initialized. Current charge: {:.1f}%\n".format(battery.soc_percent))

    # Base speed configuration
    speed = 50
    min_speed = 20
    max_speed = 120

    # Deadman watchdog timeout (seconds)
    DEADMAN_TIMEOUT = 0.5
    last_cmd_time = time.time()
    last_batt_time = time.time()
    current_cmd = (0, 0)
    current_action = "IDLE (Stopped)"

    # Save terminal settings for raw character input
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    print(">>> RC READY: Press Arrow Keys or WASD to drive! <<<\n")

    try:
        # Switch terminal to non-canonical / cbreak mode
        tty.setcbreak(fd)

        while True:
            key = get_key(timeout=0.08)
            now = time.time()

            # Update battery consumption model
            battery.update(current_cmd[0], current_cmd[1])

            new_cmd = None
            action = None

            if key is not None:
                key_lower = key.lower()

                # FORWARD
                if key == 'UP' or key_lower == 'w':
                    new_cmd = (speed, speed)
                    action = "FORWARD"
                # REVERSE
                elif key == 'DOWN' or key_lower == 's':
                    new_cmd = (-speed, -speed)
                    action = "REVERSE"
                # TURN LEFT
                elif key == 'LEFT' or key_lower == 'a':
                    new_cmd = (-speed, speed)
                    action = "PIVOT LEFT"
                # TURN RIGHT
                elif key == 'RIGHT' or key_lower == 'd':
                    new_cmd = (speed, -speed)
                    action = "PIVOT RIGHT"
                # STOP
                elif key == ' ' or key_lower == 'x':
                    new_cmd = (0, 0)
                    action = "BRAKE (Stop)"
                # SPEED UP
                elif key == '+' or key == '=':
                    speed = min(max_speed, speed + 10)
                    print("\r[SPEED] Power increased to {}%   ".format(speed))
                # SPEED DOWN
                elif key == '-' or key == '_':
                    speed = max(min_speed, speed - 10)
                    print("\r[SPEED] Power decreased to {}%   ".format(speed))
                # QUIT
                elif key_lower == 'q':
                    print("\nExiting RC Teleop...")
                    break

            # If a new movement command was triggered
            if new_cmd is not None:
                current_cmd = new_cmd
                current_action = action
                last_cmd_time = now
                ard.send_arduino_cmd_motor(current_cmd[0], current_cmd[1])
                print("\r[RC ACTIVE] {:12s} | Speed: {:3d}% | Motors: L={:+4d}, R={:+4d} | Batt: {:4.1f}%   ".format(
                    current_action, speed, current_cmd[0], current_cmd[1], battery.soc_percent), end="", flush=True)

            # WATCHDOG CHECK: If user stopped pressing keys for > DEADMAN_TIMEOUT, stop motors!
            elif current_cmd != (0, 0) and (now - last_cmd_time) > DEADMAN_TIMEOUT:
                current_cmd = (0, 0)
                current_action = "IDLE (Watchdog Stop)"
                ard.send_arduino_cmd_motor(0, 0)
                print("\r[RC IDLE]   {:12s} | Motors: (   0,    0) | Batt: {:4.1f}%        ".format(
                    current_action, battery.soc_percent), end="", flush=True)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\nCtrl+C detected. Stopping...")
    finally:
        # Restore terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # Always shut down motors on exit
        ard.send_arduino_cmd_motor(0, 0)
        battery.save_state()
        print("\n[OK] Motors shut down (0, 0). Battery state saved ({:.1f}%). Terminal restored.\n".format(battery.soc_percent))

if __name__ == "__main__":
    main()
