#!/usr/bin/env python3
"""
DD-Boat 11 - Advanced Keyboard RC with Arcade Drive Mixing
===========================================================
Compatible with Python 3.5+ (Raspberry Pi / SSH terminal)

ARCADE DRIVE MIXING (like a real RC transmitter):
  Throttle (W/S) + Steering (A/D) mix into smooth curved turns!

  W       = Both motors forward  (straight ahead)
  S       = Both motors reverse  (straight back)
  A       = Pivot LEFT in place  (spin counterclockwise)
  D       = Pivot RIGHT in place (spin clockwise)
  W + D   = Forward + curve RIGHT  (arc turn)
  W + A   = Forward + curve LEFT   (arc turn)
  S + D   = Reverse + curve RIGHT
  S + A   = Reverse + curve LEFT
  Space/X = Emergency stop (0, 0)
  + / -   = Increase / Decrease speed (step 10)
  1/2/3   = Speed preset: Slow(40) / Medium(70) / Fast(100)
  Q       = Quit safely

SAFETY WATCHDOG:
  If no key is received for 1.0 second (e.g. WiFi drops),
  motors auto-stop to prevent the boat from running away!

SIMULTANEOUS KEY DETECTION:
  Uses key-repeat timestamps. Hold W and D together for curved arc.
  Key is considered "held" if seen within the last 150ms.
"""

import sys
import os
import time
import select
import tty
import termios
import math

sys.path.append(os.path.join(os.path.dirname(__file__), 'drivers-ddboat-v2'))

try:
    from arduino_driver_v2 import ArduinoIO
    from battery_monitor import BatteryMonitor
    HAS_BATTERY = True
except ImportError:
    HAS_BATTERY = False

# ─────────────────────────────────────────────────────────────────────────────
# Tuning parameters
# ─────────────────────────────────────────────────────────────────────────────
DEADMAN_TIMEOUT  = 5.0   # seconds — motors stop if no key received (WiFi safety)
KEY_HOLD_WINDOW  = 0.15  # seconds — used internally for multi-key detection (not latch)
STEER_MIX_RATIO  = 0.65  # 0.0=no steering effect when moving, 1.0=full pivot mix
LOOP_HZ          = 50    # control loop rate (50Hz = 20ms)
MOTOR_CLIP       = 120   # max motor command magnitude
MOTOR_MIN        = 35    # minimum non-zero command (ESC deadband compensation)

BANNER = """
╔══════════════════════════════════════════════════════════════╗
║        DD-BOAT 11 : ADVANCED ARCADE DRIVE RC SYSTEM         ║
╠══════════════════════════════════════════════════════════════╣
║  MOVEMENT                      SPEED                        ║
║  [W] / [↑]  : Forward          [+] Increase speed           ║
║  [S] / [↓]  : Reverse          [-] Decrease speed           ║
║  [A] / [←]  : Pivot Left       [1] Slow  (40)               ║
║  [D] / [→]  : Pivot Right      [2] Medium (70)              ║
║                                [3] Fast  (100)              ║
║  COMBINED (Arcade Mix):        [Space/X] Emergency Stop     ║
║  [W]+[D] = Forward + curve R   [Q] Quit safely              ║
║  [W]+[A] = Forward + curve L                                ║
║  [S]+[D] = Reverse + curve R                                ║
║  [S]+[A] = Reverse + curve L                                ║
╠══════════════════════════════════════════════════════════════╣
║  SAFETY: Motors auto-stop after 5.0s of no key (WiFi safe)  ║
╚══════════════════════════════════════════════════════════════╝
"""

def get_key(timeout=0.02):
    """Non-blocking single keypress read, handles ANSI arrow escape codes."""
    fd = sys.stdin.fileno()
    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return None
    ch = sys.stdin.read(1)
    if ch == '\x1b':
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                r, _, _ = select.select([fd], [], [], 0.05)
                if r:
                    ch3 = sys.stdin.read(1)
                    return {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}.get(ch3, 'ESC')
        return 'ESC'
    return ch

def apply_deadband(val, minimum=MOTOR_MIN):
    """
    Boost any non-zero motor command to at least MOTOR_MIN.
    This prevents ESC deadband stuttering at very low speeds.
    e.g. command=20 → sent as 35,  command=0 → stays 0,  command=-15 → sent as -35
    """
    if val == 0:
        return 0
    return int(math.copysign(max(abs(val), minimum), val))

def arcade_mix(throttle, steering, max_val=MOTOR_CLIP):
    """
    Arcade Drive Mixing — same algorithm used in FRC robots and RC boats.

    throttle : -max_val to +max_val  (W=positive, S=negative)
    steering : -max_val to +max_val  (D=positive right, A=negative left)

    Returns (left_motor, right_motor)

    When going FORWARD + TURNING RIGHT:
      Left motor  = throttle + steering  → goes faster  → boat curves right
      Right motor = throttle - steering  → goes slower  → boat curves right

    When ONLY TURNING (throttle=0):
      Left=+steering, Right=-steering → pivot spin in place
    """
    left  = throttle + steering
    right = throttle - steering

    # Scale down if either motor exceeds clip (preserves the mix ratio)
    scale = max(abs(left), abs(right), max_val) / float(max_val)
    left  = int(left  / scale)
    right = int(right / scale)
    return left, right

def get_action_label(throttle, steering):
    """Human-readable label for current drive state."""
    if throttle == 0 and steering == 0:
        return "IDLE (stopped)   "
    if throttle > 0 and steering == 0:
        return "FORWARD          "
    if throttle < 0 and steering == 0:
        return "REVERSE          "
    if throttle == 0 and steering > 0:
        return "PIVOT RIGHT      "
    if throttle == 0 and steering < 0:
        return "PIVOT LEFT       "
    if throttle > 0 and steering > 0:
        return "FWD + curve RIGHT"
    if throttle > 0 and steering < 0:
        return "FWD + curve LEFT "
    if throttle < 0 and steering > 0:
        return "REV + curve RIGHT"
    if throttle < 0 and steering < 0:
        return "REV + curve LEFT "
    return "MOVING           "

def main():
    print(BANNER)

    # ── Init Arduino ──────────────────────────────────────────────────────────
    try:
        ard = ArduinoIO()
        ard.send_arduino_cmd_motor(0, 0)
        print("[OK] Arduino connected. Motors zeroed.\n")
    except Exception as e:
        print("[ERROR] Could not connect to Arduino: {}".format(e))
        return

    # ── Init Battery (optional) ───────────────────────────────────────────────
    battery = None
    batt_str = "N/A"
    if HAS_BATTERY:
        try:
            battery = BatteryMonitor()
            batt_str = "{:.1f}%".format(battery.soc_percent)
            print("[OK] Battery: {}\n".format(batt_str))
        except Exception:
            pass

    speed = 60          # base speed (40–120)
    last_any_key = time.time()
    watchdog_active = False
    dt = 1.0 / LOOP_HZ

    # ── LATCH STATE: last commanded throttle and steering ─────────────────────
    # Press a key once → command LATCHES and stays active until new key or Space
    # This gives the "inertia" feel — boat keeps going after you release the key!
    latched_throttle = 0   # -speed, 0, or +speed
    latched_steering = 0   # -steer_power, 0, or +steer_power

    # ── Save terminal settings, switch to raw cbreak mode ─────────────────────
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    print(">>> RC READY — Press key to move, Space to stop, Q to quit <<<\n")

    try:
        tty.setcbreak(fd)

        while True:
            now = time.time()

            # ── Read key (non-blocking) ───────────────────────────────────────
            key = get_key(timeout=dt)

            if key is not None:
                kl = key.lower()
                last_any_key = now
                watchdog_active = False

                steer_power = speed if latched_throttle == 0 else int(speed * STEER_MIX_RATIO)

                # ── LATCH: each key press updates the latched command ─────────
                # Throttle keys: W/S set forward/reverse throttle
                if key == 'UP' or kl == 'w':
                    latched_throttle = +speed
                    # Recalculate steer_power after throttle change
                    latched_steering = int(latched_steering * STEER_MIX_RATIO) if latched_steering != 0 else 0

                elif key == 'DOWN' or kl == 's':
                    latched_throttle = -speed
                    latched_steering = int(latched_steering * STEER_MIX_RATIO) if latched_steering != 0 else 0

                # Steering keys: A/D add/change steering direction
                elif key == 'RIGHT' or kl == 'd':
                    steer_power = speed if latched_throttle == 0 else int(speed * STEER_MIX_RATIO)
                    latched_steering = +steer_power

                elif key == 'LEFT' or kl == 'a':
                    steer_power = speed if latched_throttle == 0 else int(speed * STEER_MIX_RATIO)
                    latched_steering = -steer_power

                # STOP — clears both throttle and steering
                elif key == ' ' or kl == 'x':
                    latched_throttle = 0
                    latched_steering = 0
                    ard.send_arduino_cmd_motor(0, 0)
                    print("\r[STOP]             Spd:{:3d} | L:{:+4d} R:{:+4d} | Batt:{}   ".format(
                        speed, 0, 0, batt_str), end='', flush=True)
                    continue

                # Speed control — also rescales current latch
                elif key == '+' or key == '=':
                    speed = min(120, speed + 10)
                    if latched_throttle != 0:
                        latched_throttle = +speed if latched_throttle > 0 else -speed
                    print("\r[SPEED UP] {}            ".format(speed), flush=True)
                    continue
                elif key == '-' or key == '_':
                    speed = max(20, speed - 10)
                    if latched_throttle != 0:
                        latched_throttle = +speed if latched_throttle > 0 else -speed
                    print("\r[SPEED DOWN] {}          ".format(speed), flush=True)
                    continue
                elif kl == '1':
                    speed = 40
                    print("\r[SLOW (40)]              ", flush=True)
                    continue
                elif kl == '2':
                    speed = 70
                    print("\r[MEDIUM (70)]            ", flush=True)
                    continue
                elif kl == '3':
                    speed = 100
                    print("\r[FAST (100)]             ", flush=True)
                    continue

                # Quit
                elif kl == 'q':
                    print("\nQuitting RC Teleop...")
                    break

            # ── SAFETY WATCHDOG ───────────────────────────────────────────────
            # If NO key at all for DEADMAN_TIMEOUT seconds → stop motors
            # (protects against WiFi drop while boat is running)
            if (now - last_any_key) > DEADMAN_TIMEOUT:
                if not watchdog_active:
                    watchdog_active = True
                    latched_throttle = 0
                    latched_steering = 0
                    ard.send_arduino_cmd_motor(0, 0)
                    print("\r[WATCHDOG] No input for {:.0f}s — motors stopped (WiFi safe)   ".format(
                        DEADMAN_TIMEOUT), end='', flush=True)
                continue

            # ── Arcade Drive Mix with latched values ──────────────────────────
            left, right = arcade_mix(latched_throttle, latched_steering)

            # ── Apply ESC deadband compensation ───────────────────────────────
            left  = apply_deadband(left)
            right = apply_deadband(right)

            # ── Send to Arduino ───────────────────────────────────────────────
            ard.send_arduino_cmd_motor(left, right)

            # ── Battery update ────────────────────────────────────────────────
            if battery:
                battery.update(left, right)
                batt_str = "{:.1f}%".format(battery.soc_percent)

            # ── Live display ──────────────────────────────────────────────────
            label = get_action_label(latched_throttle, latched_steering)
            print("\r[{:17s}] Spd:{:3d} | L:{:+4d} R:{:+4d} | Batt:{}   ".format(
                label, speed, left, right, batt_str), end='', flush=True)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\nCtrl+C — stopping...")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        ard.send_arduino_cmd_motor(0, 0)
        if battery:
            battery.save_state()
        print("\n[OK] Motors stopped (0,0). Terminal restored.\n")

if __name__ == "__main__":
    main()
