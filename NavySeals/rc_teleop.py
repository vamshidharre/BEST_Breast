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
DEADMAN_TIMEOUT  = 1.0   # seconds — motors stop if no key received (WiFi safety)
KEY_HOLD_WINDOW  = 0.15  # seconds — how long a key is considered "held" after last seen
STEER_MIX_RATIO  = 0.65  # 0.0=no steering effect when moving, 1.0=full pivot mix
LOOP_HZ          = 50    # control loop rate (50Hz = 20ms)
MOTOR_CLIP       = 120   # max motor command magnitude

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
║  SAFETY: Motors auto-stop after 1.0s of no key (WiFi safe)  ║
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

    # ── Key state: track timestamp each key was last seen ─────────────────────
    # When you HOLD a key in a terminal, it sends repeated keypress events.
    # We consider a key "held" if it was seen within the last KEY_HOLD_WINDOW seconds.
    key_ts = {
        'fwd': 0.0,   # W / UP
        'bwd': 0.0,   # S / DOWN
        'rgt': 0.0,   # D / RIGHT
        'lft': 0.0,   # A / LEFT
    }

    speed = 60          # base speed (40–120)
    last_any_key = time.time()
    watchdog_active = False
    dt = 1.0 / LOOP_HZ

    # ── Save terminal settings, switch to raw cbreak mode ─────────────────────
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    print(">>> RC READY — Hold keys to drive! Combine W+D for arc turns <<<\n")

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

                # Update key timestamps
                if key == 'UP'   or kl == 'w':   key_ts['fwd'] = now
                elif key == 'DOWN' or kl == 's':  key_ts['bwd'] = now
                elif key == 'RIGHT'or kl == 'd':  key_ts['rgt'] = now
                elif key == 'LEFT' or kl == 'a':  key_ts['lft'] = now

                # Instant stop
                elif key == ' ' or kl == 'x':
                    key_ts = {k: 0.0 for k in key_ts}
                    ard.send_arduino_cmd_motor(0, 0)
                    print("\r[EMERGENCY STOP]                                             ", end='', flush=True)
                    continue

                # Speed control
                elif key == '+' or key == '=':
                    speed = min(120, speed + 10)
                    print("\r[SPEED] {}%   ".format(speed), flush=True)
                    continue
                elif key == '-' or key == '_':
                    speed = max(20, speed - 10)
                    print("\r[SPEED] {}%   ".format(speed), flush=True)
                    continue
                elif kl == '1':
                    speed = 40
                    print("\r[SPEED PRESET] SLOW (40)   ", flush=True)
                    continue
                elif kl == '2':
                    speed = 70
                    print("\r[SPEED PRESET] MEDIUM (70)   ", flush=True)
                    continue
                elif kl == '3':
                    speed = 100
                    print("\r[SPEED PRESET] FAST (100)   ", flush=True)
                    continue

                # Quit
                elif kl == 'q':
                    print("\nQuitting RC Teleop...")
                    break

            # ── Determine which keys are currently "held" ─────────────────────
            # A key is held if it was seen within KEY_HOLD_WINDOW seconds
            fwd_held = (now - key_ts['fwd']) < KEY_HOLD_WINDOW
            bwd_held = (now - key_ts['bwd']) < KEY_HOLD_WINDOW
            rgt_held = (now - key_ts['rgt']) < KEY_HOLD_WINDOW
            lft_held = (now - key_ts['lft']) < KEY_HOLD_WINDOW

            # ── SAFETY WATCHDOG ───────────────────────────────────────────────
            if (now - last_any_key) > DEADMAN_TIMEOUT:
                if not watchdog_active:
                    watchdog_active = True
                    key_ts = {k: 0.0 for k in key_ts}
                    ard.send_arduino_cmd_motor(0, 0)
                    print("\r[WATCHDOG] No key for {:.0f}s — motors stopped (WiFi safe) ".format(
                        DEADMAN_TIMEOUT), end='', flush=True)
                continue

            # ── Compute throttle and steering ─────────────────────────────────
            throttle = 0
            if fwd_held and not bwd_held:
                throttle = +speed
            elif bwd_held and not fwd_held:
                throttle = -speed

            # When moving, reduce steering to give an ARC (not a pivot)
            # When stationary, full steering = pivot spin in place
            steer_power = speed if throttle == 0 else int(speed * STEER_MIX_RATIO)

            steering = 0
            if rgt_held and not lft_held:
                steering = +steer_power
            elif lft_held and not rgt_held:
                steering = -steer_power

            # ── Arcade Drive Mix ──────────────────────────────────────────────
            left, right = arcade_mix(throttle, steering)

            # ── Send to Arduino ───────────────────────────────────────────────
            ard.send_arduino_cmd_motor(left, right)

            # ── Battery update ────────────────────────────────────────────────
            if battery:
                battery.update(left, right)
                batt_str = "{:.1f}%".format(battery.soc_percent)

            # ── Live display ──────────────────────────────────────────────────
            label = get_action_label(throttle, steering)
            print("\r[{:17s}] Spd:{:3d} | L:{:+4d} R:{:+4d} | Batt:{}   ".format(
                label, speed, left, right, batt_str), end='', flush=True)

            time.sleep(0.01)  # small sleep to not hammer the CPU

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
