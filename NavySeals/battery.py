#!/usr/bin/env python3
"""
DD-Boat 11 - Battery Status & State-of-Charge (SoC) Monitor
===========================================================
Compatible with Python 3.5.3 (Raspbian Stretch on Boat 11)

Usage:
  python3 battery.py              -> Display current battery percentage & runtime
  python3 battery.py reset        -> Reset battery to 100% (when fresh battery plugged)
  python3 battery.py reset 8.2V   -> Set battery starting level from voltage (e.g. 8.2V or 7.9V)
  python3 battery.py reset 85%    -> Set battery starting level from percentage
  python3 battery.py capacity 5000-> Set battery rating in mAh (default 5000 mAh)
  python3 battery.py -m           -> Live continuous monitor mode (updates every second)
"""

import sys
import os
import json
import time
import subprocess

STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "battery_state.json")

# Default LiPo 2S parameters
DEFAULT_CAPACITY_MAH = 5000.0  # Default rating (can be changed anytime)
I_QUIESCENT_MA = 650.0         # Electronics draw (Raspberry Pi + Arduino + IMU + GPS) ~0.65A
I_CRUISE_MA = 2500.0           # Dual thrusters at cruising speed (cmd ~40-50) ~2.5A

# LiPo 2S Voltage-to-SoC lookup table (Open-circuit / rest voltage)
VOLTAGE_TABLE = [
    (8.40, 100.0),
    (8.30,  95.0),
    (8.20,  90.0),
    (8.10,  85.0),
    (8.00,  80.0), # Course minimum for long missions
    (7.90,  72.0),
    (7.80,  65.0),
    (7.70,  55.0),
    (7.60,  40.0),
    (7.50,  25.0), # Course minimum for short missions
    (7.40,  15.0), # Nominal cutoff
    (7.20,   5.0),
    (7.00,   0.0)  # Empty
]

def voltage_to_soc(v):
    """Convert 2S LiPo voltage to estimated state of charge percentage."""
    if v >= VOLTAGE_TABLE[0][0]:
        return 100.0
    if v <= VOLTAGE_TABLE[-1][0]:
        return 0.0
    for i in range(len(VOLTAGE_TABLE) - 1):
        v_high, soc_high = VOLTAGE_TABLE[i]
        v_low, soc_low = VOLTAGE_TABLE[i+1]
        if v_low <= v <= v_high:
            ratio = (v - v_low) / (v_high - v_low)
            return soc_low + ratio * (soc_high - soc_low)
    return 50.0

def load_state():
    """Load persistent battery state from JSON file."""
    if os.path.exists(STATE_FILE):
        try:
            with open(STATE_FILE, "r") as f:
                data = json.load(f)
                return data
        except Exception:
            pass
    # Default initial state
    return {
        "capacity_mah": DEFAULT_CAPACITY_MAH,
        "soc_percent": 100.0,
        "consumed_mah": 0.0,
        "last_update_ts": time.time(),
        "reset_ts": time.time(),
        "reset_note": "Initial default 100%"
    }

def save_state(state):
    """Save persistent battery state to JSON file."""
    try:
        with open(STATE_FILE, "w") as f:
            json.dump(state, f, indent=2)
    except Exception as e:
        print("Warning: Could not save battery state: {}".format(e))

def check_pi_rail():
    """Check Raspberry Pi hardware power health via vcgencmd."""
    try:
        out = subprocess.check_output(["vcgencmd", "get_throttled"]).decode("utf-8").strip()
        code = int(out.split("=")[-1], 16)
        if code == 0:
            return "HEALTHY (5V rail stable, no undervoltage)"
        flags = []
        if code & 0x1:
            flags.append("UNDERVOLTAGE ACTIVE (<4.63V)")
        if code & 0x10000:
            flags.append("Undervoltage occurred since boot")
        return "WARNING: " + "; ".join(flags)
    except Exception:
        return "N/A (running on PC or vcgencmd unavailable)"

def render_bar(pct, width=20):
    """Render an ASCII-safe progress bar."""
    filled = int(round((pct / 100.0) * width))
    filled = max(0, min(width, filled))
    empty = width - filled
    return "[" + ("#" * filled) + ("-" * empty) + "]"

def display_dashboard(state):
    """Print the clean battery status dashboard."""
    cap = state.get("capacity_mah", DEFAULT_CAPACITY_MAH)
    soc = max(0.0, min(100.0, state.get("soc_percent", 100.0)))
    consumed = state.get("consumed_mah", 0.0)
    remaining_mah = max(0.0, cap - consumed)
    
    # Estimate runtime
    standby_hours = remaining_mah / I_QUIESCENT_MA if I_QUIESCENT_MA > 0 else 0
    cruise_hours = remaining_mah / (I_QUIESCENT_MA + I_CRUISE_MA)
    standby_mins = int(round(standby_hours * 60))
    cruise_mins = int(round(cruise_hours * 60))
    
    # Evaluate mission fitness according to best_ddboat.pdf rules
    if soc >= 75.0:
        mission_status = "READY FOR ALL MISSIONS (Long & Short)"
    elif soc >= 30.0:
        mission_status = "OK FOR SHORT / STANDARD MISSIONS ONLY (Recharge soon)"
    else:
        mission_status = "CRITICAL / RECHARGE REQUIRED (Do NOT launch into lake!)"
        
    bar = render_bar(soc, width=20)
    pi_health = check_pi_rail()
    reset_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(state.get("reset_ts", time.time())))
    
    print("\n" + "=" * 56)
    print("           DD-BOAT 11 - BATTERY LEVEL & STATUS          ")
    print("=" * 56)
    print("  Battery Charge : {} {:5.1f}%".format(bar, soc))
    print("  Remaining      : {:.0f} / {:.0f} mAh (Consumed: {:.0f} mAh)".format(remaining_mah, cap, consumed))
    print("  Est. Runtime   : ~{} min (Lake Cruise @ speed 40)".format(cruise_mins))
    print("                   ~{} min (Standby / Sensors only)".format(standby_mins))
    print("-" * 56)
    print("  Mission Status : {}".format(mission_status))
    print("  Pi 5V Rail     : {}".format(pi_health))
    print("  Last Reset     : {} ({})".format(reset_time_str, state.get("reset_note", "Fresh pack")))
    print("=" * 56)
    print("Commands:")
    print("  python3 battery.py reset        -> Reset to 100% after charging")
    print("  python3 battery.py reset 8.1V   -> Set starting level from voltage")
    print("  python3 battery.py capacity <mA>-> Set battery capacity (e.g. 5000)")
    print("  python3 battery.py -m           -> Continuous live display")
    print("=" * 56 + "\n")

def handle_reset(args):
    """Handle reset command."""
    state = load_state()
    cap = state.get("capacity_mah", DEFAULT_CAPACITY_MAH)
    
    if len(args) == 0:
        # Simple reset to 100%
        state["soc_percent"] = 100.0
        state["consumed_mah"] = 0.0
        state["last_update_ts"] = time.time()
        state["reset_ts"] = time.time()
        state["reset_note"] = "Fresh full pack (100%)"
        save_state(state)
        print("[OK] Battery reset to 100.0% ({} mAh).".format(int(cap)))
        display_dashboard(state)
        return
        
    val_str = args[0].strip().upper()
    if val_str.endswith("V"):
        try:
            volts = float(val_str[:-1])
            soc = voltage_to_soc(volts)
            state["soc_percent"] = soc
            state["consumed_mah"] = cap * (1.0 - (soc / 100.0))
            state["last_update_ts"] = time.time()
            state["reset_ts"] = time.time()
            state["reset_note"] = "Tester measured {:.2f}V ({:.1f}%)".format(volts, soc)
            save_state(state)
            print("[OK] Calibrated from tester: {:.2f}V -> {:.1f}% SoC.".format(volts, soc))
            display_dashboard(state)
        except ValueError:
            print("Error: Invalid voltage format. Use e.g. '8.2V' or '7.9V'")
    elif val_str.endswith("%"):
        try:
            soc = float(val_str[:-1])
            soc = max(0.0, min(100.0, soc))
            state["soc_percent"] = soc
            state["consumed_mah"] = cap * (1.0 - (soc / 100.0))
            state["last_update_ts"] = time.time()
            state["reset_ts"] = time.time()
            state["reset_note"] = "Manual set {:.1f}%".format(soc)
            save_state(state)
            print("[OK] Battery set to {:.1f}%.".format(soc))
            display_dashboard(state)
        except ValueError:
            print("Error: Invalid percent format. Use e.g. '85%'")
    else:
        # Try raw float as percent or voltage
        try:
            val = float(val_str)
            if val <= 10.0:  # Assumed voltage (e.g. 8.1)
                soc = voltage_to_soc(val)
                note = "Tester measured {:.2f}V ({:.1f}%)".format(val, soc)
            else:            # Assumed percentage (e.g. 85)
                soc = max(0.0, min(100.0, val))
                note = "Manual set {:.1f}%".format(soc)
            state["soc_percent"] = soc
            state["consumed_mah"] = cap * (1.0 - (soc / 100.0))
            state["last_update_ts"] = time.time()
            state["reset_ts"] = time.time()
            state["reset_note"] = note
            save_state(state)
            print("[OK] Battery set: {}".format(note))
            display_dashboard(state)
        except ValueError:
            print("Error: Unrecognized value '{}'. Examples: 'reset', 'reset 8.2V', 'reset 90%'".format(val_str))

def handle_capacity(args):
    """Set the battery pack capacity in mAh."""
    if len(args) == 0:
        print("Usage: python3 battery.py capacity <mAh> (e.g. 5000 or 4200)")
        return
    try:
        new_cap = float(args[0])
        state = load_state()
        state["capacity_mah"] = new_cap
        # Recompute consumed based on current SoC
        soc = state.get("soc_percent", 100.0)
        state["consumed_mah"] = new_cap * (1.0 - (soc / 100.0))
        save_state(state)
        print("[OK] Battery capacity updated to {:.0f} mAh.".format(new_cap))
        display_dashboard(state)
    except ValueError:
        print("Error: Invalid capacity number '{}'".format(args[0]))

def run_live_monitor():
    """Continuously refresh the dashboard every 1 second."""
    print("Starting live battery monitor (Press Ctrl+C to stop)...")
    try:
        while True:
            # Re-read state in case a navigation script is updating it
            state = load_state()
            display_dashboard(state)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nMonitor stopped.")

def main():
    if len(sys.argv) == 1:
        state = load_state()
        display_dashboard(state)
        return
        
    cmd = sys.argv[1].lower()
    if cmd in ["reset", "set"]:
        handle_reset(sys.argv[2:])
    elif cmd in ["capacity", "cap"]:
        handle_capacity(sys.argv[2:])
    elif cmd in ["-m", "--monitor", "live"]:
        run_live_monitor()
    else:
        print("Unknown command: {}".format(cmd))
        print("Usage:")
        print("  python3 battery.py              (Show status)")
        print("  python3 battery.py reset        (Reset to 100%)")
        print("  python3 battery.py reset 8.2V   (Calibrate from tester voltage)")
        print("  python3 battery.py capacity <mA>(Set pack mAh rating)")
        print("  python3 battery.py -m           (Continuous live monitor)")

if __name__ == "__main__":
    main()
