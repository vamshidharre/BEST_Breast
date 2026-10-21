#!/usr/bin/env python3
"""
DD-Boat 11 - Software Battery State-of-Charge (SoC) & Fuel Gauge Monitor
========================================================================
Compatible with Python 3.5.3 (Raspbian Stretch)

Provides real-time battery tracking via:
1. Coulomb Counting / Energy Integration based on motor thrust & run time
2. Electronics quiescent draw monitoring
3. Raspberry Pi 5V power-sag & undervoltage alarm detection
4. Automatic safety trigger threshold (e.g. for Return-To-Home)
"""

import os
import json
import time
import subprocess

STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "battery_state.json")

class BatteryMonitor:
    def __init__(self, battery_capacity_mah=None, initial_soc_percent=None):
        """
        Loads existing persistent battery state if available, or initializes new pack.
        """
        # Load from disk if available
        disk_state = self._load_state()
        
        if battery_capacity_mah is not None:
            self.capacity_mah = float(battery_capacity_mah)
        else:
            self.capacity_mah = float(disk_state.get("capacity_mah", 5000.0))
            
        if initial_soc_percent is not None:
            self.soc_percent = float(initial_soc_percent)
            self.consumed_mah = self.capacity_mah * (1.0 - (self.soc_percent / 100.0))
        else:
            self.soc_percent = float(disk_state.get("soc_percent", 100.0))
            self.consumed_mah = float(disk_state.get("consumed_mah", 0.0))
        
        # Current draw parameters (approximate physical constants for DD-Boat)
        self.i_quiescent_ma = 650.0   # Raspberry Pi 3B + Arduino Mega + GPS + IMU (~0.65A)
        self.i_motor_max_ma = 6000.0  # Peak current per motor at 100% thrust (~6A per motor)
        
        self.last_update_time = time.time()
        self.last_save_time = time.time()
        self.last_cmdl = 0
        self.last_cmdr = 0
        
        # Safety thresholds
        self.warning_threshold_pct = 30.0   # 30% remaining -> yellow warning
        self.critical_threshold_pct = 15.0  # 15% remaining -> abort / Return to Home

    def _load_state(self):
        if os.path.exists(STATE_FILE):
            try:
                with open(STATE_FILE, "r") as f:
                    return json.load(f)
            except Exception:
                pass
        return {}

    def save_state(self):
        """Write current battery state back to JSON file."""
        state = self._load_state()
        state["capacity_mah"] = self.capacity_mah
        state["soc_percent"] = round(self.soc_percent, 2)
        state["consumed_mah"] = round(self.consumed_mah, 2)
        state["last_update_ts"] = time.time()
        try:
            with open(STATE_FILE, "w") as f:
                json.dump(state, f, indent=2)
        except Exception:
            pass
        
    def update(self, cmd_left=0, cmd_right=0):
        """
        Integrate energy consumed since last call based on commanded motor thrust.
        Call this in your control loop (e.g. at 1 Hz - 10 Hz).
        """
        now = time.time()
        dt_hours = (now - self.last_update_time) / 3600.0
        self.last_update_time = now
        
        self.last_cmdl = cmd_left
        self.last_cmdr = cmd_right
        
        # Estimate current draw:
        # Thrust response is quadratic-to-linear: (abs(cmd) / 255)^1.5
        thrust_ratio_l = min(abs(float(cmd_left)) / 255.0, 1.0)
        thrust_ratio_r = min(abs(float(cmd_right)) / 255.0, 1.0)
        
        i_left_ma = self.i_motor_max_ma * (thrust_ratio_l ** 1.3)
        i_right_ma = self.i_motor_max_ma * (thrust_ratio_r ** 1.3)
        total_current_ma = self.i_quiescent_ma + i_left_ma + i_right_ma
        
        # Integrate mAh consumed
        delta_mah = total_current_ma * dt_hours
        self.consumed_mah += delta_mah
        
        # Clamp SoC between 0% and 100%
        remaining_mah = max(self.capacity_mah - self.consumed_mah, 0.0)
        self.soc_percent = (remaining_mah / self.capacity_mah) * 100.0
        
        # Auto-save to disk every 3 seconds
        if now - self.last_save_time > 3.0:
            self.save_state()
            self.last_save_time = now
            
        return self.soc_percent
    
    def get_status(self):
        """
        Returns a dictionary with comprehensive battery stats.
        """
        remaining_mah = max(self.capacity_mah - self.consumed_mah, 0.0)
        
        # Estimate remaining time at current draw
        thrust_ratio_l = min(abs(float(self.last_cmdl)) / 255.0, 1.0)
        thrust_ratio_r = min(abs(float(self.last_cmdr)) / 255.0, 1.0)
        instant_draw_ma = self.i_quiescent_ma + self.i_motor_max_ma * (thrust_ratio_l**1.3 + thrust_ratio_r**1.3)
        
        if instant_draw_ma > 0:
            remaining_mins = (remaining_mah / instant_draw_ma) * 60.0
        else:
            remaining_mins = 0.0
            
        return {
            "soc_percent": round(self.soc_percent, 1),
            "remaining_mah": round(remaining_mah, 0),
            "consumed_mah": round(self.consumed_mah, 0),
            "draw_ma": round(instant_draw_ma, 0),
            "remaining_minutes": round(remaining_mins, 1),
            "is_warning": self.soc_percent <= self.warning_threshold_pct,
            "is_critical": self.soc_percent <= self.critical_threshold_pct
        }
    
    def check_pi_undervoltage(self):
        """
        Queries Raspberry Pi internal PMIC for hardware 5V dropouts.
        """
        try:
            out = subprocess.check_output(["vcgencmd", "get_throttled"]).decode("utf-8").strip()
            code = int(out.split("=")[-1], 16)
            is_undervolt = bool(code & 0x1 or code & 0x10000)
            return is_undervolt, code
        except Exception:
            return False, 0

    def format_status_string(self):
        """
        Returns a compact one-line status string suitable for logging / console output.
        """
        st = self.get_status()
        alert = ""
        if st["is_critical"]:
            alert = " [CRITICAL BATT!]"
        elif st["is_warning"]:
            alert = " [LOW BATT]"
            
        return "Batt: {:.1f}% ({:.0f}/{} mAh, ~{:.0f}mA, est {:.0f}m left){}".format(
            st["soc_percent"],
            st["remaining_mah"],
            int(self.capacity_mah),
            st["draw_ma"],
            st["remaining_minutes"],
            alert
        )

if __name__ == "__main__":
    mon = BatteryMonitor(5000.0, 100.0)
    print("Testing BatteryMonitor with simulated 30s mission at speed (40, 40)...")
    for sec in range(1, 31):
        mon.update(40, 40)
        time.sleep(0.1) # accelerated test
        if sec % 5 == 0:
            print("[Sim Step {:2d}] {}".format(sec, mon.format_status_string()))
