#!/usr/bin/env python3
"""
DD-Boat 11 - Auto KML & Telemetry Downloader
============================================
Waits for DD-Boat 11 (172.20.25.211) to reconnect to Wi-Fi,
then immediately downloads mission_track.kml and mission_log.csv via SCP.
"""

import time
import subprocess
import os
import sys

BOAT_IP = "172.20.25.211"
BOAT_USER = "ue32"
DEST_DIR = r"d:\Personal_Projects\BEST_Breast\NavySeals"
ROOT_DIR = r"d:\Personal_Projects\BEST_Breast"

print(f"[*] Watching for DD-Boat 11 ({BOAT_IP}) to come online...")

# Loop until boat responds to ping
connected = False
while not connected:
    res = subprocess.run(
        ["powershell", "-Command", f"Test-Connection -ComputerName {BOAT_IP} -Count 1 -Quiet"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    if "True" in res.stdout:
        connected = True
        print(f"[+] DD-Boat 11 is ONLINE! ({time.strftime('%H:%M:%S')})")
        break
    time.sleep(2)

# Wait 2 seconds for SSH service to stabilize
time.sleep(2)

print("[*] Downloading mission_track.kml and mission_log.csv from boat...")

target_kml = os.path.join(DEST_DIR, "mission_track.kml")
target_csv = os.path.join(DEST_DIR, "mission_log.csv")
root_kml = os.path.join(ROOT_DIR, "mission_track.kml")

# Download KML
r_kml = subprocess.run(
    ["scp", "-o", "ConnectTimeout=5", f"{BOAT_USER}@{BOAT_IP}:~/NavySeals/mission_track.kml", target_kml],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True
)

# Download CSV
r_csv = subprocess.run(
    ["scp", "-o", "ConnectTimeout=5", f"{BOAT_USER}@{BOAT_IP}:~/NavySeals/mission_log.csv", target_csv],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True
)

if os.path.exists(target_kml):
    # Copy to root workspace as well for easy access
    import shutil
    shutil.copy2(target_kml, root_kml)
    size = os.path.getsize(root_kml)
    print(f"[SUCCESS] Downloaded mission_track.kml ({size} bytes)!")
    print(f"Local Path: {root_kml}")
else:
    print(f"[WARN] mission_track.kml could not be transferred. Error: {r_kml.stderr}")

if os.path.exists(target_csv):
    print(f"[SUCCESS] Downloaded mission_log.csv!")
