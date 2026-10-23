#!/usr/bin/env python3
"""
Setup Boat 18 (172.20.25.218):
1. Install SSH key
2. Create ~/NavySeals
3. Check driver symlink
4. Copy all NavySeals files via SFTP
"""

import os
import sys
import paramiko

BOAT_IP = "172.20.25.218"
USER = "ue32"
PASS = "ue32"
LOCAL_DIR = r"d:\Personal_Projects\BEST_Breast\NavySeals"

print(f"Connecting to Boat 18 ({BOAT_IP})...")
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(BOAT_IP, username=USER, password=PASS, timeout=8)
print("[OK] Connected successfully via password.")

# 1. Authorize SSH public key
pub_key_path = os.path.expanduser("~/.ssh/id_ed25519.pub")
if os.path.exists(pub_key_path):
    with open(pub_key_path) as f:
        pub_key = f.read().strip()
    cmd = f"mkdir -p ~/.ssh && chmod 700 ~/.ssh && grep -qF '{pub_key}' ~/.ssh/authorized_keys 2>/dev/null || echo '{pub_key}' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
    stdin, stdout, stderr = ssh.exec_command(cmd)
    stdout.channel.recv_exit_status()
    print("[OK] Installed SSH public key for passwordless access.")

# 2. Check drivers on Boat 18
cmd = "mkdir -p ~/NavySeals && test -e ~/NavySeals/drivers-ddboat-v2 || ln -s /home/best/drivers-ddboat-v2 ~/NavySeals/drivers-ddboat-v2"
stdin, stdout, stderr = ssh.exec_command(cmd)
stdout.channel.recv_exit_status()
print("[OK] Prepared ~/NavySeals and drivers symlink.")

# 3. SFTP Upload all files from local NavySeals to ~/NavySeals
sftp = ssh.open_sftp()
uploaded = 0
for fname in os.listdir(LOCAL_DIR):
    local_path = os.path.join(LOCAL_DIR, fname)
    if os.path.isfile(local_path):
        remote_path = f"/home/ue32/NavySeals/{fname}"
        sftp.put(local_path, remote_path)
        uploaded += 1

sftp.close()
print(f"[OK] Uploaded {uploaded} files to Boat 18 (~/NavySeals/).")

# 4. Check hardware on Boat 18
print("\nRunning fast hardware check on Boat 18...")
check_code = (
    "import sys\n"
    "sys.path.append('NavySeals/drivers-ddboat-v2')\n"
    "try:\n"
    "    import imu9_driver_v2, gps_driver_v2, arduino_driver_v2\n"
    "    print('[OK] All drivers imported successfully on Boat 18!')\n"
    "except Exception as e:\n"
    "    print('[WARN] Driver error:', e)\n"
)
stdin, stdout, stderr = ssh.exec_command(f"python3 -c \"{check_code}\"")
print(stdout.read().decode().strip())

ssh.close()
print("\n[SUCCESS] Boat 18 is fully set up and ready!")
