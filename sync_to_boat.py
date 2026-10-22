import os
import paramiko

BOAT_IP = '172.20.25.211'
USERNAME = 'ue32'
PASSWORD = 'ue32'

LOCAL_DIR = r'd:\Personal_Projects\BEST_Breast\NavySeals'
REMOTE_DIR = '/home/ue32/NavySeals'

print("Connecting to DD-Boat 11 at {}...".format(BOAT_IP))
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(BOAT_IP, username=USERNAME, password=PASSWORD, timeout=8)

# 1. Setup SSH keys for passwordless auth from laptop
pubkey_path = os.path.expanduser(r'~/.ssh/id_ed25519.pub')
if os.path.exists(pubkey_path):
    with open(pubkey_path, 'r', encoding='utf-8') as f:
        pubkey = f.read().strip()
    cmd = (
        'mkdir -p ~/.ssh && chmod 700 ~/.ssh && '
        'grep -q -F "{0}" ~/.ssh/authorized_keys 2>/dev/null || echo "{0}" >> ~/.ssh/authorized_keys && '
        'chmod 600 ~/.ssh/authorized_keys'
    ).format(pubkey)
    stdin, stdout, stderr = ssh.exec_command(cmd)
    stdout.channel.recv_exit_status()
    print("SSH public key installed on boat for passwordless login.")

# 2. Setup destination dir and symlink to drivers
cmd_dir = (
    'mkdir -p {0} && '
    'cd {0} && '
    '[ -e drivers-ddboat-v2 ] || ln -s /home/best/drivers-ddboat-v2 drivers-ddboat-v2'
).format(REMOTE_DIR)
stdin, stdout, stderr = ssh.exec_command(cmd_dir)
stdout.channel.recv_exit_status()

# 3. SFTP session
sftp = ssh.open_sftp()

# First, pull any calibration or kml files from boat to local if local is missing them
remote_files = sftp.listdir(REMOTE_DIR)
for f in remote_files:
    if f.startswith('calib_') or f.endswith('.kml'):
        local_fp = os.path.join(LOCAL_DIR, f)
        if not os.path.exists(local_fp):
            print("Preserving/downloading boat file to local: {}".format(f))
            sftp.get(REMOTE_DIR + '/' + f, local_fp)

# Now push all python and md files from local to boat
print("\nUploading local files to DD-Boat 11:")
for fname in sorted(os.listdir(LOCAL_DIR)):
    if fname.endswith(('.py', '.md', '.json', '.txt')):
        local_fp = os.path.join(LOCAL_DIR, fname)
        if os.path.isfile(local_fp):
            remote_fp = REMOTE_DIR + '/' + fname
            size = os.path.getsize(local_fp)
            sftp.put(local_fp, remote_fp)
            print("  -> Uploaded: {:<25} ({:>5} bytes)".format(fname, size))

sftp.close()

stdin, stdout, stderr = ssh.exec_command('ls -lh ' + REMOTE_DIR)
print("\nFinal remote listing in {}:".format(REMOTE_DIR))
print(stdout.read().decode())

ssh.close()
print("All files loaded successfully on DD-Boat 11!")
