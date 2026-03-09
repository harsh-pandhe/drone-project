import os
import subprocess

def check_component(name, command):
    print(f"Checking {name}...")
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print(f"  [+] {name}: OK")
            return True
        else:
            print(f"  [-] {name}: FAILED (Command error)")
            return False
    except Exception as e:
        print(f"  [!] {name}: ERROR ({e})")
        return False

print("=== ASCEND System Diagnostic ===")
# Check Pixhawk Heartbeat via MAVLink (Requires MAVSDK/Mavproxy)
check_component("Pixhawk Serial", "ls /dev/ttyAMA0")
# Check LiDAR Network
check_component("LiDAR Connection", "ping -c 1 -W 1 192.168.1.199")
# Check Coral USB Mode
check_component("Coral TPU Hardware", "lsusb | grep -E '1a6e|18d1'")
# Check SSD Storage
check_component("NVMe SSD", "df -h | grep nvme0n1")