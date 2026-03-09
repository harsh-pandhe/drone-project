import os
import subprocess

def lab_status():
    print("=== ASCEND Remote Lab Check ===")
    
    # 1. LiDAR Link (192.168.1.199)
    ping = os.system("ping -c 1 -W 1 192.168.1.199 > /dev/null 2>&1")
    print(f"LiDAR: {'ONLINE' if ping == 0 else 'OFFLINE'}")

    # 2. Coral TPU Driver Status
    usb = subprocess.run(['lsusb'], capture_output=True, text=True).stdout
    if "18d1:9302" in usb:
        print("Coral: ACTIVE (Google Inc.)")
    else:
        print("Coral: DETECTED (Waiting for first inference)")

    # 3. Pixhawk 6C Link
    pixhawk = any(os.path.exists(p) for p in ["/dev/ttyACM0", "/dev/ttyAMA0"])
    print(f"Pixhawk: {'LINKED' if pixhawk else 'NOT FOUND'}")

if __name__ == "__main__":
    lab_status()
